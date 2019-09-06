package mergingBodies;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;
import no.uib.cipr.matrix.DenseVector;

import mergingBodies.RigidBody.ObjectState;

/**
 * Class for detecting and resolving collisions. Currently this class uses penalty forces between rigid bodies.
 * @author kry
 */
public class CollisionProcessor {

	public List<RigidBody> bodies;

	private HashMap<String, Contact> lastTimeStepMap = new HashMap<String, Contact>();

	public ArrayList<RigidCollection> collections = new ArrayList<RigidCollection>();
	/**
	 * The current contacts that resulted in the last call to process collisions
	 */
	public ArrayList<Contact> contacts = new ArrayList<Contact>();
	
	/** body-body contacts for pruning */
	ArrayList<Contact> tmpBodyBodyContacts = new ArrayList<Contact>();

	/**
	 * Default constructor
	 */
	public CollisionProcessor() {
		bodies = null;
	}
	
	/**
	 * Creates this collision processor with the provided set of bodies
	 * @param bodies
	 */
	public CollisionProcessor( List<RigidBody> bodies ) {
		this.bodies = bodies;
	}

	/** keeps track of the time used for collision detection on the last call */
	double collisionDetectTime = 0;

	/** keeps track of the time used to solve the LCP based velocity update on the last call */
	double collisionSolveTime = 0;

	/**list that keeps track of all the body contacts that occurred in this time step */
	public ArrayList<BodyContact> bodyContacts = new ArrayList<BodyContact>();

	/**
	 * Processes all collisions. Find collision points and calculate contact force.
	 * @param dt time step
	 */
	public void processCollisions( double dt ) {

		Contact.nextContactIndex = 0;
		
		contacts.clear();
		if (RigidBodySystem.enableSleeping.getValue() || RigidBodySystem.enableMerging.getValue()) 
			rememberBodyContacts();
		
		long now = System.nanoTime();
		broadPhase(); // fill contacts list
		
		collisionDetectTime = ( System.nanoTime() - now ) * 1e-9;
		if (contacts.size() == 0)  
			lastTimeStepMap.clear();

		if ( contacts.size() > 0) {
			now = System.nanoTime();
			PGS( dt,  now, false);
			removeCollectionsInternalContacts();
			calculateContactForce(dt);	
		}
	}

	public void calculateContactForce(double dt) {
		Vector2d cForce = new Vector2d();
		double cTorque = 0;
		for (Contact c: contacts) {
			cForce.set(c.lamda.x*c.j_1.get(0) + c.lamda.y*c.j_2.get(0),c.lamda.x*c.j_1.get(1) + c.lamda.y*c.j_2.get(1) );
			cTorque = c.lamda.x*c.j_1.get(2) + c.lamda.y*c.j_2.get(2);
			cForce.scale(1/dt);
			c.subBody1.transformW2B.transform(cForce);
			c.contactForceB1.set(cForce);
			c.contactTorqueB1 = cTorque/dt;

			c.body1ContactForceHistory.add(c.contactForceB1);
			c.body1ContactTorqueHistory.add(c.contactTorqueB1);
			if (c.body1ContactForceHistory.size() > CollisionProcessor.sleepAccum.getValue()) {
				c.body1ContactForceHistory.remove(0);
				c.body1ContactTorqueHistory.remove(0);
			}

			//if Body1 is a parent, also apply the contact force to the appropriate subBody

			cForce.set(c.lamda.x*c.j_1.get(3) + c.lamda.y*c.j_2.get(3),c.lamda.x*c.j_1.get(4) + c.lamda.y*c.j_2.get(4) );
			cTorque = c.lamda.x*c.j_1.get(5) + c.lamda.y*c.j_2.get(5);
			cForce.scale(1/dt);
			c.subBody2.transformW2B.transform(cForce);
			c.contactForceB2.set(cForce);
			c.contactTorqueB2 = cTorque/dt;

			c.body2ContactForceHistory.add(c.contactForceB2);
			c.body2ContactTorqueHistory.add(c.contactTorqueB2);
			if (c.body2ContactForceHistory.size() > CollisionProcessor.sleepAccum.getValue()) {
				c.body2ContactForceHistory.remove(0);
				c.body2ContactTorqueHistory.remove(0);
			}

			//if Body2 is a parent, also apply the contact force to the appropriate subBody
		}
	}

	/**
	 * Remember body contacts if:
	 * <p><ul>
	 * <li> merged or not active
	 * </ul><p>
	 */
	private void rememberBodyContacts() {
		ArrayList<BodyContact> savedBodyContacts = new ArrayList<BodyContact>();
		
		for (BodyContact bc : bodyContacts) {
			if (!bc.merged && ( bc.body1.state == ObjectState.ACTIVE || bc.body2.state == ObjectState.ACTIVE ))
				bc.contactList.clear();
			
			if (bc.updatedThisTimeStep || bc.body1.state == ObjectState.SLEEPING || bc.body2.state == ObjectState.SLEEPING) {
				if ( bc.body1.state == ObjectState.SLEEPING || bc.body2.state == ObjectState.SLEEPING) 
					contacts.addAll(bc.contactList);

				savedBodyContacts.add(bc);
				bc.updatedThisTimeStep = false;
			}
		}
		bodyContacts.clear();
		bodyContacts.addAll(savedBodyContacts);
		for (BodyContact bc: bodyContacts) {
			bc.addToBodyLists();
		}
	}

	private void organize() {
		for (Contact c : contacts) {
			c.index = contacts.indexOf(c);
		}
	}


	public void PGS(double dt, double now, boolean updateCollectionInternalContact) {
		double mu = friction.getValue();
		DenseVector lamdas = new DenseVector(2*contacts.size());
		lamdas.zero();
		int iteration = iterations.getValue();
		int i = 0;
		organize();
		if(shuffle.getValue()) knuth_shuffle();
		
		if (warmStart.getValue()) {
			for (Contact contact_i : contacts) {

				DenseVector j_1 = new DenseVector(contact_i.j_1);
				DenseVector j_2 = new DenseVector(contact_i.j_2);
				Block block1 = contact_i.block1;
				Block block2 = contact_i.block2;

				if(lastTimeStepMap.containsKey("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode() ))|| lastTimeStepMap.containsKey("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode() ))) {

					double m1inv = (contact_i.body1.temporarilyPinned)? 0: contact_i.body1.minv; 
					double m2inv = (contact_i.body2.temporarilyPinned)? 0: contact_i.body2.minv;
					double j1inv = (contact_i.body1.temporarilyPinned)? 0: contact_i.body1.jinv;
					double j2inv = (contact_i.body2.temporarilyPinned)? 0: contact_i.body2.jinv;

					Contact c = lastTimeStepMap.get("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode()));
					if(lastTimeStepMap.containsKey("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode() )))
						c = lastTimeStepMap.get("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode()));
					//if the old map contains this key, then get the lamda of the old map

					if (c.body1 != contact_i.body1 || c.body2 != contact_i.body2) {
						continue;
					}

					double old_lamda_n = c.lamda.x;
					double old_lamda_t = c.lamda.y;
					double old_delta_lamda_n = old_lamda_n;
					double old_delta_lamda_t = old_lamda_t;
					//set this lamda to the old lamda
					lamdas.set(2*contact_i.index, old_lamda_n);
					lamdas.set(2*contact_i.index + 1, old_lamda_t);
					//recompute Delta V's

					//first recompute t.

					double t_1_x_n = 0, t_1_y_n =0 , t_1_omega_n=0, t_2_x_n=0, t_2_y_n=0, t_2_omega_n = 0;
					double t_1_x_t=0, t_1_y_t=0, t_1_omega_t=0, t_2_x_t=0, t_2_y_t=0, t_2_omega_t = 0;
					//first body
					t_1_x_n = j_1.get(0) * m1inv*old_delta_lamda_n;
					t_1_y_n = j_1.get(1)* m1inv*old_delta_lamda_n;
					t_1_omega_n = j_1.get(2)* j1inv*old_delta_lamda_n;
					//second body
					t_2_x_n = j_1.get(3) * m2inv*old_delta_lamda_n;
					t_2_y_n = j_1.get(4) * m2inv*old_delta_lamda_n;
					t_2_omega_n = j_1.get(5) * j2inv*old_delta_lamda_n;

					//first body
					t_1_x_t = j_2.get(0) * m1inv*old_delta_lamda_t;
					t_1_y_t =  j_2.get(1) * m1inv*old_delta_lamda_t;
					t_1_omega_t =  j_2.get(2)  * j1inv*old_delta_lamda_t;
					//second body
					t_2_x_t =  j_2.get(3)  * m2inv*old_delta_lamda_t;
					t_2_y_t =  j_2.get(4)  * m2inv*old_delta_lamda_t;
					t_2_omega_t =  j_2.get(5) * j2inv* old_delta_lamda_t;

					//update delta V;
					DenseVector dV1 = c.body1.delta_V; 
					DenseVector dV2 = c.body2.delta_V; 
					dV1.set( 0, dV1.get( 0) + t_1_x_n  + t_1_x_t);
					dV1.set( 1, dV1.get( 1) + t_1_y_n + t_1_y_t );
					dV1.set(2, dV1.get(2) +  t_1_omega_n + t_1_omega_t);

					//update delta V;
					dV2.set(0, dV2.get(0) + t_2_x_n + t_2_x_t);
					dV2.set(1, dV2.get(1) + t_2_y_n + t_2_y_t );
					dV2.set(2, dV2.get(2) + t_2_omega_n + t_2_omega_t );
					
					contact_i.body1ContactForceHistory.clear();
					contact_i.body1ContactTorqueHistory.clear();
					contact_i.body2ContactForceHistory.clear();
					contact_i.body2ContactTorqueHistory.clear();
					contact_i.body1ContactForceHistory.addAll(c.body1ContactForceHistory);
					contact_i.body1ContactTorqueHistory.addAll(c.body1ContactTorqueHistory);
					contact_i.body2ContactForceHistory.addAll(c.body2ContactForceHistory);
					contact_i.body2ContactTorqueHistory.addAll(c.body2ContactTorqueHistory);
				
				}
			}
		} 

		while(iteration > 0) {
			//shuffle for stability
			
			if (iteration==1 && updateCollectionInternalContact) 
				lamdas = addCollectionsInternalContacts(lamdas);			

			for (i = 0; i < 2*contacts.size(); i++) {

				//if we are looking at a normal component of lamda
				double lamda_i = lamdas.get(i);

				Contact contact_i = contacts.get(i/2);
				DenseVector j_1 = new DenseVector(contact_i.j_1);
				DenseVector j_2 = new DenseVector(contact_i.j_2);

				// eulalie: actually the body in contact always are subBody...
				RigidBody body1 = (contact_i.body1.parent!=null)? contact_i.subBody1: contact_i.body1;
				RigidBody body2 = (contact_i.body2.parent!=null)? contact_i.subBody2: contact_i.body2;
				
				double m1inv = (body1.temporarilyPinned)? 0: body1.minv; 
				double m2inv = (body2.temporarilyPinned)? 0: body2.minv;
				double j1inv = (body1.temporarilyPinned)? 0: body1.jinv;
				double j2inv = (body2.temporarilyPinned)? 0: body2.jinv;

				//calculate D_i_i 
				double d_i = 0;
				if (i%2 == 0) { //normal component
					//first body component
					d_i+= Math.pow(j_1.get(0), 2) * m1inv;
					d_i+= Math.pow(j_1.get(1), 2) * m1inv;
					d_i+= Math.pow(j_1.get(2), 2) * j1inv;
					//second body component
					d_i+= Math.pow(j_1.get(3), 2) * m2inv;
					d_i+= Math.pow(j_1.get(4), 2) * m2inv;
					d_i+= Math.pow(j_1.get(5), 2) * j2inv;
				}
				else { //tangential component
					//first body component
					d_i+= Math.pow(j_2.get(0), 2) * m1inv;
					d_i+= Math.pow(j_2.get(1), 2) * m1inv;
					d_i+= Math.pow(j_2.get(2), 2) * j1inv; 
					//second body component
					d_i+= Math.pow(j_2.get(3), 2) * m2inv;
					d_i+= Math.pow(j_2.get(4), 2) * m2inv;
					d_i+= Math.pow(j_2.get(5), 2) * j2inv;
				}

				//get J Row i Delta V term first
				//multiply the 6 J values with the appropriate delta V values

				DenseVector dV1 = body1.delta_V; 
				DenseVector dV2 = body2.delta_V; 
				double j_row_i_delta_V = 0;

				if (i%2 == 0) { //normal component
					//first body
					j_row_i_delta_V += j_1.get(0) * dV1.get(0);
					j_row_i_delta_V += j_1.get(1) * dV1.get(1);
					j_row_i_delta_V += j_1.get(2) * dV1.get(2);
					//second body
					j_row_i_delta_V +=  j_1.get(3) * dV2.get(0);
					j_row_i_delta_V +=  j_1.get(4) * dV2.get(1);
					j_row_i_delta_V +=  j_1.get(5) * dV2.get(2);
				}
				else { //tangential component
					//first body
					j_row_i_delta_V += j_2.get(0) * dV1.get(0);
					j_row_i_delta_V += j_2.get(1) * dV1.get(1);
					j_row_i_delta_V += j_2.get(2) * dV1.get(2);
					//second body
					j_row_i_delta_V += j_2.get(3) * dV2.get(0);
					j_row_i_delta_V += j_2.get(4) * dV2.get(1);
					j_row_i_delta_V += j_2.get(5) * dV2.get(2);
				}
				j_row_i_delta_V /= d_i;

				//now take care of assembling b
				// find all relevant values of u.
				double u_1_x = body1.v.x;
				double u_1_y = body1.v.y;
				double u_1_omega = body1.omega;

				double u_2_x = body2.v.x;
				double u_2_y = body2.v.y;
				double u_2_omega = body2.omega;

				//add all relevant values of f, multiplied by appropriate minv to u_1_x etc
				u_1_x += body1.force.x * m1inv*dt;
				u_1_y += body1.force.y * m1inv*dt;
				u_1_omega += body1.torque * j1inv*dt;

				u_2_x += body2.force.x * m2inv*dt;
				u_2_y += body2.force.y * m2inv*dt;
				u_2_omega += body2.torque * j2inv*dt;

				//multiply all the u values by the appropriate J values.
				if (i%2 == 0) {
					u_1_x = u_1_x *( j_1.get(0));
					u_1_y = u_1_y * (j_1.get(1));
					u_1_omega = u_1_omega * (j_1.get(2));

					u_2_x = u_2_x *j_1.get(3);
					u_2_y =   u_2_y * j_1.get(4);
					u_2_omega =  u_2_omega *j_1.get(5);
				}
				else {
					u_1_x = u_1_x *(j_2.get(0));
					u_1_y = u_1_y * j_2.get(1);
					u_1_omega = u_1_omega * j_2.get(2);

					u_2_x =  u_2_x *j_2.get(3);
					u_2_y =  u_2_y *j_2.get(4);
					u_2_omega =  u_2_omega * j_2.get(5);
				}

				//add the Bounce vector to the u's over here, but don't need to do that just yet
				// bounce bounce bounce bounce bounce bounce bounce bounce bounce bounce ///

				// calculate Baumgarte Feedback (overlap of the two bodies)
				double c = feedbackStiffness.getValue();
				double bf = c*contact_i.constraintViolation;

				//putting b together.
				double b = 0;
				if (i%2 ==0) {
					b = (u_1_x + u_2_x + u_1_y + u_2_y + u_1_omega + u_2_omega - restitution.getValue() + bf)/d_i;
				}
				else{
					b = (u_1_x + u_2_x + u_1_y + u_2_y + u_1_omega + u_2_omega)/d_i;
				}

				double prev_lamda = lamda_i;
				//putting everything  for lamda together
				lamda_i= prev_lamda-b - j_row_i_delta_V;


				if (i%2 == 0) {
					lamda_i = Math.max(0, lamda_i);
				} else {
					//tangential lamda, constrained by mu* normal lamda
					//get previous normal value for lamda
					double normal_lamda = lamdas.get(i - 1);
					lamda_i = Math.max(lamda_i, -mu*normal_lamda);
					lamda_i = Math.min(lamda_i, mu*normal_lamda);

				}
				double delta_lamda = lamda_i - prev_lamda;
				//update the force on each body to account for the collision


				//updating lamda vector
				if (i%2 ==0) {
					lamdas.set(2*contact_i.index,  lamda_i);
					contact_i.lamda.set(lamda_i, contact_i.lamda.y);
				} else {
					lamdas.set(2*contact_i.index + 1, lamda_i);
					contact_i.lamda.set(contact_i.lamda.x, lamda_i);
				}
				//Now we still need to do the velocity update.
				// for that we must compute T = MinvJT TIMES deltaLamda
				double t_1_x, t_1_y, t_1_omega, t_2_x, t_2_y, t_2_omega = 0;
				if (i%2 == 0) {
					//first body
					t_1_x = j_1.get(0) * m1inv*delta_lamda;
					t_1_y = j_1.get(1) * m1inv*delta_lamda;
					t_1_omega = j_1.get(2) * j1inv*delta_lamda;
					//second body
					t_2_x = j_1.get(3) * m2inv*delta_lamda;
					t_2_y = j_1.get(4) * m2inv*delta_lamda;
					t_2_omega = j_1.get(5) * j2inv*delta_lamda;
				} else {
					//first body
					t_1_x = j_2.get(0) * m1inv*delta_lamda;
					t_1_y = j_2.get(1) * m1inv*delta_lamda;
					t_1_omega = j_2.get(2) * j1inv*delta_lamda;
					//second body
					t_2_x = j_2.get(3) * m2inv*delta_lamda;
					t_2_y = j_2.get(4) * m2inv*delta_lamda;
					t_2_omega = j_2.get(5) * j2inv*delta_lamda;
				}

				//update delta V;
				dV1.set( 0, dV1.get(0) + t_1_x );
				dV1.set( 1, dV1.get(1) + t_1_y );
				dV1.set( 2, dV1.get(2) + t_1_omega );

				//update delta V;
				dV2.set( 0, dV2.get(0) + t_2_x );
				dV2.set( 1, dV2.get(1) + t_2_y );
				dV2.set( 2, dV2.get(2) + t_2_omega );
			}
			iteration--;
		}

		//fill the new map
		lastTimeStepMap.clear();
		for (Contact co : contacts) {
			Block block1 = co.block1;
			Block block2 = co.block2;

			lastTimeStepMap.put("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode()), co);
		} 
		collisionSolveTime = (System.nanoTime() - now) * 1e-9;
	}
	
	protected DenseVector addCollectionsInternalContacts(DenseVector lamdas)
	{
		boolean hasCollection = false;
		
		// Add contacts inside collection for the last time step
		DenseVector prevLamdas = new DenseVector(lamdas);
		for (RigidBody b: bodies) {	
			if (b instanceof RigidCollection) {	
				RigidCollection colB = (RigidCollection) b;	
				contacts.addAll(colB.internalContacts); 
				hasCollection = true;
			}	
		}
		if (hasCollection) {
			organize();
			lamdas = new DenseVector(2*contacts.size());
			for (int j=0; j<lamdas.size(); j++) {
				if (j<prevLamdas.size()) {
					lamdas.set(j, prevLamdas.get(j));
				} else {
					lamdas.set(j, contacts.get(j/2).lamda.x);
					lamdas.set(j+1, contacts.get(j/2).lamda.y);
					j++;
				}
			}
		}	
		return lamdas;
	}
	
	protected void removeCollectionsInternalContacts()
	{
		// Add contacts inside collection for the last time step
		for (RigidBody b: bodies) {	
			if (b instanceof RigidCollection) {	
				RigidCollection colB = (RigidCollection) b;	
				contacts.removeAll(colB.internalContacts); 
			}	
		}
		organize();
	}
	

	private void knuth_shuffle() {
		//go through each element in contacts 2.
		//at each element, swap it for another random member of contacts2. 
		//at each element, get a random index from i to contacts.size.
		//swap this element with that element at that index. 
		// go to next element in contacts 2

		Collections.shuffle(contacts);
		for (Contact c : contacts) {
			c.index = contacts.indexOf(c);
		}
	}

	/**
	 * Checks for collisions between bodies.  Note that you can optionally implement some broad
	 * phase test such as spatial hashing to reduce the n squared body-body tests.
	 * Currently this does the naive n squared collision check.
	 * Has prune option.
	 */
	private void broadPhase() {
		
		// Naive n squared body test.. might not be that bad for small number of bodies 
		visitID++;
		for ( RigidBody b1 : bodies ) {
			for ( RigidBody b2 : bodies ) { // not so inefficient given the continue on the next line
				
				if ( b1.index >= b2.index ) continue;
				if ( b1.pinned && b2.pinned ) continue; 
				
				tmpBodyBodyContacts.clear();
				narrowPhase( b1, b2 );
				
				if ( pruneContacts.getValue() && tmpBodyBodyContacts.size() >= 3 ) {
					
					ArrayList<Point2d> points = new ArrayList<Point2d>();
					Point2d meanPos = new Point2d();
					Vector2d v = new Vector2d();
					int N = tmpBodyBodyContacts.size();
					for ( Contact c : tmpBodyBodyContacts ) {
						points.add( c.contactW );
						meanPos.add( c.contactW );						
					}
					meanPos.scale( 1.0 / N );
					Matrix2d covariance = new Matrix2d();
					for ( Contact c : tmpBodyBodyContacts ) {
						v.sub( c.contactW, meanPos );						
						covariance.rank1( 1.0 / N, v );
					}
					covariance.evd();
					double eps = 1e-4;
					if ( !(covariance.ev1 > eps && covariance.ev2 > eps) ) { // not a flat region... could do convex hull
						
						Vector2d dir = null;
						if ( covariance.ev1 <= eps ) {
							dir = covariance.v2;
						} else {
							dir = covariance.v1;
						}
						
						// now search for the farthest contacts in this direction!
						double minDot = Double.MAX_VALUE;
						double maxDot = Double.MIN_VALUE;
						Contact cmin = null;
						Contact cmax = null;
						double minDotViolation = Double.MAX_VALUE;
						double maxDotViolation = Double.MAX_VALUE;
						eps = 1e-2;
						
						for ( Contact c : tmpBodyBodyContacts ) {
							v.sub( c.contactW, meanPos );
							double dot = v.dot( dir ); 
							if ( dot > maxDot & c.constraintViolation <= maxDotViolation + eps) {
								maxDot = dot;
								cmax = c;
								maxDotViolation = c.constraintViolation;
							}
							if ( dot < minDot & c.constraintViolation <= minDotViolation + eps) {
								minDot = dot;
								cmin = c;
								minDotViolation = c.constraintViolation;
							}
						}
						
						tmpBodyBodyContacts.clear();
						tmpBodyBodyContacts.add( cmax );
						tmpBodyBodyContacts.add( cmin );
					}
				} // end prune
				
				if (RigidBodySystem.enableMerging.getValue() || RigidBodySystem.enableSleeping.getValue())
					for (Contact contact : tmpBodyBodyContacts) 
						if (!contact.body1.pinned || !contact.body2.pinned)
							storeContactInBodies(contact);
				
				contacts.addAll(tmpBodyBodyContacts);
			}
		}        
	}
	
	
	/**
	 * Store contact in bodyContacts and bodies.contactList list, and update relative velocity history.
	 * @param contact
	 */
	private void storeContactInBodies(Contact contact) {
		
		if (!contact.body1.contactList.contains(contact)) {
			contact.body1.contactList.add(contact);
		}
		if (!contact.body2.contactList.contains(contact)) {
			contact.body2.contactList.add(contact);
		}
		
		// check if this body contact exists already
		BodyContact bc = BodyContact.checkExists(contact.subBody1, contact.subBody2, bodyContacts);

		if (bc != null) { // if it exists
			if (!bc.updatedThisTimeStep) { // only once per time step
				bc.relativeVelHistory.add(contact.getRelativeMetric());
				if (bc.relativeVelHistory.size() > CollisionProcessor.sleepAccum.getValue()) {
					bc.relativeVelHistory.remove(0);
				}
				bc.updatedThisTimeStep = true;
			}
		} else { // body contact did not exist in previous list
			bc = new BodyContact(contact.subBody1, contact.subBody2);
			bc.relativeVelHistory.add(contact.getRelativeMetric());
			bc.updatedThisTimeStep = true;
			bodyContacts.add(bc);
		}

		if (!contact.subBody1.bodyContactList.contains(bc))
			contact.subBody1.bodyContactList.add(bc);
		if (!contact.subBody2.bodyContactList.contains(bc))
			contact.subBody2.bodyContactList.add(bc);
		
		contact.bc = bc;
		bc.contactList.add(contact);
	}
		

	/**
	 * Checks for collision between boundary blocks on two rigid bodies.
	 * @param body1
	 * @param body2
	 */
	private void narrowPhase( RigidBody body1, RigidBody body2 ) {
		//if both bodies are inactive, they do not collide

		if ((body1.state==ObjectState.SLEEPING && body2.state==ObjectState.SLEEPING)) {
			return;
		}

		if (body1 instanceof RigidCollection || body2 instanceof RigidCollection) {
			narrowCollection(body1, body2);
		} else {
			findCollisions(body1.root, body2.root, body1, body2);
		}
	}
	
	
	/**
	 * Recursive method that makes us check for collisions with each body in a rigidCollection
	 */
	private void narrowCollection(RigidBody body1, RigidBody body2) {
		if (body1 instanceof RigidCollection && body2 instanceof RigidCollection) {
			for (RigidBody b: ((RigidCollection) body1).collectionBodies) {
				narrowCollection(b, body2);
			}
		} else if (body1 instanceof RigidCollection) {
			for (RigidBody b: ((RigidCollection) body1).collectionBodies) {
				findCollisions(b.root, body2.root, b, body2);
			}
		} else if (body2 instanceof RigidCollection) {
			for (RigidBody b: ((RigidCollection) body2).collectionBodies) {
				findCollisions(body1.root, b.root, body1, b);
			}
		}
	}
	
	
	/** 
	 * Recurses through all of body_1, then body_2
	 * @param node1
	 * @param node2
	 * @param body1
	 * @param body2
	 */
	private void findCollisions(BVNode node1, BVNode node2, RigidBody body1, RigidBody body2) {
		
		if(node1.visitID != visitID) {
			node1.visitID = visitID;
			node1.boundingDisc.updatecW();
		}
		if (node2.visitID != visitID) {
			node2.visitID = visitID;
			node2.boundingDisc.updatecW();
		}

		if (node1.boundingDisc.intersects(node2.boundingDisc)) {
			if (node1.isLeaf() && node2.isLeaf()) {
				Block leafBlock1 = node1.leafBlock;
				Block leafBlock2 = node2.leafBlock;

				processCollision(body1, leafBlock1, body2, leafBlock2);
				
				// Wake neighbors, and update wokenUp boolean for display
				// eulalie: shouldn't we do that in processCollision when an actual contact is detected?
				if (RigidBodySystem.enableSleeping.getValue()){
					if (!body1.wokenUp && body1.state == ObjectState.ACTIVE && !body1.pinned && body2.state == ObjectState.SLEEPING) {
						wakeNeighbors(body2, collisionWake.getValue());
					} else if (!body2.wokenUp && body2.state == ObjectState.ACTIVE && !body2.pinned && body1.state == ObjectState.SLEEPING) {
						wakeNeighbors(body1, collisionWake.getValue());
					}
				}
			} else if(node1.isLeaf()|| node1.boundingDisc.r <= node2.boundingDisc.r){
				//if theys overlap, and body 1 is either a leaf or smaller than body_2, break down body_2

				findCollisions(node1, node2.child1, body1, body2);
				findCollisions(node1, node2.child2, body1, body2);
			} else if(node2.isLeaf() || node2.boundingDisc.r <= node1.boundingDisc.r) {
				//if they overlap, and body 2 is either a leaf or smaller than body_1, break down body_1

				findCollisions(node1.child1, node2, body1, body2);
				findCollisions(node1.child2, node2, body1, body2);
			}
		}
	}

	/** 
	 * If a collision is detected, will travel the contact graph and wake up 
	 * bodies that are n hops away in the contact graph
	 * @param body1
	 * @param body2
	 */
	private void wakeNeighbors(RigidBody body1, int hop) {
		if (hop > 0) {
			body1.state = ObjectState.ACTIVE;
			/*if (!body1.active_past.isEmpty()) {
				body1.active_past.remove(body1.active_past.size() - 1);} */
			hop--;
			body1.visited = true;
			body1.wokenUp = true;
			//	body1.active_past.add(true); makes bodies oscillate between sleeping and waking
			for (BodyContact c: body1.bodyContactList) {
				if (!c.body2.pinned)
					wakeNeighbors(c.body2, hop);
			}
		}
	}

	/** 
	 * The visitID is used to tag boundary volumes that are visited in 
	 * a given time step.  Marking boundary volume nodes as visited during
	 * a time step allows for a visualization of those used, but it can also
	 * be used to more efficiently update the centeres of bounding volumes
	 * (i.e., call a BVNode's updatecW method at most once on any given timestep)
	 */
	int visitID = 0;


	/**
	 * Resets the state of the collision processor by clearing all
	 * currently identified contacts, and reseting the visitID for
	 * tracking the bounding volumes used
	 */
	public void reset() {
		contacts.clear();
		Contact.nextContactIndex = 0;
		visitID = 0;            
	}

	
	// some working variables for processing collisions
	private Point2d tmp1 = new Point2d();
	private Point2d tmp2 = new Point2d();
	private Point2d contactW = new Point2d();
	private Vector2d normal = new Vector2d();


	/**
	 * Processes a collision between two bodies for two given blocks that are colliding.
	 * @param body1
	 * @param b1
	 * @param body2
	 * @param b2
	 */
	private void processCollision( RigidBody body1, Block b1, RigidBody body2, Block b2 ) {        

		body1.transformB2W.transform( b1.pB, tmp1 );
		body2.transformB2W.transform( b2.pB, tmp2 );
		double distance = tmp1.distance(tmp2);

		if ( distance < Block.radius * 2 ) {
			// contact point at halfway between points 
			// NOTE: this assumes that the two blocks have the same radius!
			contactW.interpolate( tmp1, tmp2, .5 );
			// contact normal
			normal.sub( tmp2, tmp1 );
			normal.normalize();
			// create the contact
			Contact contact = null;

			/*
			 *  very important here... the Contact will contain information on contact location, 
			 *  where if any of the bodies involved are parts of a collection, then the contact 
			 *  will list the collection as one of its bodies... not the actual contacting subbody
			 */
			if (body1.parent != null && body2.parent != null) {
				contact = new Contact( body1.parent, body2.parent, contactW, normal, b1, b2, distance, body1, body2);
			}
			else if (body1.parent != null) {
				contact = new Contact( body1.parent, body2, contactW, normal, b1, b2, distance,  body1, body2);
			}
			else if (body2.parent != null) {
				contact = new Contact( body1, body2.parent, contactW, normal, b1, b2, distance,  body1, body2);
			}
			else {
				contact = new Contact( body1, body2, contactW, normal, b1, b2, distance,  body1, body2);
			}

			//set normals in body coordinates
			contact.normalB1.set(normal);
			contact.normalB2.set(normal);
			body1.transformW2B.transform(contact.normalB1);
			body2.transformW2B.transform(contact.normalB2);
			contact.normalB2.scale(-1);

			// put contact into a preliminary list that will be filtered in BroadPhase
			tmpBodyBodyContacts.add( contact );
		}
	}

	/** Restitution parameter for contact constraints */
	public DoubleParameter restitution = new DoubleParameter( "restitution (bounce)", 0, 0, 1 );

	/** Coulomb friction coefficient for contact constraint */
	public DoubleParameter friction = new DoubleParameter("Coulomb friction", 0.3, 0, 2 );

	/** Number of iterations to use in projected Gauss Seidel solve */
	public IntParameter iterations = new IntParameter("iterations for GS solve", 200, 1, 500);

	/** Flag for using shuffle */
	private BooleanParameter shuffle = new BooleanParameter( "shuffle", false);
	/** Flag for using shuffle */
	private BooleanParameter warmStart = new BooleanParameter( "warm start", true);

	public static DoubleParameter feedbackStiffness = new DoubleParameter("feedback coefficient", 0, 0,50  );

	public static DoubleParameter sleepingThreshold = new DoubleParameter("sleeping threshold", 1.0, 0, 10 );

	public static DoubleParameter wakingThreshold = new DoubleParameter("waking threshold", 15, 0, 30);

	public static BooleanParameter  use_contact_graph = new BooleanParameter("enable use of contact graph heuristic", false );

	public static DoubleParameter forceMetricTolerance = new DoubleParameter("force metric tolerance", 10, 0, 15);

	public static IntParameter collisionWake = new IntParameter("wake n neighbors", 2, 0, 10 );

	public static IntParameter sleepAccum = new IntParameter("accumulate N sleep queries", 50, 0, 200 );

	public BooleanParameter pruneContacts = new BooleanParameter( "prune contacts", true );

	/**
	 * @return controls for the collision processor
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.setBorder( new TitledBorder("Collision Processing Controls") );
		vfp.add( shuffle.getControls() );
		vfp.add( pruneContacts.getControls() );
		vfp.add( warmStart.getControls() );
		vfp.add( iterations.getSliderControls() );
		vfp.add( restitution.getSliderControls(false) );
		vfp.add( friction.getSliderControls(false) );
		vfp.add( feedbackStiffness.getSliderControls(false) );

		vfp.add( sleepingThreshold.getSliderControls(false) );
		
		vfp.add( wakingThreshold.getSliderControls(false) );

		vfp.add( use_contact_graph.getControls() );
		vfp.add( forceMetricTolerance.getSliderControls(false) );
		vfp.add( collisionWake.getSliderControls());
		vfp.add( sleepAccum.getSliderControls());

		return vfp.getPanel();
	}

}
