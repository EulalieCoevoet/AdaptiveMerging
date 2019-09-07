package mergingBodies;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mergingBodies.RigidBody.ObjectState;
import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

/**
 * Maintains a list of RigidBody objects, and provides methods for collision processing and numerical integration
 * @author kry
 */
public class RigidBodySystem {

	public String name ="";

	public double simulationTime = 0;

	public ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
	public ArrayList<RigidBody> initialBodies = new ArrayList<RigidBody>();
	
	public int nbCollections = 0;

	public CollisionProcessor collisionProcessor = new CollisionProcessor(bodies);

	public MouseSpringForce mouseSpring;

	BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
	DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
	DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );

	/**Stiffness of  spring*/
	public DoubleParameter spring_k = new DoubleParameter("spring stiffness", 100, 1, 1e4 );

	/**Viscous damping coefficient for the  spring*/
	public DoubleParameter spring_c= new DoubleParameter("spring damping", 0, 0, 1000 );

	/**Viscous damping coefficient for the  spring*/
	public static IntParameter springLength= new IntParameter("spring rest length", 1, 1, 100 );

	/**
	 * Creates a new rigid body system
	 */
	public RigidBodySystem() {
		/* do nothing */
	}

	/**
	 * Adds a rigid body to the system 
	 * @param body
	 */
	public void add( RigidBody body ) {
		bodies.add( body );
	}

	/**
	 * Applies a small random acceleration to all bodies
	 */
	public void jiggle() {
		final Random rand = new Random();
		for ( RigidBody b : bodies ) {
			if ( b.pinned || b.temporarilyPinned ) continue;
			b.omega += rand.nextDouble()*2-1;
			b.v.x += rand.nextDouble()*2-1;
			b.v.y += rand.nextDouble()*2-1;    
		}
	}

	/**Time in seconds to advance the system*/
	public double computeTime;

	/**Total time in seconds for computation since last reset*/
	public double totalAccumulatedComputeTime;

	/**Total number of steps performed*/
	public int totalSteps = 0;
	
	/**
	 * Advances the state of all rigid bodies
	 * @param dt time step
	 */
	public void advanceTime( double dt ) {

		long now = System.nanoTime();  
		totalSteps++;

		clearJunkAtStartOfTimeStep();

		// apply gravity to all bodies... also take this opportunity to clear all forces at the start of the time step
		if ( useGravity.getValue() ) {
			applyGravityForce();
		}  

		if(mouseSpring != null) {
			mouseSpring.apply();
			applySpringForces(); // deal with zero length springs
		}
		
		if ( processCollisions.getValue() ) {
			collisionProcessor.processCollisions( dt );
		}
		
		if (enableMerging.getValue()||enableSleeping.getValue()) {
			applyExternalContactForces(dt);
		}

		if (enableMerging.getValue()) {
			mergeBodies();
			checkIndex();
		}

		if (enableSleeping.getValue()) {
			sleep();
		}

		// advance the system by the given time step (update position and velocities of each body)
		for (RigidBody b : bodies) {
			b.advanceTime(dt); 
		}

		if (enableSleeping.getValue()) {
			wake();
		}

		if (enableMerging.getValue()) {
			unmergeBodies();
			checkIndex();
		}

		if (this.generateBody) {
			generateBody();
			this.generateBody = false;
		}

		computeTime = (System.nanoTime() - now) / 1e9;
		simulationTime += dt;
		totalAccumulatedComputeTime += computeTime;
	}


	/**
	 * Checks if we should put these bodies to sleep.
	 * Conditions for sleeping are:
	 * <p><p><ul>
	 * <li>1. All velocities in velocity history are below threshold. 
	 * <li>2. Velocities in history are monotonically decreasing. 
	 * </ul><p>
	 */
	private void sleep() {
		double sleepingThreshold = CollisionProcessor.sleepingThreshold.getValue();
		for (RigidBody b : bodies) {
			if (b.pinned || b.temporarilyPinned || b.state == ObjectState.SLEEPING) continue;
			double vel = b.getMetric();
			b.velHistory.add(vel);
			if (b.velHistory.size() > CollisionProcessor.sleepAccum.getValue()) {
				b.velHistory.remove(0);	
			}
			boolean sleep = true;

			double previousV = Double.MAX_VALUE; 
			double epsilon = 0.00005;
			if (b.velHistory.size() < CollisionProcessor.sleepAccum.getValue()) {

				sleep = false;
			}
			else {
				for (Double v : b.velHistory) {
					if (v > previousV+epsilon) {
						sleep = false;
						break;
					}
					if (v > sleepingThreshold) {
						sleep = false;
						break;
					}
					previousV = v;
				}
			}
			if (sleep) {
				b.state = ObjectState.SLEEPING;
				b.forcePreSleep.set(b.force);
				b.torquePreSleep = b.torque;
			}
		}
	}

	/**
	 * Checks if a body should wake up
	 * conditions for waking:
	 * If total force metric acting on body is above the forceMetric threshold.
	 * total Force metric = totalForce^2/Mass
	 * total Force = sum of all forces (including contact forces)
	 */
	private void wake() {
		double threshold = CollisionProcessor.forceMetricTolerance.getValue();
		for (RigidBody b: bodies) {
			if (b.state == ObjectState.ACTIVE || b.pinned || b.temporarilyPinned) continue;
			b.transformB2W.transform(b.savedContactForce);
			double forceMetric1 = (b.force.x-b.forcePreSleep.x)*b.minv;
			double forceMetric2 = (b.force.y-b.forcePreSleep.y)*b.minv;
			double forceMetric3 = (b.torque-b.torquePreSleep)*b.jinv;
			b.transformW2B.transform(b.savedContactForce);
			if (b.state == ObjectState.SLEEPING && (forceMetric1 > threshold || forceMetric2 > threshold || forceMetric3 > threshold)) {
				b.velHistory.clear();
				b.state = ObjectState.ACTIVE;
			}
		}
	}

	private void applyGravityForce() {
		Vector2d force = new Vector2d();
		for ( RigidBody b : bodies ) {
			//fully active, regular stepping

			double theta = gravityAngle.getValue() / 180.0 * Math.PI;
			force.set( Math.cos( theta ), Math.sin(theta) );
			force.scale( b.massLinear * gravityAmount.getValue() );
			// gravity goes directly into the accumulator!  no torque!
			b.force.add( force );
			//apply force of gravity to all children as well
			if( b instanceof RigidCollection) {
				applyGravitySubBodies((RigidCollection) b, force, theta);
			}
		}
	}

	private void applySpringForces() {
		for (RigidBody b: bodies){
			for (Spring s: b.springs) {
				applySpringForce(b, s);
			}
			//also apply spring forces to the child
			if (b instanceof RigidCollection) {
				for (RigidBody sB : ((RigidCollection) b).collectionBodies) {
					for (Spring s: sB.springs) {
						applySpringForce(sB, s);

					}
				}
			}
		}
	}

	private void applySpringForce(RigidBody b, Spring s) {
		s.updateP2();
		s.computeVelocities();

		s.apply(spring_k.getValue(), spring_c.getValue());
		s.updateP2();
	}

	private void applyGravitySubBodies(RigidCollection b, Vector2d force, double theta) {
		for (RigidBody sB : ((RigidCollection) b).collectionBodies) {
			force.set( Math.cos( theta ), Math.sin(theta) );
			force.scale( sB.massLinear * gravityAmount.getValue() );
			sB.force.add( force );
		}
	}

	private void applyInternalContactForces(double dt) {
		/*for (RigidBody b: bodies) {
			if (b instanceof RigidCollection) {
				for (Contact c : ((RigidCollection) b).internalContacts) {
					  c.subBody1.contactForce.add(c.contactForceB1);
					  c.subBody1.contactTorques += (c.contactTorqueB1);
					  c.subBody2.contactForce.add(c.contactForceB2);
					  c.subBody2.contactTorques += (c.contactTorqueB2);
				}
			}
		}*/
	}

	private void clearJunkAtStartOfTimeStep() {
		for (RigidBody b: bodies) {
			b.merged = false;
			b.force.set(0, 0);
			b.torque = 0;
			b.deltaV.zero();

			b.savedContactForce.set(0, 0);

			b.currentContactForce.set(0, 0);
			b.contactTorques = 0;
			b.currentContactTorques = 0;
			b.contactList.clear();
			//b.bodyContactList.clear();
			ArrayList<BodyContact> newBodyContactList = new ArrayList<BodyContact>();
			for (BodyContact bc : b.bodyContactList) 
				if (bc.updatedThisTimeStep) 
					newBodyContactList.add(bc);
			b.bodyContactList.clear();
			b.bodyContactList.addAll(newBodyContactList);

			if (b instanceof RigidCollection) {
				((RigidCollection) b).unmergedThisTimeStep = false;
				((RigidCollection) b).updatedThisTimeStep = false; 
				for (RigidBody sB: ((RigidCollection )b).collectionBodies) {
					//sB.contactForce.set(0, 0);
					//sB.contactTorques = 0;
					sB.deltaV.zero();
					sB.currentContactForce.set(sB.savedContactForce);
					sB.currentContactTorques = sB.contactTorques;
					//	sB.contactList.clear();
					sB.force.set(0, 0);
					sB.torque = 0;
					sB.merged = false;
					newBodyContactList.clear();
					for (BodyContact bc : sB.bodyContactList) 
						if (bc.merged || bc.updatedThisTimeStep) 
							newBodyContactList.add(bc);
					sB.bodyContactList.clear();
					sB.bodyContactList.addAll(newBodyContactList);
				}
			}
		}
		for (BodyContact bc : collisionProcessor.bodyContacts) {
			bc.clearForces();
		}
	}

	/**
	 * applies contact forces/torques to the body contacts of awake and collection bodies... not sleeping bodies
	 */
	private void applyExternalContactForces(double dt) {
		for (BodyContact bc : collisionProcessor.bodyContacts) {
			for (Contact c: bc.contactList) {

				RigidBody body1 = (c.body1.isInCollection())? c.body1.parent: c.body1;
				RigidBody body2 = (c.body2.isInCollection())? c.body2.parent: c.body2;
				
				// Body 1
				body1.savedContactForce.add(c.contactForceB1);
				body1.contactTorques += (c.contactTorqueB1);
				if (body1 instanceof RigidCollection) {
					double cTorque = getSubBodyTorque(c, c.body1);

					if (!c.body1.bodyContactListPreMerging.contains(bc)) {
						c.body1.currentContactForce.add(c.contactForceB1);
						c.body1.currentContactTorques += cTorque;
					}
					applyToBodyContact(c, c.body1, c.contactForceB1, cTorque);
				} else applyToBodyContact(c, body1, c.contactForceB1, c.contactTorqueB1);
				// Body 2
				body2.savedContactForce.add(c.contactForceB2);
				body2.contactTorques += (c.contactTorqueB2);
				if (body2 instanceof RigidCollection) {
					double cTorque = getSubBodyTorque(c, c.body2);
					
					if (!c.body2.bodyContactListPreMerging.contains(bc)) {
						c.body2.currentContactForce.add(c.contactForceB2);
						c.body2.currentContactTorques += cTorque;
					}
					applyToBodyContact(c, c.body2, c.contactForceB2, cTorque);
				}else applyToBodyContact(c, body2, c.contactForceB2, c.contactTorqueB2);
			}
		}
	}

	private double getSubBodyTorque(Contact c, RigidBody body) {
		double cTorque = 0;
		if (c.bc.body1 == body) {
			double jn_omega, jt_omega;

			Point2d radius_i = new Point2d(c.bc.body1.x);
			body.transformB2W.transform(radius_i);

			radius_i.sub(c.contactW, radius_i);

			Vector2d tangeant = new Vector2d(-c.normal.y, c.normal.x);

			Vector2d r1 = new Vector2d(-radius_i.y, radius_i.x);

			jn_omega = - r1.dot(c.normal);
			jt_omega = - r1.dot(tangeant);
			if (body == c.body2) {
				jn_omega *= -1;
				jt_omega *= -1;
			}

			cTorque = c.lamda.x*jn_omega + c.lamda.y*jt_omega;
			return cTorque;
		}
		else {
			double jn_omega, jt_omega;
			Point2d radius_i = new Point2d(c.bc.body2.x);
			body.transformB2W.transform(radius_i);
			radius_i.sub(c.contactW, radius_i);
			Vector2d tangeant = new Vector2d(-c.normal.y, c.normal.x);
			Vector2d r1 = new Vector2d(-radius_i.y, radius_i.x);
			jn_omega = - r1.dot(c.normal);
			jt_omega = - r1.dot(tangeant);
			if (body == c.body2) { //contact normal may be in the wrong direction based on which body in Contact it is
				jn_omega *= -1;
				jt_omega *= -1;
			}
			cTorque = c.lamda.x*jn_omega + c.lamda.y*jt_omega;
			return cTorque;
		}
	}

	/**
	 * applies contact force to body contacts so we know how much force each body contact exhudes
	 */
	private void applyToBodyContact(Contact c, RigidBody body, Vector2d cForce, double cTorque) {
		if (c.bc.body1 == body) {
			c.bc.body1ContactForce.add(cForce);
			c.bc.body1ContactTorque += cTorque;

		}else if (c.bc.body2 == body) {
			c.bc.body2ContactForce.add(cForce);
			c.bc.body2ContactTorque += cTorque;
		}
	}

	/**
	 * takes the collection, applies the appropriate contact force to the subBody
	 * the normal and tangential contact force components will be the same...
	 * but the rotational ones will be different. 
	 */
	private void applyContactForceToSubBody(Contact c, RigidCollection body, Vector2d cForce) {

		double cTorque= 0;
		if (c.bc.body1.parent == (body)) {


			double jn_omega, jt_omega;

			Point2d radius_i = new Point2d(c.bc.body1.x);
			body.transformB2W.transform(radius_i);

			radius_i.sub(c.contactW, radius_i);

			Vector2d tangeant = new Vector2d(-c.normal.y, c.normal.x);

			Vector2d r1 = new Vector2d(-radius_i.y, radius_i.x);

			jn_omega = - r1.dot(c.normal);
			jt_omega = - r1.dot(tangeant);
			if (body == c.body2) {
				jn_omega *= -1;
				jt_omega *= -1;

			}

			cTorque = c.lamda.x*jn_omega + c.lamda.y*jt_omega;

			//add to force in subBodies because we need to remember the 
			//contact forces, but not the ones modified... otherwise itll keep accumulating
			c.bc.body1.force.add(cForce);
			c.bc.body1.torque += cTorque;

		}
		if (c.bc.body2.parent == body) {
			c.bc.body2.force.add(cForce);

			double jn_omega, jt_omega;

			Point2d radius_i = new Point2d(c.bc.body2.x);
			body.transformB2W.transform(radius_i);

			radius_i.sub(c.contactW, radius_i);

			Vector2d tangeant = new Vector2d(-c.normal.y, c.normal.x);

			Vector2d r1 = new Vector2d(-radius_i.y, radius_i.x);

			jn_omega = - r1.dot(c.normal);
			jt_omega = - r1.dot(tangeant);
			if (body == c.body2) { //contact normal may be in the wrong direction based on which body in Contact it is
				jn_omega *= -1;
				jt_omega *= -1;
			}

			cTorque = c.lamda.x*jn_omega + c.lamda.y*jt_omega;
			c.bc.body2.torque += cTorque;
		}
	}

	/**
	 * Method that deals with unmerging rigidBodies... because we will explore different solutions to
	 * this problem, it will call different methods for each unmerging solution.
	 */
	private void unmergeBodies() {
		//generalHeuristic();
		generalOneBodyAtATime();
		//forceClosureMethod();
	}

	/**
	 * Goes through all bodies in each collection. 
		If a body has enough force acting on it (over a threshold), seperate just that body
		from the rest of the collection. 
	 */
	private void generalOneBodyAtATime() {
		LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
		Vector2d totalForce = new Vector2d();
		double totalTorque = 0;
		boolean unmerge = false;
		for(RigidBody b : bodies) {
			//check if force on Collection is high enough. If it is... unmerge the entire rigidCollection
			if (b instanceof RigidCollection) {
				RigidCollection colB = (RigidCollection) b;
				if (!colB.unmergedThisTimeStep) {
					ArrayList<RigidBody> unmergingBodies = new ArrayList<RigidBody>();
					for (RigidBody sB: colB.collectionBodies) {
						unmerge = colB.metricCheck(sB, totalForce, totalTorque);
						if (unmerge)
							unmergingBodies.add(sB);
					}
					
					ArrayList<RigidBody> newBodies = new ArrayList<RigidBody>();
					if (!unmergingBodies.isEmpty()) {
						unmergeSelectBodies(colB, unmergingBodies, newBodies);				
					}

					if (!newBodies.isEmpty()) {
						for (RigidBody bd: newBodies) {
							additionQueue.add(bd);
						}
						removalQueue.add(colB);
						newBodies.clear();
						int x = 0;
					}
				}
			}
		}
		for (RigidBody b: additionQueue) {
			bodies.add(b);
		}
		for (RigidBody b: removalQueue) {
			bodies.remove(b);
		}
	}

	private void unmergeSelectBodies(RigidCollection colB, ArrayList<RigidBody> unmergingBodies, ArrayList<RigidBody> newBodies) {
		ArrayList<RigidBody> handledBodies = new ArrayList<RigidBody>();

		handledBodies.addAll((unmergingBodies));
		for (RigidBody b: unmergingBodies) {

			colB.unmergeSingleBody(b);
			newBodies.add(b);
			colB.contactsToWorld();
		}
		ArrayList<BodyContact> clearedBodyContacts = new ArrayList<BodyContact>();
		for (RigidBody b: unmergingBodies) {
			ArrayList<RigidBody> subBodies = new ArrayList<RigidBody>();

			for (BodyContact bc : b.bodyContactList) {
				RigidBody body2 = bc.getOtherBody(b);
				clearedBodyContacts.add(bc);
				if (bc.merged) {
					bc.merged = false;
					if(!handledBodies.contains(body2)) {

						subBodies.add(body2);

						handledBodies.add(body2);
						buildNeighborBody(body2, subBodies, handledBodies);

						if (subBodies.size() > 1) {
							//make a new collection
							RigidCollection newCollection = new RigidCollection(subBodies.remove(0), subBodies.remove(0));
							newCollection.addBodies(subBodies);
							newCollection.fillInternalBodyContacts();
							newCollection.v.set(colB.v);
							newCollection.omega = colB.omega;
							newCollection.contactsToBody();
							newBodies.add(newCollection);
							subBodies.clear();
						}	else if ( subBodies.size() == 1){
							colB.unmergeSingleBody(subBodies.get(0));
							newBodies.add(subBodies.remove(0));
							subBodies.clear();
						}
					}
				}
			}
			b.bodyContactList.clear();
			b.bodyContactListPreMerging.clear();
			for (BodyContact bc: clearedBodyContacts) {
				bc.body1.bodyContactList.remove(bc);
				bc.body2.bodyContactList.remove(bc);
				bc.body1.bodyContactListPreMerging.remove(bc);
				bc.body2.bodyContactListPreMerging.remove(bc);
			}
		}
	}

	private void buildNeighborBody(RigidBody b, ArrayList<RigidBody> subBodies, ArrayList<RigidBody> handledBodies) {

		for (BodyContact bc : b.bodyContactList) {
			if (!bc.merged) continue;

			RigidBody body2 = bc.getOtherBody(b);
			if (!handledBodies.contains(body2)) {
				handledBodies.add(body2);
				subBodies.add(body2);
				buildNeighborBody(body2, subBodies, handledBodies);
			}
		}
	}

	/**
	 * The idea is to check at every timestep, the force acting on the system including:
	 * Gravity, Spring, Collision Lamdas, etc.
	 * 
	 * We then go through each body in the collection, and for each body we go through each
	 * contact and see if the new applied force at that contact location lies 
	 * outside of the friction cone. If yes, then unmerge that body... (may need to recurse)
	 * if no, then dont unmerge
	 */
	private void forceClosureMethod() {
		// TODO Fill this method out		
	}

	/**goes through bodies and sees if any collection should be unmerged, and then
	 * unmerges ALLL bodies in that collecion with no discrimination
	 * 
	 */
	private void generalHeuristic() {
		LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
		Vector2d totalForce = new Vector2d();
		double totalTorque = 0;
		for(RigidBody b : bodies) {
			//check if force on Collection is high enough. If it is... unmerge the entire rigidCollection
			if (b instanceof RigidCollection) {

				totalForce.set(b.force);
				totalForce.add(b.savedContactForce);
				totalTorque = b.torque + b.contactTorques;

				double forceMetric = Math.sqrt(Math.pow(totalForce.x,2 ) + Math.pow(totalForce.y, 2))/b.massLinear + Math.sqrt(Math.pow(totalTorque, 2))/b.massAngular;
				if (forceMetric > CollisionProcessor.forceMetricTolerance.getValue()) {
					((RigidCollection) b).unmergeAllBodies();
					additionQueue.addAll(((RigidCollection) b).collectionBodies);
					removalQueue.add(b);
				}
			}
		}

		for (RigidBody b: additionQueue) {
			bodies.add(b);
		}
		for (RigidBody b : removalQueue) {
			bodies.remove(b);
		}
	}
	

	/**
	 * Merges all rigidBodies in the system that fit the appropriate criteria: 
	 * <p><ul>
	 * <li> 1. They have been in contact for 50 time steps
	 * <li> 2. The relative velocities of the two bodies in contact has been below the CollisionProcessor.sleep_accum
	 * 	  value for the ENTIRETY of the contact.
	 * </ul><p>
	 */
	public void mergeBodies() {
		LinkedList<BodyContact> removalQueue = new LinkedList<BodyContact>();
		for (BodyContact bc:collisionProcessor.bodyContacts) {
			boolean mergeCondition = false;
			double threshold = CollisionProcessor.sleepingThreshold.getValue();

			double epsilon = 0.0005;
			if ((bc.relativeVelHistory.size() == CollisionProcessor.sleepAccum.getValue())) {

				mergeCondition = true;
				double prevValue = 0; double currentValue = 0;
				for (Double relVel : bc.relativeVelHistory) {
					currentValue = relVel;
					if (relVel > threshold || currentValue > prevValue + epsilon ) {
						mergeCondition = false; break;
					}
					prevValue = relVel;
				}
			}
			
			if (!bc.updatedThisTimeStep) mergeCondition = false;
			if (bc.body1.pinned || bc.body2.pinned) mergeCondition = false;
			if (bc.body1.merged || bc.body2.merged) mergeCondition = false;
			if(bc.body1.state == ObjectState.SLEEPING && bc.body2.state == ObjectState.SLEEPING) mergeCondition = true;

			if (mergeCondition) {
				bc.merged = true;
				//if they are both not collections...make a new collection!
				if(!bc.body1.isInCollection() && !bc.body2.isInCollection()) {
					bodies.remove(bc.body1); bodies.remove(bc.body2);
					RigidCollection col = new RigidCollection(bc.body1, bc.body2);
					col.addInternalContact(bc);
					bodies.add(col);
				}
				else if (bc.body1.isInCollection() && bc.body2.isInCollection()) {
					// if they are BOTH collections... think about what to do
					//take all the bodies in the least massive one and add them to the collection of the most massive
					if (bc.body1.parent.massLinear > bc.body2.parent.massLinear) {
						bc.body1.merged = true;
						bodies.remove(bc.body2.parent);
						bc.body1.parent.addCollection(bc.body2.parent);
						bc.body1.parent.addInternalContact(bc);
						bc.body1.parent.addIncompleteCollectionContacts(bc.body2.parent, removalQueue);
					}
					else {
						bc.body2.merged = true;
						bodies.remove(bc.body1.parent);
						bc.body2.parent.addCollection(bc.body1.parent);
						bc.body2.parent.addInternalContact(bc);
						bc.body1.parent.addIncompleteCollectionContacts(bc.body2.parent, removalQueue);
					}
				}
				else if (bc.body1.isInCollection()) {
					//body1 is in a collection... body2 isnt
					bodies.remove(bc.body2);
					bc.body1.parent.addBody(bc.body2);
					bc.body1.parent.addInternalContact(bc);
					bc.body1.parent.addIncompleteContacts(bc.body2, removalQueue);
				}
				else if (bc.body2.isInCollection()) {
					//body2 is in a collection... body1 isnt
					bodies.remove(bc.body1);
					bc.body2.parent.addBody(bc.body1);
					bc.body2.parent.addInternalContact(bc);
					bc.body2.parent.addIncompleteContacts(bc.body1, removalQueue);
				}
				removalQueue.add(bc);
			}
		}
		for (BodyContact element : removalQueue) {
			collisionProcessor.bodyContacts.remove(element);
		}
	}

	private void downIndex(double i, ArrayList<RigidBody> bodyList) {
		for(RigidBody b: bodyList) {
			if (b.index > i) {
				b.index --;
			}
		}
	}

	/**
	 * Finds the body which has a block that intersects the provided point.
	 * @param p
	 * @return a body containing the given point
	 */
	public RigidBody pickBody( Point2d p ) {
		for ( RigidBody body : bodies ) {
			//must also check if intersects a collection
			if(body instanceof RigidCollection) {
				RigidBody b = collectionPick((RigidCollection) body, p);
				if (b!= null) return b;
			}
			else if ( body.intersect( p ) ) {
				return body;
			}
		}
		return null;
	}
	
	/**
	 * recurses through a collection to check if this point intersects any body. returns true if it does
	 * @param body 
	 * @param p
	 */
	private RigidBody collectionPick(RigidCollection body, Point2d p) {
		for (RigidBody b: body.collectionBodies ) {

			if (b.intersect(p)) {
				return b;
			}
		}
		return null;
	}

	/**
	 * Removes a rigid body from the system
	 * @param body
	 */
	public void remove( RigidBody body ) {
		bodies.remove(body);
	}

	/** 
	 * Resets the position of all bodies, and sets all velocities to zero
	 */
	public void reset() {
		//	bodies = this.originalBodies;
		int size = bodies.size();
		int counter = 0;
		boolean done = false;
		int i = 0;
		while(true) {
			if (bodies.size() == 0) break;
			RigidBody b = bodies.get(i);
			if (!b.created) {

				if (b instanceof RigidCollection) {
					for (RigidBody subBody: ((RigidCollection) b).collectionBodies) {
						bodies.add(subBody);
					}
					bodies.remove(b);

					b = bodies.get(i);
				}
				b.reset();


				for (Spring s: b.springs) {
					s.reset();
				}
			}
			else {
				counter = i;
				break;
			}

			i++;
			if (i >= bodies.size()) break;
		}

		int iter = size - counter;
		if ( counter > 0) {
			while(iter > 0) {
				this.bodies.remove(bodies.size() - 1);
				iter--;
			}
		}

		simulationTime = 0;
		collisionProcessor.reset();
		totalAccumulatedComputeTime = 0;        
	}

	private void collectionReset(RigidCollection b) {
		//resets the state of the rigidBodies inside all collections in the scene. 
		//loops through them and takes careful care not to mess up the index

		for (RigidBody subBody : b.collectionBodies) {
			subBody.reset();
			bodies.add(subBody);
		}
		bodies.remove(b);
		checkIndex();
	}
	
	/**
	 * if there are bodies with the index i, ups the index of all bodies greater than i to leave room 
	 * for the new body about to be "reintroduced" to the scene
	 */
	private void checkIndex() {
		int i = 0;
		nbCollections = 0;
		for(RigidBody b: bodies) {
			b.index = i;
			i++;
			if(b instanceof RigidCollection)
				nbCollections++;
		}
	}

	/**
	 * Removes all bodies from the system
	 */
	public void clear() {
		bodies.clear();
		RigidBody.nextIndex = 0;
		reset();
	}

	/**
	 * Draws all rigid bodies
	 * @param drawable
	 */
	public void display( GLAutoDrawable drawable ) {
		
		boolean updateCollectionColor = false;
		
		GL2 gl = drawable.getGL().getGL2();
		if ( Block.alpha != (float) (double) transparency.getValue()) {
			Block.alpha = (float) (double) transparency.getValue();
			// gross... need to rebuild display lists for the currently set transparency
			// which is potentially slow and bad news (memory thrashing) for openGL if we do this excessively!
			RigidBody.clearDisplayLists( gl );
			updateCollectionColor = true;
			for ( RigidBody b : bodies ) {
				b.myListID = -1;
			}
		}  
		
		if ( drawBodies.getValue() ) {
			int collectionId = 0;
			
			// Check if collections color should be updated
			// Can be removed if we build a consistent track of the collections id 
			for ( RigidBody b : bodies ) {
				if ( b instanceof RigidCollection && b.updateColor == true) {
					b.updateColor = false;
					updateCollectionColor = true;
				}	
			}
			
			for ( RigidBody b : bodies ) {
				if ( b instanceof RigidCollection) {
					RigidCollection collection = (RigidCollection)b;
					Color3f color = null;
					collectionId++;
					// Can be changed if we build a consistent track of the collections id 
					if(drawCollections.getValue() && updateCollectionColor) {
						float greenShade = collectionId/(float)nbCollections;
						float blueShade = 1 - collectionId/(float)nbCollections;
						color = new Color3f(0.f, greenShade, blueShade);
					}
					collection.displayCollection(drawable, color);
				}
				else {
					b.display(drawable);
				}

				for (Spring s : b.springs) {
					s.displayConnection(drawable);
				}
			}
		}

		gl.glLineWidth(1);
		if ( drawBoundingVolumes.getValue() ) {
			for ( RigidBody b : bodies ) {
				b.root.boundingDisc.display(drawable);
			}
		}
		
		if ( drawAllBoundingVolumes.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection))
					b.root.display( drawable );
				else {
					displayCollectionBV((RigidCollection) b, drawable);
				}
			}
		}      
		
		if ( drawBoundingVolumesUsed.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection))
					b.root.displayVisitBoundary( drawable, collisionProcessor.visitID );
				else displayVisitBoundaryCollection((RigidCollection) b, drawable);
			}
		}
		
		if ( drawContactGraph.getValue() ) {
			if (CollisionProcessor.use_contact_graph.getValue()) {
				for (RigidBody b : bodies) {
					for (Contact c:b.contactList) {
						c.displayConnection(drawable);
					}
				} 
			} 
			else {
				for ( Contact c : collisionProcessor.contacts ) {
					c.displayConnection(drawable);
				}
			}
		}
		
		if (drawConnections.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					((RigidCollection)b).displayConnection(drawable);
				}
			}
		}
		
		if(drawInternalContactForces.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
						((RigidCollection) b).drawInternalContacts(drawable);
				}
			}
		}
		
		if(drawInternalContactDeltas.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
						((RigidCollection) b).drawInternalDeltas(drawable);
				}
			}
		}
		
		if(drawInternalHistories.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
						((RigidCollection) b).drawInternalHistory(drawable);
				}
			}
		}
		
		if ( drawSpeedCOM.getValue() ) {
			for (RigidBody b: bodies) {
				b.drawSpeedCOM(drawable);
			}
		}

		if ( drawContacts.getValue() ) {
			for ( Contact c : collisionProcessor.contacts ) {
				c.display(drawable);
				if(drawExternalContactForces.getValue())
					c.drawContactForce(drawable);
			}
		}
		
		if ( drawCOMs.getValue() ) {
			for ( RigidBody b : bodies ) {
				b.displayCOM(drawable);
			}
		}
		
		if ( drawIndex.getValue()) {
			for (RigidBody b : bodies) {
				b.printIndex(drawable, GLUT.BITMAP_8_BY_13);
			}
		}
	}

	private void displayVisitBoundaryCollection(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.collectionBodies) {
			if (body instanceof RigidCollection) {
				displayVisitBoundaryCollection((RigidCollection) body, drawable);
			}
			else {
				body.root.displayVisitBoundary(drawable, collisionProcessor.visitID);
			}
		}
	}

	private void displayCollectionBV(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.collectionBodies) {
			if (body instanceof RigidCollection) {
				displayCollectionBV((RigidCollection) body, drawable);
			}
			else {
				body.root.display(drawable);
			}
		}

	}

	private DoubleParameter transparency = new DoubleParameter("body block transparency", .5, 0, 1 );
	private BooleanParameter drawBodies = new BooleanParameter( "draw bodies", true );
	private BooleanParameter drawCollections = new BooleanParameter( "draw collections", true );
	private BooleanParameter drawConnections = new BooleanParameter( "draw connections", false );
	private BooleanParameter drawBoundingVolumes = new BooleanParameter( "draw root bounding volumes", false );
	private BooleanParameter drawAllBoundingVolumes = new BooleanParameter( "draw ALL bounding volumes", false );
	private BooleanParameter drawBoundingVolumesUsed = new BooleanParameter( "draw bounding volumes used", false );
	private BooleanParameter drawInternalContactForces = new BooleanParameter("draw Internal Forces of merged collections", true);
	private BooleanParameter drawExternalContactForces = new BooleanParameter("draw External Forces", true);
	private BooleanParameter drawInternalContactDeltas = new BooleanParameter("draw Internal Deltas", true);
	private BooleanParameter drawInternalHistories = new BooleanParameter("draw Internal Histories", false );

	private BooleanParameter drawCOMs = new BooleanParameter( "draw center of mass positions", true );
	private BooleanParameter drawContacts = new BooleanParameter( "draw contact locations", true);
	private BooleanParameter drawContactGraph = new BooleanParameter( "draw contact graph", true );
	private BooleanParameter drawSpeedCOM = new BooleanParameter( "draw speed COM", false );
	private BooleanParameter processCollisions = new BooleanParameter( "process collisions", true );
	public static BooleanParameter enableMerging = new BooleanParameter( "enable merging", true);
	public static BooleanParameter enableSleeping = new BooleanParameter( "enable sleeping", false);
	public BooleanParameter drawIndex = new BooleanParameter( "dawIndex", false );


	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.setBorder( new TitledBorder("Rigid Body System Controls" ));

		VerticalFlowPanel vfpv = new VerticalFlowPanel();
		vfpv.setBorder( new TitledBorder("viewing controls") );
		vfpv.add( drawBodies.getControls() );
		vfpv.add( drawCollections.getControls() );
		vfpv.add( drawConnections.getControls() );
		vfpv.add( transparency.getSliderControls(false));
		vfpv.add( drawBoundingVolumes.getControls() );
		vfpv.add( drawAllBoundingVolumes.getControls() );
		vfpv.add( drawBoundingVolumesUsed.getControls() );
		vfpv.add( drawInternalContactForces.getControls() );
		vfpv.add( drawExternalContactForces.getControls() );
		vfpv.add( drawInternalHistories.getControls() );
		vfpv.add( drawInternalContactDeltas.getControls() );

		vfpv.add( drawCOMs.getControls() );
		vfpv.add( drawContacts.getControls() );
		vfpv.add( drawContactGraph.getControls() );
		vfpv.add( drawSpeedCOM.getControls() );
		vfpv.add( drawIndex.getControls() );

		CollapsiblePanel cp = new CollapsiblePanel(vfpv.getPanel());
		cp.collapse();
		vfp.add( cp );

		vfp.add( processCollisions.getControls() );
		vfp.add( enableMerging.getControls() );
		vfp.add( enableSleeping.getControls() );
		vfp.add( collisionProcessor.getControls() );

		vfp.add( useGravity.getControls() );
		vfp.add( gravityAmount.getSliderControls(false) );
		vfp.add( gravityAngle.getSliderControls(false) );

		vfp.add(spring_k.getSliderControls(false));
		vfp.add(spring_c.getSliderControls(false));
		vfp.add(springLength.getSliderControls());

		VerticalFlowPanel vfpv_2 = new VerticalFlowPanel();
		vfpv_2.setBorder( new TitledBorder("New body creation controls") );

		vfpv_2.add( origin_x.getSliderControls(false) );
		vfpv_2.add( origin_y.getSliderControls(false) );
		vfpv_2.add( theta.getSliderControls(false) );

		vfpv_2.add( velocity_x.getSliderControls(false) );
		vfpv_2.add( velocity_y.getSliderControls(false) );
		vfpv_2.add( omega.getSliderControls(false) );
		vfpv_2.add( index.getSliderControls() );
		CollapsiblePanel cp_2 = new CollapsiblePanel(vfpv_2.getPanel());
		cp_2.collapse();

		vfp.add(cp_2);
		return vfp.getPanel();
	}
	

	/** x value of origin of pendulum */
	public static DoubleParameter origin_x = new DoubleParameter("x position new body", 100, -100, 200 );

	/** y value of origin of pendulum */
	public static DoubleParameter origin_y = new DoubleParameter("y position of new body", 40, -100, 100 );

	/** theta value of origin of pendulum in degrees*/
	private DoubleParameter theta = new DoubleParameter("angle of creation", 0, -180, 180 );

	/** v_x value of origin of pendulum */
	private DoubleParameter velocity_x = new DoubleParameter("velocity x", -20, -100, 100 );

	/** v_y value of origin of pendulum */
	private DoubleParameter velocity_y = new DoubleParameter("velocity y", 0, -100, 100 );

	/** omega value of origin of pendulum */
	private DoubleParameter omega = new DoubleParameter("angular velocity", 0, -10, 10 );

	/** omega value of origin of pendulum */
	private IntParameter index = new IntParameter("body index", 61, 0, 200);

	public boolean generateBody = false;

	public void generateBody() {
		
		//get an unpinned random rigidbody
		RigidBody body = new RigidBody(bodies.get(0));
		RigidBody backup_body = new RigidBody(bodies.get(0));

		Boolean found = false;
		for( RigidBody b: bodies) {
			if (b.pinned) {
				continue;
			}
			if (b.index  != index.getValue()) {
				backup_body = new RigidBody(b);
				continue;			
			}
			if (b.index == index.getValue()) {
				found = true;
				body = new RigidBody(b);
			}
		}
		//degenerate case
		if (!found) {
			body = backup_body;
		}

		Point2d position = new Point2d(origin_x.getValue(), origin_y.getValue());
		Vector2d velocity = new Vector2d(velocity_x.getValue(), velocity_y.getValue());

		//also needs to scale the blocks of the body:
		//   body.scale(scale.getValue());
		body.x0.set(position);                        
		body.x.set(position);            
		body.theta = this.theta.getValue();
		body.omega = this.omega.getValue();
		body.v.set(velocity);
		body.updateTransformations();
		body.index = bodies.size();
		body.created = true;
		this.add(body);
	}
}

