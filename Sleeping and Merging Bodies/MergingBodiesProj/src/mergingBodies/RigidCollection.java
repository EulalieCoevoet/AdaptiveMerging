package mergingBodies;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class RigidCollection extends RigidBody{

	public LinkedList<RigidBody> colRemovalQueue = new LinkedList<RigidBody>();

	/** List of RigidBody of the collection */
	protected ArrayList<RigidBody> collectionBodies = new ArrayList<RigidBody>();
	/** List of BodyContact in the collection: BodyContact between RigidBody of the collection */
	protected ArrayList<BodyContact> internalBodyContacts = new ArrayList<BodyContact>();
	/** List of Contact in the collection: Contact between RigidBody of the collection */
	protected ArrayList<Contact> internalContacts = new ArrayList<Contact>();

	boolean unmergedThisTimeStep = false;
	boolean updatedThisTimeStep = false;

	CollisionProcessor collisionProcessor = new CollisionProcessor(collectionBodies);

	/**
	 * Create a RigidCollection from two RigidBody: 
	 * <p><ul>
	 * <li> copy body1, and add body1 and body2 to collectionBodies
	 * <li> save bodyContactList of RigidBody in bodyContactListPreMerging
	 * <li> update temporarilyPinned option: transfer to RigidCollection, removed from RigidBody
	 * </ul><p>
	 * @param body1
	 * @param body2
	 */
	public RigidCollection(RigidBody body1, RigidBody body2) {
		
		super(body1); // this will copy the blocks, which is not exactly what we want... fine though.
		clearUselessJunk(); // clear blocks and other things...

		collectionBodies.add(body1);
		collectionBodies.add(body2);
		body1.bodyContactListPreMerging.clear();
		body2.bodyContactListPreMerging.clear();
		body1.bodyContactListPreMerging.addAll(body1.bodyContactList);
		body2.bodyContactListPreMerging.addAll(body2.bodyContactList);

		setupCollection();

		body1.parent = this;
		body2.parent = this;
		body1.merged = true;
		body2.merged = true;
		
		updateColor = true;
		
		temporarilyPinned = (body1.temporarilyPinned || body2.temporarilyPinned);
		steps = Math.max(body1.steps, body2.steps);
		
		body1.temporarilyPinned = false;
		body2.temporarilyPinned = false;
	}


	private void clearUselessJunk() {
		blocks.clear();
		boundaryBlocks.clear();
		velHistory.clear();
		savedContactForce.set(0, 0);
		contactTorques = 0;
		springs.clear();
	}


	/**
	 * Adds a body to the collection
	 * @param body body to add 
	 */
	public void addBody(RigidBody body) {
		body.bodyContactListPreMerging.addAll(body.bodyContactList);
		collectionBodies.add(body);
		setupCollection();

		body.parent = this;
		body.merged = true;
		updateColor = true;
	}


	/**
	 * Add given BodyContact to internalBodyContacts, and BodyContact's contactList to internalContacts
	 * @param bc
	 */
	public void addInternalContact(BodyContact bc) {
		if (!internalBodyContacts.contains(bc))
			internalBodyContacts.add(bc);
		if (!bc.body1.bodyContactList.contains(bc)) {
			bc.body1.bodyContactList.add(bc);
		}
		if (!bc.body2.bodyContactList.contains(bc)) {
			bc.body2.bodyContactList.add(bc);
		}

		//convert the contacts to collections coordinates. 
		for (Contact c: bc.contactList)
			c.getHistoryStatistics();

		internalContacts.addAll(bc.contactList);
	}
	
	
	/**
	 * Adds a collection to the collection
	 * @param col collection to add 
	 */
	public void addCollection(RigidCollection col) {
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
		for (RigidBody b : col.collectionBodies) {
			//transform all the subBodies to their world coordinates... 
			b.merged = true;
			updateColor = true;
			col.transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.parent = null; // eulalie : why not this?
			additionQueue.add(b);
		}

		for (RigidBody b: additionQueue) {
			b.bodyContactListPreMerging.addAll(b.bodyContactList);
			collectionBodies.add(b);
		}
		//col.collectionBodies.clear();

		setupCollection();

		// handle the internal contacts of col... they no longer beloogn
		internalBodyContacts.addAll(col.internalBodyContacts);

		for (Contact c: col.internalContacts) {

			internalContacts.add(c);
			//col.transformB2W.transform(c.contactW);
			col.transformB2W.transform(c.normal);
		}
	}
	
	/**
	 * Update the contact's Jacobian of bodies inside a given collection
	 * @param collection
	 */
	public void updateBodiesJacobian() {
		for (Contact c : internalContacts) {
			Vector2d normal = new Vector2d(c.normalB1);
			c.body1.transformB2W.transform(normal);
			c.computeJacobian(normal);
		}
	}
	
	/**
	 * Update forces applied on bodies inside a given collection:
	 * <p><ul>
	 * <li> gravity is already added
	 * <li> spring forces are already added 
	 * <li> in this method, we add the new external contact forces
	 * <li> (note) this force is set to zero at each beginning of time step
	 * </ul><p>
	 * @param collection
	 */
	public void updateBodiesForces(double dt) {
		for (RigidBody body : collectionBodies) { 
			for (BodyContact bc : body.bodyContactList) { 
				RigidBody otherBody = bc.getOtherBody(body);
				if (!otherBody.isInCollection(this)) {
					for (Contact c : bc.contactList) {
							
						Vector2d force = new Vector2d();
						double torque;
						
						if (c.body1==body) {
							force.set(c.lambda.x*c.j1.get(0) + c.lambda.y*c.j2.get(0), c.lambda.x*c.j1.get(1) + c.lambda.y*c.j2.get(1));
							torque = c.lambda.x*c.j1.get(2) + c.lambda.y*c.j2.get(2);
						} else {
							force.set(c.lambda.x*c.j1.get(3) + c.lambda.y*c.j2.get(3), c.lambda.x*c.j1.get(4) + c.lambda.y*c.j2.get(4));
							torque = c.lambda.x*c.j1.get(5) + c.lambda.y*c.j2.get(5);
						}
						
						force.scale(1./dt);
						body.transformW2B.transform(force);
						torque /= dt;

						body.force.add(force);
						body.torque += torque;
					}
				}
			}
		}
	}	
	
	public void updateBodiesVelocity() {
		for (RigidBody body : collectionBodies) {
			body.v.set(v);
			body.omega = omega;
		}
	}
	
	public void updateBodiesContactForces(double dt) { 
		Vector2d cForce = new Vector2d();
		double cTorque = 0;
		for (Contact c: internalContacts) {
			
			cForce.set(c.lambda.x*c.j1.get(0) + c.lambda.y*c.j2.get(0),c.lambda.x*c.j1.get(1) + c.lambda.y*c.j2.get(1) );
			cTorque = c.lambda.x*c.j1.get(2) + c.lambda.y*c.j2.get(2);
			cForce.scale(1/dt);
			c.body1.transformW2B.transform(cForce);
			c.contactForceB1.set(cForce);
			c.contactTorqueB1 = cTorque/dt;

			cForce.set(c.lambda.x*c.j1.get(3) + c.lambda.y*c.j2.get(3),c.lambda.x*c.j1.get(4) + c.lambda.y*c.j2.get(4) );
			cTorque = c.lambda.x*c.j1.get(5) + c.lambda.y*c.j2.get(5);
			cForce.scale(1/dt);
			c.body2.transformW2B.transform(cForce);
			c.contactForceB2.set(cForce);
			c.contactTorqueB2 = cTorque/dt;
		}
	}

	/** 
	 * Separates each collectionBody from its parent so the bodies are ready to be added back to the system individually.
	 * x positions now need to be in world coordinates etc.
	 */
	public void unmergeAllBodies() {
		internalBodyContacts.clear();
		for (RigidBody b: collectionBodies) {
			transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.v.set(v);
			b.omega = omega;
			b.parent = null;
		}
	}
	
	/**
	 * Check if body is about to move w.r.t collection
	 * @param body
	 * @param dt
	 * @return
	 */
	public boolean checkRelativeVelocity(RigidBody body, double dt) {
		Vector2d v_rel = new Vector2d(0.,0.);
		double omega_rel = 0.;
		
		v_rel.x += body.force.x * dt / body.massLinear + body.deltaV.get(0);
		v_rel.y += body.force.y * dt / body.massLinear + body.deltaV.get(1);
		omega_rel += body.torque * dt / body.massAngular + body.deltaV.get(2);
		
		double metric = 0.5*v_rel.lengthSquared() + 0.5*omega_rel*omega_rel;
		return (metric>1e-3);
	}

	@Override
	public void advanceTime(double dt){

		if(temporarilyPinned && ++steps>=200)
			temporarilyPinned=!temporarilyPinned; 
		
		if (!pinned && !temporarilyPinned) {
			v.x += force.x * dt/massLinear + deltaV.get(0);
			v.y += force.y * dt/massLinear + deltaV.get(1);
			omega += torque * dt/ massAngular + deltaV.get(2);
			if (state == ObjectState.ACTIVE) {
				x.x += v.x * dt;
				x.y += v.y * dt;
				theta += omega*dt;
			} 
			updateTransformations();
			updateCollectionBodyTransformations(dt);
		} 
	}

	/** applies springs on the body, to the collection */
	private void addSprings() {
		ArrayList<Spring> newSprings = new ArrayList<Spring>();

		for (RigidBody body: collectionBodies) {
			for (Spring s: body.springs) {
				Spring newSpring = new Spring(s, this);
				newSprings.add(newSpring);
			}
		}
		this.springs.addAll(newSprings);
	}

	@Override
	/**
	 * Updates the B2W and W2B transformations
	 */
	public void updateTransformations() {
		transformB2W.set( theta, x );
		transformW2B.set(transformB2W);
		transformW2B.invert();
	}

	private void updateCollectionBodyTransformations(double dt) {
		//WHY doesn't this work!
		//eulalie : you sure it does not work?
		for (RigidBody b: collectionBodies) {
			b.transformB2W.set(b.transformB2C);
			b.transformB2W.leftMult(transformB2W);
			b.transformW2B.set(b.transformB2W); 
			b.transformW2B.invert();
		}
	}
	

	/** 
	 * For each body in collection, determine the transformations to go from body to collection
	 * But also, make each body's x, in collection and theta in collection, relative to this x and theta
	 */
	private void setMergedTransformationMatrices() {

		for (RigidBody body: collectionBodies) {
			body.transformB2C.set(body.transformB2W);
			body.transformB2C.leftMult(transformW2B);
			body.transformC2B.set(body.transformB2C); 
			body.transformC2B.invert();
		}
	}

	
	/**
	 * transforms body x's and thetas in world coordinates into collection cooridnates
	 */
	private void transformToCollectionCoords(){
		//RigidTransform temp = new RigidTransform();
		for (RigidBody b : collectionBodies) {
			transformW2B.transform(b.x);
			b.theta = b.transformB2C.getTheta();
		}
	}

	/**
	 * displays the Body Collection in different color.
	 * @param drawable
	 */
	public void displayCollection( GLAutoDrawable drawable, Color3f color ) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glPushMatrix();
		gl.glTranslated( x.x, x.y, 0 );
		gl.glRotated(theta*180/Math.PI, 0,0,1);

		for (RigidBody b: collectionBodies) {
			if(color!=null)
				b.updateColor = true;
			b.display(drawable, color);
		}
		gl.glPopMatrix();
	}

	public void calculateMass() {
		double mass = 0;
		for(RigidBody b: collectionBodies) {
			mass += b.massLinear;
		}
		massLinear = mass;
		minv = 1/mass;
	}

	/**
	 * displays the Body Collection as lines between the center of masses of each rigid body to the other. 
	 * Uses a string arrayList to check if a connection has already been drawn.
	 * @param drawable
	 */
	public void displayConnection( GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();

		// draw a line between the two bodies but only if they're both not pinned
		Point2d p1 = new Point2d();
		Point2d p2 = new Point2d();
		for (BodyContact bc: internalBodyContacts) {
			gl.glLineWidth(5);
			gl.glColor4f(0.f, 0.f, 0.f, 1.0f);
			gl.glBegin( GL.GL_LINES );
			p1.set(bc.body1.x);
			p2.set(bc.body2.x);
			if (bc.body1.isInCollection())
				bc.body1.parent.transformB2W.transform(p1);
			if (bc.body2.isInCollection())
				bc.body2.parent.transformB2W.transform(p2);
			gl.glVertex2d(p1.x, p1.y);
			gl.glVertex2d(p2.x, p2.y);
			gl.glEnd();
		}
	}



	/**
	 * Loops through all bodies in collectionBodies and sets the transformation matrices for each
	 */
	public void calculateCOM() {
		Vector2d com = new Vector2d();
		Vector2d bCOM = new Vector2d();
		double totalMass = massLinear;
		com.set(0,0);


		for (RigidBody b: collectionBodies) {
			double bRatio = b.massLinear/totalMass;
			if (b.isInCollection()) {
				//transform back to world if the body is in collection coordinates... dont worry, very temporary
				b.parent.transformB2W.transform(b.x);
			}
			bCOM.scale(bRatio, b.x);
			com.add(bCOM);
		}

		//	set collections new transformations...
		x.set(com);

		transformB2W.set( theta, x );
		transformW2B.set( transformB2W);
		transformW2B.invert();
	}

	/**
	 * Calculates the angular inertia. 
	 */
	public void calculateInertia() {
		double inertia = 0;
		Point2d bpB = new Point2d(0, 0);
		Point2d zero = new Point2d(0, 0);
		for (RigidBody body: collectionBodies) {
			for ( Block b : body.blocks ) {
				double mass = b.getColourMass();
				bpB.set(b.pB);
				body.transformB2C.transform(bpB);
				//b.pb is in c
				inertia += mass*bpB.distanceSquared(zero);

			}
			body.parent = this;

		}
		massAngular = inertia;
		jinv = 1/inertia;
	}

	/** display list ID for this rigid body */
	int myListID = -1;

	//list of bodies to be added to this collection in the next timestep
	ArrayList<RigidBody> bodyQueue = new ArrayList<RigidBody>();
	
	/**
	 * Checks if body sB is going to unmerge by comparing forces acting on the object with a threshold.
	 * @param sB rigid body to check
	 * @param totalForce output total force applied on the body
	 * @param totalTorque output total torque applied on the body
	 * @return true if should unmerge
	 */
	public boolean metricCheck(RigidBody sB , Vector2d totalForce, double totalTorque) {
		totalForce.set(sB.force);
		sB.transformB2W.transform(sB.currentContactForce);

		totalForce.add(sB.currentContactForce);

		sB.transformW2B.transform(sB.currentContactForce);
		sB.deltaF.set(totalForce);
		totalTorque = sB.torque + sB.currentContactTorques;
		double threshold = CollisionProcessor.forceMetricTolerance.getValue();
		double forceMetric = totalForce.x/sB.massLinear;
		double forceMetric2 = totalForce.y/sB.massLinear;
		double forceMetric3 = totalTorque/sB.massAngular;
		if (Math.abs(forceMetric) > threshold || Math.abs(forceMetric2) > threshold || Math.abs(forceMetric3) > threshold) 
			return true;
		else return false;
	}
	
	/**
	 * Makes body ready to be used by system... converts everything to world coordinates and makes body independant of collection
	 * ... does not do anything to the collection itself.
	 */
	public void unmergeSingleBody(RigidBody sB) {
		if (!sB.isInCollection()) return;
		else {
			sB.parent.transformB2W.transform(sB.x);
			sB.theta = sB.transformB2W.getTheta();
			sB.transformB2C.T.setIdentity();
			sB.transformC2B.T.setIdentity();
			sB.v.set(sB.parent.v);
			sB.omega = sB.parent.omega;
			sB.parent = null;
			sB.updateColor = true;
		}
	}
	
	
	private void removeBodyContact(BodyContact bc) {
		internalBodyContacts.remove(bc);
	}
	
	//contains all the new RigidBodies that result after this unmerging
	ArrayList<RigidBody> newRigidBodies = new ArrayList<RigidBody>();

	//the bodies here have been handled in this unmerging step...
	ArrayList<RigidBody> handledBodies = new ArrayList<RigidBody>();
	
	/**
	 * loops through the current unmerged body's neighbors. each one becomes the source of a new RigidCollection surrounding the body
	 */
	private void dealWithNeighbors(RigidBody sB) {
		for (BodyContact bc : sB.bodyContactList ) {
			if (bc.merged) {
				bc.merged = false;
				RigidBody body2 = bc.getOtherBody(sB);
				handledBodies.add(body2);
				neighborCollection.add(body2);
				makeNeighborCollection(body2);
				if (neighborCollection.size() >= 2) {
					//make a new collection
					RigidCollection newCollection = new RigidCollection(neighborCollection.remove(0), neighborCollection.remove(0));
					newCollection.addBodies(neighborCollection);
					newCollection.fillInternalBodyContacts();
					contactsToBody();
					newCollection.v.set(v);
					newCollection.omega = omega;
					newRigidBodies.add(newCollection);
					neighborCollection.clear();
				}
				else if ( neighborCollection.size() == 1){
					unmergeSingleBody(neighborCollection.get(0));
					newRigidBodies.add(neighborCollection.remove(0));
					contactsToBody();
				}
			}
		}
	}
	

	public void contactsToBody() {
		for (Contact c: internalContacts) {
			transformW2B.transform(c.normal);
		}
	}


	/**
	 * Go through all bodies and makes sure all the BodyContacts of each body is in the collection
	 */
	public void fillInternalBodyContacts() {
		for (RigidBody b: collectionBodies) {
			for (BodyContact bc: b.bodyContactList) {
				if (!internalBodyContacts.contains(bc) && bc.merged == true) {
					RigidBody body2 = bc.getOtherBody(b);
					if (b.parent.collectionBodies.contains(body2)) {
						internalBodyContacts.add(bc);
						for (Contact c : bc.contactList) {
							if (!internalContacts.contains(c)) {
								internalContacts.add(c);
							}
						}
					}
				}
			}
		}
	}


	/**
	 * Add list of bodies to rigidCollection
	 */
	public void addBodies(ArrayList<RigidBody> bodyList) {
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
		for (RigidBody b : bodyList) {
			b.bodyContactListPreMerging.clear();
			//transform all the subBodies to their world coordinates... 
			b.merged = true;
			if (b.isInCollection()) b.parent.transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.parent = null; // eulalie : why not this?
			additionQueue.add(b);
			b.bodyContactListPreMerging.addAll(b.bodyContactList);
		}

		for (RigidBody b: additionQueue) {
			collectionBodies.add(b);
		}

		setupCollection();
	}
	
	
	/**
	 * given input body, finds all the bodies this body is connected to and makes them into a single rigidCollection
	 */
	public ArrayList<RigidBody> neighborCollection = new ArrayList<RigidBody>();

	private void makeNeighborCollection(RigidBody body) {
		for (BodyContact bc : body.bodyContactList) {
			if( !bc.merged) continue;
			RigidBody body2 = bc.getOtherBody(body);
			if (handledBodies.contains(body2) ) continue;
			else {
				neighborCollection.add(body2);
				handledBodies.add(body2);
				makeNeighborCollection(body2);
			}
		}
	}


	/**
	 * Removes a body from the current collection, without changing the other Bodies
	 */
	public void unmergeSelectBodies() {
		// TODO Auto-generated method stub

		for (RigidBody b : colRemovalQueue) {
			b.bodyContactList.clear();
			transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.v.set(v);
			b.omega = omega;
			b.parent = null;
		}
		
		colRemovalQueue.clear();
		//reset up the collection
		setupCollection();
	}
	

	public void unmergeSelectBodies(ArrayList<RigidBody> bodies) {
		// TODO Auto-generated method stub

		for (RigidBody b : bodies) {
			b.bodyContactList.clear();
			transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.v.set(v);
			b.omega = omega;
			b.parent = null;
		}
		setupCollection();
	}

	/**
	 * Basic setup of the collection... calculates transforms, COM, mass, inertia, etc.
	 */
	private void setupCollection() {
		contactsToWorld();
		calculateMass();
		calculateCOM();
		//from this new collection COM determine new transforms for each body in the collection
		//set Transform B2C and C2B for each body
		setMergedTransformationMatrices();
		transformToCollectionCoords();
		calculateInertia();
		//update BVNode roots for each body
		springs.clear();
		addSprings();
	}


	public void contactsToWorld() {
		for (Contact c: internalContacts)
			transformB2W.transform(c.normal);
	}


	public void drawInternalContacts(GLAutoDrawable drawable) {

		for (BodyContact bc: internalBodyContacts) {
			if (!bc.merged) 
				continue;
			for (Contact c: bc.contactList) 
				c.drawInternalContactForce(drawable);
		}
	}

	
	/**
	 * input parameter is a body being merged. We need to also add the other contacts that body has with the collection it's being merged with
		//Must also add the bodycontacts around the body that didn't reach 50 timesteps but are still part of the same parents.

	 */
	public void addIncompleteContacts(RigidBody body, LinkedList<BodyContact> removalQueue) {
		for (BodyContact bc: body.bodyContactList) {
			if (bc.body1.parent == bc.body2.parent && bc.relativeVelHistory.size() <= CollisionProcessor.sleepAccum.getValue() && !bc.merged) {
				bc.merged = true;
				body.parent.addInternalContact(bc);
				removalQueue.add(bc);
			}
		}
	}

	//input parameter is a collection being merged . we must add also all the incomplete contacts this parent has with other collections.

	public void addIncompleteCollectionContacts(RigidCollection parent, LinkedList<BodyContact> removalQueue) {
		for (RigidBody b: parent.collectionBodies) {
			addIncompleteContacts(b, removalQueue);
		}

	}


	public void drawInternalDeltas(GLAutoDrawable drawable) {
		for (RigidBody b: collectionBodies) {
			b.drawDeltaF(drawable);
		}
	}


	public void drawInternalHistory(GLAutoDrawable drawable) {
		for (Contact c: internalContacts) {
			c.drawInternalContactHistory(drawable);;
		}
	}

}
