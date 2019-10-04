package mergingBodies;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mergingBodies.Contact.ContactState;

public class RigidCollection extends RigidBody{

	/** List of RigidBody of the collection */
	protected ArrayList<RigidBody> collectionBodies = new ArrayList<RigidBody>();
	
	/** List of BodyContact in the collection: BodyContact between RigidBody of the collection */
	protected ArrayList<BodyPairContact> internalBodyPairContacts = new ArrayList<BodyPairContact>();
	
	/** List of Contact in the collection: Contact between RigidBody of the collection */
	protected ArrayList<Contact> internalContacts = new ArrayList<Contact>();

	boolean unmergedThisTimeStep = false;
	
	public Color3f color;

	CollisionProcessor collisionProcessor = new CollisionProcessor(collectionBodies);

	/**
	 * Creates a RigidCollection from two RigidBody.
	 * @param body1
	 * @param body2
	 */
	public RigidCollection(RigidBody body1, RigidBody body2) {
		
		// These bodies being added to the collection, with the collection being new,
		// their state w.r.t the collection frame is unchanged as C2W and W2C are Identity
		
		generateColor();
		
		if (body1.massLinear > body2.massLinear) 
			copyVelocitiesFrom(body1);
		else 
			copyVelocitiesFrom(body2);
		
		addBodyInternalMethod(body1);
		updateCollectionPinnedConditions(body1);
		addBodyInternalMethod(body2);
		updateCollectionPinnedConditions(body2);
		
		updateCollection();
	}
	
	/**
	 * Adds a body to the collection
	 * @param body body to add 
	 */
	public void addBody(RigidBody body) {
		addBodyInternalMethod(body);
		updateCollectionPinnedConditions(body);
		updateCollection();
	}
	
	/**
	 * Adds given list of bodies the collection
	 */
	public void addBodies(ArrayList<RigidBody> bodies) {
		for (RigidBody body : bodies) {		
			addBodyInternalMethod(body);
			updateCollectionPinnedConditions(body);
		}
		
		updateCollection();
	}
	
	/**
	 * Adds a collection to the collection
	 * @param collection collection to add 
	 */
	public void addCollection(RigidCollection collection) {
		for (RigidBody body : collection.collectionBodies)
			addBodyInternalMethod(body);
		
		internalBodyPairContacts.addAll(collection.internalBodyPairContacts);
		internalContacts.addAll(collection.internalContacts);
		
		updateCollectionPinnedConditions(collection);
		updateCollection();
	}
	
	/**
	 * Adds a body to the collection (internal method, for factoring purposes).
	 * @param body body to add 
	 */
	protected void addBodyInternalMethod(RigidBody body) {
		body.parent = this;
		body.merged = true;
		collectionBodies.add(body);
		
		v.add(body.v);
		v.scale(0.5);
		omega += body.omega;
		omega *= 0.5;
	}
	
	/**
	 * Copy velocities of given body
	 * @param body
	 */
	protected void copyVelocitiesFrom(RigidBody body) {
		v = new Vector2d(body.v);
		omega = body.omega;
	}
	
	/**
	 * Update collection pinned condition 
	 * @param body
	 */
	protected void updateCollectionPinnedConditions(RigidBody body) {
		temporarilyPinned = (temporarilyPinned || body.temporarilyPinned);
		steps = Math.max(body.steps, steps);
		body.temporarilyPinned = false;
		
		pinned = (pinned || body.pinned);
	}
	
	/**
	 * Computes transforms, COM, mass, inertia, spring.
	 */
	private void updateCollection() {
		
		if(pinned) {
			v.set(0.,0.);
			omega = 0.;
		}
		
		updateColor = true;
		
		updateMass();
		updateCOM(); 
		updateTransformations();
		updateBodiesTransformations();
		updateInertia();
		addBodiesSpringsToCollection();
	}

	/**
	 * Compute mass of collection w.r.t bodies
	 */
	public void updateMass() {
		
		double mass = 0;
		for (RigidBody body: collectionBodies) {
			mass += body.massLinear;
		}
		massLinear = mass;
		if (pinned)
			minv = 0.;
		else
			minv = 1/mass;
	}
	
	/**
	 * Loops through all bodies in collectionBodies
	 */
	public void updateCOM() {
		
		Point2d com = new Point2d();
		Point2d tmp = new Point2d();
		double totalMass = massLinear;
		com.set(0,0);

		for (RigidBody body: collectionBodies) {
			double ratio = body.massLinear/totalMass;
			tmp.scale(ratio, body.x);
			com.add(tmp);
		}
		x.set(com);
	}
	
	/** 
	 * For each body in collection, determines the transformations to go from body to collection
	 * But also, make each body's x and theta in collection, relative to this x and theta
	 */
	public void updateBodiesTransformations() {		
		for (RigidBody body: collectionBodies) {
			body.transformB2C.set(body.transformB2W);
			body.transformB2C.leftMult(transformW2B);
			body.transformC2B.set(body.transformB2C); 
			body.transformC2B.invert();
		}
	}
	
	/**
	 * Updates the angular inertia. 
	 */
	public void updateInertia() {
				
		double inertia = 0;
		Point2d tmp = new Point2d(0, 0);
		Point2d zero = new Point2d(0, 0);
		for (RigidBody body: collectionBodies) {
			
			for ( Block block : body.blocks ) {
				double mass = block.getColourMass();
				tmp.set(block.pB);
				body.transformB2C.transform(tmp);
				inertia += mass*tmp.distanceSquared(zero);
			}
		}
		massAngular = inertia;
		if (pinned)
			jinv = 0.;
		else
			jinv = 1./inertia;
	}
	
	/**
	 * Add given BodyPairContact to internalBodyPairContacts, and BodyPairContact's contactList to internalContacts
	 * @param bpc
	 */
	public void addInternalContact(BodyPairContact bpc) {
		
		if (!internalBodyPairContacts.contains(bpc)) {
			internalBodyPairContacts.add(bpc);
		} else {
			System.err.println("RigidCollection.addInternalContact : This should probably never happen");
		}
		
		if (!bpc.body1.bodyPairContactList.contains(bpc)) {
			System.err.println("RigidCollection.addInternalContact : Shouldn't bpc already be in the body1 list?");
			bpc.body1.bodyPairContactList.add(bpc);
		}
		
		if (!bpc.body2.bodyPairContactList.contains(bpc))  {
			System.err.println("RigidCollection.addInternalContact : Shouldn't bpc already be in the body2 list?");
			bpc.body2.bodyPairContactList.add(bpc);
		}

		for (Contact c: bpc.contactList) {
			c.getHistoryStatistics();
		}
	
		internalContacts.addAll(bpc.contactList);
	}
	
	/**
	 * Check if body is about to move w.r.t collection
	 * @param body
	 * @param dt
	 * @return
	 */
	public boolean checkUnmergeCondition(RigidBody body, double dt) {		
		
		Vector2d dv = new Vector2d();
		double domega;
		
		if(body.pinned)
			return false;
		
		dv.x = body.force.x * dt / body.massLinear + body.deltaV.get(0);
		dv.y = body.force.y * dt / body.massLinear + body.deltaV.get(1);
		domega = body.torque * dt / body.massAngular + body.deltaV.get(2);
		double metric = 0.5*dv.lengthSquared() + 0.5*domega*domega;

		if (metric > body.metric && metric > 1e-10) { // rule 0. possible motion 
		
			Vector2d ft = new Vector2d(0.,0.);
			Vector2d ftsum = new Vector2d(0.,0.);
			for (BodyPairContact bpc : body.bodyPairContactList) {
				for (Contact contact : bpc.contactList) { 
					if (contact.state == ContactState.BROKEN) { 
						//System.out.println(ContactState.BROKEN);
						body.metric = Double.MAX_VALUE;
						return true; // rule 1. if one contact has broken
					} else if (contact.state == ContactState.SLIDING) {
						ft.set(contact.jt.get(0)*contact.lambda.y, contact.jt.get(1)*contact.lambda.y);
						ftsum.add(ft);
					}
				}
			}
			
			if (0.5*ftsum.lengthSquared() > 1e-14) { 
				//System.out.println(ContactState.SLIDING);
				body.metric = Double.MAX_VALUE;
				return true; // rule 2. contacts on the edge of friction cone and norm of sum of forces not zero
			}
			
			if (metric>1e-4) {
				//System.out.println("MOVING");
				body.metric = Double.MAX_VALUE;
				return true; // rule 3. the body is unbalanced
			}
		}
	
		body.metric = metric;
		return false;
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
			updateBodiesPositionAndTransformations();
			applyVelocitiesToBodies();
			updateContactJacobianAndDataAsInternal(dt);
		} 
	}
	
	/**
	 * Updates bodies position, orientation, and transformations
	 */
	protected void updateBodiesPositionAndTransformations() {
		for (RigidBody body: collectionBodies) {
			//reset position and orientation 
			body.transformW2B.transform(body.x);
			body.theta = body.transformW2B.getTheta();

			//update transformations
			body.transformB2W.set(body.transformB2C);
			body.transformB2W.leftMult(transformB2W);
			body.transformW2B.set(body.transformB2W); 
			body.transformW2B.invert();

			//update position and orientation
			body.transformB2W.transform(body.x);
			body.theta = body.transformB2W.getTheta();
		}
	}
	
	/**
	 * Updates bodies velocities
	 */
	protected void applyVelocitiesToBodies() {
		for (RigidBody body: collectionBodies) {	
			applyVelocitiesTo(body);
		}
	}
	
	/**
	 * Clear deltaV of bodies
	 */
	protected void clearBodiesDeltaV() {
		for (RigidBody body: collectionBodies) {	
			body.deltaV.zero();
		}
	}
	
	/**
	 * Update all internal contacts Jacobians, along with position and normal data
	 * @param dt time step needed to get correct contact forces from lambda impulses (only for visualization)
	 */
	public void updateContactJacobianAndDataAsInternal(double dt) {
		for (Contact c : internalContacts) {
			c.normal = new Vector2d(c.normalB1);
			c.body1.transformB2W.transform(c.normal); 
			c.body1.transformB2W.transform(c.contactB1, c.contactW);
			c.computeJacobian(true);
			c.computeContactForce(dt);
		}
	}
	
	/**
	 * Compute internal contacts force w.r.t lambdas 
	 * @param dt
	 */
	public void computeInternalContactsForce(double dt) { 
		for (Contact c: internalContacts) {
			c.computeContactForce(dt);
		}
	}

	/** Applies springs on the body, to the collection */
	private void addBodiesSpringsToCollection() {
		springs.clear();
		for ( RigidBody body : collectionBodies ) {
			springs.addAll( body.springs );
		}
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

	/** list of bodies to be added to this collection in the next time step */
	ArrayList<RigidBody> bodyQueue = new ArrayList<RigidBody>();
	
	/**
	 * Makes body ready to be used by system... converts everything to world coordinates and makes body independent of collection
	 * ... does not do anything to the collection itself.
	 */
	public void unmergeSingleBody(RigidBody body) {
		if (!body.isInCollection()) 
			return;
		else {
			body.parent = null;
			body.updateColor = true;
		}
	}

	/**
	 * Go through all bodies and makes sure all the BodyContacts of each body is in the collection
	 */
	public void fillInternalBodyContacts() {
		for (RigidBody body: collectionBodies) {
			for (BodyPairContact bpc: body.bodyPairContactList) {
				if (!internalBodyPairContacts.contains(bpc) && bpc.inCollection == true) {
					RigidBody otherBody = bpc.getOtherBody(body);
					if (body.parent.collectionBodies.contains(otherBody)) {
						internalBodyPairContacts.add(bpc);
						for (Contact contact : bpc.contactList) {
							if (!internalContacts.contains(contact)) {
								internalContacts.add(contact);
							}
						}
					}
				}
			}
		}
	}
	
	/**
	 * We need to also add the other contacts that body has with the same collection it's being merged with.
	 * Must also add the BodyPairContact around the body that didn't reach 50 time steps but are still part of the same parents.
	 * The input parameter is the body being merged, and the body pair contact removal queue so that any BPCs 
	 * identified in this call can also be later removed. 
	 */
	public void addIncompleteContacts(RigidBody body, LinkedList<BodyPairContact> removalQueue) {
		for (BodyPairContact bpc: body.bodyPairContactList) {
			if (bpc.body1.parent == bpc.body2.parent && bpc.relativeKineticEnergyHist.size() <= CollisionProcessor.sleepAccum.getValue() && !bpc.inCollection) {
				bpc.inCollection = true;
				body.parent.addInternalContact(bpc);
				removalQueue.add(bpc);
			}
		}
	}

	/** input parameter is a collection being merged . we must add also all the incomplete contacts this parent has with other collections. */
	public void addIncompleteCollectionContacts(RigidCollection parent, LinkedList<BodyPairContact> removalQueue) {
		for (RigidBody body : parent.collectionBodies) {
			addIncompleteContacts(body, removalQueue);
		}
	}	
	
	public ArrayList<Contact> getInternalContacts() {
		ArrayList<Contact> contacts = new ArrayList<Contact>(internalContacts);
		return contacts;
	}
	
	protected void generateColor() {
		Random r = new Random();
		Random g = new Random();
		Random b = new Random();
		color = new Color3f(r.nextFloat(), g.nextFloat(), b.nextFloat());
	}
	
	/** display list ID for this rigid body */
	int myListID = -1;

	public void drawInternalContacts(GLAutoDrawable drawable) {

		for (BodyPairContact bc: internalBodyPairContacts) {
			if (!bc.inCollection) 
				continue;
			for (Contact c: bc.contactList) 
				c.drawInternalContactForce(drawable);
		}
	}

	public void drawInternalHistory(GLAutoDrawable drawable) {
		for (Contact c: internalContacts) {
			c.drawInternalContactHistory(drawable);;
		}
	}
	
	/**
	 * displays the Body Collection in different color.
	 * @param drawable
	 */
	public void displayCollection( GLAutoDrawable drawable, Color3f color ) {
		for (RigidBody b: collectionBodies) {
			if(color!=null)
				b.updateColor = true;
			b.display(drawable, color);
		}
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
		for (BodyPairContact bc: internalBodyPairContacts) {
			gl.glLineWidth(5);
			gl.glColor4f(0.f, 0.f, 0.f, 1.0f);
			gl.glBegin( GL.GL_LINES );
			p1.set(bc.body1.x);
			p2.set(bc.body2.x);
			gl.glVertex2d(p1.x, p1.y);
			gl.glVertex2d(p2.x, p2.y);
			gl.glEnd();
		}
	}
}
