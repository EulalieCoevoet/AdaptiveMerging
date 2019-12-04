package mergingBodies;

import java.util.ArrayList;
import java.util.LinkedList;

import javax.vecmath.Color3f;
import javax.vecmath.Color4f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mergingBodies.RigidBodySystem.MergeParameters;

public class RigidCollection extends RigidBody{

	/** List of RigidBody of the collection */
	protected ArrayList<RigidBody> collectionBodies = new ArrayList<RigidBody>();
	
	/** List of Contact in the collection: Contact between RigidBody of the collection */
	protected ArrayList<Contact> internalContacts = new ArrayList<Contact>();
	
	public Color color;

	CollisionProcessor collisionProcessor = new CollisionProcessor(collectionBodies);
	MotionMetricProcessor motionMetricProcessor = new MotionMetricProcessor();

	/**
	 * Creates a RigidCollection from two RigidBody.
	 * @param body1
	 * @param body2
	 */
	public RigidCollection(RigidBody body1, RigidBody body2) {
		
		// These bodies being added to the collection, with the collection being new,
		// their state w.r.t the collection frame is unchanged as C2W and W2C are Identity
		
		color = new Color();
		color.setRandomColor();
		
		copyFrom(body1);
		
		addBodyInternalMethod(body1);
		updateCollectionState(body1);
		addBodyInternalMethod(body2);
		updateCollectionState(body2);
		
		updateCollection();
	}
	
	/**
	 * Adds a body to the collection
	 * @param body body to add 
	 */
	public void addBody(RigidBody body) {
		addBodyInternalMethod(body);
		updateCollectionState(body);
		updateCollection();
	}
	
	/**
	 * Adds given list of bodies the collection
	 */
	public void addBodies(ArrayList<RigidBody> bodies) {
		for (RigidBody body : bodies) {
			addBodyInternalMethod(body);
			updateCollectionState(body);
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
		
		updateCollectionState(collection);
		updateCollection();
	}
	
	/**
	 * Adds a body to the collection (internal method, for factoring purposes).
	 * @param body body to add 
	 */
	protected void addBodyInternalMethod(RigidBody body) {
		body.parent = this;
		body.mergedThisTimeStep = true;
		collectionBodies.add(body);

		updateVelocitiesFrom(body);		
	}
	
	protected void updateVelocitiesFrom(RigidBody body) {
		Point2d massCom1 = new Point2d();
		Point2d massCom2 = new Point2d();
		massCom1.scale( body.massLinear, body.x );
		massCom2.scale( massLinear, x );		
		Point2d newCom = new Point2d();
		newCom.add( massCom1, massCom2 );
		newCom.scale( 1./(body.massLinear + massLinear) );

		Vector2d tmp1 = new Vector2d();
		Vector2d tmp2 = new Vector2d();
		Vector2d tmp3 = new Vector2d();
		
		tmp1.sub( newCom, body.x );
		tmp1.scale( body.omega );
		tmp2.set( -tmp1.y, tmp1.x );
		tmp2.add(body.v);
		tmp2.scale(body.massLinear);
		tmp3.add(tmp2);
		
		
		tmp1.sub( newCom, x );
		tmp1.scale( omega );
		tmp2.set( -tmp1.y, tmp1.x );
		tmp2.add(v);
		tmp2.scale(massLinear);
		tmp3.add(tmp2);
		
		tmp3.scale(1/(body.massLinear+massLinear));
		
		v.set(tmp3);
		
		omega = (omega*massAngular+body.omega*body.massAngular)/(body.massAngular+massAngular);
	}
	
	@Override
	public void clear() {
		super.clear();
		
		for (RigidBody body: collectionBodies) {
			applyVelocitiesTo(body);
			body.clear();
		}
	}
	
	/**
	 * Copy velocities of given body
	 * @param body
	 */
	protected void copyFrom(RigidBody body) {
		v.set(body.v);
		omega = body.omega;
		x.set(body.x);
		theta = body.theta;
		massLinear = body.massLinear;
	}
	
	/**
	 * Update collection pinned condition 
	 * @param body
	 */
	protected void updateCollectionState(RigidBody body) {
		temporarilyPinned = (temporarilyPinned || body.temporarilyPinned);
		body.temporarilyPinned = false;
		steps = Math.max(body.steps, steps);
		
		pinned = (pinned || body.pinned);
		
		isSleeping = (isSleeping || body.isSleeping);
		body.isSleeping = false;
	}
	
	/**
	 * Computes transforms, COM, mass, inertia, spring.
	 */
	private void updateCollection() {
		
		if(pinned || temporarilyPinned) {
			v.set(0.,0.);
			omega = 0.;
		}
		
		updateMass();
		updateCOM(); 
		updateTheta();
		updateTransformations();
		updateBodiesTransformations();
		updateBB(); 
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
	 * Compute theta of the collection from convex hull informations
	 */
	protected void updateTheta() {

		int N = 0;
		Point2d meanPos = new Point2d();
		
		for ( RigidBody body : collectionBodies ) {
			if(body instanceof PlaneRigidBody)
				continue;
			
			for (Point2d point : body.boundingBoxB) {
				Point2d p = new Point2d(point);
				transformB2C.transform(p);
				meanPos.add( p );
				N++;
			}
		}
		meanPos.scale( 1.0 / N );

		Vector2d v = new Vector2d();
		Matrix2d covariance = new Matrix2d();
		for ( RigidBody body : collectionBodies ) {
			if(body instanceof PlaneRigidBody)
				continue;
			
			for (Point2d point : body.boundingBoxB) {
				Point2d p = new Point2d(point);
				transformB2C.transform(p);
				v.sub( p, meanPos );			
				covariance.rank1( 1.0 / N, v );
			}
		}
		
		covariance.evd();
		Vector2d dir = covariance.v1;
		dir.normalize();
		if (!Double.isNaN(Math.acos(dir.x)))
			theta += Math.acos(dir.x);
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
	
	protected void updateBB() {
		Point2d bbmaxB = new Point2d(-Double.MAX_VALUE,-Double.MAX_VALUE); 
		Point2d bbminB = new Point2d(Double.MAX_VALUE,Double.MAX_VALUE);
		for (RigidBody body : collectionBodies) {
			if (body instanceof PlaneRigidBody)
				continue;
			
			for (Point2d point : body.boundingBoxB) { 
				body.transformB2C.transform(point);
				bbmaxB.x = Math.max(bbmaxB.x, point.x);
				bbmaxB.y = Math.max(bbmaxB.y, point.y);
				bbminB.x = Math.min(bbminB.x, point.x);
				bbminB.y = Math.min(bbminB.y, point.y);
				body.transformC2B.transform(point);
			}
		}
		boundingBoxB = new ArrayList<Point2d>();
		boundingBoxB.add(0, bbmaxB);
		boundingBoxB.add(1, new Point2d(bbmaxB.x,bbminB.y));
		boundingBoxB.add(2, bbminB);
		boundingBoxB.add(3, new Point2d(bbminB.x,bbmaxB.y));
	}
	
	/**
	 * Updates the angular inertia. 
	 */
	public void updateInertia() {
				
		double inertia = 0;
		Point2d tmp = new Point2d(0, 0);
		Point2d zero = new Point2d(0, 0);
		for (RigidBody body: collectionBodies) {
			if (!(body instanceof PlaneRigidBody)) {
				for ( Block block : body.blocks ) {
					double mass = block.getColourMass();
					tmp.set(block.pB);
					body.transformB2C.transform(tmp);
					inertia += mass*tmp.distanceSquared(zero);
				}
			}
		}
		massAngular = inertia;
		if (pinned)
			jinv = 0.;
		else
			jinv = 1./inertia;
	}
	
	public void addToInternalContact(BodyPairContact bpc) {
		
		for (Contact c: bpc.contactList) {
			c.getHistoryStatistics();
		}
	
		internalContacts.addAll(bpc.contactList);
	}
	
	public void addToBodyPairContacts(BodyPairContact bpc) {
		
		bpc.addToBodyListsParent();
		
		// also add the external bpc to the collection bodyPairContactList
		for (BodyPairContact bpcExt : bpc.body1.bodyPairContactList) 
			bpcExt.addToBodyListsParent();
		for (BodyPairContact bpcExt : bpc.body2.bodyPairContactList) 
			bpcExt.addToBodyListsParent();
	}

	@Override
	public void advanceTime(double dt){

		super.advanceTime(dt);
		
		if (pinned || temporarilyPinned || isSleeping)
			return;
		
		updateBodiesPositionAndTransformations();
		collisionProcessor.updateContactJacobianAsInternal(internalContacts);
		computeInternalContactsForce(dt);
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
	 * Apply the linear and angular velocities to the given body
	 * @param body
	 */
	public void applyVelocitiesTo(RigidBody body) {
		if(pinned || temporarilyPinned) {
			if(v.lengthSquared() > 1e-14 || omega > 1e-14)
				System.err.println("[applyVelocitiesTo] velocities of pinned body is not zero.");
		}
		
    	final Vector2d rw = new Vector2d( -(body.x.y - x.y), body.x.x - x.x );
		rw.scale( omega );
		body.v.add(v, rw); // sets the value of the sum
		body.omega = omega;
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
	
	public boolean isMovingAway(RigidBody body, MergeParameters mergeParams) {
		
		motionMetricProcessor.setMotionMetricType(mergeParams.motionMetricType.getValue());
		double metric = motionMetricProcessor.getMotionMetric(this, body);
		
		if(pinned || temporarilyPinned) 
			metric/=2; 
		
		return (metric> mergeParams.thresholdUnmerge.getValue());
	}
	
	/**
	 * Makes body ready to be used by system... converts everything to world coordinates and makes body independent of collection
	 * ... does not do anything to the collection itself.
	 */
	public void unmergeSingleBody(RigidBody body) {
		if (!body.isInCollection(this)) {
			System.err.println("[unmergeSingleBody] Not suppose to happen.");
			return;
		} else {
			applyVelocitiesTo(body);
			body.deltaV.zero();
			body.parent = null;
		}
	}

	/**
	 * Go through all bodies and makes sure all the BodyContacts of each body is in the collection
	 */
	public void fillInternalBodyContacts() {
		for (RigidBody body: collectionBodies) {
			for (BodyPairContact bpc: body.bodyPairContactList) {
				if (!bodyPairContactList.contains(bpc)) {
					bodyPairContactList.add(bpc);
					if(bpc.inCollection == true) {
						RigidBody otherBody = bpc.getOtherBody(body);
						if (body.parent.collectionBodies.contains(otherBody)) {
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
	}
	
	/**
	 * We need to also add the other contacts that body has with the same collection it's being merged with.
	 * Must also add the BodyPairContact around the body that didn't reach 50 time steps but are still part of the same parents.
	 * The input parameter is the body being merged, and the body pair contact removal queue so that any BPCs 
	 * identified in this call can also be later removed. 
	 */
	public void addIncompleteContacts(RigidBody body, LinkedList<BodyPairContact> removalQueue) {
		for (BodyPairContact bpc: body.bodyPairContactList) {
			if (bpc.body1.isInSameCollection(bpc.body2) && !bpc.inCollection) {
				bpc.inCollection = true;
				body.parent.addToInternalContact(bpc);
				body.parent.addToBodyPairContacts(bpc);
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

	public void displayInternalContactForces(GLAutoDrawable drawable) {

		for (BodyPairContact bpc: bodyPairContactList) {
			if (!bpc.inCollection) 
				continue;
			for (Contact c: bpc.contactList) 
				c.displayContactForce(drawable, new Color3f(0,0,1)); //blue inside collection
		}
	}
	
	public void displayInternalContactLocations(GLAutoDrawable drawable, int size) {

		for (BodyPairContact bpc: bodyPairContactList) {
			if (!bpc.inCollection) 
				continue;
			for (Contact c: bpc.contactList) 
				c.displayContactLocation(drawable, new Color3f(0,0,1), size); //blue inside collection
		}
	}

	public void displayInternalHistory(GLAutoDrawable drawable) {
		for (Contact c: internalContacts) {
			c.drawInternalContactHistory(drawable);;
		}
	}
	
	/**
	 * displays the Body Collection in different color.
	 * @param drawable
	 */
	public void displayCollection( GLAutoDrawable drawable, Color3f color) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glEnable(GL2.GL_BLEND); 
		gl.glBlendFunc(GL2.GL_ZERO, GL2.GL_CONSTANT_COLOR);
		gl.glBlendEquation(GL2.GL_FUNC_ADD);
		gl.glBlendColor(color.x, color.y, color.z, Block.alpha);
		
		for (RigidBody b: collectionBodies) {
			//if (myListID == -1)
			//	b.myListID = -1;
			b.display(drawable, color);
		}
		
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );	
    }

	/**
	 * displays the Body Collection as lines between the center of masses of each rigid body to the other. 
	 * Uses a string arrayList to check if a connection has already been drawn.
	 * @param drawable
	 */
	public void displayContactGraph( GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();

		// draw a line between the two bodies but only if they're both not pinned
		Point2d p1 = new Point2d();
		Point2d p2 = new Point2d();
		for (BodyPairContact bpc: bodyPairContactList) {
			if(bpc.inCollection) {
				gl.glLineWidth(5);
				gl.glColor4f(0.f, 0.f, 0.f, 1.0f);
				gl.glBegin( GL.GL_LINES );
				p1.set(bpc.body1.x);
				p2.set(bpc.body2.x);
				gl.glVertex2d(p1.x, p1.y);
				gl.glVertex2d(p2.x, p2.y);
				gl.glEnd(); 
			}
		}
	}
	
	/**
	 * Displays deltaV if body is a collection. If it is a collection, the external 
	 * collections deltaV's (computed by external multi-iteration PGS) will be drawn 
	 * in a light shade of cyan. Then, all it's sub-bodies' single iteration deltaV's
	 * will also be drawn in a darker shade of cyan.
	 */
	@Override
	public void displayDeltaV(GLAutoDrawable drawable, int size, Color4f color) {
		super.displayDeltaV(drawable, 10, color);
		
		Color4f c = new Color4f(color.x/2, color.y/2, color.z/2, color.w/2);
		for (RigidBody b : collectionBodies) {
			b.displayDeltaV(drawable, size, c);
		}
	}
	
	/**
	 * displays cycles (from merge condition)
	 * @param drawable
	 */
	public void displayCycles( GLAutoDrawable drawable, int size) {
		
		for(BodyPairContact bpc : bodyPairContactList) {
			if (bpc.inCycle) {
				if(bpc.contactList.isEmpty())
					System.err.println("[displayCycles] The list of contact is empty. This should not happen. Probably due to an unwanted merge (concave?).");
				else
					bpc.contactList.get(0).displayContactLocation(drawable, bpc.cycleColor, size);
			}
		}
	}
	
	@Override
	public void displayBB(GLAutoDrawable drawable) {
		super.displayBB(drawable);
		for (RigidBody body : collectionBodies) {
			body.displayBB(drawable);
		}
	}
}
