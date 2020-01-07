package mergingBodies3D;

import java.util.ArrayList;
import java.util.LinkedList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mergingBodies3D.Merging.MergeParameters;

/**
 * Adapted from 2D verions... 
 * 
 * TODO: note for efficiency, should consider incremental updates to the inertia...
 * that is at least if bodies are being added one at a time to a collection
 * 
 * TODO: Also... consider pooling rigid collections as they probably get create and destroyed often??  
 * Perhaps not as bad as contacts... but just the same!!  The internal lists (contacts and bodies) will
 * benefit from not being re-allocated and regrown too.
 * 
 * @author kry
 */
public class RigidCollection extends RigidBody {

	/** List of RigidBody of the collection */
	protected ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
	
	/**
	 * List of Contact in the collection: Contact between RigidBody of the collection
	 */
	protected ArrayList<Contact> internalContacts = new ArrayList<Contact>();
	protected ArrayList<Contact> tmpContacts = new ArrayList<Contact>();
	
	MotionMetricProcessor motionMetricProcessor = new MotionMetricProcessor();
	
	static MergeParameters mergeParams;

	Color color = new Color();
	
	/**
	 * Creates a RigidCollection from two RigidBody.
	 * 
	 * @param body1
	 * @param body2
	 */
	public RigidCollection(RigidBody body1, RigidBody body2) {

		// These bodies being added to the collection, with the collection being new,
		// their state w.r.t the collection frame is unchanged as C2W and W2C are
		// Identity

		color.setRandomColor();
		col = new float[] { color.x, color.y, color.z, 1 };
	
		copyFrom(body1);

		addBodyInternalMethod(body1);  // this will update velocity, but it doesn't matter (body velocity averaged with itself)
		updateCollectionState(body1);
		addBodyInternalMethod(body2);
		updateCollectionState(body2);

		updateCollection();
	}

	/**
	 * Adds a body to the collection
	 * This generally follows a 3 step process
	 * <p><ul>
	 * <li> 1) Adds the body to the list, and updates the velocity to be a
	 *    weighted average of the collection and the new body.  This velocity
	 *    is updated to be in the collection's COM frame, even though other 
	 *    methods to update the COM are not yet called.  Because the velocity
	 *    is stored in a world aligned frame, this has little impact on the rest
	 *    of the steps.
	 * <li> 2) Mass COM and Inertia are updated.  The inertia will be with
	 *    respect to the current rotation of the collection, so some care necessary
	 *    to initalize massAnular0 and jinv0 too.
	 * <li> 3) Rotation is optimized for the bounding volumes of the different bodies in
	 *     the collection. (currently not implemented)  Because this changes the 
	 *     rest rotation of the collection, it will influence the rest inertia.
	 * <li> 4) Transformations are updated (jinv too, perhaps wasteful), and likewise
	 *     the body to collection transforms 
	 * </p></ul>
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
	 * 
	 * @param collection collection to add
	 */
	public void addCollection(RigidCollection collection) {
		for (RigidBody body : collection.bodies)
			addBodyInternalMethod(body);

		updateCollectionState(collection);
		updateCollection();
	}

	/**
	 * Adds a body to the collection (internal method, for factoring purposes).
	 * 
	 * @param body body to add
	 */
	private void addBodyInternalMethod( RigidBody body ) {
		body.parent = this;
		bodies.add(body);

		updateVelocitiesFrom(body);
	}

	/**
	 * Update's the collection's velocity given a newly added body
	 * The velocities should match... but we'll do a mass weighted
	 * average in the new COM frame to make sure that things work out.
	 * Note: this might not do what you expect if either body is pinned!!
	 * 
	 * CAREFUL this will set the velocity at the new COM position!!
	 * 
	 * @param body
	 */
	private void updateVelocitiesFrom(RigidBody body) {
		Point3d massCom1 = new Point3d();
		Point3d massCom2 = new Point3d();
		massCom1.scale(body.massLinear, body.x);
		massCom2.scale(massLinear, x);
		Point3d xCom = new Point3d();
		xCom.add(massCom1, massCom2);
		double oneOverTotalMass = 1. / (body.massLinear + massLinear);
		xCom.scale( oneOverTotalMass );

		Vector3d r = new Vector3d();
		Vector3d wxr = new Vector3d();
		Vector3d tmp1 = new Vector3d();
		Vector3d tmp2 = new Vector3d();

		r.sub(xCom, body.x);
		wxr.cross( body.omega, r );
		tmp1.add( wxr, v );
		tmp1.scale( body.massLinear );
		
		r.sub( xCom, x );
		wxr.cross( omega, r );
		tmp2.add( wxr, v );
		tmp2.scale( massLinear );
		
		tmp1.add( tmp2 );
		tmp1.scale( oneOverTotalMass );
		
		v.set(tmp1); 

		omega.scale( massLinear );
		omega.scaleAdd( body.massLinear, body.omega, omega );
		omega.scale( oneOverTotalMass );
	}

	/**
	 * Zero working variables for accumulating forces during a time step
	 */
	public void clearBodies() {
		for (RigidBody body : bodies) {
			applyVelocitiesTo(body);
			body.clear();
		}
	}

	/**
	 * Copy velocities of given body
	 * 
	 * @param body
	 */
	private void copyFrom(RigidBody body) {
		v.set(body.v);
		omega.set( body.omega );
		x.set(body.x);
		theta.set( body.theta );
		massLinear = body.massLinear;
		massAngular.set( body.massAngular );
		massAngular0.set( body.massAngular0 );
		jinv.set( body.jinv );
		jinv0.set( body.jinv0 );
		boundingBoxB = new ArrayList<Point3d>(body.boundingBoxB);
		updateTransformations();
	}

	/**
	 * Update collection pinned condition
	 * 
	 * @param body
	 */
	private void updateCollectionState(RigidBody body) {
//		temporarilyPinned = (temporarilyPinned || body.temporarilyPinned);
//		body.temporarilyPinned = false;
//		steps = Math.max(body.steps, steps);

		pinned = (pinned || body.pinned);

		isSleeping = (isSleeping || body.isSleeping);
		body.isSleeping = false;
	}

	/**
	 * Computes transforms, COM, mass, inertia, spring.
	 * NOTE: velocity should already be updated into the COM frame at this point!
	 */
	private void updateCollection() {

		if ( pinned ) { //|| temporarilyPinned) {
			v.set(0,0,0);
			omega.set(0,0,0);
			minv = 0;
			jinv0.setZero();
			jinv.setZero();
			massAngular.setZero(); // actually infinity.. but won't be used??
			massAngular0.setZero();
		} else {
			updateMassCOMInertia();
		}
		
		updateTheta(); // currently does nothing, but can help optimize the BB
		updateTransformations();
		updateBodiesTransformations();
		updateBB();
	}

	/** 
	 * TODO: SPEED: this could be a fast incremental update rather than recomputing for all bodies 
	 */
	private void updateMassCOMInertia() {
		double massLinear = 0;
		Matrix3d massAngular = new Matrix3d();

		Point3d com = new Point3d();
		for ( RigidBody b : bodies ) {
			massLinear += b.massLinear;		
			com.scaleAdd( b.massLinear, b.x, com );
		}
		com.scale( 1./massLinear );
		x.set(com);
		this.massLinear = massLinear;
		this.minv = 1./massLinear;
		
		for ( RigidBody b : bodies ) {
			massAngular.add( b.massAngular );
			// should certainly have a b.x squared type term for the mass being at a distance...
			//			I [p]    J  0    I   0 
			//			0  I    0 mI    [p] I
			//
			//			I [p]   J   0
			//			0  I   m[p] 0
			//
			//			Thus.. J + mI [p][p] in the upper left...
			// recall lemma 2.3: [a] = a a^T - ||a||^2 I
			double x = b.x.x;
			double y = b.x.y;
			double z = b.x.z;			
			double x2 = x*x;
			double y2 = y*y;
			double z2 = z*z;
			Matrix3d op = new Matrix3d();
			op.m00 = y2+z2; op.m01 = x*y; op.m02 = x*z;
			op.m10 = y*x; op.m11 = x2+z2; op.m12 = y*z;
			op.m20 = z*x; op.m21 = z*y; op.m22 = x2 + y2;
			op.mul( b.massLinear );
			massAngular.add( op );			
		}
		// Let's get massAngular0
		this.massAngular.set( massAngular );
		this.jinv.invert( massAngular );	 // is this avoidable by construction above?  :/
		//	    J = R J0 R^T
		//	so J0 = R^T J R
		// and...      Jinv = R Jinv R^T
		this.massAngular0.mul( thetaT, massAngular );
		this.massAngular0.mul( theta );
		this.jinv0.mul( thetaT, jinv);
		this.jinv0.mul( theta );		
	}
	

	/**
	 * Compute theta of the collection from convex hull informations
	 */
	private void updateTheta() {

		int N = 0;
		Point3d meanPos = new Point3d();

		for (RigidBody body : bodies) {
			if (body instanceof PlaneRigidBody)
				continue;

			for (Point3d point : body.boundingBoxB) {
				Point3d p = new Point3d(point);
				transformB2C.transform(p);
				meanPos.add(p);
				N++;
			}
		}
		meanPos.scale(1.0 / N);

		Vector3d v = new Vector3d();
		//MyMatrix3f covariance = new MyMatrix3f();
		for (RigidBody body : bodies) {
			if (body instanceof PlaneRigidBody) continue;
			for (Point3d point : body.boundingBoxB) {
				Point3d p = new Point3d(point);
				transformB2C.transform(p);
				v.sub(p, meanPos);
				// TODO: UNFINISHED: Need to do our own rank 1 update :(
				
				//covariance.rank1(1.0 / N, v);
				
			}
		}
		// Need to do our own EVD ?  :(
		
		// or use the code in MyMatrix (harvested from web)
	
		// TODO: UNFINISHED: set theta... and make sure it is a right handed coordinate system!!!

		theta.setIdentity();
		
	}

	/**
	 * For each body in collection, determines the transformations to go from body
	 * to collection But also, make each body's x and theta in collection, relative
	 * to this x and theta
	 */
	private void updateBodiesTransformations() {
		for (RigidBody body : bodies) {
			body.transformB2C.set(body.transformB2W);
			body.transformB2C.leftMult(transformW2B);
			body.transformC2B.set(body.transformB2C);
			body.transformC2B.invert();
		}
	}

	private void updateBB() {
		Point3d bbmaxB = new Point3d(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
		Point3d bbminB = new Point3d( Double.MAX_VALUE,  Double.MAX_VALUE,  Double.MAX_VALUE);
		for (RigidBody body : bodies) {
			if (body instanceof PlaneRigidBody)
				continue;

			for (Point3d point : body.boundingBoxB) {
				body.transformB2C.transform(point);
				bbmaxB.x = Math.max(bbmaxB.x, point.x);
				bbmaxB.y = Math.max(bbmaxB.y, point.y);
				bbmaxB.z = Math.max(bbmaxB.z, point.z);
				bbminB.x = Math.min(bbminB.x, point.x);
				bbminB.y = Math.min(bbminB.y, point.y);
				bbminB.z = Math.min(bbminB.z, point.z);
				body.transformC2B.transform(point);
			}
		}
		boundingBoxB.clear();
		boundingBoxB.add(bbmaxB);
		boundingBoxB.add(new Point3d(bbmaxB.x, bbminB.y, bbminB.z));
		boundingBoxB.add(new Point3d(bbminB.x, bbmaxB.y, bbminB.z));
		boundingBoxB.add(new Point3d(bbminB.x, bbminB.y, bbmaxB.z));
		boundingBoxB.add(bbminB);
		boundingBoxB.add(new Point3d(bbminB.x, bbmaxB.y, bbmaxB.z));
		boundingBoxB.add(new Point3d(bbmaxB.x, bbminB.y, bbmaxB.z));
		boundingBoxB.add(new Point3d(bbmaxB.x, bbmaxB.y, bbminB.z));
	}

	/** 
	 * Migrates the provided BPC to the collection, understanding that the contacts between these two
	 * bodies will now be internal contacts.  The internal contacts and internal BPC lists are updated.
	 * @param bpc
	 */
	public void addToInternalContact(BodyPairContact bpc) {
		tmpContacts.clear();
		for (Contact contact : bpc.contactList) {
			//if (!internalContacts.contains(contact)) { // TODO: eulalie: weird weird! doesn't seemed to work. I'm pretty sure we don't need it but still I'd like to understand...
				// need to make new to not mess with the memory pools 
				Contact c = new Contact(contact);
				tmpContacts.add( c );
				internalContacts.add( c );
			//} else {
			//	System.out.println("[addToInternalContact] Contacts already there. How does this happen?");			
			//}
		}
		bpc.contactList.clear();
		bpc.contactList.addAll( tmpContacts );
	}

	/**
	 * Add bpc and external bpc to the collection BodyPairContact
	 * 
	 * @param bpc
	 */
	public void addBPCsToCollection(BodyPairContact bpc) {
		bpc.addToBodyListsParent();

		// add the external bpc to the collection bodyPairContactList
		for (int i=0; i<2; i++) {
			RigidBody body = bpc.getBody(i);
			for (BodyPairContact bpcExt : body.bodyPairContacts)
				bpcExt.addToBodyListsParent();
		}
	}
	
	@Override
	public void advanceTime( double dt ) {

		super.advanceTime( dt );

		if ( pinned || isSleeping ) // || temporarilyPinned )
			return;

		updateBodiesPositionAndTransformations();

		// Advance velocities for internal bodies
		if (!mergeParams.enableUnmergeRelativeMotionCondition.getValue())
			applyVelocitiesToBodies();
	}

	/**
	 * Updates bodies position, orientation, and transformations
	 */
	protected void updateBodiesPositionAndTransformations() {
		for (RigidBody body : bodies) {
			
			// update transformations
			body.transformB2W.set(body.transformB2C);
			body.transformB2W.leftMult(transformB2W);
			body.transformW2B.set(body.transformB2W);
			body.transformW2B.invert();

			// update position and orientation
			body.x.x = body.transformB2W.T.m03;
			body.x.y = body.transformB2W.T.m13;
			body.x.z = body.transformB2W.T.m23;
			body.transformB2W.T.getRotationScale( body.theta );
			
			if ( ! pinned ) {  // a normal update would do this... so we should do it too for a correct single cycle update.
		        jinv.mul( theta, jinv0 );
		        jinv.mul( thetaT );
		        massAngular.mul( theta, massAngular0 );
		        massAngular.mul( thetaT );
	        } 
		}
	}

	/**
	 * Updates bodies velocities
	 */
	protected void applyVelocitiesToBodies() {
		for (RigidBody body : bodies) {
			applyVelocitiesTo(body);
		}
	}

	/**
	 * Apply the linear and angular velocities to the given body
	 * 
	 * @param body
	 */
	public void applyVelocitiesTo(RigidBody body) {
		if ( pinned ) { //|| temporarilyPinned) {
			if (v.lengthSquared() > 1e-14 || omega.lengthSquared() > 1e-14)
				System.err.println("[applyVelocitiesTo] velocities of pinned body is not zero. " + omega.toString() );
		}

		Vector3d r = new Vector3d();
		Vector3d wxr = new Vector3d();
		r.sub( body.x, x );
		wxr.cross( omega, r );
		
		body.v.add( v, wxr ); // sets the value of the sum
		body.omega.set( omega );
	}

	public boolean isMovingAway(RigidBody body, MergeParameters mergeParams) {

		double metric = motionMetricProcessor.getMotionMetric(this, body);

		if (pinned ) // || temporarilyPinned)
			metric /= 2;

		return (metric > mergeParams.thresholdUnmerge.getValue());
	}

	/**
	 * Makes body ready to be used by system... converts everything to world
	 * coordinates and makes body independent of collection ... does not do anything
	 * to the collection itself.
	 */
	public void unmergeBody(RigidBody body) {
		if (!body.isInCollection(this)) {
			System.err.println("[unmergeBody] Not suppose to happen.");
			return;
		} else {
			applyVelocitiesTo(body);
			body.deltaV.setZero();
			body.parent = null;
		}
	}

	/**
	 * Go through all bodies and makes sure all the BodyContacts of each body is in
	 * the collection
	 */
	public void fillInternalBodyContacts() {
		for (RigidBody body : bodies) {
			for (BodyPairContact bpc : body.bodyPairContacts) {
				if (!bodyPairContacts.contains(bpc)) {
					bodyPairContacts.add(bpc);
					RigidBody otherBody = bpc.getOtherBody(body);
					if (body.isInSameCollection(otherBody)) {
						bpc.inCollection = true;
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
	 * We need to also add the other contacts that body has with the same collection
	 * it's being merged with. Must also add the BodyPairContact around the body
	 * that didn't reach 50 time steps but are still part of the same parents. The
	 * input parameter is the body being merged, and the body pair contact removal
	 * queue so that any BPCs identified in this call can also be later removed.
	 */
	public void addIncompleteContacts(RigidBody body, LinkedList<BodyPairContact> removalQueue) {
		for (BodyPairContact bpc : body.bodyPairContacts) {
			if (bpc.body1.isInSameCollection(bpc.body2) && !bpc.inCollection) {
				bpc.inCollection = true;
				body.parent.addToInternalContact(bpc);
				body.parent.addBPCsToCollection(bpc);
				removalQueue.add(bpc);
			}
		}
	}

	/**
	 * input parameter is a collection being merged . we must add also all the
	 * incomplete contacts this parent has with other collections.
	 */
	public void addIncompleteCollectionContacts(RigidCollection collection, LinkedList<BodyPairContact> removalQueue) {
		for (RigidBody body : collection.bodies) {
			addIncompleteContacts(body, removalQueue);
		}
	}

	public ArrayList<Contact> getInternalContacts() {
		ArrayList<Contact> contacts = new ArrayList<Contact>(internalContacts);
		return contacts;
	}

	public void displayInternalContactForces(GLAutoDrawable drawable, double dt ) {
		for (Contact c : internalContacts ) {
			c.displayContactForce( drawable, true, dt ); // blue inside collection
		}
	}

	public void displayInternalContactLocations( GLAutoDrawable drawable ) {
		for (Contact c : internalContacts ) {
			c.display( drawable, true ); // blue inside collection
		}
	}

	/**
	 * Displays the Body Collection in different color.
	 * @param drawable
	 */
	@Override
	public void display(GLAutoDrawable drawable) {
		for (RigidBody b : bodies) {
			b.display(drawable);
		}
	}

	/**
	 * displays the Body Collection as lines between the center of masses of each
	 * rigid body to the other. Uses a string arrayList to check if a connection has
	 * already been drawn.
	 * 
	 * @param drawable
	 */
	public void displayContactGraph(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();

		// draw a line between the two bodies but only if they're both not pinned
		Point3d p1 = new Point3d();
		Point3d p2 = new Point3d();
		for (BodyPairContact bpc : bodyPairContacts) {
			if (bpc.inCollection) {
				gl.glLineWidth(5);
				gl.glColor4f(0.f, 0.f, 0.f, 1.0f);
				gl.glBegin(GL.GL_LINES);
				p1.set(bpc.body1.x);
				p2.set(bpc.body2.x);
				gl.glVertex2d(p1.x, p1.y);
				gl.glVertex2d(p2.x, p2.y);
				gl.glEnd();
			}
		}
	}

//	/**
//	 * displays cycles (from merge condition)
//	 * 
//	 * @param drawable
//	 */
//	public void displayCycles(GLAutoDrawable drawable, int size) {
//
//		for (BodyPairContact bpc : bodyPairContacts) {
//			if (bpc.inCycle) {
//				if (bpc.contactList.isEmpty())
//					System.err.println(
//							"[displayCycles] The list of contact is empty. This should not happen. Probably due to an unwanted merge (concave?).");
//				else
//					bpc.contactList.get(0).display(drawable, bpc.cycleColor, size);
//			}
//		}
//	}

	@Override
	public void displayBB(GLAutoDrawable drawable) {
		super.displayBB(drawable);
		for (RigidBody body : bodies) {
			body.displayBB(drawable);
		}
	}
}
