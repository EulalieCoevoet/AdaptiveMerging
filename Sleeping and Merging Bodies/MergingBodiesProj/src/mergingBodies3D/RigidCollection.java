package mergingBodies3D;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
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
	protected HashSet<Contact> internalContacts = new HashSet<Contact>();
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
		thetaT.set( body.thetaT );
		
		massLinear = body.massLinear;
		minv = body.minv;
		
		massAngular.set( body.massAngular );
		massAngular0.set( body.massAngular0 );
		jinv.set( body.jinv );
		jinv0.set( body.jinv0 );
		
		boundingBoxB = new ArrayList<Point3d>(body.boundingBoxB);
		
		updateTransformations();
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
	}

	/**
	 * Adds given list of bodies the collection
	 */
	public void addBodies(Collection<RigidBody> bodies) {
		for (RigidBody body : bodies) {
			addBodyInternalMethod(body);
			updateCollectionState(body);
		}
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
	}
	
	/**
	 * Adds a body to the collection (internal method, for factoring purposes).
	 * 
	 * @param body body to add
	 */
	private void addBodyInternalMethod( RigidBody body ) {
		body.parent = this;
		bodies.add(body);

		if (bodies.size()<2)
			return; 
		
		Point3d com = new Point3d();
		double totalMassInv = 0;

		updateTheta(body); 
		updateBB(body); // set BB temporarily in world coordinates
		
		com.set(x);
		com.scale(massLinear);
		com.scaleAdd(body.massLinear, body.x, com);
		totalMassInv = 1./(body.massLinear + massLinear);
		com.scale( totalMassInv );
		
		if ( pinned ) { 
			v.set(0,0,0);
			omega.set(0,0,0);
			minv = 0;
			jinv0.setZero();
			jinv.setZero();
			massAngular.setZero(); // actually infinity.. but won't be used??
			massAngular0.setZero();
		} else {
			updateVelocitiesFrom(body, com, totalMassInv);
			updateInertia(body, com);
			massLinear += body.massLinear;
			minv = totalMassInv;
		}
		
		x.set(com);
		
		updateTransformations();
		for (Point3d point : boundingBoxB) // put back BB in collection coordinates
			transformW2B.transform(point);
		
		updateBodyTransformations(body);
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
	private void updateVelocitiesFrom(RigidBody body, Point3d com, double totalMassInv) {

		Vector3d r = new Vector3d();
		Vector3d wxr = new Vector3d();
		Vector3d tmp1 = new Vector3d();
		Vector3d tmp2 = new Vector3d();

		r.sub( com, body.x );
		wxr.cross( body.omega, r );
		tmp1.add( wxr, body.v );
		tmp1.scale( body.massLinear );
		
		r.sub( com, x );
		wxr.cross( omega, r );
		tmp2.add( wxr, v );
		tmp2.scale( massLinear );
		
		tmp1.add( tmp2 );
		tmp1.scale( totalMassInv );
		
		v.set(tmp1); 

		omega.scale( massLinear );
		omega.scaleAdd( body.massLinear, body.omega, omega );
		omega.scale( totalMassInv );
	}
	
	/**
	 * Compute theta of the collection from covariance's eigen vectors
	 */
	private void updateTheta(RigidBody newBody) {

		if (newBody instanceof PlaneRigidBody) // TODO: eulalie: what if we copied a PlaneRigidBody...
			return;

		int N = 16;
		Point3d meanPos = new Point3d();
		Point3d p = new Point3d();
		
		for (int i=0; i<2; i++) {
			RigidBody body = (i==0)? this: newBody;
			for (Point3d point : body.boundingBoxB) {
				p.set(point);
				body.transformB2W.transform(p);
				meanPos.add(p);
			}
		}
		meanPos.scale(1.f/N);

		Vector3d v = new Vector3d();
		MyMatrix3f tmp = new MyMatrix3f();
		MyMatrix3f covariance = new MyMatrix3f();
		
		for (int i=0; i<2; i++) {
			RigidBody body = (i==0)? this: newBody;
			for (Point3d point : body.boundingBoxB) {
				p.set(point);
				body.transformB2W.transform(p);
				v.sub(p, meanPos);
				tmp.m00 = (float)(v.x*v.x); tmp.m01 = (float)(v.x*v.y); tmp.m02 = (float)(v.x*v.z);
				tmp.m10 = (float)(v.y*v.x); tmp.m11 = (float)(v.y*v.y); tmp.m12 = (float)(v.y*v.z);
				tmp.m20 = (float)(v.z*v.x); tmp.m21 = (float)(v.z*v.y); tmp.m22 = (float)(v.z*v.z);
				covariance.add(tmp);
			}
		}
		covariance.mul(1.f/N);
		covariance.getEigen(tmp);
		tmp.normalize();
		
		theta.set(tmp);
		thetaT.transpose(theta);
	}

	/**
	 * Update collection pinned condition
	 * @param body
	 */
	private void updateCollectionState(RigidBody body) {
		pinned = (pinned || body.pinned);

		isSleeping = (isSleeping || body.isSleeping);
		body.isSleeping = false;
	}
	
	private void updateInertia(RigidBody newBody, Point3d com) {
				
	    Matrix3d massAngular = new Matrix3d();
		for (int i=0; i<2; i++) {
			RigidBody body = (i==0)? this: newBody;
					
			massAngular.add( body.massAngular );
			// translate inertia tensor to center of mass
			// should certainly have a b.x squared type term for the mass being at a distance...
			//			I [p]    J  0    I   0 
			//			0  I    0 mI    [p] I
			//
			//			I [p]   J   0
			//			0  I   m[p] 0
			//
			//			Thus.. J + mI [p][p] in the upper left...
			// recall lemma 2.3: [a] = a a^T - ||a||^2 I
			double x = body.x.x - com.x; 
			double y = body.x.y - com.y;
			double z = body.x.z - com.z;
			double x2 = x*x;
			double y2 = y*y;
			double z2 = z*z;
			Matrix3d op = new Matrix3d();
			op.m00 = y2+z2;  op.m01 = -x*y;  op.m02 = -x*z;
			op.m10 = -y*x;   op.m11 = x2+z2; op.m12 = -y*z;
			op.m20 = -z*x;   op.m21 = -z*y;  op.m22 = x2+y2;
			op.mul( body.massLinear );
			massAngular.add( op );	
		}
		this.massAngular.set(massAngular);

			
		// Let's get massAngular0
		jinv.invert( massAngular );	 // is this avoidable by construction above?  :/
		
		//	    J = R J0 R^T
		//	so J0 = R^T J R
		// and...      Jinv = R Jinv R^T
		massAngular0.mul( thetaT, massAngular );
		massAngular0.mul( theta );
		jinv0.mul( thetaT, jinv);
		jinv0.mul( theta );		
	}

	/**
	 * For each body in collection, determines the transformations to go from body
	 * to collection But also, make each body's x and theta in collection, relative
	 * to this x and theta
	 */
	private void updateBodyTransformations(RigidBody body) {
		body.transformB2C.set(body.transformB2W);
		body.transformB2C.leftMult(transformW2B);
		body.transformC2B.set(body.transformB2C);
		body.transformC2B.invert();
	}

	private void updateBB(RigidBody newBody) {
		
		if (newBody instanceof PlaneRigidBody)
			return;
		
		Point3d bbmaxB = new Point3d(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
		Point3d bbminB = new Point3d( Double.MAX_VALUE,  Double.MAX_VALUE,  Double.MAX_VALUE);

		Point3d p = new Point3d();
		for (int i=0; i<2; i++) {
			RigidBody body = (i==0)? this: newBody;
			for (Point3d point : body.boundingBoxB) {
				p.set(point);
				body.transformB2W.transform(p);
				bbmaxB.x = Math.max(bbmaxB.x, p.x);
				bbmaxB.y = Math.max(bbmaxB.y, p.y);
				bbmaxB.z = Math.max(bbmaxB.z, p.z);
				bbminB.x = Math.min(bbminB.x, p.x);
				bbminB.y = Math.min(bbminB.y, p.y);
				bbminB.z = Math.min(bbminB.z, p.z);
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
	 * Migrates the contacts of the BPC to the internal contacts. 
	 * @param bpc
	 */
	public void addToInternalContact(BodyPairContact bpc) {
		tmpContacts.clear();
		for (Contact contact : bpc.contactList) {
			// need to make new to not mess with the memory pools 
			Contact c = new Contact(contact);
			tmpContacts.add( c );
			internalContacts.add( c );
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

		if ( pinned || isSleeping )
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
		if ( pinned ) { 
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

		if (pinned)
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
    private static final float[] colGraph = new float[] { 0, 0.2f, 0, 0.25f };
	public void displayContactGraph(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();

		gl.glLineWidth(5);
		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, colGraph, 0 );
		gl.glBegin(GL.GL_LINES);
		for (BodyPairContact bpc : bodyPairContacts) {
			if (bpc.inCollection) {
				gl.glVertex3d(bpc.body1.x.x, bpc.body1.x.y, bpc.body1.x.z);
				gl.glVertex3d(bpc.body2.x.x, bpc.body2.x.y, bpc.body2.x.z);
			}
		}
		gl.glEnd();
	}

	/**
	 * displays cycles (from merge condition)
	 * 
	 * @param drawable
	 */
	public void displayCycles(GLAutoDrawable drawable, int size) {

		for (BodyPairContact bpc : bodyPairContacts) {
			if (bpc.inCycle) {
				if (bpc.contactList.isEmpty())
					System.err.println("[displayCycles] The list of contact is empty. This should not happen. Probably due to an unwanted merge (concave?).");
				else {
					for (Contact contact : bpc.contactList) 
						contact.display(drawable, true);
				}
			}
		}
	}

	@Override
	public void displayBB(GLAutoDrawable drawable) {
		super.displayBB(drawable);
		/*for (RigidBody body : bodies) {
			body.displayBB(drawable);
		}*/
	}
}
