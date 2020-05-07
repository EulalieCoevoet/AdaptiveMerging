package mergingBodies2D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;
import no.uib.cipr.matrix.DenseVector;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Implementation of a contact constraint.
 * @author kry
 */
public class Contact {

	/** Next available contact index, used for determining which rows of the Jacobian a contact uses */
	static public int nextContactIndex = 0;

	/** Index of this contact, determines its rows in the Jacobian */
	int index;
	
	// Used for merge/unmerge condition
	public ContactState state = ContactState.CLEAR;
	public enum ContactState {BROKEN, ONEDGE, CLEAR};
	
	/** First RigidBody in contact */
	public RigidBody body1;

	/** Second RigidBody in contact */
	public RigidBody body2;

	//The two blocks within the bodies that caused the collision
	Block block1;
	Block block2;

	/** Contact normal in world coordinates. GOES FROM BODY1 to BODY2*/
	Vector2d normal = new Vector2d();

	/** Contact normal in Body1 coordinates. GOES FROM BODY1 to BODY2*/
	Vector2d normalB1 = new Vector2d();

	/** Contact normal in Body2 coordinates. GOES FROM BODY1 to BODY2*/
	Vector2d normalB2 = new Vector2d();

	/** Contact force being applied by this contact on body1*/
	Vector2d forceB1 = new Vector2d();

	/** Contact torque being applied by this contact on body1*/
	double torqueB1 = 0;

	/** Contact force being applied by this contact on body2*/
	Vector2d forceB2 = new Vector2d();

	/** Contact torque being applied by this contact on body2*/
	double torqueB2 = 0;

	/** Position of contact point in world coordinates */
	Point2d contactW = new Point2d();
	
	/** Position of contact point in Body1 coordinates */
	Point2d contactB1  = new Point2d();
	
	/** Position of contact point in Body2 coordinates */
	Point2d contactB2  = new Point2d();
	
	/** Jacobian for normal direction */ 
	// TODO: change that one matrix for all... actually, for better performance consider the following:
	// Make these all variables like jna0 jna1 jna2 jnb0 jnb1 jna2 etc as the setters and getters
	// on dense vectors all have range checking with extra overhead (eventually optimized by JIT).
	// do the same for lambda.
	DenseVector jn = new DenseVector(6);
	
	/** Jacobian for normal direction collection frame */ 
	DenseVector jnc = new DenseVector(6);
	
	/** Jacobian for tangential direction */ 
	DenseVector jt = new DenseVector(6);
	
	/** Jacobian for tangential direction collection frame */ 
	DenseVector jtc = new DenseVector(6);

	/** Lagrange multiplier for contact, Vector2d(normal, tangent) */
	Vector2d lambda = new Vector2d();
	
	// Values used in PGS resolution
	double bn = 0; /** b value for normal component */
	double bt = 0; /** b value for tangent component */
	double diin = 0; /** diagonal entry for normal component */
	double diit = 0; /** diagonal entry for tangent component */
	
	BVNode bvn1;
	BVNode bvn2;

	/** vector points from body 2 to body 1, magnitude is the amount of overlap.*/
	double constraintViolation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
	double prevConstraintViolation;
	
	boolean newThisTimeStep;
	
	/**
	 *  Creates a new contact, and assigns it an index
	 * @param body1
	 * @param body2
	 * @param contactW
	 * @param normal
	 * @param b1
	 * @param b2
	 * @param interpenetration (should be negative)
	 */
	public Contact( RigidBody body1, RigidBody body2, Point2d contactW, Vector2d normal, Block b1, Block b2, double interpenetration) {
				
		if(body1 instanceof RigidCollection || body2 instanceof RigidCollection)
			System.err.println("[Contact] Contact should never involve a collection.");
		
		this.body1 = body1;
		this.body2 = body2;
		this.contactW.set( contactW );
		this.normal.set( normal );   
		block1 = b1;
		block2 = b2;
		constraintViolation =  interpenetration;
		index = nextContactIndex++;        

		// TODO: the body frame contacts are NEVER used... only transformed here from world to body, and then back to world later
		// Not clear that the bodies move in the mean time, yes??  Consider eliminating these conversions and the body frame variables.
		contactB1.set(contactW);
		contactB2.set(contactW);
		body1.transformW2B.transform(contactB1);
		body2.transformW2B.transform(contactB2);
		
		// set normals in body coordinates
		normalB1.set(normal);
		normalB2.set(normal);
		body1.transformW2B.transform(normalB1);
		body2.transformW2B.transform(normalB2);
		normalB2.scale(-1);
		
		computeJacobian(false);
		computeJacobian(true);
	}
	
	public Contact(Contact contact) {		
		this.body1 = contact.body1;	
		this.body2 = contact.body2;	
		this.normal.set(contact.normal);   	
		normalB1.set(contact.normalB1);	
		normalB2.set(contact.normalB2);	
		contactW.set(contact.contactW);	
		contactB1.set(contact.contactB1);	
		contactB2.set(contact.contactB2);	
		jn.set(contact.jn);	
		jt.set(contact.jt);	
		jnc.set(contact.jnc);	
		jtc.set(contact.jtc);	
		lambda.set(contact.lambda);	
		constraintViolation = contact.constraintViolation;	
		prevConstraintViolation = contact.prevConstraintViolation;	
	}
	
	/**
	 * Computes the Jacobian matrix of the contact.
	 * In case of body in a collection, use COM of parent to compute the torque component of the Jacobian.
	 */
	public void computeJacobian(boolean computeInCollection) {
				
		RigidBody b1 = (body1.isInCollection() && !computeInCollection )? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection )? body2.parent: body2;

		body1.transformB2W.transform(contactB1, contactW);
		body1.transformB2W.transform(normalB1, normal);
		
		Point2d radiusBody1 = new Point2d();
		Point2d radiusBody2 = new Point2d();

		radiusBody1.sub(contactW, b1.x);
		radiusBody2.sub(contactW, b2.x);

		Vector2d r1 = new Vector2d(-radiusBody1.y, radiusBody1.x);
		Vector2d r2 = new Vector2d(-radiusBody2.y, radiusBody2.x);
		Vector2d tangent = new Vector2d(-normal.y, normal.x);
		
		DenseVector jn;
		DenseVector jt;

		jn = (b1 instanceof RigidCollection)? jnc: this.jn;
		jt = (b1 instanceof RigidCollection)? jtc: this.jt;
		jn.set(0, -normal.x); 
		jn.set(1, -normal.y);
		jn.set(2, -r1.dot(normal));
		jt.set(0, -tangent.x);
		jt.set(1, -tangent.y);
		jt.set(2, -r1.dot(tangent));

		jn = (b2 instanceof RigidCollection)? jnc: this.jn;
		jt = (b2 instanceof RigidCollection)? jtc: this.jt;
		jn.set(3, normal.x);
		jn.set(4, normal.y);
		jn.set(5, r2.dot(normal)); 
		jt.set(3, tangent.x);
		jt.set(4, tangent.y);
		jt.set(5, r2.dot(tangent));
	}
	
	/**
	 * Stores contact forces and torques for visualization purposes
	 * @param dt
	 */
	public void computeForces(boolean computeInCollection, double dt) {

		DenseVector jn;
		DenseVector jt;

		jn = (body1.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		jt = (body1.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		forceB1.set(lambda.x*jn.get(0) + lambda.y*jt.get(0),lambda.x*jn.get(1) + lambda.y*jt.get(1));
		torqueB1 = lambda.x*jn.get(2) + lambda.y*jt.get(2);
		forceB1.scale(1/dt);
		torqueB1/=dt;

		jn = (body2.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		jt = (body2.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		forceB2.set(lambda.x*jn.get(3) + lambda.y*jt.get(3),lambda.x*jn.get(4) + lambda.y*jt.get(4));
		torqueB2 = lambda.x*jn.get(5) + lambda.y*jt.get(5);
		forceB2.scale(1/dt);
		torqueB2/=dt;
	}
	
	/**
	 * Update state of the contact: either BROKE, SLIDING or CLEAR
	 * @param mu
	 */
	protected void updateContactState(double mu) {
		if (Math.abs(lambda.x) <= 1e-14) // (math.abs is for magnet)
			state = ContactState.BROKEN;	
		else if (Math.abs(lambda.y) == lambda.x*mu) 
			state = ContactState.ONEDGE;
		else
			state = ContactState.CLEAR;
	}
	
	/**
	 * 
	 * @param dt
	 * @param feedbackStiffness
	 * @param computeInCollection
	 */
	public void computeB(double dt, double feedbackStiffness,  boolean computeInCollection) {
		
		RigidBody b1 = (body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection)? body2.parent: body2;

		double m1inv = (b1.temporarilyPinned)? 0: b1.minv; 
		double m2inv = (b2.temporarilyPinned)? 0: b2.minv;
		double j1inv = (b1.temporarilyPinned)? 0: b1.jinv;
		double j2inv = (b2.temporarilyPinned)? 0: b2.jinv;
		
		// add the Bounce vector to the u's over here, but don't need to do that just yet
		double restitution = 0.;
		if (!computeInCollection) {
			restitution=(body1.restitution+body2.restitution)/2.;
		}
		
		DenseVector j;
		
		j = (b1 instanceof RigidCollection)? this.jnc: this.jn;
		double u1xn =     (b1.v.x + b1.force.x * m1inv * dt) * j.get(0);
		double u1yn =     (b1.v.y + b1.force.y * m1inv * dt) * j.get(1);
		double u1omegan = (b1.omega + b1.torque * j1inv * dt) * j.get(2);
		double bBounce = restitution*(b1.v.x*j.get(0) + b1.v.y*j.get(1) + b1.omega*j.get(2));

		j = (b1 instanceof RigidCollection)? this.jtc: this.jt;
		double u1xt =     (b1.v.x + b1.force.x * m1inv * dt) * j.get(0);
		double u1yt =     (b1.v.y + b1.force.y * m1inv * dt) * j.get(1);
		double u1omegat = (b1.omega + b1.torque * j1inv * dt) * j.get(2);
			
		j = (b2 instanceof RigidCollection)? this.jnc: this.jn;
		double u2xn =     (b2.v.x + b2.force.x * m2inv * dt) * j.get(3);
		double u2yn =     (b2.v.y + b2.force.y * m2inv * dt) * j.get(4);
		double u2omegan = (b2.omega + b2.torque * j2inv * dt) * j.get(5);
		bBounce += restitution*(b2.v.x*j.get(3) + b2.v.y*j.get(4) + b2.omega*j.get(5));

		j = (b2 instanceof RigidCollection)? this.jtc: this.jt; 
		double u2xt =     (b2.v.x + b2.force.x * m2inv * dt) * j.get(3);
		double u2yt =     (b2.v.y + b2.force.y * m2inv * dt) * j.get(4);
		double u2omegat = (b2.omega + b2.torque * j2inv * dt) * j.get(5);

		// calculate Baumgarte Feedback (overlap of the two bodies)
		double baumgarteFeedback = feedbackStiffness*constraintViolation;
		
		// putting b together.
		bn = u1xn + u2xn + u1yn + u2yn + u1omegan + u2omegan + bBounce + baumgarteFeedback;
		bt = u1xt + u2xt + u1yt + u2yt + u1omegat + u2omegat;
	}
	
	/**
	 * Compute Dii values and store in contact
	 * @param computeInCollection
	 * @param compliance 
	 */
	public void computeJMinvJtDiagonal(boolean computeInCollection) {
		
		RigidBody b1 = (body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection)? body2.parent: body2;
		
		double m1inv = (b1.temporarilyPinned)? 0: b1.minv; 
		double m2inv = (b2.temporarilyPinned)? 0: b2.minv;
		double j1inv = (b1.temporarilyPinned)? 0: b1.jinv;
		double j2inv = (b2.temporarilyPinned)? 0: b2.jinv;
		
		DenseVector jn;
		DenseVector jt;
		
		diin = 0.;
		diit = 0.;
		
		jn = (b1 instanceof RigidCollection)? this.jnc: this.jn;
		jt = (b1 instanceof RigidCollection)? this.jtc: this.jt;
		diin += jn.get(0) * m1inv * jn.get(0);
		diin += jn.get(1) * m1inv * jn.get(1);
		diin += jn.get(2) * j1inv * jn.get(2);
		diit += jt.get(0) * m1inv * jt.get(0);
		diit += jt.get(1) * m1inv * jt.get(1);
		diit += jt.get(2) * j1inv * jt.get(2);
		
		jn = (b2 instanceof RigidCollection)? this.jnc: this.jn;
		jt = (b2 instanceof RigidCollection)? this.jtc: this.jt;
		diin += jn.get(3) * m2inv * jn.get(3);
		diin += jn.get(4) * m2inv * jn.get(4);
		diin += jn.get(5) * j2inv * jn.get(5);
		diit += jt.get(3) * m2inv * jt.get(3);
		diit += jt.get(4) * m2inv * jt.get(4);
		diit += jt.get(5) * j2inv * jt.get(5);
	}
	
	/**
	 * Returns Jdv values for normal component.
	 * @param computeInCollection
	 */
	public double getJdvn(boolean computeInCollection) {
		
		DenseVector dv1 = (body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
		DenseVector dv2 = (body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
		DenseVector jn;
		
		double Jdvn = 0;  		
		
		jn = (body1.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		Jdvn += jn.get(0) * dv1.get(0);
		Jdvn += jn.get(1) * dv1.get(1);
		Jdvn += jn.get(2) * dv1.get(2);
		
		jn = (body2.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		Jdvn += jn.get(3) * dv2.get(0);
		Jdvn += jn.get(4) * dv2.get(1);
		Jdvn += jn.get(5) * dv2.get(2);
		
		return Jdvn;
	}
	
	/**
	 * Returns Jdv values for tangent component.
	 * @param computeInCollection
	 */
	public double getJdvt(boolean computeInCollection) {
		
		DenseVector dv1 = (body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
		DenseVector dv2 = (body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
		
		DenseVector jt;
		
		// normal component
		double Jdvt = 0;  		

		jt = (body1.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		Jdvt += jt.get(0) * dv1.get(0);
		Jdvt += jt.get(1) * dv1.get(1);
		Jdvt += jt.get(2) * dv1.get(2);

		jt = (body2.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		Jdvt += jt.get(3) * dv2.get(0);
		Jdvt += jt.get(4) * dv2.get(1);
		Jdvt += jt.get(5) * dv2.get(2);
		
		return Jdvt;
	}
	
	/**
	 * Returns if given body is part of the contact
	 * @param body
	 * @return
	 */
	public boolean withBody(RigidBody body) {
		return (body1==body || body2==body);
	}
	
	/**
	 * Getter for lambda
	 * @return metric
	 */
	public Vector2d getLambda() {
		Vector2d l = new Vector2d(lambda);
		return l;
	}

	/**
	 * Draws the connections between bodies to visualize 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void displayContactGraph( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		// draw a line between the two bodies but only if they're both not pinned
		if ( !body1.pinned && ! body2.pinned ) {
			gl.glLineWidth(2);
			gl.glColor4f(0,.3f,0, 0.5f);
			gl.glBegin( GL.GL_LINES );

			gl.glVertex2d(body1.x.x, body1.x.y);
			gl.glVertex2d(body2.x.x, body2.x.y);
			gl.glEnd();
		}
	}

	/**
	 * Draws the contact force
	 * @param drawable
	 */
	public void displayContactForce( GLAutoDrawable drawable, Color3f color ) {
		GL2 gl = drawable.getGL().getGL2();

		gl.glLineWidth(2);
		if (state == ContactState.ONEDGE)
			gl.glColor4f(0, 0, 0, 1);
		else
			gl.glColor4f(color.x, color.y, color.z, 1);
		gl.glBegin( GL.GL_LINES );

		double scale = forceVizScale.getValue();
		gl.glVertex2d(contactW.x + scale*forceB1.x, contactW.y+scale*forceB1.y);
		gl.glVertex2d(contactW.x + scale*forceB2.x, contactW.y+scale*forceB2.y);
		
		gl.glEnd();
	}
	
	/**
	 * Draws the contact points
	 * @param drawable
	 */
	public void displayContactLocation( GLAutoDrawable drawable, Color3f color, int size) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glPointSize(size);
		gl.glColor4f(color.x, color.y, color.z, 1);
		gl.glBegin( GL.GL_POINTS );
		gl.glVertex2d(contactW.x, contactW.y);
		gl.glEnd();
	}

	static DoubleParameter forceVizScale = new DoubleParameter("force viz scale", 0.05, 0.0001, 1);

}
