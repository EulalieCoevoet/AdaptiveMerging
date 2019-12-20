package mergingBodies3D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import no.uib.cipr.matrix.DenseMatrix;
import no.uib.cipr.matrix.DenseVector;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Implementation of a contact constraint.
 * @author kry
 */
public class Contact {

    /** Next available contact index, used for determining which rows of the jacobian a contact uses */
    static public int nextContactIndex = 0;
    
    /** Index of this contact, determines its rows in the jacobian */
    int index;
    
    /** First RigidBody in contact */
    RigidBody body1;
    
    /** Second RigidBody in contact */
    RigidBody body2;
    
    /** Contact normal in body1 coordinates */
    Vector3d normalB = new Vector3d();
    
    /** Contact tangent1 in body1 coordinates */
    Vector3d tangent1B = new Vector3d();
    
    /** Contact tangent2 in body1 coordinates */
    Vector3d tangent2B = new Vector3d();
    
	/** Contact force being applied by this contact on body1*/
	Vector3d forceB1 = new Vector3d();

	/** Contact torque being applied by this contact on body1*/
	Vector3d torqueB1 = new Vector3d();

	/** Contact force being applied by this contact on body2*/
	Vector3d forceB2 = new Vector3d();

	/** Contact torque being applied by this contact on body2*/
	Vector3d torqueB2 = new Vector3d();
    
    /** Position of contact point in world coordinates */
    Point3d contactW = new Point3d();
    
    /** Position of contact point in body1 coordinates */
    Point3d contactB = new Point3d();
    
	/** vector points from body 2 to body 1, magnitude is the amount of overlap.*/
	double constraintViolation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
	double prevConstraintViolation;
	
	// Used for merge/unmerge condition
	public ContactState state = ContactState.CLEAR;
	public enum ContactState {BROKEN, ONEDGE, CLEAR};
	
	boolean newThisTimeStep;
	
	/** Jacobian matrix */
	DenseMatrix j = new DenseMatrix(3,12);
	
	/** Jacobian matrix, collection frame */
	DenseMatrix jc = new DenseMatrix(3,12);
	
	/** Lagrange multiplier for contact, Vector2d(normal, tangent1, tangent2) */
	DenseVector lambda = new DenseVector(3);
	
    /**
     * Creates a new contact, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW
     * @param normal
     */
    public Contact( RigidBody body1, RigidBody body2, Point3d contactW, Vector3d normal, double constraintViolation ) {
        this.body1 = body1;
        this.body2 = body2;
        this.contactW.set( contactW ); 
		this.constraintViolation =  constraintViolation;     
        index = nextContactIndex++;     
        
        contactB.set(contactW);
        body1.transformW2B.transform(contactB);
        
		normalB.set(normal);
		body1.transformW2B.transform(normalB);
		
		tangent1B = new Vector3d(normalB.z, normalB.x, normalB.y);
		tangent2B = new Vector3d();
		tangent2B.cross(normalB, tangent1B);
		tangent2B.normalize();
		tangent1B.scale(-1, normalB);
		tangent1B.cross(normalB, tangent2B);
		
        computeJacobian(true);
        computeJacobian(false);
    }
    
	public Contact(Contact contact) {		
		this.body1 = contact.body1;	
		this.body2 = contact.body2;	
		this.normalB.set(contact.normalB);   	
		normalB.set(contact.normalB);	
		contactW.set(contact.contactW);	
		contactB.set(contact.contactB);	
		j.set(contact.j);	
		jc.set(contact.jc);	
		lambda.set(contact.lambda);	
		constraintViolation = contact.constraintViolation;	
		prevConstraintViolation = contact.prevConstraintViolation;	
	}
    
    /**
	 * Computes the Jacobian matrix of the contact.
	 * In case of body in a collection, use COM of parent to compute the torque component of the Jacobian.
	 */
	public void computeJacobian(boolean computeInCollection) {
				
		RigidBody b1 = body1;//(body1.isInCollection() && !computeInCollection )? body1.parent: body1;
		RigidBody b2 = body2;//(body2.isInCollection() && !computeInCollection )? body2.parent: body2;

	    Vector3d normal = new Vector3d();
	    Vector3d tangent1 = new Vector3d();
	    Vector3d tangent2 = new Vector3d();
		
		body1.transformB2W.transform(contactB, contactW);
		body1.transformB2W.transform(normalB, normal);
		body1.transformB2W.transform(tangent1B, tangent1);
		body1.transformB2W.transform(tangent2B, tangent2);

		// Normal direction for both bodies
		Vector3d r1 = new Vector3d();
		Vector3d r2 = new Vector3d();

		r1.sub(contactW, b1.x);
		r2.sub(contactW, b2.x);
		
		Vector3d rn1 = new Vector3d();
		Vector3d rn2 = new Vector3d();
		rn1.cross(r1, normal);
		rn2.cross(r2, normal);
		
		DenseMatrix j;
		
		j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		j.set(0, 0, -normal.x);
		j.set(0, 1, -normal.y);
		j.set(0, 2, -normal.z);
		
		j.set(0, 3, -rn1.x);
		j.set(0, 4, -rn1.y);
		j.set(0, 5, -rn1.z);

		j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		j.set(0, 6, normal.x);
		j.set(0, 7, normal.y);
		j.set(0, 8, normal.z);
		
		j.set(0, 9, -rn2.x);
		j.set(0, 10, -rn2.y);
		j.set(0, 11, -rn2.z);
		
		// Tangential direction for both bodies
		Vector3d rt1 = new Vector3d();
		Vector3d rt2 = new Vector3d();
		rt1.cross(r1, tangent1);
		rt2.cross(r2, tangent1);
		
		j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		j.set(1, 0, -tangent1.x);
		j.set(1, 1, -tangent1.y);
		j.set(1, 2, -tangent1.z);
		
		j.set(1, 3, -rt1.x);
		j.set(1, 4, -rt1.y);
		j.set(1, 5, -rt1.z);

		j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		j.set(1, 6, tangent1.x);
		j.set(1, 7, tangent1.y);
		j.set(1, 8, tangent1.z);
		
		j.set(1, 9, -rt2.x);
		j.set(1, 10, -rt2.y);
		j.set(1, 11, -rt2.z);
		
		rt1.cross(r1, tangent2);
		rt2.cross(r2, tangent2);
		
		j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		j.set(2, 0, -tangent2.x);
		j.set(2, 1, -tangent2.y);
		j.set(2, 2, -tangent2.z);
		
		j.set(2, 3, -rt1.x);
		j.set(2, 4, -rt1.y);
		j.set(2, 5, -rt1.z);

		j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		j.set(2, 6, tangent2.x);
		j.set(2, 7, tangent2.y);
		j.set(2, 8, tangent2.z);
		
		j.set(2, 9, -rt2.x);
		j.set(2, 10, -rt2.y);
		j.set(2, 11, -rt2.z);
	}
	
	/**
	 * Stores contact forces and torques for visualization purposes
	 * @param dt
	 */
	public void computeForces(boolean computeInCollection, double dt) {

		DenseMatrix j;

		j = this.j; //(body1.isInCollection() && !computeInCollection)? this.jc: this.j;
		double f1 = lambda.get(0)*j.get(0,0) + lambda.get(1)*j.get(1,0) + lambda.get(2)*j.get(2,0);
		double f2 = lambda.get(0)*j.get(0,1) + lambda.get(1)*j.get(1,1) + lambda.get(2)*j.get(2,1);
		double f3 = lambda.get(0)*j.get(0,2) + lambda.get(1)*j.get(1,2) + lambda.get(2)*j.get(2,2);
		forceB1.set(f1,f2,f3);
		forceB1.scale(1/dt);
		f1 = lambda.get(0)*j.get(0,3) + lambda.get(1)*j.get(1,3) + lambda.get(2)*j.get(2,3);
		f2 = lambda.get(0)*j.get(0,4) + lambda.get(1)*j.get(1,4) + lambda.get(2)*j.get(2,4);
		f3 = lambda.get(0)*j.get(0,5) + lambda.get(1)*j.get(1,5) + lambda.get(2)*j.get(2,5);
		torqueB1.set(f1,f2,f3);
		torqueB1.scale(1/dt);

		j = this.j; //(body2.isInCollection() && !computeInCollection)? this.jc: this.j;
		f1 = lambda.get(0)*j.get(0,6) + lambda.get(1)*j.get(1,6) + lambda.get(2)*j.get(2,6);
		f2 = lambda.get(0)*j.get(0,7) + lambda.get(1)*j.get(1,7) + lambda.get(2)*j.get(2,7);
		f3 = lambda.get(0)*j.get(0,8) + lambda.get(1)*j.get(1,8) + lambda.get(2)*j.get(2,8);
		forceB2.set(f1,f2,f3);
		forceB2.scale(1/dt);
		f1 = lambda.get(0)*j.get(0,9) + lambda.get(1)*j.get(1,9) + lambda.get(2)*j.get(2,9);
		f2 = lambda.get(0)*j.get(0,10) + lambda.get(1)*j.get(1,10) + lambda.get(2)*j.get(2,10);
		f3 = lambda.get(0)*j.get(0,11) + lambda.get(1)*j.get(1,11) + lambda.get(2)*j.get(2,11);
		torqueB2.set(f1,f2,f3);
		torqueB2.scale(1/dt);
	}
	
	/**
	 * 
	 * @param dt
	 * @param feedbackStiffness
	 * @param computeInCollection
	 */
	public void computeB(double dt, double feedbackStiffness,  boolean computeInCollection) {
		
//		RigidBody b1 = body1;//(body1.isInCollection() && !computeInCollection)? body1.parent: body1;
//		RigidBody b2 = body2;//(body2.isInCollection() && !computeInCollection)? body2.parent: body2;
//
//		double m1inv = b1.minv;//(b1.temporarilyPinned)? 0: b1.minv; 
//		double m2inv = b2.minv;//(b2.temporarilyPinned)? 0: b2.minv;
//		Matrix3d j1inv = b1.jinv;//(b1.temporarilyPinned)? 0: b1.jinv;
//		Matrix3d j2inv = b2.jinv;//(b2.temporarilyPinned)? 0: b2.jinv;
//		
//		// add the Bounce vector to the u's over here, but don't need to do that just yet
//		double restitution = 0.;
//		if (!computeInCollection) {
//			restitution=(body1.restitution+body2.restitution)/2.;
//		}
//		
//		DenseMatrix j;
//		
//		j = this.j;//(b1 instanceof RigidCollection)? this.jc: this.j;
//		double u1xn =     (b1.v.x + b1.force.x * m1inv * dt) * j.get(0);
//		double u1yn =     (b1.v.y + b1.force.y * m1inv * dt) * j.get(1);
//		double u1omegan = (b1.omega + b1.torque * j1inv * dt) * j.get(2);
//		double bBounce = restitution*(b1.v.x*j.get(0) + b1.v.y*j.get(1) + b1.omega*j.get(2));
//
//		double u1xt =     (b1.v.x + b1.force.x * m1inv * dt) * j.get(0);
//		double u1yt =     (b1.v.y + b1.force.y * m1inv * dt) * j.get(1);
//		double u1omegat = (b1.omega + b1.torque * j1inv * dt) * j.get(2);
//
//		j = this.j;//(b2 instanceof RigidCollection)? this.jc: this.j;
//		double u2xn =     (b2.v.x + b2.force.x * m2inv * dt) * j.get(3);
//		double u2yn =     (b2.v.y + b2.force.y * m2inv * dt) * j.get(4);
//		double u2omegan = (b2.omega + b2.torque * j2inv * dt) * j.get(5);
//		bBounce += restitution*(b2.v.x*j.get(3) + b2.v.y*j.get(4) + b2.omega*j.get(5));
//
//		double u2xt =     (b2.v.x + b2.force.x * m2inv * dt) * j.get(3);
//		double u2yt =     (b2.v.y + b2.force.y * m2inv * dt) * j.get(4);
//		double u2omegat = (b2.omega + b2.torque * j2inv * dt) * j.get(5);
//
//		// calculate Baumgarte Feedback (overlap of the two bodies)
//		double baumgarteFeedback = feedbackStiffness*constraintViolation;
//		
//		// putting b together.
//		bn = u1xn + u2xn + u1yn + u2yn + u1omegan + u2omegan + bBounce + baumgarteFeedback;
//		bt = u1xt + u2xt + u1yt + u2yt + u1omegat + u2omegat;
	}
	
	/**
	 * Compute Dii values and store in contact
	 * @param computeInCollection
	 * @param compliance 
	 */
	public void computeJMinvJtDiagonal(boolean computeInCollection) {
		
//		RigidBody b1 = (body1.isInCollection() && !computeInCollection)? body1.parent: body1;
//		RigidBody b2 = (body2.isInCollection() && !computeInCollection)? body2.parent: body2;
//		
//		double m1inv = (b1.temporarilyPinned)? 0: b1.minv; 
//		double m2inv = (b2.temporarilyPinned)? 0: b2.minv;
//		double j1inv = (b1.temporarilyPinned)? 0: b1.jinv;
//		double j2inv = (b2.temporarilyPinned)? 0: b2.jinv;
//		
//		DenseVector jn;
//		DenseVector jt;
//		
//		diin = 0.;
//		diit = 0.;
//		
//		jn = (b1 instanceof RigidCollection)? this.jnc: this.jn;
//		jt = (b1 instanceof RigidCollection)? this.jtc: this.jt;
//		diin += jn.get(0) * m1inv * jn.get(0);
//		diin += jn.get(1) * m1inv * jn.get(1);
//		diin += jn.get(2) * j1inv * jn.get(2);
//		diit += jt.get(0) * m1inv * jt.get(0);
//		diit += jt.get(1) * m1inv * jt.get(1);
//		diit += jt.get(2) * j1inv * jt.get(2);
//		
//		jn = (b2 instanceof RigidCollection)? this.jnc: this.jn;
//		jt = (b2 instanceof RigidCollection)? this.jtc: this.jt;
//		diin += jn.get(3) * m2inv * jn.get(3);
//		diin += jn.get(4) * m2inv * jn.get(4);
//		diin += jn.get(5) * j2inv * jn.get(5);
//		diit += jt.get(3) * m2inv * jt.get(3);
//		diit += jt.get(4) * m2inv * jt.get(4);
//		diit += jt.get(5) * j2inv * jt.get(5);
	}
	
	/**
	 * Returns Jdv values for normal component.
	 * @param computeInCollection
	 */
	public double getJdvn(boolean computeInCollection) {
		
//		DenseVector dv1 = (body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
//		DenseVector dv2 = (body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
//		DenseVector jn;
		
		double Jdvn = 0;  		
		
//		jn = (body1.isInCollection() && !computeInCollection)? this.jnc: this.jn;
//		Jdvn += jn.get(0) * dv1.get(0);
//		Jdvn += jn.get(1) * dv1.get(1);
//		Jdvn += jn.get(2) * dv1.get(2);
//		
//		jn = (body2.isInCollection() && !computeInCollection)? this.jnc: this.jn;
//		Jdvn += jn.get(3) * dv2.get(0);
//		Jdvn += jn.get(4) * dv2.get(1);
//		Jdvn += jn.get(5) * dv2.get(2);
		
		return Jdvn;
	}
	
	/**
	 * Returns Jdv values for tangent component.
	 * @param computeInCollection
	 */
	public double getJdvt(boolean computeInCollection) {
		
//		DenseVector dv1 = (body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
//		DenseVector dv2 = (body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
//		
//		DenseVector jt;
		
		// normal component
		double Jdvt = 0;  		

//		jt = (body1.isInCollection() && !computeInCollection)? this.jtc: this.jt;
//		Jdvt += jt.get(0) * dv1.get(0);
//		Jdvt += jt.get(1) * dv1.get(1);
//		Jdvt += jt.get(2) * dv1.get(2);
//
//		jt = (body2.isInCollection() && !computeInCollection)? this.jtc: this.jt;
//		Jdvt += jt.get(3) * dv2.get(0);
//		Jdvt += jt.get(4) * dv2.get(1);
//		Jdvt += jt.get(5) * dv2.get(2);
		
		return Jdvt;
	}
    
	/**
	 * Update state of the contact: either BROKE, SLIDING or CLEAR
	 * @param mu
	 */
	protected void updateContactState(double mu) {
		if (Math.abs(lambda.get(0)) <= 1e-14) // (math.abs is for magnet)
			state = ContactState.BROKEN;	
		else if (Math.sqrt(lambda.get(1)*lambda.get(1) + lambda.get(2)*lambda.get(2)) == lambda.get(0)*mu) 
			state = ContactState.ONEDGE;
		else
			state = ContactState.CLEAR;
	}
	
    /**
     * Draws the contact points
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glPointSize(3);
        gl.glColor3f(.7f,0,0);
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex2d(contactW.x, contactW.y);
        gl.glEnd();
    }
    
    /**
     * Draws the connections between bodies to visualize the 
     * the adjacency structure of the matrix as a graph.
     * @param drawable
     */
    public void displayConnection( GLAutoDrawable drawable ) {
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
    
}
