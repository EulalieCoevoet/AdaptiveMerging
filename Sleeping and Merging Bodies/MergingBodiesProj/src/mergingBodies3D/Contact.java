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
    
	/** Bounding volume that caused the collision */
	BVSphere bv1;
	/** Bounding volume that caused the collision */
	BVSphere bv2;
    
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
	
	// Values used in PGS resolution
	double bn = 0; /** b value for normal component */
	double bt1 = 0; /** b value for tangent1 component */
	double bt2 = 0; /** b value for tangent2 component */

	DenseMatrix D = new DenseMatrix(3,3); 
	
    /**
     * Creates a new contact, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW
     * @param normal
     */
    public Contact( RigidBody body1, RigidBody body2, Point3d contactW, Vector3d normal, BVSphere disc1, BVSphere disc2, double constraintViolation ) {
        this.body1 = body1;
        this.body2 = body2;
        this.contactW.set( contactW ); 
		bv1 = disc1;
		bv2 = disc2;
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
		
		lambda.zero();
		
        computeJacobian(true);
        computeJacobian(false);
    }
    
	public Contact(Contact contact) {		
		body1 = contact.body1;	
		body2 = contact.body2;	
		normalB.set(contact.normalB);   	
		tangent1B.set(contact.tangent1B);   	
		tangent2B.set(contact.tangent2B);	
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
		
		j.set(0, 9, rn2.x);
		j.set(0, 10, rn2.y);
		j.set(0, 11, rn2.z);
		
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
		
		j.set(1, 9, rt2.x);
		j.set(1, 10, rt2.y);
		j.set(1, 11, rt2.z);
		
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
		
		j.set(2, 9, rt2.x);
		j.set(2, 10, rt2.y);
		j.set(2, 11, rt2.z);
		
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
		forceB1.scale(1./dt);
		f1 = lambda.get(0)*j.get(0,3) + lambda.get(1)*j.get(1,3) + lambda.get(2)*j.get(2,3);
		f2 = lambda.get(0)*j.get(0,4) + lambda.get(1)*j.get(1,4) + lambda.get(2)*j.get(2,4);
		f3 = lambda.get(0)*j.get(0,5) + lambda.get(1)*j.get(1,5) + lambda.get(2)*j.get(2,5);
		torqueB1.set(f1,f2,f3);
		torqueB1.scale(1./dt);

		j = this.j; //(body2.isInCollection() && !computeInCollection)? this.jc: this.j;
		f1 = lambda.get(0)*j.get(0,6) + lambda.get(1)*j.get(1,6) + lambda.get(2)*j.get(2,6);
		f2 = lambda.get(0)*j.get(0,7) + lambda.get(1)*j.get(1,7) + lambda.get(2)*j.get(2,7);
		f3 = lambda.get(0)*j.get(0,8) + lambda.get(1)*j.get(1,8) + lambda.get(2)*j.get(2,8);
		forceB2.set(f1,f2,f3);
		forceB2.scale(1./dt);
		f1 = lambda.get(0)*j.get(0,9)  + lambda.get(1)*j.get(1,9) +  lambda.get(2)*j.get(2,9);
		f2 = lambda.get(0)*j.get(0,10) + lambda.get(1)*j.get(1,10) + lambda.get(2)*j.get(2,10);
		f3 = lambda.get(0)*j.get(0,11) + lambda.get(1)*j.get(1,11) + lambda.get(2)*j.get(2,11);
		torqueB2.set(f1,f2,f3);
		torqueB2.scale(1./dt);
	}
	
	/**
	 * 
	 * @param dt
	 * @param feedbackStiffness
	 * @param computeInCollection
	 */
	public void computeB(double dt, double feedbackStiffness,  boolean computeInCollection) {
		
		RigidBody b1 = body1;//(body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = body2;//(body2.isInCollection() && !computeInCollection)? body2.parent: body2;

		double m1inv = b1.minv;//(b1.temporarilyPinned)? 0: b1.minv; 
		double m2inv = b2.minv;//(b2.temporarilyPinned)? 0: b2.minv;
		Matrix3d j1inv = b1.jinv;//(b1.temporarilyPinned)? 0: b1.jinv;
		Matrix3d j2inv = b2.jinv;//(b2.temporarilyPinned)? 0: b2.jinv;
		
		// add the Bounce vector to the u's over here, but don't need to do that just yet
		double restitution = 0.;
		if (!computeInCollection) {
			restitution=(body1.restitution+body2.restitution)/2.;
		}
		
		DenseMatrix j;
		Vector3d u = new Vector3d();
		bn = 0; bt1 = 0; bt2 = 0;
		
		j = this.j;//(b1 instanceof RigidCollection)? this.jc: this.j;		
		u.set(b1.force);
		u.scale(m1inv*dt);	
		u.add(b1.v);
		bn  += u.x*j.get(0,0) + u.y*j.get(0,1) + u.z*j.get(0,2);
		bt1 += u.x*j.get(1,0) + u.y*j.get(1,1) + u.z*j.get(1,2);
		bt2 += u.x*j.get(2,0) + u.y*j.get(2,1) + u.z*j.get(2,2);
		u.set(b1.torque.x*j1inv.m00 + b1.torque.y*j1inv.m01 + b1.torque.z*j1inv.m02,
			  b1.torque.x*j1inv.m10 + b1.torque.y*j1inv.m11 + b1.torque.z*j1inv.m12,
			  b1.torque.x*j1inv.m20 + b1.torque.y*j1inv.m21 + b1.torque.z*j1inv.m22);
		u.scale(dt);
		u.add(b1.omega);
		bn  += u.x*j.get(0,3) + u.y*j.get(0,4) + u.z*j.get(0,5);
		bt1 += u.x*j.get(1,3) + u.y*j.get(1,4) + u.z*j.get(1,5);
		bt2 += u.x*j.get(2,3) + u.y*j.get(2,4) + u.z*j.get(2,5);
		
		double bBounce = (b1.v.x*j.get(0,0) + b1.v.y*j.get(0,1) + b1.v.z*j.get(0,2));
		bBounce += (b1.omega.x*j.get(0,3) + b1.omega.y*j.get(0,4) + b1.omega.z*j.get(0,5));
		bBounce *= restitution;
		bn += bBounce;
		
		j = this.j;//(b2 instanceof RigidCollection)? this.jc: this.j;
		u.set(b2.force);
		u.scale(m2inv*dt);	
		u.add(b2.v);
		bn  += u.x*j.get(0,6) + u.y*j.get(0,7) + u.z*j.get(0,8);
		bt1 += u.x*j.get(1,6) + u.y*j.get(1,7) + u.z*j.get(1,8);
		bt2 += u.x*j.get(2,6) + u.y*j.get(2,7) + u.z*j.get(2,8);
		u.set(b2.torque.x*j2inv.m00 + b2.torque.y*j2inv.m01 + b2.torque.z*j2inv.m02,
			  b2.torque.x*j2inv.m10 + b2.torque.y*j2inv.m11 + b2.torque.z*j2inv.m12,
			  b2.torque.x*j2inv.m20 + b2.torque.y*j2inv.m21 + b2.torque.z*j2inv.m22);
		u.scale(dt);
		u.add(b2.omega);
		bn  += u.x*j.get(0,9) + u.y*j.get(0,10) + u.z*j.get(0,11);
		bt1 += u.x*j.get(1,9) + u.y*j.get(1,10) + u.z*j.get(1,11);
		bt2 += u.x*j.get(2,9) + u.y*j.get(2,10) + u.z*j.get(2,11);

		bBounce = (b2.v.x*j.get(0,6) + b2.v.y*j.get(0,7) + b2.v.z*j.get(0,8));
		bBounce += (b2.omega.x*j.get(0,9) + b2.omega.y*j.get(0,10) + b2.omega.z*j.get(0,11));
		bBounce *= restitution;
		bn += bBounce;
		
		// calculate Baumgarte Feedback (overlap of the two bodies)
		double baumgarteFeedback = feedbackStiffness*constraintViolation;
		bn += baumgarteFeedback;
	}
	
	/**
	 * Compute Dii values and store in contact
	 * @param computeInCollection
	 * @param compliance 
	 */
	public void computeJMinvJt(boolean computeInCollection) {
		
		RigidBody b1 = body1;//(body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = body2;//(body2.isInCollection() && !computeInCollection)? body2.parent: body2;

		double m1inv = b1.minv;//(b1.temporarilyPinned)? 0: b1.minv; 
		double m2inv = b2.minv;//(b2.temporarilyPinned)? 0: b2.minv;
		Matrix3d j1inv = b1.jinv;//(b1.temporarilyPinned)? 0: b1.jinv;
		Matrix3d j2inv = b2.jinv;//(b2.temporarilyPinned)? 0: b2.jinv;
		DenseMatrix j, j1, j2;
		
		DenseMatrix Minv = new DenseMatrix(12,12);
		
		//eulalie: could be optimized
		Minv.set(0, 0, m1inv);
		Minv.set(1, 1, m1inv);
		Minv.set(2, 2, m1inv);

		for (int k=0; k<3; k++)
			for (int l=0; l<3; l++)
				Minv.set(3+k, 3+l, j1inv.getElement(k, l));
		
		Minv.set(6, 6, m2inv);
		Minv.set(7, 7, m2inv);
		Minv.set(8, 8, m2inv);
		
		for (int k=0; k<3; k++)
			for (int l=0; l<3; l++)
				Minv.set(9+k, 9+l, j2inv.getElement(k, l));

		j1 = this.j;//(b1 instanceof RigidCollection)? this.jc: this.j;	
		j2 = this.j;//(b2 instanceof RigidCollection)? this.jc: this.j;	
		
		j = new DenseMatrix(3,12);
		for (int k=0; k<3; k++)
			for (int l=0; l<6; l++)
				j.set(k, l, j1.get(k, l));
		for (int k=0; k<3; k++)
			for (int l=0; l<6; l++)
				j.set(k, 6+l, j2.get(k, l));
		
		DenseMatrix MinvJT = new DenseMatrix(12,3);
		Minv.transBmult(j, MinvJT);
		j.mult(MinvJT, D);
	}
	
	/**
	 * Returns Jdv values for given component.
	 * @param computeInCollection
	 * @param index (0 for normal, 1 for tangent1, 2 for tangent2)
	 */
	public double getJdv(boolean computeInCollection, int index) {
		
		DenseVector dv1 = body1.deltaV;//(body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
		DenseVector dv2 = body2.deltaV;//(body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
		DenseMatrix j;
		
		double Jdv = 0;  		
		
		j = this.j;//(body1.isInCollection() && !computeInCollection)? this.jc: this.j;
		Jdv += j.get(index,0) * dv1.get(0);
		Jdv += j.get(index,1) * dv1.get(1);
		Jdv += j.get(index,2) * dv1.get(2);
		Jdv += j.get(index,3) * dv1.get(3);
		Jdv += j.get(index,4) * dv1.get(4);
		Jdv += j.get(index,5) * dv1.get(5);

		j = this.j;//(body2.isInCollection() && !computeInCollection)? this.jc: this.j;
		Jdv += j.get(index,6) * dv2.get(0);
		Jdv += j.get(index,7) * dv2.get(1);
		Jdv += j.get(index,8) * dv2.get(2);
		Jdv += j.get(index,9) * dv2.get(3);
		Jdv += j.get(index,10) * dv2.get(4);
		Jdv += j.get(index,11) * dv2.get(5);
		
		return Jdv;
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
