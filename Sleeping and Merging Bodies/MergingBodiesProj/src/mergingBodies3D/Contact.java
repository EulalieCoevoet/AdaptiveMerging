package mergingBodies3D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import no.uib.cipr.matrix.DenseMatrix;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
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
	Vector3d lambda = new Vector3d();
	
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
	public void computeContactForce(boolean computeInCollection, double dt) {

//		DenseVector jn;
//		DenseVector jt;
//		
//		Vector2d cForce = new Vector2d();
//		double cTorque = 0;
//
//		jn = (body1.isInCollection() && !computeInCollection)? this.jnc: this.jn;
//		jt = (body1.isInCollection() && !computeInCollection)? this.jtc: this.jt;
//		cForce.set(lambda.x*jn.get(0) + lambda.y*jt.get(0),lambda.x*jn.get(1) + lambda.y*jt.get(1));
//		cTorque = lambda.x*jn.get(2) + lambda.y*jt.get(2);
//		cForce.scale(1/dt);
//		cTorque/=dt;
//		contactForceB1.set(cForce);
//		contactTorqueB1 = cTorque;
//
//		jn = (body2.isInCollection() && !computeInCollection)? this.jnc: this.jn;
//		jt = (body2.isInCollection() && !computeInCollection)? this.jtc: this.jt;
//		cForce.set(lambda.x*jn.get(3) + lambda.y*jt.get(3),lambda.x*jn.get(4) + lambda.y*jt.get(4));
//		cTorque = lambda.x*jn.get(5) + lambda.y*jt.get(5);
//		cForce.scale(1/dt);
//		cTorque/=dt;
//		contactForceB2.set(cForce);
//		contactTorqueB2 = cTorque;
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
