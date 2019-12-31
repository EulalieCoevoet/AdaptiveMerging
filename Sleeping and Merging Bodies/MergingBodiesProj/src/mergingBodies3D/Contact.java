package mergingBodies3D;

import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;
import no.uib.cipr.matrix.DenseMatrix;
import no.uib.cipr.matrix.DenseVector;

/**
 * Implementation of a contact constraint.
 * TODO: MEMORY: Consider pooling contacts to avoid reallocation all the time... 
 * This is one of a few larger objects that will be created VERY OFTEN!
 * @author kry
 */
public class Contact {

    /** Next available contact index, used for determining which rows of the jacobian a contact uses */
    static public int nextContactIndex = 0;
    
    /** Index of this contact, determines its (effective) rows in the jacobian (unassembled)*/
    int index;
    
    /** First RigidBody in contact */
    RigidBody body1;
    
    /** Second RigidBody in contact */
    RigidBody body2;
    
	/** Bounding volume that caused the collision... this is only used to track contact identity for warm starts */
	BVSphere bv1;
	/** Bounding volume that caused the collision... this is only used to track contact identity for warm starts  */
	BVSphere bv2;
	/** Information to help in warm starts by allowing contacts between bodies across time steps to be matched */
	int info;
    
    /** Contact normal in body1 coordinates */
	private Vector3d normalW = new Vector3d();
    
    /** Contact tangent1 in body1 coordinates */
    private Vector3d tangent1W = new Vector3d();
    
    /** Contact tangent2 in body1 coordinates */
    private Vector3d tangent2W = new Vector3d();
    
	/** Contact force being applied by this contact on body1 (note this is world aligned at body COM) */
    private Vector3d forceW1 = new Vector3d();

	/** Contact torque being applied by this contact on body1 */
    private Vector3d torqueW1 = new Vector3d();

	/** Contact force being applied by this contact on body2 (note this is world aligned at body COM) */
    private Vector3d forceW2 = new Vector3d();

	/** Contact torque being applied by this contact on body2 */
    private Vector3d torqueW2 = new Vector3d();
    
    /** Position of contact point in world coordinates */
    private Point3d contactW = new Point3d();
    
	/** vector points from body 2 to body 1, magnitude is the amount of overlap.*/
	double constraintViolation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
	double prevConstraintViolation;
	
	/** Used for merge/unmerge condition */
	public ContactState state = ContactState.CLEAR;
	public enum ContactState {BROKEN, ONEDGE, CLEAR};
	
	boolean newThisTimeStep;
	
	/** Jacobian matrix, packed as trans rot trans rot on each row */
	DenseMatrix j = new DenseMatrix(3,12);
	
	/** Jacobian matrix, collection frame, packed as trans rot trans rot on each row */
	DenseMatrix jc = new DenseMatrix(3,12);
	
	/** Lagrange multiplier for contact, Vector2d(normal, tangent1, tangent2) */
	double lambda0;
	double lambda1;
	double lambda2;
	
	/** b value for normal component (used in PGS resolution) */
	double bn; 
	/** b value for tangent1 component (used in PGS resolution) */
	double bt1; 
	/** b value for tangent2 component (used in PGS resolution) */
	double bt2; 
	
	/** Diagonals of J Minv J^T  */
	double D00;
	double D11;
	double D22;
	
    /**
     * Creates a new contact, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW	in world coordinates
     * @param normal	in world woordinates
     */
    public Contact( RigidBody body1, RigidBody body2, Point3d contactW, Vector3d normal, BVSphere disc1, BVSphere disc2, int info, double constraintViolation ) {
        this.body1 = body1;
        this.body2 = body2;
        this.contactW.set( contactW ); 
		bv1 = disc1;
		bv2 = disc2;
		this.info = info;
		this.constraintViolation =  constraintViolation;     
        index = nextContactIndex++;     
        
		this.normalW.set(normal);

		double anx = Math.abs( normalW.x );
		double any = Math.abs( normalW.y );
		double anz = Math.abs( normalW.z );
		if ( anx < any && anx < anz ) {
			tangent1W.set( 1, 0, 0 );
		} else if ( any < anz ) {
			tangent1W.set( 0, 1, 0 );
		} else {
			tangent1W.set( 0, 0, 1 );
		}
		
		tangent2W.cross( normalW, tangent1W );
		tangent2W.normalize();
		tangent1W.cross( tangent2W,  normalW );  // and doesn't need normalization 

		lambda0 = 0; 
		lambda1 = 0;
		lambda2 = 0;
		
        computeJacobian(true);
        computeJacobian(false);
    }
    
    /**
	 * Computes the Jacobian matrix of the contact.
	 * In case of body in a collection, use COM of parent to compute the torque component of the Jacobian.
	 */
	public void computeJacobian(boolean computeInCollection) {
				
		RigidBody b1 = body1;//(body1.isInCollection() && !computeInCollection )? body1.parent: body1;
		RigidBody b2 = body2;//(body2.isInCollection() && !computeInCollection )? body2.parent: body2;
	    
		r1.sub( contactW, b1.x );
		r2.sub( contactW, b2.x );
		
		// Normal direction for both bodies
				
		DenseMatrix j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		j.set(0, 0, -normalW.x);
		j.set(0, 1, -normalW.y);
		j.set(0, 2, -normalW.z);
		
		tmp1.cross( r1,  normalW );
		j.set(0, 3, -tmp1.x);
		j.set(0, 4, -tmp1.y);
		j.set(0, 5, -tmp1.z);

		j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		j.set(0, 6, normalW.x);
		j.set(0, 7, normalW.y);  // HELP... is this getting overridden somewhere?  I'm seeing some funy signs in this matrix later on...
		j.set(0, 8, normalW.z);
		
		tmp1.cross(r2, normalW);
		j.set(0, 9, tmp1.x);
		j.set(0, 10, tmp1.y);
		j.set(0, 11, tmp1.z);
		
		// Tangential direction for both bodies
		
		j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		j.set(1, 0, -tangent1W.x);
		j.set(1, 1, -tangent1W.y);
		j.set(1, 2, -tangent1W.z);
		
		tmp1.cross(r1, tangent1W);
		j.set(1, 3, -tmp1.x);
		j.set(1, 4, -tmp1.y);
		j.set(1, 5, -tmp1.z);

		j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		j.set(1, 6, tangent1W.x);
		j.set(1, 7, tangent1W.y);
		j.set(1, 8, tangent1W.z);
		
		tmp1.cross(r2, tangent1W);		
		j.set(1, 9, tmp1.x);
		j.set(1, 10, tmp1.y);
		j.set(1, 11, tmp1.z);
		
		j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		j.set(2, 0, -tangent2W.x);
		j.set(2, 1, -tangent2W.y);
		j.set(2, 2, -tangent2W.z);

		tmp1.cross(r1, tangent2W);
		j.set(2, 3, -tmp1.x);
		j.set(2, 4, -tmp1.y);
		j.set(2, 5, -tmp1.z);

		j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		j.set(2, 6, tangent2W.x);
		j.set(2, 7, tangent2W.y);
		j.set(2, 8, tangent2W.z);

		tmp1.cross(r2, tangent2W);		
		j.set(2, 9, tmp1.x);
		j.set(2, 10, tmp1.y);
		j.set(2, 11, tmp1.z);
	}
	
	/**
	 * Stores contact forces and torques for visualization purposes
	 * @param dt
	 */
	public void computeForces(boolean computeInCollection, double dt) {

		DenseMatrix j;

		j = this.j; //(body1.isInCollection() && !computeInCollection)? this.jc: this.j;
		double f1 = lambda0*j.get(0,0) + lambda1*j.get(1,0) + lambda2*j.get(2,0);
		double f2 = lambda0*j.get(0,1) + lambda1*j.get(1,1) + lambda2*j.get(2,1);
		double f3 = lambda0*j.get(0,2) + lambda1*j.get(1,2) + lambda2*j.get(2,2);
		forceW1.set(f1,f2,f3);
		forceW1.scale(1./dt);
		f1 = lambda0*j.get(0,3) + lambda1*j.get(1,3) + lambda2*j.get(2,3);
		f2 = lambda0*j.get(0,4) + lambda1*j.get(1,4) + lambda2*j.get(2,4);
		f3 = lambda0*j.get(0,5) + lambda1*j.get(1,5) + lambda2*j.get(2,5);
		torqueW1.set(f1,f2,f3);
		torqueW1.scale(1./dt);

		j = this.j; //(body2.isInCollection() && !computeInCollection)? this.jc: this.j;
		f1 = lambda0*j.get(0,6) + lambda1*j.get(1,6) + lambda2*j.get(2,6);
		f2 = lambda0*j.get(0,7) + lambda1*j.get(1,7) + lambda2*j.get(2,7);
		f3 = lambda0*j.get(0,8) + lambda1*j.get(1,8) + lambda2*j.get(2,8);
		forceW2.set(f1,f2,f3);
		forceW2.scale(1./dt);
		f1 = lambda0*j.get(0,9)  + lambda1*j.get(1,9) +  lambda2*j.get(2,9);
		f2 = lambda0*j.get(0,10) + lambda1*j.get(1,10) + lambda2*j.get(2,10);
		f3 = lambda0*j.get(0,11) + lambda1*j.get(1,11) + lambda2*j.get(2,11);
		torqueW2.set(f1,f2,f3);
		torqueW2.scale(1./dt);
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
	
	
	/** THREADS: not threadsafe, but feels gross to allocate more temporary computation objects into the object itself */
	static private Vector3d tmp1 = new Vector3d();
	static private Vector3d tmp2 = new Vector3d();
	static private Vector3d r1 = new Vector3d();
	static private Vector3d r2 = new Vector3d();

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
		
		DenseMatrix j1 = this.j;//(b1 instanceof RigidCollection)? this.jc: this.j;	
		DenseMatrix j2 = this.j;//(b2 instanceof RigidCollection)? this.jc: this.j;	

		// Code below could have been done in a loop, but unrolled like this we
		// can also avoid the bounds checking (not j should probably be broken down into basic variables 
		tmp1.set( j1.get(0,0), j1.get(0,1), j1.get(0,2) );
		D00 = m1inv * tmp1.dot(tmp1);
		tmp1.set( j1.get(0,3), j1.get(0,4), j1.get(0,5) );
		j1inv.transform(tmp1, tmp2);
		D00 += tmp1.dot( tmp2 );
		tmp1.set( j2.get(0,6), j2.get(0,7), j2.get(0,8) );
		D00 += m2inv * tmp1.dot(tmp1);
		tmp1.set( j2.get(0,9), j2.get(0,10), j2.get(0,11) );
		j2inv.transform(tmp1, tmp2);
		D00 += tmp1.dot( tmp2 );

		tmp1.set( j1.get(1,0), j1.get(1,1), j1.get(1,2) );
		D11 = m1inv * tmp1.dot(tmp1);
		tmp1.set( j1.get(1,3), j1.get(1,4), j1.get(1,5) );
		j1inv.transform(tmp1, tmp2);
		D11 += tmp1.dot( tmp2 );
		tmp1.set( j2.get(1,6), j2.get(1,7), j2.get(1,8) );
		D11 += m2inv * tmp1.dot(tmp1);
		tmp1.set( j2.get(1,9), j2.get(1,10), j2.get(1,11) );
		j2inv.transform(tmp1, tmp2);
		D11 += tmp1.dot( tmp2 );

		tmp1.set( j1.get(2,0), j1.get(2,1), j1.get(2,2) );
		D22 = m1inv * tmp1.dot(tmp1);
		tmp1.set( j1.get(2,3), j1.get(2,4), j1.get(2,5) );
		j1inv.transform(tmp1, tmp2);
		D22 += tmp1.dot( tmp2 );
		tmp1.set( j2.get(2,6), j2.get(2,7), j2.get(2,8) );
		D22 += m2inv * tmp1.dot(tmp1);
		tmp1.set( j2.get(2,9), j2.get(2,10), j2.get(2,11) );
		j2inv.transform(tmp1, tmp2);
		D22 += tmp1.dot( tmp2 );
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
		if (Math.abs(lambda0) <= 1e-14) // (math.abs is for magnet)
			state = ContactState.BROKEN;	
		else if (Math.sqrt(lambda1*lambda1 + lambda2*lambda2) == lambda0*mu) 
			state = ContactState.ONEDGE;
		else
			state = ContactState.CLEAR;
	}
	
	/** Colour for drawing contacts */
    private static final float[] col = new float[] { 1f, 0, 0, 0.25f };

    /**
     * Draws the contact points
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glPointSize(3);
        //gl.glColor4d(.7,0,0,0.15);
        gl.glMaterialfv( GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, col, 0 );
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex3d( contactW.x, contactW.y, contactW.z );
        gl.glEnd();
    }
    
	/**
	 * Draws the connections between bodies to visualize the 
	 * the adjacency structure of the matrix as a graph.
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
		gl.glVertex3d(contactW.x + scale*forceW1.x, contactW.y+scale*forceW1.y, contactW.z+scale*forceW1.z );
		gl.glVertex3d(contactW.x + scale*forceW2.x, contactW.y+scale*forceW2.y, contactW.z+scale*forceW2.z );		
		gl.glEnd();
	}
    
	static DoubleParameter forceVizScale = new DoubleParameter("force viz scale", 0.05, 0.0001, 1);

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
            gl.glVertex3d(body1.x.x, body1.x.y, body1.x.z );
            gl.glVertex3d(body2.x.x, body2.x.y, body2.x.z );
            gl.glEnd();
        }
    }
    
    /**
     * Returns a hash code which matches even if the contact was created with
     * the two rigid bodies swapped
     */
    @Override
    public int hashCode() {
    	int b1h = hashcode(body1);
    	int b2h = hashcode(body2);
    	int bv1h = hashcode(bv1);
    	int bv2h = hashcode(bv2);
    	// hashing primes from https://planetmath.org/goodhashtableprimes
    	if ( b1h < b2h ) {
    		return 53 * bv1h + 97 * bv2h + 193 * b1h + 389 * b2h + info; 
    	}
        return 53 * bv2h + 97 * bv1h + 193 * b2h + 389 * b1h + info; 
    }

    private static int hashcode(Object o) {
        return o == null ? 0 : o.hashCode();
    }

    /**
     * Compare contacts for warm starting across time steps
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Contact)) return false;
        if (this == obj) return true;
        Contact c = (Contact) obj;
        if ( info != c.info ) return false;
        return body1 == c.body1 && bv1 == c.bv1 && body2 == c.body2 && bv2 == c.bv2 ||
        		body2 == c.body1 && bv2 == c.bv1 && body1 == c.body2 && bv1 == c.bv2;    
    }

}
