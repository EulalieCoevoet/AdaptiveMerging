package mergingBodies3D;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;

/**
 * Implementation of a contact constraint.
 * @author kry
 */
public class Contact {

    /** Next available contact index, used for determining which rows of the jacobian a contact uses */
    static public int nextContactIndex = 0;
    
    /** Index of this contact, determines its (effective) rows in the jacobian (unassembled)*/
    int index; // TODO: can this index be removed?
    
    /** First RigidBody in contact */
    RigidBody body1;
    
    /** Second RigidBody in contact */
    RigidBody body2;
    
	/** Bounding volume that caused the collision... this is only used to track contact identity for warm starts */
	BVSphere bv1;
	/** Bounding volume that caused the collision... this is only used to track contact identity for warm starts  */
	BVSphere bv2;
	/** Information to help in warm starts by allowing contacts between bodies across time steps to be matched 
	 * ONLY PUBLIC FOR DEBUGGING
	 */
	public int info;
    
    /** Position of contact point in world coordinates 
     * ONLY MADE PUBLIC FOR TESTING... can be private */
    public Point3d contactW = new Point3d();
    
    /** Contact normal in body1 coordinates 
     * ONLY PUBLIC FOR TESTING... CAN BE MADE PRIVATE */
	public Vector3d normalW = new Vector3d();
    
    /** Contact tangent1 in body1 coordinates */
    private Vector3d tangent1W = new Vector3d();
    
    /** Contact tangent2 in body1 coordinates */
    private Vector3d tangent2W = new Vector3d();
      
	/** vector points from body 2 to body 1, magnitude is the amount of overlap.*/
	public double constraintViolation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
	double prevConstraintViolation;
	
	/** Used for merge/unmerge condition */
	public ContactState state = ContactState.CLEAR;
	public enum ContactState {BROKEN, ONEDGE, CLEAR};
	
	boolean newThisTimeStep;
	
	/** Jacobian matrix, packed as trans rot trans rot on each row */
	//DenseMatrix j = new DenseMatrix(3,12);
	Vector6d jna = new Vector6d();
	Vector6d jnb = new Vector6d();
	Vector6d jt1a = new Vector6d();
	Vector6d jt1b = new Vector6d();
	Vector6d jt2a = new Vector6d();
	Vector6d jt2b = new Vector6d();
	
	/** Jacobian matrix, collection frame, packed as trans rot trans rot on each row */
	// in comments as not yet used!
	//DenseMatrix jc = new DenseMatrix(3,12);
	
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
	
	public Contact() {
		// constructor for pre-allocation
	}
	
	
    /**
     * Sets the contact, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW	in world coordinates
     * @param normal	in world woordinates
     */
    public void set( RigidBody body1, RigidBody body2, Point3d contactW, Vector3d normal, BVSphere disc1, BVSphere disc2, int info, double constraintViolation ) {
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
		
		//Vector6d jna = this.jna;   // j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		//Vector6d jt1a = this.jt1a; // j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		//Vector6d jt2a = this.jt2a; // j = this.j; //(b1 instanceof RigidCollection)? jc: this.j;
		//Vector6d jnb = this.jnb;   // j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		//Vector6d jt1b = this.jt1b; // j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;
		//Vector6d jt2b = this.jt2b; // j = this.j; //(b2 instanceof RigidCollection)? jc: this.j;

		jna.v.scale( -1, normalW );		
		jna.w.cross( normalW, r1 ); // - r1 x nW
		jnb.v.set( normalW );		
		jnb.w.cross( r2, normalW );
		
		jt1a.v.scale( -1, tangent1W );
		jt1a.w.cross( tangent1W, r1 ); // -r1 x t1W
		jt1b.v.set( tangent1W );		
		jt1b.w.cross( r2, tangent1W );	
		
		jt2a.v.scale( -1, tangent2W );
		jt2a.w.cross( tangent2W, r1 ); // -r1 x t2W
		jt2b.v.set( tangent2W );
		jt2b.w.cross( r2, tangent2W );		
	}
	
	/**
	 * Computes the b vector, which is the constraint velocity at the next time step given current forces
	 * @param dt
	 * @param feedbackStiffness
	 * @param computeInCollection
	 */
	public void computeB(double dt, double feedbackStiffness, boolean computeInCollection) {
		
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
		
		bn = 0; bt1 = 0; bt2 = 0;
		
		// Vector6d jna,t1a,t2a = this.jna,t1a,t2a; // j = this.j;//(b1 instanceof RigidCollection)? this.jc: this.j;	
		
		tmp1.scaleAdd( m1inv*dt, b1.force, b1.v );	
		bn  += tmp1.dot(jna.v);
		bt1 += tmp1.dot(jt1a.v);
		bt2 += tmp1.dot(jt2a.v);
		j1inv.transform( b1.torque, tmp1 );
		tmp1.scale( dt );
		tmp1.add( b1.omega );
		bn  += tmp1.dot( jna.w );
		bt1 += tmp1.dot( jt1a.w );
		bt2 += tmp1.dot( jt2a.w );
		
		double bBounce = b1.v.dot( jna.v ) + b1.omega.dot( jna.w );   
		bBounce *= restitution;
		bn += bBounce;
		
		tmp1.scaleAdd( m2inv*dt, b2.force, b2.v );
		bn  += tmp1.dot(jnb.v);
		bt1 += tmp1.dot(jt1b.v);
		bt2 += tmp1.dot(jt2b.v);
		j2inv.transform( b2.torque, tmp1 );
		tmp1.scale( dt );
		tmp1.add( b2.omega );
		bn  += tmp1.dot( jnb.w );
		bt1 += tmp1.dot( jt1b.w ); 
		bt2 += tmp1.dot( jt2b.w ); 

		bBounce = b2.v.dot( jnb.v ) + b2.omega.dot( jnb.w );
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
		
		//DenseMatrix j1 = this.j;//(b1 instanceof RigidCollection)? this.jc: this.j;	
		//DenseMatrix j2 = this.j;//(b2 instanceof RigidCollection)? this.jc: this.j;	
		j1inv.transform( jna.w, tmp1 );
		j2inv.transform( jnb.w, tmp2 );
		D00 = m1inv * jna.v.dot( jna.v )   + jna.w.dot( tmp1 )  + m2inv * jnb.v.dot( jnb.v )   + jnb.w.dot( tmp2 );
		j1inv.transform( jt1a.w, tmp1 );
		j2inv.transform( jt1b.w, tmp2 );
		D11 = m1inv * jt1a.v.dot( jt1a.v ) + jt1a.w.dot( tmp1 ) + m2inv * jt1b.v.dot( jt1b.v ) + jt1b.w.dot( tmp2 );
		j1inv.transform( jt2a.w, tmp1 );
		j2inv.transform( jt2b.w, tmp2 );
		D22 = m1inv * jt2a.v.dot( jt2a.v ) + jt2a.w.dot( tmp1 ) + m2inv * jt2b.v.dot( jt2b.v ) + jt2b.w.dot( tmp2 );
	}
	
	/**
	 * Returns Jdv values for given component.
	 * @param computeInCollection
	 * @param index (0 for normal, 1 for tangent1, 2 for tangent2)
	 */
	public double getJdv(boolean computeInCollection, int index) {
		
//		DenseVector dv1 = body1.deltaV;//(body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
//		DenseVector dv2 = body2.deltaV;//(body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
		Vector6d dv1 = body1.deltaV; 
		Vector6d dv2 = body2.deltaV; 
		
		Vector6d ja = jna; // j = this.j;//(body1.isInCollection() && !computeInCollection)? this.jc: this.j;
		Vector6d jb = jnb;
		if ( index == 1 ) {
			ja = jt1a;
			jb = jt1b;
		} else if ( index == 2 ) {
			ja = jt2a;
			jb = jt2b;			
		}
		
		double Jdv = ja.dot( dv1 ) + jb.dot( dv2 );
		return Jdv;
	}
    
	/**
	 * Update state of the contact: either BROKE, SLIDING or CLEAR
	 * @param mu
	 */
	protected void updateContactState(double mu) {
		if (Math.abs(lambda0) <= 1e-14) // (math.abs is for magnet)
			state = ContactState.BROKEN;	
		else if ( Math.abs(lambda1) == lambda0*mu || Math.abs(lambda2) == lambda0*mu ) 
			state = ContactState.ONEDGE;
		else
			state = ContactState.CLEAR;
	}
	
	/** Colour for drawing contacts */
    private static final float[] col = new float[] { 1f, 0, 0, 0.25f };
    private static final float[] colNew = new float[] { 0, 0.5f, 0, 0.65f };
    private static final float[] colOnEdge = new float[] { 0, 0, 0, 0.75f };
    
    /**
     * Draws the contact points
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glPointSize(3);
        gl.glMaterialfv( GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, col, 0 );
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex3d( contactW.x, contactW.y, contactW.z );
        gl.glEnd();
    }
    
    // I think these are only used for display!!! Move to the display call and use static members to save memory?
	// Seems to likewise be the case for
    
	/** temporary Contact force being applied by this contact on body1 (note this is world aligned at body COM) */
    static private Vector3d forceW1 = new Vector3d();

//	/** Contact torque being applied by this contact on body1 */
//    private Vector3d torqueW1 = new Vector3d();
//
//	/** Contact force being applied by this contact on body2 (note this is world aligned at body COM) */
//    private Vector3d forceW2 = new Vector3d();
//
//	/** Contact torque being applied by this contact on body2 */
//    private Vector3d torqueW2 = new Vector3d();

	/**
	 * Stores contact forces and torques for visualization purposes
	 * TODO: is this ONLY done for visualization?  If so, more this code to the 
	 * display method to free up memory in the contact data structure 
	 * (i.e., waste computation in drawing, rather than wasting memory!!)
	 * @param dt
	 */
	private void computeForces( boolean computeInCollection, double dt, Vector3d forceW1 ) {
		// Vector6d jna = this.jna; // j = this.j; //(body1.isInCollection() && !computeInCollection)? this.jc: this.j;
		
		forceW1.scale( lambda0, jna.v );
		forceW1.scaleAdd( lambda1, jt1a.v, forceW1 );
		forceW1.scaleAdd( lambda2, jt2a.v, forceW1 );		
		forceW1.scale(1./dt);
		
//		f1 = lambda0*j.get(0,3) + lambda1*j.get(1,3) + lambda2*j.get(2,3);
//		f2 = lambda0*j.get(0,4) + lambda1*j.get(1,4) + lambda2*j.get(2,4);
//		f3 = lambda0*j.get(0,5) + lambda1*j.get(1,5) + lambda2*j.get(2,5);
//		torqueW1.set(f1,f2,f3);
//		torqueW1.scale(1./dt);
//
//		j = this.j; //(body2.isInCollection() && !computeInCollection)? this.jc: this.j;
//		f1 = lambda0*j.get(0,6) + lambda1*j.get(1,6) + lambda2*j.get(2,6);
//		f2 = lambda0*j.get(0,7) + lambda1*j.get(1,7) + lambda2*j.get(2,7);
//		f3 = lambda0*j.get(0,8) + lambda1*j.get(1,8) + lambda2*j.get(2,8);
//		forceW2.set(f1,f2,f3);
//		forceW2.scale(1./dt);
//		f1 = lambda0*j.get(0,9)  + lambda1*j.get(1,9) +  lambda2*j.get(2,9);
//		f2 = lambda0*j.get(0,10) + lambda1*j.get(1,10) + lambda2*j.get(2,10);
//		f3 = lambda0*j.get(0,11) + lambda1*j.get(1,11) + lambda2*j.get(2,11);
//		torqueW2.set(f1,f2,f3);
//		torqueW2.scale(1./dt);
	}
	
	/**
	 * Draws the connections between bodies to visualize the 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void displayContactForce( GLAutoDrawable drawable, double dt ) {
		GL2 gl = drawable.getGL().getGL2();
		computeForces(false, dt, forceW1); // This might seem wasteful (e.g., if sim not running), but only used for debug visualization!
		gl.glLineWidth(0.75f);
		if (state == ContactState.ONEDGE) {
    		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, colOnEdge, 0 );
		} else {
			if ( newThisTimeStep ) {				
				gl.glLineWidth(4f);
	    		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, colNew, 0 );
			} else {
	    		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, col, 0 );
			}
		} 
		gl.glBegin( GL.GL_LINES );
		double scale = forceVizScale.getValue();
		gl.glVertex3d(contactW.x + scale*forceW1.x, contactW.y+scale*forceW1.y, contactW.z+scale*forceW1.z );
		gl.glVertex3d(contactW.x + -scale*forceW1.x, contactW.y+-scale*forceW1.y, contactW.z+-scale*forceW1.z );		
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
