package mergingBodies3D;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.DoubleParameter;
import mintools.viewer.EasyViewer;

/**
 * Implementation of a contact constraint.
 * @author kry
 */
public class Contact {
    
    /** First RigidBody in contact */
    RigidBody body1;
    /** Second RigidBody in contact */
    RigidBody body2;
    
    // CONTACT ID INFO, i.e., the origin of the contact, for warm starting and used in the hash computation
    
    /** Bounding volume that caused the collision... this is only used to track contact identity for warm starts */
	BVSphere bv1;
	/** Bounding volume that caused the collision... this is only used to track contact identity for warm starts  */
	BVSphere bv2;
	/** Information to help in warm starts by allowing contacts between bodies across time steps to be matched */
	public int info;
	/** composite sub body 1, for warm starting with composites **/
	private RigidBody csb1 = null;
	/** composite sub body 2, for warm starting with composites **/
	private RigidBody csb2 = null;
    
	// CONTACT FRAME, stored in body, but originally specified in world.
	
    /** Position of contact point in world coordinates ONLY MADE PUBLIC FOR TESTING... can be private */
    public Point3d contactB1 = new Point3d();
    /** Contact normal in world coordinates  ONLY PUBLIC FOR TESTING... CAN BE MADE PRIVATE */
	public Vector3d normalB1 = new Vector3d();
    /** Contact tangent1 in world coordinates */
    private Vector3d tangent1B1 = new Vector3d();
    /** Contact tangent2 in world coordinates */
    private Vector3d tangent2B1 = new Vector3d();
    
    
	/** vector points from body 2 to body 1, magnitude is the amount of overlap.*/
	public double constraintViolation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
	double prevConstraintViolation;
	
	/** Used for merge/unmerge condition */
	public ContactState state = ContactState.CLEAR;
	public enum ContactState {BROKEN, ONEDGE, CLEAR};
	
	boolean newThisTimeStep;
	
	/** Jacobian matrix, packed as trans rot trans rot on each row */
	Vector6d jna = new Vector6d();
	Vector6d jnb = new Vector6d();
	Vector6d jt1a = new Vector6d();
	Vector6d jt1b = new Vector6d();
	Vector6d jt2a = new Vector6d();
	Vector6d jt2b = new Vector6d();
		
	/** Lagrange multiplier for contact, Vector2d(normal, tangent1, tangent2) */
	double lambda0;
	double lambda1;
	double lambda2;
	
	/** Warm start values are needed more than once if we do both a single cycle and a full solve */
	double lambda0warm;
	double lambda1warm;
	double lambda2warm;
	
	// TODO: the b values and D values are needed per contact, but don't need to be persistent (they are working variables)
	// If contacts had unique IDs then these could simply be sotred in some other array... that said, the far easiest thing
	// is to leave these here just like this
	
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
	
	/** temporary Contact force being applied by this contact on body1 (note this is world aligned at body COM)... only used for display */
    static private Vector3d forceW1 = new Vector3d();
    	
    /** Temporary contact frame information, used for display and to recompute jacobians */
    static private Point3d contactInW = new Point3d();
    static private Vector3d normalInW = new Vector3d();
    static private Vector3d tangent1InW = new Vector3d();
    static private Vector3d tangent2InW = new Vector3d();
	
	public Contact() {
		// constructor for pre-allocation
	}
	
	/**
	 * Copy constructor, this does not exploit our memory pooling efforts! 
	 * 
	 * This is how contacts become persistent in longer lived collections, thus we
	 * store information about the contact frame arbitrarily in Body1 coordinates 
	 * as Body1 and Body2 will not move relative to one another for the lifetime 
	 * of this copied contact.
	 * 
	 * This should ONLY be called when migrating an active external contat in the world
	 * into a collection.
	 * 
	 * @param contact
	 */
	public Contact(Contact contact) {
		body1 = contact.body1;
		body2 = contact.body2;
		bv1 = contact.bv1;
		bv2 = contact.bv2;
		info = contact.info;
		csb1 = contact.csb1;
		csb2 = contact.csb2;
		
		contactB1.set(contact.contactB1);
		normalB1.set(contact.normalB1);   	
		tangent1B1.set(contact.tangent1B1);  
		tangent2B1.set(contact.tangent2B1);  
												
		lambda0 = contact.lambda0;
		lambda1 = contact.lambda1;
		lambda2 = contact.lambda2;
		
		jna.set( contact.jna );
		jnb.set( contact.jnb );
		jt1a.set( contact.jt1a );
		jt1b.set( contact.jt1b );
		jt2a.set( contact.jt2a );
		jt2b.set( contact.jt2b );
		
		state = contact.state;
		newThisTimeStep = contact.newThisTimeStep;
				
		constraintViolation = contact.constraintViolation;	
		prevConstraintViolation = contact.prevConstraintViolation;	
	}
	
    /**
     * Sets the contact, and assigns it an index, only ever called for NEW contacts!
     * @param b1
     * @param b2
     * @param contactW	in world coordinates
     * @param normalW	in world coordinates
     */
    public void set( RigidBody b1, RigidBody b2, Point3d contactW, Vector3d normalW, BVSphere disc1, BVSphere disc2, int info, double constraintViolation ) {    	
    	body1 = b1;
        body2 = b2;
		this.info = info;
        
		// Just add the composite sub bodies here!!  it will be needed for warm start information...
		
		if ( body1.isInComposite() ) { // use the real body if this is a sub body in a composite
        	body1 = b1.compositeBodyParent;
        	csb1 = b1;
        }
        if ( this.body2.isInComposite() ) { // same for body 2
        	body2 = b2.compositeBodyParent;
        	csb2 = b2;
        }
        
        // TODO: last thing to fix here (Before writing the collision processing) is the info
        // Perhaps a body can know its index in the composite body list, and we can add 100 * id to the info to
        // make sure that different sub bodies with the same info do not collide wrt the warm start.
        
		bv1 = disc1;
		bv2 = disc2;
		this.constraintViolation =  constraintViolation; //  + 1e-3;  // try to not actually push things completely apart!   perhaps??
		
		// why would initializing this break merging?
		//this.prevConstraintViolation = 0; // don't know it until after warm start.
		
		lambda0 = 0; 
		lambda1 = 0;
		lambda2 = 0;
		
		lambda0warm = 0;
		lambda1warm = 0;
		lambda2warm = 0;
        
        this.contactB1.set( contactW ); 	// note naming mactch... in world for the next few lines, and to build jacobian, but will be converted to body!
		this.normalB1.set(normalW);

		double anx = Math.abs( normalW.x );
		double any = Math.abs( normalW.y );
		double anz = Math.abs( normalW.z );
		if ( anx < any && anx < anz ) {
			tangent1B1.set( 1, 0, 0 );
		} else if ( any < anz ) {
			tangent1B1.set( 0, 1, 0 );
		} else {
			tangent1B1.set( 0, 0, 1 );
		}
		
		tangent2B1.cross( normalW, tangent1B1 );
		tangent2B1.normalize();
		tangent1B1.cross( tangent2B1,  normalW );  // and doesn't need normalization 
		
		// these quantities were built in world, so we can build the jacobian with them...
        computeJacobian(false, contactB1, normalB1, tangent1B1, tangent2B1 ); // the boolean doesn't matter here
        // Now need to store them in body coords in case this contact becomes
        // persistent and the frames need to move with the bodies
        body1.transformB2W.inverseTransform( contactB1 );
        body1.transformB2W.inverseTransform( normalB1 );
        body1.transformB2W.inverseTransform( tangent1B1 );
        body1.transformB2W.inverseTransform( tangent2B1 );
    }
    
    private void computeJacobian( boolean computeInCollection, Point3d contactW, Vector3d normalW, Vector3d tangent1W, Vector3d tangent2W ) {
		RigidBody b1 = (body1.isInCollection() && !computeInCollection )? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection )? body2.parent: body2;
		r1.sub( contactW, b1.x );
		r2.sub( contactW, b2.x );

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
	 * Computes the Jacobian matrix of the contact, where the contact frame is built in the world using
	 * the body's current orientation (i.e., so that it can change orientation if inside a collection
	 * 
	 * uses static mem to get the job done (some variable shared with the display code)
	 * 
	 * In case of body in a collection, use COM of parent to compute the torque component of the Jacobian.
	 */
	public void computeJacobian(boolean computeInCollection) {
        body1.transformB2W.transform( contactB1, contactInW );
        body1.transformB2W.transform( normalB1, normalInW );
        body1.transformB2W.transform( tangent1B1, tangent1InW );
        body1.transformB2W.transform( tangent2B1, tangent2InW );
        computeJacobian(computeInCollection, contactInW, normalInW, tangent1InW, tangent2InW );        
	}
	
	/**
	 * Computes the b vector, which is the constraint velocity at the next time step given current forces
	 * @param dt
	 * @param feedbackStiffness
	 * @param computeInCollection
	 */
	public void computeB(double dt, double feedbackStiffness, boolean computeInCollection, boolean restituionOverride, double restituionOverrideVal ) {
		
		RigidBody b1 = (body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection)? body2.parent: body2;
		
		// add the Bounce vector to the u's over here, but don't need to do that just yet
		// Note: For the single iteration PGS the restitution should be considered.
		double restitution = (body1.restitution+body2.restitution)/2.;
		if ( restituionOverride ) {
			restitution = restituionOverrideVal;
		}
		
		bn = 0; bt1 = 0; bt2 = 0;
		
		tmp1.scaleAdd( b1.minv*dt, b1.force, b1.v );	
		bn  += tmp1.dot(jna.v);
		bt1 += tmp1.dot(jt1a.v);
		bt2 += tmp1.dot(jt2a.v);
		b1.jinv.transform( b1.torque, tmp1 );
		tmp1.scale( dt );
		tmp1.add( b1.omega );
		bn  += tmp1.dot( jna.w );
		bt1 += tmp1.dot( jt1a.w );
		bt2 += tmp1.dot( jt2a.w );
		
		double bBounce = b1.v.dot( jna.v ) + b1.omega.dot( jna.w );   
		bBounce *= restitution;
		bn += bBounce;
		
		tmp1.scaleAdd( b2.minv*dt, b2.force, b2.v );
		bn  += tmp1.dot(jnb.v);
		bt1 += tmp1.dot(jt1b.v);
		bt2 += tmp1.dot(jt2b.v);
		b2.jinv.transform( b2.torque, tmp1 );
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
		
		RigidBody b1 = (body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection)? body2.parent: body2;
		
		b1.jinv.transform( jna.w, tmp1 );
		b2.jinv.transform( jnb.w, tmp2 );
		D00 = b1.minv * jna.v.dot( jna.v )   + jna.w.dot( tmp1 )  + b2.minv * jnb.v.dot( jnb.v )   + jnb.w.dot( tmp2 );
		b1.jinv.transform( jt1a.w, tmp1 );
		b2.jinv.transform( jt1b.w, tmp2 );
		D11 = b1.minv * jt1a.v.dot( jt1a.v ) + jt1a.w.dot( tmp1 ) + b2.minv * jt1b.v.dot( jt1b.v ) + jt1b.w.dot( tmp2 );
		b1.jinv.transform( jt2a.w, tmp1 );
		b2.jinv.transform( jt2b.w, tmp2 );
		D22 = b1.minv * jt2a.v.dot( jt2a.v ) + jt2a.w.dot( tmp1 ) + b2.minv * jt2b.v.dot( jt2b.v ) + jt2b.w.dot( tmp2 );
	}
	
	/**
	 * Returns Jdv values for given component.
	 * @param computeInCollection
	 * @param index (0 for normal, 1 for tangent1, 2 for tangent2)
	 * TODO: Flow control looks like it could be better here... :/
	 */
	public double getJdv(boolean computeInCollection, int index) {
		Vector6d dv1 = (body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
		Vector6d dv2 = (body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
		
		Vector6d ja = jna;
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
    private static final float[] colInCollection = new float[] { 0, 0, 1, 0.25f };
    private static final float[] colNew = new float[] { 0, 0.5f, 0, 0.65f };
    private static final float[] colOnEdge = new float[] { 0, 0, 0, 0.75f };
    private static final float[] colGraph = new float[] { 0, 0.2f, 0, 0.25f };
    private static final float[] colText = new float[] { 0,0,0, 0.9f };
    
    /**
     * Draws the contact points
     * @param drawable
     */
    public void display( GLAutoDrawable drawable, boolean isInCollection ) {
        GL2 gl = drawable.getGL().getGL2();
        float[] c = col;
        if (state == ContactState.ONEDGE) {
        	c = colOnEdge;
		} else {
			if ( newThisTimeStep ) {				
				c = colNew;
			} else if ( isInCollection ){
				c = colInCollection;
			}
		} 
        gl.glNormal3f(0, 0, 1);

		body1.transformB2W.transform(contactB1, contactInW);

		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, c, 0 );
        
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex3d( contactInW.x, contactInW.y, contactInW.z );
        gl.glEnd();

        gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, colText, 0 );
        gl.glRasterPos3d( contactInW.x, contactInW.y, contactInW.z );
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "  " + info );
    }

	/**
	 * Comptues contact forces for visualization purposes
	 * @param dt
	 */
	private void computeForces( double dt, Vector3d forceW1 ) {		
        body1.transformB2W.transform( normalB1, normalInW );
        body1.transformB2W.transform( tangent1B1, tangent1InW );
        body1.transformB2W.transform( tangent2B1, tangent2InW );
        forceW1.scale( lambda0, normalInW );
		forceW1.scaleAdd( lambda1, tangent1InW, forceW1 );
		forceW1.scaleAdd( lambda2, tangent2InW, forceW1 );		
		forceW1.scale(1./dt);
        
		//computeJacobian(true);
//		forceW1.scale( lambda0, jna.v );
//		forceW1.scaleAdd( lambda1, jt1a.v, forceW1 );
//		forceW1.scaleAdd( lambda2, jt2a.v, forceW1 );		
//		forceW1.scale(1./dt);
	}
	
	/**
	 * Draws the connections between bodies to visualize 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void displayContactGraph( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		if ( !body1.pinned && ! body2.pinned ) {
			
			float[] c = colGraph;
			//gl.glLineWidth(2);
			gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, c, 0 );
			gl.glBegin( GL.GL_LINES );
			
			gl.glVertex3d(body1.x.x, body1.x.y, body1.x.z);
			gl.glVertex3d(body2.x.x, body2.x.y, body2.x.z);
			gl.glEnd();
		}
	}
	
	/**
	 * Draws the contact force
	 * @param drawable
	 * @param isInCollection  signals that this is an internal contact and should be drawn differently using the appropraite jacobian
	 * @param dt
	 */
	public void displayContactForce( GLAutoDrawable drawable, boolean isInCollection, double dt ) {
		GL2 gl = drawable.getGL().getGL2();
		computeForces(dt, forceW1); // This might seem wasteful (e.g., if sim not running), but only used for debug visualization!
		
        float[] c = col;
        if (state == ContactState.ONEDGE) {
        	c = colOnEdge;
		} else {
			if ( newThisTimeStep ) {	
				c = colNew;
			} else if ( isInCollection ){
				c = colInCollection;
			}
		} 
		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, c, 0 );

		gl.glBegin( GL.GL_LINES );
		double scale = forceVizScale.getValue();
		body1.transformB2W.transform(contactB1, contactInW);
		gl.glVertex3d(contactInW.x + scale*forceW1.x, contactInW.y+scale*forceW1.y, contactInW.z+scale*forceW1.z );
		gl.glVertex3d(contactInW.x + -scale*forceW1.x, contactInW.y+-scale*forceW1.y, contactInW.z+-scale*forceW1.z );		
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
    	int csb1h = hashcode(csb1);
    	int csb2h = hashcode(csb2);
    	// hashing primes from https://planetmath.org/goodhashtableprimes
    	if ( b1h < b2h ) {
    		return 53 * bv1h + 97 * bv2h + 193 * b1h + 389 * b2h + info + 769 * csb1h + 1543 * csb2h; 
    	}
        return 53 * bv2h + 97 * bv1h + 193 * b2h + 389 * b1h + info + 769 * csb2h + 1543 * csb1h; 
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
        return body1 == c.body1 && bv1 == c.bv1 && body2 == c.body2 && bv2 == c.bv2 && csb1 == c.csb1 && csb2 == c.csb2 ||
        		body2 == c.body1 && bv2 == c.bv1 && body1 == c.body2 && bv1 == c.bv2 && csb1 == c.csb2 && csb2 == c.csb1;    
    }

    @Override
    public String toString() {
    	return body1.name + " " + body2.name + " objhash=" + super.hashCode() + " " + super.toString();
    }
}
