package mergingBodies3D;

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
    
    /** Position of contact point in body coordinates, used too draw contact in collections */
    private Point3d contactB = new Point3d();
    
    /** Contact normal in world coordinates 
     * ONLY PUBLIC FOR TESTING... CAN BE MADE PRIVATE */
	public Vector3d normalW = new Vector3d();
    
    /** Contact tangent1 in world coordinates */
    private Vector3d tangent1W = new Vector3d();
    
    /** Contact tangent2 in world coordinates */
    private Vector3d tangent2W = new Vector3d();
      
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
	 * Copy constructor, for convenience for now, though this does not exploit our memory pooling efforts, 
	 * it might not matter as contacts created in this way happen in longer lived merged collections...
	 * These could still be pooled if necessary.
	 * @param contact
	 */
	public Contact(Contact contact) {
		body1 = contact.body1;
		body2 = contact.body2;
		bv1 = contact.bv1;
		bv2 = contact.bv2;
		info = contact.info;
		
		contactW.set(contact.contactW);
		contactB.set(contact.contactB);
		normalW.set(contact.normalW);   	// CRAP...  will we ever need these in a body frame for the "other" jacobian??  hmm... 	
		tangent1W.set(contact.tangent1W);  // TODO: figure out if the body frame vectors are really needed or not.  I feel the answer is yes. :(
		tangent2W.set(contact.tangent2W);  // but perhaps we can get by with ONLY those... 
											// jacobian can be built directly rather than storing the world ones...  ?
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
     * @param body1
     * @param body2
     * @param contactW	in world coordinates
     * @param normal	in world coordinates
     */
    public void set( RigidBody body1, RigidBody body2, Point3d contactW, Vector3d normal, BVSphere disc1, BVSphere disc2, int info, double constraintViolation ) {    	
    	this.body1 = body1;
        this.body2 = body2;
		this.info = info;
        if ( this.body1.isInComposite() ) { // use the real body if this is a sub body in a composite
        	this.body1 = body1.compositeBodyParent;
        }
        if ( this.body2.isInComposite() ) { // same for body 2
        	this.body2 = body2.compositeBodyParent;
        }
        
        // modify the info to include information about the sub body...  Note the ID will be zero by default
        // note we assume less than 0xffff contaacts between two primitives
        // and we assume that no composite body will be made of more than 256 bodies (otherwise it is expensive!)
        // Perhaps this should be done down in the hash?
        // perhaps we should actually just maintain more information type fields in the contact?
        // contacts are already pretty big so this wouldn't really make a big deal...   
        if ( hashcode( this.body1 ) < hashcode( this.body2) ) {        	
	    	this.info += body1.subBodyID << 16; // put body2 subbody ID in highest byte
	        this.info += body2.subBodyID << 24; // put body1 subbody ID in 3rd byte
        } else {
        	this.info += body2.subBodyID << 16; // put body2 subbody ID in highest byte
	        this.info += body1.subBodyID << 24; // put body1 subbody ID in 3rd byte
        }
        
        // TODO: last thing to fix here (Before writing the collision processing) is the info
        // Perhaps a body can know its index in the composite body list, and we can add 100 * id to the info to
        // make sure that different sub bodies with the same info do not collide wrt the warm start.
        
        this.contactW.set( contactW ); 
        body1.transformB2W.inverseTransform( contactW, contactB );
//        body1.transformW2B.transform(contactW, contactB);
		bv1 = disc1;
		bv2 = disc2;
		this.constraintViolation =  constraintViolation;     
        
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

        computeJacobian(false); // the boolean doesn't matter here
    }
    
    /**
	 * Computes the Jacobian matrix of the contact.
	 * In case of body in a collection, use COM of parent to compute the torque component of the Jacobian.
	 */
	public void computeJacobian(boolean computeInCollection) {
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
		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, c, 0 );
        
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex3d( contactW.x, contactW.y, contactW.z );
        gl.glEnd();
    }
    
    
	/** temporary Contact force being applied by this contact on body1 (note this is world aligned at body COM)... only used for display */
    static private Vector3d forceW1 = new Vector3d();

	/**
	 * Comptues contact forces for visualization purposes
	 * @param dt
	 */
	private void computeForces( double dt, Vector3d forceW1 ) {		
		forceW1.scale( lambda0, jna.v );
		forceW1.scaleAdd( lambda1, jt1a.v, forceW1 );
		forceW1.scaleAdd( lambda2, jt2a.v, forceW1 );		
		forceW1.scale(1./dt);
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
				gl.glLineWidth(4f);
				c = colNew;
			} else if ( isInCollection ){
				c = colInCollection;
			}
		} 
		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, c, 0 );

		gl.glBegin( GL.GL_LINES );
		double scale = forceVizScale.getValue();
		body1.transformB2W.transform(contactB, contactW);
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

    @Override
    public String toString() {
    	return body1.name + " " + body2.name + " objhash=" + super.hashCode() + " " + super.toString();
    }
}
