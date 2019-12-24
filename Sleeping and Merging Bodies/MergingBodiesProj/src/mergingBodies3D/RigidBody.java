package mergingBodies3D;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.EasyViewer;
import mintools.viewer.FlatMatrix4d;
import no.uib.cipr.matrix.DenseVector;

/**
 * Simple 2D rigid body based on image samples
 * @author kry
 */
public class RigidBody {
    
    /** Variable to keep track of identifiers that can be given to rigid bodies */
    static public int nextIndex = 0;
    
    RigidBodyGeom geom;
        
    BVNode root;
    
    /** accumulator for forces acting on this body */
    Vector3d force = new Vector3d();
    
    /** accumulator for torques acting on this body */
    Vector3d torque = new Vector3d();
    
    /** rotational inertia in rest pose with rotation equal to identity */
    private Matrix3d massAngular0 = new Matrix3d();
    /** rotational inertia in the current pose */
    private Matrix3d massAngular = new Matrix3d();
    
    double massLinear;
        
    public boolean pinned;
    
    public boolean selected;
    
    /** Transforms points in Body coordinates to World coordinates */
    RigidTransform transformB2W = new RigidTransform();
    
    /** Transforms points in World coordinates to Body coordinates */
    RigidTransform transformW2B = new RigidTransform();
    
    /** DeltaV for PGS resolution */
    DenseVector deltaV = new DenseVector(6);
    
    /** linear velocity */
    public Vector3d v = new Vector3d();
    
    /** Position of center of mass in the world frame */
    public Point3d x = new Point3d();
    
    /** initial position of center of mass in the world frame */
    Point3d x0 = new Point3d();
    
    /** orientation of the body TODO: refactor to be R later?? */
    public Matrix3d theta = new Matrix3d();

    /** orientation of the body TODO: refactor to be R later?? */
    public Matrix3d theta0 = new Matrix3d();
    
    /** angular velocity in radians per second */
    public Vector3d omega = new Vector3d();

    /** inverse of the linear mass, or zero if pinned */
    double minv;
    
    /** inverse of the angular mass, or zero if pinned, for current pose */
    Matrix3d jinv = new Matrix3d();
    /** inverse of the angular mass, or zero if pinned, for REST pose */
    Matrix3d jinv0 = new Matrix3d();
    
	/** 
	 * bounding box, in the body frame (all 8 points)
	 * TODO: the purpose of this is to bound maximum velocity relative to another
	 * body so we might want to do something specific or different for special geometry.
	 */
	public ArrayList<Point3d> boundingBoxB = new ArrayList<Point3d>(); 
	
	/** Friction coefficient */
	public double friction; 
	
	/** Restitution coefficient */
	public double restitution; 
	
	/** true if body is a magnetic */
	public boolean magnetic = false;
	
	/** true if magnetic field is activate */
	public boolean activateMagnet = false;
    
	/**
	 * Constructs a new rigid body
	 * @param massLinear  can be zero if pinned
	 * @param massAngular can be null if pinned
	 * @param pinned
	 * @param boundingBoxB can be null if pinned
	 */
	public RigidBody( double massLinear, Matrix3d massAngular, boolean pinned, ArrayList<Point3d> boundingBoxB ) {
		this.pinned = pinned;
        if ( pinned ) {
        	this.massLinear = 0;
        	this.minv = 0;
        	this.massAngular.setZero();
        	this.massAngular0.setZero();
        	this.jinv0.setZero();
        	this.jinv.setZero();        
        } else {
        	this.boundingBoxB.addAll( boundingBoxB );
        	this.massLinear = massLinear;
        	this.minv = 1.0 / massLinear;
            this.massAngular0.set( massAngular );
            this.massAngular.set( massAngular );            
        	this.jinv0.invert(massAngular0);
	        this.jinv.set( jinv0 );
        } 
        theta.setIdentity();
        theta0.setIdentity();
	}
	
    /**
     * Creates a copy of the provided rigid body 
     * @param body
     */
    public RigidBody( RigidBody body ) {
        massLinear = body.massLinear;
        massAngular0.set( body.massAngular0 );
        x0.set( body.x0 );
        x.set( body.x );
        theta.set( body.theta );
        theta0.set( body.theta0 );        
        omega.set( body.omega );
		boundingBoxB = new ArrayList<Point3d>(body.boundingBoxB);
        // we can share the blocks and boundary blocks...
        // no need to update them as they are in the correct body coordinates already        
        updateTransformations();
        root = new BVNode( body.root, this ); // create a copy
        pinned = body.pinned;
        minv = body.minv;
        jinv = body.jinv;
    }
    
	/**
	 * Clear deltaV, force and torque
	 */
	public void clear() {
		force.set(0., 0., 0.);
		torque.set(0., 0., 0.);
		deltaV.zero();
	}
    
    /**
     * Updates the B2W and W2B transformations
     */
    public void updateTransformations() {
        transformB2W.set( theta, x );
        transformW2B.set( theta, x );
        transformW2B.invert();
        // might be done more often than necessary, but need to have 
        // rotational inertia updated give we are storing information in a 
        // world aligned frame
        if ( ! pinned ) {
	        jinv.mul( theta, jinv0 );
	        Matrix3d RT = new Matrix3d();
	        RT.transpose(theta);
	        jinv.mul( RT );
	        massAngular.invert(jinv); // otherwise could compute by composition.
        } 
    }
    
    /**
     * Apply a contact force specified in world coordinates
     * @param contactPointW
     * @param contactForceW
     */
    public void applyContactForceW( Point3d contactPointW, Vector3d contactForceW ) {
        force.add( contactForceW );
        
        Vector3d r = new Vector3d();
        r.sub( contactPointW, x );
        Vector3d torqueW = new Vector3d();
        torqueW.cross( r,  contactForceW );
        torque.add( torqueW );
    }
    
    /**
     * Adds to the torque a corriolis term (J omega) * omegacross
     * TODO: Check that this is correct!! And make more efficient?
     */
    public void applyCoriollisTorque() {
    	if ( !pinned ) {
	    	Vector3d tmp = new Vector3d();
	    	Vector3d tmp2 = new Vector3d();
	    	massAngular.transform( omega, tmp );
	    	tmp2.cross( tmp, omega );
	    	//torque.sub(tmp2); // seems like this should be added, but could also be a sign error. :(
    	}
    }
    
    /**
     * Advances the body state using symplectic Euler, first integrating accumulated force and torque 
     * (which are then set to zero), and then updating position and angle.  The internal rigid transforms
     * are also updated. 
     * @param dt step size
     */
    public void advanceTime( double dt ) {
        if ( !pinned ) {   
			advanceVelocities(dt);
			advancePositions(dt);
        }        
    }
    
	/**
	 * Given normalized R^3 vector of rotation w, we compute exp([w]t) using
	 * Rodrigues' formula:
	 * 
	 * exp([w]t) = I + [w] sin(t) + [w](1-cos(t)).
	 * 
	 * @param R :=
	 *            exp([w]t)
	 * @param w
	 *            Normalized 3D vector.
	 * @param t
	 *            Step size (in radians).
	 */
	private static void expRodrigues(Matrix3d R, Vector3d w, double t) {
		double wX = w.x;
		double wY = w.y;
		double wZ = w.z;
		double c = Math.cos(t);
		double s = Math.sin(t);
		double c1 = 1 - c;

		R.m00 = c + wX * wX * c1;
		R.m10 = wZ * s + wX * wY * c1;
		R.m20 = -wY * s + wX * wZ * c1;

		R.m01 = -wZ * s + wX * wY * c1;
		R.m11 = c + wY * wY * c1;
		R.m21 = wX * s + wY * wZ * c1;

		R.m02 = wY * s + wX * wZ * c1;
		R.m12 = -wX * s + wY * wZ * c1;
		R.m22 = c + wZ * wZ * c1;
	}

	public void advanceVelocities(double dt) {
		v.x += force.x * dt * minv + deltaV.get(0);
		v.y += force.y * dt * minv + deltaV.get(1);
		v.z += force.z * dt * minv + deltaV.get(2);

    	Vector3d domega = new Vector3d();
        jinv.transform( torque, domega );
        domega.scale( dt );
        omega.add( domega );
        
        omega.x += deltaV.get(3);
        omega.y += deltaV.get(4);
        omega.z += deltaV.get(5);
	}
	
	public void advancePositions(double dt) {
		x.x += v.x * dt;
		x.y += v.y * dt;
		x.z += v.z * dt;
		
		double t = omega.length()*dt;
    	Vector3d domega = new Vector3d();
        domega.set(omega);
        domega.normalize();
        if ( t > 1e-8 ) {
            Matrix3d dR = new Matrix3d();
        	expRodrigues(dR, domega, t);
        	dR.mul( theta );                
        	theta.normalizeCP( dR ); // keep it clean!
        }
		
        updateTransformations();
	}	

    /**
     * Computes the total kinetic energy of the body.
     * @return the total kinetic energy
     */
    public double getKineticEnergy() {
    	Vector3d result = new Vector3d();
    	massAngular0.transform(omega,result);
        return 0.5 * massLinear * v.lengthSquared() + 0.5 * result.dot( omega ); 
    }
    
    /** 
     * Computes the velocity of the provided point provided in world coordinates due
     * to motion of this body.   
     * @param contactPointW
     * @param result the velocity
     */
    public void getSpatialVelocity( Point3d contactPointW, Vector3d result ) {
        Vector3d tmp = new Vector3d();
    	tmp.sub( contactPointW, x );
        result.cross( omega, tmp );
        result.add( v );
    }
        
    /**
     * Resets this rigid body to its initial position and zero velocity, recomputes transforms
     */
    public void reset() {
        x.set(x0);        
        theta.set(theta0);
        v.set(0,0,0);
        omega.set(0,0,0);
        transformB2W.set( theta, x );
        transformW2B.set( transformB2W );
        transformW2B.invert();
    }
   
    /** 
     * Draws the blocks of a rigid body
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
    	
        GL2 gl = drawable.getGL().getGL2();
     
        gl.glPushMatrix();        
        FlatMatrix4d M = new FlatMatrix4d();
        M.setBackingMatrix( transformB2W.T );        
        gl.glMultMatrixd( M.asArray(),0 );

        geom.display( drawable );
    	
        gl.glPopMatrix();
    }
    
    static public double kineticEnergyThreshold = 1e-6;
    
    /**
     * Draws the center of mass position with a circle.  The circle will be 
     * drawn differently if the block is at rest (i.e., close to zero kinetic energy)
     * @param drawable
     */
    public void displayCOM( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        
        gl.glPushMatrix();
        gl.glTranslated(x.x, x.y, x.z);
        EasyViewer.glut.glutSolidSphere(1, 10, 10);
        gl.glPopMatrix();
    }
    
}