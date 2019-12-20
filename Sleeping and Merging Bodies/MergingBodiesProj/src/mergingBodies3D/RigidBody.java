package mergingBodies3D;

import java.util.ArrayList;
import java.util.HashMap;

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
    
    /** Block approximation of geometry */
    ArrayList<Block> blocks;
    
    /** Boundary blocks */
    ArrayList<Block> boundaryBlocks;
        
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
    
    /** TODO: should this even live here?  perhaps this should only be in the mouse spring */
    //public Point3d selectedPoint = new Point3d();
    
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
    
    /** angular velocity in radians per second */
    public Vector3d omega = new Vector3d();

    /** inverse of the linear mass, or zero if pinned */
    double minv;
    
    /** inverse of the angular mass, or zero if pinned, for current pose */
    Matrix3d jinv = new Matrix3d();
    /** inverse of the angular mass, or zero if pinned, for REST pose */
    Matrix3d jinv0 = new Matrix3d();
    
	/** bounding box, in the body frame */
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
	 * Default empty constructor
	 */
	public RigidBody() {
	}
	
    /**
     * Creates a new rigid body from a collection of blocks
     * @param blocks
     * @param boundaryBlocks
     */
    public RigidBody( ArrayList<Block> blocks, ArrayList<Block> boundaryBlocks ) {

        this.blocks = blocks;
        this.boundaryBlocks = boundaryBlocks;   
		Point3d bbmaxB = new Point3d(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
		Point3d bbminB = new Point3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);     
        // compute the mass and center of mass position   
        for ( Block b : blocks ) {
            double mass = b.getColourMass();
            massLinear += mass;            
            x0.x += b.j * mass;
			bbmaxB.x = Math.max(bbmaxB.x, b.j + Block.h);
			bbminB.x = Math.min(bbminB.x, b.j - Block.h);
            x0.y += b.i * mass; 
			bbmaxB.y = Math.max(bbmaxB.y, b.i + Block.h);
			bbminB.y = Math.min(bbminB.y, b.i - Block.h);
            x0.z += b.k * mass;
			bbmaxB.z = Math.max(bbmaxB.z, b.k + Block.h);
			bbminB.z = Math.min(bbminB.z, b.k - Block.h);
        }

        x0.scale ( 1 / massLinear );
        // set block positions in world and body coordinates 
        for ( Block b : blocks ) {
            b.pB.x = b.j - x0.x;
            b.pB.y = b.i - x0.y;
            b.pB.z = b.k - x0.z;
        }
        
        x.set(x0);      
        theta.setIdentity();
        transformB2W.set( theta, x );
        transformW2B.set( theta, x );
        transformW2B.invert();		
        
        transformW2B.transform(bbmaxB);
		transformW2B.transform(bbminB);
		boundingBoxB.add(bbmaxB);
		boundingBoxB.add(new Point3d(bbminB.x,bbmaxB.y,bbmaxB.z));
		boundingBoxB.add(new Point3d(bbmaxB.x,bbminB.y,bbmaxB.z));
		boundingBoxB.add(new Point3d(bbmaxB.x,bbmaxB.y,bbminB.z));
		boundingBoxB.add(bbminB);
		boundingBoxB.add(new Point3d(bbmaxB.x,bbminB.y,bbminB.z));
		boundingBoxB.add(new Point3d(bbminB.x,bbmaxB.y,bbminB.z));
		boundingBoxB.add(new Point3d(bbminB.x,bbminB.y,bbmaxB.z));
        
//        // Approximate computation of the rotational inertia
//        double l=bbmaxB.x-bbminB.x; 
//        double w=bbmaxB.y-bbminB.y;  
//        double h=bbmaxB.z-bbminB.z; 
//        massAngular0.setZero();
//        massAngular0.m00 = massLinear/12.*(w*w+h*h);
//        massAngular0.m11 = massLinear/12.*(l*l+h*h);
//        massAngular0.m22 = massLinear/12.*(w*w+l*l);
        
        // prevent zero angular inertia in the case of a single block
        if ( blocks.size() == 1 ) {
            Block b = blocks.get(0);
            double mass = b.getColourMass();
            massAngular0.m00 = mass * (1+1)/12;
            massAngular0.m11 = mass * (1+1)/12;
            massAngular0.m22 = mass * (1+1)/12;
        } else {
        	// always prevent a zero angular inertia in 
        	// whatever the smallest axis, if this is a
        	// single row or column of blocks...
        	massAngular0.m00 = (1+1)/12.0;
            massAngular0.m11 = (1+1)/12.0;
            massAngular0.m22 = (1+1)/12.0;
        	for ( Block b : blocks ) {
                double mass = b.getColourMass();
        		double x = b.pB.x;
        		double y = b.pB.y;
        		double z = b.pB.z;
           		massAngular0.m00 += mass * ( y*y + z*z );
        		massAngular0.m11 += mass * ( x*x + z*z );
        		massAngular0.m22 += mass * ( x*x + y*y );        	
        		massAngular0.m01 += mass * x*y;
        		massAngular0.m10 += mass * x*y;        		
        		massAngular0.m02 += mass * x*z;
        		massAngular0.m20 += mass * x*z;        		
        		massAngular0.m12 += mass * y*z;
        		massAngular0.m21 += mass * y*z;
        	}
	    }
        jinv0.invert(massAngular0);
        massAngular.set( massAngular );
        jinv.set( jinv0 );
             
        root = new BVNode( boundaryBlocks, this );
        
        pinned = isAllBlueBlocks();
        
        if ( pinned ) {
            minv = 0;
            jinv.setZero();
        } else {
            minv = 1/massLinear;
            jinv.invert(massAngular0);
        }
    }
    
    /**
     * Creates a copy of the provided rigid body 
     * @param body
     */
    public RigidBody( RigidBody body ) {
        blocks = body.blocks;
        boundaryBlocks = body.boundaryBlocks;
        massLinear = body.massLinear;
        massAngular0 = body.massAngular0;
        x0.set( body.x0 );
        x.set( body.x );
        theta = body.theta;
        omega = body.omega;
		boundingBoxB = new ArrayList<Point3d>(body.boundingBoxB);
        // we can share the blocks and boundary blocks...
        // no need to update them as they are in the correct body coordinates already        
        updateTransformations();
        // We do need our own bounding volumes!  can't share!
        root = new BVNode( boundaryBlocks, this );        
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
        result.cross( tmp, omega );
        result.add( v );
    }
    
    /**
     * Checks if all blocks are shades of blue
     * @return true if all blue
     */
    boolean isAllBlueBlocks() {
        for ( Block b : blocks ) {
            if ( ! (b.c.x == b.c.y && b.c.x < b.c.z) ) return false;
        }
        return true;
    }

    /**
     * Checks to see if the point intersects the body in its current position
     * @param pW
     * @return true if intersection
     */
    public boolean intersect( Point3d pW ) {
        if ( root.boundingDisc.isInDisc( pW ) ) {
            Point3d pB = new Point3d();
            transformW2B.transform( pW, pB );
            for ( Block b : blocks ) {
                if ( b.pB.distanceSquared( pB ) < Block.radius * Block.radius ) return true;
            }
        }
        return false;
    }
    
    /**
     * Resets this rigid body to its initial position and zero velocity, recomputes transforms
     */
    public void reset() {
        x.set(x0);        
        theta.setIdentity();
        v.set(0,0,0);
        omega.set(0,0,0);
        transformB2W.set( theta, x );
        transformW2B.set( transformB2W );
        transformW2B.invert();
    }
    
    /** Map to keep track of display list IDs for drawing our rigid bodies efficiently */
    static private HashMap<ArrayList<Block>,Integer> mapBlocksToDisplayList = new HashMap<ArrayList<Block>,Integer>();
    
    /** display list ID for this rigid body */
    int myListID = -1;
    
    /**
     * Deletes all display lists.
     * This is called when clearing all rigid bodies from the simulation, or when display lists need to be updated due to 
     * changing transparency of the blocks.
     * @param gl
     */
    static public void clearDisplayLists( GL2 gl ) {
        for ( int id : mapBlocksToDisplayList.values() ) {
            gl.glDeleteLists(id, 1);
        }
        mapBlocksToDisplayList.clear();
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
        
        if ( myListID == -1 ) {
            Integer ID = mapBlocksToDisplayList.get(blocks);
            if ( ID == null ) {
                myListID = gl.glGenLists(1);
                gl.glNewList( myListID, GL2.GL_COMPILE_AND_EXECUTE );
                for ( Block b : blocks ) {
                    b.display( drawable );
                }
                gl.glEndList();
                mapBlocksToDisplayList.put( blocks, myListID );
            } else {
                myListID = ID;
                gl.glCallList(myListID);
            }
        } else {
            gl.glCallList(myListID);
        }
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