package mergingBodies3D;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

/**
 * 3D rigid transformation
 * @author kry
 */
public class RigidTransform3D {

    private Matrix3d R;
    private Vector3d t;
    
    /** 
     * Creates a new identity transformation with provided backing rotation and translation
     * References to the provided matrix and vector are held by this object and can be changed
     * by called methods.  Likewise, this transformation can be changed by alterning the backing
     * matrices.
     * 
     * NOTE: R is assumed to be a rotation matrix!
     */
    public RigidTransform3D( Matrix3d R, Vector3d t ) {
    	this.R = R;
    	this.t = t;
    }
    
    /**
     * Sets a 16 element double array that can be provided to glMultMatrix or glLoadMatrix
     */
    public void getAsArray( double[] outputArray) {
        outputArray[0] = R.m00;
        outputArray[1] = R.m10;
        outputArray[2] = R.m20;
        outputArray[3] = 0;
        outputArray[4] = R.m01;
        outputArray[5] = R.m11;
        outputArray[6] = R.m21;
        outputArray[7] = 0;
        outputArray[8] = R.m02;
        outputArray[9] = R.m12;
        outputArray[10] = R.m22;
        outputArray[11] = 0;
        outputArray[12] = t.x;
        outputArray[13] = t.y;
        outputArray[14] = t.z;
        outputArray[15] = 1;
    }

    public void setIdentity() {
    	R.setIdentity(); 
    	t.set(0,0,0);
    }
    
    /**
     * Sets this rigid transformation from another rigid transformation by copying the values
     * @param R
     */
    public void set( RigidTransform3D T ) {
        this.R.set( T.R );
        this.t.set( t );
    }
    
    /**
     * Sets this transformation to be the given rotation and translation by copying the values
     * @param theta
     * @param p
     */    
    public void set( Matrix3d R, Tuple3d t ) {
    	R.set( R );
    	t.set( t );
    }
        
    /**
     * Inverts this transformation
     */
    public void invert() {
        R.transpose(); 
        R.transform( t );
        t.scale(-1);
    }
    
    /**
     * Transforms the given point
     * @param p
     */
    public void transform( Point3d p ) {
    	R.transform( p );
    	p.add( t );
    }
    
    /**
     * Transforms the given point and stores the result in the provided point
     * @param p
     * @param result
     */
    public void transform( Point3d p, Point3d result ) {
    	R.transform( p, result );
    	result.add( t );
    }
 
    /**
     * Transforms the given vector
     * @param v
     */
    public void transform( Vector3d v ) {
    	R.transform( v );
    }
 
    /**
     * Transforms the given vector and stores the result in the provided vector
     * @param p
     * @param result
     */
    public void transform( Vector3d v, Vector3d result ) {
    	R.transform( v, result );
    }

    public void inverseTransform( Point3d p ) {
    	inverseTransform( p, p );
    }

    public void inverseTransform( Point3d p, Point3d result ) {
    	double x = p.x - t.x;
    	double y = p.y - t.y;
    	double z = p.z - t.z;
    	result.x = R.m00*x + R.m01*y + R.m02*z;  
        result.y = R.m10*x + R.m11*y + R.m12*z;
        result.z = R.m20*x + R.m21*y + R.m22*z;
    }
    
    public void inverseTransform( Vector3d v ) {
    	inverseTransform( v, v );
    }
    
    public void inverseTransform( Vector3d v, Vector3d result ) {
    	double x = v.x;
    	double y = v.y;
    	double z = v.z;
    	result.x = R.m00*x + R.m01*y + R.m02*z;  
        result.y = R.m10*x + R.m11*y + R.m12*z;
        result.z = R.m20*x + R.m21*y + R.m22*z;
    }

    /**
     * this = this * A
     * @param A
     */
    public void mult( RigidTransform3D A ) {
    	double x = t.x;
    	double y = t.y;
    	double z = t.z;
    	R.transform( A.t, t );
    	t.x += x;
    	t.y += y;
    	t.z += z;
    	R.mul( A.R );
    }
    
    /**
     * this = Ainv * B
     * @param A
     * @param B
     */
    public void multAinvB( RigidTransform3D A, RigidTransform3D B ) {
		//    	ART  -ART At     BR  Bt
		//        0     1     	 0   1
		//
    	//    ART BR    ART Bt - ART At
		//                  ART (Bt-At)
        t.sub( B.t, A.t );
        R.transpose( A.R );
    	R.transform( t );
    	R.mul( B.R );
    }
 
//    /**
//     * Uses the adjoint transform to change the coordinates of a given 6D vector 
//     * @param phi
//     * @param result
//     */
//    public void transform6DVelocity( Vector6d phi, Vector6d result ) {
//    	R.transform( phi.w, result.w );
//    	
//    }


//	/**
//	 * Uses the adjoint inverse transpose to change the coordinates of a given 6D vector
//	 * @param phi
//	 * @param Result
//	 */
//    public void transform6DWrench( Vector6d phi, Vector6d Result ) {
//    	
//    }
    
    // could also add inverse adjoint transforms to take velocities from world to body frame.
    
}
