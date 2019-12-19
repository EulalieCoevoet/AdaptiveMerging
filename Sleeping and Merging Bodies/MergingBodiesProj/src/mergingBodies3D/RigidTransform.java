package mergingBodies3D;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

/**
 * 2D rigid transformation
 * @author kry
 */
public class RigidTransform {

    /** homogeneous representation of 3D transformation */
    Matrix4d T = new Matrix4d();
    
    /** 
     * Creates a new identity transformation 
     */
    public RigidTransform() {
        T.setIdentity();
    }
    
    /**
     * Sets this rigid transformation from another rigid transformation
     * @param R
     */
    public void set( RigidTransform R ) {
        T.set(R.T);
    }
    
    /**
     * Sets this transformation to be the given rotation and translation
     * (i.e., points will be transformed by rotation followed by translation)
     * @param theta
     * @param p
     */    
    public void set( Matrix3d R, Tuple3d p ) {
    	T.set( R );
    	T.m03 = p.x;
    	T.m13 = p.y;
    	T.m23 = p.z;
    }
    
    /**
     * Inverts this transformation
     */
    public void invert() {
        // gross but convenient... 
        T.invert();
    }
    
    /**
     * Transforms the given point
     * @param p
     */
    public void transform( Point3d p ) {
    	T.transform( p );
    }
 
    /**
     * Transforms the given vector
     * @param v
     */
    public void transform( Vector3d v ) {
    	T.transform( v );
    }

    /**
     * Transforms the given point
     * @param p
     * @param result
     */
    public void transform( Point3d p, Point3d result ) {
    	T.transform( p, result );
    }
 
    /**
     * Transforms the given vector
     * @param p
     * @param result
     */
    public void transform( Vector3d v, Vector3d result ) {
    	T.transform( v, result);
    }

}
