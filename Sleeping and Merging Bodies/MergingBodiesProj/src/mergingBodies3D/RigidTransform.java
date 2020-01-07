package mergingBodies3D;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import mintools.viewer.FlatMatrix4d;

/**
 * 3D rigid transformation
 * @author kry
 */
public class RigidTransform {
	
	// TODO: Really want to have this backed by a rotation matrix and translation vector...
	// Then can implement our own inverse and adjoint transformations??
	
	/** 
	 * Flat matrix wrapper for transformation matrix to use with opengl 
	 * Alternatively, we could have a toarray in this class if ever we 
	 * wanted to split the transform into rotational and translation parts...
	 * but they are already split into parts R and x in the rigid body.
	 */
	FlatMatrix4d Tflat = new FlatMatrix4d();

    /** homogeneous representation of 3D transformation */	
	Matrix4d T;
    
    /** 
     * Creates a new identity transformation 
     */
    public RigidTransform() {
    	T = Tflat.getBackingMatrix();
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
    	T.invert();
         // TODO: fix me
//        double x = T.m03; // extract translation t
//        double y = T.m13;
//        double z = T.m23;
//        T.transpose(); // invert rotational part
//        T.m30 = 0; // zero out translation that was put into last row
//        T.m31 = 0;
//        T.m32 = 0;
//        T.m02 = - T.m00*x - T.m01*y - T.m02*z; // compute new translation -R^T t 
//        T.m12 = - T.m10*x - T.m11*y - T.m12*z;
//        T.m22 = - T.m20*x - T.m21*y - T.m22*z;
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
    
    /**
     * TODO: Consider making rigid transformations friendly to composition while staying efficient 
     * Sets this transform to A * this
     * @param A
     */
    public void leftMult( RigidTransform A ) {
    	T.mul(A.T, this.T);
    }

//    /**
//     * Transforms the given point by the inverse of this 
//     * @param p
//     * @param result
//     */
//    public void inverseTransform( Point3d p, Point3d result ) {
//    	// R^T p - R^T t
//    	
//    }
    
}
