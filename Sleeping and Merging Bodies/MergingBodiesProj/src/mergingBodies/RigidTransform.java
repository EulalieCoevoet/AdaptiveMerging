package mergingBodies;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

/**
 * 2D rigid transformation
 * @author kry
 */
public class RigidTransform {

    /** homogeneous representation of 2D transformation */
    Matrix3d T = new Matrix3d();

    
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
    public void set( double theta, Tuple2d p ) {
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        T.m00 = c; T.m01 = -s; T.m02 = p.x;
        T.m10 = s; T.m11 =  c; T.m12 = p.y;
        T.m20 = 0; T.m21 =  0; T.m22 = 1;
    }
    
    /*
     * returns angle from transformation in radians
     */
    public double getTheta() {
    	return Math.atan2(T.m10, T.m00);
    }
    
    /*
     * 
     */
    public void getTranslation(Tuple2d x) {
    		x.set(T.m02, T.m12);
    }

    /**
     * Sets this transform to A * this
     * @param A
     */
    public void leftMult( RigidTransform A ) {
    	T.mul(A.T, this.T);
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
    public void transform( Point2d p ) {
        double x = T.m00 * p.x + T.m01 * p.y + T.m02;
        double y = T.m10 * p.x + T.m11 * p.y + T.m12;
        p.x = x;
        p.y = y;
    }
 
    /**
     * Transforms the given vector
     * @param p
     */
    public void transform( Vector2d p ) {
        double x = T.m00 * p.x + T.m01 * p.y;
        double y = T.m10 * p.x + T.m11 * p.y;
        p.x = x;
        p.y = y;
    }

    /**
     * Transforms the given point
     * @param p
     * @param result
     */
    public void transform( Point2d p, Point2d result ) {
        result.x = T.m00 * p.x + T.m01 * p.y + T.m02;
        result.y = T.m10 * p.x + T.m11 * p.y + T.m12;
    }
 
    /**
     * Transforms the given vector
     * @param p
     * @param result
     */
    public void transform( Vector2d p, Vector2d result ) {
        result.x = T.m00 * p.x + T.m01 * p.y;
        result.y = T.m10 * p.x + T.m11 * p.y;
    }
}
