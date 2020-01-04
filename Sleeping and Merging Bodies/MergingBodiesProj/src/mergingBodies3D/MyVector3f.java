package mergingBodies3D;

import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Tuple3d;

/**
 * MyVector3f a Vector3f class that implement several methods missing in
 * javac.vecmath.Vector3f
 * 
 * This come's from ståle pedersen's provided code
 * http://folk.uio.no/staalep/hfag/source.shtml
 */
final public class MyVector3f extends Vector3f {

  /**
   * Constructs and initializes a MyVector3f to (0,0,0)
   */
  public MyVector3f() {
    super();
  }
  /**
   * Constructs and initializes a MyVector3f from the array of length 3
   * @param v - the array of length 3 containing xyz in order
   */
  public MyVector3f(float[] v) {
    super(v);
  }
  /**
   * Constructs and initializes a MyVector3f from the specified xyz coordinates
   * @param x - the x coordinate
   * @param y - the y coordinate
   * @param z - the z coordinate
   */
  public MyVector3f(float x, float y, float z) {
    super(x, y, z);
  }
  /**
   * Constructs and initializes a MyVector3f from the specified Vector3f
   * @param v - the MyVector3f containing the initialization x y z data
   */
  public MyVector3f(Vector3f v) {
    super(v);
  }
  /**
   * Constructs and initializes a MyVector3f from the specified Vector3d
   * @param v - the MyVector3d containing the initialization x y z data
   */
  public MyVector3f(Vector3d v) {
    super(v);
  }
  /**
   * Constructs and initializes a MyVector3f from the specified Tuple3f
   * @param v - the Tuple3f containing the initialization x y z data
   */
  public MyVector3f(Tuple3f v) {
    super(v);
  }
  /**
   * Constructs and initializes a MyVector3f from the specified Tuple3d
   * @param v - the Tuple3d containing the initialization x y z data
   */
  public MyVector3f(Tuple3d v) {
    super(v);
  }

 
  public float get(int i) {
    if(i == 0)
      return x;
    else if(i == 1)
      return y;
    else
      return z;
  }

  public void set(int i, float value) {
    if(i == 0)
      x = value;
    else if(i == 1)
      y = value;
    else
      z = value;
  }

  public void add(Tuple3f t1, Tuple3f t2, Tuple3f t3) {
    x = t1.x + t2.x + t3.x;
    y = t1.y + t2.y + t3.y;
    z = t1.z + t2.z + t3.z;
  }

  /**
   * Set the value of this Vector to the vector multiplication of Matrix3f m and itself.
   * @param m the matrix that is multiplied with the vector
   */
  final public void mul(MyMatrix3f m) {
    float x1 = x*m.m00 + y*m.m10 + z*m.m20;
    float y1 = x*m.m01 + y*m.m11 + z*m.m21;
    z = x*m.m02 + y*m.m12 + z*m.m22;
    x = x1;
    y = y1;
  }

  /**
   * Return the Vector sum of the vector multiplication of Matrix3f m and itself.
   * @param m the matrix that is multiplied with the vector
   */
  final public Vector3f mulNewVector(Matrix3f m) {
    return new Vector3f(x*m.m00 + y*m.m10 + z*m.m20,
			x*m.m01 + y*m.m11 + z*m.m21,
			x*m.m02 + y*m.m12 + z*m.m22);
  }

  /**
   * Set the vector to the product of the vector multiplication of Matrix3f m 
   * and Vector v
   * @param m matrix
   * @param v vector
   */
  public final void mul(MyMatrix3f m, MyVector3f v) {
    if(this != v) {
      x = m.m00*v.x + m.m01*v.y + m.m02*v.z;
      y = m.m10*v.x + m.m11*v.y + m.m12*v.z;
      z = m.m20*v.x + m.m21*v.y + m.m22*v.z;
    }
    else {
      float x1 = m.m00*v.x + m.m01*v.y + m.m02*v.z;
      float y1 = m.m10*v.x + m.m11*v.y + m.m12*v.z;
      z = m.m20*v.x + m.m21*v.y + m.m22*v.z;
      x = x1; y = y1;

    }
  }

  /**
   * Set the vector to the product of the vector multiplication of  
   * Vector v and Matrix3f m
   * @param v vector
   * @param m matrix
   */
  public final void mul(MyVector3f v, MyMatrix3f m) {
    if(this != v) {
      x = v.x*m.m00 + v.y*m.m10 + v.z*m.m20;
      y = v.x*m.m01 + v.y*m.m11 + v.z*m.m21;
      z = v.x*m.m02 + v.y*m.m12 + v.z*m.m22;
    }
    else {
      float x1 = v.x*m.m00 + v.y*m.m10 + v.z*m.m20;
      float y1 = v.x*m.m01 + v.y*m.m11 + v.z*m.m21;
      z = v.x*m.m02 + v.y*m.m12 + v.z*m.m22;
      x = x1; y = y1;
    }
  }
	
  /**
   * return the vector to the product of the vector multiplication of Matrix3f m 
   * and Vector v
   * @param m matrix
   * @param v vector
   */
  public final Vector3f mulNewVector(MyMatrix3f m, MyVector3f v) {
    return new Vector3f( m.m00*v.x + m.m01*v.y + m.m02*v.z, 
                     m.m10*v.x + m.m11*v.y + m.m12*v.z, 
		     m.m20*v.x + m.m21*v.y + m.m22*v.z);
  }
       
}

