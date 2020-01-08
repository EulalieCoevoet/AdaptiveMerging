package mergingBodies3D;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;

/**
 * MyMatrix3f implements several methods not supported in Matrix3f
 * 
 * This come's from ståle pedersen's provided code
 * http://folk.uio.no/staalep/hfag/source.shtml
 * 
 */
final public class MyMatrix3f extends Matrix3f {


  /**
   * Constructs and initializes a MyMatrix3f to all zeros
   */
  public MyMatrix3f() {
    super();
  }
  /**
   * Constructs and initializes a Matrix3f from the specified nine-element array
   * @param v - the array of length 9 containing in order
   */
  public MyMatrix3f(float[] v) {
    super(v);
  }
  /**
   * Constructs and initializes a Matrix3f from the specified nine values
   * @param m00 - the [0][0] element
   * @param m01 - the [0][1] element
   * @param m02 - the [0][2] element
   * @param m10 - the [1][0] element
   * @param m11 - the [1][1] element
   * @param m12 - the [1][2] element
   * @param m20 - the [2][0] element
   * @param m21 - the [2][1] element
   * @param m22 - the [2][2] element
   */
  public MyMatrix3f(float m00, float m01, float m02, 
		    float m10, float m11, float m12, 
		    float m20, float m21, float m22) {

    super(m00, m01, m02, 
	  m10, m11, m12, 
	  m20, m21, m22);
  }
  /**
   * Constructs a new matrix with the same values as the Matrix3f parameter
   */
  public MyMatrix3f(Matrix3f m) {
    super(m);
  }
  /**
   * Constructs a new matrix with the same values as the Matrix3d parameter
   */
  public MyMatrix3f(Matrix3d m) {
    super(m);
  }

  /**
   * Return the Vectorsum of the vector multiplication of itself and MyVector3f v
   * @param v - vector to multiplicate
   * @return MyVector3f
   */
  public final MyVector3f mul(MyVector3f v) {
    return new MyVector3f(m00*v.x + m01*v.y + m02*v.z,
			m10*v.x + m11*v.y + m12*v.z,
			m20*v.x + m21*v.y + m22*v.z);
  }

  /**
   * Metod to get eigenvector from matrix <br>
   * @see #tridiagonal
   * @see #qlAlgorithm
   */
  public void getEigenStuff() {
    MyVector3f diag = new MyVector3f(); 	// eigenvalue
    MyVector3f sub = new MyVector3f();
    MyMatrix3f mat = new MyMatrix3f(this);	// eigenvector

    tridiagonal(mat, diag, sub);
    qlAlgorithm(diag, sub, mat);

    System.out.println("diag=\n"+diag.toString());
    System.out.println("sub=\n"+sub.toString());
    System.out.println("mat=\n"+mat.toString());
  }

  /**
   * Make a tridiagonal vector of our matrix, this is used in solving the eigenvector
   * in qlAlgorithm <br>
   * This method is taken from David Eberly's MagicSoftware sources
   */
  private void tridiagonal(MyMatrix3f mat, MyVector3f diag, MyVector3f subd) {

    float fm00 = mat.m00;
    float fm01 = mat.m01;
    float fm02 = mat.m02;
    float fm11 = mat.m11;
    float fm12 = mat.m12;
    float fm22 = mat.m22;

    diag.x = fm00;
    subd.z = 0.0f;

    if(fm02 != 0.0f) {
      float length = (float) Math.sqrt(fm01*fm01 + fm02*fm02);
      float invLength = 1.0f/length;

      fm01 *= invLength;
      fm02 *= invLength;

      float fq = 2.0f * fm01 * fm12 + fm02*(fm22-fm11);
      diag.y = fm11+fm02*fq;
      diag.z = fm22-fm02*fq;
      subd.x = length;
      subd.y = fm12-fm01*fq;

      mat.m00 = 1.0f; mat.m01 = 0.0f; mat.m02 = 0.0f;
      mat.m10 = 0.0f; mat.m11 = fm01; mat.m12 = fm02;
      mat.m20 = 0.0f; mat.m21 = fm02; mat.m22 = -fm01;
    }

    else {
      diag.y = fm11;
      diag.z = fm22;
      subd.x = fm01;
      subd.y = fm12;

      mat.m00 = 1.0f; mat.m01 = 0.0f; mat.m02 = 0.0f;
      mat.m10 = 0.0f; mat.m11 = 1.0f; mat.m12 = 0.0f;
      mat.m20 = 0.0f; mat.m21 = 0.0f; mat.m22 = 1.0f;

    }
  }

  /**
   * Make a eigenvector to according matrix based on vectors createn in tridiagonal() <br>
   * This method is taken from David Eberly's MagicSoftware sources
   */
  private void qlAlgorithm(MyVector3f diag, MyVector3f subd, MyMatrix3f mat) {

    int maxIter = 32;
    int size = 3;

    for(int i0=0; i0 < size; i0++) {

      int i1;
      for(i1=0; i1 < maxIter; i1++) {

	int i2;
	for(i2=0; i2 <= size-2; i2++) {
	  float tmp = fabs(diag.get(i2)) + fabs(diag.get(i2+1));

	  if( fabs(subd.get(i2)) + tmp == tmp)
	    break;
	}
	if( i2 == i0)
	  break;

	float fg = (diag.get(i0+1) - diag.get(i0)) / (2.0f * subd.get(i0));
	float fr = (float) Math.sqrt(fg*fg+1.0f);
	if( fg < 0.0f)
	  fg = diag.get(i2) - diag.get(i0) + subd.get(i0) / (fg - fr);
	else
	  fg = diag.get(i2) - diag.get(i0) + subd.get(i0) / (fg + fr);

	float sin = 1.0f;
	float cos = 1.0f;
	float fp  = 0.0f;

	for(int i3 = i2-1; i3 >= i0; i3--) {
	  float ff = sin * subd.get(i3);
	  float fb = cos * subd.get(i3);
	  if( fabs(ff) >= fabs(fg)) {
	    cos = fg/ff;
	    fr = (float) Math.sqrt(cos*cos+1.0f);
	    subd.set(i3+1, ff*fr);
	    sin = 1.0f/fr;
	    cos *= sin;
	  }
	  else {
	    sin = ff/fg;
	    fr = (float) Math.sqrt(sin*sin+1.0f);
	    subd.set(i3+1, fg*fr);
	    cos = 1.0f/fr;
	    sin *= cos;
	  }
	  fg = diag.get(i3+1) - fp;
	  fr = (diag.get(i3)-fg)*sin + 2.0f*fb*cos;
	  fp = sin*fr;
	  diag.set(i3+1, fg+fp);
	  fg = cos*fr-fb;

	  for(int i4=0; i4 < size; i4++) {
	    ff = mat.getElement(i4, i3+1);
	    mat.setElement(i4, i3+1, sin*mat.getElement(i4, i3) + cos*ff);
	    mat.setElement(i4, i3, cos*mat.getElement(i4, i3) - sin*ff);
	  }
	}
	diag.set(i0, diag.get(i0) - fp);
	subd.set(i0, fg);
	subd.set(i2, 0.0f);
      }
      if(i1 == maxIter) {
	System.out.println("i1 == maxIter");
	return;
      }
    }
  }

  /**
   * A private little abs method, its faster than calling Math.abs, and we do 
   * that a lot in solving eigenvectors
   * @param x value to check
   * @return float absolute value
   */
  private final float fabs(float x) {
    return (x < 0 ? -x : x);
  }

  public void getEigen(MyMatrix3f cov) {
    MyVector3f evals = new MyVector3f();
    mEigen(cov, evals, this);
  }

  /**
   * Calculate the eigenvector from matrix
   * @param vout eigenvector
   * @param dout eigenvalue
   * @param a matrix
   */
  private final void mEigen(MyMatrix3f vout, MyVector3f dout, MyMatrix3f a) {

    int i;
    float tresh, theta, tau, t, sm, s, h, g, c;
    int nrot;
    MyVector3f b = new MyVector3f();
    MyVector3f z = new MyVector3f();
    MyVector3f d = new MyVector3f();
    MyMatrix3f v = new MyMatrix3f();

    v.setIdentity();

    b.x = a.getElement(0,0);
    b.y = a.getElement(1,1);
    b.z = a.getElement(2,2);

    d.x = a.getElement(0,0);
    d.y = a.getElement(1,1);
    d.z = a.getElement(2,2);


    nrot = 0;

    for(i=0; i < 50; i++) {
      sm = 0.0f;
      sm += fabs(a.m01);
      sm += fabs(a.m02);
      sm += fabs(a.m12);

      if(sm == 0.0f) {
	vout.set(v);
	dout.set(d);
	System.out.println("count = "+i);
	return;
      }

      if(i < 3)
	tresh = 0.2f*sm/(3.0f*3.0f);
      else
	tresh = 0.0f;

      {
	g = 100.0f*fabs(a.m01);
	if(i > 3 && (fabs(d.x) + g) == fabs(d.x) && (fabs(d.y) + g) == fabs(d.y))
	  a.m01 = 0.0f;
	else if(fabs(a.m01) > tresh) {
	  h = d.y - d.x;
	  if( (fabs(h)+g) == fabs(h))
	    t = (a.m01)/h;
	  else {
	    theta = 0.5f * h / a.m01;
	    t = 1.0f/(fabs(theta) + (float) Math.sqrt(1.0f+theta*theta));
	    if(theta < 0.0f)
	      t = -t;
	  }
	  c = (float) 1.0f/(float) Math.sqrt(1.0f+t*t);
	  s = t*c;
	  tau = s/(1.0f + c);
	  h = t * a.m01;
	  z.x -= h;
	  z.y += h;
	  d.x -= h;
	  d.y += h;
	  a.m01 = 0.0f;
	  // ROT(a, 0, 2, 1, 2)
	  g = a.getElement(0,2);
	  h = a.getElement(1,2);
	  a.setElement(0,2, g-s*(h+g*tau));
	  a.setElement(1,2, h+s*(g-h*tau));
	  // ROT(v, 0, 0, 0, 1)
	  g = v.getElement(0,0);		// i,j
	  h = v.getElement(0,1);		// k,l
	  v.setElement(0,0, g-s*(h+g*tau));	// i,j
	  v.setElement(0,1, h+s*(g-h*tau));	// k,l
	  // ROT(v, 1, 0, 1, 1)
	  g = v.getElement(1,0);		// i,j
	  h = v.getElement(1,1);		// k,l
	  v.setElement(1,0, g-s*(h+g*tau));	// i,j
	  v.setElement(1,1, h+s*(g-h*tau));	// k,l
	  // ROT(v, 2, 0, 2, 1)
	  g = v.getElement(2,0);		// i,j
	  h = v.getElement(2,1);		// k,l
	  v.setElement(2,0, g-s*(h+g*tau));	// i,j
	  v.setElement(2,1, h+s*(g-h*tau));	// k,l

	  nrot++;
	}
      }

      {
	g = 100.0f * fabs(a.m02);
	if( i > 3 && (fabs(d.x) + g) == fabs(d.x) && (fabs(d.z) + g) == fabs(d.z))
	  a.m02 = 0.0f;
	else if( fabs(a.m02) > tresh) {
	  h = d.z - d.x;
	  if( (fabs(h) + g) == fabs(h))
	    t = (a.m02) / h;
	  else {
	    theta = 0.5f * h / a.m02;
	    t = 1.0f / (fabs(theta) + (float) Math.sqrt(1.0f + theta*theta));
	    if(theta < 0.0f)
	      t = -t;
	  }
	  c = 1.0f / (float) Math.sqrt(1.0f + t*t);
	  s = t*c;
	  tau = s / (1.0f+c);
	  h = t * a.m02;

	  z.x -= h;
	  z.z += h;
	  d.x -= h;
	  d.z += h;
	  
	  a.m02 = 0.0f;
	  // ROT(a, 0, 1, 1, 2)
	  g = a.getElement(0,1);
	  h = a.getElement(1,2);
	  a.setElement(0,1, g-s*(h+g*tau));
	  a.setElement(1,2, h+s*(g-h*tau));
	  // ROT(v, 0, 0, 0, 2)
	  g = v.getElement(0,0);
	  h = v.getElement(0,2);
	  v.setElement(0,0, g-s*(h+g*tau));
	  v.setElement(0,2, h+s*(g-h*tau));
	  // ROT(v, 1, 0, 1, 2)
	  g = v.getElement(1,0);
	  h = v.getElement(1,2);
	  v.setElement(1,0, g-s*(h+g*tau));
	  v.setElement(1,2, h+s*(g-h*tau));
	  // ROT(v, 2, 0, 2, 2)
	  g = v.getElement(2,0);
	  h = v.getElement(2,2);
	  v.setElement(2,0, g-s*(h+g*tau));
	  v.setElement(2,2, h+s*(g-h*tau));

	  nrot++;
	}
      }


      {
	g = 100.0f * fabs(a.m12);
	if( i > 3 && (fabs(d.y) + g) == fabs(d.y) && (fabs(d.z) + g) == fabs(d.z))
	  a.m12 = 0.0f;
	else if( fabs(a.m12) > tresh) {
	  h = d.z - d.y;
	  if( (fabs(h) + g) == fabs(h))
	    t = (a.m12) / h;
	  else {
	    theta = 0.5f * h / a.m12;
	    t = 1.0f / (fabs(theta) + (float) Math.sqrt(1.0f + theta*theta));
	    if(theta < 0.0f)
	      t = -t;
	  }
	  c = 1.0f / (float) Math.sqrt(1.0f + t*t);
	  s = t*c;
	  tau = s / (1.0f+c);
	  h = t * a.m12;

	  z.y -= h;
	  z.z += h;
	  d.y -= h;
	  d.z += h;
	  
	  a.m12 = 0.0f;
	  // ROT(a, 0, 1, 0, 2)
	  g = a.getElement(0,1);
	  h = a.getElement(0,2);
	  a.setElement(0,1, g-s*(h+g*tau));
	  a.setElement(0,2, h+s*(g-h*tau));
	  // ROT(v, 0, 1, 0, 2)
	  g = v.getElement(1,0);
	  h = v.getElement(0,2);
	  v.setElement(1,0, g-s*(h+g*tau));
	  v.setElement(0,2, h+s*(g-h*tau));
	  // ROT(v, 1, 1, 1, 2)
	  g = v.getElement(1,1);
	  h = v.getElement(1,2);
	  v.setElement(1,1, g-s*(h+g*tau));
	  v.setElement(1,2, h+s*(g-h*tau));
	  // ROT(v, 2, 1, 2, 2)
	  g = v.getElement(2,1);
	  h = v.getElement(2,2);
	  v.setElement(2,1, g-s*(h+g*tau));
	  v.setElement(2,2, h+s*(g-h*tau));

	  nrot++;
	}
      }

      b.x += z.x;
      b.y += z.y;
      b.z += z.z;

      d.x += b.x;
      d.y += b.y;
      d.z += b.z;

      z.x += 0.0f;
      z.y += 0.0f;
      z.z += 0.0f;
    }

    System.out.println("Error: too many iterations in Jacobi transform "+i);

  }

  /**
   * Get the sorted eigenvector, use mEigen to get the vector, and sort it
   * based on eigenvalues
   * @param evecs eigenvector
   * @param cov matrix
   * @return MyMatrix3f eigenvector
   */
  public MyMatrix3f eigenSort(MyMatrix3f evecs, MyMatrix3f cov) {
    float t;
    MyVector3f evals = new MyVector3f();
    int n;

    mEigen(evecs, evals, cov);

    if(evals.z > evals.y) {

      if(evals.z > evals.y) {
	// 2 is largest, swap woth column 0
	t = evecs.m02;
	evecs.m02 = evecs.m00;
	evecs.m00 = t;
	t = evecs.m12;
	evecs.m12 = evecs.m10;
	evecs.m10 = t;
	t = evecs.m22;
	evecs.m22 = evecs.m20;
	evecs.m20 = t;
      }
      else {
	// 1 is largest, swap with column 0
	t = evecs.m01;
	evecs.m01 = evecs.m00;
	evecs.m00 = t;
	t = evecs.m11;
	evecs.m11 = evecs.m10;
	evecs.m10 = t;
	t = evecs.m21;
	evecs.m21 = evecs.m20;
	evecs.m20 = t;
      }
    }
    else {
      if(evals.x > evals.y) {
	// 0 is largest, do nothing
      }
      else {
	// 1 is largest
	t = evecs.m01;
	evecs.m01 = evecs.m00;
	evecs.m00 = t;
	t = evecs.m11;
	evecs.m11 = evecs.m10;
	evecs.m10 = t;
	t = evecs.m21;
	evecs.m21 = evecs.m20;
	evecs.m20 = t;
      }
    }

    //System.out.println("EigenVectors=\n"+evecs.toString());
    //System.out.println("EigenValues=\n"+evals.toString());
    return evecs;

  }

  public MyMatrix3f getEigen2() {
    //MyVector3f evals = new MyVector3f();
    MyMatrix3f cov = new MyMatrix3f();
    MyMatrix3f mat = new MyMatrix3f(this);

    return eigenSort(cov, mat);

    //System.out.println("EigenVectors=\n"+cov.toString());
    //System.out.println("EigenValues=\n"+evals.toString());
  }


}
