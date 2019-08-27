package src;

import java.text.DecimalFormat;

import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

/**
 * 2D matrices to work with vecmath and provide convenient methods for
 * decomposition.
 * 
 * @author kry
 */
public class Matrix2d {

	/**
	 * a b
	 * c d
	 */
	double a,b,c,d;
	
	public Matrix2d() {
		zero();
	}
	
	public Matrix2d( Matrix2d M ) {
		set(M);
	}
	
	public Matrix2d( double a, double b, double c, double d ) {
		set(a,b,c,d);
	}
	
	public void zero() {
		a = 0;
		b = 0;
		c = 0;
		d = 0;
	}
	
	public void identity() {
		a = 1;
		b = 0;
		c = 0;
		d = 1;
	}

	public void set( Matrix2d M ) {
		a = M.a;
		b = M.b;
		c = M.c;
		d = M.d;
	}
	
	/**
	 * this = s * M
	 * @param s
	 * @param M
	 */
	public void scale( double s, Matrix2d M ) {
		a = s * M.a;
		b = s * M.b;
		c = s * M.c;
		d = s * M.d;
	}
	
	/*
	 * this = s * this
	 */
	public void scale( double s ) {
		a *= s;
		b *= s;
		c *= s;
		d *= s;		
	}
	
	public void set( double a, double b, double c, double d ) {
		this.a = a;
		this.b = b;
		this.c = c;
		this.d = d;
	}
	
	/**
	 * Frobeneus norm squared
	 * @return
	 */
	public double fro2() {
		return a*a + b*b* + c*c + d*d;
	}

	/**
	 * Frobeneus norm
	 * @return
	 */
	public double fro() {
		return Math.sqrt( a*a + b*b* + c*c + d*d );
	}

	
	/**
	 * this += alpha * v * v^T 
	 * @param alpha
	 * @param v
	 */
	public void rank1( double alpha, Tuple2d v ) {
		a += alpha * v.x * v.x;
		b += alpha * v.x * v.y;
		c += alpha * v.y * v.x;
		d += alpha * v.y * v.y;
	}
	
	/**
	 * Sets this matrix to be the transpose of the passed matrix
	 */
	public void transpose( Matrix2d M ) {
		a = M.a;
		b = M.c;
		c = M.b;
		d = M.d;
	}
	
	/**
	 * this = this^T
	 */
	public void transpose() {
		double tmp = c;
		c = b;
		b = tmp;
	}
	
	/**
	 * @return the determinant of the matrix
	 */
	public double det() {
		return a*d - b*c;
	}
	
	/**
	 * @return the trace of the matrix
	 */
	public double trace() {
		return a + d;
	}
	
	/**
	 * Sets this matrix to be the inverse of the passed matrix.
	 */
	public void inverse( Matrix2d M ) {
		double invdet = 1.0 / M.det();
		// we'll charge through without checking for errors...
		// could produce infinity or NaN 
		double ta = invdet * M.d;
		double tb = - invdet * M.b;
		double tc = - invdet * M.c;
		double td = invdet * M.a;
		// safety in case we call with ourselves
		a = ta;
		b = tb;
		c = tc;
		d = td;
	}

	/**
	 * Inverts this matrix in place
	 */
	public void inverse() {
		inverse(this);
	}
	
	/**
	 * this = this * M
	 * @param M
	 */
	public void mult( Matrix2d M ) {
		double ta = a * M.a + b * M.c;
		double tb = a * M.b + b * M.d;
		double tc = c * M.a + d * M.c;
		double td = c * M.b + d * M.d;
		a = ta;
		b = tb;
		c = tc;
		d = td;
	}
	
	/**
	 * this = A^T * B
	 */
	public void transAmult( Matrix2d A, Matrix2d B ) {
		double ta = A.a * B.a + A.c * B.c;
		double tb = A.a * B.b + A.c * B.d;
		double tc = A.b * B.a + A.d * B.c;
		double td = A.b * B.b + A.d * B.d;
		a = ta;
		b = tb;
		c = tc;
		d = td;
	}
	
	/**
	 * this = A * B
	 */
	public void mult( Matrix2d A, Matrix2d B ) {
		double ta = A.a * B.a + A.b * B.c;
		double tb = A.a * B.b + A.b * B.d;
		double tc = A.c * B.a + A.d * B.c;
		double td = A.c * B.b + A.d * B.d;
		a = ta;
		b = tb;
		c = tc;
		d = td;
	}
	
	/**
	 * this = A + B
	 * @param A
	 * @param B
	 */
	public void add( Matrix2d A, Matrix2d B ) {
		a = A.a + B.a;
		b = A.b + B.b;
		c = A.c + B.c;
		d = A.d + B.d;
	}

	/**
	 * this += A
	 * @param A
	 * @param B
	 */
	public void add( Matrix2d A ) {
		a += A.a;
		b += A.b;
		c += A.c;
		d += A.d;
	}

	/**
	 * this = this * M^T
	 */
	public void multTrans( Matrix2d M ) {
		double ta = a * M.a + b * M.b;
		double tb = a * M.c + b * M.d;
		double tc = c * M.a + d * M.b;
		double td = c * M.c + d * M.d;
		a = ta;
		b = tb;
		c = tc;
		d = td;
	}
	
	/**
	 * this = this^T * M
	 */
	public void transMult( Matrix2d M ) {
		double ta = a * M.a + c * M.c;
		double tb = a * M.b + c * M.d;
		double tc = b * M.a + d * M.c;
		double td = b * M.b + d * M.d;
		a = ta;
		b = tb;
		c = tc;
		d = td;
	}
	/**
	 * result = this * v
	 * @param v
	 * @param result
	 */
	public void transform( Tuple2d v, Tuple2d result ) {
		result.x = a * v.x + b * v.y;
		result.y = c * v.x + d * v.y;
	}
	
	boolean imaginaryEigenvalues = false;
	/** First eigenvalue real part */
	double ev1;
	/** Second eigenvalue real part */
	double ev2;
	/** First eigenvalue imaginary part */
	double ev1i;
	/** Second eigenvalue imaginary part */
	double ev2i;
	/** First eigenvector */
	Vector2d v1 = new Vector2d();
	/** Second eigenvector */
	Vector2d v2 = new Vector2d();

	/**
	 * Computes the eigenvalue decomposition.
	 * If the eigenvalues are complex, then the vectors are NOT computed :(
	 * see https://math.stackexchange.com/questions/395698/fast-way-to-calculate-eigen-of-2x2-matrix-using-a-formula
	 */
	public void evd() {
		// first compute the eigenvalues
		double disc = (a+d)*(a+d) - 4*(a*d-b*c);
		if ( disc < 0 ) {
			imaginaryEigenvalues = true;
			double sd = Math.sqrt(-disc);
			ev1 = (a+d)/2.0;
			ev2 = (a+d)/2.0;
			ev1i = -sd/2.0;
			ev2i = sd/2.0;
			// not sure we really want to deal with this case,
			// the vectors are harder to deal with. :(
			v1.set(0,0);
			v2.set(0,0);
		} else {
			imaginaryEigenvalues = false;
			double sd = Math.sqrt(disc);
			ev1 = ((a+d) - sd)/2.0;
			ev2 = ((a+d) + sd)/2.0;
			ev1i = 0;
			ev2i = 0;
			if ( sd < 1e-10 ) {
				v1.set( 1, 0 );
				v2.set( 0, 1 );
			} else {
				// If c is zero and ev1 or ev2 is equal to d
				// then we'll want to do something different.
				// Note that we'll catch the case of ev1=ev2 above.
				// so pick the bigger of b and c to set the vecs
				if ( Math.abs(c) > Math.abs(b) ) {
					v1.set( ev1 - d, c );
					v2.set( ev2 - d, c );
				} else {
					v1.set( b, ev1 - a );
					v2.set( b, ev2 - a );
				}
				if ( v1.length() < 1e-10 ) {
					v2.normalize();
					v1.set( v2.y, -v2.x );
				} else if ( v2.length() < 1e-10 ) {
					v1.normalize();
					v2.set( -v1.y, v1.x );
				} else {
					v1.normalize();
					v2.normalize();
				}
			}
		}
	}
	
	private final DecimalFormat df = new DecimalFormat("0.0000");

	public String toString() {
		return "[" + df.format(a) + " " + df.format(b) + "; " +
			   df.format(c) + " " + df.format(d) + "]";
	}
	
	public void printEVD() {
		System.out.println("imaginary eigenvalues = " + imaginaryEigenvalues );
		if ( imaginaryEigenvalues) {
			System.out.println("evs = " + ev1 + "+" + ev1i + "i "+ ev2 + "+" + ev2i + "i" );
		} else {
			System.out.println("evs = " + ev1 + " " + ev2 );
		}
		System.out.println("v1 = " + v1.toString() );
		System.out.println("v2 = " + v2.toString() );
	}

}
