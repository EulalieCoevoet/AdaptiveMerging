package mergingBodies3D;

import javax.vecmath.Vector3d;

public class Vector6d {

	/** Linear components */
	public Vector3d v;
	/** Angular components */
	public Vector3d w;
	
	public Vector6d() {
		v = new Vector3d();
		w = new Vector3d();
	}
	
	/**
	 * constructs a 6D vector using provided vectors as memory
	 * @param v
	 * @param w
	 */
	public Vector6d( Vector3d v, Vector3d w ) {
		this.v = v;
		this.w = w;
	}
	
	/** 
	 * Dot product (JIT should quickly inline this)
	 * @param a
	 * @return
	 */
	public double dot( Vector6d a ) {
		return v.dot( a.v ) + w.dot( a.w );
	}
	
	/**
	 * Sets this vector to be equal to the given vector
	 * @param a
	 */
	public void set( Vector6d a ) {
		v.set( a.v );
		w.set( a.w );
	}
	
	public void setZero() {
		w.set(0,0,0);
		v.set(0,0,0);
	}
	
	@Override
	public String toString() {
		return "v=" + v.toString() + " w=" + w.toString() + " " + super.toString();
	}
	
}
