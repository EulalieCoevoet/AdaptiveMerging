package mergingBodies3D.collision;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

public class DContactGeom {
	
	DContactGeom() {
		// do nothing
	}
	
	DContactGeom( Tuple3d pos, Tuple3d normal, double depth, int info ) {
		this.pos.set(pos);
		this.normal.set(normal);
		this.depth = depth;
		this.info = info;
	}
	
	/**
	 * Position of this contact in world coordinates
	 */
	public Point3d pos = new Point3d();
	
	/**
	 * Normal of this contact in world coordinates, points from body 1 to body 2
	 */
	public Vector3d normal = new Vector3d();       ///< normal vector
	
	/**
	 * Penetration depth, a positive number when there is interpenetration 
	 */
	public double depth;
	
	/** 
	 * Information for matching contacts across simulation time steps for warm starts.
	 * Can be left zero if it is unused (e.g., box-sphere)
	 */ 
	public int info; 
}

