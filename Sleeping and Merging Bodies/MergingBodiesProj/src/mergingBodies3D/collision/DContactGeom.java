package mergingBodies3D.collision;

import javax.vecmath.Vector3d;

public class DContactGeom {
	
	DContactGeom() {
		// Non-public
	}
	
	public Vector3d pos = new Vector3d();          ///< contact position
	public Vector3d normal = new Vector3d();       ///< normal vector
	public double depth;           ///< penetration depth
	//public DGeom g1;         ///< the colliding geoms
	//public DGeom g2;
	public int side1;       ///< (to be documented)
	public int side2;
}

