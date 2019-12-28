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
	}
	
	public Point3d pos = new Point3d();          ///< contact position
	public Vector3d normal = new Vector3d();       ///< normal vector
	public double depth;           ///< penetration depth
	//public DGeom g1;         ///< the colliding geoms
	//public DGeom g2;
//	public int side1;       ///< (to be documented)
//	public int side2;
	
	/** Should do this in some other better way */
	public int info; 
}

