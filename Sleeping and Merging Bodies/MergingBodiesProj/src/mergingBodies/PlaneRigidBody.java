package mergingBodies;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * A rigid body like all the rest, but always pinned, and its geometry defined by a plane equation. 
 * @author kry
  */
public class PlaneRigidBody extends RigidBody {

	/**
	 * Default normal for the plane
	 */
	Vector2d n = new Vector2d(0,1);
	/**
	 * d parameter of plane equation (computed in construtor, i.e., n_x x + n_y y + d = 0)
	 */
	double d;
	
	Point2d p = new Point2d( 0, -10); 
	
	/** Used for warm starting contacts with this plane */
	Block dummyBlock = new Block(0,0, new Color3f());
	
	public PlaneRigidBody() {
		d = - (p.x*n.x + p.y*n.y);
	}
}
