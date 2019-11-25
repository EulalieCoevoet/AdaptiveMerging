package mergingBodies;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

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
	
	Point2d p = new Point2d( 0, -20); 
	
	/** Used for warm starting contacts with this plane */
	Block dummyBlock = new Block(0,0, new Color3f());
	
	public PlaneRigidBody() {
		pinned = true;
		d = - (p.x*n.x + p.y*n.y);
	}
	
	public PlaneRigidBody( Point2d p, Vector2d n ) {
		pinned = true;
		n.normalize();
		this.n.set(n);
		this.p.set(p);
		d = - (p.x*n.x + p.y*n.y);
	}
	
	@Override
	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glBegin( GL.GL_LINES );
		double s = 1000;
		gl.glVertex2d( p.x - n.y*s, p.y + n.x*s );
		gl.glVertex2d( p.x + n.y*s, p.y - n.x*s );
		gl.glEnd();
	}
}
