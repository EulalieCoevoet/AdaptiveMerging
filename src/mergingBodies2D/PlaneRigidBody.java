package mergingBodies2D;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

/**
 * A rigid body like all the rest, but always pinned, and its geometry defined by a plane equation. 
  */
public class PlaneRigidBody extends RigidBody {

	/**
	 * Normal for the plane
	 */
	Vector2d n;
	
	/**
	 * Point for the plane
	 */
	Point2d p; 
	
	/**
	 * d parameter of plane equation (computed in constructor, i.e., n_x x + n_y y + d = 0)
	 */
	double d;
	
	/** Used for warm starting contacts with this plane */
	Block dummyBlock = new Block(0,0, new Color3f());
	
	public PlaneRigidBody() {
		this(new Point2d(0,40), new Vector2d(0, -1));
	}
	
	public PlaneRigidBody( Point2d p, Vector2d n ) {
		n.normalize();
		this.n = new Vector2d(n);
		this.p = new Point2d(p);

		pinned = true;
		d = - (p.x*n.x + p.y*n.y);
	}
	
	@Override
	public void display(GLAutoDrawable drawable, Color3f color) {
		GL2 gl = drawable.getGL().getGL2();
		
		gl.glColor4f(0f, 0f, 0f, Block.alpha);
		
		double s = 1000;		
		gl.glBegin(GL.GL_TRIANGLE_STRIP);
		gl.glVertex2d( p.x - n.y*s, p.y + n.x*s );
		gl.glVertex2d( p.x + n.y*s, p.y - n.x*s );
		gl.glVertex2d( p.x - n.x - n.y*s, p.y -n.y + n.x*s );
		gl.glVertex2d( p.x - n.x + n.y*s, p.y -n.y - n.x*s );
        gl.glEnd();
	}
}
