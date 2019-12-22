package mergingBodies3D;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.EasyViewer;

/**
 * A rigid body like all the rest, but always pinned, and its geometry defined by a plane equation. 
 * @author kry
  */
public class PlaneRigidBody extends RigidBody {

	/**
	 * Normal for the plane
	 */
	Vector3d n;
	
	/**
	 * Point for the plane
	 */
	Point3d p; 
	
	/**
	 * Size for the plane
	 */
	float size = 1000; 
	
	/**
	 * d parameter of plane equation (computed in constructor, i.e., n_x x + n_y y + n_z z + d = 0)
	 */
	double d;
	
	/** Used for warm starting contacts with this plane */
	Block dummyBlock = new Block(0,0,0, new Color3f());
	
	public PlaneRigidBody() {
		this(new Point3d(0,40,0), new Vector3d(0, -1, 0));
	}
	
	public PlaneRigidBody( Point3d p, Vector3d n ) {
		super( 0, null, true, null );
		n.normalize();
		this.n = new Vector3d(n);
		this.p = new Point3d(p);
		pinned = true;
		d = - (p.x*n.x + p.y*n.y + p.z*n.z);
		geom = new PlaneRigidBodyGeom();
	}
	
	private class PlaneRigidBodyGeom extends RigidBodyGeom {
		@Override
		public void drawGeom(GLAutoDrawable drawable) {
			GL2 gl = drawable.getGL().getGL2();
			
			gl.glColor4f(0f, 0f, 0f, Block.alpha);
			
			float[] col = new float[] { 0.75f,0.75f,1, Block.alpha };		
			gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, col, 0 );
			
			float s = size;		
			
			Vector3d tmp = new Vector3d(0,-1,0);
			Vector3d axis = new Vector3d();
			axis.cross( tmp, n );
			double angle = tmp.angle(n);
			
			gl.glPushMatrix();
			gl.glTranslated(p.x, p.y + Block.h, p.z);
			gl.glRotated(angle, axis.x, axis.y, axis.z);
			gl.glScaled(s, 1, s);
			EasyViewer.glut.glutSolidCube(1);

			gl.glPopMatrix();			
		}
	}
	
}
