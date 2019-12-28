package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mergingBodies3D.RigidTransform;
import mintools.parameters.DoubleParameter;
import mintools.parameters.Vec3Parameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.SceneGraphNode;

/**
 * Quick test of Box Sphere collision detection 
 * @author kry
 */
public class TestBoxSphereCollisionApp implements SceneGraphNode{

	public static void main( String[] args ) {
		new EasyViewer("test box sphere collision", new TestBoxSphereCollisionApp() );
	}
	
	public TestBoxSphereCollisionApp() {
		// not much to do here
	}
	
	Vector3d size1 = new Vector3d( 1,2,3 );
	
	RigidTransform TB2W1 = new RigidTransform();
	RigidTransform TW2B1 = new RigidTransform();

	@Override
	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glDisable( GL2.GL_LIGHTING );

		double r = radius.getValue();
		
		Vector3d p1 = new Vector3d( pos1.x, pos1.y, pos1.z );
		Vector3d a1 = new Vector3d( axis1.x, axis1.y, axis1.z );
		a1.normalize();
		AxisAngle4d aa1 = new AxisAngle4d( a1, angle1.getValue() );
		Matrix3d R1 = new Matrix3d();
		R1.set( aa1 );
		Point3d p2 = new Point3d( pos2.x, pos2.y, pos2.z );
		
		ArrayList<DContactGeom> contacts = new ArrayList<DContactGeom>();
		
		TB2W1.set( R1, p1 );
		TW2B1.set( R1, p1 );
		TW2B1.invert();
		
		int cnum = BoxSphere.dBoxSphere(TB2W1, TW2B1, size1, p2, r, contacts);
				
		gl.glPointSize(10);
		gl.glLineWidth(3);
		gl.glColor3f(1, 0, 0);
		for ( DContactGeom c : contacts ) {
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex3d( c.pos.x, c.pos.y, c.pos.z );
			gl.glEnd();
			gl.glBegin(GL.GL_LINES);
			gl.glVertex3d( c.pos.x, c.pos.y, c.pos.z );
			double x = c.normal.x;
			double y = c.normal.y;
			double z = c.normal.z;
			gl.glVertex3d( x + c.pos.x, y + c.pos.y, z + c.pos.z );
			gl.glVertex3d( c.normal.x + c.pos.x, c.normal.y + c.pos.y, c.normal.z + c.pos.z );
			gl.glEnd();
		}
		
		gl.glColor3f(1,1,1);
		gl.glLineWidth(1);
	
		gl.glPushMatrix();
		gl.glTranslated( pos1.x, pos1.y, pos1.z );
		gl.glRotated(angle1.getValue()*180/Math.PI, axis1.x, axis1.y, axis1.z);
		gl.glScaled( size1.x, size1.y, size1.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
		
		gl.glPushMatrix();
		gl.glTranslated( pos2.x, pos2.y, pos2.z );
		EasyViewer.glut.glutWireSphere( r, 10, 10 );
		gl.glPopMatrix();

		String msg = "#contacts = 0\n";
		if ( cnum > 0 ) {
			DContactGeom c = contacts.get(0);
			msg = "#contacts = " + cnum + "\n" +
				"depth = " + c.depth + "\n";
		}
		EasyViewer.beginOverlay(drawable);
		EasyViewer.printTextLines(drawable, msg);
		EasyViewer.endOverlay(drawable);
	}
	
	@Override
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.add( pos1 );
		vfp.add( axis1 );
		vfp.add( angle1.getSliderControls(false) );
		vfp.add( pos2 );
		vfp.add( radius.getSliderControls(false) );
		return vfp.getPanel();
	}
	
	Vec3Parameter pos1 = new Vec3Parameter("pos1", 0,  5, 0 );
	Vec3Parameter pos2 = new Vec3Parameter("pos2", 0, -5, 0 );
	Vec3Parameter axis1 = new Vec3Parameter("axis1", 0,  5, 0 );
	DoubleParameter angle1 = new DoubleParameter("angle1", 0, -3.14, 3.14 );
	DoubleParameter radius = new DoubleParameter("radius", 1, 0.1, 3);
	
	@Override
	public void init(GLAutoDrawable drawable) {
		// nothing to do
	}

}
