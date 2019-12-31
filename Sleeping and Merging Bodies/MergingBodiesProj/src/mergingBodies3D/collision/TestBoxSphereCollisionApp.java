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

import mergingBodies3D.Contact;
import mergingBodies3D.ContactPool;
import mergingBodies3D.RigidBody;
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

	ArrayList<Contact> contacts = new ArrayList<Contact>();
	ContactPool pool = new ContactPool();
	RigidBody b1 = new RigidBody( 0, null,true, null );
	RigidBody b2 = new RigidBody( 0, null,true, null );

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
		
		
		b1.transformB2W.set( R1, p1 );
		b1.transformW2B.set( R1, p1 );
		b1.transformW2B.invert();
		
		contacts.clear();
		pool.swapPools();
		
		int cnum = BoxSphere.dBoxSphere( b1, size1, b2, p2, r, contacts, pool );
		boolean test = BoxSphere.dBoxSphereTest(b1.transformB2W, b1.transformW2B, size1, p2, r );
				
		gl.glPointSize(10);
		gl.glLineWidth(3);
		gl.glColor3f(1, 0, 0);
		for ( Contact c : contacts ) {
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex3d( c.contactW.x, c.contactW.y, c.contactW.z );
			gl.glEnd();
			gl.glBegin(GL.GL_LINES);
			gl.glVertex3d( c.contactW.x, c.contactW.y, c.contactW.z );
			double x = c.normalW.x;
			double y = c.normalW.y;
			double z = c.normalW.z;
			gl.glVertex3d( x + c.contactW.x, y + c.contactW.y, z + c.contactW.z );
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

		String msg = "#contacts = 0, test result = " + test + "\n";
		if ( cnum > 0 ) {
			Contact c = contacts.get(0);
			msg = "#contacts = " + cnum + "\n" +
				"depth = " + c.constraintViolation + "\n";
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
	
	Vec3Parameter pos1 = new Vec3Parameter("pos1", -0.5,  1, 0 ); // funny test case
	Vec3Parameter pos2 = new Vec3Parameter("pos2", 0, -1, 0 );
	Vec3Parameter axis1 = new Vec3Parameter("axis1", 0,  1, 0 );
	DoubleParameter angle1 = new DoubleParameter("angle1", 0, -3.14, 3.14 );
	DoubleParameter radius = new DoubleParameter("radius", 1, 0.1, 3);
	
	@Override
	public void init(GLAutoDrawable drawable) {
		// nothing to do
	}

}
