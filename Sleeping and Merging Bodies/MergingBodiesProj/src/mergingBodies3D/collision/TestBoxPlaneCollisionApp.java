package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mergingBodies3D.Contact;
import mergingBodies3D.ContactPool;
import mergingBodies3D.RigidBody;
import mintools.parameters.DoubleParameter;
import mintools.parameters.Vec3Parameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.SceneGraphNode;

/**
 * Quick test of Box Plane collision detection 
 * @author kry
 */
public class TestBoxPlaneCollisionApp implements SceneGraphNode{

	public static void main( String[] args ) {
		new EasyViewer("test box collision", new TestBoxPlaneCollisionApp() );
	}
	
	public TestBoxPlaneCollisionApp() {
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

		double d = planed.getValue();
		
		Vector3d p1 = new Vector3d( pos1.x, pos1.y, pos1.z );
		Vector3d a1 = new Vector3d( axis1.x, axis1.y, axis1.z );
		a1.normalize();
		AxisAngle4d aa1 = new AxisAngle4d( a1, angle1.getValue() );
		Matrix3d R1 = new Matrix3d();
		R1.set( aa1 );
		Vector3d n = new Vector3d( normal.x, normal.y, normal.z );
		n.normalize();

		contacts.clear();
		pool.swapPools();
		
		b1.transformB2W.set( R1, p1 );
		
		int cnum = BoxPlane.dBoxPlane( b1, size1, b2, n, d, contacts, pool );
				
		gl.glPointSize(10);
		gl.glLineWidth(3);
		for ( Contact c : contacts ) {
			gl.glColor3f(1, 0, 0);
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
			gl.glColor3f( 1,1,1 );
			gl.glRasterPos3d( c.contactW.x, c.contactW.y, c.contactW.z );
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "  " + c.info + " " + c.constraintViolation); 			
		}
		
		gl.glColor4f(1,1,1,0.5f);
		gl.glLineWidth(1);
	
		gl.glPushMatrix();
		gl.glTranslated( pos1.x, pos1.y, pos1.z );
		gl.glRotated(angle1.getValue()*180/Math.PI, axis1.x, axis1.y, axis1.z);
		gl.glScaled( size1.x, size1.y, size1.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
				
		// this is dumb and slow... but just a test
		Vector3d tmp = new Vector3d(0,0,1);
		Vector3d axis = new Vector3d();
		axis.cross( tmp, n );
		double angle = tmp.angle(n);
		
		gl.glPushMatrix();
		// need a point on the plane... chose p.x and p.z zero and solve n.y * p.y = -d
		// which is fine for most y normalish planes... but otherwise can fail badly..
		gl.glTranslated(0,-d/n.y,0);
		gl.glRotated(angle*180/Math.PI, axis.x, axis.y, axis.z);
		gl.glScaled( 1, 1, 0 );
		EasyViewer.glut.glutWireSphere( 10, 10, 10 );
		gl.glPopMatrix();

		gl.glColor4f(1,1,1,1);
		String msg = "#contacts = 0\n";
		if ( cnum > 0 ) {
			//DContactGeom c = contacts.get(0);
			msg = "#contacts = " + cnum + "\n";
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
		vfp.add( normal );
		vfp.add( planed.getSliderControls(false) );
		return vfp.getPanel();
	}
	
	Vec3Parameter pos1 = new Vec3Parameter("pos1", 0,  5, 0 );
	Vec3Parameter axis1 = new Vec3Parameter("axis1", 0,  5, 0 );
	DoubleParameter angle1 = new DoubleParameter("angle1", 0, -3.14, 3.14 );
	Vec3Parameter normal = new Vec3Parameter("plane normal", 0,  5, 0 );
	DoubleParameter planed = new DoubleParameter("plane d", 0, -1, 1);
	
	@Override
	public void init(GLAutoDrawable drawable) {
		// nothing to do
	}

}
