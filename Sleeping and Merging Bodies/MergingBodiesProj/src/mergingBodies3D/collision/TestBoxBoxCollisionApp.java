package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.swing.JLabel;
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
 * This is a quick test of a port of the box box collision detection of a java port of ODE, now working!
 * @author kry
 */
public class TestBoxBoxCollisionApp implements SceneGraphNode{

	public static void main( String[] args ) {
		new EasyViewer("test box box collision", new TestBoxBoxCollisionApp() );
	}
	
	public TestBoxBoxCollisionApp() {
		// not much to do here
	}
	
	Vector3d size1 = new Vector3d( 1,2,3 );
	Vector3d size2 = new Vector3d( 2,3,4 );
	
	ArrayList<Contact> contacts = new ArrayList<Contact>();
	ArrayList<Contact> contacts2 = new ArrayList<Contact>();
	ContactPool pool = new ContactPool();
	RigidBody b1 = new RigidBody( 0, null,true, null );
	/** same as body 1 with a tiny change in rotation */
	RigidBody b1e = new RigidBody( 0, null,true, null );
	RigidBody b2 = new RigidBody( 0, null,true, null );
	
	@Override
	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glDisable( GL2.GL_LIGHTING );

		Vector3d p1 = new Vector3d( pos1.x, pos1.y, pos1.z );
		Vector3d a1 = new Vector3d( axis1.x, axis1.y, axis1.z );
		a1.normalize();
		
		AxisAngle4d aa1 = new AxisAngle4d( a1, angle1.getValue() );
		Matrix3d R1 = new Matrix3d();
		R1.set( aa1 );
		
		AxisAngle4d aa1e = new AxisAngle4d( a1, angle1.getValue()+0.01 );
		Matrix3d R1e = new Matrix3d();
		R1e.set( aa1e );

		Vector3d p2 = new Vector3d( pos2.x, pos2.y, pos2.z );
		Vector3d a2 = new Vector3d( axis2.x, axis2.y, axis2.z );
		a2.normalize();
		AxisAngle4d aa2 = new AxisAngle4d( a2, angle2.getValue() );
		Matrix3d R2 = new Matrix3d();
		R2.set( aa2 );
		
		BoxBox.fudgeFactor = fudgeFactor.getValue();
		BoxBox.tinyOffset = tinyOffset.getValue();
		BoxBox.dLineClosestApproachEPS = LCAEPS.getValue();
		
		Vector3d normal = new Vector3d();
		double[] depth = new double[1];
		int[] return_code = new int[1];
		
		Vector3d normale = new Vector3d();
		double[] depthe = new double[1];
		int[] return_codee = new int[1];
		
		b1.radius = size1.length() / 2;
		b1e.radius = size1.length() / 2;
		b2.radius = size2.length() / 2;
		
		
		b1.theta.set( R1 );
		b1.x.set( p1 );

		b1e.theta.set(R1e);
		b1e.x.set( p1 );

		b2.theta.set( R2 );
		b2.x.set( p2 );
		
		b1.updateRotationalInertaionFromTransformation();
		b1e.updateRotationalInertaionFromTransformation();
		b2.updateRotationalInertaionFromTransformation();

		contacts.clear();
		pool.swapPools();

		int cnum = BoxBox.dBoxBox( b1, size1, b2, size2, normal, depth, return_code, contacts, pool );

		contacts2.clear();
		pool.swapPools();
		
		int cnume = BoxBox.dBoxBox( b1e, size1, b2, size2, normale, depthe, return_codee, contacts2, pool );


		
		gl.glPointSize(10);
		gl.glLineWidth(3);
		int i = 0; 
		for ( Contact c : contacts ) {
			c.info = i++; // cheap way to keep track of contacts for warm starts?
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
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "  " + c.info + " " + c.constraintViolation ); 
//			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, c.side1 + " " + c.side2 ); 
			// side is unused :(  but doesn't need to be... 
			// thus we are somewhat on our own for warmstarts..
		}
		
		gl.glPointSize(10);
		gl.glLineWidth(3);
		i = 0; 
		for ( Contact c : contacts2 ) {
			c.info = i++; // cheap way to keep track of contacts for warm starts?
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
			gl.glRasterPos3d( c.contactW.x, c.contactW.y+labelOffy.getValue(), c.contactW.z );
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "  " + c.info + " " + c.constraintViolation ); 
//			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, c.side1 + " " + c.side2 ); 
			// side is unused :(  but doesn't need to be... 
			// thus we are somewhat on our own for warmstarts..
		}
		
		gl.glColor4f(1,1,1,0.5f);
		gl.glLineWidth(1);
	
		gl.glPushMatrix();
		gl.glTranslated( pos1.x, pos1.y, pos1.z );
		gl.glRotated(angle1.getValue()*180/Math.PI, axis1.x, axis1.y, axis1.z);
		gl.glScaled( size1.x, size1.y, size1.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
		
		gl.glPushMatrix();
		gl.glTranslated( pos2.x, pos2.y, pos2.z );
		gl.glRotated(angle2.getValue()*180/Math.PI, axis2.x, axis2.y, axis2.z);
		gl.glScaled( size2.x, size2.y, size2.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
		
		gl.glColor4f(1,1,1,1);

		String msg = "#contacts = " + cnum + " and " + cnume + "\n" +
				"depth = " + depth[0] + "\n" +
				"normal = " + normal.toString() + "\n" +
				"code = " + return_code[0];
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
		vfp.add( axis2 );
		vfp.add( angle2.getSliderControls(false) );
		vfp.add( new JLabel("tweaks..."));
		vfp.add( fudgeFactor.getSliderControls(false));
		vfp.add( tinyOffset.getSliderControls(true));
		vfp.add( LCAEPS.getSliderControls(true));
		vfp.add( labelOffy.getSliderControls(false));
		return vfp.getPanel();
	}
	
	Vec3Parameter pos1 = new Vec3Parameter("pos1", 0,  1, 0 );
	Vec3Parameter pos2 = new Vec3Parameter("pos2", 0, -1, 0 );
	Vec3Parameter axis1 = new Vec3Parameter("axis1", 0,  1, 0 );
	Vec3Parameter axis2 = new Vec3Parameter("axis2", 0,  1, 0 );
	DoubleParameter angle1 = new DoubleParameter("angle1", -2.9076, -3.14, 3.14 );
	DoubleParameter angle2 = new DoubleParameter("angle2", 0, -3.14, 3.14 );
	
	DoubleParameter fudgeFactor = new DoubleParameter( "fudgeFactor", BoxBox.fudgeFactor, 1, 1.20 );
	DoubleParameter LCAEPS= new DoubleParameter( "line closest approach EPS", BoxBox.dLineClosestApproachEPS, 1e-12, 1e-2 );
	DoubleParameter tinyOffset = new DoubleParameter( "tinyOffset", BoxBox.tinyOffset, 1e-12, 1e-2 );
	
	DoubleParameter labelOffy = new DoubleParameter( "label offset y", 0.1, 0, 1 );
	
	@Override
	public void init(GLAutoDrawable drawable) {
		// nothing to do
	}

}
