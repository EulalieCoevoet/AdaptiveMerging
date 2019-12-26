package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.cpp4j.java.RefDouble;
import org.cpp4j.java.RefInt;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DContactGeomBuffer;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.DoubleParameter;
import mintools.parameters.Vec3Parameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.SceneGraphNode;

/**
 * This is a quick test of glue to the box box collision detection of a java port of ODE,
 * 
 * @author kry
 *
 */
public class BoxCollisionTest2 implements SceneGraphNode{

	public static void main( String[] args ) {
		new EasyViewer("test box collision", new BoxCollisionTest2() );
	}
	
	public BoxCollisionTest2() {
		// not much to do here
	}
	
	Vector3d size1 = new Vector3d( 1,2,3 );
	Vector3d size2 = new Vector3d( 2,3,4 );
	
	@Override
	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glDisable( GL2.GL_LIGHTING );

		DVector3 p1 = new DVector3( pos1.x, pos1.y, pos1.z );
		Vector3d p1v = new Vector3d( pos1.x, pos1.y, pos1.z );
		Vector3d a1 = new Vector3d( axis1.x, axis1.y, axis1.z );
		a1.normalize();
		AxisAngle4d aa1 = new AxisAngle4d( a1, angle1.getValue() );		
		Matrix3d R1t = new Matrix3d();
		R1t.set( aa1 );
		DMatrix3 R1 = new DMatrix3();
		
		DVector3 p2 = new DVector3( pos2.x, pos2.y, pos2.z );
		Vector3d p2v = new Vector3d( pos2.x, pos2.y, pos2.z );
		Vector3d a2 = new Vector3d( axis2.x, axis2.y, axis2.z );
		a2.normalize();
		AxisAngle4d aa2 = new AxisAngle4d( a2, angle2.getValue() );
		Matrix3d R2t = new Matrix3d();		
		R2t.set( aa2 );
		DMatrix3 R2 = new DMatrix3();

		for ( int i = 0; i < 3; i++ ) {
			for ( int j = 0; j < 3; j++ ) {
				R1.set(i, j, R1t.getElement(i,j) );
				R2.set(i, j, R2t.getElement(i,j) );				
			}
		}

		DVector3 normal = new DVector3();
		RefDouble depth = new RefDouble();
		RefInt return_code = new RefInt();
		int flags = 0xffff; // as many contacts as we can get!
		int skip = 1; // stride in the geom buffer... 
		DContactGeomBuffer contacts = new DContactGeomBuffer(20);
		
		DVector3 side1 = new DVector3();
		DVector3 side2 = new DVector3();
		side1.set( size1.x, size1.y, size1.z );
		side2.set( size2.x, size2.y, size2.z );
		
		int cnum = org.ode4j.ode.internal.DxBox.dBoxBox( p1, R1, side1, p2, R2, side2, normal, depth, return_code, flags, contacts, skip );

		Vector3d normal2 = new Vector3d();
		double[] depth2 = new double[1];
		int[] return_code2 = new int[1];
		ArrayList<DContactGeom>contacts2 = new ArrayList<DContactGeom>();
		int cnum2 = 0;
		boolean died = false;
		try {
			cnum2 = DxBox.dBoxBox( p1v, R1t, size1, p2v, R2t, size2, normal2, depth2, return_code2, flags, contacts2, skip );
		} catch ( Exception e ) {
			died = true;
		}
		
		gl.glPointSize(10);
		gl.glLineWidth(3);
		gl.glColor3f(1, 0, 0);
		for ( int i = 0; i < cnum; i++ ) {
			org.ode4j.ode.DContactGeom c = contacts.get(i);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex3d( c.pos.get0(), c.pos.get1(), c.pos.get(2) );
			gl.glEnd();
			gl.glBegin(GL.GL_LINES);
			gl.glVertex3d( c.pos.get0(), c.pos.get1(), c.pos.get(2) );
			double x = normal.get0();
			double y = normal.get1();
			double z = normal.get2();
			gl.glVertex3d( x + c.pos.get0(), y + c.pos.get1(), z + c.pos.get2() );
			gl.glEnd();
		}

		gl.glDisable(GL.GL_DEPTH_TEST);
		gl.glPointSize(7);
		gl.glLineWidth(1);
		gl.glColor3f(0, 1, 0);
		for ( int i = 0; i < cnum2; i++ ) {
			DContactGeom c = contacts2.get(i);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex3d( c.pos.x, c.pos.y, c.pos.z );
			gl.glEnd();
			gl.glBegin(GL.GL_LINES);
			gl.glVertex3d( c.pos.x, c.pos.y, c.pos.z );
			double x = normal2.x;
			double y = normal2.y;
			double z = normal2.z;
			gl.glVertex3d( x + c.pos.x, y + c.pos.y, z + c.pos.z );
			gl.glEnd();
		}
		gl.glEnable(GL.GL_DEPTH_TEST);

		
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
		gl.glRotated(angle2.getValue()*180/Math.PI, axis2.x, axis2.y, axis2.z);
		gl.glScaled( size2.x, size2.y, size2.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
		
		String msg = "JAR:\n" +
				"#contacts = " + cnum + "\n" +
				"depth = " + depth.get() +"\n" +
				"normal = " + normal.toString() + "\n" + 
				"code = " + return_code.get() + "\n" + 
				"\nPORT:\n"+
				"#contacts = " + cnum2 + "\n" +
				"depth = " + depth2[0] + "\n" +
				"normal = " + normal2.toString() + "\n" +
				"code = " + return_code2[0] + " \n" +
				(died?"death!":"");
		EasyViewer.beginOverlay(drawable);
		EasyViewer.printTextLines(drawable, msg,20,20,15,GLUT.BITMAP_8_BY_13);
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
		return vfp.getPanel();
	}
	
	Vec3Parameter pos1 = new Vec3Parameter("pos1", 0,  0.99, 0 );
	Vec3Parameter pos2 = new Vec3Parameter("pos2", 0, -1, 0 );
	Vec3Parameter axis1 = new Vec3Parameter("axis1", 0, 1, 0 );
	Vec3Parameter axis2 = new Vec3Parameter("axis2", 0, 1, 0 );
	DoubleParameter angle1 = new DoubleParameter("angle1", 0, -3.14, 3.14 );
	DoubleParameter angle2 = new DoubleParameter("angle2", 0, -3.14, 3.14 );
	
	@Override
	public void init(GLAutoDrawable drawable) {
		// nothing to do
	}

}
