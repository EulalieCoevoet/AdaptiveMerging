package mergingBodies3D.collision;

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
		AxisAngle4d aa1 = new AxisAngle4d( axis1.x, axis1.y, axis1.z, angle1.getValue() );		
		Matrix3d R1t = new Matrix3d();
		R1t.set( aa1 );
		DMatrix3 R1 = new DMatrix3();
		
		DVector3 p2 = new DVector3( pos2.x, pos2.y, pos2.z );
		AxisAngle4d aa2 = new AxisAngle4d( axis2.x, axis2.y, axis2.z, angle2.getValue() );
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
		int skip = 0; // only use skip if we want to dump other tests into the same arraylist
		DContactGeomBuffer contacts = new DContactGeomBuffer(20);
		
		DVector3 side1 = new DVector3();
		DVector3 side2 = new DVector3();
		side1.set( size1.x, size1.y, size1.z );
		side2.set( size2.x, size2.y, size2.z );
		
		int cnum = org.ode4j.ode.internal.DxBox.dBoxBox( p1, R1, side1, p2, R2, side2, normal, depth, return_code, flags, contacts, skip );

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
			gl.glVertex3d( c.normal.get0() + c.pos.get0(), c.normal.get1() + c.pos.get1(), c.normal.get2() + c.pos.get2() );
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
		gl.glRotated(angle2.getValue()*180/Math.PI, axis2.x, axis2.y, axis2.z);
		gl.glScaled( size2.x, size2.y, size2.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
		
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
	
	Vec3Parameter pos1 = new Vec3Parameter("pos1", 0,  5, 0 );
	Vec3Parameter pos2 = new Vec3Parameter("pos2", 0, -5, 0 );
	Vec3Parameter axis1 = new Vec3Parameter("axis1", 0,  5, 0 );
	Vec3Parameter axis2 = new Vec3Parameter("axis2", 0, -5, 0 );
	DoubleParameter angle1 = new DoubleParameter("angle1", 0, -3.14, 3.14 );
	DoubleParameter angle2 = new DoubleParameter("angle2", 0, -3.14, 3.14 );
	
	@Override
	public void init(GLAutoDrawable drawable) {
		// nothing to do
	}

}
