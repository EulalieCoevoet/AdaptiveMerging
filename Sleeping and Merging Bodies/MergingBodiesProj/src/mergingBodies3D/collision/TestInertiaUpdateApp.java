package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mergingBodies3D.Contact;
import mergingBodies3D.ContactPool;
import mergingBodies3D.RigidBody;
import mergingBodies3D.RigidCollection;
import mintools.parameters.DoubleParameter;
import mintools.parameters.Vec3Parameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.SceneGraphNode;

/**
 * This is a quick test of a port of the box box collision detection of a java port of ODE, now working!
 * 
 */
public class TestInertiaUpdateApp implements SceneGraphNode{

	public static void main( String[] args ) {
		new EasyViewer("test inertia update", new TestInertiaUpdateApp() );
	}
	
	public TestInertiaUpdateApp() {
		// not much to do here
	}
	
	Vec3Parameter size1 = new Vec3Parameter("size of box1", 3,1,1,5);
	Vec3Parameter size2 = new Vec3Parameter("size of box2", 1,3,1,5);
	
	Vec3Parameter pos1 = new Vec3Parameter("pos of box1", -1,0,0,15);
	Vec3Parameter pos2 = new Vec3Parameter("pos of box2",  1,0,0,15);
	
	Vec3Parameter axis1 = new Vec3Parameter("axis1", 0,  1, 0 );
	Vec3Parameter axis2 = new Vec3Parameter("axis2", 0,  1, 0 );
	Vec3Parameter axisCollection = new Vec3Parameter("axisCollection", 0,  1, 0 );
	
	DoubleParameter angle1 = new DoubleParameter("angle1", 0, -3.14, 3.14 );
	DoubleParameter angle2 = new DoubleParameter("angle2", 0, -3.14, 3.14 );
	DoubleParameter angleCollection = new DoubleParameter("angle collection", 0, -3.14, 3.14 );
		
	@Override
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.add(size1);
		vfp.add(pos1);
		vfp.add(axis1);
		vfp.add(angle1.getSliderControls(false));
		vfp.add(size2);
		vfp.add(pos2);
		vfp.add(axis2);
		vfp.add(angle2.getSliderControls(false));
		vfp.add(axisCollection);
		vfp.add(angleCollection.getSliderControls(false));
		return vfp.getPanel();
	}
	
	ArrayList<Contact> contacts = new ArrayList<Contact>();
	ArrayList<Contact> contacts2 = new ArrayList<Contact>();
	ContactPool pool = new ContactPool();
	RigidBody b1 = new RigidBody( 1., null, true, null );
	RigidBody b2 = new RigidBody( 2., null, true, null );
	
	public void setBodyStuff( RigidBody b, Vec3Parameter axis, DoubleParameter angle, Vec3Parameter pos, Vec3Parameter size ) {
		b.pinned = false;		
		AxisAngle4d aa = new AxisAngle4d( axis.x, axis.y, axis.z, angle.getValue() );
		Matrix3d R = new Matrix3d();
		R.set( aa );		
		b.theta.set( R );
		b.x.set( pos.x, pos.y, pos.z );
		Vector3d s = new Vector3d();
		s.set( size.x, size.y, size.z );
		b.massLinear = s.x*s.y*s.z ;
		b.massAngular0.setIdentity();
		b.massAngular0.m00 = 1.0/12*b.massLinear*(s.y*s.y+s.z*s.z);
		b.massAngular0.m11 = 1.0/12*b.massLinear*(s.x*s.x+s.z*s.z);
		b.massAngular0.m22 = 1.0/12*b.massLinear*(s.x*s.x+s.y*s.y);
		b.jinv0.invert(b.massAngular0);
		b.transformB2W.computeRJinv0RT(b.massAngular0, b.massAngular);
		b.transformB2W.computeRJinv0RT(b.jinv0, b.jinv);
	}
	
	
	RigidCollection collection;
	String collectionAngularMass = "";
	
	Matrix3d diff =new Matrix3d();

	public void update() {
		
		setBodyStuff( b1, axis1, angle1, pos1, size1 );
		setBodyStuff( b2, axis2, angle2, pos2, size2 );
		
		collection = new RigidCollection(b1, b2);
		
		collectionAngularMass = "merged = \n" + collection.massAngular.toString();
		
		AxisAngle4d aa = new AxisAngle4d( axisCollection.x, axisCollection.y, axisCollection.z, angleCollection.getValue() );
		Matrix3d R = new Matrix3d();
		R.set( aa );		
		collection.theta.set( R );
		collection.updateRotationalInertaionFromTransformation();
		collection.updateBodiesPositionAndTransformations();
		
		ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
		bodies.add(b1);		
		collection.removeBodies(bodies);

		collectionAngularMass += "unmerged = \n" + collection.massAngular.toString();
		
		diff.set( b2.massAngular);
		diff.sub( collection.massAngular );
	}

	private double[] M = new double[16];

	@Override
	public void display(GLAutoDrawable drawable) {
		
		update();
		
		GL2 gl = drawable.getGL().getGL2();
		gl.glDisable( GL2.GL_LIGHTING );
		
		gl.glColor4f(1,1,1,0.5f);
		gl.glLineWidth(1);
	
		gl.glPushMatrix();
		b1.transformB2W.getAsArray(M);
		gl.glMultMatrixd( M, 0 );
		gl.glScaled( size1.x, size1.y, size1.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
		
		gl.glPushMatrix();
		b2.transformB2W.getAsArray(M);
		gl.glMultMatrixd( M, 0 );
		gl.glScaled( size2.x, size2.y, size2.z );
		EasyViewer.glut.glutWireCube(1);
		gl.glPopMatrix();
		
		gl.glColor4f(1,1,1,1);
		
		String msg = "b1 angular is \n" + b1.massAngular.toString() + " \n" + 
				"b2 angular is \n" + b2.massAngular.toString() + " \n" +
				collectionAngularMass + "\n diff=\n" +
					diff.toString()	; 
				
		EasyViewer.beginOverlay(drawable);
		EasyViewer.printTextLines(drawable, msg, 20, 20, 20, GLUT.BITMAP_8_BY_13 );
		EasyViewer.endOverlay(drawable);
		
	}
	
	@Override
	public void init(GLAutoDrawable drawable) {
		// nothing to do
	}

}
