package mergingBodies3D;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;


public class Spring {

	/** The body to which this spring is attached */
	private RigidBody body;
	/** The point at which this spring is attached in body coordinates */
	private Point3d pb = new Point3d();
	/** The point on the body transformed to world coordinates */
	private Point3d pbw = new Point3d();
	/** The point in the world to which this spring is attached */
	private Point3d pw = new Point3d();

	/** 
	 * Rest length of the spring 
	 * TODO: better to make this a parameter.
	 */
	private double l0 = 0.5;

	/** 
	 * Creates a new body pinned to world spring.
	 * @param pB 	The attachment point in the body frame
	 * @param body	The body to which the spring should be attached
	 */
	public Spring(Point3d pB, RigidBody body) {
		this.body = body;
		this.pb.set( pB );
		body.transformB2W.transform( pb, pw );
		pbw.set( pw );
	}
	
	/**
	 * Consider creating non-zero rest length spring with this constructor
	 * to a pinned location in the world.
	 * TODO: consider likewise springs that go between two bodies!
	 * @param pB
	 * @param body
	 * @param pW
	 */
	public Spring( Point3d pB, RigidBody body, Point3d pW ) {
		
	}
	
	/**
	 * Applies the spring force by adding a force and a torque to the body.
	 * If this body is in a collection, then it applies the force to *BOTH* the sub-body 
	 * and the collection.
	 */
	public void apply(double k, double c) {
		//l0 = RigidBodySystem.springLength.getValue();
		body.transformB2W.transform( pb, pbw );
		Vector3d displacement = new Vector3d();
		Vector3d velocity = new Vector3d(); // velocity of the point on the body
		Vector3d force = new Vector3d();
		displacement.sub( pw, pbw ); 

		// Silly fix... the force should go gracefully to zero without giving NaNs :(
		if ( displacement.length() < 1e-3 ) return;  // hmm... should just give it some rest length?

		body.getSpatialVelocity( pbw, velocity );
//		velocity.set( -pb.y, pb.x );
//		velocity.scale( body.omega );
//		body.transformB2W.transform( velocity );
//		velocity.add( body.v );
		
		double scale = 
				- (k * (displacement.length()  - l0) - 	c * (velocity.dot(displacement) / displacement.length())) 
				/ displacement.length();

		force.scale( - scale, displacement );

		body.applyForceW( pbw, force );
//		if ( body.isInCollection() ) {
//			body.parent.applyForceW( pbw, force );
//		}
		
	}

	/**
	 * Draws the spring end points with a red line through them
	 * @param drawable
	 */
	public void displaySpring( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glLineWidth(2);
		gl.glColor4f(1, 0 ,0, 0.5f);
		gl.glBegin( GL.GL_LINES );
		gl.glVertex3d( pw.x, pw.y, pw.z );
		gl.glVertex3d( pbw.x, pbw.y, pbw.z );
		gl.glEnd();
	}
	
	/** adjust the spring properties */
	public void moveWorldAttachmentAndRestLength( double dx, double dy, double dl ) {
		pw.x += dx;
		pw.y += dy;
		l0 += dl;
		if (l0 < 0 ) l0 = 0;
	}
	
}
