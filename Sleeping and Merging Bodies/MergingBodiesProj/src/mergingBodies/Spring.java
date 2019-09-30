package mergingBodies;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;


public class Spring {

	/** The body to which this spring is attached */
	private RigidBody body;
	/** The point at which this spring is attached in body coordinates */
	private Point2d pb = new Point2d();
	/** The point on the body transformed to world coordinates */
	private Point2d pbw = new Point2d();
	/** The point in the world to which this spring is attached */
	private Point2d pw = new Point2d();

	/** Rest length of the spring */
	private double l0 = 0;

	/** 
	 * Creates a new body pinned to world spring.
	 * @param p 	The attachment point in the world
	 * @param body	The body to which the spring should be attached
	 */
	public Spring(Point2d p, RigidBody body) {
		this.body = body;
		pw.set( p );
		body.transformW2B.transform( pw, pb );
		pbw.set( p );
	}
	
	/**
	 * Applies the spring force by adding a force and a torque to the body.
	 * If this body is in a collection, then it applies the force to *BOTH* the sub-body 
	 * and the collection.
	 */
	public void apply(double k, double c) {
		l0 = RigidBodySystem.springLength.getValue();
		body.transformB2W.transform( pb, pbw );
		final Vector2d displacement = new Vector2d();
		final Vector2d velocity = new Vector2d(); // velocity of the point on the body
		final Vector2d force = new Vector2d();

		displacement.sub( pw, pbw ); 

		velocity.set( -pb.y, pb.x );
		velocity.scale( body.omega );
		body.transformB2W.transform( velocity );
		velocity.add( body.v );
		
		double scale = 
				- (k * (displacement.length()  - l0) + 	c * (velocity.dot(displacement) / displacement.length())) 
				/ displacement.length();

		force.scale( - scale, displacement );

		body.applyForceW( pbw, force );
		if ( body.parent != null ) {
			body.parent.applyForceW( pbw, force );
		}
	}

	/**
	 * Draws the spring end points with a red line through them
	 * @param drawable
	 */
	public void displayConnection( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glLineWidth(2);
		gl.glColor4f(1, 0 ,0, 0.5f);
		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(pw.x, pw.y);
		gl.glVertex2d(pbw.x, pbw.y);
		gl.glEnd();
	}
}
