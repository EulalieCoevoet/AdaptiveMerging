package mergingBodies;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;


public class Spring {

	ArrayList<Block> blocks;

	RigidBody originalBody;
	Point2d p1;
	Vector2d v1 = new Vector2d();
	RigidBody b1;

	Point2d p2;
	Point2d p2b; // coordinate of second spring in body frame
	Vector2d v2 = new Vector2d();

	double l0 = 1;

	Spring originalSpring;

	public Spring(Point2d p, RigidBody body) {
		// TODO Auto-generated constructor stub
		this.l0 = RigidBodySystem.springLength.getValue();
		this.p1 = new Point2d(p.x, p.y - l0);
		this.p2 = new Point2d(p);
		this.b1 = body;
		this.originalBody = body;
		this.p2b = new Point2d(p);
		b1.transformW2B.transform(p2b);
		originalSpring = new Spring(this);
	}
	
	public Spring (Spring s) {   
		// TODO Auto-generated constructor stub
		this.l0 = s.l0;
		this.p1 = s.p1;
		this.v1 = s.v1;
		this.b1 = s.b1;

		this.p2 = s.p2;
		this.p2b = s.p2b;
		this.v2 = s.v2;
		this.originalSpring = s.originalSpring;
		this.originalBody = s.originalBody;
	}

	//create the same spring attached to a new body
	public Spring (Spring s, RigidBody newBody) {   
		// TODO Auto-generated constructor stub
		l0 = s.l0;
		p1 = s.p1;
		v1 = s.v1;
		b1 = newBody;

		p2 = new Point2d(s.p2b);
		s.b1.transformB2W.transform(p2);
		p2b = new Point2d(p2);
		//NEED TO WATCH OUT WITH TRANSFORM
		b1.transformW2B.transform(p2b);
		v2.set(s.v2);
		originalSpring = s.originalSpring;
	}

	public void updateP2() {
		this.p2 = new Point2d(this.p2b);
		this.l0 = RigidBodySystem.springLength.getValue();
		this.b1.transformB2W.transform(p2);
	}
	
	/**
	 * changes the body associated with this spring. Can be useful for applying springs to collections 
	 */
	public void changeBody(RigidBody b) {
		this.b1 = b;
	}

	public void computeVelocities() {
		//double r1 = p1.distance(b1.x);		
		double w1 = b1.omega;
		v2 = new Vector2d(b1.v);
		Vector2d temp = new Vector2d(w1 * v2.y, -w1 * v2.x);
		v2.add(temp);
	}

	/**
	 * Applies the spring force by adding a force and a torque to both bodies
	 */
	public void apply(double k, double c) {

		Vector2d displacement, velocity, spring_force;
		double spring_force_scale;

		displacement = new Vector2d();
		velocity = new Vector2d();

		displacement.sub(p1,  p2); //displacement from a to b
		velocity.sub(v1,  v2);

		spring_force_scale = 
				- (k * (displacement.length()  - l0) + 	c * (velocity.dot(displacement) / displacement.length())) 
				/ displacement.length();

		spring_force = new Vector2d(displacement);	

		spring_force.scale(-spring_force_scale);

		//adding the forces
		b1.applyForceW(p2, spring_force);
	}

	/**
	 * Draws the spring end points with a red line through them
	 * @param drawable
	 */
	public void displayConnection( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		// draw a line between the two bodies but only if they're both not pinned

		gl.glLineWidth(2);
		gl.glColor4f(1, 0 ,0, 0.5f);
		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(p1.x, p1.y);
		gl.glVertex2d(p2.x, p2.y);
		gl.glEnd();

	}

	public void reset() {
		// TODO Auto-generated constructor stub
		this.l0 = originalSpring.l0;
		this.p1 =  originalSpring.p1;
		this.v1 = originalSpring.v1;
		this.b1 =  originalSpring.b1;

		this.p2 =  originalSpring.p2;
		this.p2b =  originalSpring.p2b;
		this.v2 =  originalSpring.v2;
	}
}
