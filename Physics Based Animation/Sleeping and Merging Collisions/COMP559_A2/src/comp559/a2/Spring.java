package comp559.a2;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;


public class Spring {

	ArrayList<Block> blocks;
	
	Block block1;
	Block block2;
	
    Point2d p1;
    Vector2d v1 = new Vector2d();
    RigidBody b1;
    
    Point2d p2;
    Point2d p2b; // coordinate of second spring in body frame
    Vector2d v2 = new Vector2d();

    double l0 = 1;


    
    public Spring(Point2d p, RigidBody body) {
		// TODO Auto-generated constructor stub
    	this.l0 = RigidBodySystem.springLength.getValue();
    	this.p1 = new Point2d(p.x, p.y - l0);
    	this.p2 = new Point2d(p);
    	this.b1 = body;
    	this.p2b = new Point2d(p);
    	b1.transformW2B.transform(p2b);
    	
	}

    public void updateP2() {
    	this.p2 = new Point2d(this.p2b);
    	this.l0 = RigidBodySystem.springLength.getValue();
    	this.b1.transformB2W.transform(p2);
    }

    
    public void computeVelocities() {
	
		double r1 = p1.distance(b1.x);

		
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
    	b1.applyContactForceW(p2, spring_force);

    	
    	
    }
    
    /**
     * Draws the spring endpoints with a red line through them
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
    
}
