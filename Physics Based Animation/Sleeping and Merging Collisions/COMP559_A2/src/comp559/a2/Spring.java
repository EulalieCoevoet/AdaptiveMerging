package comp559.a2;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;


public class Spring {

	ArrayList<Block> blocks;
	
	Block block1;
	Block block2;
	
    Point2d p1;
    Vector2d v1 = new Vector2d();
    RigidBody b1;
    
    Point2d p2;
    Vector2d v2 = new Vector2d();
    RigidBody b2;
    /** Spring stiffness, sometimes written k_s in equations */
    public static double k = 1;
    /** Spring damping (along spring direction), sometimes written k_d in equations */
    public static double c = 1;
    /** Rest length of this spring */
    double l0 = 0;
    
    public Spring(ArrayList<Block> blocks) {
    	
    	//find outermost blocks
    	this.blocks = blocks;
    	findEndpoints(blocks);
    	computeRestLength();
    	
    
    	return;
    }
    
    public void findBodies(ArrayList<RigidBody> bodies) {
		// TODO Auto-generated method stub
    	
    	for (RigidBody b: bodies) {
    		if (b.blocks.contains(block1)) {
    			b1 = b;
    		}else if(b.blocks.contains(block2)) {
    			b2 = b;
    		}
    		
    	}
    	
		
	}

	private void findEndpoints(ArrayList<Block> blocks) {
		// TODO Auto-generated method stub
		int minimum_row = 1000000000;//very large number
		Block highest_block = null;//
		
		int maximum_row = 0;//very small number
		Block lowest_block = null;//
    	for (Block b: blocks) {
			if (b.i < minimum_row) {
				highest_block = b;
				minimum_row = b.i;
			}
			if (b.i >  minimum_row) {
				lowest_block = b;
				maximum_row = b.i;
			}
		}
    	block1 = highest_block; block2 = lowest_block;
    	p1 = new Point2d(block1.j, block1.i);
    	p2 = new Point2d(block2.j, block2.i);

	}

    
    private void computeVelocities() {
		// TODO Auto-generated method stub
		double r1 = p1.distance(b1.x);
		double r2 = p2.distance(b2.x);
		
		double w1 = b1.omega;
		double w2 = b2.omega;
		
		v1 = new Vector2d(b1.v);
		Vector2d temp = new Vector2d(w1 * v1.y, -w1 * v1.x);
		v1.add(temp);
		
		v2 = new Vector2d(b2.v);
		temp = new Vector2d(w2 * v2.y, -w2 * v2.x);
		v2.add(temp);
		
	}

	/**
     * Computes and sets the rest length based on the original position of the two particles 
     */
    public void computeRestLength() {
        l0 = p1.distance( p2 );
    }
    
    
    /**
     * Applies the spring force by adding a force and a torque to both bodies
     */
    public void apply() {

    	
    	Vector2d displacement, velocity, spring_force;
    	double spring_force_scale;
    	
    	displacement = new Vector2d();
    	velocity = new Vector2d();
    	
    	displacement.sub(p1,  p2); //displacement from a to b
    	velocity.sub(v1,  v2);
    	
    	spring_force_scale = 
    			- (k * (displacement.length() - l0) + 	c * (velocity.dot(displacement) / displacement.length())) 
    			/ displacement.length();
    	   	
    	spring_force = new Vector2d(displacement);	
    		
    	spring_force.scale(spring_force_scale);
    		
    	//adding the forces
    	b1.applyContactForceW(p1, spring_force);
    	spring_force.scale(-1);
    	b2.applyContactForceW(p2, spring_force);
    	
    	
    	
    }
    
}
