package comp559.a2;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class RigidCollection extends RigidBody{

	ArrayList<RigidBody> collectionBodies = new ArrayList<RigidBody>();
	
	ArrayList<Contact> inner_contacts = new ArrayList<Contact>();
	
	public RigidCollection(RigidBody body) {
		
		super(body); // this will copy the blocks, which is not exactly what we want... fine though.
	/*	body.x.set(0,0);
		body.theta = 0;
		body.updateTransformations(); */
		
		this.blocks.clear();
		this.boundaryBlocks.clear();
		
		addBlocks(body);
		
		collectionBodies.add(body);
		this.index = body.index;
		this.active_past.clear();
		this.contact_list.clear();
		this.body_contact_list.clear();
		this.contactForces.clear();
		this.contactTorques.clear();
		
		root = new BVNode(this.boundaryBlocks, this);
		//
		springs.clear();
		addSprings();
	

	//	this.created = true;
	
		
	}
	


	@Override
	public void advanceTime(double dt){
    	
		if (!pinned) {
			v.x += force.x * dt/massLinear;
	    	v.y += force.y * dt/massLinear;
	    	omega += torque * dt/ massAngular;
	    	
	       	x.x += v.x * dt;
	    	x.y += v.y * dt;
	    	theta += omega*dt;
	    	
	    	updateTransformations();
	    	force.set(0, 0);
	    	torque = 0;
	    	
	    
	    	//advance time for each sub rigid bodies so the circles can move?
	    	/*for (RigidBody b: collectionBodies) {
	    		if (!b.pinned) {
		    		//get radius between body COM and this COM to cross product with omega
		    		Vector2d r = new Vector2d(-b.x.y + this.x.y, b.x.x - this.x.x );
		    		r.scale(omega);
		    		
		    		b.v.x += v.x + r.x;
		    		b.v.y += v.y +r.y;
		    		b.omega += omega;
		    		
		    		b.x.x += b.v.x*dt;
		    		b.x.y += b.v.y * dt;
		        	b.theta += b.omega*dt;
		        	b.updateTransformations();
		        
	    		}
	    	} */
	  
		}

	}
	
	//applies springs on the body, to the collection
	private void addSprings() {
		ArrayList<Spring> newSprings = new ArrayList<Spring>();
		
		for (RigidBody body: collectionBodies) {
			for (Spring s: body.springs) {
				Spring newSpring = new Spring(s, this);
				newSprings.add(newSpring);
			
			}
		}
	
		this.springs.addAll(newSprings);
		
	}

	/*
	 * adds Body to the collection
	 */
	public void addBody(RigidBody body) {
		
		collectionBodies.add(body);
		if (body.pinned) this.pinned = true;  // probably want to do something different for pinned.
		
		
		this.calculateMass();
		
		this.calculateCOM();
		
		//from this new collection COM determine new transforms for each body in the collection
		//set Transform B2C and C2B for each body
		updateMergedTransformationMatrices();
	

		calculateInertia();
		
		root = new BVNode(this.boundaryBlocks, this);
		
		this.springs.clear();
		addSprings();
		
	}
	
	@Override
    /**
     * Updates the B2W and W2B transformations
     */
    public void updateTransformations() {
        transformB2W.set( theta, x );
        transformW2B.set( theta, x );
        transformW2B.invert();
        /*for(RigidBody b: collectionBodies) {
        	b.transformB2W.set(b.transformB2C);
        	b.transformB2W.leftMult(transformB2W);
        	b.transformW2B.set(transformB2W); b.transformW2B.
        }*/
    }
    
	/*For each body in collection, determine the transformations to go from body to collection
	*But also, make each body's x, in collection and theta in collection, relative to this x and theta
	*/
	private void updateMergedTransformationMatrices() {
		RigidTransform temp = new RigidTransform();
		for (RigidBody body: collectionBodies) {
			body.transformB2C.set(body.transformB2W);
			body.transformB2C.leftMult(transformW2B);
			body.transformC2B.set(body.transformB2C); body.transformC2B.invert();
			
			//set x and theta
			temp.set(body.transformW2B);
			temp.leftMult(body.transformB2C);
			
			temp.transform(body.x);
			
			body.theta = body.transformB2C.getTheta();
			//body.theta = transformW2B.getTheta();
		}
		
	}
	@Override
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glPushMatrix();
        gl.glTranslated( x.x, x.y, 0 );
        gl.glRotated(theta*180/Math.PI, 0,0,1);
        
        for (RigidBody b: collectionBodies) {
        	b.display(drawable);
        }
        gl.glPopMatrix();
	}


	//Goes through each block and changes the position of block in body coordinates
	private void changeBlockCoordinates() {
		//blocks are in body frame. for them to be in the correct position/orientation, we need to rotate
		//them in local frame first and THEN translate them.
		for (RigidBody b: collectionBodies) {
			
			for (Block bl : b.blocks) {
		
				b.transformB2C.transform(bl.pB);
				
			
			}
		}
	}

	/*
	@Override 
	public void reset() {
		for (RigidBody b: collectionBodies) {
			//collectionBodies.remove(b);
			
			b.reset();
			CollisionProcessor.bodies.add(b);
			
			
		}
		collectionBodies.clear();
		CollisionProcessor.bodies.remove(this);
	} */
	private void addBlocks(RigidBody body) {
		this.boundaryBlocks.addAll(body.boundaryBlocks);
		this.blocks.addAll(body.blocks);
		
	}

	public void calculateMass() {
		double mass = 0;
		for(RigidBody b: collectionBodies) {
			mass += b.massLinear;
		}
		massLinear = mass;
		minv = 1/mass;
	}
	

	
	/*
	 * Loops through all bodies in collectionBodies
	 */
	public void calculateCOM() {
		Vector2d com = new Vector2d();
		Vector2d bCOM = new Vector2d();
		double totalMass = massLinear;
		com.set(0,0);
		for (RigidBody b: collectionBodies) {
			double bRatio = b.massLinear/totalMass;
			bCOM.scale(bRatio, b.x);
			com.add(bCOM);
		}
		/*for (RigidBody b: collectionBodies) {
			b.x.sub(com);
			b.updateTransformations();
		}*/
		
		x.set(com);
	    transformB2W.set( theta, x );
	    transformW2B.set( theta, x );
	    transformW2B.invert();
	}
	
	/**
	 * Calculates the angular inertia. 
	 */
	public void calculateInertia() {
		double inertia = 0;
		Point2d zero = new Point2d(0, 0);
		for (RigidBody body: collectionBodies) {
			   for ( Block b : blocks ) {
		            double mass = b.getColourMass();
		            inertia += mass*b.pB.distanceSquared(zero);
		        }
			//double distance = body.x.distance(this.x);
			//double mass = body.massLinear*distance*distance;
			
			
		}
		this.massAngular = inertia;
		this.jinv = 1/inertia;
		
	}

    /**
     * Draws the center of mass position with a circle.  
     * @param drawable
     */
    public void displayCOM( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
          
	    gl.glPointSize(8);
	    gl.glColor3f(0,0,0.7f);
	    gl.glBegin( GL.GL_POINTS );
	    gl.glVertex2d(x.x, x.y);
	    gl.glEnd();
	    gl.glPointSize(4);
	    gl.glColor3f(1,1,1);
	    gl.glBegin( GL.GL_POINTS );
	    gl.glVertex2d(x.x, x.y);
	    gl.glEnd();

    }
    /** Map to keep track of display list IDs for drawing our rigid bodies efficiently */
    static private HashMap<ArrayList<Block>,Integer> mapBlocksToDisplayList = new HashMap<ArrayList<Block>,Integer>();
    
    /** display list ID for this rigid body */
    int myListID = -1;

    
    /**
     * displays the Body Collection as lines between the center of masses of each rigid body to the other. 
     * Uses a string arrayList to check if a connection has already been drawn.
     * @param drawable
     */
    
    public void displayCollection( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        ArrayList<String> connections = new ArrayList<String>();
        // draw a line between the two bodies but only if they're both not pinned
        for (RigidBody b1: collectionBodies) {
        	for (RigidBody b2: collectionBodies) {
        		String option1 = Integer.toString(b1.index) + "_" + Integer.toString(b2.index);
        		String option2 = Integer.toString(b2.index) + "_" + Integer.toString(b1.index);
        		if (!b1.equals(b2) && (!(connections.contains(option1)||connections.contains(option2)))) {
        			gl.glLineWidth(1);
        			gl.glColor4f(1, 0,0, 0.5f);
        			gl.glBegin( GL.GL_LINES );
        			gl.glVertex2d(b1.x.x, b1.x.y);
        			gl.glVertex2d(b2.x.x, b2.x.y);
        			gl.glEnd();
        			connections.add(Integer.toString(b1.index) + "_" + Integer.toString(b2.index));
        		}
        	}
        }

        
    }
}
