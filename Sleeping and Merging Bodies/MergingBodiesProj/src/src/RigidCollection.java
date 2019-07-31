package src;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class RigidCollection extends RigidBody{
	
	public LinkedList<RigidBody> colRemovalQueue = new LinkedList<RigidBody>();

	ArrayList<RigidBody> collectionBodies = new ArrayList<RigidBody>();
	
	//ArrayList<Contact> inner_contacts = new ArrayList<Contact>();
	
	ArrayList<BodyContact> internalBodyContacts = new ArrayList<BodyContact>();
	
	boolean unMergedThisTimestep = false;

	
	public RigidCollection(RigidBody body1, RigidBody body2) {
		
		super(body1); // this will copy the blocks, which is not exactly what we want... fine though.
		
		clearUselessJunk();
		
		index = body1.index;
		collectionBodies.add(body1);
		collectionBodies.add(body2);
		
		setupCollection();
		
		body1.parent = this;
		body2.parent = this;
		body1.merged = true;
		body2.merged = true;
		
	}

	
	private void clearUselessJunk() {
		blocks.clear();
		boundaryBlocks.clear();
		
		active_past.clear();
		contactList.clear();
		bodyContactList.clear();
		contactForces.clear();
		contactTorques = 0;
		springs.clear();
	}


	/**
	 * adds Body to the collection
	 * @param bc 
	 */
	public void addBody(RigidBody body) {
		
		collectionBodies.add(body);
		setupCollection();
		
		body.parent = this;
		body.merged = true;
		
	}
	
	
	
	public void addInternalContact(BodyContact bc) {
		internalBodyContacts.add(bc);

	}
	//like addBody but with another collection...
	public void addCollection(RigidCollection col) {
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
		for (RigidBody b : col.collectionBodies) {
			//transform all the subBodies to their world coordinates... 
			b.merged = true;
			col.transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.parent = null;
			additionQueue.add(b);
			
			
		}
		
		for (RigidBody b: additionQueue) {
			collectionBodies.add(b);
		}
		col.collectionBodies.clear();
		
		setupCollection();
		
		
		internalBodyContacts.addAll(col.internalBodyContacts);
	}
	
	/** 
	 * seperates each collectionBody from its parent so the bodies are ready to be added back to the system individually
	 * x positions now need to be in world coordinates etc.
	 */
	public void unmergeAllBodies() {
		internalBodyContacts.clear();
		for (RigidBody b: collectionBodies) {
			b.bodyContactList.clear();
			b.contactList.clear();
			transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.v.set(v);
			b.omega = omega;
			b.parent = null;
		}
		
	}
	
	@Override
	public void advanceTime(double dt){
    	
		if (!pinned) {
				    	
			v.x += force.x * dt/massLinear + delta_V.get(0);
	    	v.y += force.y * dt/massLinear + delta_V.get(1);
	    	omega += torque * dt/ massAngular + delta_V.get(2);
	    	
	       	x.x += v.x * dt;
	    	x.y += v.y * dt;
	    	theta += omega*dt;
	    	
	    	updateTransformations();
	    	updateCollectionBodyTransformations(dt);
	    		    
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

	@Override
    /**
     * Updates the B2W and W2B transformations
     */
    public void updateTransformations() {
        transformB2W.set( theta, x );
        transformW2B.set(transformB2W);
        transformW2B.invert();
      
    }
	
	private void updateCollectionBodyTransformations(double dt) {
		//WHY doesn't this work!
		for (RigidBody b: collectionBodies) {
	       	b.transformB2W.set(b.transformB2C);
	       	b.transformB2W.leftMult(transformB2W);
	       	b.transformW2B.set(b.transformB2W); b.transformW2B.invert();
		}

    	
	}
    
	/*For each body in collection, determine the transformations to go from body to collection
	*But also, make each body's x, in collection and theta in collection, relative to this x and theta
	*/
	
	private void setMergedTransformationMatrices() {
				
		for (RigidBody body: collectionBodies) {
			body.transformB2C.set(body.transformB2W);
			body.transformB2C.leftMult(transformW2B);
			body.transformC2B.set(body.transformB2C); body.transformC2B.invert();
	
	
		}

		
		
	}
	
	/*
	 * transforms body x's and thetas in world coordinates into collection cooridnates
	 */
	private void transformToCollectionCoords(){
		RigidTransform temp = new RigidTransform();
		for (RigidBody b : collectionBodies) {
			
			transformW2B.transform(b.x);
			
		 	b.theta =  b.transformB2C.getTheta();
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


	public void calculateMass() {
		double mass = 0;
		for(RigidBody b: collectionBodies) {
			mass += b.massLinear;
		}
		massLinear = mass;
		minv = 1/mass;
	}
	

	
	/**
	 * Loops through all bodies in collectionBodies and sets the transformation matrices for each
	 */
	public void calculateCOM() {
		Vector2d com = new Vector2d();
		Vector2d bCOM = new Vector2d();
		double totalMass = massLinear;
		com.set(0,0);
	
		
		for (RigidBody b: collectionBodies) {
			double bRatio = b.massLinear/totalMass;
			if (b.parent != null) {
				//transform back to world if the body is in collection coordinates... dont worry, very temporary
				b.parent.transformB2W.transform(b.x);
				
			}
			bCOM.scale(bRatio, b.x);
			com.add(bCOM);
			
			
		
		}
		
	
	//	set collections new transformations...
		x.set(com);
		
	    transformB2W.set( theta, x );
	    transformW2B.set( transformB2W);
	    transformW2B.invert();
	}
	
	/**
	 * Calculates the angular inertia. 
	 */
	public void calculateInertia() {
		double inertia = 0;
		Point2d bpB = new Point2d(0, 0);
		Point2d zero = new Point2d(0, 0);
		for (RigidBody body: collectionBodies) {
			   for ( Block b : body.blocks ) {
		            double mass = b.getColourMass();
		            bpB.set(b.pB);
		            body.transformB2C.transform(bpB);
		            //b.pb is in c
		            inertia += mass*bpB.distanceSquared(zero);
		            
		        }
			   body.parent = this;
		
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
    
    //list of bodies to be added to this collection in the next timestep
	ArrayList<RigidBody> bodyQueue = new ArrayList<RigidBody>();

    
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

    
    /*
     * goes through each body in collection and sees if it should be unmerged. Fill the removal queue with the bodies that need to be unmerged
     */
    public void fillRemovalQueue(Vector2d totalForce, double totalTorque, double forceMetric){
    	for (RigidBody sB : collectionBodies) {
    		checkMetric(sB, totalForce, totalTorque, forceMetric);
			
		}
    }
    
    
    /*c
     * checks if body sB is going to unmerge by comparing the acceleration vector magnitude with a threshold
     */
    
    private void checkMetric(RigidBody sB, Vector2d totalForce, double totalTorque, double forceMetric) {
    	totalForce.set(sB.force);
		sB.transformB2W.transform(sB.contactForce);
		totalForce.add(sB.contactForce);
		sB.transformW2B.transform(sB.contactForce);
		totalTorque = sB.torque + sB.contactTorques;
		forceMetric = Math.sqrt(Math.pow(totalForce.x,2 ) + Math.pow(totalForce.y, 2))/sB.massLinear + Math.sqrt(Math.pow(totalTorque, 2))/sB.massAngular;
		
		if (forceMetric > CollisionProcessor.impulseTolerance.getValue()) {
			colRemovalQueue.add(sB);
			checkSubBodyNeighbors(sB, totalForce, totalTorque, forceMetric);
		}
	}


/*
     * for each neighbor of sB, check whether or not that neighbor is still awake by applying the previous contact forces.
     */
private void checkSubBodyNeighbors(RigidBody sB, Vector2d totalForce, double totalTorque, double forceMetric) {
		for (BodyContact bc : sB.bodyContactList) {
			if (!bc.merged) continue;
			
			if (sB.equals(bc.thisBody)) {
				checkMetric(bc.otherBody, totalForce, totalTorque, forceMetric);
			}else {
				checkMetric(bc.thisBody, totalForce, totalTorque, forceMetric);
			}
		}
		
	}


/**
 * Removes a body from the current collection, without changing the other Bodies
 */
	public void unmergeSelectBodies() {
		// TODO Auto-generated method stub

		for (RigidBody b : colRemovalQueue) {
			b.bodyContactList.clear();
			b.contactList.clear();
			transformB2W.transform(b.x);
			b.theta = b.transformB2W.getTheta();
			b.transformB2C.T.setIdentity();
			b.transformC2B.T.setIdentity();
			b.v.set(v);
			b.omega = omega;
			b.parent = null;
			collectionBodies.remove(b);
			
			/*int i = 0;
			while (true) {
				BodyContact bc = bodyContactList.get(i);
				if (bc.thisBody == b || bc.otherBody== b) {
					bodyContactList.remove(bc);
					if (i >= bodyContactList.size()) break;
					continue;
				}
				i++;
				if (i >= bodyContactList.size()) break;
			}*/
		}
		colRemovalQueue.clear();
		//reset up the collection
		setupCollection();

		
	}

/*basic setup of the collection... calculates transforms, COM, inertia , etc.
 * 
 */
	private void setupCollection() {
		calculateMass();
	
		calculateCOM();
		//addBlocks(body);
		//from this new collection COM determine new transforms for each body in the collection
		//set Transform B2C and C2B for each body
		setMergedTransformationMatrices();
		transformToCollectionCoords();
	
		calculateInertia();
		//update BVNode roots for each body

		springs.clear();
		addSprings();
}
	




	
}
