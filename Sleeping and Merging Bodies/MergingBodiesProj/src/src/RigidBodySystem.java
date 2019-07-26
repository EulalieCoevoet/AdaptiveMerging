package src;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

/**
 * Maintains a list of RigidBody objects, and provides methods for collision processing and numerical integration
 * @author kry
 */
public class RigidBodySystem {

	public String name ="";
	
    public double simulationTime = 0;
    
	public ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
	
	public ArrayList<RigidBody> originalBodies = new ArrayList<RigidBody>();
    
    
	public CollisionProcessor collisionProcessor = new CollisionProcessor(bodies);
	
	public PendulumProcessor pendulumProcessor = new PendulumProcessor(bodies);
	
	public Pendulum pendulum = new Pendulum(this);
    
    public MouseSpringForce mouseSpring;
    
    BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
    
    DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
    
    DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );
    
    /**
     * Stiffness of  spring
     */
    public DoubleParameter spring_k = new DoubleParameter("spring stiffness", 100, 1, 1e4 );
    
    /**
     * Viscous damping coefficient for the  spring
     */
    public DoubleParameter spring_c= new DoubleParameter("spring damping", 0, 0, 1000 );
    
    /**
     * Viscous damping coefficient for the  spring
     */
    public static IntParameter springLength= new IntParameter("spring rest length", 1, 1, 100 );
    
    
    
    /**
     * Creates a new rigid body system
     */
    public RigidBodySystem() {
        /* do nothing */
    }
    
    /**
     * Adds a rigid body to the system 
     * @param body
     */
    public void add( RigidBody body ) {
        bodies.add( body );
    }
    
    /**
     * Applies a small random acceleration to all bodies
     */
    public void jiggle() {
        final Random rand = new Random();
        for ( RigidBody b : bodies ) {
            if ( b.pinned ) continue;
            b.omega += rand.nextDouble()*2-1;
            b.v.x += rand.nextDouble()*2-1;
            b.v.y += rand.nextDouble()*2-1;    
  
        }
    }
    
    /** 
     * Time in seconds to advance the system
     */
    public double computeTime;
    
    /**
     * Total time in seconds for computation since last reset
     */
    public double totalAccumulatedComputeTime;
    
    /**
     * Advances the state of all rigid bodies
     * @param dt time step
     */
    public void advanceTime( double dt ) {
    	 
    
    	long now = System.nanoTime();        
    	
    	
  
        // apply gravity to all bodies... also take this opportunity to clear all forces at the start of the timestep
        if ( useGravity.getValue() ) {
            Vector2d force = new Vector2d();
            for ( RigidBody b : bodies ) {
            	//fully active, regular stepping
            	
            	clearJunkAtStartOfTimestep(b);
                double theta = gravityAngle.getValue() / 180.0 * Math.PI;
                force.set( Math.cos( theta ), Math.sin(theta) );
                force.scale( b.massLinear * gravityAmount.getValue() );
                // gravity goes directly into the accumulator!  no torque!
                b.force.add( force );
                //apply force of gravity to all children as well
                if( b instanceof RigidCollection) {
                	for (RigidBody sB : ((RigidCollection) b).collectionBodies) {
                		clearJunkAtStartOfTimestep(sB);
                        force.set( Math.cos( theta ), Math.sin(theta) );
                        force.scale( sB.massLinear * gravityAmount.getValue() );
                        sB.force.add( force );
                	}
                }
                
            }
        }
        
        mouseSpring.apply();
        
        //deal with zero length springs
       for (RigidBody b: bodies){
	        for (Spring s: b.springs) {
	        	s.updateP2();
	        	s.computeVelocities();
	        	
	        	s.apply(spring_k.getValue(), spring_c.getValue());
	        	s.updateP2();
	        
	        }
	        //also apply spring forces to the child
	        if (b instanceof RigidCollection) {
	        	for (RigidBody sB : ((RigidCollection) b).collectionBodies) {
	        		for (Spring s: sB.springs) {
	    	        	s.updateP2();
	    	        	s.computeVelocities();
	    	        	
	    	        	s.apply(spring_k.getValue(), spring_c.getValue());
	    	        	s.updateP2();
	    	        
	    	        }
	        	}
	        }
       }
       
       
        if ( processCollisions.getValue() ) {
            // process collisions, given the current time step
        	//apply all forces present on collecitonbodies first:
             collisionProcessor.processCollisions( dt );
            
        }
        
        if (enableMerging.getValue()) {
        	applyContactForces();
        	applyExternalForces();
        }

   
        if (use_pendulum.getValue()) {
        	Point2d origin = new Point2d(Pendulum.origin_x.getValue(), Pendulum.origin_y.getValue());
        	pendulumProcessor.processPendulum(dt, origin, Pendulum.pendulum_length.getValue());
        }
     
        if (enableMerging.getValue()) {
        	// Put your body merging stuff here?  
            mergeBodies();
            checkIndex();
            
        }
        
        // advance the system by the given time step
        for ( RigidBody b : bodies ) {

            b.advanceTime(dt);
        }
        
        if (enableMerging.getValue()) {
        	unmergeBodies();
        	//checkIndex();
        }
        
    	if (this.generateBody) {
    		generateBody();
    		this.generateBody = false;
    	}
    	
        computeTime = (System.nanoTime() - now) / 1e9;
        simulationTime += dt;
        totalAccumulatedComputeTime += computeTime;
    }

    
   private void clearJunkAtStartOfTimestep(RigidBody b) {
		
    	b.force.set(0, 0);
    	b.torque = 0;
    	b.delta_V.zero();
		b.contactForce.set(0, 0);
    	b.contactTorques = 0;
    	if (b instanceof RigidCollection) {

    		b.bodyContactList.clear();
    		b.contactList.clear();
   		}
	}

private void applyExternalForces() {
		// TODO Auto-generated method stub
		
	}

/**
    * goes through each contact, now with their appropriate lamda values, and adds the contact forces+torque 
    * to each rigidBody.
    */
	    private void applyContactForces() {

		Vector2d cForce = new Vector2d();
		double cTorque= 0;
	    for (Contact c: collisionProcessor.contacts) {
	    	cForce.set(c.lamda.x*c.j_1.get(0) + c.lamda.y*c.j_2.get(0),c.lamda.x*c.j_1.get(1) + c.lamda.y*c.j_2.get(1) );
	    	cTorque = c.lamda.x*c.j_1.get(2) + c.lamda.y*c.j_2.get(2);
	    	c.body1.contactForce.add(cForce);
	    	c.body1.contactTorques += cTorque;
	    	
	    	//if Body1 is a parent, also apply the contact force to the appropriate subBody
	    	if (c.body1 instanceof RigidCollection) {
	    		applyContactForceToSubBody(c, (RigidCollection) c.body1, cForce);
	    	}
	    	
	    	cForce.set(c.lamda.x*c.j_1.get(3) + c.lamda.y*c.j_2.get(3),c.lamda.x*c.j_1.get(4) + c.lamda.y*c.j_2.get(4) );
	    	cTorque = c.lamda.x*c.j_1.get(5) + c.lamda.y*c.j_2.get(5);
	    	c.body2.contactForce.add(cForce);
	    	c.body2.contactTorques += cTorque;
	    	
	    	if (c.body2 instanceof RigidCollection) {
	    		applyContactForceToSubBody(c, (RigidCollection) c.body2, cForce);
	    	}
	    	//if Body2 is a parent, also apply the contact force to the appropriate subBody
		}
	    
	}

	    /*
	     * takes the collection, applies the appropriate contact force to the subBody
	     * the normal and tangential contact force components will be the same...
	     * but the rotational ones will be different. 
	     * 
	     */
	    private void applyContactForceToSubBody(Contact c, RigidCollection body, Vector2d cForce) {
	// TODO Auto-generated method stub
	    	//get new cTorque
			double cTorque= 0;
	    	if (c.bc.thisBody.parent == body) {
	    		c.bc.thisBody.contactForce.add(cForce);
	    		
	    		double jn_omega, jt_omega;
	    		
	    		Point2d radius_i = new Point2d(c.bc.thisBody.x);
	    		body.transformB2W.transform(radius_i);
	    		
	    		radius_i.sub(c.contactW, radius_i);
	    		
	    		Vector2d tangeant = new Vector2d(-c.normal.y, c.normal.x);
	    			
	    		Vector2d r1 = new Vector2d(-radius_i.y, radius_i.x);
	    		
	    		jn_omega = - r1.dot(c.normal);
	    		jt_omega = - r1.dot(tangeant);
	    		if (body == c.body2) {
	    			jn_omega *= -1;
	    			jt_omega *= -1;
	    			
	    		}
	    		
	    		cTorque = c.lamda.x*jn_omega + c.lamda.y*jt_omega;
	    		c.bc.thisBody.contactTorques += cTorque;
	    		
	    	}
	    	if (c.bc.otherBody.parent == body) {
	    		c.bc.otherBody.contactForce.add(cForce);
	    		
	    		double jn_omega, jt_omega;
	    		
	    		Point2d radius_i = new Point2d(c.bc.otherBody.x);
	    		body.transformB2W.transform(radius_i);
	    		
	    		radius_i.sub(c.contactW, radius_i);
	    		
	    		Vector2d tangeant = new Vector2d(-c.normal.y, c.normal.x);
	    			
	    		Vector2d r1 = new Vector2d(-radius_i.y, radius_i.x);
	    		
	    		jn_omega = - r1.dot(c.normal);
	    		jt_omega = - r1.dot(tangeant);
	    		if (body == c.body2) { //contact normal may be in the wrong direction based on which body in Contact it is
	    			jn_omega *= -1;
	    			jt_omega *= -1;
	    			
	    		}
	    		
	    		cTorque = c.lamda.x*jn_omega + c.lamda.y*jt_omega;
	    		c.bc.otherBody.contactTorques += cTorque;
	    		
	    	}

	    }

		/*
	     * method that deals with unmerging rigidBodies... because we will explore different solutions to
	     * this problem, it will call different methods for each unmerging solution.
	     * 
	     */
		private void unmergeBodies() {
		//	generalHeuristic();
			generalOneBodyAtATime();
			
			
			
			//forceClosureMethod();
		}
		/**
		 *	 goes through all bodies in each collection. 
			If a body has enough force acting on it (over a threshold), seperate just that body
			from the rest of the collection. 
		 */
		private void generalOneBodyAtATime() {
			LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
			LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
	    	Vector2d totalForce = new Vector2d();
	    	double totalTorque = 0;
	    	double forceMetric = 0;
			for(RigidBody b : bodies) {
	    		//check if force on Collection is high enough. If it is... unmerge the entire rigidCollection
	    		if (b instanceof RigidCollection) {
	    			
	    			for (RigidBody sB : ((RigidCollection) b).collectionBodies) {
	    				totalForce.set(sB.force);
	    				totalForce.add(sB.contactForce);
	    				totalTorque = sB.torque + sB.contactTorques;
	    				forceMetric = Math.sqrt(Math.pow(totalForce.x,2 ) + Math.pow(totalForce.y, 2))/sB.massLinear + Math.sqrt(Math.pow(totalTorque, 2))/sB.massAngular;
	    				
	    				if (forceMetric > CollisionProcessor.impulseTolerance.getValue()) {
		    				((RigidCollection) b).colRemovalQueue.add(sB);
		    				
		    			}
	    			}
	    			if (!((RigidCollection) b).colRemovalQueue.isEmpty()) {
	    				additionQueue.addAll(((RigidCollection) b).colRemovalQueue);
	    				((RigidCollection) b).unmergeSelectBodies();
	    			}
	    			if (((RigidCollection) b).collectionBodies.size() == 1) {
	    				//only one body left in collection... time to unmerge everything
	    				removalQueue.add(b);
	    				additionQueue.add(((RigidCollection) b).collectionBodies.get(0));
	    				((RigidCollection) b).unmergeAllBodies();
	    				
	    			}
	 
	    		}
	    		
	    	}
			
			for (RigidBody b: additionQueue) {
				bodies.add(b);
			}
			for (RigidBody b : removalQueue) {
				bodies.remove(b);
			}
			
		}

		/**
		 * The idea is to check at every timestep, the force acting on the system including:
		 * Gravity, Spring, Collision Lamdas, etc.
		 * 
		 * We then go through each body in the collection, and for each body we go through each
		 * contact and see if the new applied force at that contact location lies 
		 * outside of the friction cone. If yes, then unmerge that body... (may need to recurse)
		 * if no, then dont unmerge
		 */
		private void forceClosureMethod() {
			// TODO Fill this method out		
		}

		/**goes through bodies and sees if any collection should be unmerged, and then
		 * unmerges ALLL bodies in that collecion with no discrimination
		 * 
		 */
		
	    private void generalHeuristic() {
	    	LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
			LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
	    	Vector2d totalForce = new Vector2d();
	    	double totalTorque = 0;
			for(RigidBody b : bodies) {
	    		//check if force on Collection is high enough. If it is... unmerge the entire rigidCollection
	    		if (b instanceof RigidCollection) {
	    	
	    			totalForce.set(b.force);
	    			totalForce.add(b.contactForce);
	    			totalTorque = b.torque + b.contactTorques;
	    			
	    			double forceMetric = Math.sqrt(Math.pow(totalForce.x,2 ) + Math.pow(totalForce.y, 2))/b.massLinear + Math.sqrt(Math.pow(totalTorque, 2))/b.massAngular;
	    			if (forceMetric > CollisionProcessor.impulseTolerance.getValue()) {
	    				((RigidCollection) b).unmergeAllBodies();
	    				additionQueue.addAll(((RigidCollection) b).collectionBodies);
	    				removalQueue.add(b);
	    			}
	    			
	    		
	    		}
	    		
	    	}
			
			for (RigidBody b: additionQueue) {
				bodies.add(b);
			}
			for (RigidBody b : removalQueue) {
				bodies.remove(b);
			}
		}

		/**
	     * Merges all rigidBodies in the system that fit the appropriate criteria: 
	     * 1. They have been in contact for 50 timesteps
	     * 2. The relative velocities of the two bodies in contact has been below the CollisionProcessor.sleep_accum
	     * 		value for the ENTIRETY of the contact.
	     * 
	     */
		public void mergeBodies() {
    	LinkedList<BodyContact> removalQueue = new LinkedList<BodyContact>();
		for (BodyContact bc: collisionProcessor.bodyContacts) {
			boolean mergeCondition = false;
			if ((bc.relativeVelHistory.size() == CollisionProcessor.sleep_accum.getValue())) {
				mergeCondition = true;
				for (Double relVel : bc.relativeVelHistory) {
					if (relVel > CollisionProcessor.sleepingThreshold.getValue()) {
						mergeCondition = false;
					}
				}
			}
			if (bc.thisBody.pinned || bc.thisBody.pinned) mergeCondition = false;
			if (mergeCondition) {
				//if they are both not collections...make a new collection!
				if(bc.thisBody.parent == null && bc.otherBody.parent == null) {
					bodies.remove(bc.thisBody); bodies.remove(bc.otherBody);
					RigidCollection col = new RigidCollection(bc.thisBody, bc.otherBody);
					col.addInternalContact(bc);
					bodies.add(col);
				}else if (bc.thisBody.parent != null && bc.otherBody.parent != null) {
					// if they are BOTH collections... think about what to do
					//take all the bodies in the least massive one and add them to the collection of the most massive
					if (bc.thisBody.parent.massLinear > bc.otherBody.parent.massLinear) {
						bodies.remove(bc.otherBody.parent);
						bc.thisBody.parent.addCollection(bc.otherBody.parent);
						
					}else {
						bodies.remove(bc.thisBody.parent);
						bc.otherBody.parent.addCollection(bc.thisBody.parent);
						
					}
				}else if (bc.thisBody.parent != null) {
					//thisBody is in a collection... otherBody isnt
					bodies.remove(bc.otherBody);
					bc.thisBody.parent.addBody(bc.otherBody);
					bc.thisBody.parent.addInternalContact(bc);
					
				}else if (bc.otherBody.parent != null) {
					//otherBody is in a collection... thisBody isnt
					bodies.remove(bc.thisBody);
					bc.otherBody.parent.addBody(bc.thisBody);
					bc.otherBody.parent.addInternalContact(bc);
					
				}
				removalQueue.add(bc);
				checkIndex();
				
			}
		
			
		}
		for (BodyContact element : removalQueue) {
			collisionProcessor.bodyContacts.remove(element);
		}
	}

	private void downIndex(double i, ArrayList<RigidBody> bodyList) {
		for(RigidBody b: bodyList) {
			if (b.index > i) {
				b.index --;
			}
		}
		
	}

	/**
     * Finds the body which has a block that intersects the provided point.
     * @param p
     * @return a body containing the given point
     */
    public RigidBody pickBody( Point2d p ) {
        for ( RigidBody body : bodies ) {
        	//must also check if intersects a collection
        	if(body instanceof RigidCollection) {
        		RigidBody b = collectionPick((RigidCollection) body, p);
        		if (b!= null) return b;
        	}
        	else if ( body.intersect( p ) ) {
                return body;
            }
        }
        return null;
    }
    /**
     * recurses through a collection to check if this point intersects any body. returns true if it does
     * @param body 
     * @param p
     */
    private RigidBody collectionPick(RigidCollection body, Point2d p) {
		for (RigidBody b: body.collectionBodies ) {
			
			if (b.intersect(p)) {
				return b;
			}
		}
		return null;
	}

	/**
     * Removes a rigid body from the system
     * @param body
     */
    public void remove( RigidBody body ) {
        bodies.remove(body);
    }
    
    /** 
     * Resets the position of all bodies, and sets all velocities to zero
     */
    public void reset() {
    //	bodies = this.originalBodies;
        int size = bodies.size();
        int counter = 0;
        boolean done = false;
        int i = 0;
    	while(true) {
    		if (bodies.size() == 0) break;
        	RigidBody b = bodies.get(i);
        	if (!b.created) {
        		
        		if (b instanceof RigidCollection) {
        			for (RigidBody subBody: ((RigidCollection) b).collectionBodies) {
        				bodies.add(subBody);
        			}
        			bodies.remove(b);
        		
        			b = bodies.get(i);
        		}
        		b.reset();
        		
        		
        		for (Spring s: b.springs) {
        			s.reset();
        		}
        	}else {
        		counter = i;
        		break;
        	}
        
        i++;
        if (i >= bodies.size()) break;
    	}
    	
    	int iter = size - counter;
    	if ( counter > 0) {
    		while(iter > 0) {
        		this.bodies.remove(bodies.size() - 1);
        		iter--;
            
        	}
    	}
    	
    	
        simulationTime = 0;
        collisionProcessor.reset();
        totalAccumulatedComputeTime = 0;        
    }
    
    private void collectionReset(RigidCollection b) {
    	//resets the state of the rigidBodies inside all collections in the scene. 
    	//loops through them and takes careful care not to mess up the index

		for (RigidBody subBody : b.collectionBodies) {
			subBody.reset();
			bodies.add(subBody);
		}
		bodies.remove(b);
		checkIndex();
	}
    /*
     * if there are bodies with the index i, ups the index of all bodies greater than i to leave room for the ne
     * for the new body about to be "reintroduced" to the scene
     */
	private void checkIndex() {
		int i = 0;
		for(RigidBody b: bodies) {
				b.index = i;
				i++;
		}
	}

	/**
     * Removes all bodies from the system
     */
    public void clear() {
        bodies.clear();
        RigidBody.nextIndex = 0;
        reset();
    }
    
    /**
     * Draws all rigid bodies
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        if ( Block.alpha != (float) (double) transparency.getValue()) {
            Block.alpha = (float) (double) transparency.getValue();
            // gross... need to rebuild display lists for the currently set transparency
            // which is potentially slow and bad news (memory thrashing) for openGL if we do this excessively!
            RigidBody.clearDisplayLists( gl );
            for ( RigidBody b : bodies ) {
                b.myListID = -1;
            }
        }        
        if ( drawBodies.getValue() ) {
            for ( RigidBody b : bodies ) {
          
            	b.display( drawable );
            	
                for (Spring s : b.springs) {
                	s.displayConnection(drawable);
                }
            }
        }
        
        gl.glLineWidth(1);
        if ( drawBoundingVolumes.getValue() ) {
            for ( RigidBody b : bodies ) {
           
            		b.root.boundingDisc.display(drawable);
            	
            }
        }
        if ( drawAllBoundingVolumes.getValue() ) {
            for ( RigidBody b : bodies ) {
            		if (!(b instanceof RigidCollection))
            		b.root.display( drawable );
            		else {
            			displayCollectionBV((RigidCollection) b, drawable);
            		}
            	
            }
        }        
        if ( drawBoundingVolumesUsed.getValue() ) {
            for ( RigidBody b : bodies ) {
            		if (!(b instanceof RigidCollection))
            			b.root.displayVisitBoundary( drawable, collisionProcessor.visitID );
            		else displayVisitBoundaryCollection((RigidCollection) b, drawable);
            }
        }
        if ( drawContactGraph.getValue() ) {
        	if (CollisionProcessor.use_contact_graph.getValue()) {
	            for (RigidBody b : bodies) {
	            	/*for (RigidBody c: b.contact_body_list) {
	            		b.displayConnection(drawable, c);
	            	}*/
	            	for (Contact c:b.contactList) {
	            		c.displayConnection(drawable);
	            	}
	            } 
        	} 
        	else {
        		for ( Contact c : collisionProcessor.contacts ) {
                    c.displayConnection(drawable);
        		}
        	}

        }
        if ( drawSpeedCOM.getValue() ) {
        	for (RigidBody b: bodies) {
        		b.drawSpeedCOM(drawable);
        	}

        }
        
        if ( drawContacts.getValue() ) {
            for ( Contact c : collisionProcessor.contacts ) {
                c.display(drawable);
                c.drawContactForce(drawable);
            }
        }
        if ( drawCOMs.getValue() ) {
            for ( RigidBody b : bodies ) {
                b.displayCOM(drawable);
            }
        }        
        if ( this.use_pendulum.getValue() ) {
            pendulum.displayOrigin(drawable);
        }    
        
        if ( drawIndex.getValue()) {
        	for (RigidBody b : bodies) {
        		b.printIndex(drawable, GLUT.BITMAP_8_BY_13);
        	}
        }
    }

  
    private void displayVisitBoundaryCollection(RigidCollection b, GLAutoDrawable drawable) {
    	for (RigidBody body: b.collectionBodies) {
			if (body instanceof RigidCollection) {
				displayVisitBoundaryCollection((RigidCollection) body, drawable);
			}else {
				body.root.displayVisitBoundary(drawable, collisionProcessor.visitID);
			}
		}
	}

	private void displayCollectionBV(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.collectionBodies) {
			if (body instanceof RigidCollection) {
				displayCollectionBV((RigidCollection) body, drawable);
			}else {
				body.root.display(drawable);
			}
		}
		
	}

	private DoubleParameter transparency = new DoubleParameter("body block transparency", .5, 0, 1 );
    private BooleanParameter drawBodies = new BooleanParameter( "draw bodies", true );
    private BooleanParameter drawBoundingVolumes = new BooleanParameter( "draw root bounding volumes", false );
    private BooleanParameter drawAllBoundingVolumes = new BooleanParameter( "draw ALL bounding volumes", false );
    private BooleanParameter drawBoundingVolumesUsed = new BooleanParameter( "draw bounding volumes used", false );
    private BooleanParameter drawCOMs = new BooleanParameter( "draw center of mass positions", false );
    private BooleanParameter drawContacts = new BooleanParameter( "draw contact locations", false);
    private BooleanParameter drawContactGraph = new BooleanParameter( "draw contact graph", true );
    private BooleanParameter drawSpeedCOM = new BooleanParameter( "draw speed COM", true );
    private BooleanParameter processCollisions = new BooleanParameter( "process collisions", true );
    public static BooleanParameter enableMerging = new BooleanParameter( "enable merging", true);
    public BooleanParameter use_pendulum = new BooleanParameter( "create pendulum", false );
    public BooleanParameter drawIndex = new BooleanParameter( "dawIndex", false );


    
    /**
     * @return control panel for the system
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Rigid Body System Controls" ));
        
        VerticalFlowPanel vfpv = new VerticalFlowPanel();
        vfpv.setBorder( new TitledBorder("viewing controls") );
        vfpv.add( drawBodies.getControls() );
        vfpv.add( transparency.getSliderControls(false));
        vfpv.add( drawBoundingVolumes.getControls() );
        vfpv.add( drawAllBoundingVolumes.getControls() );
        vfpv.add( drawBoundingVolumesUsed.getControls() );
        vfpv.add( drawCOMs.getControls() );
        vfpv.add( drawContacts.getControls() );
        vfpv.add( drawContactGraph.getControls() );
        vfpv.add( drawSpeedCOM.getControls() );
        vfpv.add( drawIndex.getControls() );
       
        
        CollapsiblePanel cp = new CollapsiblePanel(vfpv.getPanel());
        cp.collapse();
        vfp.add( cp );
        
        vfp.add( processCollisions.getControls() );
        vfp.add( enableMerging.getControls() );
        vfp.add( collisionProcessor.getControls() );
        
        vfp.add( use_pendulum.getControls() );
        vfp.add( useGravity.getControls() );
        vfp.add( gravityAmount.getSliderControls(false) );
        vfp.add( gravityAngle.getSliderControls(false) );
        
        vfp.add(spring_k.getSliderControls(false));
        vfp.add(spring_c.getSliderControls(false));
        vfp.add(springLength.getSliderControls());
        
        
        VerticalFlowPanel vfpv_2 = new VerticalFlowPanel();
        vfpv_2.setBorder( new TitledBorder("New body creation controls") );

        
        vfpv_2.add( origin_x.getSliderControls(false) );
	    vfpv_2.add( origin_y.getSliderControls(false) );
	    vfpv_2.add( theta.getSliderControls(false) );
	        
	    vfpv_2.add( velocity_x.getSliderControls(false) );
	    vfpv_2.add( velocity_y.getSliderControls(false) );
	    vfpv_2.add( omega.getSliderControls(false) );
	    vfpv_2.add( index.getSliderControls() );
        CollapsiblePanel cp_2 = new CollapsiblePanel(vfpv_2.getPanel());
        cp_2.collapse();
        
        vfp.add(cp_2);
        return vfp.getPanel();
    }
    
    
    
    
	  /** x value of origin of pendulum */
    public static DoubleParameter origin_x = new DoubleParameter("x position new body", 100, -100, 200 );
    
    /** y value of origin of pendulum */
    public static DoubleParameter origin_y = new DoubleParameter("y position of new body", 40, -100, 100 );
    
    /** theta value of origin of pendulum in degrees*/
    private DoubleParameter theta = new DoubleParameter("angle of creation", 0, -180, 180 );
    
    /** v_x value of origin of pendulum */
    private DoubleParameter velocity_x = new DoubleParameter("velocity x", -20, -100, 100 );
    
    /** v_y value of origin of pendulum */
    private DoubleParameter velocity_y = new DoubleParameter("velocity y", 0, -100, 100 );
    
    /** omega value of origin of pendulum */
    private DoubleParameter omega = new DoubleParameter("angular velocity", 0, -10, 10 );

    /** omega value of origin of pendulum */
    private IntParameter index = new IntParameter("body index", 61, 0, 200);
    

    
	public boolean generateBody = false;

    


	public void generateBody() {
		//get an unpinned random rigidbody
	
		RigidBody body = new RigidBody(bodies.get(0));
		RigidBody backup_body = new RigidBody(bodies.get(0));

		
		Boolean found = false;
		for( RigidBody b: bodies) {
			if (b.pinned) {
				
				continue;
			}
			if (b.index  != index.getValue()) {
				backup_body = new RigidBody(b);
				continue;			
			}
			if (b.index == index.getValue()) {
				found = true;
				body = new RigidBody(b);
			}

			
		}
		//degenerate case
		if (!found) {
			body = backup_body;
		}
		
      
        Point2d position = new Point2d(origin_x.getValue(), origin_y.getValue());
        Vector2d velocity = new Vector2d(velocity_x.getValue(), velocity_y.getValue());
        
        //also needs to scale the blocks of the body:
     //   body.scale(scale.getValue());
        body.x0.set(position);                        
        body.x.set(position);            
        body.theta = this.theta.getValue();
        body.omega = this.omega.getValue();
        body.v.set(velocity);
        body.updateTransformations();
        body.index = bodies.size();
        body.created = true;
        this.add(body);
       // collisionProcessor.bodies.add(body);
	}
    
}

