package comp559.a2;

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

        mouseSpring.apply();
        // apply gravity to all bodies
        if ( useGravity.getValue() ) {
            Vector2d force = new Vector2d();
            for ( RigidBody b : bodies ) {
            	//fully active, regular stepping
            	/*if (CollisionProcessor.use_contact_graph.getValue()) {
            		b.set_activity_contact_graph(CollisionProcessor.sleepingThreshold.getValue());
            	}*/
                double theta = gravityAngle.getValue() / 180.0 * Math.PI;
                force.set( Math.cos( theta ), Math.sin(theta) );
                force.scale( b.massLinear * gravityAmount.getValue() );
                // gravity goes directly into the accumulator!  no torque!
                b.force.add( force );
               
            }
        }
        
        //deal with zero length springs
       for (RigidBody b: bodies){
	        for (Spring s: b.springs) {
	        	s.updateP2();
	        	s.computeVelocities();
	        	
	        	s.apply(spring_k.getValue(), spring_c.getValue());
	        	s.updateP2();
	        
	        }
       }
       
       
        if ( processCollisions.getValue() ) {
            // process collisions, given the current time step
        	
             collisionProcessor.processCollisions( dt );
            
        }

   
        if (use_pendulum.getValue()) {
        	Point2d origin = new Point2d(Pendulum.origin_x.getValue(), Pendulum.origin_y.getValue());
        	pendulumProcessor.processPendulum(dt, origin, Pendulum.pendulum_length.getValue());
        }
        
        
    	 // advance the system by the given time step
        for ( RigidBody b : bodies ) {

            b.advanceTime(dt);
        }
        
        if (enableMerging.getValue()) {
        	// Put your body merging stuff here?  
            mergeBodies();
            
        }
       
        
    	if (this.generateBody) {
    		generateBody();
    		this.generateBody = false;
    	}
        computeTime = (System.nanoTime() - now) / 1e9;
        simulationTime += dt;
        totalAccumulatedComputeTime += computeTime;
    }

    /*
     * Method that shifts all the indeces of each body after i, down by 1. 
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
					RigidCollection col = new RigidCollection(bc.thisBody, bc.otherBody);
					col.addInternalContact(bc);
					bodies.remove(bc.thisBody); bodies.remove(bc.otherBody);
					bodies.add(col);
				}else if (bc.thisBody.parent != null && bc.otherBody.parent != null) {
					// if they are BOTH collections... think about what to do
					//take all the bodies in the least massive one and add them to the collection of the most massive
					if (bc.thisBody.parent.massLinear > bc.otherBody.parent.massLinear) {
						bc.thisBody.parent.addCollection(bc.otherBody.parent);
					}
				}else if (bc.thisBody.parent != null) {
					//thisBody is in a collection... otherBody isnt
					bc.thisBody.parent.addBody(bc.otherBody);
					bc.thisBody.parent.addInternalContact(bc);
					bodies.remove(bc.otherBody);
				}else if (bc.otherBody.parent != null) {
					//otherBody is in a collection... thisBody isnt
					bc.otherBody.parent.addBody(bc.thisBody);
					bc.otherBody.parent.addInternalContact(bc);
					bodies.remove(bc.thisBody);
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
            if ( body.intersect( p ) ) {
                return body;
            }else {
            	//must also check if intersects a collection
            	if(body instanceof RigidCollection) {
            		if (collectionPick((RigidCollection)body, p)){
            			return body;
            		}
            	}
            }
        }
        return null;
    }
    /**
     * recurses through a collection to check if this point intersects any body. returns true if it does
     * @param body 
     * @param p
     */
    private boolean collectionPick(RigidCollection body, Point2d p) {
    
		for (RigidBody b: body.collectionBodies ) {
			
			if (b.intersect(p)) {
				return true;
			}
		}
		return false;
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
    	while(!done) {
    		if (bodies.size() == 0) break;
        	RigidBody b = bodies.get(i);
        	if (!b.created) {
        		if (b instanceof RigidCollection) {
        			collectionReset((RigidCollection) b);
        			continue;
        		}else {
        			b.reset();
        		}
        		for (Spring s: b.springs) {
        			s.reset();
        		}
        	}else {
        		counter = i;
        		break;
        	}
        
        i++;
        if (i == bodies.size()) done = true;
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
	            	for (Contact c:b.contact_list) {
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
    private BooleanParameter drawBoundingVolumesUsed = new BooleanParameter( "draw bounding volumes used", true );
    private BooleanParameter drawCOMs = new BooleanParameter( "draw center of mass positions", false );
    private BooleanParameter drawContacts = new BooleanParameter( "draw contact locations", true);
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
