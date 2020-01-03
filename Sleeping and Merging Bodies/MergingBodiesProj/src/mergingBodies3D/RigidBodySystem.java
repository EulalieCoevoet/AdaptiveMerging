package mergingBodies3D;

import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Color3f;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
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
	
	public CollisionProcessor collision = new CollisionProcessor(bodies);
    
    public MouseSpringForce mouseSpring;
    
    BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
    
    DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
    
    DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );
    
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
            b.omega.x += rand.nextDouble()*2-1;
            b.omega.y += rand.nextDouble()*2-1;
            b.omega.z += rand.nextDouble()*2-1;
            b.v.x += rand.nextDouble()*2-1;
            b.v.y += rand.nextDouble()*2-1;                
            b.v.z += rand.nextDouble()*2-1;                
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

        for ( RigidBody b : bodies ) 
            b.clear();
        
		applyExternalForces();

		collision.updateContactsMap();
        collision.collisionDetection(dt);
		collision.warmStart(); 	
        
		collision.solveLCP(dt); 
        
        // advance the system by the given time step
        for ( RigidBody b : bodies ) 
            b.advanceTime(dt);
		
        computeTime = (System.nanoTime() - now) / 1e9;
        simulationTime += dt;
        totalAccumulatedComputeTime += computeTime;
    }
    
    /**
	 * Apply gravity, mouse spring and impulse
	 */
	protected void applyExternalForces() {
		if (useGravity.getValue()) {
			applyGravityForce();
		}  
	
		if (mouseSpring != null) {
			mouseSpring.apply();
			applySpringForces(); 
		}
		
//		if (mouseImpulse != null && mouseImpulse.released) {
//			impulse.set(mouseImpulse.getPickedBody());
//			impulse.set(mouseImpulse.getPickedPoint());
//			mouseImpulse.apply();
//			impulse.set(mouseImpulse.getForce());
//		} else {
//			applyImpulse();
//		}
	}
	
	/**
	 * Apply stored impulse to the picked body
	 */
//	protected void applyImpulse() {
//		if (impulse.isHoldingForce()) {
//			impulse.pickedBody.applyForceW(impulse.pickedPoint, impulse.force);
//			impulse.clear();
//		}
//	}
	
	/** Temporary working variable */
	private Vector3d tmpForce = new Vector3d();

	/**
	 * Apply gravity to bodies, collections and bodies in collections
	 */
	private void applyGravityForce() {
		for ( RigidBody body : bodies ) {
			
//			if (body.isSleeping)
//				continue;
			
			//fully active, regular stepping
			double theta = gravityAngle.getValue() / 180.0 * Math.PI;
			tmpForce.set( Math.cos( theta ), Math.sin(theta), 0 );
			tmpForce.scale( - body.massLinear * gravityAmount.getValue() );
			body.force.add( tmpForce ); // gravity goes directly into the accumulator, no torque
            body.applyCoriollisTorque(); // TODO: sadly, this appears to be buggy :(
			
//			if( body instanceof RigidCollection) 
//				applyGravityCollection((RigidCollection) body, theta);
		}
	}
	
	/**
	 * Apply gravity to bodies in given collection
	 * @param collection
	 * @param theta
	 */
//	private void applyGravityCollection(RigidCollection collection, double theta) {
//		Vector2d force = new Vector2d();
//		for (RigidBody body : collection.bodies) {
//			force.set( Math.cos( theta ), Math.sin(theta) );
//			force.scale( body.massLinear * gravityAmount.getValue() );
//			body.force.add( force );
//		}
//	}

	/**
	 * Apply spring forces
	 */
	private void applySpringForces() {
		for (RigidBody body: bodies){
//			if (body.isSleeping)
//				continue;
			for (Spring s: body.springs) {
				s.apply(springStiffness.getValue(), springDamping.getValue());
			}
		}
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
        for ( RigidBody b : bodies ) {
            b.reset();
        }
        simulationTime = 0;
        collision.reset();
        totalAccumulatedComputeTime = 0;        
    }
    
    /**
     * Removes all bodies from the system
     */
    public void clear() {
        bodies.clear();
        RigidBody.nextIndex = 0;
        reset();
    }
        
    /** Might want to allow for different coloured blocks?? but for now, in 3D this is easiest */
    private float[] green = new float[] { 0, 1, 0, 0.25f };
    private float[] colourPinned = new float[] { 0.75f,0.75f,1, 1 };		        			
	private float[] colour = new float[] { 0.9f,0.9f,0.9f, 1 };        			
    private float[] red = new float[] { 1, 0, 0, 0.5f };
    private float[] blue = new float[] { 0, 0, 1, 0.25f };
    
    public void displayNonShadowable( GLAutoDrawable drawable, double dt ) {
    	GL2 gl = drawable.getGL().getGL2();
        gl.glDisable(GL2.GL_DEPTH_TEST);

    	// Should move this stuff below to a display non-shadowable function
        gl.glMaterialfv(GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, green, 0);
        if ( drawContactGraph.getValue() ) {
            for ( Contact c : collision.contacts ) {
                c.displayConnection(drawable);
            }
        }
        
        if ( drawContacts.getValue() ) {
            for ( Contact c : collision.contacts ) {
                c.display(drawable);
    			if (drawContactForces.getValue()) {
    				c.displayContactForce(drawable, dt );  // should let it draw its own colour
    			}
            }
        }
//        if ( drawCOMs.getValue() ) {
//            for ( RigidBody b : bodies ) {
//                b.displayCOM(drawable);
//            }
//        }
        gl.glEnable(GL2.GL_DEPTH_TEST);
    }

    /**
     * Draws all rigid bodies
     * @param drawable
     */
    public void display( GLAutoDrawable drawable, boolean picking ) {
        GL2 gl = drawable.getGL().getGL2();
        
        if ( drawCOMs.getValue() && ! picking ) {
        	for ( RigidBody b : bodies ) {
        		b.displayFrame(drawable);
        	}
        }
        
        // TODO: perhaps do clipping planes here instead to only clip bodies?
        // TODO: perhaps also have cull face options?  think we want to cull to better see contacts after clipping
        if ( drawBodies.getValue() ) {
        	int i = 0;
        	for ( RigidBody b : bodies ) {
        		if ( picking ) {
        			LCPApp3D.setColorWithID( gl, i++ );
        		} else {
        			// let's control the colour of geometry here as it will let us 
        			// decide when we want to override this colour (e.g., if we have a 
        			// rigid body collection)
        			float[] c = colour;
        			if ( b.pinned ) {
        				c = colourPinned;
        			} else if ( b.col != null ) {
        				c = b.col;
        			}
    				c[3] = transparency.getFloatValue();         			
        			gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, c, 0 );
        		}
                b.display( drawable );
        	}
        }
       
        
        // TODO: end clipping here to continue to see other debug visualizations un-clipped..
        // again, perhaps an option to end clipping here or at the end of all of this!
        
        if ( ! picking ) {
	        gl.glMaterialfv(GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, red, 0);
	        gl.glNormal3f(0,0,1);
	    	for ( RigidBody b : bodies ) {
	    		for (Spring s : b.springs) {
					s.displaySpring(drawable);
				}
	    	}        
	        gl.glLineWidth(1);
	        gl.glMaterialfv(GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, blue, 0);
	        if ( drawBoundingVolumes.getValue() ) {
	            for ( RigidBody b : bodies ) {
	            	if ( b.root == null ) continue; // rigid body planes don't have a BVH
	                b.root.boundingSphere.display(drawable);
	            }
	        }
	        if ( drawAllBoundingVolumes.getValue() ) {
	            for ( RigidBody b : bodies ) {
	            	if ( b.root == null ) continue; // rigid body planes don't have a BVH
	                b.root.display( drawable );
	            }
	        }        
	        if ( drawBoundingVolumesUsed.getValue() ) {
	            for ( RigidBody b : bodies ) {
	            	if ( b.root == null ) continue; // rigid body planes don't have a BVH
	                b.root.displayVisitBoundary( drawable, collision.visitID );
	            }
	        }	        
        }
    }

    private DoubleParameter transparency = new DoubleParameter("body block transparency", 1, 0, 1 );
    private BooleanParameter drawBodies = new BooleanParameter( "draw bodies", true );
    private BooleanParameter drawBoundingVolumes = new BooleanParameter( "draw root bounding volumes", false );
    private BooleanParameter drawAllBoundingVolumes = new BooleanParameter( "draw ALL bounding volumes", false );
    private BooleanParameter drawBoundingVolumesUsed = new BooleanParameter( "draw bounding volumes used", false );
    private BooleanParameter drawCOMs = new BooleanParameter( "draw center of mass positions", false );
    private BooleanParameter drawContacts = new BooleanParameter( "draw contact locations", false );
    private BooleanParameter drawContactGraph = new BooleanParameter( "draw contact graph", false );

	public DoubleParameter springStiffness = new DoubleParameter("spring stiffness", 100, 1, 1e4 );
	public DoubleParameter springDamping= new DoubleParameter("spring damping", 1, 0, 1000 );
	
	private BooleanParameter drawContactForces = new BooleanParameter("draw contact forces", false );

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
        vfpv.add( drawContactGraph.getControls() );
        vfpv.add( drawContacts.getControls() );
        vfpv.add( drawContactForces.getControls() );
		vfpv.add( Contact.forceVizScale.getSliderControls(true) ); // Gross?
        CollapsiblePanel cp = new CollapsiblePanel(vfpv.getPanel());
        cp.collapse();
        vfp.add( cp );
        
        vfp.add( collision.getControls() );
        
        vfp.add( useGravity.getControls() );
        vfp.add( gravityAmount.getSliderControls(false) );
        vfp.add( gravityAngle.getSliderControls(false) );
        
		vfp.add(springStiffness.getSliderControls(false));
		vfp.add(springDamping.getSliderControls(false));

		return vfp.getPanel();
    }
    
}
