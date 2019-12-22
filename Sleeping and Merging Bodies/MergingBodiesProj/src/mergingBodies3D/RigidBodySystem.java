package mergingBodies3D;

import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Vector3d;

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
        
        if ( collision.doLCP.getValue() ) 
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

	/**
	 * Apply gravity to bodies, collections and bodies in collections
	 */
	private void applyGravityForce() {
		Vector3d force = new Vector3d();
		for ( RigidBody body : bodies ) {
			
//			if (body.isSleeping)
//				continue;
			
			//fully active, regular stepping
			double theta = gravityAngle.getValue() / 180.0 * Math.PI;
			force.set( Math.cos( theta ), Math.sin(theta), 0 );
			force.scale( - body.massLinear * gravityAmount.getValue() );
			body.force.add( force ); // gravity goes directly into the accumulator, no torque
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
//		for (RigidBody body: bodies){
////			if (body.isSleeping)
////				continue;
//			for (Spring s: body.springs) {
//				s.apply(springStiffness.getValue(), springDamping.getValue());
//			}
//		}
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
            // TODO: This display list stuff should probably be changed in 3D :/
            RigidBodyGeom.clearDisplayLists( gl );
            for ( RigidBody b : bodies ) {                
            	b.geom.myListID = -1;
            }
        }        
        if ( drawBodies.getValue() ) {
        	int i = 0;
        	for ( RigidBody b : bodies ) {
            	gl.glLoadName( i++ ); // for picking
                b.display( drawable );
            }
        }
        gl.glLineWidth(1);
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
        if ( drawContactGraph.getValue() ) {
            for ( Contact c : collision.contacts ) {
                c.displayConnection(drawable);
            }
        }
        
        if ( drawContacts.getValue() ) {
            for ( Contact c : collision.contacts ) {
                c.display(drawable);
            }
        }
        if ( drawCOMs.getValue() ) {
            for ( RigidBody b : bodies ) {
                b.displayCOM(drawable);
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
        
        CollapsiblePanel cp = new CollapsiblePanel(vfpv.getPanel());
        cp.collapse();
        vfp.add( cp );
        
        vfp.add( collision.getControls() );
        
        vfp.add( useGravity.getControls() );
        vfp.add( gravityAmount.getSliderControls(false) );
        vfp.add( gravityAngle.getSliderControls(false) );
        return vfp.getPanel();
    }
    
}
