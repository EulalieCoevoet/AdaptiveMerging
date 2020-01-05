package mergingBodies3D;

import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Vector3d;

import mergingBodies3D.Merging.MergeConditions;
import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.swing.VerticalFlowPanel;

/**
 * Maintains a list of RigidBody objects, and provides methods for collision processing and numerical integration
 * @author kry
 */
public class RigidBodySystem {

	public String name ="";
	
    public double simulationTime = 0;
    
	public ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
	    
    public MouseSpringForce mouseSpring;
    
    BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
    
    DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
    
    DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );
    
	public CollisionProcessor collision = new CollisionProcessor(bodies);
	public Merging merging = new Merging(bodies, collision);
	public Sleeping sleeping = new Sleeping(bodies);
    
	public Display display = new Display(bodies, collision);

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

        for ( RigidBody b : bodies ) {
            b.clear();
	        if (b instanceof RigidCollection) {
				((RigidCollection)b).clearBodies();
	        }
		}
        
		applyExternalForces();

		collision.updateContactsMap();
        collision.collisionDetection(dt);
		collision.warmStart(); 	
        
		collision.updateBodyPairContacts(); 

		
		sleeping.wakeAll();
		
		sleeping.wake();
		
		merging.unmergeAll(); // this only happens upon request (i.e., a UI checkbox)
		
		collision.updateInCollections(dt, merging.params);

		collision.solveLCP(dt); 
        
        // advance the system by the given time step
		
        RigidCollection.mergeParams = merging.params;
        		
		for ( RigidBody b : bodies )
            b.advanceTime( dt );
		
		for (BodyPairContact bpc : collision.bodyPairContacts) 
			bpc.accumulate(merging.params);

		merging.merge();
		
		sleeping.sleep();
		
		merging.unmerge(MergeConditions.RELATIVEMOTION, dt); 

		
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
			
			if( body instanceof RigidCollection) {				
				for ( RigidBody b : ((RigidCollection)body).bodies ) {
					tmpForce.set( Math.cos( theta ), Math.sin(theta), 0 );
					tmpForce.scale( - body.massLinear * gravityAmount.getValue() );
					b.force.add( tmpForce );
				}
			}
		}
	}
	
	/**
	 * Apply spring forces
	 */
	private void applySpringForces() {
		for (RigidBody body: bodies){
//			if (body.isSleeping)
//				continue;
			for (Spring s: body.springs) {
				s.apply(springStiffnessMod.getValue(), springDampingMod.getValue());
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
    	//int size = bodies.size();
		//int counter = 0;
		int i = 0;
		while (true) {
			if (bodies.size() == 0) break;
			RigidBody b = bodies.get(i);
			//if (!b.created) {

				if (b instanceof RigidCollection) {
					for (RigidBody subBody: ((RigidCollection) b).bodies) {
						subBody.parent = null;
						bodies.add(subBody);
					}
					bodies.remove(b);

					b = bodies.get(i);
				}
				b.reset();		
//			} else {
//				counter = i;
//				break;
//			}

			i++;
			if (i >= bodies.size()) break;
		}
		
		collision.reset();
		
//		int iter = size - counter;
//		if ( counter > 0) {
//			while(iter > 0) {
//				this.bodies.remove(bodies.size() - 1);
//				iter--;
//			}
//		}
    	
//        for ( RigidBody b : bodies ) {
//            b.reset();
//        }
        
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
    
	public DoubleParameter springStiffnessMod = new DoubleParameter("spring stiffness multiplier", 1, 0, 10 );
	public DoubleParameter springDampingMod= new DoubleParameter("spring damping multiplier", 1, 0, 10 );

    /**
     * @return control panel for the system
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Rigid Body System Controls" ));
        
        vfp.add( collision.getControls() );
        
        vfp.add( useGravity.getControls() );
        vfp.add( gravityAmount.getSliderControls(false) );
        vfp.add( gravityAngle.getSliderControls(false) );
        
		vfp.add(springStiffnessMod.getSliderControls(false));
		vfp.add(springDampingMod.getSliderControls(false));

		return vfp.getPanel();
    }
    
}
