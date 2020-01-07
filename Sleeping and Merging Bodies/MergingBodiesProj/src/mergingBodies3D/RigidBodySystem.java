package mergingBodies3D;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
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
    
	public CollisionProcessor collision = new CollisionProcessor(bodies);
	public Merging merging = new Merging(bodies, collision);
	public Sleeping sleeping = new Sleeping(bodies);
	public Display display = new Display(bodies, collision);
	
	public PrintStream stream = null;
	public String sceneName = null;
    
	/**Time in seconds to advance the system*/
	public double computeTime;

	/**Total time in seconds for computation since last reset*/
	public double totalAccumulatedComputeTime;
	
	/**Total number of steps performed*/
	public int totalSteps = 0;
	
    /**
     * Creates a new rigid body system
     */
    public RigidBodySystem() {
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
     * Advances the state of all rigid bodies
     * @param dt time step
     */
    public void advanceTime( double dt ) {
    	
        long now = System.nanoTime();      
		totalSteps++;  

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

		if (sleeping.params.wakeAll.getValue()) sleeping.wakeAll();
		sleeping.wake();
		
		if (merging.params.unmergeAll.getValue()) merging.unmergeAll();
		
		collision.updateInCollections(dt, merging.params);
		
		merging.unmerge(MergeConditions.CONTACTS, dt);	
		if (merging.mergingEvent) {
			for (RigidBody body: bodies)
				body.clear();
			applyExternalForces();
		}
		
		collision.solveLCP(dt); 
		collision.clearBodyPairContacts();
        
        RigidCollection.mergeParams = merging.params;
		for ( RigidBody b : bodies )
            b.advanceTime( dt );
		
		for (BodyPairContact bpc : collision.bodyPairContacts) 
			bpc.accumulate(merging.params);

		merging.merge();
		
		sleeping.sleep();
		
		merging.unmerge(MergeConditions.RELATIVEMOTION, dt); 

		applyViscousDecay();
		
        computeTime = (System.nanoTime() - now) / 1e9;
        simulationTime += dt;
		totalAccumulatedComputeTime += computeTime;
		
		exportDataToFile();
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
			
			if (body.isSleeping)
				continue;
			
			//fully active, regular stepping
			double theta = gravityAngle.getValue() / 180.0 * Math.PI;
			tmpForce.set( Math.cos( theta ), Math.sin(theta), 0 );
			tmpForce.scale( - body.massLinear * gravityAmount.getValue() );
			body.force.add( tmpForce ); // gravity goes directly into the accumulator, no torque
            body.applyCoriollisTorque(); // TODO: CORIOLLIS: sadly, this appears to be buggy :(
			
			if( body instanceof RigidCollection) {				
				for ( RigidBody b : ((RigidCollection)body).bodies ) {
					tmpForce.set( Math.cos( theta ), Math.sin(theta), 0 );
					tmpForce.scale( - b.massLinear * gravityAmount.getValue() );
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
			if (body.isSleeping)
				continue;
			for (Spring s: body.springs) {
				s.apply(springStiffnessMod.getValue(), springDampingMod.getValue());
			}
			
			if (body instanceof RigidCollection) {
				for (RigidBody b: ((RigidCollection)body).bodies){
					for (Spring s: b.springs) {
						s.apply(springStiffnessMod.getValue(), springDampingMod.getValue());
					}
				}
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
		int i = 0;
		while (true) {
			if (bodies.size() == 0) break;
			RigidBody body = bodies.get(i);

			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection) body;
				for (RigidBody subBody: collection.bodies) {
					subBody.parent = null;
					subBody.bodyPairContacts.clear();
					bodies.add(subBody);
				}
				bodies.remove(body);
				body = bodies.get(i);
			}  
			
			if (!(body instanceof RigidCollection)) {
				body.bodyPairContacts.clear();
				body.reset();
				i++;
			}
			
			if (i >= bodies.size()) break;
		}
		
        simulationTime = 0;
        collision.reset();
        totalAccumulatedComputeTime = 0;     
    }
    
    protected void applyViscousDecay()
    {
		double alpha = globalViscousDecay.getValue();
		for ( RigidBody b : bodies ) {
			b.v.scale( alpha );
			b.omega.scale(alpha);
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
	 * Export data to csv file
	 */
	protected void exportDataToFile() {
			
		if (saveCSV.getValue()) {
			if (stream == null) {
				try {
					String filename;
					if (merging.params.enableMerging.getValue()) 
						filename = new String(sceneName + "_merged.csv");
					else
						filename = new String(sceneName + ".csv");
					File file = new File(filename);
					stream = new PrintStream(file);
				} catch (FileNotFoundException e) {	}
			} else {
				stream.print(bodies.size()); stream.print(", ");
				stream.print(collision.contacts.size()); stream.print(", ");
				stream.print(collision.collisionDetectTime); stream.print(", ");
				stream.print(collision.collisionSolveTime); stream.print(", ");
				stream.print(computeTime); stream.print("\n ");
			}
		} else if (stream != null) {
			stream.close();
			stream = null;
		}
	}
	
    BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
    DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
    DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );
   
    public BooleanParameter saveCSV = new BooleanParameter( "save CSV", false);
	public DoubleParameter globalViscousDecay = new DoubleParameter("global viscous decay", 1, 0.1, 1 );

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
		vfp.add( globalViscousDecay.getSliderControls(false) );

		return vfp.getPanel();
    }
    
}
