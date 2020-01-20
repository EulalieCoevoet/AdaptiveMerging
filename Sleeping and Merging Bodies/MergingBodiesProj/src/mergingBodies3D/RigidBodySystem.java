package mergingBodies3D;

import java.awt.Font;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Vector3d;

import mergingBodies3D.Merging.UnmergingCondition;
import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.Vec3Parameter;
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
	    
    public MouseSpringForce mouseSpring;
	public ArrayList<Spring> springs = new ArrayList<Spring>();
	public MouseImpulse mouseImpulse;
	public Impulse impulse = new Impulse();
    
	public CollisionProcessor collision = new CollisionProcessor(bodies);
	public Merging merging = new Merging(bodies, collision);
	public Sleeping sleeping = new Sleeping(bodies, springs);
	public Animation animation = new Animation(); 
	public Display display = new Display(bodies, springs, collision);
	
	public PrintStream stream = null;
	public String sceneName = null;
    
	/**Time in seconds to advance the system*/
	public double computeTime;

	/**Total time in seconds for computation since last reset*/
	public double totalAccumulatedComputeTime;
	
    /** keeps track of the time used to merge on the last call */
    public double mergingTime = 0;
    
    /** keeps track of the time used to unmerge on the last call */
    public double unmergingTime = 0;
    
    public double warmStartTime = 0;
	
	/**Total number of steps performed*/
	public int totalSteps = 0;
	
	public boolean generateBody = false;
	
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
    	
        long start = System.nanoTime();      
		totalSteps++;  

        for ( RigidBody b : bodies ) {
            b.clear();
	        if (b instanceof RigidCollection) {
	        	RigidCollection collection = (RigidCollection)b;
				collection.clearBodies();
	        	if (merging.params.changeColors.getValue())
	        		collection.generateColor();
	        }
		}
        if (merging.params.changeColors.getValue()) 
    		merging.params.changeColors.setValue(false);
        
		applyExternalForces();

		collision.updateContactsMap(); // also called after the LCP solve below... certainly not needed in both places!  :/
		
        collision.collisionDetection(dt);
		collision.updateBodyPairContacts();         
		
		long now = System.nanoTime();   
		collision.warmStart(); 	
        warmStartTime = (System.nanoTime() - now) / 1e9;

		if (sleeping.params.wakeAll.getValue()) sleeping.wakeAll();
		sleeping.wake();
		
		if (merging.params.unmergeAll.getValue()) merging.unmergeAll();

        animation.apply(dt);
		collision.updateInCollections(dt, merging.params);

        now = System.nanoTime();   
		merging.unmerge(UnmergingCondition.CONTACTS, dt);	
        unmergingTime = (System.nanoTime() - now) / 1e9;
        
		if (merging.mergingEvent) {
			for (RigidBody body: bodies)
				body.clear();
			applyExternalForces();
		}
		
		// this is the main LCP solve, and we'll need to redo the warm start
		// if the updateInCollections was run and mucked up the warm started values already
		
		collision.redoWarmStart();

		collision.solveLCP(dt); 
		collision.clearBodyPairContacts();

        RigidCollection.mergeParams = merging.params;
//        for ( RigidBody b : bodies )  
//			b.advanceTime(dt);
        
		for ( RigidBody b : bodies )
			if (!b.pinned && !b.isSleeping)   
				b.advanceVelocities(dt);

		for (BodyPairContact bpc : collision.bodyPairContacts) 
			bpc.accumulateForMerging(merging.params);

		for (RigidBody body: bodies) {
			if (body instanceof RigidCollection)
				for (BodyPairContact bpc : ((RigidCollection)body).bodyPairContacts) 
					bpc.accumulateForUnmerging(merging.params);
		}
		
		for ( RigidBody b : bodies ) {
			if (!b.pinned && !b.isSleeping) { 
				b.advancePositions(dt);
			
				if (b instanceof RigidCollection) {
					((RigidCollection)b).updateBodiesPositionAndTransformations();
					// Accumulation for unmerge is done before this step, so we can clean the bodies velocities
					((RigidCollection)b).applyVelocitiesToBodies();
				}
			}
		}
		
        now = System.nanoTime();   
		merging.merge();
        mergingTime = (System.nanoTime() - now) / 1e9;
		
		sleeping.sleep();

        now = System.nanoTime();   
		merging.unmerge(UnmergingCondition.RELATIVEMOTION, dt); 
        unmergingTime += (System.nanoTime() - now) / 1e9;

		applyViscousDecay();
		
        computeTime = (System.nanoTime() - start) / 1e9;
        simulationTime += dt;
		totalAccumulatedComputeTime += computeTime;
		
		if (generateBody) {
			generateBody();
			generateBody = false;
		}
		
		exportDataToFile();
    }
    
    /**
	 * Apply gravity, mouse spring and impulse
	 */
	protected void applyExternalForces() {
		if ( useGravity.getValue() ) {
			applyGravityForce();
		}  
		if ( useCoriolis.getValue() ) {
			applyCoriolis();
		}
	
		if (mouseSpring != null) {
			mouseSpring.apply( applyMouseSpringAtCOM.getValue() );
			applySpringForces(); 
		}
		
		if (mouseImpulse != null && mouseImpulse.released) {
			impulse.set(mouseImpulse.getPickedBody());
			impulse.set(mouseImpulse.getPickedPoint());
			mouseImpulse.apply();
			impulse.set(mouseImpulse.getForce());
		} else {
			applyImpulse();
		}
	}
	
	/**
	 * Apply stored impulse to the picked body
	 */
	protected void applyImpulse() {
		if (impulse.isHoldingForce()) {
			impulse.pickedBody.applyForceW(impulse.pickedPointW, impulse.force);
			impulse.clear();
		}
	}
	
	/** Temporary working variable */
	private Vector3d tmpDir = new Vector3d();
	private Vector3d tmpForce = new Vector3d();

	/**
	 * Apply gravity to bodies, collections and bodies in collections
	 */
	private void applyGravityForce() {
		double theta = gravityAngle.getValue() / 180.0 * Math.PI;
		double gravAmount = gravityAmount.getValue();
		tmpDir.set( gravAmount * Math.cos( theta ), gravAmount * Math.sin(theta), 0 );
		for ( RigidBody body : bodies ) {		
			tmpForce.scale( - body.massLinear, tmpDir );
			body.force.add( tmpForce ); // gravity goes directly into the accumulator, no torque
			if( body instanceof RigidCollection) {				
				for ( RigidBody b : ((RigidCollection)body).bodies ) {
					tmpForce.scale( - b.massLinear, tmpDir );
					b.force.add( tmpForce );
				}
			}
		}
	}
	
	/**
	 * Apply Coriolis to bodies, collections and bodies in collections
	 */
	private void applyCoriolis() {
		for ( RigidBody body : bodies ) {			
            body.applyCoriolisTorque();			
			if( body instanceof RigidCollection) {				
				for ( RigidBody b : ((RigidCollection)body).bodies ) {
					b.applyCoriolisTorque();
				}
			}
		}
	}
	
	/**
	 * Apply spring forces
	 */
	private void applySpringForces() {
		double h = springMotionStepSize.getValue();
    	double k1 = animationk1.getValue();
    	// tiny bit underdamped and we don't notice!
    	double k2 = 2*Math.sqrt(k1)*animationCDM.getValue(); // critical damping at sqrt(b^2- 4 k)
    	
		for (Spring s: springs) {
			if (s.controllable) {
				s.updateScale(moveScale.getValue()); //is there another, nicer way to do this without making moveScale static or putting it as a member variable in LCPApp? 
				s.moveSpring(h,k1, k2 );
			}
			s.apply(springStiffnessMod.getValue(), springDampingMod.getValue());
			
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

			if(body.created) {
				bodies.remove(body);
				continue;
			}
			
			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection) body;
				for (RigidBody subBody: collection.bodies) {
					subBody.parent = null;
					bodies.add(subBody);
				}
				bodies.remove(body);
			} else {
				body.bodyPairContacts.clear();
				body.reset();
				i++;
			}
			
			if (i >= bodies.size()) break;
		}
		
        for ( Spring s : springs ) 
        	s.reset();
		
        simulationTime = 0;
        collision.reset();
        animation.reset();
        totalAccumulatedComputeTime = 0;     
        totalSteps = 0;
    }
    
    protected void applyViscousDecay()
    {
		double alpha1 = globalViscousLinearDecay.getValue();
		double alpha2 = globalViscousAngularDecay.getValue();
		for ( RigidBody b : bodies ) {
			b.v.scale( alpha1 );
			b.omega.scale( alpha2 );
		}
    }
    
    /**
     * Removes all bodies AND springs from the system
     */
    public void clear() {
        bodies.clear();
        springs.clear();
        RigidBody.nextIndex = 0;
        reset();
    }
    
    /**
     * Removes all bodies from the system. Not springs. Used for initializing factory.
     */
    public void clearBodies() {
    	bodies.clear();
    	RigidBody.nextIndex = 0;
    	reset();
    }
    
    /**
	 * Generate a body (called when G is pressed)
	 */
	private void generateBody() {

		RigidBody genbody = null;
		
		//get an unpinned random RigidBody
		for (RigidBody body: bodies) {
			
			if (bodies.indexOf(body)==0)
				continue;
							
			if (!(body instanceof RigidCollection) && !body.pinned) {
				genbody = new RigidBody(body);
				break;
			}
			
			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection)body;
				for (RigidBody b: collection.bodies) {
					if (!b.pinned) {
						genbody = new RigidBody(b);
						break;
					}
				}
			}
		}

		//also needs to scale the blocks of the body:
		//   body.scale(scale.getValue());
		if (genbody != null) {
			genbody.x0.set(generatedBodyPos.x, generatedBodyPos.y, generatedBodyPos.z);                        
			genbody.x.set(generatedBodyPos.x, generatedBodyPos.y, generatedBodyPos.z);            
			genbody.v.set(generatedBodyVel.x, generatedBodyVel.y, generatedBodyVel.z);
			genbody.theta.setIdentity();
			genbody.omega.set(0.,0.,0.);
			genbody.updateRotationalInertaionFromTransformation();
			genbody.created = true;
			bodies.add(genbody);
		} else {
			System.err.println("[generateBody] Could not find a body to generate");
		}
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
					
					stream.print("#bodies"); stream.print(", ");
					stream.print("#contacts"); stream.print(", ");
					stream.print("detection"); stream.print(", ");
					stream.print("warmstart"); stream.print(", ");
					stream.print("LCPSolve"); stream.print(", ");
					stream.print("updateCollections"); stream.print(", ");
					stream.print("contactOrdering"); stream.print(", ");
					stream.print("singleItPGS"); stream.print(", ");
					stream.print("merging"); stream.print(", ");
					stream.print("mergingCheckContacts"); stream.print(", ");
					stream.print("mergingCheckMotion"); stream.print(", ");
					stream.print("mergingCheckViolation"); stream.print(", ");
					stream.print("mergingCheckCycle"); stream.print(", ");
					stream.print("mergingBuild"); stream.print(", ");
					stream.print("unmerging"); stream.print(", ");
					stream.print("unmergingCheckContacts"); stream.print(", ");
					stream.print("unmergingCheckMotion"); stream.print(", ");
					stream.print("unmergingCheckCycle"); stream.print(", ");
					stream.print("unmergingBuild"); stream.print(", ");
					stream.print("computeTime"); stream.print("\n ");
					
				} catch (FileNotFoundException e) {	}
			} else {
				stream.print(bodies.size()); stream.print(", ");
				stream.print(collision.contacts.size()); stream.print(", ");
				stream.print(collision.collisionDetectTime); stream.print(", ");
				stream.print(warmStartTime); stream.print(", ");
				stream.print(collision.collisionSolveTime); stream.print(", ");
				stream.print(collision.collectionUpdateTime); stream.print(", ");
				stream.print(collision.contactOrderingTime); stream.print(", ");
				stream.print(collision.singleItPGSTime); stream.print(", ");
				stream.print(mergingTime); stream.print(", ");
				stream.print(merging.params.mergingCheckContactTime); stream.print(", ");
				stream.print(merging.params.mergingCheckMotionTime); stream.print(", ");
				stream.print(merging.params.mergingCheckViolationTime); stream.print(", ");
				stream.print(merging.params.mergingCheckCycleTime); stream.print(", ");
				stream.print(merging.params.mergingBuildTime); stream.print(", ");
				stream.print(unmergingTime); stream.print(", ");
				stream.print(merging.params.unmergingCheckContactTime); stream.print(", ");
				stream.print(merging.params.unmergingCheckMotionTime); stream.print(", ");
				stream.print(merging.params.unmergingCheckCycleTime); stream.print(", ");
				stream.print(merging.params.unmergingBuildTime); stream.print(", ");
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
    BooleanParameter useCoriolis = new BooleanParameter( "enable Coriolis (for stability: use smaller time steps, global angular viscous, or both)", false );
    
    public BooleanParameter saveCSV = new BooleanParameter( "save CSV", false);
	public DoubleParameter globalViscousLinearDecay = new DoubleParameter("global viscous linear decay", 1, 0.1, 1 );
	public DoubleParameter globalViscousAngularDecay = new DoubleParameter("global viscous angular decay", 1, 0.1, 1 );

	public DoubleParameter springStiffnessMod = new DoubleParameter("spring stiffness multiplier", 1, 0, 10 );
	public DoubleParameter springDampingMod= new DoubleParameter("spring damping multiplier", 1, 0, 10 );
	
    public BooleanParameter applyMouseSpringAtCOM = new BooleanParameter( "apply mouse spring at COM", false );
    
	public static Vec3Parameter generatedBodyPos = new Vec3Parameter("position new body", 50, 50, 0, 200 );
	public static Vec3Parameter generatedBodyVel = new Vec3Parameter("velocity of new body", 0, 0, 0, 100 );

    
	/** Parameters to help solve ODE to get controllable springs to desired position smoothly
	 */
	private DoubleParameter springMotionStepSize = new DoubleParameter( "anim step size", 0.005, 1e-4, 1 );
	private DoubleParameter animationk1= new DoubleParameter( "motion stiffness", 100, 10, 1e6 );
	private DoubleParameter animationCDM = new DoubleParameter( "critical damping multiplyer", 0.9, 0, 2);
	public  DoubleParameter moveScale = new DoubleParameter("step Movement Scale", 0.2, 0.01, 10); 
	
	
    /**
     * @return control panel for the system
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Rigid Body System Controls" ));
        ((TitledBorder) vfp.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));
        
        vfp.add( useGravity.getControls() );
        vfp.add( gravityAmount.getSliderControls(false) );
        vfp.add( gravityAngle.getSliderControls(false) );
        vfp.add( useCoriolis.getControls() );
        
		vfp.add(springStiffnessMod.getSliderControls(false));
		vfp.add(springDampingMod.getSliderControls(false));
		vfp.add( globalViscousLinearDecay.getSliderControls(false) );
		vfp.add( globalViscousAngularDecay.getSliderControls(false) );

		vfp.add( generatedBodyPos );
		vfp.add( generatedBodyVel );
		
		vfp.add( applyMouseSpringAtCOM.getControls() );

		return vfp.getPanel();
    }


    /**
     * @return control panel for controllable springs
     */
    public JPanel getMoveableSpringControls() {
    	VerticalFlowPanel vfp = new VerticalFlowPanel();
    	vfp.setBorder( new TitledBorder("Moveable Springs Controls" ));
    	((TitledBorder) vfp.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));
    	vfp.add( springMotionStepSize.getSliderControls(true) );
    	vfp.add( animationk1.getSliderControls(true) );
    	vfp.add( animationCDM.getSliderControls(false) );
    	vfp.add( moveScale.getSliderControls(true) );
    	CollapsiblePanel cp = new CollapsiblePanel(vfp.getPanel());
    	return cp;
    }
		
    
}
