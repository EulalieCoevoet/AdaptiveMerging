package mergingBodies;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mergingBodies.Merging.MergeConditions;
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
	public ArrayList<RigidBody> initialBodies = new ArrayList<RigidBody>();
	
	public ArrayList<Spring> controllableSprings = new ArrayList<Spring>();

	public CollisionProcessor collision = new CollisionProcessor(bodies);
	public Merging merging = new Merging(bodies, collision);
	public Sleeping sleeping = new Sleeping(bodies);
	public Display display = new Display(bodies, collision);

	public MouseSpringForce mouseSpring;
	public MouseImpulse mouseImpulse;
	public Impulse impulse = new Impulse();
	
	public PrintStream stream = null;
	
	public String sceneName = null;
	
	public boolean generateBody = false;
	
	/**
	 * Creates a new rigid body system
	 */
	public RigidBodySystem() {
		/* do nothing */
	}

	/**
	 * Applies a small random acceleration to all bodies
	 */
	public void jiggle() {
		final Random rand = new Random();
		for ( RigidBody b : bodies ) {
			if ( b.pinned || b.temporarilyPinned ) continue;
			b.omega += rand.nextDouble()*2-1;
			b.v.x += rand.nextDouble()*2-1;
			b.v.y += rand.nextDouble()*2-1;    
		}
	}

	/**Time in seconds to advance the system*/
	public double computeTime;

	/**Total time in seconds for computation since last reset*/
	public double totalAccumulatedComputeTime;

	/**Total number of steps performed*/
	public int totalSteps = 0;
	
	/**
	 * Advances the state of all rigid bodies
	 * @param dt time step
	 */
	public void advanceTime( double dt ) {

		long now = System.nanoTime();  
		totalSteps++;
		
		for (RigidBody body: bodies) {
			body.clear();
			if (body instanceof RigidCollection)
				((RigidCollection)body).clearBodies();
		}
		
		applyExternalForces();
		
		collision.updateContactsMap();
		collision.collisionDetection(dt); 
		collision.warmStart(); 		
		collision.updateBodyPairContacts(); 
		
		sleeping.wake();
		
		merging.unmergeAll();
		
		collision.updateInCollections(dt, merging.params);
			
		merging.unmerge(MergeConditions.CONTACTS, dt);	
		if (merging.mergingEvent) {
			for (RigidBody body: bodies)
				body.clear();
			applyExternalForces();
		}
		
		collision.solveLCP(dt); 
		collision.clearBodyPairContacts();
		
		for (RigidBody b : bodies) 
			b.advanceTime(dt, merging.params); 
		
		for (BodyPairContact bpc : collision.bodyPairContacts) 
			bpc.accumulate(merging.params);
		
		merging.merge();
		
		sleeping.sleep();
			
		merging.unmerge(MergeConditions.RELATIVEMOTION, dt); 

		if (generateBody) {
			generateBody();
			generateBody = false;
		}
		
		double alpha = globalViscousDecay.getValue();
		for ( RigidBody b : bodies ) {
			b.v.scale( alpha );
			b.omega *= alpha;
		}
		
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
		
		if (mouseImpulse != null && mouseImpulse.released) {
			impulse.set(mouseImpulse.getPickedBody());
			impulse.set(mouseImpulse.getPickedPoint());
			mouseImpulse.apply();
			impulse.set(mouseImpulse.getForce());
		} else {
			applyImpulse();
		}
	}
	
	protected void applyImpulse() {
		if (impulse.isHoldingForce()) {
			impulse.pickedBody.applyForceW(impulse.pickedPoint, impulse.force);
			impulse.clear();
		}
	}

	private void applyGravityForce() {
		Vector2d force = new Vector2d();
		for ( RigidBody body : bodies ) {
			
			if (body.isSleeping)
				continue;
			
			//fully active, regular stepping
			double theta = gravityAngle.getValue() / 180.0 * Math.PI;
			force.set( Math.cos( theta ), Math.sin(theta) );
			force.scale( body.massLinear * gravityAmount.getValue() );
			body.force.add( force ); // gravity goes directly into the accumulator, no torque
			
			if( body instanceof RigidCollection) 
				applyGravityCollection((RigidCollection) body, theta);
		}
	}
	
	private void applyGravityCollection(RigidCollection collection, double theta) {
		Vector2d force = new Vector2d();
		for (RigidBody body : collection.bodies) {
			force.set( Math.cos( theta ), Math.sin(theta) );
			force.scale( body.massLinear * gravityAmount.getValue() );
			body.force.add( force );
		}
	}

	private void applySpringForces() {
		for (RigidBody body: bodies){
			if (body.isSleeping)
				continue;
			for (Spring s: body.springs) {
				s.apply(springStiffness.getValue(), springDamping.getValue());
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
			if(body instanceof PlaneRigidBody)
				continue;
			
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
		for (RigidBody b: body.bodies ) {

			if(b instanceof PlaneRigidBody)
				continue;
			
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
		int size = bodies.size();
		int counter = 0;
		int i = 0;
		while(true) {
			if (bodies.size() == 0) break;
			RigidBody b = bodies.get(i);
			if (!b.created) {

				if (b instanceof RigidCollection) {
					for (RigidBody subBody: ((RigidCollection) b).bodies) {
						bodies.add(subBody);
					}
					bodies.remove(b);

					b = bodies.get(i);
				}
				b.reset();		
			} else {
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
		collision.reset();
		totalAccumulatedComputeTime = 0; 
		totalSteps = 0;
	}

	/**
	 * Removes all bodies from the system
	 */
	public void clear() {
		bodies.clear();
		reset();
	}
	
	public void generateBody() {

		RigidBody genbody = null;
		int index = this.index.getValue();
		if( bodies.size() > index && !(bodies.get(index) instanceof RigidCollection) && !bodies.get(index).pinned) 
			genbody = new RigidBody(bodies.get(index));
		
		//get an unpinned random RigidBody
		if (genbody == null) {
			for (RigidBody body: bodies) {
				if (!(body instanceof RigidCollection) && !body.pinned) {
					genbody = new RigidBody(body);
					break;
				}
			}
		}
		
		if (genbody == null) {
			if(bodies.get(0) instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection)bodies.get(0);
				for (RigidBody body: collection.bodies) {
					if (!body.pinned) {
						genbody = new RigidBody(body);
						break;
					}
				}
			}
		}
		
		Point2d position = new Point2d(origin_x.getValue(), origin_y.getValue());
		Vector2d velocity = new Vector2d(velocity_x.getValue(), velocity_y.getValue());

		//also needs to scale the blocks of the body:
		//   body.scale(scale.getValue());
		if (genbody != null) {
			genbody.x0.set(position);                        
			genbody.x.set(position);            
			genbody.theta = this.theta.getValue();
			genbody.omega = this.omega.getValue();
			genbody.v.set(velocity);
			genbody.updateTransformations();
			genbody.created = true;
			bodies.add(genbody);
		}
	}
	
	protected void exportDataToFile() {
		if (LCPApp.openCSV.getValue()) {
			//open file with merged if merge checkbox is checked
			if (merging.params.enableMerging.getValue() && stream == null) {
				sceneName += "_merged";
			}
			File file = new File(sceneName + ".csv");
			if (stream == null) {
				try {
					stream = new PrintStream(file);
				} catch (FileNotFoundException e) {	}
			}
			if (stream != null && LCPApp.writeToCSV.getValue()) {
				
				stream.print(bodies.size()); stream.print(", ");
				stream.print(collision.contacts.size()); stream.print(", ");
				stream.print(collision.collisionDetectTime); stream.print(", ");
				stream.print(collision.collisionSolveTime); stream.print(", ");
				stream.print(computeTime); stream.print("\n ");
			}
			if (stream != null && LCPApp.closeCSV.getValue()) {
				stream.close();
				LCPApp.openCSV.setValue(false);
			}
		}
	}
	
	BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
	DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
	DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );
	public static DoubleParameter tempSleepCount = new DoubleParameter( "temp pinned body sleep (s)", 200, 1, 10000 );
	public DoubleParameter globalViscousDecay = new DoubleParameter("global viscous decay", 1, 0.1, 1 );
	public DoubleParameter springStiffness = new DoubleParameter("spring stiffness", 100, 1, 1e4 );
	public DoubleParameter springDamping= new DoubleParameter("spring damping", 1, 0, 1000 );
	public static IntParameter springLength= new IntParameter("spring rest length", 1, 1, 100 );

	public static DoubleParameter origin_x = new DoubleParameter("x position new body", 100, -100, 200 );
	public static DoubleParameter origin_y = new DoubleParameter("y position of new body", 35, -100, 100 );
	private DoubleParameter theta = new DoubleParameter("angle of creation", 0, -180, 180 );
	private DoubleParameter velocity_x = new DoubleParameter("velocity x", -30, -100, 100 );
	private DoubleParameter velocity_y = new DoubleParameter("velocity y", 0, -100, 100 );
	private DoubleParameter omega = new DoubleParameter("angular velocity", 0, -10, 10 );
	private IntParameter index = new IntParameter("index", 0, 0, 1000 );

	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.setBorder( new TitledBorder("Rigid Body System Controls" ));
		
		vfp.add( tempSleepCount.getSliderControls(false) );
		vfp.add( useGravity.getControls() );
		vfp.add( gravityAmount.getSliderControls(false) );
		vfp.add( gravityAngle.getSliderControls(false) );

		vfp.add( globalViscousDecay.getSliderControls(false) );
		vfp.add(springStiffness.getSliderControls(false));
		vfp.add(springDamping.getSliderControls(false));
		vfp.add(springLength.getSliderControls());

		VerticalFlowPanel vfpv_2 = new VerticalFlowPanel();
		vfpv_2.setBorder( new TitledBorder("New body creation controls") );

		vfpv_2.add( origin_x.getSliderControls(false) );
		vfpv_2.add( origin_y.getSliderControls(false) );
		vfpv_2.add( theta.getSliderControls(false) );

		vfpv_2.add( velocity_x.getSliderControls(false) );
		vfpv_2.add( velocity_y.getSliderControls(false) );
		vfpv_2.add( omega.getSliderControls(false) );
		vfpv_2.add( index.getControls() );
        CollapsiblePanel vcp = new CollapsiblePanel(vfpv_2.getPanel());
        vcp.collapse();

		vfp.add(vcp);
		return vfp.getPanel();
	}
	
}

