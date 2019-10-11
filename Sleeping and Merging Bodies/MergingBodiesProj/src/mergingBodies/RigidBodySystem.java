package mergingBodies;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mergingBodies.RigidBody.ObjectState;
import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.HorizontalFlowPanel;
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

	public CollisionProcessor collisionProcessor = new CollisionProcessor(bodies);

	public MouseSpringForce mouseSpring;

	BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
	DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
	DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );
	public static DoubleParameter tempSleepCount = new DoubleParameter( "temp pinned body sleep (s)", 1000, 1, 10000 );
	/** Viscous damping on all bodies */
	public DoubleParameter globalViscousDecay = new DoubleParameter("global viscous decay", 1, 0.1, 1 );
	
	/**Stiffness of  spring*/
	public DoubleParameter springStiffness = new DoubleParameter("spring stiffness", 100, 1, 1e4 );

	/**Viscous damping coefficient for the  spring*/
	public DoubleParameter springDamping= new DoubleParameter("spring damping", 0, 0, 1000 );

	/**Viscous damping coefficient for the  spring*/
	public static IntParameter springLength= new IntParameter("spring rest length", 1, 1, 100 );

	public boolean mergingEvent = false;
	public boolean triggerMergingEvent = false;
	
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
		
		clearJunkAtStartOfTimeStep();

		if (useGravity.getValue()) {
			applyGravityForce();
		}  

		if (mouseSpring != null) {
			mouseSpring.apply();
			applySpringForces(); 
		}
		
		if (processCollisions.getValue()) {
			collisionProcessor.processCollisions(dt); 
		}
		
		if (enableUpdateContactsInCollections.getValue()) {
			// this should be done before advanceTime() so that external forces and velocities are of the same time step
			collisionProcessor.updateContactsInCollections(dt);
		}

		for (RigidBody b : bodies) {
			b.advanceTime(dt); 
		}
		
		if (enableMerging.getValue()) {
			// this should be done after advanceTime() so that the deltaVs are applied to each bodies before merging
			mergeBodies(); 
			checkIndex();
		}
		
		if (enableSleeping.getValue()) {
			sleep();
		}
		
		if (enableSleeping.getValue()) {
			wake();
		}

		if (enableUnmerging.getValue() || unmergeAll.getValue()) {
			unmergeBodies(dt);
			checkIndex();
		}

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
	}


	/**
	 * Checks if we should put these bodies to sleep.
	 * Conditions for sleeping are:
	 * <p><p><ul>
	 * <li>1. All velocities in velocity history are below threshold. 
	 * <li>2. Velocities in history are monotonically decreasing. 
	 * </ul><p>
	 */
	private void sleep() {
		double sleepingThreshold = CollisionProcessor.sleepingThreshold.getValue();
		for (RigidBody b : bodies) {
			if (b.pinned || b.temporarilyPinned || b.state == ObjectState.SLEEPING) continue;
			double vel = b.getMetric();
			b.velHistory.add(vel);
			if (b.velHistory.size() > CollisionProcessor.sleepAccum.getValue()) {
				b.velHistory.remove(0);	
			}
			boolean sleep = true;

			double previousV = Double.MAX_VALUE; 
			double epsilon = 0.00005;
			if (b.velHistory.size() < CollisionProcessor.sleepAccum.getValue()) {

				sleep = false;
			}
			else {
				for (Double v : b.velHistory) {
					if (v > previousV+epsilon) {
						sleep = false;
						break;
					}
					if (v > sleepingThreshold) {
						sleep = false;
						break;
					}
					previousV = v;
				}
			}
			if (sleep) {
				b.state = ObjectState.SLEEPING;
				b.forcePreSleep.set(b.force);
				b.torquePreSleep = b.torque;
			}
		}
	}

	/**
	 * Checks if a body should wake up
	 * conditions for waking:
	 * If total force metric acting on body is above the forceMetric threshold.
	 * total Force metric = totalForce^2/Mass
	 * total Force = sum of all forces (including contact forces)
	 */
	private void wake() {
		double threshold = CollisionProcessor.forceMetricTolerance.getValue();
		for (RigidBody b: bodies) {
			if (b.state == ObjectState.ACTIVE || b.pinned || b.temporarilyPinned) continue;
			double forceMetric1 = (b.force.x-b.forcePreSleep.x)*b.minv;
			double forceMetric2 = (b.force.y-b.forcePreSleep.y)*b.minv;
			double forceMetric3 = (b.torque-b.torquePreSleep)*b.jinv;
			if (b.state == ObjectState.SLEEPING && (forceMetric1 > threshold || forceMetric2 > threshold || forceMetric3 > threshold)) {
				b.velHistory.clear();
				b.state = ObjectState.ACTIVE;
			}
		}
	}

	private void applyGravityForce() {
		Vector2d force = new Vector2d();
		for ( RigidBody body : bodies ) {
			//fully active, regular stepping
			double theta = gravityAngle.getValue() / 180.0 * Math.PI;
			force.set( Math.cos( theta ), Math.sin(theta) );
			force.scale( body.massLinear * gravityAmount.getValue() );
			// gravity goes directly into the accumulator!  no torque!
			body.force.add( force );
			//apply force of gravity to all children as well
			if( body instanceof RigidCollection) {
				applyGravitySubBodies((RigidCollection) body, theta);
			}
		}
	}
	
	private void applyGravitySubBodies(RigidCollection collection, double theta) {
		Vector2d force = new Vector2d();
		for (RigidBody body : collection.collectionBodies) {
			force.set( Math.cos( theta ), Math.sin(theta) );
			force.scale( body.massLinear * gravityAmount.getValue() );
			body.force.add( force );
		}
	}

	private void applySpringForces() {
		for (RigidBody b: bodies){
			for (Spring s: b.springs) {
				s.apply(springStiffness.getValue(), springDamping.getValue());
			}
		}
	}

	private void clearJunkAtStartOfTimeStep() {

		for (RigidBody body: bodies) {
			body.mergedThisTimeStep = false;
			body.force.set(0, 0);
			body.torque = 0;
			body.deltaV.zero();

			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection)body;
				collection.unmergedThisTimeStep = false;
				for (RigidBody sB: collection.collectionBodies) {
					sB.mergedThisTimeStep = false;
					sB.force.set(0, 0);
					sB.torque = 0;
					sB.deltaV.zero();
				}
			}
		}
	}
	
	/**
	 * Merges all rigidBodies in the system that fit the appropriate criteria: 
	 * <p><ul>
	 * <li> 1. They have been in contact for at least "sleepAccum" number of time steps
	 * <li> 2. The "metric" of the two bodies in contact has been below the "sleepingThreshold"
	 * 	  value for the ENTIRETY of the contact.
	 * <li> 3. The contacts have been stable for "sleepAccum" number of time steps
	 * <li> 4. Satisfies the conservative force closure: only bodies that share two
     * contacts, or cycles formed by 3 bodies with one contact between each
	 * </ul><p>
	 */
	public void mergeBodies() {
		LinkedList<BodyPairContact> removalQueue = new LinkedList<BodyPairContact>();
		
		collisionProcessor.processBodyPairContacts();

		for (BodyPairContact bpc : collisionProcessor.bodyPairContacts) {
			
			boolean mergeCondition = bpc.checkRelativeKineticEnergy();
			if (enableMergeStableContactCondition.getValue()) mergeCondition = (mergeCondition && bpc.areContactsStable());
			if (enableMergeCheckCycleCondition.getValue()) mergeCondition = (mergeCondition && bpc.checkForceClosureCritera());
			
			if (!enableMergePinned.getValue() && (bpc.body1.pinned || bpc.body2.pinned)) mergeCondition = false;
			if (bpc.body1.isInSameCollection(bpc.body2)) mergeCondition = false;
			if (bpc.body1.state == ObjectState.SLEEPING && bpc.body2.state == ObjectState.SLEEPING) mergeCondition = true;

			if (mergeCondition) {
				mergingEvent = true;
				bpc.inCollection = true;
				if(!bpc.body1.isInCollection() && !bpc.body2.isInCollection()) {
					//both are not collections: make a new collection
					bodies.remove(bpc.body1); 
					bodies.remove(bpc.body2);
					RigidCollection collection = new RigidCollection(bpc.body1, bpc.body2);
					collection.addInternalContact(bpc);
					bodies.add(collection);
				}
				else if (bpc.body1.isInCollection() && bpc.body2.isInCollection() && !bpc.body1.isInSameCollection(bpc.body2)) {
					//both are collections:
					//take all the bodies in the least massive one and add them to the collection of the most massive
					if (bpc.body1.parent.massLinear > bpc.body2.parent.massLinear) {
						bodies.remove(bpc.body2.parent);
						bpc.body1.parent.addCollection(bpc.body2.parent);
						bpc.body1.parent.addInternalContact(bpc);
						bpc.body1.parent.addIncompleteCollectionContacts(bpc.body2.parent, removalQueue);
					}
					else {
						bodies.remove(bpc.body1.parent);
						bpc.body2.parent.addCollection(bpc.body1.parent);
						bpc.body2.parent.addInternalContact(bpc);
						bpc.body2.parent.addIncompleteCollectionContacts(bpc.body1.parent, removalQueue);
					}
				}
				else if (bpc.body1.isInCollection()) {
					//body1 is in a collection, body2 is not
					bodies.remove(bpc.body2);
					bpc.body1.parent.addBody(bpc.body2);
					bpc.body1.parent.addInternalContact(bpc);
					bpc.body1.parent.addIncompleteContacts(bpc.body2, removalQueue);
				}
				else if (bpc.body2.isInCollection()) {
					//body2 is in a collection, body1 is not
					bodies.remove(bpc.body1);
					bpc.body2.parent.addBody(bpc.body1);
					bpc.body2.parent.addInternalContact(bpc);
					bpc.body2.parent.addIncompleteContacts(bpc.body1, removalQueue);
				}
				removalQueue.add(bpc);
			}
		}
		
		for (BodyPairContact element : removalQueue) {
			collisionProcessor.bodyPairContacts.remove(element);
		}
	}

	/**
	 * Method that deals with unmerging rigidBodies... because we will explore different solutions to
	 * this problem, it will call different methods for each unmerging solution.
	 * Goes through all bodies in each collection. 
		If a body has enough force acting on it (over a threshold), separate just that body
		from the rest of the collection. 
	 */
	private void unmergeBodies(double dt) {
		LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();

		for(RigidBody body : bodies) {
			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection) body;
				if (!collection.unmergedThisTimeStep && !collection.temporarilyPinned) {
					
					ArrayList<RigidBody> unmergingBodies = new ArrayList<RigidBody>();
					for (RigidBody b: collection.collectionBodies) {
						boolean unmerge = collection.checkUnmergeCondition(b, dt, 
																			enableUnmergeMovingCondition.getValue(), 
																			enableUnmergeNormalCondition.getValue(), 
																			enableUnmergeFrictionCondition.getValue());
						unmerge = (unmergeAll.getValue() || unmerge); 
						if (unmerge) 
							unmergingBodies.add(b);
					}
					
					ArrayList<RigidBody> newBodies = new ArrayList<RigidBody>();
					if (!unmergingBodies.isEmpty()) {
						unmergeSelectBodies(collection, unmergingBodies, newBodies);				
					}

					if (!newBodies.isEmpty()) {
						for (RigidBody bd: newBodies) {
							additionQueue.add(bd);
						}
						removalQueue.add(collection);
						newBodies.clear();
					}
				}
			}
		}
		for (RigidBody b: additionQueue) {
			bodies.add(b);
		}
		for (RigidBody b: removalQueue) {
			bodies.remove(b);
		}
		
		processCollectionsColor();
		unmergeAll.setValue(false);
	}

	private void unmergeSelectBodies(RigidCollection collection, ArrayList<RigidBody> unmergingBodies, ArrayList<RigidBody> newBodies) {

		mergingEvent = true;
		
		ArrayList<RigidBody> handledBodies = new ArrayList<RigidBody>();

		handledBodies.addAll((unmergingBodies));
		for (RigidBody body: unmergingBodies) {
			collection.unmergeSingleBody(body);
			newBodies.add(body);
		}
		
		ArrayList<BodyPairContact> clearedBodyPairContacts = new ArrayList<BodyPairContact>();
		for (RigidBody body: unmergingBodies) {
			ArrayList<RigidBody> subBodies = new ArrayList<RigidBody>();

			for (BodyPairContact bpc : body.bodyPairContactList) {
				RigidBody otherBody = bpc.getOtherBody(body);
				clearedBodyPairContacts.add(bpc);
				if (bpc.inCollection) {
					bpc.inCollection = false;
					if(!handledBodies.contains(otherBody)) {

						subBodies.add(otherBody);
						handledBodies.add(otherBody);
						buildNeighborBody(otherBody, subBodies, handledBodies);

						if (subBodies.size() > 1) {
							//make a new collection
							RigidCollection newCollection = new RigidCollection(subBodies.remove(0), subBodies.remove(0));
							newCollection.addBodies(subBodies);
							newCollection.fillInternalBodyContacts();
							newCollection.color = new Color3f(collection.color);
							collection.applyVelocitiesTo(newCollection);
							newBodies.add(newCollection);
							subBodies.clear();
						} else if (subBodies.size() == 1) {
							collection.unmergeSingleBody(subBodies.get(0));
							collection.applyVelocitiesTo(body);
							newBodies.add(subBodies.remove(0));
							subBodies.clear();
						}
					}
				}
			}
			body.bodyPairContactList.clear();
			
			for (BodyPairContact bpc: clearedBodyPairContacts) {
				// Store in contacts map for warm start
				for (Contact contact : bpc.contactList) {
					collisionProcessor.contacts.add(contact);
					Block block1 = contact.block1;
					Block block2 = contact.block2;
					collisionProcessor.lastTimeStepMap.put("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode()), contact);
				}
				bpc.removeFromBodyLists();
			}
		}			
	}

	private void buildNeighborBody(RigidBody body, ArrayList<RigidBody> subBodies, ArrayList<RigidBody> handledBodies) {

		for (BodyPairContact bpc : body.bodyPairContactList) {
			if (!bpc.inCollection) 
				continue;

			RigidBody otherBody = bpc.getOtherBody(body);
			if (!handledBodies.contains(otherBody)) {
				handledBodies.add(otherBody);
				subBodies.add(otherBody);
				buildNeighborBody(otherBody, subBodies, handledBodies);
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
		int size = bodies.size();
		int counter = 0;
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
		collisionProcessor.reset();
		totalAccumulatedComputeTime = 0; 
		totalSteps = 0;
	}

	/**
	 * if there are bodies with the index i, ups the index of all bodies greater than i to leave room 
	 * for the new body about to be "reintroduced" to the scene
	 * TODO: remove me?  Can we remove all body indices?
	 */
	@Deprecated
	private void checkIndex() {
		int i = 0;
		for(RigidBody body: bodies) {
			body.index = i;
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
	 * Process collections color:
	 * <p><ul>
	 * <li> if two collections have the same color, the most massive one will keep it
	 * </ul><p>
	 */
	protected void processCollectionsColor() {
		ArrayList<Color3f> colors = new ArrayList<Color3f>();
		ArrayList<RigidCollection> collections = new ArrayList<RigidCollection>();
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection)body;
				if (colors.contains(collection.color)) {
					RigidCollection sameColorCollection = collections.get(colors.indexOf(collection.color));
					if(sameColorCollection.massLinear>collection.massLinear)
						collection.generateColor();
					else
						sameColorCollection.generateColor();
				}
				colors.add(collection.color);
				collections.add(collection);
			}
		}
	}

	/**
	 * Draws all rigid bodies
	 * @param drawable
	 */
	public void display( GLAutoDrawable drawable ) {
		
		boolean updateCollectionColor = false;
		
		GL2 gl = drawable.getGL().getGL2();
		if ( Block.alpha != (float) (double) transparency.getValue()) {
			Block.alpha = (float) (double) transparency.getValue();
			// gross... need to rebuild display lists for the currently set transparency
			// which is potentially slow and bad news (memory thrashing) for openGL if we do this excessively!
			RigidBody.clearDisplayLists( gl );
			updateCollectionColor = true;
			for ( RigidBody b : bodies ) {
				b.myListID = -1;
			}
		}  
		
		if ( drawBodies.getValue() ) {
			
			// Check if collections color should be 
			for ( RigidBody b : bodies ) {
				if ( b instanceof RigidCollection && b.updateColor == true) {
					b.updateColor = false;
					updateCollectionColor = true;
				}	
			}
			
			for ( RigidBody b : bodies ) {
				if ( b instanceof RigidCollection) {
					RigidCollection collection = (RigidCollection)b;
					Color3f color = null;
					if(drawCollections.getValue() && updateCollectionColor)
						color = new Color3f(collection.color);
					collection.displayCollection(drawable, color);
				} else {
					b.display(drawable);
				}

				for (Spring s : b.springs) {
					s.displaySpring(drawable);
				}
			}
		}

		gl.glLineWidth(1);
		if ( drawBoundingVolumes.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection)) {
					b.root.boundingDisc.display(drawable);
				}
			}
		}
		
		if ( drawAllBoundingVolumes.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection)) {
					b.root.display( drawable );
				} else {
					displayCollectionBV((RigidCollection) b, drawable);
				}
			}
		}      
		
		if ( drawBoundingVolumesUsed.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection)) {
					b.root.displayVisitBoundary( drawable, collisionProcessor.visitID );
				} else {
					displayVisitBoundaryCollection((RigidCollection) b, drawable);
				}
			}
		}
		
		if ( drawContactGraph.getValue() ) {
			if (CollisionProcessor.useContactGraph.getValue()) {
				for (BodyPairContact bpc : collisionProcessor.bodyPairContacts) {
					for (Contact c : bpc.contactList) {
						c.displayContactGraph(drawable);
					}
				} 
			} 
			else {
				for ( Contact c : collisionProcessor.contacts ) {
					c.displayContactGraph(drawable);
				}
			}
		}
		
		if (drawCollectionContactGraph.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					((RigidCollection)b).displayContactGraph(drawable);
				}
			}
		}
		
		if(drawInternalHistories.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
						((RigidCollection) b).displayInternalHistory(drawable);
				}
			}
		}
		
		if ( drawSpeedCOMs.getValue() ) {
			for (RigidBody b: bodies) {
				b.displaySpeedCOM(drawable);
			}
		}

		if (drawContactLocations.getValue() || drawContactForces.getValue()) {
			for ( Contact c : collisionProcessor.contacts ) {
				if (drawContactLocations.getValue()) 
					c.displayContactLocation(drawable, new Color3f(1,0,0)); 
				if (drawContactForces.getValue())
					c.displayContactForce(drawable, new Color3f(1,0,0));
			}
		}
		
		if (drawContactLocations.getValue() || drawContactForcesInCollection.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					RigidCollection collection = (RigidCollection)b;
					if (drawContactLocations.getValue())
						collection.displayInternalContactLocations(drawable);
					if (drawContactForcesInCollection.getValue())
						collection.displayInternalContactForces(drawable);
				}
			}
		}
		
		if ( drawCOMs.getValue() ) {
			for ( RigidBody b : bodies ) {
				b.displayCOMs(drawable);
			}
		}
		
		if ( drawIndex.getValue()) {
			for (RigidBody b : bodies) {
				b.displayIndex(drawable, GLUT.BITMAP_8_BY_13);
			}
		}
	}

	private void displayVisitBoundaryCollection(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.collectionBodies) {
			if (body instanceof RigidCollection) {
				displayVisitBoundaryCollection((RigidCollection) body, drawable);
			}
			else {
				body.root.displayVisitBoundary(drawable, collisionProcessor.visitID);
			}
		}
	}

	private void displayCollectionBV(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.collectionBodies) {
			if (body instanceof RigidCollection) {
				displayCollectionBV((RigidCollection) body, drawable);
			}
			else {
				body.root.display(drawable);
			}
		}

	}

	private DoubleParameter transparency = new DoubleParameter("body block transparency", .5, 0, 1 );
	private BooleanParameter drawBodies = new BooleanParameter( "draw bodies", true );
	private BooleanParameter drawCollections = new BooleanParameter( "draw collections with different colors", true );
	
	private BooleanParameter drawBoundingVolumes = new BooleanParameter( "draw root bounding volumes", false );
	private BooleanParameter drawBoundingVolumesUsed = new BooleanParameter( "draw bounding volumes used", false );
	private BooleanParameter drawAllBoundingVolumes = new BooleanParameter( "draw ALL bounding volumes", false );
	
	private BooleanParameter drawContactForces = new BooleanParameter("draw contact forces", true);
	private BooleanParameter drawContactForcesInCollection = new BooleanParameter("draw contact forces in collections", true);
	private BooleanParameter drawContactLocations = new BooleanParameter( "draw contact locations", true);
	private BooleanParameter drawInternalHistories = new BooleanParameter("draw internal histories", false );
	private BooleanParameter drawContactGraph = new BooleanParameter( "draw contact graph", true );
	private BooleanParameter drawCollectionContactGraph = new BooleanParameter( "draw collections' contact graph", false );
	
	private BooleanParameter drawCOMs = new BooleanParameter( "draw COM", true );
	private BooleanParameter drawSpeedCOMs = new BooleanParameter( "draw speed COM", false );
	public BooleanParameter drawIndex = new BooleanParameter( "dawIndex", false );
	
	private BooleanParameter processCollisions = new BooleanParameter( "process collisions", true );
	
	public BooleanParameter enableMerging = new BooleanParameter( "merging", true);
	public BooleanParameter enableMergePinned = new BooleanParameter( "merging pinned body", true);
	public BooleanParameter enableMergeCheckCycleCondition = new BooleanParameter( "merging check cycle condition", true);
	public BooleanParameter enableMergeStableContactCondition = new BooleanParameter( "merging stable contact condition", true);
	public BooleanParameter enableUnmerging = new BooleanParameter( "unmerging", true);
	public BooleanParameter enableUnmergeFrictionCondition = new BooleanParameter( "unmerging friction condition", true);
	public BooleanParameter enableUnmergeNormalCondition = new BooleanParameter( "unmerging contact normal condition", true);
	public BooleanParameter enableUnmergeMovingCondition = new BooleanParameter( "unmerging moving condition", false);
	public BooleanParameter enableUpdateContactsInCollections = new BooleanParameter( "update contact in collection", true);
	public BooleanParameter unmergeAll = new BooleanParameter("unmerge all", false);
	public static BooleanParameter enableSleeping = new BooleanParameter( "sleeping", false);


	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.setBorder( new TitledBorder("Rigid Body System Controls" ));

		VerticalFlowPanel vfpv = new VerticalFlowPanel();
		vfpv.setBorder( new TitledBorder("viewing controls") );
		vfpv.add( transparency.getSliderControls(false));
		vfpv.add( drawBodies.getControls() );
		vfpv.add( drawCollections.getControls() );
		
		vfpv.add( drawBoundingVolumes.getControls() );
		vfpv.add( drawBoundingVolumesUsed.getControls() );
		vfpv.add( drawAllBoundingVolumes.getControls() );
		
		vfpv.add( drawContactForces.getControls() );
		vfpv.add( drawContactForcesInCollection.getControls() );
		vfpv.add( drawContactLocations.getControls() );
		vfpv.add( drawInternalHistories.getControls() );
		vfpv.add( drawContactGraph.getControls() );
		vfpv.add( drawCollectionContactGraph.getControls() );

		vfpv.add( drawCOMs.getControls() );
		vfpv.add( drawSpeedCOMs.getControls() );
		vfpv.add( drawIndex.getControls() );
		vfpv.add( Contact.forceVizScale.getSliderControls(true) ); // Gross?

		CollapsiblePanel cp = new CollapsiblePanel(vfpv.getPanel());
		cp.collapse();
		vfp.add( cp );
		
		VerticalFlowPanel vfpm = new VerticalFlowPanel();
		vfpm.setBorder( new TitledBorder("merging unmerging rules") );
		vfpm.add( enableMerging.getControls() );
		vfpm.add( enableMergePinned.getControls() );
		vfpm.add( enableMergeCheckCycleCondition.getControls() );
		vfpm.add( enableMergeStableContactCondition.getControls() );
		vfpm.add( enableUnmerging.getControls() );
		vfpm.add( enableUnmergeFrictionCondition.getControls() );
		vfpm.add( enableUnmergeNormalCondition.getControls() );
		vfpm.add( enableUnmergeMovingCondition.getControls() );
		vfpm.add( enableUpdateContactsInCollections.getControls() );
        JButton umergeButton = new JButton("unmerge all");
        vfpm.add( umergeButton);
        umergeButton.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	unmergeAll.setValue(true);
            }
        });
		vfpm.add( enableSleeping.getControls() );
		CollapsiblePanel cpm = new CollapsiblePanel(vfpm.getPanel());
		cpm.collapse();
		vfp.add( cpm );

		vfp.add( processCollisions.getControls() );		
		vfp.add( collisionProcessor.getControls() );
		
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
	}
}

