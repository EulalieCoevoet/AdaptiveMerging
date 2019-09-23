package mergingBodies;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;

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
	
	public int nbCollections = 0;

	public CollisionProcessor collisionProcessor = new CollisionProcessor(bodies);

	public MouseSpringForce mouseSpring;

	BooleanParameter useGravity = new BooleanParameter( "enable gravity", true );
	DoubleParameter gravityAmount = new DoubleParameter( "gravitational constant", 1, -20, 20 );
	DoubleParameter gravityAngle = new DoubleParameter( "gravity angle", 90, 0, 360 );

	/** Viscous damping on all bodies */
	public DoubleParameter globalViscousDecay = new DoubleParameter("global viscous decay", 1, 0.1, 1 );
	
	/**Stiffness of  spring*/
	public DoubleParameter spring_k = new DoubleParameter("spring stiffness", 100, 1, 1e4 );

	/**Viscous damping coefficient for the  spring*/
	public DoubleParameter spring_c= new DoubleParameter("spring damping", 0, 0, 1000 );

	/**Viscous damping coefficient for the  spring*/
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
			applySpringForces(); // deal with zero length springs
		}
		
		if (processCollisions.getValue()) {
			collisionProcessor.processCollisions(dt);
		}

		if (enableMerging.getValue()) {
			mergeBodies();
			checkIndex();
			if (enableUpdateContactsInCollections.getValue())
				collisionProcessor.updateContactsInCollections(dt);
		}

		if (enableSleeping.getValue()) {
			sleep();
		}

		// advance the system by the given time step (update position and velocities of each body)
		for (RigidBody b : bodies) {
			b.advanceTime(dt); 
		}

		if (enableSleeping.getValue()) {
			wake();
		}

		if (enableMerging.getValue()) {
			if(enableUnmerging.getValue()) {
				unmergeBodies(dt);
				checkIndex();
			}
		}

		if (generateBody) {
			generateBody();
			generateBody = false;
		}
		
		double alpha = globalViscousDecay.getValue();
		for ( RigidBody b : bodies ) {
			b.deltaV.scale( alpha );
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
			b.transformB2W.transform(b.savedContactForce);
			double forceMetric1 = (b.force.x-b.forcePreSleep.x)*b.minv;
			double forceMetric2 = (b.force.y-b.forcePreSleep.y)*b.minv;
			double forceMetric3 = (b.torque-b.torquePreSleep)*b.jinv;
			b.transformW2B.transform(b.savedContactForce);
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
				applySpringForce(b, s);
			}
			//also apply spring forces to the child
			if (b instanceof RigidCollection) {
				for (RigidBody sB : ((RigidCollection) b).collectionBodies) {
					for (Spring s: sB.springs) {
						applySpringForce(sB, s);
					}
				}
			}
		}
	}

	private void applySpringForce(RigidBody b, Spring s) {
		s.updateP2();
		s.computeVelocities();

		s.apply(spring_k.getValue(), spring_c.getValue());
		s.updateP2();
	}

	private void clearJunkAtStartOfTimeStep() {
		for (RigidBody body: bodies) {
			body.merged = false;
			body.force.set(0, 0);
			body.torque = 0;
			body.deltaV.zero();

			body.savedContactForce.set(0, 0);
			body.contactTorques = 0;
			
			ArrayList<BodyPairContact> newBodyContactList = new ArrayList<BodyPairContact>();
			for (BodyPairContact bc : body.bodyPairContactList) 
				if (bc.updatedThisTimeStep) 
					newBodyContactList.add(bc);
			
			body.bodyPairContactList.clear();
			body.bodyPairContactList.addAll(newBodyContactList);

			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection)body;
				collection.unmergedThisTimeStep = false;
				for (RigidBody sB: collection.collectionBodies) {
					sB.deltaV.zero();
					sB.force.set(0, 0);
					sB.torque = 0;
					sB.merged = false;
					
					newBodyContactList.clear();
					for (BodyPairContact bpc : sB.bodyPairContactList) 
						if (bpc.merged || bpc.updatedThisTimeStep) 
							newBodyContactList.add(bpc);
					
					sB.bodyPairContactList.clear();
					sB.bodyPairContactList.addAll(newBodyContactList);
				}
			}
		}
	}

	/**
	 * Method that deals with unmerging rigidBodies... because we will explore different solutions to
	 * this problem, it will call different methods for each unmerging solution.
	 */
	private void unmergeBodies(double dt) {
		generalOneBodyAtATime(dt);
	}

	/**
	 * Goes through all bodies in each collection. 
		If a body has enough force acting on it (over a threshold), separate just that body
		from the rest of the collection. 
	 */
	private void generalOneBodyAtATime(double dt) {
		
		LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();

		for(RigidBody body : bodies) {
			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection) body;
				if (!collection.unmergedThisTimeStep) {
					
					ArrayList<RigidBody> unmergingBodies = new ArrayList<RigidBody>();
					for (RigidBody b: collection.collectionBodies) {
						boolean unmerge = collection.checkUnmergeCondition(b, dt);
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
	}

	private void unmergeSelectBodies(RigidCollection collection, ArrayList<RigidBody> unmergingBodies, ArrayList<RigidBody> newBodies) {
		ArrayList<RigidBody> handledBodies = new ArrayList<RigidBody>();

		handledBodies.addAll((unmergingBodies));
		for (RigidBody body: unmergingBodies) {
			collection.unmergeSingleBody(body);
			newBodies.add(body);
			//collection.contactsToWorld();
		}
		
		ArrayList<BodyPairContact> clearedBodyContacts = new ArrayList<BodyPairContact>();
		for (RigidBody body: unmergingBodies) {
			ArrayList<RigidBody> subBodies = new ArrayList<RigidBody>();

			for (BodyPairContact bpc : body.bodyPairContactList) {
				RigidBody otherBody = bpc.getOtherBody(body);
				clearedBodyContacts.add(bpc);
				if (bpc.merged) {
					bpc.merged = false;
					if(!handledBodies.contains(otherBody)) {

						subBodies.add(otherBody);
						handledBodies.add(otherBody);
						buildNeighborBody(otherBody, subBodies, handledBodies);

						if (subBodies.size() > 1) {
							//make a new collection
							// eulalie : I think this code should be reviewed because when you 
							// remove a body on top of a stack it creates a new collection, while it shouldn't...
							RigidCollection newCollection = new RigidCollection(subBodies.remove(0), subBodies.remove(0));
							newCollection.addBodies(subBodies);
							newCollection.fillInternalBodyContacts();
							newCollection.v.set(collection.v);
							newCollection.omega = collection.omega;
							newBodies.add(newCollection);
							subBodies.clear();
						} else if (subBodies.size() == 1) {
							collection.unmergeSingleBody(subBodies.get(0));
							newBodies.add(subBodies.remove(0));
							subBodies.clear();
						}
					}
				}
			}
			body.bodyPairContactList.clear();
			
			for (BodyPairContact bpc: clearedBodyContacts)
				bpc.removeFromBodyLists();
		}
	}

	private void buildNeighborBody(RigidBody body, ArrayList<RigidBody> subBodies, ArrayList<RigidBody> handledBodies) {

		for (BodyPairContact bpc : body.bodyPairContactList) {
			if (!bpc.merged) 
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
	 * Merges all rigidBodies in the system that fit the appropriate criteria: 
	 * <p><ul>
	 * <li> 1. They have been in contact for 50 time steps
	 * <li> 2. The relative velocities of the two bodies in contact has been below the CollisionProcessor.sleep_accum
	 * 	  value for the ENTIRETY of the contact.
	 * </ul><p>
	 */
	public void mergeBodies() {
		LinkedList<BodyPairContact> removalQueue = new LinkedList<BodyPairContact>();
		
		for (BodyPairContact bpc:collisionProcessor.bodyPairContacts) {

			boolean mergeCondition = (bpc.isRelativeVelocityDecreasing() /*|| bpc.areContactsStable()*/);
			
			if (!bpc.updatedThisTimeStep) mergeCondition = false;
			if (bpc.body1.merged && bpc.body2.merged) mergeCondition = false;
			if (bpc.body1.isInSameCollection(bpc.body2)) mergeCondition = false;
			if (bpc.body1.state == ObjectState.SLEEPING && bpc.body2.state == ObjectState.SLEEPING) mergeCondition = true;

			if (mergeCondition) {
				bpc.merged = true;
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
						bpc.body1.parent.addIncompleteCollectionContacts(bpc.body2.parent, removalQueue);
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
			}
			else {
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
		nbCollections = 0;
		for(RigidBody body: bodies) {
			body.index = i;
			i++;
			if(body instanceof RigidCollection)
				nbCollections++;
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
				}
				else {
					b.display(drawable);
				}

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
				for (BodyPairContact bc : collisionProcessor.bodyPairContacts) {
					for (Contact c : bc.contactList) {
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
		
		if (drawConnections.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					((RigidCollection)b).displayConnection(drawable);
				}
			}
		}
		
		if(drawInternalContactForces.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					((RigidCollection) b).drawInternalContacts(drawable);
				}
			}
		}
		
		if(drawInternalContactDeltas.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					((RigidCollection) b).drawInternalDeltas(drawable);
				}
			}
		}
		
		if(drawInternalHistories.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
						((RigidCollection) b).drawInternalHistory(drawable);
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
				if(drawExternalContactForces.getValue())
					c.drawContactForce(drawable);
			}
		}
		
		if ( drawCOMs.getValue() ) {
			for ( RigidBody b : bodies ) {
				b.displayCOM(drawable);
			}
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
	private BooleanParameter drawCollections = new BooleanParameter( "draw collections", true );
	private BooleanParameter drawConnections = new BooleanParameter( "draw connections", false );
	private BooleanParameter drawBoundingVolumes = new BooleanParameter( "draw root bounding volumes", false );
	private BooleanParameter drawAllBoundingVolumes = new BooleanParameter( "draw ALL bounding volumes", false );
	private BooleanParameter drawBoundingVolumesUsed = new BooleanParameter( "draw bounding volumes used", false );
	private BooleanParameter drawInternalContactForces = new BooleanParameter("draw Internal Forces of merged collections", true);
	private BooleanParameter drawExternalContactForces = new BooleanParameter("draw External Forces", true);
	private BooleanParameter drawInternalContactDeltas = new BooleanParameter("draw Internal Deltas", true);
	private BooleanParameter drawInternalHistories = new BooleanParameter("draw Internal Histories", false );

	private BooleanParameter drawCOMs = new BooleanParameter( "draw center of mass positions", true );
	private BooleanParameter drawContacts = new BooleanParameter( "draw contact locations", true);
	private BooleanParameter drawContactGraph = new BooleanParameter( "draw contact graph", true );
	private BooleanParameter drawSpeedCOM = new BooleanParameter( "draw speed COM", false );
	private BooleanParameter processCollisions = new BooleanParameter( "process collisions", true );
	public static BooleanParameter enableMerging = new BooleanParameter( "enable merging", false);
	public static BooleanParameter enableUnmerging = new BooleanParameter( "enable unmerging", false);
	public static BooleanParameter enableUpdateContactsInCollections = new BooleanParameter( "enable update contact in collection", false);
	public static BooleanParameter enableSleeping = new BooleanParameter( "enable sleeping", false);
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
		vfpv.add( drawCollections.getControls() );
		vfpv.add( drawConnections.getControls() );
		vfpv.add( transparency.getSliderControls(false));
		vfpv.add( drawBoundingVolumes.getControls() );
		vfpv.add( drawAllBoundingVolumes.getControls() );
		vfpv.add( drawBoundingVolumesUsed.getControls() );
		vfpv.add( drawInternalContactForces.getControls() );
		vfpv.add( drawExternalContactForces.getControls() );
		vfpv.add( drawInternalHistories.getControls() );
		vfpv.add( drawInternalContactDeltas.getControls() );

		vfpv.add( drawCOMs.getControls() );
		vfpv.add( drawContacts.getControls() );
		vfpv.add( drawContactGraph.getControls() );
		vfpv.add( drawSpeedCOM.getControls() );
		vfpv.add( drawIndex.getControls() );
		vfpv.add( Contact.forceVizScale.getSliderControls(true) ); // Gross?

		CollapsiblePanel cp = new CollapsiblePanel(vfpv.getPanel());
		cp.collapse();
		vfp.add( cp );

		vfp.add( processCollisions.getControls() );
		vfp.add( enableMerging.getControls() );
		vfp.add( enableUnmerging.getControls() );
		vfp.add( enableUpdateContactsInCollections.getControls() );
		vfp.add( enableSleeping.getControls() );
		vfp.add( collisionProcessor.getControls() );

		vfp.add( useGravity.getControls() );
		vfp.add( gravityAmount.getSliderControls(false) );
		vfp.add( gravityAngle.getSliderControls(false) );

		vfp.add( globalViscousDecay.getSliderControls(false) );
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
	}
}

