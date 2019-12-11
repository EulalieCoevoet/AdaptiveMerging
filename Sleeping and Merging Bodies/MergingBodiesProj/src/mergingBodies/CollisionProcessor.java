package mergingBodies;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;
import mergingBodies.RigidBodySystem.MergeParameters;

/**
 * Class for detecting and resolving collisions. Currently this class uses penalty forces between rigid bodies.
 * @author kry
 */
public class CollisionProcessor {

	public List<RigidBody> bodies;

	public HashMap<String, Contact> lastTimeStepMap = new HashMap<String, Contact>();
	
	/**
	 * The current contacts that resulted in the last call to process collisions
	 */
	public ArrayList<Contact> contacts = new ArrayList<Contact>();
	
	/** body-body contacts for pruning */
	ArrayList<Contact> tmpContacts = new ArrayList<Contact>();
	
	/** PGS solver */
	PGS solver = new PGS();

	/**
	 * Default constructor
	 */
	public CollisionProcessor() {
		bodies = null;
	}
	
	/**
	 * Creates this collision processor with the provided set of bodies
	 * @param bodies
	 */
	public CollisionProcessor( List<RigidBody> bodies ) {
		this.bodies = bodies;
	}

	/** keeps track of the time used for collision detection on the last call */
	double collisionDetectTime = 0;

	/** keeps track of the time used to solve the LCP based velocity update on the last call */
	double collisionSolveTime = 0;
	
	/** keeps track of the time used to update the contacts inside the collections on the last call */
	double collectionUpdateTime = 0;
	
	/**list that keeps track of all the body pair contacts that occurred in this time step */
	public ArrayList<BodyPairContact> bodyPairContacts = new ArrayList<BodyPairContact>();

	/**
	 * Find collision points
	 * @param dt time step
	 */
	public void collisionDetection( double dt ) {

		Contact.nextContactIndex = 0;
		
		contacts.clear();
		
		// detect contacts and fill contacts list
		long now = System.nanoTime();
		broadPhase(); 
		collisionDetectTime = ( System.nanoTime() - now ) * 1e-9;
		
		if (contacts.isEmpty())
			lastTimeStepMap.clear();
	}
	
	/**
	 * Solve LCP
	 * @param dt time step
	 */
	public void solveLCP( double dt ) {
		
		if(!contacts.isEmpty()) {
	    	
	    	if (shuffle.getValue()) 
	    		knuthShuffle();
	    	
	    	// set up contacts solver 
			solver.init(friction.getValue(), iterations.getValue());
			solver.restitution = restitution.getValue();
			solver.feedbackStiffness = feedbackStiffness.getValue();
			solver.compliance = (enableCompliance.getValue())? compliance.getValue() : 0.;
			solver.warmStart = true;
			solver.contacts = contacts;

			// eulalie: this can be optimized, only new collections need an update 
			for (Contact contact: solver.contacts)
				contact.computeJacobian(false);
			
			// solve contacts
			long now = System.nanoTime();
			solver.solve(dt);
			collisionSolveTime = (System.nanoTime() - now) * 1e-9;
			
			updateContactsMap();
			computeContactsForce(dt);	
		}
	}
	
	/**
	 * Updates bodyPairContacts list
	 */
	public void updateBodyPairContacts() {
		
		for (BodyPairContact bpc : bodyPairContacts) // clear contactList of existing bpc
			bpc.contactList.clear();

		for (Contact contact : contacts) // loop over all contacts between bodies
			storeInBodyPairContacts(contact);
		
		removeEmptyBodyPairContacts();
	}
	
	/**
	 * Remove inactive contact from bodyPairContacts list
	 */
	public void clearBodyPairContacts() {
		
		for (BodyPairContact bpc : bodyPairContacts) {
			ArrayList<Contact> tmpContacts = new ArrayList<Contact>();
			for (Contact contact : bpc.contactList) 
				if (Math.abs(contact.lambda.x) > 1e-14) // (math.abs is for magnet)
					tmpContacts.add(contact);
			bpc.contactList.clear();
			bpc.contactList.addAll(tmpContacts);
		}

		removeEmptyBodyPairContacts();
	}
	
	/**
	 * Remove empty body pair contacts
	 */
	protected void removeEmptyBodyPairContacts() {
		ArrayList<BodyPairContact> tmpBodyPairContacts = new ArrayList<BodyPairContact>();
		for (BodyPairContact bpc : bodyPairContacts) {
			if(!bpc.contactList.isEmpty())
				tmpBodyPairContacts.add(bpc);
			else {
				bpc.removeFromBodyLists();
				bpc.removeFromBodyListsParent();
			}
		}
		
		bodyPairContacts.clear();
		bodyPairContacts.addAll(tmpBodyPairContacts);
	}
	
	/**
	 * Does one iteration of PGS to update contacts inside collections
	 * @param dt
	 */
	public void updateInCollections(double dt) {

		long now = System.nanoTime();
		
		solver.init(friction.getValue(), iterationsInCollection.getValue());
		solver.computeInCollection = true;
		solver.restitution = restitution.getValue();
		solver.feedbackStiffness = feedbackStiffness.getValue();
		solver.compliance = (enableCompliance.getValue())? compliance.getValue() : 0.;
		solver.warmStart = true;
		
		solver.contacts = new ArrayList<Contact>();
		solver.contacts.addAll(contacts);
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection && !body.isSleeping) {
				RigidCollection collection = (RigidCollection)body;
				solver.contacts.addAll(collection.internalContacts);
			}
		}
		
		// eulalie: this can be optimized, only collections need an update 
		for (Contact contact: solver.contacts) 
			contact.computeJacobian(true);
		
		solver.solve(dt);
		
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection && !body.isSleeping) {
				
				RigidCollection collection = (RigidCollection)body;
				collection.computeInternalContactsForce(dt);
				
				for (RigidBody b : collection.bodies)
					if(!b.pinned && !b.temporarilyPinned)
						b.advanceVelocities(dt);	
			}
			body.deltaV.zero();
		}
		
		collectionUpdateTime = ( System.nanoTime() - now ) * 1e-9;
	}
	
	protected void updateContactsMap() {
		lastTimeStepMap.clear();
		for (Contact contact : contacts) {
			Block block1 = contact.block1;
			Block block2 = contact.block2;
			lastTimeStepMap.put("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode()), contact);
		} 
	}
	
	public void updateContactsHistory(MergeParameters mergeParams) {
		for (Contact c: contacts) {
			c.body1ContactForceHistory.add(c.contactForceB1);
			c.body1ContactTorqueHistory.add(c.contactTorqueB1);
			if (c.body1ContactForceHistory.size() > mergeParams.stepAccum.getValue()) {
				c.body1ContactForceHistory.remove(0);
				c.body1ContactTorqueHistory.remove(0);
			}
	
			c.body2ContactForceHistory.add(c.contactForceB2);
			c.body2ContactTorqueHistory.add(c.contactTorqueB2);
			if (c.body2ContactForceHistory.size() > mergeParams.stepAccum.getValue()) {
				c.body2ContactForceHistory.remove(0);
				c.body2ContactTorqueHistory.remove(0);
			}
		}
	}

	/**
	 * Compute the contact force J*lambda.
	 * @param dt
	 */
	public void computeContactsForce(double dt) {
		for (Contact c: contacts)
			c.computeContactForce(false, dt);	
	}

	/**go through each element in contacts 2.
	* at each element, swap it for another random member of contacts2. 
	* at each element, get a random index from i to contacts.size.
	* swap this element with that element at that index. 
	* go to next element in contacts 2
	*/
	private void knuthShuffle() {
		Collections.shuffle(contacts);
		for (Contact c : contacts) {
			c.index = contacts.indexOf(c);
		}
	}

	/**
	 * Checks for collisions between bodies. Note that you can optionally implement some broad
	 * phase test such as spatial hashing to reduce the n squared body-body tests.
	 * Currently this does the naive n squared collision check.
	 * Has prune option.
	 */
	private void broadPhase() {
		
		// Naive n squared body test.. might not be that bad for small number of bodies 
		visitID++;
		for ( RigidBody b1 : bodies ) {
			for ( RigidBody b2 : bodies ) { // not so inefficient given the continue on the next line
				
				if ( b1.index >= b2.index ) continue;
				if ( (b1.pinned || b1.temporarilyPinned) && (b2.pinned || b2.temporarilyPinned) ) continue; 
				
				tmpContacts.clear();
				narrowPhase( b1, b2 );
				
				if ( pruneContacts.getValue() && tmpContacts.size() > 2 ) {
					if ( b1 instanceof RigidCollection ) {
						RigidCollection collection = (RigidCollection) b1;
						pruneCollection(collection);
					} else if ( b2 instanceof RigidCollection ) {
						RigidCollection collection = (RigidCollection) b2;
						pruneCollection(collection);
					} else {
						prune(tmpContacts);
					}
				}
				
				contacts.addAll(tmpContacts);
			}
		}        
	}
	
	protected void pruneCollection(RigidCollection collection) {
		
		ArrayList<Contact> collectionContacts = new ArrayList<Contact>();
		ArrayList<Contact> contacts = new ArrayList<Contact>();
		for (RigidBody body : collection.bodies) {
			contacts.clear();
			for (Contact contact : tmpContacts)
				if (contact.withBody(body))
					contacts.add(contact);
			if (contacts.size()>2)
				prune(contacts);
			collectionContacts.addAll(contacts);
		}
		tmpContacts.clear();
		tmpContacts.addAll(collectionContacts);
	}

	/**
	 * Prune tmpBodyBodyContacts list
	 */
	protected void prune(ArrayList<Contact> contacts) {
		
		Point2d meanPos = new Point2d();
		Vector2d v = new Vector2d();
		int N = contacts.size();
		for ( Contact c : contacts ) {
			meanPos.add( c.contactW );						
		}
		
		meanPos.scale( 1.0 / N );
		Matrix2d covariance = new Matrix2d();
		for ( Contact c : contacts ) {
			v.sub( c.contactW, meanPos );						
			covariance.rank1( 1.0 / N, v );
		}
		
		covariance.evd();
		double eps = epsilonPruneConvexHull.getValue();
		if ( covariance.ev1 <= eps || covariance.ev2 <= eps ) { // not a flat region... could do convex hull
			
			Vector2d dir = null;
			if ( covariance.ev1 <= eps ) {
				dir = covariance.v2;
			} else {
				dir = covariance.v1;
			}
			
			// now search for the farthest contacts in this direction!
			double minDot = Double.MAX_VALUE;
			double maxDot = Double.MIN_VALUE;
			Contact cmin = null;
			Contact cmax = null;
			double minDotViolation = Double.MAX_VALUE;
			double maxDotViolation = Double.MAX_VALUE;
			eps = 1e-2;
			
			for ( Contact c : contacts ) {
				v.sub( c.contactW, meanPos );
				double dot = v.dot( dir ); 
				if ( dot > maxDot & c.constraintViolation <= maxDotViolation + eps) {
					maxDot = dot;
					cmax = c;
					maxDotViolation = c.constraintViolation;
				}
				if ( dot < minDot & c.constraintViolation <= minDotViolation + eps) {
					minDot = dot;
					cmin = c;
					minDotViolation = c.constraintViolation;
				}
			}
			
			contacts.clear();
			contacts.add( cmax );
			contacts.add( cmin );
		}
	}
	
	/**
	 * Store contact in bodyPairContacts.
	 * @param contact newly detected contact
	 */
	private void storeInBodyPairContacts(Contact contact) {

		// eulalie : I think this will never happen as we don't detect collision between two pinned bodies
		if (contact.body1.pinned && contact.body2.pinned) 
			return;
		
		BodyPairContact bpc = BodyPairContact.checkExists(contact.body1, contact.body2, bodyPairContacts);

		if (bpc == null) { // body contact did not exist in previous list
			bpc = new BodyPairContact(contact.body1, contact.body2);
			bodyPairContacts.add(bpc);
		} 
		
		bpc.addToBodyLists();
		bpc.addToBodyListsParent();
		bpc.contactList.add(contact);
	}
	
	/**
	 * Fills lambdas with values from previous time step
	 */
	protected void warmStart() {
		for (Contact contact : contacts) {
			Contact oldContact = getOldContact(contact);
			if(oldContact != null) { 
				contact.lambda.x = oldContact.lambda.x;
				contact.lambda.y = oldContact.lambda.y;
				contact.prevConstraintViolation = oldContact.constraintViolation;
			}
		}
	}
	
	/**
	 * Checks if lastTimeStepMap (contact map of last time step) contains a similar contact and returns the corresponding contact. 
	 * @param contact
	 * @return oldContact
	 */
	protected Contact getOldContact(Contact contact) {
		
		Contact oldContact;
		
		Block block1 = contact.block1;
		Block block2 = contact.block2;

		if(lastTimeStepMap.containsKey("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode() ))
				|| lastTimeStepMap.containsKey("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode() ))) {

			RigidBody body1 = (contact.body1.isInCollection())? contact.body1.parent: contact.body1;
			RigidBody body2 = (contact.body2.isInCollection())? contact.body2.parent: contact.body2;

			// if the old map contains this key, then get the lambda of the old map
			oldContact = lastTimeStepMap.get("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode()));
			if(lastTimeStepMap.containsKey("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode() )))
				oldContact = lastTimeStepMap.get("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode()));
			
			RigidBody oldBody1 = (oldContact.body1.isInCollection())? oldContact.body1.parent: oldContact.body1;
			RigidBody oldBody2 = (oldContact.body2.isInCollection())? oldContact.body2.parent: oldContact.body2;
			
			if ((oldBody1 != body1 && oldBody1 != body2) || (oldBody2 != body2 && oldBody2 != body1)) 
				return null;
		}
		else
			return null;
		
		return oldContact;
	}
		
	/**
	 * Checks for collision between boundary blocks on two rigid bodies.
	 * @param body1
	 * @param body2
	 */
	private void narrowPhase( RigidBody body1, RigidBody body2 ) {
		if (body1 instanceof RigidCollection || body2 instanceof RigidCollection) {
			narrowCollection(body1, body2);
		} else {
			if ( body1 instanceof PlaneRigidBody ) {
				findCollisionsWithPlane( body2.root, body2, (PlaneRigidBody) body1 ); 
			} else if (body2 instanceof PlaneRigidBody ) {
				findCollisionsWithPlane( body1.root, body1, (PlaneRigidBody) body2 );
			} else {
				findCollisions(body1.root, body2.root, body1, body2);
			}
		}
	}

	/**
	 * Recursive method that makes us check for collisions with each body in a rigidCollection
	 */
	private void narrowCollection(RigidBody body1, RigidBody body2) {
		if (body1 instanceof RigidCollection && body2 instanceof RigidCollection) {
			for (RigidBody b: ((RigidCollection) body1).bodies) {
				narrowCollection(b, body2);
			}
		} else if (body1 instanceof RigidCollection) {
			for (RigidBody b: ((RigidCollection) body1).bodies) {
				if ( b instanceof PlaneRigidBody ) {
					findCollisionsWithPlane( body2.root, body2, (PlaneRigidBody) b ); 
				} else if (body2 instanceof PlaneRigidBody ) {
					findCollisionsWithPlane( b.root, b, (PlaneRigidBody) body2 );
				} else {
					findCollisions(b.root, body2.root, b, body2);
				}
			}
		} else if (body2 instanceof RigidCollection) {
			for (RigidBody b: ((RigidCollection) body2).bodies) {
				if ( b instanceof PlaneRigidBody ) {
					findCollisionsWithPlane( body1.root, body1, (PlaneRigidBody) b ); 
				} else if (body1 instanceof PlaneRigidBody ) {
					findCollisionsWithPlane( b.root, b, (PlaneRigidBody) body1 );
				} else {
					findCollisions(body1.root, b.root, body1, b);
				}
			}
		}
	}
	
	private void findCollisionsWithPlane( BVNode node1, RigidBody body1, PlaneRigidBody planeBody ) {
		if(node1.visitID != visitID) {
			node1.visitID = visitID;
			node1.boundingDisc.updatecW();
		}
		// check bounding disc with plane
		Tuple2d c = node1.boundingDisc.cW;
		Tuple2d n = planeBody.n;
		double d = n.x*c.x + n.y*c.y + planeBody.d - node1.boundingDisc.r;
		if ( d < 0 ) {
			if ( node1.isLeaf() ) {
				// create a collision here!
				
				// contact position in world coordinates will be the closest point on the plane
				// using the temp working variable normal, but don't get confused by the name!!!
				// just computing p = c-n (n \cdot (c-x))
				normal.sub( c, planeBody.p ); 
				double val = normal.dot( planeBody.n );
				normal.scale( val, planeBody.n );
				contactW.sub( c, normal );
				normal.scale( -1, planeBody.n );
				
				Contact contact = new Contact(body1, planeBody, contactW, normal, node1.leafBlock, planeBody.dummyBlock, d);
				// put contact into a preliminary list that will be filtered in BroadPhase
				tmpContacts.add( contact );
			} else {
				findCollisionsWithPlane( node1.child1, body1,planeBody );
				findCollisionsWithPlane( node1.child2, body1,planeBody );
			}
		}		
	}
	
	/** 
	 * Recurses through all of body_1, then body_2
	 * @param node1
	 * @param node2
	 * @param body1
	 * @param body2
	 */
	private void findCollisions(BVNode node1, BVNode node2, RigidBody body1, RigidBody body2) {
		
		if(node1.visitID != visitID) {
			node1.visitID = visitID;
			node1.boundingDisc.updatecW();
		}
		if (node2.visitID != visitID) {
			node2.visitID = visitID;
			node2.boundingDisc.updatecW();
		}

		if (node1.boundingDisc.intersects(node2.boundingDisc)) {
			if (node1.isLeaf() && node2.isLeaf()) {
				Block leafBlock1 = node1.leafBlock;
				Block leafBlock2 = node2.leafBlock;

				processCollision(body1, leafBlock1, body2, leafBlock2);
				
			} else if(node1.isLeaf()|| node1.boundingDisc.r <= node2.boundingDisc.r){
				//if they overlap, and body 1 is either a leaf or smaller than body_2, break down body_2

				findCollisions(node1, node2.child1, body1, body2);
				findCollisions(node1, node2.child2, body1, body2);
			} else if(node2.isLeaf() || node2.boundingDisc.r <= node1.boundingDisc.r) {
				//if they overlap, and body 2 is either a leaf or smaller than body_1, break down body_1

				findCollisions(node1.child1, node2, body1, body2);
				findCollisions(node1.child2, node2, body1, body2);
			}
		}
	}

	/** 
	 * The visitID is used to tag boundary volumes that are visited in 
	 * a given time step.  Marking boundary volume nodes as visited during
	 * a time step allows for a visualization of those used, but it can also
	 * be used to more efficiently update the centers of bounding volumes
	 * (i.e., call a BVNode's updatecW method at most once on any given time step)
	 */
	int visitID = 0;

	/**
	 * Resets the state of the collision processor by clearing all
	 * currently identified contacts, and reseting the visitID for
	 * tracking the bounding volumes used
	 */
	public void reset() {
		contacts.clear();
		Contact.nextContactIndex = 0;
		visitID = 0;            
	}

	// some working variables for processing collisions
	private Point2d tmp1 = new Point2d();
	private Point2d tmp2 = new Point2d();
	private Point2d contactW = new Point2d();
	private Vector2d normal = new Vector2d();

	/**
	 * Processes a collision between two bodies for two given blocks that are colliding.
	 * @param body1
	 * @param b1
	 * @param body2
	 * @param b2
	 */
	private void processCollision( RigidBody body1, Block b1, RigidBody body2, Block b2 ) {        

		body1.transformB2W.transform( b1.pB, tmp1 );
		body2.transformB2W.transform( b2.pB, tmp2 );
		double distance = tmp1.distance(tmp2);

		if ( distance < Block.radius * 2 ) {
			// contact point at halfway between points 
			// NOTE: this assumes that the two blocks have the same radius!
			contactW.interpolate( tmp1, tmp2, 0.5 );
			normal.sub( tmp2, tmp1 );
			normal.normalize();
			
			Contact contact = null;
			contact = new Contact(body1, body2, contactW, normal, b1, b2, distance - Block.radius * 2 );
			
			// put contact into a preliminary list that will be filtered in BroadPhase
			tmpContacts.add( contact );
		}
	}

	public DoubleParameter restitution = new DoubleParameter( "restitution (bounce)", 0, 0, 1 );
	public DoubleParameter friction = new DoubleParameter("Coulomb friction coefficient", 0.6, 0, 2 );
	public IntParameter iterations = new IntParameter("iterations for PGS solve", 1000, 1, 5000);
	public IntParameter iterationsInCollection = new IntParameter("iterations for PGS solve in collection", 1, 1, 5000);
	private BooleanParameter shuffle = new BooleanParameter( "shuffle", false);
	private BooleanParameter warmStart = new BooleanParameter( "warm start", true);
	public static DoubleParameter feedbackStiffness = new DoubleParameter("feedback coefficient", 0.5, 0, 50 );
	public static DoubleParameter constraintOffset = new DoubleParameter("constraintOffset", 0.0, -0.5, 0.5 );
	public static BooleanParameter enableCompliance = new BooleanParameter("enable compliance", true );
	public static DoubleParameter compliance = new DoubleParameter("compliance", 1e-3, 1e-10, 1  );
	public static BooleanParameter  useContactGraph = new BooleanParameter("enable use of contact graph heuristic", false );
	public static IntParameter collisionWake = new IntParameter("wake n neighbors", 2, 0, 10 );
	public BooleanParameter pruneContacts = new BooleanParameter( "prune contacts", true );
	public DoubleParameter epsilonPruneConvexHull = new DoubleParameter( "epsilon for convex hull (prune contacts)", 1e-4, 1e-14, 10);

	/**
	 * @return controls for the collision processor
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		
		vfp.setBorder( new TitledBorder("Collision Processing Controls") );
		vfp.add( shuffle.getControls() );
		vfp.add( pruneContacts.getControls() );
		vfp.add( epsilonPruneConvexHull.getSliderControls(true) );
		vfp.add( warmStart.getControls() );
		vfp.add( iterations.getSliderControls() );
		vfp.add( iterationsInCollection.getSliderControls() );
		vfp.add( restitution.getSliderControls(false) );
		vfp.add( friction.getSliderControls(false) );
		vfp.add( feedbackStiffness.getSliderControls(false) );
		vfp.add( constraintOffset.getSliderControls(false) );
		vfp.add( enableCompliance.getControls() );
		vfp.add( compliance.getSliderControls(true) );
		vfp.add( useContactGraph.getControls() );
		vfp.add( collisionWake.getSliderControls() );

		return vfp.getPanel();
	}

}
