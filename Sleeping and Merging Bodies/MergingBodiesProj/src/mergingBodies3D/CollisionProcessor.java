package mergingBodies3D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import mergingBodies3D.collision.BoxBox;
import mergingBodies3D.collision.BoxPlane;
import mergingBodies3D.collision.BoxSphere;
import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;

/**
 * Class for detecting and resolving collisions.  Currently this class uses penalty forces between rigid bodies.
 * @author kry
 */
public class CollisionProcessor {

    private ArrayList<RigidBody> bodies;

    /**
     * We want warm starts to be very efficient so let's hope that the Contact hashing function is good
     * enough to avoid collisions, and likewise that the hash table for this map is large enough
     * to also avoid collisions.  This will grow dynamically, but we could likewise set the number
     * higher by default if we are running large simulations.
     */
    private HashMap<Contact,Contact> lastTimeStepContacts = new HashMap<Contact,Contact>(8192);
	
    /**
     * The current contacts that resulted in the last call to process collisions
     */
    ArrayList<Contact> contacts = new ArrayList<Contact>();

    private ContactPool contactPool = new ContactPool();
    
	/** PGS solver */
    private PGS solver = new PGS();
	
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
    public CollisionProcessor( ArrayList<RigidBody> bodies ) {
        this.bodies = bodies;
    }
    
    /** keeps track of the time used for collision detection on the last call */
    double collisionDetectTime = 0;
    
    /** keeps track of the time used to solve the LCP based velocity update on the last call */
    double collisionSolveTime = 0;
	
	/** keeps track of the time used to update the contacts inside the collections on the last call */
	double collectionUpdateTime = 0;
	
	// TODO: uncomment this when we are ready... 
	// TODO: might want a memory pool for BodyPairContact too?  Could be complicated since bodyPairContacts have some dynamic internal memory??  
	/**list that keeps track of all the body pair contacts that occurred in this time step */
	//public ArrayList<BodyPairContact> bodyPairContacts = new ArrayList<BodyPairContact>();
    
    /**
     * Processes all collisions 
     * @param dt time step
     */
    public void collisionDetection( double dt ) {
        contacts.clear();
        contactPool.swapPools();
        Contact.nextContactIndex = 0;
        
        long now = System.nanoTime();
        broadPhase();
        collisionDetectTime = ( System.nanoTime() - now ) * 1e-9;
                
        if (contacts.isEmpty()) {
        	lastTimeStepContacts.clear();        	
        }
    }
    
	/**
	 * Solve LCP
	 * @param dt time step
	 */
	public void solveLCP( double dt ) {
		
		if(!contacts.isEmpty()) {
	    	
	    	if (shuffle.getValue()) 
	    		knuthShuffle(contacts);
	    	
	    	// set up contacts solver 
			solver.init(iterations.getValue());
			solver.feedbackStiffness = feedbackStiffness.getValue();
			solver.compliance = (enableCompliance.getValue())? compliance.getValue() : 0.;
			solver.warmStart = true;
			solver.contacts = contacts;

			// TODO: eulalie: this can be optimized, only new collections need an update 
			for (Contact contact: solver.contacts)
				contact.computeJacobian(false);
			
			// solve contacts
			long now = System.nanoTime();
			
			if ( !skipSolve.getValue() ) {
				solver.solve(dt);
			}
			
			collisionSolveTime = (System.nanoTime() - now) * 1e-9;
			
			updateContactsMap();
			// computeContactsForce(dt);	// this was ONLY done for drawing ???
		}
	}
	
	protected void updateContactsMap() {
		lastTimeStepContacts.clear();
		for ( Contact contact : contacts ) {
			lastTimeStepContacts.put( contact, contact );
		}
	}
	
	/**
	 * Fills lambdas with values from previous time step
	 */
	protected void warmStart() {
//		// check for hash collisions??
//		for ( Contact c1 : contacts ) {
//			for ( Contact c2 : contacts ) {
//				if ( c2 == c1 ) continue;
//				if ( c1.hashCode() == c2.hashCode() ) {
//					System.out.println( "hash collision" );
//				}
//			}
//		}
		
		for (Contact contact : contacts) {
			Contact oldContact = lastTimeStepContacts.get( contact );
			if (oldContact != null) {
				contact.lambda0 = oldContact.lambda0;
				contact.lambda1 = oldContact.lambda1;
				contact.lambda2 = oldContact.lambda2;
				contact.prevConstraintViolation = oldContact.constraintViolation;
				contact.newThisTimeStep = false;
			} else {
				contact.newThisTimeStep = true;
			}
		}
	}
	
	// computeContatForce is only used for viz, yes?  
	// I note that internal and external forces are needed to update the b vectors for single
	// cycles LCP solves, but this shouldn't cause a problem.  Leaving this in comments to be 
	// removed in a future push, just in case this comment and code snippet are useful.
//	/**
//	 * Compute the contact force J*lambda.
//	 * @param dt
//	 */
//	public void computeContactsForce(double dt) {
//		for (Contact c: contacts)
//			c.computeForces(false, dt);	
//	}

	/**go through each element in contacts 2.
	* at each element, swap it for another random member of contacts2. 
	* at each element, get a random index from i to contacts.size.
	* swap this element with that element at that index. 
	* go to next element in contacts 2
	*/
	private void knuthShuffle(ArrayList<Contact> contacts) {
		Collections.shuffle(contacts);
		for (Contact c : contacts) {
			c.index = contacts.indexOf(c);
		}
	}
    
    /**
     * Checks for collisions between bodies.  Note that you can optionaly implement some broad
     * phase test such as spatial hashing to reduce the n squared body-body tests.
     * Currently this does the naive n squared collision check.
     */
    private void broadPhase() {
        // Naive n squared body test.. might not be that bad for even a larger number of bodies
    	// as the LCP solve for large number of contacts is really the killer!
        visitID++;
        int N = bodies.size();
        for ( int i = 0; i < N-1; i++ ) {
        	RigidBody b1 = bodies.get(i);
        	for ( int j = i+1; j < N; j++ ) {
        		RigidBody b2 = bodies.get( j );
                if ( b1.pinned && b2.pinned ) continue;
                narrowPhase( b1, b2 );                
        	}
        }    
    }

    double[] depth = new double[1];
    int [] rc = new int[1];

    /**
     * Checks for collision between boundary blocks on two rigid bodies.
     * @param body1
     * @param body2
     */
	private void narrowPhase( RigidBody body1, RigidBody body2 ) {
		if ( body1.geom instanceof RigidBodyGeomComposite ) {
			RigidBodyGeomComposite g1 = (RigidBodyGeomComposite) body1.geom;
			for ( RigidBody b : g1.bodies ) {
				// Need these collisions to be with the composite body rather than the child
				// Need up update transforms of the children?
				// This is all very much like an adaptively merged body...
				//narrowPhase( b, body2 );				
			}
		} else if ( body2.geom instanceof RigidBodyGeomComposite ) {
			RigidBodyGeomComposite g2 = (RigidBodyGeomComposite) body2.geom;
			for ( RigidBody b : g2.bodies ) {
				// same thing here... 
			}			
		} else if ( body1 instanceof PlaneRigidBody ) {
			PlaneRigidBody b1 = (PlaneRigidBody) body1;
			if ( body2 instanceof PlaneRigidBody ) {
				System.err.println("plane plane collision is impossible!");
			} else if ( body2.geom instanceof RigidBodyGeomBox ) { // box plane
				RigidBodyGeomBox g2 = (RigidBodyGeomBox) body2.geom;
				BoxPlane.dBoxPlane(body2, g2.size, b1, b1.n, b1.d, contacts, contactPool );
			} else {  // spheretree plane
				collideSphereTreeAndPlane( body2.root, body2, (PlaneRigidBody) body1 ); 
			}
		} else if ( body1.geom instanceof RigidBodyGeomBox ) {
			RigidBodyGeomBox g1 = (RigidBodyGeomBox) body1.geom;
			if ( body2 instanceof PlaneRigidBody ) { // box plane
				PlaneRigidBody b2 = (PlaneRigidBody) body2;
				BoxPlane.dBoxPlane(body1, g1.size, b2, b2.n, b2.d, contacts, contactPool );
			} else if ( body2.geom instanceof RigidBodyGeomBox ) {
				RigidBodyGeomBox g2 = (RigidBodyGeomBox) body2.geom;
				BoxBox.dBoxBox(body1,g1.size,body2, g2.size, normal, depth, rc, contacts, contactPool );
			} else { // box spheretree
				collideBoxAndSphereTree( g1, body2.root, body1, body2 );
			}
		} else { // all others have a sphere tree
			if ( body2 instanceof PlaneRigidBody ) {
				collideSphereTreeAndPlane( body1.root, body1, (PlaneRigidBody) body2 );
			} else if ( body2.geom instanceof RigidBodyGeomBox ) { // box spheretree
				collideBoxAndSphereTree( (RigidBodyGeomBox) body2.geom, body1.root, body2, body1 );
			} else {
				collideSphereTrees(body1.root, body2.root, body1, body2);
			}
		}
		// NOTE: might be easier to handle this flow control structure with a check on the first parameter 
		// and calling appropriate functions that check the second parameter, filtering the result down
		// to the appropriate pair of primitives?	
		// That said.... this is certainly faster with if/else than with function/method calls
	}
	
    /** 
     * The visitID is used to tag boundary volumes that are visited in 
     * a given time step.  Marking boundary volume nodes as visited during
     * a time step allows for a visualization of those used, but it can also
     * be used to more efficiently update the centeres of bounding volumes
     * (i.e., call a BVNode's updatecW method at most once on any given timestep)
     */
    int visitID = 0;
	
	private void collideSphereTreeAndPlane( BVNode node1, RigidBody body1, PlaneRigidBody planeBody ) {
		if ( node1.visitID != visitID ) {
			node1.visitID = visitID;
			node1.boundingSphere.updatecW();
		}
		// check bounding disc with plane
		Tuple3d c = node1.boundingSphere.cW;
		Tuple3d n = planeBody.n;
		double d = n.x*c.x + n.y*c.y + n.z*c.z + planeBody.d - node1.boundingSphere.r;
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
				
				// only 1 contact (per leaf) if there is a contact, so we set info to zero
				Contact contact = contactPool.get(); 
	            contact.set( body1, planeBody, contactW, normal, node1.boundingSphere, planeBody.dummyBV, 0, d);
	            // simple option... add to contact list...
	            contacts.add( contact );
			} else {
				for ( BVNode child : node1.children ) {
					collideSphereTreeAndPlane( child, body1,planeBody );					
				}
			}
		}		
	}
	
	private void collideBoxAndSphereTree( RigidBodyGeomBox geom1, BVNode node2, RigidBody body1, RigidBody body2 ) {
		if ( node2.visitID != visitID ) {
			node2.visitID = visitID;
			node2.boundingSphere.updatecW();
		}
		if ( !BoxSphere.dBoxSphereTest( body1, geom1.size, node2.boundingSphere.cW, node2.boundingSphere.r ) ) return;
		if ( !node2.isLeaf() ) {
			for ( BVNode child : node2.children ) {
				collideBoxAndSphereTree( geom1, child, body1, body2 );
			}
		} else { // this is a leaf! we are colliding for real!
			BoxSphere.dBoxSphere(body1, geom1.size, body2, node2.boundingSphere.cW, node2.boundingSphere.r, contacts, contactPool );
		}
	}
	
	/** 
	 * Recurses through all of body_1, then body_2
	 * @param node1
	 * @param node2
	 * @param body1
	 * @param body2
	 */
	private void collideSphereTrees(BVNode node1, BVNode node2, RigidBody body1, RigidBody body2) {
		if (node1.visitID != visitID) {
			node1.visitID = visitID;
			node1.boundingSphere.updatecW();
		}	
		if (node2.visitID != visitID) {
			node2.visitID = visitID;
			node2.boundingSphere.updatecW();
		}

		if (node1.boundingSphere.intersects(node2.boundingSphere)) {
			if (node1.isLeaf() && node2.isLeaf()) {
				BVSphere leafBV1 = node1.boundingSphere;
				BVSphere leafBV2 = node2.boundingSphere;
				processCollision(body1, leafBV1, body2, leafBV2);
			} else if ( node1.isLeaf() ) {
				for ( BVNode child : node2.children ) {
					collideSphereTrees(node1, child, body1, body2);					
				}
			} else if ( node2.isLeaf() ) {
				for ( BVNode child : node1.children ) {
					collideSphereTrees(child, node2, body1, body2);					
				}
			} else if ( node1.boundingSphere.r <= node2.boundingSphere.r ) {
				// if we have the choice, descend subtree with larger sphere
				for ( BVNode child : node2.children ) {
					collideSphereTrees(node1, child, body1, body2);					
				}
			} else {
				for ( BVNode child : node1.children ) {
					collideSphereTrees(child, node2, body1, body2);					
				}
			}
		}
	}
    
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
    private Point3d contactW = new Point3d();
    private Vector3d normal = new Vector3d();
        
    /**
     * Processes a collision between two bodies for two given blocks that are colliding.
     * Currently this implements a penalty force
     * @param body1
     * @param bv1
     * @param body2
     * @param bv2
     */
    private void processCollision( RigidBody body1, BVSphere bv1, RigidBody body2, BVSphere bv2 ) {        
        double distance = bv1.cW.distance( bv2.cW );
        double distanceBetweenCenters = bv2.r + bv1.r;
        if ( distance < distanceBetweenCenters ) {
            // contact point at halfway between points 
            // NOTE: this assumes that the two blocks have the same radius!
            contactW.interpolate( bv1.cW, bv2.cW, .5 );
            // contact normal
            normal.sub( bv2.cW, bv1.cW );
            normal.normalize();
            // create the contact, and again info is zero as we only generate one contact per pair of leaves
            Contact contact = contactPool.get(); 
            contact.set( body1, body2, contactW, normal, bv1, bv2, 0, distance - distanceBetweenCenters);
            // simple option... add to contact list...
            contacts.add( contact );
        }
    }

	public BooleanParameter shuffle = new BooleanParameter( "shuffle", false);
	public DoubleParameter feedbackStiffness = new DoubleParameter("feedback coefficient", 0.5, 0, 50 );
	public BooleanParameter enableCompliance = new BooleanParameter("enable compliance", true );
	public DoubleParameter compliance = new DoubleParameter("compliance", 1e-3, 1e-10, 1  );
	    
    /** Restitution parameter for contact constraints */
    public DoubleParameter restitution = new DoubleParameter( "restitution (bounce)", 0, 0, 1 );
    
    /** Coulomb friction coefficient for contact constraint */
    public DoubleParameter friction = new DoubleParameter("Coulomb friction", 0.8, 0, 2 );
    
    /** Number of iterations to use in projected Gauss Seidel solve */
    public IntParameter iterations = new IntParameter("iterations for PGS solve", 20, 1, 500);
        
    private BooleanParameter skipSolve = new BooleanParameter(" skip LCP solve", false );
    
    /**
     * @return controls for the collision processor
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Collision Processing Controls") );
        
		vfp.add( shuffle.getControls() );
		
        vfp.add( skipSolve.getControls() );
        vfp.add( iterations.getSliderControls() );
        vfp.add( restitution.getSliderControls(false) );
        vfp.add( friction.getSliderControls(false) );
		vfp.add( feedbackStiffness.getSliderControls(false) );
		vfp.add( enableCompliance.getControls() );
		vfp.add( compliance.getSliderControls(true) );
        
        return vfp.getPanel();
    }
    
}
