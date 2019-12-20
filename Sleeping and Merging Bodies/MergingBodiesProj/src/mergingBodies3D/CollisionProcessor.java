package mergingBodies3D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import mergingBodies.BodyPairContact;
import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

/**
 * Class for detecting and resolving collisions.  Currently this class uses penalty forces between rigid bodies.
 * @author kry
 */
public class CollisionProcessor {

    private List<RigidBody> bodies;

	public HashMap<String, Contact> lastTimeStepMap = new HashMap<String, Contact>();
	
    /**
     * The current contacts that resulted in the last call to process collisions
     */
    public ArrayList<Contact> contacts = new ArrayList<Contact>();

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
     * Processes all collisions 
     * @param dt time step
     */
    public void collisionDetection( double dt ) {
        contacts.clear();
        Contact.nextContactIndex = 0;
        
        long now = System.nanoTime();
        broadPhase();
        collisionDetectTime = ( System.nanoTime() - now ) * 1e-9;
                
        if (contacts.isEmpty())
			lastTimeStepMap.clear();
        
        if ( contacts.size() > 0  && doLCP.getValue() ) {
            now = System.nanoTime();

            double bounce = restitution.getValue();
            double mu = friction.getValue();
            // TODO: Compute velocity update with iterative solve of contact constraint matrix.

            collisionSolveTime = (System.nanoTime() - now) * 1e-9;
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
	
	protected void updateContactsMap() {
		lastTimeStepMap.clear();
//		for (Contact contact : contacts) {
//			Block block1 = contact.block1;
//			Block block2 = contact.block2;
//			lastTimeStepMap.put("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode()), contact);
//		} 
	}
	
	/**
	 * Compute the contact force J*lambda.
	 * @param dt
	 */
	public void computeContactsForce(double dt) {
		for (Contact c: contacts)
			c.computeForces(false, dt);	
	}

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
        // Naive n squared body test.. might not be that bad for small number of bodies 
        visitID++;
        for ( RigidBody b1 : bodies ) {
            for ( RigidBody b2 : bodies ) { // not so inefficient given the continue on the next line
                if ( bodies.indexOf(b1) >= bodies.indexOf(b2) ) continue;
                if ( b1.pinned && b2.pinned ) continue;                
                narrowPhase( b1, b2 );                
            }
        }        
    }
    
    /**
     * Checks for collision between boundary blocks on two rigid bodies.
     * TODO: This needs to be improved as the n-squared block test is too slow!
     * @param body1
     * @param body2
     */
	private void narrowPhase( RigidBody body1, RigidBody body2 ) {
		if ( body1 instanceof PlaneRigidBody ) {
			findCollisionsWithPlane( body2.root, body2, (PlaneRigidBody) body1 ); 
		} else if (body2 instanceof PlaneRigidBody ) {
			findCollisionsWithPlane( body1.root, body1, (PlaneRigidBody) body2 );
		} else {
			if ( ! useBVTree.getValue() ) {
	            for ( Block b1 : body1.blocks ) {
	                for ( Block b2 : body2.blocks ) {
	                    processCollision( body1, b1, body2, b2 );
	                }
	            }
	        }
		}
	}
	
    /** 
     * The visitID is used to tag boundary volumes that are visited in 
     * a given time step.  Marking boundary volume nodes as visited during
     * a time step allows for a visualization of those used, but it can also
     * be used to more efficiently update the centeres of bounding volumes
     * (i.e., call a BVNode's updatecW method at most once on any given timestep)
     */
    int visitID = 0;
	
	private void findCollisionsWithPlane( BVNode node1, RigidBody body1, PlaneRigidBody planeBody ) {
		if(node1.visitID != visitID) {
			node1.visitID = visitID;
			node1.boundingDisc.updatecW();
		}
		// check bounding disc with plane
		Tuple3d c = node1.boundingDisc.cW;
		Tuple3d n = planeBody.n;
		double d = n.x*c.x + n.y*c.y + n.z*c.z + planeBody.d - node1.boundingDisc.r;
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
			} else {
				findCollisionsWithPlane( node1.child1, body1,planeBody );
				findCollisionsWithPlane( node1.child2, body1,planeBody );
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
    private Point3d tmp1 = new Point3d();
    private Point3d tmp2 = new Point3d();
    private Point3d contactW = new Point3d();
    private Vector3d force = new Vector3d();
    private Vector3d contactV1 = new Vector3d();
    private Vector3d contactV2 = new Vector3d();
    private Vector3d relativeVelocity = new Vector3d();
    private Vector3d normal = new Vector3d();
        
    /**
     * Processes a collision between two bodies for two given blocks that are colliding.
     * Currently this implements a penalty force
     * @param body1
     * @param b1
     * @param body2
     * @param b2
     */
    private void processCollision( RigidBody body1, Block b1, RigidBody body2, Block b2 ) {        
        double k = contactSpringStiffness.getValue();
        double c1 = contactSpringDamping.getValue();
        double threshold = separationVelocityThreshold.getValue();
        boolean useSpring = enableContactSpring.getValue();
        boolean useDamping = enableContactDamping.getValue();
        
        body1.transformB2W.transform( b1.pB, tmp1 );
        body2.transformB2W.transform( b2.pB, tmp2 );
        double distance = tmp1.distance(tmp2);
        if ( distance < Block.radius * 2 ) {
            // contact point at halfway between points 
            // NOTE: this assumes that the two blocks have the same radius!
            contactW.interpolate( tmp1, tmp2, .5 );
            // contact normal
            normal.sub( tmp2, tmp1 );
            normal.normalize();
            // create the contact
            Contact contact = new Contact( body1, body2, contactW, normal, 0.);
            // simple option... add to contact list...
            contacts.add( contact );
            if ( ! doLCP.getValue() ) {
                // compute relative body velocity at contact point
                body1.getSpatialVelocity( contactW, contactV1 );
                body2.getSpatialVelocity( contactW, contactV2 );
                relativeVelocity.sub( contactV1, contactV2 );
                if ( -relativeVelocity.dot( normal ) < threshold ) {
                    if ( useSpring ) {
                        // spring force
                        double interpenetration = distance - Block.radius * 2; // a negative quantity
                        force.scale( -interpenetration * k, normal );
                        body2.applyContactForceW(contactW, force);
                        force.scale(-1);
                        body1.applyContactForceW(contactW, force);
                    }
                    if ( useDamping ) {
                        // spring damping forces!
                        // vertical
                        force.scale( relativeVelocity.dot(normal) * c1, normal );                    
                        body2.applyContactForceW( contactW, force );
                        force.scale(-1);
                        body1.applyContactForceW( contactW, force );
                    }
                }
            }
        }
    }

	public BooleanParameter shuffle = new BooleanParameter( "shuffle", false);
	public DoubleParameter feedbackStiffness = new DoubleParameter("feedback coefficient", 0.5, 0, 50 );
	public BooleanParameter enableCompliance = new BooleanParameter("enable compliance", true );
	public DoubleParameter compliance = new DoubleParameter("compliance", 1e-3, 1e-10, 1  );
	
    /** Stiffness of the contact penalty spring */
    private DoubleParameter contactSpringStiffness = new DoubleParameter("penalty contact stiffness", 1e3, 1, 1e5 );
    
    /** Viscous damping coefficient for the contact penalty spring */
    private DoubleParameter contactSpringDamping = new DoubleParameter("penalty contact damping", 10, 1, 1e4 );
    
    /** Threshold for the relative velocity in the normal direction, for determining if spring force will be applied. */
    private DoubleParameter separationVelocityThreshold = new DoubleParameter( "penalty separation velocity threshold (controls bounce)", 1e-9, 1e-9, 1e3 );
    
    /** Enables the contact penalty spring */
    private BooleanParameter enableContactSpring = new BooleanParameter("enable penalty contact spring", true );
    
    /** Enables damping of the contact penalty spring */
    private BooleanParameter enableContactDamping = new BooleanParameter("enable penalty contact damping", true );
    
    /** Restitution parameter for contact constraints */
    public DoubleParameter restitution = new DoubleParameter( "restitution (bounce)", 0, 0, 1 );
    
    /** Coulomb friction coefficient for contact constraint */
    public DoubleParameter friction = new DoubleParameter("Coulomb friction", 0.33, 0, 2 );
    
    /** Number of iterations to use in projected Gauss Seidel solve */
    public IntParameter iterations = new IntParameter("iterations for GS solve", 10, 1, 500);
    
    /** Flag for switching between penalty based contact and contact constraints */
    private BooleanParameter doLCP = new BooleanParameter( "do LCP solve", false );
    
    /** Flag for enabling the use of hierarchical collision detection for body pairs */
    private BooleanParameter useBVTree = new BooleanParameter( "use BVTree", false );
    
    
    /**
     * @return controls for the collision processor
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Collision Processing Controls") );
        
		vfp.add( shuffle.getControls() );
		
        vfp.add( useBVTree.getControls() );
        vfp.add( doLCP.getControls() );
        vfp.add( iterations.getSliderControls() );
        vfp.add( restitution.getSliderControls(false) );
        vfp.add( friction.getSliderControls(false) );
		vfp.add( feedbackStiffness.getSliderControls(false) );
		vfp.add( enableCompliance.getControls() );
		vfp.add( compliance.getSliderControls(true) );
        
        VerticalFlowPanel vfp2 = new VerticalFlowPanel();
        vfp2.setBorder( new TitledBorder("penalty method controls") );
        vfp2.add( contactSpringStiffness.getSliderControls(true) );
        vfp2.add( contactSpringDamping.getSliderControls(true) );
        vfp2.add( separationVelocityThreshold.getSliderControls( true ) );
        vfp2.add( enableContactDamping.getControls() );
        vfp2.add( enableContactSpring.getControls() );
        
        CollapsiblePanel cp = new CollapsiblePanel(vfp2.getPanel());
        cp.collapse();
        vfp.add( cp );        
        return vfp.getPanel();
    }
    
}
