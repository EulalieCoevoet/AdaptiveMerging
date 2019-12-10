package mergingBodies;

import java.util.ArrayList;
import java.util.HashMap;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mergingBodies.Contact.ContactState;
import mergingBodies.RigidBodySystem.MergeParameters;
import mergingBodies.RigidBodySystem.SleepParameters;
import mergingBodies.MotionMetricProcessor;
import mintools.viewer.EasyViewer;
import no.uib.cipr.matrix.DenseVector;

import javax.vecmath.Color3f;
import javax.vecmath.Color4f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Simple 2D rigid body based on image samples
 * 
 * @author kry
 */
public class RigidBody {

	/**Pointer to the parent of this body if it is merged*/
	RigidCollection parent = null;
	/** Unique identifier for this body */
	public int index = 0;
	/** Variable to keep track of identifiers that can be given to rigid bodies */
	static public int nextIndex = 0;

	/** visitID of this contact at this time step. */
	boolean visited = false;

	/** Block approximation of geometry */
	ArrayList<Block> blocks;
	/** Boundary blocks */
	ArrayList<Block> boundaryBlocks;

	BVNode root;

	/** accumulator for forces acting on this body in world coordinate*/
	Vector2d force = new Vector2d();
	/** accumulator for torques acting on this body */
	double torque;

	double massAngular;
	double massLinear;

	public boolean pinned = false;
	/** option used to pinned object for a fixed amount of steps */
	public boolean temporarilyPinned = false;
	int steps = 0;

	/**
	 * Transforms points in Body coordinates to World coordinates
	 */
	RigidTransform transformB2W = new RigidTransform();

	/**
	 * Transforms points in World coordinates to Body coordinates
	 */
	RigidTransform transformW2B = new RigidTransform();

	/**
	 * Transforms points in body coordinates to collection coordinates, if a
	 * collection exists
	 */
	RigidTransform transformB2C = new RigidTransform();

	/**
	 * Transforms points in collection coordinates to body coordinates, if a
	 * collection exists
	 */
	RigidTransform transformC2B = new RigidTransform();
	
	/** Linear velocity */
	public Vector2d v = new Vector2d(0.,0.); 
	 /** Angular velocity in radians per second */
	public double omega = 0.;
	/** Position of center of mass in the world frame */
	public Point2d x = new Point2d(); 
	/** Initial position of center of mass in the world frame */
	public Point2d x0 = new Point2d(); 
	/** Orientation angle in radians */
	public double theta = 0.; 
	/** bounding box, in the body frame */
	public ArrayList<Point2d> boundingBoxB = new ArrayList<Point2d>(); 

	/** inverse of the linear mass, or zero if pinned */
	double minv;

	/** inverse of the angular mass, or zero if pinned */
	double jinv;

	public boolean isSleeping = false;
	
	/**
	 * rho value, 0 if active, 1 if inactive, function of kinetic energy when in
	 * transition
	 **/
	public double rho;
	
	/**
	 * true if body is a magnetic body
	 */
	public boolean magneticBody = false;
	
	/**
	 * true if magnetic field is activated
	 */
	public boolean activateMagnet = false;

	/**
	 * list of contacting bodies present with this RigidBody. In case of a collection, the list will contain
	 * both internal and external bpc.
	 **/
	public ArrayList<BodyPairContact> bodyPairContactList = new ArrayList<BodyPairContact>();

	/** list of springs attached to the body **/
	public ArrayList<Spring> springs = new ArrayList<Spring>();

	/**
	 * keeps track of the activity of the last N steps, if it is false, means that
	 * should be asleep, if true, should be awake
	 */
	public ArrayList<Double> metricHistory = new ArrayList<Double>();

	DenseVector deltaV = new DenseVector(3);
	
	MotionMetricProcessor motionMetricProcessor = new MotionMetricProcessor();
	
	public RigidBody() {
	}
	
	/**
	 * Creates a new rigid body from a collection of blocks
	 * 
	 * @param blocks
	 * @param boundaryBlocks
	 */
	public RigidBody(ArrayList<Block> blocks, ArrayList<Block> boundaryBlocks) {

		this.blocks = blocks;
		this.boundaryBlocks = boundaryBlocks;
		
		Point2d bbmaxB = new Point2d(-Double.MAX_VALUE, -Double.MAX_VALUE);
		Point2d bbminB = new Point2d(Double.MAX_VALUE, Double.MAX_VALUE);
		// compute the mass and center of mass position
		for (Block b : blocks) {
			double mass = b.getColourMass();
			massLinear += mass;
			x0.x += b.j * mass;
			bbmaxB.x = Math.max(bbmaxB.x, b.j + b.h);
			bbminB.x = Math.min(bbminB.x, b.j - b.h);
			x0.y += b.i * mass;
			bbmaxB.y = Math.max(bbmaxB.y, b.i + b.h);
			bbminB.y = Math.min(bbminB.y, b.i - b.h); 
		}
		x0.scale(1 / massLinear);
		// set block positions in world and body coordinates
		for (Block b : blocks) {
			b.pB.x = b.j - x0.x;
			b.pB.y = b.i - x0.y;			
		}
		// compute the rotational inertia
		final Point2d zero = new Point2d(0, 0);
		for (Block b : blocks) {
			double mass = b.getColourMass();
			massAngular += mass * b.pB.distanceSquared(zero);
		}
		// prevent zero angular inertia in the case of a single block
		if (blocks.size() == 1) {
			Block b = blocks.get(0);
			double mass = b.getColourMass();
			massAngular = mass * (1 + 1) / 12;
		}
		x.set(x0);
		transformB2W.set(theta, x);
		transformW2B.set(theta, x);
		transformW2B.invert();
		
		transformW2B.transform(bbmaxB);
		transformW2B.transform(bbminB);
		boundingBoxB.add(bbmaxB);
		boundingBoxB.add(new Point2d(bbmaxB.x,bbminB.y));
		boundingBoxB.add(bbminB);
		boundingBoxB.add(new Point2d(bbminB.x,bbmaxB.y));

		root = new BVNode(boundaryBlocks, this);

		pinned = isAllBlueBlocks();
		temporarilyPinned = hasGreenBlocks();

		if (pinned) {
			minv = 0;
			jinv = 0;
		} else {
			minv = 1 / massLinear;
			jinv = 1 / massAngular;
		}

		// set our index
		index = nextIndex++;
	}

	/**
	 * Creates a copy of the provided rigid body
	 * 
	 * @param body
	 */
	public RigidBody(RigidBody body) {
		blocks = new ArrayList<Block>(body.blocks);
		boundaryBlocks = new ArrayList<Block>(body.boundaryBlocks);
		massLinear = body.massLinear;
		massAngular = body.massAngular;
		x0.set(body.x0);
		x.set(body.x);
		boundingBoxB = new ArrayList<Point2d>(body.boundingBoxB);
		v.set(body.v);
		theta = body.theta;
		omega = body.omega;
		// we can share the blocks and boundary blocks...
		// no need to update them as they are in the correct body coordinates already
		updateTransformations();
		// We do need our own bounding volumes! can't share!
		root = new BVNode(boundaryBlocks, this);
		pinned = body.pinned;
		temporarilyPinned = body.temporarilyPinned;
		steps = body.steps;
		minv = body.minv;
		jinv = body.jinv;
		isSleeping = false;
		rho = 0;
		
		// set our index
		index = nextIndex++;
	}
	
	/**
	 * Clear deltaV, force and torque
	 */
	public void clear() {
		force.set(0, 0);
		torque = 0;
		deltaV.zero();
	}
	
	public boolean isInCollection() {
		return (parent!=null);
	}
	
	public boolean isInSameCollection(RigidBody body) {
		return (parent!=null && parent==body.parent);
	}
	
	public boolean isInCollection(RigidCollection collection) {
		return (parent==collection);
	}
	
	/**
	 * Track metric over time steps
	 */
	public void accumulate(SleepParameters sleepParams) {
		
		RigidBody dummyBody = new RigidBody();
		dummyBody.x.set(x);
		dummyBody.theta = theta;
		dummyBody.v.set(0.,0.);
		dummyBody.omega = 0;
		
		motionMetricProcessor.setMotionMetricType(0);
		metricHistory.add(motionMetricProcessor.getMotionMetric(this, dummyBody));
		if (metricHistory.size() > sleepParams.stepAccum.getValue()) {
			metricHistory.remove(0);	
		}
	}
	
	public void wake() {
		if (isSleeping) {
	    	isSleeping = false;
	    	metricHistory.clear();
	    }
		
		if (isInCollection() && parent.isSleeping)
	    	parent.wake();
	}

	/**
	 * Updates the B2W and W2B transformations
	 */
	public void updateTransformations() {
		transformB2W.set(theta, x);
		transformW2B.set(theta, x);
		transformW2B.invert();
	}

	/**
	 * Apply a contact force specified in world coordinates
	 * 
	 * @param contactPointW
	 * @param contactForceW
	 */
	public void applyForceW(Point2d contactPointW, Vector2d contactForceW) {
		force.add(contactForceW);
		// TODO: Objective 1: Compute the torque applied to the body

		// holds r vector.
		Vector2d r = new Vector2d(contactPointW);
		Point2d tempX = new Point2d(x);
		r.sub(tempX);
		Vector2d ortho_r = new Vector2d(-r.y, r.x);

		torque += ortho_r.dot(contactForceW);
	}

	/**
	 * Advances the body state using symplectic Euler, first integrating accumulated
	 * force and torque (which are then set to zero), and then updating position and
	 * angle. The internal rigid transforms are also updated.
	 * 
	 * @param dt step size
	 */
	public void advanceTime(double dt) {
		
		if(temporarilyPinned && ++steps>=RigidBodySystem.tempSleepCount.getValue())
			temporarilyPinned = !temporarilyPinned; 

		if (!pinned && !temporarilyPinned && !isSleeping) {

			advanceVelocities(dt);
			advancePositions(dt);
		}
	}
	
	public void advanceVelocities(double dt) {
		v.x += force.x * dt / massLinear + deltaV.get(0);
		v.y += force.y * dt / massLinear + deltaV.get(1);
		omega += torque * dt / massAngular + deltaV.get(2);
	}
	
	public void advancePositions(double dt) {
		x.x += v.x * dt;
		x.y += v.y * dt;
		theta += omega * dt;
		
		updateTransformations();
	}	

	/**
	 * Computes the velocity of the provided point provided in world coordinates due
	 * to motion of this body.
	 * 
	 * @param contactPointW
	 * @param result        the velocity
	 */
	public void getSpatialVelocity(Point2d contactPointW, Vector2d result) {
		result.sub(contactPointW, x);
		result.scale(omega);
		double xpart = -result.y;
		double ypart = result.x;
		result.set(xpart, ypart);
		result.add(v);
	}
	
	/**
	 * Check if the body is part of a cycle formed by three bodies with one contact between each.
	 * @param count
	 * @param startBody
	 * @param bpcFrom
	 * @return true or false
	 */
	protected boolean checkCycle(int count, RigidBody startBody, BodyPairContact bpcFrom, MergeParameters mergeParams) {
		
		if (count>2) { // more than three bodies in the cycle
			bpcFrom.clearCycle();
			return false;
		}
		
		// if we come across a collection in the contact graph, we consider it as a body and check the external bpc 
		ArrayList<BodyPairContact> bpcList = (this.isInCollection())? parent.bodyPairContactList : bodyPairContactList;
		
		RigidBody otherBodyFrom = bpcFrom.getOtherBodyWithCollectionPerspective(this);		
		
		BodyPairContact bpcToCheck = null;
		for (BodyPairContact bpc: bpcList) {
			
			if(bpc != bpcFrom && !bpc.inCollection && !bpc.inCycle) { // we do not want to check the bpc we come from, nor the bpc inside the collection
																	  // a bpc should only be part of one cycle
				
				// we only consider bodies that are ready to be merged
				if(bpc.checkMergeCondition(mergeParams, false)) {
			
					RigidBody otherBody = bpc.getOtherBodyWithCollectionPerspective(this);
	
					int nbActiveContact = 0;
					for (Contact contact : bpc.contactList)
						if (contact.state != ContactState.BROKEN)
							nbActiveContact += 1;
					
					if (nbActiveContact==1) // if there is only one active contact in the bpc, it is a direction we want to check for cycle
						bpcToCheck = bpc;
					
					if(otherBody == otherBodyFrom || // we are touching two different bodies in a same collection
					   otherBody.isInSameCollection(startBody) || // we are part of a collection that touches a same body 
					   otherBody == startBody || // we have reached the body from which the cycle started
					  (otherBody.pinned && startBody.pinned)) { // there is a body between two pinned body	
						bpcFrom.updateCycle(bpc);
						return true;
					}
				}
			}
		}

		// we did not find a cycle, but there is another candidate, continue
		if(bpcToCheck!=null) {
			bpcFrom.updateCycle(bpcToCheck);
			RigidBody otherBody = bpcToCheck.getOtherBodyWithCollectionPerspective(this);
			return otherBody.checkCycle(++count, startBody, bpcToCheck, mergeParams); 
		}
		
		// we did not find a cycle, and there is no another candidate, break
		bpcFrom.clearCycle();
		return false;
	}

	/**
	 * Checks if all blocks are shades of blue
	 * 
	 * @return true if all blue
	 */
	boolean isAllBlueBlocks() {
		for (Block b : blocks) {
			if (!(b.c.x == b.c.y && b.c.x < b.c.z))
				return false;
		}
		return true;
	}
	
	/**
	 * Checks if some blocks are shades of green
	 * 
	 * @return true if all green
	 */
	boolean hasGreenBlocks() {
		for (Block b : blocks) {
			if ((b.c.x == b.c.z && b.c.x < b.c.y))
				return true;
		}
		return false;
	}
	
	/**
	 * Checks to see if the point intersects the body in its current position
	 * 
	 * @param pW
	 * @return true if intersection
	 */
	public boolean intersect(Point2d pW) {

		if (root.boundingDisc.isInDisc(pW)) {
			Point2d pB = new Point2d();
			transformW2B.transform(pW, pB);

			for (Block b : blocks) {
				if (b.pB.distanceSquared(pB) < Block.radius * Block.radius)
					return true;
			}
		}
		return false;
	}

	public double getMetric() {
		return 0.5 * v.lengthSquared() + 0.5 * omega * omega;
	}

	/**
	 * Resets this rigid body to its initial position and zero velocity, recomputes
	 * transforms
	 */
	public void reset() {
		x.set(x0);
		theta = 0;
		v.set(0, 0);
		omega = 0;
		force.set(0, 0);
		torque = 0;
		transformB2W.set(theta, x);
		transformW2B.set(transformB2W);
		transformW2B.invert();

		transformB2C.T.setIdentity();
		transformC2B.T.setIdentity();
		bodyPairContactList.clear();
		isSleeping = false;
		metricHistory.clear();
		parent = null;
	}

	/**
	 * Map to keep track of display list IDs for drawing our rigid bodies
	 * efficiently
	 */
	static private HashMap<ArrayList<Block>, Integer> mapBlocksToDisplayList = new HashMap<ArrayList<Block>, Integer>();

	/** display list ID for this rigid body */
	int myListID = -1;

	public boolean created = false;

	/**
	 * Deletes all display lists. This is called when clearing all rigid bodies from
	 * the simulation, or when display lists need to be updated due to changing
	 * transparency of the blocks.
	 * 
	 * @param gl
	 */
	static public void clearDisplayLists(GL2 gl) {
		for (int id : mapBlocksToDisplayList.values()) {
			gl.glDeleteLists(id, 1);
		}
		mapBlocksToDisplayList.clear();
	}

	/**
	 * Draws the blocks of a rigid body
	 * 
	 * @param drawable
	 */
	public void display(GLAutoDrawable drawable) {
		display(drawable, null);
	}

	public void display(GLAutoDrawable drawable, Color3f color) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glPushMatrix();
		gl.glTranslated(x.x, x.y, 0);
		gl.glRotated(theta * 180 / Math.PI, 0, 0, 1);
		
		if (myListID == -1) {
			Integer ID = mapBlocksToDisplayList.get(blocks);
			if (ID == null) {
				myListID = gl.glGenLists(1);
				gl.glNewList(myListID, GL2.GL_COMPILE_AND_EXECUTE);
				for (Block b : blocks) {
					b.display(drawable, color);
				}
				gl.glEndList();
				mapBlocksToDisplayList.put(blocks, myListID);
			} else {
				myListID = ID;
				gl.glCallList(myListID);
			}
		} else {
			gl.glCallList(myListID);
		}
		
		gl.glPopMatrix();
	}
	
	/**
	 * Displays deltaV computed by LCP solve at each frame. Goes from each body's center of mass
	 * outwards in the direction of deltaV
	 */
	public void displayDeltaV(GLAutoDrawable drawable, int size, Color4f color) {
		GL2 gl = drawable.getGL().getGL2();

		gl.glLineWidth(size);
		
		gl.glColor4f(color.x, color.y, color.z, color.w);
		gl.glBegin( GL.GL_LINES );
		double scale = RigidBodySystem.deltaVVizScale.getValue();
		
		gl.glVertex2d(x.x, x.y);
		gl.glVertex2d(x.x + scale*deltaV.get(0), x.y+scale*deltaV.get(1));

		gl.glEnd();
	}

	/**
	 * Draws the center of mass position with a circle. The circle will be drawn
	 * differently if the block is at rest (i.e., close to zero kinetic energy)
	 * 
	 * @param drawable
	 */
	public void displayCOMs(GLAutoDrawable drawable) {
		
		GL2 gl = drawable.getGL().getGL2();
		if (!isSleeping) {
			gl.glPointSize(8);
			gl.glColor3f(0, 0, 0.7f);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex2d(x.x, x.y);			
			gl.glEnd();
			gl.glPointSize(4);
			gl.glColor3f(1, 1, 1);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex2d(x.x, x.y);	
			gl.glEnd();
		} else {
			gl.glPointSize(8);
			gl.glColor3f(0, 0, 0.7f);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex2d(x.x, x.y);
			gl.glEnd();
			gl.glPointSize(4);
			gl.glColor3f(0, 0, 1);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex2d(x.x, x.y);
			gl.glEnd();
		}
	}
	
	/**
	 * Draws bounding box
	 * @param drawable
	 */
	public void displayBB(GLAutoDrawable drawable) {
		
		GL2 gl = drawable.getGL().getGL2();
		
		gl.glLineWidth(1);
		gl.glColor3f(1, 0, 0);
		
		gl.glBegin(GL.GL_LINE_LOOP);
		for (Point2d point : boundingBoxB) {
			Point2d p = new Point2d(point);
			transformB2W.transform(p);
			gl.glVertex2d(p.x, p.y);
		}
					
		gl.glEnd();
	}

	public void displaySpeedCOM(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		double k = this.v.length() + this.omega;

		if (k > 255) {
			k = 255;
		}
		k /= 10;
		gl.glPointSize(8);
		gl.glColor3f((float) k, 0, 0);
		gl.glBegin(GL.GL_POINTS);
		gl.glVertex2d(x.x, x.y);
		gl.glEnd();
		gl.glPointSize(4);
		gl.glColor3f((float) k, 0, 0);
		gl.glBegin(GL.GL_POINTS);
		gl.glVertex2d(x.x, x.y);
		gl.glEnd();
	}

	public void displayIndex(GLAutoDrawable drawable, int font) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glColor3f(1, 0, 0);
		gl.glRasterPos2d(this.x.x, this.x.y);

		EasyViewer.glut.glutBitmapString(font, Integer.toString(this.index));
	}
}