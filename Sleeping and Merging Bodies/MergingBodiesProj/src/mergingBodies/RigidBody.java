package mergingBodies;

import java.util.ArrayList;
import java.util.HashMap;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.EasyViewer;
import no.uib.cipr.matrix.DenseVector;

import javax.vecmath.Color3f;
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
	public int index;

	/** Variable to keep track of identifiers that can be given to rigid bodies */
	static public int nextIndex = 0;

	public boolean wokenUp = false;

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

	/** linear velocity */
	public Vector2d v = new Vector2d(0.,0.);

	/** Position of center of mass in the world frame */
	public Point2d x = new Point2d();

	/** initial position of center of mass in the world frame */
	Point2d x0 = new Point2d();

	/** orientation angle in radians */
	public double theta = 0.;

	/** angular velocity in radians per second */
	public double omega = 0.;

	/** inverse of the linear mass, or zero if pinned */
	double minv;

	/** inverse of the angular mass, or zero if pinned */
	double jinv;

	/** linear momentum of this body **/
	public Vector2d p_lin = new Vector2d();

	/** angular momentum of this body **/
	public double p_ang;

	public ObjectState state = ObjectState.ACTIVE;

	public enum ObjectState { ACTIVE, SLEEPING };
	
	/**
	 * rho value, 0 if active, 1 if inactive, function of kinetic energy when in
	 * transition
	 **/
	public double rho;

	/**
	 * list of contacting bodies present with this RigidBody. cleared after every
	 * time step, unless the contact was between two sleeping bodies
	 **/
	public ArrayList<BodyPairContact> bodyPairContactList = new ArrayList<BodyPairContact>();

	/** list of springs attached to the body **/
	public ArrayList<Spring> springs = new ArrayList<Spring>();

	/**
	 * keeps track of the activity of the last N steps, if it is false, means that
	 * should be asleep, if true, should be awake
	 */
	public ArrayList<Double> velHistory = new ArrayList<Double>();
	
	public double metric = Double.MAX_VALUE;

	public boolean merged = false;

	DenseVector deltaV = new DenseVector(3);

	public Vector2d forcePreSleep = new Vector2d();
	public double torquePreSleep = 0;
	
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
		// compute the mass and center of mass position
		for (Block b : blocks) {
			double mass = b.getColourMass();
			massLinear += mass;
			x0.x += b.j * mass;
			x0.y += b.i * mass;
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
		v.set(body.v);
		theta = body.theta;
		omega = body.omega;
		// we can share the blocks and boundary blocks...
		// no need to update them as they are in the correct body coordinates already
		updateTransformations();
		// We do need our own bounding volumes! can't share!
		root = new BVNode(boundaryBlocks, body);
		pinned = body.pinned;
		temporarilyPinned = body.temporarilyPinned;
		steps = body.steps;
		minv = body.minv;
		jinv = body.jinv;
		state = ObjectState.ACTIVE;
		rho = 0;
		// set our index

		index = nextIndex++;
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
		
		// check for temporary pinned condition
		if(temporarilyPinned && ++steps>=200)
			temporarilyPinned=!temporarilyPinned; 

		// update particles activity or sleepiness.
		// fully active, regular stepping
		setActivityContactGraph(CollisionProcessor.sleepingThreshold.getValue());

		if (!pinned && !temporarilyPinned) {

			// non ARPS
			v.x += force.x * dt / massLinear + deltaV.get(0);
			v.y += force.y * dt / massLinear + deltaV.get(1);
			omega += torque * dt / massAngular + deltaV.get(2);
			
			if (state == ObjectState.ACTIVE) {
				x.x += v.x * dt;
				x.y += v.y * dt;
				theta += omega * dt;
			} else {
				v.set(0, 0);
				omega = 0;
			}

			updateTransformations();
		}
	}
	
	/**
	 * Computes the total kinetic energy of the body.
	 * 
	 * @return the total kinetic energy
	 */
	public double getKineticEnergy() {
		return 0.5 * massLinear * v.lengthSquared() + 0.5 * massAngular * omega * omega;
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

	public void setActivityContactGraph(double sleeping_threshold) {

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
		p_lin.set(0, 0);
		p_ang = 0;
		transformB2W.set(theta, x);
		transformW2B.set(transformB2W);
		transformW2B.invert();

		transformB2C.T.setIdentity();
		transformC2B.T.setIdentity();
		bodyPairContactList.clear();
		state = ObjectState.ACTIVE;
		velHistory.clear();
		parent = null;
	}

	/**
	 * Map to keep track of display list IDs for drawing our rigid bodies
	 * efficiently
	 */
	static private HashMap<ArrayList<Block>, Integer> mapBlocksToDisplayList = new HashMap<ArrayList<Block>, Integer>();

	/** display list ID for this rigid body */
	int myListID = -1;
	boolean updateColor = false;

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
		
		if (myListID == -1 || updateColor) {
			Integer ID = mapBlocksToDisplayList.get(blocks);
			if (ID == null || updateColor) {
				updateColor = false;
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
	 * Draws the center of mass position with a circle. The circle will be drawn
	 * differently if the block is at rest (i.e., close to zero kinetic energy)
	 * 
	 * @param drawable
	 */
	public void displayCOM(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		if (!pinned) {
			if (state == ObjectState.ACTIVE || wokenUp) {
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
			} else if (state == ObjectState.SLEEPING && !wokenUp) {
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
	}

	public void drawSpeedCOM(GLAutoDrawable drawable) {
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

	public void printIndex(GLAutoDrawable drawable, int font) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glColor3f(1, 0, 0);
		gl.glRasterPos2d(this.x.x, this.x.y);

		EasyViewer.glut.glutBitmapString(font, Integer.toString(this.index));
	}

	public void unmergeBodyContacts() {

		for (BodyPairContact bc : bodyPairContactList) {
			RigidBody otherBody = bc.getOtherBody(this);
			otherBody.bodyPairContactList.remove(bc);
		}
		bodyPairContactList.clear();
	}
}