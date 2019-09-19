package mergingBodies;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import no.uib.cipr.matrix.DenseVector;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Implementation of a contact constraint.
 * @author kry
 */
public class Contact {

	/** Next available contact index, used for determining which rows of the Jacobian a contact uses */
	static public int nextContactIndex = 0;

	/** Index of this contact, determines its rows in the Jacobian */
	int index;

	/** visitID of this contact at this time step. */
	boolean visited = false;

	/**    If the contact is between two sleeping bodies or not  */
	boolean passive = false;
	
	/**    Used for unmerge condition, true if the contact changed (break or slide) during the time step  */
	public ContactState state = ContactState.CLEAR;
	
	public enum ContactState {BROKE, SLIDING, CLEAR};
	
	/** First RigidBody in contact */
	public RigidBody body1;

	/** Second RigidBody in contact */
	public RigidBody body2;

	//The two blocks within the bodies that caused the collision
	Block block1;

	Block block2;

	/** Contact normal in world coordinates. GOES FROM BODY1 to BODY2*/
	Vector2d normal = new Vector2d();

	/** Contact normal in Body1 coordinates. GOES FROM BODY1 to BODY2*/
	Vector2d normalB1 = new Vector2d();

	/** Contact normal in Body2 coordinates. GOES FROM BODY1 to BODY2*/
	Vector2d normalB2 = new Vector2d();

	/** Contact force being applied by this contact on body1*/
	Vector2d contactForceB1 = new Vector2d();

	/** Contact torque being applied by this contact on body1*/
	double contactTorqueB1 = 0;

	/** Contact force being applied by this contact on body2*/
	Vector2d contactForceB2 = new Vector2d();

	/** Contact torque being applied by this contact on body2*/
	double contactTorqueB2 = 0;

	/** Position of contact point in world coordinates */
	Point2d contactW = new Point2d();
	
	/** Position of contact point in Body1 coordinates */
	Point2d contactB1  = new Point2d();
	
	/** Position of contact point in Body2 coordinates */
	Point2d contactB2  = new Point2d();
	
	/** Jacobian for normal direction */ 
	DenseVector j1 = new DenseVector(6);
	
	/** Jacobian for tangential direction */ 
	DenseVector j2 = new DenseVector(6);

	/** Lagrange multiplier for contact, Vector2d(normal, tangent) */
	Vector2d lambda = new Vector2d();
	
	BVNode bvn1;
	BVNode bvn2;

	/** vector points from body 2 to body 1, magnitude is the amount of overlap.*/
	double constraintViolation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact

	Vector2d relativeVelocity = new Vector2d();

	double relativeAngularVelocity = 0;

	/** Pointer to the BodyContact this contact is a part of. */
	BodyPairContact bc;

	/** history of the last (max N) time steps for this contact. */
	public ArrayList<Vector2d> body1ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body1ContactTorqueHistory = new ArrayList<Double>();
	public ArrayList<Vector2d> body2ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body2ContactTorqueHistory = new ArrayList<Double>();

	/**
	 * Creates a new contact, and assigns it an index
	 * @param body1
	 * @param body2
	 * @param contactW
	 * @param normal
	 */
	public Contact( RigidBody body1, RigidBody body2, Point2d contactW, Vector2d normal, Block b1, Block b2, double distance) {
		
		this.body1 = body1;
		this.body2 = body2;
		this.contactW.set( contactW );
		this.normal.set( normal );   
		block1 = b1;
		block2 = b2;
		constraintViolation =  distance - 2*Block.radius;
		index = nextContactIndex++;        

		computeRelativeVelocity();
		computeJacobian(false);

		contactB1.set(contactW);
		contactB2.set(contactW);
		body1.transformW2B.transform(contactB1);
		body2.transformW2B.transform(contactB2);
	}

	/**
	 * Computes the relative velocity between the two bodies in contact.
	 * In case of body in a collection, use velocities of parent.
	 */
	protected void computeRelativeVelocity() {
		RigidBody body1 = (this.body1.isInCollection())? this.body1.parent: this.body1;
		RigidBody body2 = (this.body2.isInCollection())? this.body2.parent: this.body2;
		
		relativeVelocity.sub(body2.v, body1.v);
		relativeAngularVelocity = body2.omega - body1.omega;
	}
	
	/**
	 * Computes the Jacobian matrix of the contact.
	 * In case of body in a collection, use COM of parent to compute the torque component of the Jacobian.
	 */
	public void computeJacobian(boolean computeInCollection) {
		
		RigidBody b1 = (body1.isInCollection() && !computeInCollection )? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection )? body2.parent: body2;

		Point2d radiusBody1 = new Point2d(b1.x);
		Point2d radiusBody2 = new Point2d(b2.x);

		Vector2d contactPoint = new Vector2d(contactW);
		radiusBody1.sub(contactPoint, radiusBody1);
		radiusBody2.sub(contactPoint, radiusBody2);

		Vector2d r1 = new Vector2d(-radiusBody1.y, radiusBody1.x);
		Vector2d r2 = new Vector2d(-radiusBody2.y, radiusBody2.x);
		j1.set(0, -normal.x); 
		j1.set(1, -normal.y);
		j1.set(2, - r1.dot(normal));
		j1.set(3, normal.x);
		j1.set(4, normal.y);
		j1.set(5, r2.dot(normal)); 

		Vector2d tangent = new Vector2d(-normal.y, normal.x);
		j2.set(0, -tangent.x);
		j2.set(1, -tangent.y);
		j2.set(2, - r1.dot(tangent));
		j2.set(3, tangent.x);
		j2.set(4, tangent.y);
		j2.set(5, r2.dot(tangent));
	}
	
	public void computeContactForce(double dt) {
		
		Vector2d cForce = new Vector2d();
		double cTorque = 0;
		
		cForce.set(lambda.x*j1.get(0) + lambda.y*j2.get(0),lambda.x*j1.get(1) + lambda.y*j2.get(1));
		cTorque = lambda.x*j1.get(2) + lambda.y*j2.get(2);
		cForce.scale(1/dt);
		cTorque/=dt;
		contactForceB1.set(cForce);
		contactTorqueB1 = cTorque;

		cForce.set(lambda.x*j1.get(3) + lambda.y*j2.get(3),lambda.x*j1.get(4) + lambda.y*j2.get(4));
		cTorque = lambda.x*j1.get(5) + lambda.y*j2.get(5);
		cForce.scale(1/dt);
		cTorque/=dt;
		contactForceB2.set(cForce);
		contactTorqueB2 = cTorque;
	}
	
	/**
	 * Returns if given body is part of the contact
	 * @param body
	 * @return
	 */
	public boolean withBody(RigidBody body) {
		return (body1==body || body2==body);
	}
	
	/**
	 * Computes and returns the relative kinetic energy without the mass
	 * @return metric
	 */
	public double getRelativeMetric() {
		double k = 0.5*relativeVelocity.lengthSquared() + 0.5*relativeAngularVelocity*relativeAngularVelocity;
		return k;
	}

	/**
	 * Draws the contact points
	 * @param drawable
	 */
	public void display( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glPointSize(5);
		gl.glColor3f(.7f,0,0);
		gl.glBegin( GL.GL_POINTS );
		gl.glVertex2d(contactW.x, contactW.y);
		gl.glEnd();
	}

	/**
	 * Draws the connections between bodies to visualize 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void displayConnection( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		// draw a line between the two bodies but only if they're both not pinned
		if ( !body1.pinned && ! body2.pinned ) {
			gl.glLineWidth(2);
			gl.glColor4f(0,.3f,0, 0.5f);
			gl.glBegin( GL.GL_LINES );
			
			Point2d p1 = new Point2d(body1.x);
			Point2d p2 = new Point2d(body2.x);

			gl.glVertex2d(p1.x, p1.y);
			gl.glVertex2d(p2.x, p2.y);
			gl.glEnd();
		}
	}

	/**
	 * Draws the connections between bodies to visualize the 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void drawContactForce( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();

		gl.glLineWidth(2);
		gl.glColor4f(1, 0, 0, 1);
		gl.glBegin( GL.GL_LINES );

		double scale = 0.05;
		gl.glVertex2d(contactW.x + scale*contactForceB1.x, contactW.y+scale*contactForceB1.y);
		gl.glVertex2d(contactW.x + scale*contactForceB2.x, contactW.y+scale*contactForceB2.y);
		
		gl.glEnd();
	}


	/**
	 * Draws the connections between bodies to visualize the 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void drawInternalContactForce( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();

		gl.glColor4f(0, 0, 1, 1);
		
		gl.glLineWidth(2);
		gl.glBegin( GL.GL_LINES );
		
		double scale = 0.05;
		gl.glVertex2d(contactW.x + scale*contactForceB1.x, contactW.y+scale*contactForceB1.y);
		gl.glVertex2d(contactW.x + scale*contactForceB2.x, contactW.y+scale*contactForceB2.y);
		
		gl.glEnd();
		
		gl.glPointSize(5);		
		gl.glBegin( GL.GL_POINTS );
		
		gl.glVertex2d(contactW.x, contactW.y);
		
		gl.glEnd();
	}

	/*
	 * Variables that help with measuring the variance and average contact force
	 */
	Vector2d body1Force = new Vector2d();
	Vector2d body1ForceVar = new Vector2d();
	Vector2d body2Force = new Vector2d();
	Vector2d body2ForceVar = new Vector2d();

	Point2d max_x_1 = new Point2d();
	Point2d max_y_1 = new Point2d();
	Point2d min_x_1 = new Point2d();
	Point2d min_y_1= new Point2d();

	Point2d max_x_2 = new Point2d();
	Point2d max_y_2 = new Point2d();
	Point2d min_x_2 = new Point2d();
	Point2d min_y_2= new Point2d();


	public void getHistoryStatistics(){
		// start with body1.
		body1Force= getAverageContactForce(body1ContactForceHistory);
		//	double body1Torque = getAverageContactTorque(body1ContactTorqueHistory);
		body1ForceVar= getContactForceVar(body1ContactForceHistory, body1Force);
		//double body1TorqueVar = getContactTorqueVar(body1ContactTorqueHistory, body1Torque);
		//convert variance to stdev
		body1ForceVar.set(Math.sqrt(body1ForceVar.x), Math.sqrt(body1ForceVar.y));
		//for confidence interval must scale
		body1ForceVar.scale(1/Math.sqrt((body1ContactForceHistory.size())));

		//get average body2.
		body2Force= getAverageContactForce(body2ContactForceHistory);
		//	double body2Torque = getAverageContactTorque(body2ContactTorqueHistory);
		body2ForceVar= getContactForceVar(body2ContactForceHistory, body2Force);
		//	double body2TorqueVar = getContactTorqueVar(body2ContactTorqueHistory,body2Torque);
		body2ForceVar.set(Math.sqrt(body2ForceVar.x), Math.sqrt(body2ForceVar.y));
		body2ForceVar.scale(1/Math.sqrt((body2ContactForceHistory.size())));

		//for 96 percent confidence interval, scale the stdev by two
		body1ForceVar.scale(2);
		body2ForceVar.scale(2);

		Point2d p1 = new Point2d(contactB1);
		Point2d p2 = new Point2d(contactB2);

		double scale = 0.05;
		max_x_1.set(p1.x + scale*body1Force.x + scale*body1ForceVar.x, p1.y + scale*body1Force.y );
		min_x_1.set(p1.x + scale*body1Force.x - scale*body1ForceVar.x, p1.y + scale*body1Force.y);
		max_y_1.set(p1.x + scale*body1Force.x, p1.y + scale*body1Force.y + scale*body1ForceVar.y);
		min_y_1.set(p1.x + scale*body1Force.x, p1.y + scale*body1Force.y - scale*body1ForceVar.y);

		max_x_2.set(p2.x + scale*body2Force.x + scale*body2ForceVar.x, p2.y + scale*body2Force.y );
		min_x_2.set(p2.x + scale*body2Force.x - scale*body2ForceVar.x, p2.y + scale*body2Force.y );
		max_y_2.set(p2.x + scale*body2Force.x, p2.y + scale*body2Force.y + scale*body2ForceVar.y);
		min_y_2.set(p2.x + scale*body2Force.x, p2.y + scale*body2Force.y - scale*body2ForceVar.y);
	}
	
	
	/**
	 * Draws the connections between bodies to visualize the 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void drawInternalContactHistory( GLAutoDrawable drawable ) {

		GL2 gl = drawable.getGL().getGL2();
		// draw a line between the two bodies but only if they're both not pinned
		Point2d p1 = new Point2d(contactB1);
		Point2d p2 = new Point2d(contactB2);

		body1.transformB2W.transform(p1); 
		body2.transformB2W.transform(p2);
		body1.transformB2W.transform(body1Force);
		body2.transformB2W.transform(body2Force);

		body1.transformB2W.transform(min_x_1);
		body1.transformB2W.transform(min_y_1);
		body1.transformB2W.transform(max_x_1);
		body1.transformB2W.transform(max_y_1);
		body2.transformB2W.transform(min_x_2);
		body2.transformB2W.transform(min_y_2);
		body2.transformB2W.transform(max_x_2);
		body2.transformB2W.transform(max_y_2);

		double scale = 0.05;
		//draw average 	
		gl.glLineWidth(1);
		gl.glColor4f(0,0 , 1, 1);
		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(p2.x, p2.y );
		gl.glVertex2d(p2.x + scale*body2Force.x, p2.y+ scale*body2Force.y);
		gl.glEnd();
		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(p1.x, p1.y );
		gl.glVertex2d(p1.x + scale*body1Force.x, p1.y+ scale*body1Force.y);
		gl.glEnd();

		//draw variance error margins
		gl.glLineWidth(1);
		gl.glColor4f(0, 1, 1, 1);
		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(min_x_2.x, min_x_2.y);
		gl.glVertex2d(max_x_2.x, max_x_2.y);
		gl.glEnd();

		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(min_y_2.x, min_y_2.y);
		gl.glVertex2d(max_y_2.x, max_y_2.y);
		gl.glEnd();

		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(min_x_1.x, min_x_1.y);
		gl.glVertex2d(max_x_1.x, max_x_1.y);
		gl.glEnd();

		gl.glBegin( GL.GL_LINES );
		gl.glVertex2d(min_y_1.x, min_y_1.y);
		gl.glVertex2d(max_y_1.x, max_y_1.y);
		gl.glEnd();

		body1.transformW2B.transform(body1Force);
		body2.transformW2B.transform(body2Force);

		body1.transformW2B.transform(min_x_1);
		body1.transformW2B.transform(min_y_1);
		body1.transformW2B.transform(max_x_1);
		body1.transformW2B.transform(max_y_1);
		body2.transformW2B.transform(min_x_2);
		body2.transformW2B.transform(min_y_2);
		body2.transformW2B.transform(max_x_2);
		body2.transformW2B.transform(max_y_2);
		
		gl.glPointSize(3);
		gl.glColor3f(0,0,0.7f);
		gl.glBegin( GL.GL_POINTS );
		gl.glVertex2d(p1.x, p1.y);
		gl.glEnd();
	}

	private Vector2d getContactForceVar(ArrayList<Vector2d> list, Vector2d avg) {

		double sum1 = 0;
		double sum2 = 0;
		for (Vector2d vec : list) {
			sum1+= (vec.x - avg.x)*(vec.x - avg.x);
			sum2+= (vec.y - avg.y)*(vec.y - avg.y);
		}
		if (list.size()>1) {
			sum1/=(list.size() - 1);
			sum2 /= (list.size() - 1);
		}
		return new Vector2d(sum1, sum2);
	}

	private Vector2d getAverageContactForce(ArrayList<Vector2d> list) {
		double x = 0;
		double y = 0;
		for (Vector2d vec: list) {
			x += vec.x;
			y += vec.y;
		}
		x/=list.size();
		y/= list.size();
		return new Vector2d(x, y);
	}

}