package mergingBodies;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;
import no.uib.cipr.matrix.DenseVector;

import java.util.ArrayList;

import javax.vecmath.Color3f;
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
	
	/**    Used for unmerge condition, true if the contact changed (break or slide) during the time step  */
	public ContactState state = ContactState.CLEAR;
	
	public enum ContactState {BROKEN, ONEDGE, CLEAR};
	
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
	DenseVector jn = new DenseVector(6);
	
	/** Jacobian for normal direction collection frame */ 
	DenseVector jnc = new DenseVector(6);
	
	/** Jacobian for tangential direction */ 
	DenseVector jt = new DenseVector(6);
	
	/** Jacobian for tangential direction collection frame */ 
	DenseVector jtc = new DenseVector(6);

	/** Lagrange multiplier for contact, Vector2d(normal, tangent) */
	Vector2d lambda = new Vector2d();
	
	// Values used in PGS resolution
	double bn = 0; /** b value for normal component */
	double bt = 0; /** b value for tangent component */
	double diin = 0; /** diagonal entry for normal component */
	double diit = 0; /** diagonal entry for tangent component */
	
	BVNode bvn1;
	BVNode bvn2;

	/** vector points from body 2 to body 1, magnitude is the amount of overlap.*/
	double constraintViolation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
	double prevConstraintViolation;
	
	/** history of the last (max N) time steps for this contact. */
	public ArrayList<Vector2d> body1ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body1ContactTorqueHistory = new ArrayList<Double>();
	public ArrayList<Vector2d> body2ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body2ContactTorqueHistory = new ArrayList<Double>();
	
	/**
	 *  Creates a new contact, and assigns it an index
	 * @param body1
	 * @param body2
	 * @param contactW
	 * @param normal
	 * @param b1
	 * @param b2
	 * @param interpenetration (should be negative)
	 */
	public Contact( RigidBody body1, RigidBody body2, Point2d contactW, Vector2d normal, Block b1, Block b2, double interpenetration) {
				
		if(body1 instanceof RigidCollection || body2 instanceof RigidCollection)
			System.err.println("[Contact] Contact should never involve a collection.");
		
		this.body1 = body1;
		this.body2 = body2;
		this.contactW.set( contactW );
		this.normal.set( normal );   
		block1 = b1;
		block2 = b2;
		double offset  = CollisionProcessor.constraintOffset.getValue();
		constraintViolation =  interpenetration + offset;
		index = nextContactIndex++;        

		contactB1.set(contactW);
		contactB2.set(contactW);
		body1.transformW2B.transform(contactB1);
		body2.transformW2B.transform(contactB2);
		
		// set normals in body coordinates
		normalB1.set(normal);
		normalB2.set(normal);
		body1.transformW2B.transform(normalB1);
		body2.transformW2B.transform(normalB2);
		normalB2.scale(-1);
		
		computeJacobian(false);
		computeJacobian(true);
	}
	
	/**
	 * Computes the Jacobian matrix of the contact.
	 * In case of body in a collection, use COM of parent to compute the torque component of the Jacobian.
	 */
	public void computeJacobian(boolean computeInCollection) {
				
		RigidBody b1 = (body1.isInCollection() && !computeInCollection )? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection )? body2.parent: body2;

		body1.transformB2W.transform(contactB1, contactW);
		body1.transformB2W.transform(normalB1, normal);
		
		Point2d radiusBody1 = new Point2d();
		Point2d radiusBody2 = new Point2d();

		radiusBody1.sub(contactW, b1.x);
		radiusBody2.sub(contactW, b2.x);

		Vector2d r1 = new Vector2d(-radiusBody1.y, radiusBody1.x);
		Vector2d r2 = new Vector2d(-radiusBody2.y, radiusBody2.x);
		Vector2d tangent = new Vector2d(-normal.y, normal.x);
		
		DenseVector jn;
		DenseVector jt;

		jn = (b1 instanceof RigidCollection)? jnc: this.jn;
		jt = (b1 instanceof RigidCollection)? jtc: this.jt;
		jn.set(0, -normal.x); 
		jn.set(1, -normal.y);
		jn.set(2, -r1.dot(normal));
		jt.set(0, -tangent.x);
		jt.set(1, -tangent.y);
		jt.set(2, -r1.dot(tangent));

		jn = (b2 instanceof RigidCollection)? jnc: this.jn;
		jt = (b2 instanceof RigidCollection)? jtc: this.jt;
		jn.set(3, normal.x);
		jn.set(4, normal.y);
		jn.set(5, r2.dot(normal)); 
		jt.set(3, tangent.x);
		jt.set(4, tangent.y);
		jt.set(5, r2.dot(tangent));
	}
	
	/**
	 * Stores contact forces and torques for visualization purposes
	 * @param dt
	 */
	public void computeContactForce(boolean computeInCollection, double dt) {

		DenseVector jn;
		DenseVector jt;
		
		Vector2d cForce = new Vector2d();
		double cTorque = 0;

		jn = (body1.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		jt = (body1.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		cForce.set(lambda.x*jn.get(0) + lambda.y*jt.get(0),lambda.x*jn.get(1) + lambda.y*jt.get(1));
		cTorque = lambda.x*jn.get(2) + lambda.y*jt.get(2);
		cForce.scale(1/dt);
		cTorque/=dt;
		contactForceB1.set(cForce);
		contactTorqueB1 = cTorque;

		jn = (body2.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		jt = (body2.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		cForce.set(lambda.x*jn.get(3) + lambda.y*jt.get(3),lambda.x*jn.get(4) + lambda.y*jt.get(4));
		cTorque = lambda.x*jn.get(5) + lambda.y*jt.get(5);
		cForce.scale(1/dt);
		cTorque/=dt;
		contactForceB2.set(cForce);
		contactTorqueB2 = cTorque;
	}
	
	/**
	 * Update state of the contact: either BROKE, SLIDING or CLEAR
	 * @param prevLambda_n
	 * @param mu
	 */
	protected void updateContactState(double mu) {
		if (Math.abs(lambda.x) <= 1e-14) // (math.abs is for magnet)
			state = ContactState.BROKEN;	
		else if (Math.abs(lambda.y) == lambda.x*mu) 
			state = ContactState.ONEDGE;
		else
			state = ContactState.CLEAR;
	}
	
	/**
	 * 
	 * @param dt
	 * @param restitution
	 * @param feedbackStiffness
	 * @param computeInCollection
	 */
	public void computeB(double dt, double restitution, double feedbackStiffness,  boolean computeInCollection) {
		
		RigidBody b1 = (body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection)? body2.parent: body2;

		double m1inv = (b1.temporarilyPinned)? 0: b1.minv; 
		double m2inv = (b2.temporarilyPinned)? 0: b2.minv;
		double j1inv = (b1.temporarilyPinned)? 0: b1.jinv;
		double j2inv = (b2.temporarilyPinned)? 0: b2.jinv;
		
		// add the Bounce vector to the u's over here, but don't need to do that just yet
		if (computeInCollection)
			restitution=0.;
		
		DenseVector j;
		
		j = (b1 instanceof RigidCollection)? this.jnc: this.jn;
		double u1xn =     (b1.v.x + b1.force.x * m1inv * dt) * j.get(0);
		double u1yn =     (b1.v.y + b1.force.y * m1inv * dt) * j.get(1);
		double u1omegan = (b1.omega + b1.torque * j1inv * dt) * j.get(2);
		double bBounce = restitution*(b1.v.x*j.get(0) + b1.v.y*j.get(1) + b1.omega*j.get(2));

		j = (b1 instanceof RigidCollection)? this.jtc: this.jt;
		double u1xt =     (b1.v.x + b1.force.x * m1inv * dt) * j.get(0);
		double u1yt =     (b1.v.y + b1.force.y * m1inv * dt) * j.get(1);
		double u1omegat = (b1.omega + b1.torque * j1inv * dt) * j.get(2);
			
		j = (b2 instanceof RigidCollection)? this.jnc: this.jn;
		double u2xn =     (b2.v.x + b2.force.x * m2inv * dt) * j.get(3);
		double u2yn =     (b2.v.y + b2.force.y * m2inv * dt) * j.get(4);
		double u2omegan = (b2.omega + b2.torque * j2inv * dt) * j.get(5);
		bBounce += restitution*(b2.v.x*j.get(3) + b2.v.y*j.get(4) + b2.omega*j.get(5));

		j = (b2 instanceof RigidCollection)? this.jtc: this.jt; 
		double u2xt =     (b2.v.x + b2.force.x * m2inv * dt) * j.get(3);
		double u2yt =     (b2.v.y + b2.force.y * m2inv * dt) * j.get(4);
		double u2omegat = (b2.omega + b2.torque * j2inv * dt) * j.get(5);

		// calculate Baumgarte Feedback (overlap of the two bodies)
		double baumgarteFeedback = feedbackStiffness*constraintViolation;
		
		// putting b together.
		bn = u1xn + u2xn + u1yn + u2yn + u1omegan + u2omegan + bBounce + baumgarteFeedback;
		bt = u1xt + u2xt + u1yt + u2yt + u1omegat + u2omegat;
	}
	
	/**
	 * Compute Dii values and store in contact
	 * @param computeInCollection
	 * @param compliance 
	 */
	public void computeJMinvJtDiagonal(boolean computeInCollection) {
		
		RigidBody b1 = (body1.isInCollection() && !computeInCollection)? body1.parent: body1;
		RigidBody b2 = (body2.isInCollection() && !computeInCollection)? body2.parent: body2;
		
		double m1inv = (b1.temporarilyPinned)? 0: b1.minv; 
		double m2inv = (b2.temporarilyPinned)? 0: b2.minv;
		double j1inv = (b1.temporarilyPinned)? 0: b1.jinv;
		double j2inv = (b2.temporarilyPinned)? 0: b2.jinv;
		
		DenseVector jn;
		DenseVector jt;
		
		diin = 0.;
		diit = 0.;
		
		jn = (b1 instanceof RigidCollection)? this.jnc: this.jn;
		jt = (b1 instanceof RigidCollection)? this.jtc: this.jt;
		diin += jn.get(0) * m1inv * jn.get(0);
		diin += jn.get(1) * m1inv * jn.get(1);
		diin += jn.get(2) * j1inv * jn.get(2);
		diit += jt.get(0) * m1inv * jt.get(0);
		diit += jt.get(1) * m1inv * jt.get(1);
		diit += jt.get(2) * j1inv * jt.get(2);
		
		jn = (b2 instanceof RigidCollection)? this.jnc: this.jn;
		jt = (b2 instanceof RigidCollection)? this.jtc: this.jt;
		diin += jn.get(3) * m2inv * jn.get(3);
		diin += jn.get(4) * m2inv * jn.get(4);
		diin += jn.get(5) * j2inv * jn.get(5);
		diit += jt.get(3) * m2inv * jt.get(3);
		diit += jt.get(4) * m2inv * jt.get(4);
		diit += jt.get(5) * j2inv * jt.get(5);
	}
	
	/**
	 * Returns Jdv values for normal component.
	 * @param computeInCollection
	 */
	public double getJdvn(boolean computeInCollection) {
		
		DenseVector dv1 = (body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
		DenseVector dv2 = (body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
		DenseVector jn;
		
		double Jdvn = 0;  		
		
		jn = (body1.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		Jdvn += jn.get(0) * dv1.get(0);
		Jdvn += jn.get(1) * dv1.get(1);
		Jdvn += jn.get(2) * dv1.get(2);
		
		jn = (body2.isInCollection() && !computeInCollection)? this.jnc: this.jn;
		Jdvn += jn.get(3) * dv2.get(0);
		Jdvn += jn.get(4) * dv2.get(1);
		Jdvn += jn.get(5) * dv2.get(2);
		
		return Jdvn;
	}
	
	/**
	 * Returns Jdv values for tangent component.
	 * @param computeInCollection
	 */
	public double getJdvt(boolean computeInCollection) {
		
		DenseVector dv1 = (body1.isInCollection() && !computeInCollection)? body1.parent.deltaV : body1.deltaV; 
		DenseVector dv2 = (body2.isInCollection() && !computeInCollection)? body2.parent.deltaV : body2.deltaV; 
		
		DenseVector jt;
		
		// normal component
		double Jdvt = 0;  		

		jt = (body1.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		Jdvt += jt.get(0) * dv1.get(0);
		Jdvt += jt.get(1) * dv1.get(1);
		Jdvt += jt.get(2) * dv1.get(2);

		jt = (body2.isInCollection() && !computeInCollection)? this.jtc: this.jt;
		Jdvt += jt.get(3) * dv2.get(0);
		Jdvt += jt.get(4) * dv2.get(1);
		Jdvt += jt.get(5) * dv2.get(2);
		
		return Jdvt;
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
	 * Getter for lambda
	 * @return metric
	 */
	public Vector2d getLambda() {
		Vector2d l = new Vector2d(lambda);
		return l;
	}

	/**
	 * Draws the connections between bodies to visualize 
	 * the adjacency structure of the matrix as a graph.
	 * @param drawable
	 */
	public void displayContactGraph( GLAutoDrawable drawable ) {
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
	public void displayContactForce( GLAutoDrawable drawable, Color3f color ) {
		GL2 gl = drawable.getGL().getGL2();

		gl.glLineWidth(2);
		if (state == ContactState.ONEDGE)
			gl.glColor4f(0, 0, 0, 1);
		else
			gl.glColor4f(color.x, color.y, color.z, 1);
		gl.glBegin( GL.GL_LINES );

		double scale = forceVizScale.getValue();
		gl.glVertex2d(contactW.x + scale*contactForceB1.x, contactW.y+scale*contactForceB1.y);
		gl.glVertex2d(contactW.x + scale*contactForceB2.x, contactW.y+scale*contactForceB2.y);
		
		gl.glEnd();
	}
	
	/**
	 * Draws the contact points
	 * @param drawable
	 */
	public void displayContactLocation( GLAutoDrawable drawable, Color3f color, int size) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glPointSize(size);
		gl.glColor4f(color.x, color.y, color.z, 1);
		gl.glBegin( GL.GL_POINTS );
		gl.glVertex2d(contactW.x, contactW.y);
		gl.glEnd();
	}

	static DoubleParameter forceVizScale = new DoubleParameter("force viz scale", 0.05, 0.0001, 1);

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
	Point2d min_y_1 = new Point2d();

	Point2d max_x_2 = new Point2d();
	Point2d max_y_2 = new Point2d();
	Point2d min_x_2 = new Point2d();
	Point2d min_y_2 = new Point2d();


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
