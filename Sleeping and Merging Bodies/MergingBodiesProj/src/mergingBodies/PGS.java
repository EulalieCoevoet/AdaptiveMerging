package mergingBodies;

import java.util.ArrayList;
import java.util.HashMap;

import no.uib.cipr.matrix.DenseVector;

/**
 * Projected Gauss Seidel contact solver.
 * 
 * @author 
 */
public class PGS {
	
	public PGS() {
		mu=0.6;
		iterations=200;
		restitution=0;
		warmStart=false;
	}
	
	public PGS(double mu, int iteration, double restitution) {
		this.mu=mu;
		this.iterations=iteration;
		this.restitution=restitution;
	}
	
	/** List of contacts to solve */
	public ArrayList<Contact> contacts = null;

	/** Friction coefficient */
	public double mu;
	
	/** Number of iterations */
	public int iterations;
	
	/** Feedback stiffness coefficient*/
	public double feedbackStiffness;
	
	/** Coefficient of restitution (bounce) */
	public double restitution;
	
	/** Special case for contacts in collections */
	public boolean computeInCollections;
	public boolean confidentWarmStart;
	
	/** Warm start option */
	protected boolean warmStart;
	protected HashMap<String, Contact> lastTimeStepMap = null;
	
	public void enableWarmStart(HashMap<String, Contact> lastTimeStepMap){
		warmStart = true;
		this.lastTimeStepMap = lastTimeStepMap;
	}
	
	public void disableWarmStart(){
		warmStart = false;
	}

	/** Dimension of the system */
	protected int dim;
	
	protected DenseVector lambdas;
	
	/**
	 * Solve contact problem
	 * @param dt time step
	 */
	public void solve(double dt) {
		
		if (contacts == null) {
			System.out.println("Error: PGS.solve() method needs the list PGS.contacts to be filled.");
			return;
		}

		initLambdas();
		organizeContactIndex();
		
		if (confidentWarmStart)
			confidentWarmStart();
		else if (warmStart && lastTimeStepMap != null) 
			warmStart();
		
		while(iterations > 0) {
			
			for (int i=0; i<dim; i++) {
				
				double d = computeJMinvJtDiagonal(i); //move up and store in contact
				double Jdv = computeJdv(i);  
				double b = computeB(i, dt); //move up and store in contact
				// in b we use the velocity

				double lambda = lambdas.get(i);
				double prevLambda = lambda;
				lambda -= (b + Jdv)/d;
				lambda = handleContactConstraints(lambda, i);
				
				//update the force on each body to account for the collision
				//updating lambda vector
				Contact contact = contacts.get(i/2);
				if (i%2 ==0) {
					lambdas.set(2*contact.index,  lambda);
					contact.lambda.set(lambda, contact.lambda.y);
				} else {
					lambdas.set(2*contact.index + 1, lambda);
					contact.lambda.set(contact.lambda.x, lambda);
				}
				
				if (!computeInCollections) {
					double dLambda = lambda - prevLambda;
					updateVelocity(i, dLambda);
				}
			}
			
			iterations--;
		}
	}
	
	protected void organizeContactIndex() {
		for (Contact c : contacts) {
			c.index = contacts.indexOf(c);
		}
	}
	
	protected void initLambdas() {
		lambdas = new DenseVector(2*contacts.size());
		lambdas.zero();
		dim = lambdas.size();
	}
	
	/**
	 * 
	 * @param index index in lambdas list
	 * @return d(index)
	 */
	protected double computeJMinvJtDiagonal(int index) {
		
		Contact contact = contacts.get(index/2);
		RigidBody body1 = (contact.body1.isInCollection() && !computeInCollections)? contact.body1.parent: contact.body1;
		RigidBody body2 = (contact.body2.isInCollection() && !computeInCollections)? contact.body2.parent: contact.body2;
		
		double m1inv = (body1.temporarilyPinned)? 0: body1.minv; 
		double m2inv = (body2.temporarilyPinned)? 0: body2.minv;
		double j1inv = (body1.temporarilyPinned)? 0: body1.jinv;
		double j2inv = (body2.temporarilyPinned)? 0: body2.jinv;

		DenseVector j;
		if (index%2 == 0) // normal component
			j = new DenseVector(contact.j1);
		else // tangent component
			j = new DenseVector(contact.j2);
		
		double d = 0.;
		//first body component
		d += Math.pow(j.get(0), 2) * m1inv;
		d += Math.pow(j.get(1), 2) * m1inv;
		d += Math.pow(j.get(2), 2) * j1inv;
		//second body component
		d += Math.pow(j.get(3), 2) * m2inv;
		d += Math.pow(j.get(4), 2) * m2inv;
		d += Math.pow(j.get(5), 2) * j2inv;
		return d;
	}

	/**
	 * get J Row i Delta V term first
	 * multiply the 6 J values with the appropriate delta V values
	 */
	protected double computeJdv(int index) {
		
		Contact contact = contacts.get(index/2);
		RigidBody body1 = (contact.body1.isInCollection() && !computeInCollections)? contact.body1.parent: contact.body1;
		RigidBody body2 = (contact.body2.isInCollection() && !computeInCollections)? contact.body2.parent: contact.body2;
		
		DenseVector dv1 = body1.deltaV; 
		DenseVector dv2 = body2.deltaV; 
		
		DenseVector j;
		if (index%2 == 0) // normal component
			j = new DenseVector(contact.j1);
		else // tangent component
			j = new DenseVector(contact.j2);
		
		double Jdv = 0.;
		
		//first body
		Jdv += j.get(0) * dv1.get(0);
		Jdv += j.get(1) * dv1.get(1);
		Jdv += j.get(2) * dv1.get(2);
		//second body
		Jdv += j.get(3) * dv2.get(0);
		Jdv += j.get(4) * dv2.get(1);
		Jdv += j.get(5) * dv2.get(2);
		
		return Jdv;
	}
	
	/**
	 * TODO
	 * @param index
	 * @param dt
	 * @param d
	 * @return
	 */
	protected double computeB(int index, double dt) {	
		
		Contact contact = contacts.get(index/2);
		RigidBody body1 = (contact.body1.isInCollection() && !computeInCollections)? contact.body1.parent: contact.body1;
		RigidBody body2 = (contact.body2.isInCollection() && !computeInCollections)? contact.body2.parent: contact.body2;

		double m1inv = (body1.temporarilyPinned)? 0: body1.minv; 
		double m2inv = (body2.temporarilyPinned)? 0: body2.minv;
		double j1inv = (body1.temporarilyPinned)? 0: body1.jinv;
		double j2inv = (body2.temporarilyPinned)? 0: body2.jinv;
		
		DenseVector j;
		if (index%2 == 0) // normal component
			j = new DenseVector(contact.j1);
		else // tangent component
			j = new DenseVector(contact.j2);
		
		// find all relevant values of u.
		double u1x = body1.v.x;
		double u1y = body1.v.y;
		double u1omega = body1.omega;

		double u2x = body2.v.x;
		double u2y = body2.v.y;
		double u2omega = body2.omega;

		// add all relevant values of f, multiplied by appropriate minv to u_1_x etc
		u1x += body1.force.x * m1inv*dt;
		u1y += body1.force.y * m1inv*dt;
		u1omega += body1.torque * j1inv*dt;

		u2x += body2.force.x * m2inv*dt;
		u2y += body2.force.y * m2inv*dt;
		u2omega += body2.torque * j2inv*dt;

		// multiply all the u values by the appropriate J values.
		u1x *= j.get(0);
		u1y *= j.get(1);
		u1omega *= j.get(2);

		u2x *= j.get(3);
		u2y *= j.get(4);
		u2omega *= j.get(5);

		// add the Bounce vector to the u's over here, but don't need to do that just yet
		// bounce bounce bounce bounce bounce bounce bounce bounce bounce bounce ///
		// calculate Baumgarte Feedback (overlap of the two bodies)
		double baumgarteFeedback = feedbackStiffness*contact.constraintViolation;

		// putting b together.
		double b = 0;
		if (index%2 ==0)
			b = u1x + u2x + u1y + u2y + u1omega + u2omega - restitution + baumgarteFeedback;
		else
			b = u1x + u2x + u1y + u2y + u1omega + u2omega;
			
		return b;
	}
	
	/**
	 * Process contact constraints: lambda_n>0, |lambda_t|>=lambda_n*mu
	 * @param lambda
	 * @param index
	 * @return
	 */
	protected double handleContactConstraints(double lambda, int index) {
		if (index%2 == 0) { // normal component
			lambda = Math.max(0, lambda);
		} else { // tangential component
			double lambda_n = lambdas.get(index-1);
			lambda = Math.max(lambda, -mu*lambda_n);
			lambda = Math.min(lambda, mu*lambda_n);
		}
		return lambda;
	}
	
	/**
	 * Update velocity of bodies in contact w.r.t new value of dLambda
	 * @param index
	 * @param dLambda
	 */
	protected void updateVelocity(int index, double dLambda) {
		
		Contact contact = contacts.get(index/2);
		RigidBody body1 = (contact.body1.isInCollection() && !computeInCollections)? contact.body1.parent: contact.body1;
		RigidBody body2 = (contact.body2.isInCollection() && !computeInCollections)? contact.body2.parent: contact.body2;

		double m1inv = (body1.temporarilyPinned)? 0: body1.minv; 
		double m2inv = (body2.temporarilyPinned)? 0: body2.minv;
		double j1inv = (body1.temporarilyPinned)? 0: body1.jinv;
		double j2inv = (body2.temporarilyPinned)? 0: body2.jinv;
		
		DenseVector j;
		if (index%2 == 0) // normal component
			j = new DenseVector(contact.j1);
		else // tangent component
			j = new DenseVector(contact.j2);

		//update delta V;
		DenseVector dv1 = body1.deltaV; 
		DenseVector dv2 = body2.deltaV;
		
		dv1.set( 0, dv1.get(0) + j.get(0) * m1inv * dLambda );
		dv1.set( 1, dv1.get(1) + j.get(1) * m1inv * dLambda );
		dv1.set( 2, dv1.get(2) + j.get(2) * j1inv * dLambda );
		dv2.set( 0, dv2.get(0) + j.get(3) * m2inv * dLambda );
		dv2.set( 1, dv2.get(1) + j.get(4) * m2inv * dLambda );
		dv2.set( 2, dv2.get(2) + j.get(5) * j2inv * dLambda );
	}
	
	/**
	 * Fills lambdas with values from previous time step
	 * TODO: refactor
	 */
	protected void warmStart() {
		for (Contact contact : contacts) {

			DenseVector j1 = new DenseVector(contact.j1);
			DenseVector j2 = new DenseVector(contact.j2);
			Block block1 = contact.block1;
			Block block2 = contact.block2;

			if(lastTimeStepMap.containsKey("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode() ))
					|| lastTimeStepMap.containsKey("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode() ))) {

				RigidBody body1 = (contact.body1.isInCollection())? contact.body1.parent: contact.body1;
				RigidBody body2 = (contact.body2.isInCollection())? contact.body2.parent: contact.body2;
				
				double m1inv = (body1.temporarilyPinned)? 0: body1.minv; 
				double m2inv = (body2.temporarilyPinned)? 0: body2.minv;
				double j1inv = (body1.temporarilyPinned)? 0: body1.jinv;
				double j2inv = (body2.temporarilyPinned)? 0: body2.jinv;

				Contact c = lastTimeStepMap.get("contact:" + Integer.toString(block1.hashCode()) + "_" + Integer.toString(block2.hashCode()));
				if(lastTimeStepMap.containsKey("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode() )))
					c = lastTimeStepMap.get("contact:" + Integer.toString(block2.hashCode()) + "_" + Integer.toString(block1.hashCode()));
				//if the old map contains this key, then get the lamda of the old map

				RigidBody cbody1 = (c.body1.isInCollection())? c.body1.parent: c.body1;
				RigidBody cbody2 = (c.body2.isInCollection())? c.body2.parent: c.body2;
				if (cbody1 != body1 || cbody2 != body2) {
					continue;
				}

				double old_lamda_n = c.lambda.x;
				double old_lamda_t = c.lambda.y;
				double old_delta_lamda_n = old_lamda_n;
				double old_delta_lamda_t = old_lamda_t;
				//set this lamda to the old lamda
				lambdas.set(2*contact.index, old_lamda_n);
				lambdas.set(2*contact.index + 1, old_lamda_t);
				//recompute Delta V's

				//first recompute t.
				double t_1_x_n=0, t_1_y_n=0, t_1_omega_n=0, t_2_x_n=0, t_2_y_n=0, t_2_omega_n=0;
				double t_1_x_t=0, t_1_y_t=0, t_1_omega_t=0, t_2_x_t=0, t_2_y_t=0, t_2_omega_t=0;
				//first body
				t_1_x_n = j1.get(0) * m1inv*old_delta_lamda_n;
				t_1_y_n = j1.get(1) * m1inv*old_delta_lamda_n;
				t_1_omega_n = j1.get(2) * j1inv*old_delta_lamda_n;
				//second body
				t_2_x_n = j1.get(3) * m2inv*old_delta_lamda_n;
				t_2_y_n = j1.get(4) * m2inv*old_delta_lamda_n;
				t_2_omega_n = j1.get(5) * j2inv*old_delta_lamda_n;

				//first body
				t_1_x_t = j2.get(0) * m1inv*old_delta_lamda_t;
				t_1_y_t = j2.get(1) * m1inv*old_delta_lamda_t;
				t_1_omega_t = j2.get(2) * j1inv*old_delta_lamda_t;
				//second body
				t_2_x_t = j2.get(3) * m2inv*old_delta_lamda_t;
				t_2_y_t = j2.get(4) * m2inv*old_delta_lamda_t;
				t_2_omega_t = j2.get(5) * j2inv* old_delta_lamda_t;

				//update delta V;
				DenseVector dV1 = cbody1.deltaV; 
				DenseVector dV2 = cbody2.deltaV; 
				dV1.set(0, dV1.get(0) + t_1_x_n  + t_1_x_t);
				dV1.set(1, dV1.get(1) + t_1_y_n + t_1_y_t );
				dV1.set(2, dV1.get(2) + t_1_omega_n + t_1_omega_t);

				//update delta V;
				dV2.set(0, dV2.get(0) + t_2_x_n + t_2_x_t);
				dV2.set(1, dV2.get(1) + t_2_y_n + t_2_y_t );
				dV2.set(2, dV2.get(2) + t_2_omega_n + t_2_omega_t );
				
				contact.body1ContactForceHistory.clear();
				contact.body1ContactTorqueHistory.clear();
				contact.body2ContactForceHistory.clear();
				contact.body2ContactTorqueHistory.clear();
				contact.body1ContactForceHistory.addAll(c.body1ContactForceHistory);
				contact.body1ContactTorqueHistory.addAll(c.body1ContactTorqueHistory);
				contact.body2ContactForceHistory.addAll(c.body2ContactForceHistory);
				contact.body2ContactTorqueHistory.addAll(c.body2ContactTorqueHistory);
			}
		}
	}
	
	/**
	 * Warm start if the confidence that each contact already has a value for lambda
	 */
	protected void confidentWarmStart() {
		int index = 0;
		for (Contact contact : contacts) {
			lambdas.set(index, contact.lambda.x);
			lambdas.set(index+1, contact.lambda.y);
			index+=2;
		}
	}
}
