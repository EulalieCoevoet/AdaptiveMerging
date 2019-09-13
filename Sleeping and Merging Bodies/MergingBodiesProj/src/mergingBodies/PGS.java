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
			
			for (int i=0; i<lambdas.size(); i++) {
				
				double d = computeJMinvJtDiagonal(i); 
				double Jdv = computeJdv(i);  
				double b = computeB(i, dt); 

				double lambda = lambdas.get(i);
				double prevLambda = lambda;
				lambda -= (b + Jdv)/d;
				lambda = processContactConstraints(lambda, i);
				
				updateLambda(i, lambda);
				
				double dLambda = lambda - prevLambda;
				updateVelocity(i, dLambda);
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
	}
	
	/**
	 * Compute Dii
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
		d += j.get(0) * m1inv * j.get(0);
		d += j.get(1) * m1inv * j.get(1);
		d += j.get(2) * j1inv * j.get(2);
		//second body component
		d += j.get(3) * m2inv * j.get(3);
		d += j.get(4) * m2inv * j.get(4);
		d += j.get(5) * j2inv * j.get(5);
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
		double u1x =     (body1.v.x + body1.force.x * m1inv * dt) * j.get(0);
		double u1y =     (body1.v.y + body1.force.y * m1inv * dt) * j.get(1);
		double u1omega = (body1.omega + body1.torque * j1inv * dt) * j.get(2);

		double u2x =     (body2.v.x + body2.force.x * m2inv * dt) * j.get(3);
		double u2y =     (body2.v.y + body2.force.y * m2inv * dt) * j.get(4);
		double u2omega = (body2.omega + body2.torque * j2inv * dt) * j.get(5);

		// add the Bounce vector to the u's over here, but don't need to do that just yet
		// bounce bounce bounce bounce bounce bounce bounce bounce bounce bounce ///
		// calculate Baumgarte Feedback (overlap of the two bodies)
		double baumgarteFeedback = (computeInCollections)? 0. : feedbackStiffness*contact.constraintViolation;

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
	protected double processContactConstraints(double lambda, int index) {
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
	 * @param lambda
	 */
	protected void updateVelocity(int index, double lambda) {
		
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
		dv1.add( 0, j.get(0) * m1inv * lambda );
		dv1.add( 1, j.get(1) * m1inv * lambda );
		dv1.add( 2, j.get(2) * j1inv * lambda );
		dv2.add( 0, j.get(3) * m2inv * lambda );
		dv2.add( 1, j.get(4) * m2inv * lambda );
		dv2.add( 2, j.get(5) * j2inv * lambda );
	}
	
	/**
	 * Updates the force on each body to account for the collision and 
	 * also updates lambdas vector.
	 * @param index
	 * @param lambda
	 */
	protected void updateLambda(int index, double lambda) {
		Contact contact = contacts.get(index/2);
		if (index%2 ==0) {
			lambdas.set(2*contact.index,  lambda);
			contact.lambda.set(lambda, contact.lambda.y);
		} else {
			lambdas.set(2*contact.index + 1, lambda);
			contact.lambda.set(contact.lambda.x, lambda);
		}
	}
	
	/**
	 * Fills lambdas with values from previous time step
	 */
	protected void warmStart() {
		for (Contact contact : contacts) {
			Contact oldContact = getOldContact(contact);
			if(oldContact != null) { 
				
				double oldLamda_n = oldContact.lambda.x;
				double oldLamda_t = oldContact.lambda.y;

				lambdas.set(contact.index*2, oldLamda_n);
				lambdas.set(contact.index*2 + 1, oldLamda_t);
				
				updateVelocity(contact.index*2, oldLamda_n);
				updateVelocity(contact.index*2 + 1, oldLamda_t);
				
				contact.body1ContactForceHistory.clear();
				contact.body1ContactTorqueHistory.clear();
				contact.body2ContactForceHistory.clear();
				contact.body2ContactTorqueHistory.clear();
				contact.body1ContactForceHistory.addAll(oldContact.body1ContactForceHistory);
				contact.body1ContactTorqueHistory.addAll(oldContact.body1ContactTorqueHistory);
				contact.body2ContactForceHistory.addAll(oldContact.body2ContactForceHistory);
				contact.body2ContactTorqueHistory.addAll(oldContact.body2ContactTorqueHistory);
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
			
			if (oldBody1 != body1 || oldBody2 != body2) 
				return null;
		}
		else
			return null;
		
		return oldContact;
	}
	
	/**
	 * Warm start if the confidence that each contact already has a value for lambda
	 */
	protected void confidentWarmStart() {
		int index = 0;
		for (Contact contact : contacts) {
			lambdas.set(index, contact.lambda.x);
			lambdas.set(index+1, contact.lambda.y);
			
			updateVelocity(contact.index*2, contact.lambda.x);
			updateVelocity(contact.index*2 + 1, contact.lambda.y);
			
			index+=2;
		}
	}
}
