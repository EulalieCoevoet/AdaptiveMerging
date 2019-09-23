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
	
	public PGS(double mu, int iterations) {
		init(mu, iterations);
	}
	
	public void init(double mu, int iterations) {
		this.mu=mu;
		this.iterations=iterations;
		this.restitution=0.;
		this.feedbackStiffness=0.;
		contacts = null;
		warmStart = false;
		lastTimeStepMap = null;
		confidentWarmStart = false;
		computeInCollection = false;
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
	public boolean computeInCollection;
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
	
	/**
	 * Solve contact problem
	 * @param dt time step
	 */
	public void solve(double dt) {
				
		if (contacts == null) {
			System.out.println("Error: PGS.solve() method needs the list PGS.contacts to be filled.");
			return;
		}

		for (Contact contact: contacts) {
			contact.computeB(dt, restitution, feedbackStiffness, computeInCollection);
			contact.computeJMinvJtDiagonal(computeInCollection);
		}
		
		if (confidentWarmStart)
			confidentWarmStart();
		else if (warmStart && lastTimeStepMap != null) 
			warmStart();
		
		int iter = iterations;
		while(iter > 0) {
			
			for (int i=0; i<contacts.size(); i++) {
				
				Contact contact = contacts.get(i);
				double Jdvn = contact.getJdvn(computeInCollection);
				double Jdvt = contact.getJdvt(computeInCollection);
				
				double prevLambda_n = contact.lambda.x;
				double prevLambda_t = contact.lambda.y;
				contact.lambda.x -= (contact.bn + Jdvn)/contact.diin;
				contact.lambda.y -= (contact.bt + Jdvt)/contact.diit;
				
				contact.processContactConstraints(mu);
				if (computeInCollection)
					contact.updateContactState(prevLambda_n, mu);
				
				double dLambda_n = contact.lambda.x - prevLambda_n;
				double dLambda_t = contact.lambda.y - prevLambda_t;
				updateVelocity(contact, dLambda_n, dLambda_t);
			}
			
			iter--;
		}
	}
	
	/**
	 * Update velocity of bodies in contact w.r.t new value of dLambda
	 * @param index
	 * @param lambda
	 */
	protected void updateVelocity(Contact contact, double lambda_n, double lambda_t) {
		RigidBody body1 = (contact.body1.isInCollection() && !computeInCollection)? contact.body1.parent: contact.body1;
		RigidBody body2 = (contact.body2.isInCollection() && !computeInCollection)? contact.body2.parent: contact.body2;

		double m1inv = (body1.temporarilyPinned)? 0: body1.minv; 
		double m2inv = (body2.temporarilyPinned)? 0: body2.minv;
		double j1inv = (body1.temporarilyPinned)? 0: body1.jinv;
		double j2inv = (body2.temporarilyPinned)? 0: body2.jinv;
		
		//update delta V;
		DenseVector dv1 = body1.deltaV; 
		DenseVector dv2 = body2.deltaV;
		
		dv1.add( 0, contact.jn.get(0) * m1inv * lambda_n );
		dv1.add( 1, contact.jn.get(1) * m1inv * lambda_n );
		dv1.add( 2, contact.jn.get(2) * j1inv * lambda_n );
		dv2.add( 0, contact.jn.get(3) * m2inv * lambda_n );
		dv2.add( 1, contact.jn.get(4) * m2inv * lambda_n );
		dv2.add( 2, contact.jn.get(5) * j2inv * lambda_n );
		
		dv1.add( 0, contact.jt.get(0) * m1inv * lambda_t );
		dv1.add( 1, contact.jt.get(1) * m1inv * lambda_t );
		dv1.add( 2, contact.jt.get(2) * j1inv * lambda_t );
		dv2.add( 0, contact.jt.get(3) * m2inv * lambda_t );
		dv2.add( 1, contact.jt.get(4) * m2inv * lambda_t );
		dv2.add( 2, contact.jt.get(5) * j2inv * lambda_t );
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

				contact.lambda.x = oldLamda_n;
				contact.lambda.y = oldLamda_t;
				
				updateVelocity(contact, oldLamda_n, oldLamda_t);
				
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
		for (Contact contact : contacts) {						
			updateVelocity(contact, contact.lambda.x, contact.lambda.y);	
		}
	}
}
