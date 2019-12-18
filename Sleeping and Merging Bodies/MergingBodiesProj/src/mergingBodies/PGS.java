package mergingBodies;

import java.util.ArrayList;

import no.uib.cipr.matrix.DenseVector;

/**
 * Projected Gauss Seidel contact solver.
 * 
 * @author 
 */
public class PGS {
	
	public PGS() {
		init(200);
	}
	
	public PGS(int iterations) {
		init(iterations);
	}
	
	public void init(int iterations) {
		this.iterations=iterations;
		feedbackStiffness=0.;
		compliance=0.;
		contacts = null;
		warmStart = false;
		computeInCollection = false;
	}
	
	/** List of contacts to solve */
	public ArrayList<Contact> contacts = null;
	
	/** Number of iterations */
	public int iterations;
	
	/** Feedback stiffness coefficient*/
	public double feedbackStiffness;
	
	/** Compliance */
	public double compliance;
	
	/** Special case for contacts in collections */
	public boolean computeInCollection;
	
	/** Warm start option */
	protected boolean warmStart;
	
	public void disableWarmStart(){
		warmStart = false;
	}
	
	/**
	 * Solve contact problem
	 * @param dt time step
	 */
	public void solve(double dt) {
				
		if (contacts == null) {
			System.err.println("PGS.solve() method needs the list PGS.contacts to be filled.");
			return;
		}
		
		if (warmStart)
			confidentWarmStart();

		for (Contact contact: contacts) {
			contact.computeB(dt, feedbackStiffness, computeInCollection);
			contact.computeJMinvJtDiagonal(computeInCollection);
		}
		
		int iter = iterations;
		while(iter > 0) {
			
			for (int i=0; i<contacts.size(); i++) {
				
				Contact contact = contacts.get(i);
					
				double Jdvn = contact.getJdvn(computeInCollection);
				double prevLambda_n = contact.lambda.x;
				contact.lambda.x = (contact.lambda.x*contact.diin - contact.bn - Jdvn)/(contact.diin+compliance);
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active				
				if ((!contact.body1.magneticBody || !contact.body1.activateMagnet) && (!contact.body2.magneticBody || !contact.body2.activateMagnet)) 
					contact.lambda.x = Math.max(0., contact.lambda.x);
				
				double dLambda_n = contact.lambda.x - prevLambda_n;
				
				//update delta V;
				updateDeltaV(contact, dLambda_n, 0.);
				
				double Jdvt = contact.getJdvt(computeInCollection);
				double prevLambda_t = contact.lambda.y;
				contact.lambda.y = (contact.lambda.y*contact.diit - contact.bt - Jdvt)/(contact.diit+compliance);
				
				double mu = 0.;
				
				// eulalie : we should assign material property to bodies and have a table for the corresponding friction coefficient...
				if (contact.body1.friction<0.2 || contact.body2.friction<0.2) 
					mu = Math.min(contact.body1.friction, contact.body2.friction);
				else if (contact.body1.friction>1. || contact.body2.friction>1.) 
					mu = Math.max(contact.body1.friction, contact.body2.friction);
				else
					mu = (contact.body1.friction + contact.body2.friction)/2.;
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magneticBody || !contact.body1.activateMagnet) && (!contact.body2.magneticBody || !contact.body2.activateMagnet)) {
					contact.lambda.y = Math.max(contact.lambda.y, -mu*contact.lambda.x);
					contact.lambda.y = Math.min(contact.lambda.y, mu*contact.lambda.x);
				}
				
				double dLambda_t = contact.lambda.y - prevLambda_t;
				
				//update delta V;
				updateDeltaV(contact, 0., dLambda_t);
				
				if (iter == 1)
					contact.updateContactState(mu);
			}
			
			iter--;
		}
	}
	
	/**
	 * Update deltaV of bodies in contact w.r.t new value of dLambda
	 * @param index
	 * @param lambda
	 */
	protected void updateDeltaV(Contact contact, double lambda_n, double lambda_t) {
		RigidBody body1 = (contact.body1.isInCollection() && !computeInCollection)? contact.body1.parent: contact.body1;
		RigidBody body2 = (contact.body2.isInCollection() && !computeInCollection)? contact.body2.parent: contact.body2;

		double m1inv = (body1.temporarilyPinned)? 0: body1.minv; 
		double m2inv = (body2.temporarilyPinned)? 0: body2.minv;
		double j1inv = (body1.temporarilyPinned)? 0: body1.jinv;
		double j2inv = (body2.temporarilyPinned)? 0: body2.jinv;
		
		DenseVector dv1 = body1.deltaV; 
		DenseVector dv2 = body2.deltaV;
		DenseVector jn;
		DenseVector jt;
		
		jn = (body1 instanceof RigidCollection)? contact.jnc: contact.jn;
		jt = (body1 instanceof RigidCollection)? contact.jtc: contact.jt;
		
		dv1.add( 0, jn.get(0) * m1inv * lambda_n );
		dv1.add( 1, jn.get(1) * m1inv * lambda_n );
		dv1.add( 2, jn.get(2) * j1inv * lambda_n );		
		dv1.add( 0, jt.get(0) * m1inv * lambda_t );
		dv1.add( 1, jt.get(1) * m1inv * lambda_t );
		dv1.add( 2, jt.get(2) * j1inv * lambda_t );
		
		jn = (body2 instanceof RigidCollection)? contact.jnc: contact.jn;
		jt = (body2 instanceof RigidCollection)? contact.jtc: contact.jt;
		
		dv2.add( 0, jn.get(3) * m2inv * lambda_n );
		dv2.add( 1, jn.get(4) * m2inv * lambda_n );
		dv2.add( 2, jn.get(5) * j2inv * lambda_n );
		dv2.add( 0, jt.get(3) * m2inv * lambda_t );
		dv2.add( 1, jt.get(4) * m2inv * lambda_t );
		dv2.add( 2, jt.get(5) * j2inv * lambda_t );
	}
	
	/**
	 * Warm start if the confidence that each contact already has a value for lambda
	 */
	protected void confidentWarmStart() {
		for (Contact contact : contacts) {		
			updateDeltaV(contact, contact.lambda.x, contact.lambda.y);	
		}
	}
}
