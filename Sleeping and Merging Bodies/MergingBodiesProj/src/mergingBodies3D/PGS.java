package mergingBodies3D;

import java.util.ArrayList;

import javax.vecmath.Vector3d;


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
	public void solve(double dt, boolean restitutionOverride, double restitutionOverrideVal, boolean frictionOverride, double frictionOverrideVal ) {
				
		if (contacts == null) {
			System.err.println("PGS.solve() method needs the list PGS.contacts to be filled.");
			return;
		} else if (contacts.isEmpty()) {
			return;
		}
		
		if (warmStart)
			confidentWarmStart();

		for (Contact contact: contacts) {
			contact.computeB(dt, feedbackStiffness, computeInCollection, restitutionOverride, restitutionOverrideVal );
			contact.computeJMinvJt(computeInCollection);
		}

		int iter = iterations;
		while(iter > 0) {
			
			for (int i=0; i<contacts.size(); i++) {
				
				Contact contact = contacts.get(i);
					
				// Normal direction
				double Jdvn = contact.getJdv(computeInCollection,0);
				double prevLambda_n = contact.lambda0;
				contact.lambda0 = (contact.D00 * contact.lambda0 - contact.bn - Jdvn)/(contact.D00+compliance);
				
				// only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active				
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) 
					contact.lambda0 = Math.max( 0, contact.lambda0 );
				
				updateDeltaVwithLambdai(contact, contact.lambda0 - prevLambda_n, 0);
				
				// Tangential directions
				double mu = 0.;
				
				// TODO: eulalie : we should assign material property to bodies and have a table for the corresponding friction coefficient...
				// Note: Collection as no friction, we only consider the friction coefficient of the merged body being in contact (which is correct)
				if ( frictionOverride ) {
					mu = frictionOverrideVal;
				} else {
					if (contact.body1.friction<0.2 || contact.body2.friction<0.2) 
						mu = Math.min(contact.body1.friction, contact.body2.friction);
					else if (contact.body1.friction>1. || contact.body2.friction>1.) 
						mu = Math.max(contact.body1.friction, contact.body2.friction);
					else
						mu = (contact.body1.friction + contact.body2.friction)/2.;				
				}
				
				// Tangential1 direction
				double Jdvt1 = contact.getJdv(computeInCollection,1);
				double prevLambda_t1 = contact.lambda1;
				contact.lambda1 = (contact.D11*contact.lambda1 - contact.bt1 - Jdvt1)/(contact.D11+compliance);
				
				// only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) {
					contact.lambda1 = Math.max(contact.lambda1, -mu*contact.lambda0);
					contact.lambda1 = Math.min(contact.lambda1, mu*contact.lambda0);
				}
				
				updateDeltaVwithLambdai(contact, contact.lambda1 - prevLambda_t1, 1);

				
				// Tangential2 direction
				double Jdvt2 = contact.getJdv(computeInCollection,2);
				double prevLambda_t2 = contact.lambda2;
				contact.lambda2 = (contact.D22*contact.lambda2 - contact.bt2 - Jdvt2)/(contact.D22+compliance);
				
				// only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) {
					contact.lambda2 = Math.max(contact.lambda2, -mu*contact.lambda0);
					contact.lambda2 = Math.min(contact.lambda2, mu*contact.lambda0);
				}

				updateDeltaVwithLambdai(contact, contact.lambda2 - prevLambda_t2, 2);
				
				if (iter == 1) // Last iteration: avoid looping again over contacts
					contact.updateContactState(mu, dt, computeInCollection);
			}
			
			iter--;
		}
	}
	
	/** temporary working variable */
	private Vector3d tmp = new Vector3d();
	
	/**
	 * Update deltaV of bodies in contact w.r.t new value of dLambda
	 */
	private void updateDeltaVwithLambdai(Contact contact, double lambda, int i ) {
		RigidBody body1 = (contact.body1.isInCollection() && !computeInCollection)? contact.body1.parent: contact.body1;
		RigidBody body2 = (contact.body2.isInCollection() && !computeInCollection)? contact.body2.parent: contact.body2;

		Vector6d dv1 = body1.deltaV; 
		Vector6d dv2 = body2.deltaV;
		
		Vector6d ja = contact.jna;
		Vector6d jb = contact.jnb;
		if ( i == 1 ) {
			ja = contact.jt1a;
			jb = contact.jt1b;
		} else if ( i == 2 ) {
			ja = contact.jt2a;
			jb = contact.jt2b;
		}

		dv1.v.scaleAdd( body1.minv*lambda, ja.v, dv1.v );
		body1.jinv.transform( ja.w, tmp );
		dv1.w.scaleAdd( lambda, tmp, dv1.w );
		
		dv2.v.scaleAdd( body2.minv*lambda, jb.v, dv2.v );
		body2.jinv.transform( jb.w, tmp );
		dv2.w.scaleAdd( lambda, tmp, dv2.w );
	}
	
	/**
	 * Warm start if the confidence that each contact already has a value for lambda
	 */
	protected void confidentWarmStart() {
		for (Contact contact : contacts) {	
			updateDeltaVwithLambdai( contact, contact.lambda0, 0 );
			updateDeltaVwithLambdai( contact, contact.lambda1, 1 );
			updateDeltaVwithLambdai( contact, contact.lambda2, 2 );
		}
	}
}
