package mergingBodies3D;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;

import no.uib.cipr.matrix.DenseMatrix;
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
		} else if (contacts.isEmpty()) {
			return;
		}
		
		if (warmStart)
			confidentWarmStart();

		for (Contact contact: contacts) {
			contact.computeB(dt, feedbackStiffness, computeInCollection);
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
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active				
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) 
					contact.lambda0 = Math.max( 0, contact.lambda0 );
				
				updateDeltaVwithLambdai(contact, contact.lambda0 - prevLambda_n, 0);
				
				// Tangential directions
				double mu = 0.;
				
				// TODO: eulalie : we should assign material property to bodies and have a table for the corresponding friction coefficient...
				if (contact.body1.friction<0.2 || contact.body2.friction<0.2) 
					mu = Math.min(contact.body1.friction, contact.body2.friction);
				else if (contact.body1.friction>1. || contact.body2.friction>1.) 
					mu = Math.max(contact.body1.friction, contact.body2.friction);
				else
					mu = (contact.body1.friction + contact.body2.friction)/2.;				
				
				// Tangential1 direction
				double Jdvt1 = contact.getJdv(computeInCollection,1);
				double prevLambda_t1 = contact.lambda1;
				contact.lambda1 = (contact.D11*contact.lambda1 - contact.bt1 - Jdvt1)/(contact.D11+compliance);
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) {
					contact.lambda1 = Math.max(contact.lambda1, -mu*contact.lambda0);
					contact.lambda1 = Math.min(contact.lambda1, mu*contact.lambda0);
				}
				
				updateDeltaVwithLambdai(contact, contact.lambda1 - prevLambda_t1, 1);

				
				// Tangential2 direction
				double Jdvt2 = contact.getJdv(computeInCollection,2);
				double prevLambda_t2 = contact.lambda2;
				contact.lambda2 = (contact.D22*contact.lambda2 - contact.bt2 - Jdvt2)/(contact.D22+compliance);
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) {
					contact.lambda2 = Math.max(contact.lambda2, -mu*contact.lambda0);
					contact.lambda2 = Math.min(contact.lambda2, mu*contact.lambda0);
				}

				updateDeltaVwithLambdai(contact, contact.lambda2 - prevLambda_t2, 2);

				
				if (iter == 1) // TODO: weird way to do a post process?
					contact.updateContactState(mu);
			}
			
			iter--;
		}
	}
	
	/**
	 * Update deltaV of bodies in contact w.r.t new value of dLambda
	 */
	private void updateDeltaVwithLambdai(Contact contact, double lambda, int i ) {
		RigidBody body1 = contact.body1; //(contact.body1.isInCollection() && !computeInCollection)? contact.body1.parent: contact.body1;
		RigidBody body2 = contact.body2; //(contact.body2.isInCollection() && !computeInCollection)? contact.body2.parent: contact.body2;
		double m1inv = body1.minv; //(body1.temporarilyPinned)? 0: body1.minv; 
		double m2inv = body2.minv; //(body2.temporarilyPinned)? 0: body2.minv;
		Matrix3d j1inv = body1.jinv; //(body1.temporarilyPinned)? 0: body1.jinv;
		Matrix3d j2inv = body2.jinv; //(body2.temporarilyPinned)? 0: body2.jinv;
		DenseVector dv1 = body1.deltaV; 
		DenseVector dv2 = body2.deltaV;
		
		DenseMatrix j = contact.j; //(body1 instanceof RigidCollection)? contact.jc: contact.j;
		double jTli0 = j.get(i,0) * lambda;
		double jTli1 = j.get(i,1) * lambda;
		double jTli2 = j.get(i,2) * lambda;
		double jTli3 = j.get(i,3) * lambda;
		double jTli4 = j.get(i,4) * lambda;
		double jTli5 = j.get(i,5) * lambda;
		dv1.add( 0, jTli0 * m1inv );	
		dv1.add( 1, jTli1 * m1inv );	
		dv1.add( 2, jTli2 * m1inv );
		dv1.add( 3, jTli3 * j1inv.m00 + jTli4 * j1inv.m01 + jTli5 * j1inv.m02);
		dv1.add( 4, jTli3 * j1inv.m10 + jTli4 * j1inv.m11 + jTli5 * j1inv.m12);	
		dv1.add( 5, jTli3 * j1inv.m20 + jTli4 * j1inv.m21 + jTli5 * j1inv.m22);	
		// j = contact.j; //(body2 instanceof RigidCollection)? contact.jc: contact.j;
		double jTli6 = j.get(i,6) * lambda;
		double jTli7 = j.get(i,7) * lambda;
		double jTli8 = j.get(i,8) * lambda;
		double jTli9 = j.get(i,9) * lambda;
		double jTli10 = j.get(i,10) * lambda;
		double jTli11 = j.get(i,11) * lambda;	
		dv2.add( 0, jTli6 * m2inv );	
		dv2.add( 1, jTli7 * m2inv );	
		dv2.add( 2, jTli8 * m2inv );		
		dv2.add( 3, jTli9 * j2inv.m00 + jTli10 * j2inv.m01 + jTli11 * j2inv.m02 );
		dv2.add( 4, jTli9 * j2inv.m10 + jTli10 * j2inv.m11 + jTli11 * j2inv.m12 );	
		dv2.add( 5, jTli9 * j2inv.m20 + jTli10 * j2inv.m21 + jTli11 * j2inv.m22 );	
	}
	
	/**
	 * Warm start if the confidence that each contact already has a value for lambda
	 */
	protected void confidentWarmStart() {
//		DenseVector l = new DenseVector(3);
		for (Contact contact : contacts) {	
//			l.set(0, contact.lambda0 );
//			l.set(1, contact.lambda1 );
//			l.set(2, contact.lambda2 );
//			updateDeltaV( contact, l );	// TODO: update this to 3 calls to update each individually...
			updateDeltaVwithLambdai( contact, contact.lambda0, 0 );
			updateDeltaVwithLambdai( contact, contact.lambda1, 1 );
			updateDeltaVwithLambdai( contact, contact.lambda2, 2 );
		}
	}
}
