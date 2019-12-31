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

		DenseVector l = new DenseVector(3);
		int iter = iterations;
		while(iter > 0) {
			
			for (int i=0; i<contacts.size(); i++) {
				
				Contact contact = contacts.get(i);
					
				// TODO: SPEED: could avoid all the MTJ bounds checking by simply having 3 lambdas and 3 D member variables and access them directly
				// i.e., not even a java array (which also does bounds checking)! 
				
				// Normal direction
				double Jdvn = contact.getJdv(computeInCollection,0);
				double prevLambda_n = contact.lambda0;
				contact.lambda0 = (contact.D00 * contact.lambda0 - contact.bn - Jdvn)/(contact.D00+compliance);
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active				
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) 
					contact.lambda0 = Math.max( 0, contact.lambda0 );
				
				l.zero();
				l.set(0, contact.lambda0 - prevLambda_n);
				// TODO: SLOW!!: should only update the 12 components of delta v based on what just 
				// got updated... this approach is very slow!
				updateDeltaV(contact, l);
				
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
				
				l.zero();
				l.set(1, contact.lambda1 - prevLambda_t1);
				updateDeltaV(contact, l);
				
				// Tangential2 direction
				double Jdvt2 = contact.getJdv(computeInCollection,2);
				double prevLambda_t2 = contact.lambda2;
				contact.lambda2 = (contact.D22*contact.lambda2 - contact.bt2 - Jdvt2)/(contact.D22+compliance);
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) {
					contact.lambda2 = Math.max(contact.lambda2, -mu*contact.lambda0);
					contact.lambda2 = Math.min(contact.lambda2, mu*contact.lambda0);
				}

				l.zero();
				l.set(2, contact.lambda2 - prevLambda_t2);
				updateDeltaV(contact, l);
				
				if (iter == 1) // TODO: weird way to do a post process?
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
	protected void updateDeltaV(Contact contact, DenseVector lambda) {
		RigidBody body1 = contact.body1; //(contact.body1.isInCollection() && !computeInCollection)? contact.body1.parent: contact.body1;
		RigidBody body2 = contact.body2; //(contact.body2.isInCollection() && !computeInCollection)? contact.body2.parent: contact.body2;

		double m1inv = body1.minv; //(body1.temporarilyPinned)? 0: body1.minv; 
		double m2inv = body2.minv; //(body2.temporarilyPinned)? 0: body2.minv;
		Matrix3d j1inv = body1.jinv; //(body1.temporarilyPinned)? 0: body1.jinv;
		Matrix3d j2inv = body2.jinv; //(body2.temporarilyPinned)? 0: body2.jinv;
		
		DenseVector dv1 = body1.deltaV; 
		DenseVector dv2 = body2.deltaV;
		DenseMatrix j;
		DenseVector JTlambda = new DenseVector(12);  // TODO: MEMORY: OUCH!
		
		//JTlambda.zero();
		// TODO: watchout... if JTlambda becomes a working variable, then it can 
		// get polluted with NAN as transMult doesn't do exactly what you might think!
		j = contact.j; //(body1 instanceof RigidCollection)? contact.jc: contact.j;
		j.transMult(lambda, JTlambda);
		
		dv1.add( 0, JTlambda.get(0) * m1inv);	
		dv1.add( 1, JTlambda.get(1) * m1inv);	
		dv1.add( 2, JTlambda.get(2) * m1inv);
		
		dv1.add( 3, JTlambda.get(3) * j1inv.m00 + JTlambda.get(4) * j1inv.m01 + JTlambda.get(5) * j1inv.m02);
		dv1.add( 4, JTlambda.get(3) * j1inv.m10 + JTlambda.get(4) * j1inv.m11 + JTlambda.get(5) * j1inv.m12);	
		dv1.add( 5, JTlambda.get(3) * j1inv.m20 + JTlambda.get(4) * j1inv.m21 + JTlambda.get(5) * j1inv.m22);	
		
		// TODO: WHY IS THIS DONE TWICE?  it is the same computation :/
//		j = contact.j; //(body2 instanceof RigidCollection)? contact.jc: contact.j;
//		j.transMult(lambda, JTlambda);
		
		dv2.add( 0, JTlambda.get(6) * m2inv);	
		dv2.add( 1, JTlambda.get(7) * m2inv);	
		dv2.add( 2, JTlambda.get(8) * m2inv);
		
		dv2.add( 3, JTlambda.get(9) * j2inv.m00 + JTlambda.get(10) * j2inv.m01 + JTlambda.get(11) * j2inv.m02);
		dv2.add( 4, JTlambda.get(9) * j2inv.m10 + JTlambda.get(10) * j2inv.m11 + JTlambda.get(11) * j2inv.m12);	
		dv2.add( 5, JTlambda.get(9) * j2inv.m20 + JTlambda.get(10) * j2inv.m21 + JTlambda.get(11) * j2inv.m22);	
		
	}
	
	/**
	 * Warm start if the confidence that each contact already has a value for lambda
	 */
	protected void confidentWarmStart() {
		DenseVector l = new DenseVector(3);
		for (Contact contact : contacts) {	
			l.set(0, contact.lambda0 );
			l.set(1, contact.lambda1 );
			l.set(2, contact.lambda2 );
			updateDeltaV( contact, l );	// TODO: update this to 3 calls to update each individually...
		}
	}
}
