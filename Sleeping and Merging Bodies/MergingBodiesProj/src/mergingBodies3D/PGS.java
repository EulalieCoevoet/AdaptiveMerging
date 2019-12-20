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
					
				// Normal direction
				double Jdvn = contact.getJdv(computeInCollection,0);
				double prevLambda_n = contact.lambda.get(0);
				double Arow = contact.lambda.get(0)*contact.D.get(0,0)+ contact.lambda.get(1)*contact.D.get(0,1)+ contact.lambda.get(2)*contact.D.get(0,2);
				contact.lambda.set(0, (Arow - contact.bn - Jdvn)/(contact.D.get(0,0)+compliance));
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active				
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) 
					contact.lambda.set(0, Math.max(0., contact.lambda.get(0)));
				
				l.zero();
				l.set(0, contact.lambda.get(0) - prevLambda_n);
				updateDeltaV(contact, l);
				
				// Tangential directions
				double mu = 0.;
				
				// eulalie : we should assign material property to bodies and have a table for the corresponding friction coefficient...
				if (contact.body1.friction<0.2 || contact.body2.friction<0.2) 
					mu = Math.min(contact.body1.friction, contact.body2.friction);
				else if (contact.body1.friction>1. || contact.body2.friction>1.) 
					mu = Math.max(contact.body1.friction, contact.body2.friction);
				else
					mu = (contact.body1.friction + contact.body2.friction)/2.;
				
				// Tangential1 direction
				double Jdvt1 = contact.getJdv(computeInCollection,1);
				double prevLambda_t1 = contact.lambda.get(1);
				Arow = contact.lambda.get(0)*contact.D.get(1,0)+ contact.lambda.get(1)*contact.D.get(1,1)+ contact.lambda.get(2)*contact.D.get(1,2);
				contact.lambda.set(1, (Arow - contact.bt1 - Jdvt1)/(contact.D.get(1,1)+compliance));
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) {
					contact.lambda.set(1, Math.max(contact.lambda.get(1), -mu*contact.lambda.get(0)));
					contact.lambda.set(1, Math.min(contact.lambda.get(1), mu*contact.lambda.get(0)));
				}
				
				l.zero();
				l.set(1, contact.lambda.get(1) - prevLambda_t1);
				updateDeltaV(contact, l);
				
				// Tangential2 direction
				double Jdvt2 = contact.getJdv(computeInCollection,2);
				double prevLambda_t2 = contact.lambda.get(2);
				Arow = contact.lambda.get(0)*contact.D.get(2,0)+ contact.lambda.get(1)*contact.D.get(2,1)+ contact.lambda.get(2)*contact.D.get(2,2);
				contact.lambda.set(2, (Arow - contact.bt2 - Jdvt2)/(contact.D.get(2,2)+compliance));
				
				//only clamp lambdas if both bodies aren't magnetic or both bodies are magnetic but the magnet isn't active
				if ((!contact.body1.magnetic || !contact.body1.activateMagnet) && (!contact.body2.magnetic || !contact.body2.activateMagnet)) {
					contact.lambda.set(2, Math.max(contact.lambda.get(2), -mu*contact.lambda.get(0)));
					contact.lambda.set(2, Math.min(contact.lambda.get(2), mu*contact.lambda.get(0)));
				}

				l.zero();
				l.set(2, contact.lambda.get(2) - prevLambda_t2);
				updateDeltaV(contact, l);
				
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
		DenseVector jlambda = new DenseVector(12);
		
		j = contact.j; //(body1 instanceof RigidCollection)? contact.jc: contact.j;
		j.transMult(lambda, jlambda);
		
		dv1.add( 0, jlambda.get(0) * m1inv);	
		dv1.add( 1, jlambda.get(1) * m1inv);	
		dv1.add( 2, jlambda.get(2) * m1inv);
		
		dv1.add( 3, jlambda.get(3) * j1inv.m00 + jlambda.get(4) * j1inv.m01 + jlambda.get(5) * j1inv.m02);
		dv1.add( 4, jlambda.get(3) * j1inv.m10 + jlambda.get(4) * j1inv.m11 + jlambda.get(5) * j1inv.m12);	
		dv1.add( 5, jlambda.get(3) * j1inv.m20 + jlambda.get(4) * j1inv.m21 + jlambda.get(5) * j1inv.m22);	
		
		j = contact.j; //(body2 instanceof RigidCollection)? contact.jc: contact.j;
		j.transMult(lambda, jlambda);
		
		dv2.add( 0, jlambda.get(6) * m2inv);	
		dv2.add( 1, jlambda.get(7) * m2inv);	
		dv2.add( 2, jlambda.get(8) * m2inv);
		
		dv2.add( 3, jlambda.get(9) * j2inv.m00 + jlambda.get(10) * j2inv.m01 + jlambda.get(11) * j2inv.m02);
		dv2.add( 4, jlambda.get(9) * j2inv.m10 + jlambda.get(10) * j2inv.m11 + jlambda.get(11) * j2inv.m12);	
		dv2.add( 5, jlambda.get(9) * j2inv.m20 + jlambda.get(10) * j2inv.m21 + jlambda.get(11) * j2inv.m22);	
		
	}
	
	/**
	 * Warm start if the confidence that each contact already has a value for lambda
	 */
	protected void confidentWarmStart() {
		for (Contact contact : contacts) {		
			updateDeltaV(contact, contact.lambda);	
		}
	}
}
