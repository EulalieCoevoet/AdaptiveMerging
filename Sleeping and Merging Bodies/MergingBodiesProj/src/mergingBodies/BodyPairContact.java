package mergingBodies;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import javafx.util.Pair;


/**
 * This class holds info about two colliding bodies (list of contacts, relative velocity, if they are merged...).
 */
public class BodyPairContact {
	
	public RigidBody body1; 
	public RigidBody body2;
	
	public ArrayList<Contact> contactList = new ArrayList<Contact>();

	Vector2d relativeLinearVelocity = new Vector2d();
	double relativeAngularVelocity = 0;
	
	public ArrayList<Double> relativeKineticEnergyHist = new ArrayList<Double>();
	public ArrayList<Pair<Integer, Double>> contactStateHist = new ArrayList<Pair<Integer, Double>>();
		
	public boolean inCollection = false;
	
	public BodyPairContact(RigidBody body1, RigidBody body2) {
		this.body1 = body1;
		this.body2 = body2;
	}

	/**
	 * Check if two bodies are in a list of BodyContact.
	 * @param body1
	 * @param body2
	 * @param bodyPairContacts
	 * @return the corresponding BodyContact object
	 */
	public static BodyPairContact checkExists(RigidBody body1, RigidBody body2, ArrayList<BodyPairContact> bodyPairContacts) {
		for (BodyPairContact bpc: bodyPairContacts) {
			if ((bpc.body1.equals(body1) && bpc.body2.equals(body2))
				|| (bpc.body1.equals(body2) && bpc.body2.equals(body1))) {
				return bpc;
			}
		}
		return null;
	}

	/**
	 * Check if BodyPairContact is in a list of BodyPairContacts
	 * @param bodyPairContacts
	 * @return true of false
	 */
	public boolean isIn(ArrayList<BodyPairContact> bodyPairContacts) {
		
		for (BodyPairContact c : bodyPairContacts) {
			if (c.body1.equals(this.body1) && c.body2.equals(this.body2)) { 
				return true;
			}
		}
		return false;
	}
	
	 /**
	 * Computes the relative velocity between the two bodies in contact.
	 * In case of body in a collection, use velocities of parent.
	 * NOTE MUST COMPARE IN THE SAME COORDINATE FRAME!  In this case, use the 
	 * combined mass center of mass frame.  Note that we migth want an effective 
	 * mass weighted relative velocity to measure relative velocity kinetic energy.
	 */
	protected void computeRelativeVelocity() {
		RigidBody body1 = (this.body1.isInCollection())? this.body1.parent: this.body1;
		RigidBody body2 = (this.body2.isInCollection())? this.body2.parent: this.body2;
		
		if ( body1.pinned ) {
			// ASSERT that body 1 velocity is zero for this to be correct
			// NOTE this will break if we ever have non zero velocities on pinned bodies!
			relativeLinearVelocity.set( body2.v );
			relativeAngularVelocity = body2.omega;
			return;
		} else if ( body2.pinned ) {
			// ASSERT that body 2 velocity is zero for this to be correct
			// NOTE this will break if we ever have non zero velocities on pinned bodies!
			relativeLinearVelocity.set( body1.v );
			relativeAngularVelocity = body1.omega;
			return;
		}

		Point2d massCom1 = new Point2d();
		Point2d massCom2 = new Point2d();
		massCom1.scale( body1.massLinear, body1.x );
		massCom2.scale( body2.massLinear, body2.x );		
		Point2d newCom = new Point2d();
		newCom.add( massCom1, massCom2 );
		newCom.scale( 1./(body1.massLinear + body2.massLinear) );
			
		relativeLinearVelocity.sub(body2.v, body1.v);

		Vector2d tmp = new Vector2d();
		Vector2d tmp2 = new Vector2d();
		
		tmp.sub( newCom, body2.x );
		tmp.scale( body2.omega );
		tmp2.set( -tmp.y, tmp.x );
		relativeLinearVelocity.add( tmp2 );
		
		tmp.sub( newCom, body1.x );
		tmp.scale( body1.omega );
		tmp2.set( -tmp.y, tmp.x );
		relativeLinearVelocity.sub( tmp2 );
		
		relativeAngularVelocity = body2.omega - body1.omega;
	}
	
	/**
	 * Computes and returns the relative kinetic energy without the mass
	 * @return metric
	 */
	public double getRelativeKineticEnergy() {
		double k = 0.5*relativeLinearVelocity.lengthSquared() + 0.5*relativeAngularVelocity*relativeAngularVelocity;
		return k;
	}
	
	/**
	 * Accumulate criteria for unmerge condition
	 * @param contact
	 */
	public void accumulate() {
		
		computeRelativeVelocity();
		
		relativeKineticEnergyHist.add(getRelativeKineticEnergy());
		if (relativeKineticEnergyHist.size() > CollisionProcessor.sleepAccum.getValue())
			relativeKineticEnergyHist.remove(0);
	
		double meanLambda_n = 0.;
		for (Contact contact : contactList) 
			meanLambda_n += contact.lambda.x;
		meanLambda_n /= contactList.size();
		Pair<Integer, Double> state = new Pair<Integer, Double>(contactList.size(), meanLambda_n);
		
		contactStateHist.add(state);
		if (contactStateHist.size() > CollisionProcessor.sleepAccum.getValue())
			contactStateHist.remove(0);
	}
	
	/**
	 * Check if:
	 * <p><ul> 
	 * <li> 1. the two bodies have been in contact for at least "sleepAccum" number of time steps
	 * <li> 2. the relative kinetic energy (without the mass) has been strictly decreasing 
	 * <li> 3. and lower than a threshold over CollisionProcessor.sleepAccum time steps.
	 * PGK: what does without the mass mean?
	 * </ul><p>
	 * @return true or false
	 */
	public boolean checkRelativeKineticEnergy() {

		double epsilon = 5e-4;
		double threshold = CollisionProcessor.sleepingThreshold.getValue();

		if ((relativeKineticEnergyHist.size() == CollisionProcessor.sleepAccum.getValue())) {
			double previousValue = 0; 
			double currentValue = 0;
			for (Double relativeEnergy : relativeKineticEnergyHist) {
				currentValue = relativeEnergy;
				if (relativeEnergy > threshold || currentValue > previousValue + epsilon ) 
					return false;
				previousValue = relativeEnergy;
			}
		} else {
			return false;
		}

		return true;
	}  

	/**
	 * Check if contacts have been stable over CollisionProcessor.sleepAccum time steps.
	 * eulalie : work in progress
	 * 			- missing class for cleaner access instead of using Pair
	 * 			- currently a simple test on the mean of lambda.x
	 * @return true or false
	 */
	public boolean areContactsStable() {

		if ((contactStateHist.size() == CollisionProcessor.sleepAccum.getValue())) {
			Pair<Integer, Double> previousState = contactStateHist.get(0);
			Pair<Integer, Double> currentState;
			for (Pair<Integer, Double> state : contactStateHist) {
				currentState = state;
				double epsilon = currentState.getValue()/4.;
				if (currentState.getKey() != previousState.getKey())
					return false;
				if (currentState.getValue() < previousState.getValue()-epsilon)
					return false;
				if (currentState.getValue() > previousState.getValue()+epsilon)
					return false;
				previousState = currentState;
			}
		} else {
			return false;
		}
		
		return true;
	}
	
	/**
	 * Check if it satisfies the force closure criteria: only bodies that share two
	 * contacts, or cycles formed by 3 bodies with one contact between each.
	 * @return true if the criteria is satisfied
	 */
	public boolean checkForceClosureCritera() {
		
		if (contactList.size()>2)
			return true;
		
		if (true /*checkForCycles()*/)
			return true;
		
		return false;
	}
	
	/**
	 * Given a RigidBody sB in a BodyPairContact, return the adjacent RigidBody in the BodyPairContact
	 * @param body
	 * @return
	 */
	public RigidBody getOtherBody(RigidBody body) {
		if (body1 == body) return body2;
		if (body2 == body) return body1;
		return null;
	}

	/**
	 * Add the BodyContact to bodyContactList of members body1 and body2  
	 */
	public void addToBodyLists() {
		if (!body1.bodyPairContactList.contains(this)) {
			body1.bodyPairContactList.add(this);
		}
		if (!body2.bodyPairContactList.contains(this)) {
			body2.bodyPairContactList.add(this);
		}
	}
	
	/**
	 * Remove the BodyContact from bodyContactList of members body1 and body2
	 */
	public void removeFromBodyLists() {
		body1.bodyPairContactList.remove(this);
		body2.bodyPairContactList.remove(this);
	}
}
