package mergingBodies;

import java.util.ArrayList;

import javax.vecmath.Vector2d;

import javafx.util.Pair;


/**
 * This class holds info about two colliding bodies (list of contacts, relative velocity, if they are merged...).
 */
public class BodyPairContact {
	
	public RigidBody body1; 
	public RigidBody body2;
	
	public ArrayList<Contact> contactList = new ArrayList<Contact>();

	Vector2d relativeVelocity = new Vector2d();
	double relativeAngularVelocity = 0;
	
	public ArrayList<Double> relativeKineticEnergyHist = new ArrayList<Double>();
	public ArrayList<Pair<Integer, Double>> contactStateHist = new ArrayList<Pair<Integer, Double>>();
	
	boolean updatedThisTimeStep = false;
	
	boolean inCollection = false;
	
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
			if (c.body2.equals(this.body2)){ // eulalie: why only testing body2?
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Computes the relative velocity between the two bodies in contact.
	 * In case of body in a collection, use velocities of parent.
	 */
	protected void computeRelativeVelocity() {
		RigidBody body1 = (this.body1.isInCollection())? this.body1.parent: this.body1;
		RigidBody body2 = (this.body2.isInCollection())? this.body2.parent: this.body2;
		
		relativeVelocity.sub(body2.v, body1.v);
		relativeAngularVelocity = body2.omega - body1.omega;
	}
	
	/**
	 * Computes and returns the relative kinetic energy without the mass
	 * @return metric
	 */
	public double getRelativeKineticEnergy() {
		double k = 0.5*relativeVelocity.lengthSquared() + 0.5*relativeAngularVelocity*relativeAngularVelocity;
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
	
		double totalLambda_n = 0.;
		for (Contact contact : contactList) 
			totalLambda_n+=contact.lambda.x;
		Pair<Integer, Double> state = new Pair<Integer, Double>(contactList.size(), totalLambda_n);
		
		contactStateHist.add(state);
		if (contactStateHist.size() > CollisionProcessor.sleepAccum.getValue())
			contactStateHist.remove(0);
	}
	
	/**
	 * Check if relative kinetic energy (without the mass) has been strictly decreasing and lower that a threshold over CollisionProcessor.sleepAccum time steps.
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
	 * 			- currently a simple test on the sum of lambda.x
	 * @return true or false
	 */
	public boolean areContactsStable() {

		double epsilon = 1e-3;
		
		if ((contactStateHist.size() == CollisionProcessor.sleepAccum.getValue())) {
			Pair<Integer, Double> previousState = contactStateHist.get(0);
			Pair<Integer, Double> currentState;
			for (Pair<Integer, Double> state : contactStateHist) {
				currentState = state;
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
