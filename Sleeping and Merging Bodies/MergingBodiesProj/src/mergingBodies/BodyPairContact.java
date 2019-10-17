package mergingBodies;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mergingBodies.Contact.ContactState;

//import javafx.util.Pair;


/**
 * This class holds info about two colliding bodies (list of contacts, relative velocity, if they are merged...).
 */
public class BodyPairContact {
	
	public RigidBody body1; 
	public RigidBody body2;
	
	public ArrayList<Contact> contactList = new ArrayList<Contact>();

	Vector2d relativeLinearVelocity = new Vector2d();
	double relativeAngularVelocity = 0;
	
	public ArrayList<Double> relativeKineticEnergyMetricHist = new ArrayList<Double>();
	public ArrayList<Contact.ContactState> contactStateHist = new ArrayList<Contact.ContactState>();
		
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
		
		if ( body1.pinned || body1.temporarilyPinned ) {
			// ASSERT that body 1 velocity is zero for this to be correct
			// NOTE this will break if we ever have non zero velocities on pinned bodies!
			relativeLinearVelocity.set( body2.v );
			relativeAngularVelocity = body2.omega;
			return;
		} else if ( body2.pinned || body2.temporarilyPinned ) {
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
	 * Computes and returns the relative kinetic energy normalized by the mass
	 * @return metric
	 */
	public double getRelativeKineticEnergyMassNormalized() {
		double k = 0.5*relativeLinearVelocity.lengthSquared() + 0.5*relativeAngularVelocity*relativeAngularVelocity;
		return k;
	}
	
	/**
	 * Computes and returns the relative kinetic energy 
	 * @return metric
	 */
	public double getRelativeKineticEnergy() {
		double massDifference = Math.abs(body1.massLinear - body2.massLinear);
		double inertiaDifference = Math.abs(body1.massAngular - body2.massAngular);
		double k = 0.5*relativeLinearVelocity.lengthSquared()*massDifference+ 0.5*relativeAngularVelocity*relativeAngularVelocity*inertiaDifference;
		
		return k/massDifference;
	}
	
	/**
	 * Accumulate criteria for unmerge condition
	 * @param contact
	 */
	public void accumulate() {
		
		computeRelativeVelocity();
		if (RigidBodySystem.useMassNormKinEnergy.getValue())
			relativeKineticEnergyMetricHist.add(getRelativeKineticEnergyMassNormalized());
		else
			relativeKineticEnergyMetricHist.add(getRelativeKineticEnergy());
		if (relativeKineticEnergyMetricHist.size() > CollisionProcessor.sleepAccum.getValue())
			relativeKineticEnergyMetricHist.remove(0);
	
		Contact.ContactState state = Contact.ContactState.CLEAR;
		for (Contact contact : contactList) 
			if (contact.state == Contact.ContactState.ONEDGE)
				state = Contact.ContactState.ONEDGE;
		
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

		if ((relativeKineticEnergyMetricHist.size() == CollisionProcessor.sleepAccum.getValue())) {
			double previousValue = 0; 
			double currentValue = 0;
			for (Double relativeEnergy : relativeKineticEnergyMetricHist) {
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
	 * Check if contacts have been stable over CollisionProcessor.sleepAccum time steps 
	 * (i.e not on edge of friction cone during the entire time)
	 * @return true or false
	 */
	public boolean areContactsStable() {

		if ((contactStateHist.size() == CollisionProcessor.sleepAccum.getValue())) {
			for (Contact.ContactState state : contactStateHist) {
				if (state == Contact.ContactState.ONEDGE)
					return false;
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
		
		int nbActiveContact = 0;
		for (Contact contact : contactList)
			if (contact.state != ContactState.BROKEN)
				nbActiveContact += 1;
		
		if (nbActiveContact>1)
			return true;
		
		return body1.checkForCycles(0, body1, this);
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
	 * Given a RigidBody or RigidCollection sB in a BodyPairContact, return the adjacent RigidBody in the BodyPairContact
	 * @param body
	 * @return
	 */
	public RigidBody getOtherBodyFromCollectionPerspective(RigidBody body) {
		if (body.isInCollection()) 
			return (body1.isInSameCollection(body))? body2 : body1;
		else 
			return getOtherBody(body);
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
		
		if (body1.isInCollection() && !body1.parent.bodyPairContactList.contains(this)) {
			body1.parent.bodyPairContactList.add(this);
		}
		if (body2.isInCollection() && !body2.isInSameCollection(body1) && !body2.parent.bodyPairContactList.contains(this)) {
			body2.parent.bodyPairContactList.add(this);
		}
	}
	
	/**
	 * Remove the BodyContact from bodyContactList of members body1 and body2
	 */
	public void removeFromBodyLists() {
		body1.bodyPairContactList.remove(this);
		body2.bodyPairContactList.remove(this);
		
		if (body1.isInCollection()) {
			body1.parent.bodyPairContactList.remove(this);
		}
		if (body2.isInCollection() && !body2.isInSameCollection(body1)) {
			body2.parent.bodyPairContactList.remove(this);
		}
	}
}
