package src;

import java.util.ArrayList;

import javax.vecmath.Vector2d;


/**
 * This class holds info about two colliding bodies (list of contacts, relative velocity, if they are merged...).
 */
public class BodyContact {
	
	public RigidBody body1; 
	public RigidBody body2;
	
	public ArrayList<Contact> contactList = new ArrayList<Contact>();

	public ArrayList<Double> relativeVelHistory = new ArrayList<Double>();
	
	boolean updatedThisTimeStep = false;
	
	public ArrayList<Vector2d> body1ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body1ContactTorqueHistory = new ArrayList<Double>();
	
	public ArrayList<Vector2d> body2ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body2ContactTorqueHistory = new ArrayList<Double>();
	
	Vector2d body1ContactForce = new Vector2d();
	Vector2d body2ContactForce = new Vector2d();
	
	double body1ContactTorque = 0;
	double body2ContactTorque = 0;
	
	boolean merged = false;
	
	public BodyContact(RigidBody body1, RigidBody body2) {
		this.body1 = body1;
		this.body2 = body2;
		//updatedThisTimeStep = true;
	}

	/**
	 * Check if two bodies are in a list of BodyContact.
	 * @param body1
	 * @param body2
	 * @param list
	 * @return the corresponding BodyContact object
	 */
	public static BodyContact checkExists(RigidBody body1, RigidBody body2, ArrayList<BodyContact> list) {
		for (BodyContact c: list) {
			if ((c.body1.equals(body1) && c.body2.equals(body2))
				|| (c.body1.equals(body2) && c.body2.equals(body1))) {
				return c;
			}
		}
		return null;
	}

	/**
	 * Check if a BodyContact is in a list of BodyContact
	 * @param bc
	 * @param list
	 * @return the corresponding BodyContact object
	 */
	public BodyContact alreadyExists(BodyContact bc, ArrayList<BodyContact> list) {
		//returns the BodyContact in the list, if it exists already
		//otherwise returns null
		for (BodyContact c : list) {
			if (c.body1.equals(bc.body1) && c.body2.equals(bc.body2)){
				return c;
			}
		}
		return null;
	}

	/**
	 * Check if BodyContact is in a list of BodyContact
	 * @param bodyContactList
	 * @return true of false
	 */
	public boolean isIn(ArrayList<BodyContact> bodyContactList) {
		
		for (BodyContact c : bodyContactList) {
			if (c.body2.equals(this.body2)){ // eulalie: why only testing body2?
				return true;
			}
		}
		return false;
	}
	
	public void clearForces() {
		body1ContactForce.set(0, 0);
		body2ContactForce.set(0, 0);
		body1ContactTorque = 0;
		body2ContactTorque = 0;
	}
	
	/**
	 * Given a RigidBody sB in a contact, return the same RigidBody in the BodyContact
	 * @param sB
	 * @return
	 */
	public RigidBody getThisBody(RigidBody sB) {
		// TODO Auto-generated method stub
		if (body1 == sB) return body1;
		if (body2 == sB) return body2;
		return null;
	}
	
	/**
	 * Given a RigidBody sB in a BodyContact, return the adjacent RigidBody in the BodyContact
	 * @param sB
	 * @return
	 */
	public RigidBody getOtherBody(RigidBody sB) {
		// TODO Auto-generated method stub
		if (body1 == sB) return body2;
		if (body2 == sB) return body1;
		return null;
	}

	public RigidBody getOtherSubBodyFromParent(RigidCollection body) {
		if (body.collectionBodies.contains(body1)) return body2;
		if (body.collectionBodies.contains(body2)) return body1;
	    return null;
	}
	
	public RigidBody getThisSubBodyFromParent(RigidCollection body) {
		if (body1.parent == body) return body1;
		if (body2.parent == body) return body2;
		return null;
	}

	/**
	 * Add the BodyContact to bodyContactList of members body1 and body2  
	 */
	public void addToBodyLists() {
		if (!body1.bodyContactList.contains(this)) {
			body1.bodyContactList.add(this);
		}
		if (!body2.bodyContactList.contains(this)) {
			body2.bodyContactList.add(this);
		}
	}
	
	/**
	 * Remove the BodyContact from bodyContactList of members body1 and body2
	 */
	public void removeFromBodyLists() {
		body1.bodyContactList.remove(this);
		body2.bodyContactList.remove(this);
	}
}
