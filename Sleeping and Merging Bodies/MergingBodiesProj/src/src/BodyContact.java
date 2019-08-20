package src;

import java.util.ArrayList;

import javax.vecmath.Vector2d;

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

	public static BodyContact checkExists(RigidBody body1, RigidBody body2, ArrayList<BodyContact> list) {
		for (BodyContact c: list) {
			if ((c.body1.equals(body1) && c.body2.equals(body2))
				|| (c.body1.equals(body2) && c.body2.equals(body1))) {
				return c;
			}
		}
		return null;
	}

	
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



	public boolean isIn(ArrayList<BodyContact> body_contact_list) {
		
		for (BodyContact c : body_contact_list) {
			if (c.body2.equals(this.body2)){
				return true;
			}
			
		}
		return false;
		
	}
	

	public void clearForces() {
		body1ContactForce.set(0, 0);
		body2ContactForce.set(0, 0);
		body1ContactTorque =0 ;
		body2ContactTorque = 0;

	}
	
	/* 
	 * given a body sB in a contact, return the same body in the body contact
	 */

	public RigidBody getThisBody(RigidBody sB) {
		// TODO Auto-generated method stub
		if (body1 == sB) return body1;
		else if (body2 == sB) return body2;
		else return null;
	
	}
	
	/* 
	 * given a body sB in a body contact, return the adjacent body in the body contact
	 */

	public RigidBody getOtherBody(RigidBody sB) {
		// TODO Auto-generated method stub
		if (body1 == sB) return body2;
		else if (body2 == sB) return body1;
		else {
		return null;
		}	
	}

	public void clearFromBodies() {
		body1.bodyContactList.remove(this);
		body2.bodyContactList.remove(this);
	}

	public RigidBody getOtherSubBodyFromParent(RigidCollection body) {
		if (body.collectionBodies.contains(body1)) return body2;
		else if (body.collectionBodies.contains(body2)) return body1;
		else {
			return null;
		}
	}
	
	public RigidBody getThisSubBodyFromParent(RigidCollection body) {
		if (body1.parent == body) return body1;
		if (body2.parent == body) return body2;
		return null;
	}
	
	

}
