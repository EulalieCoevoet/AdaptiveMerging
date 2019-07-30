package src;

import java.util.ArrayList;

import javax.vecmath.Vector2d;

public class BodyContact {
	
	public RigidBody thisBody; 
	
	public RigidBody otherBody;
	
	public ArrayList<Contact> contactList = new ArrayList<Contact>();

	public ArrayList<Double> relativeVelHistory = new ArrayList<Double>();
	
	boolean updatedThisTimeStep = false;
	
	boolean merged = false;
	
	Vector2d contactForceThisBody = new Vector2d();
	
	double contactTorqueThisBody = 0;
	
	Vector2d contactForceOtherBody = new Vector2d();
	
	double contactTorqueOtherBody = 0;
	
	public BodyContact(RigidBody thisBody, RigidBody otherBody) {
		this.thisBody = thisBody;
		this.otherBody = otherBody;
		//updatedThisTimeStep = true;
	}

	public static BodyContact checkExists(RigidBody body1, RigidBody body2, ArrayList<BodyContact> list) {
		for (BodyContact c: list) {
			if ((c.thisBody.equals(body1) && c.otherBody.equals(body2))
				|| (c.thisBody.equals(body2) && c.otherBody.equals(body1))) {
				return c;
			}
		}
		return null;
	}

	
	public BodyContact alreadyExists(BodyContact bc, ArrayList<BodyContact> list) {
		//returns the BodyContact in the list, if it exists already
		//otherwise returns null
		for (BodyContact c : list) {
			if (c.thisBody.equals(bc.thisBody) && c.otherBody.equals(bc.otherBody)){
				return c;
			}
			
		}
		return null;
		
		
	}



	public boolean isIn(ArrayList<BodyContact> body_contact_list) {
		
		for (BodyContact c : body_contact_list) {
			if (c.otherBody.equals(this.otherBody)){
				return true;
			}
			
		}
		return false;
		
	}
	

	public void clearForces() {
		contactForceOtherBody.set(0, 0);
		contactForceThisBody.set(0, 0);
		contactTorqueThisBody = 0;
		contactTorqueOtherBody = 0;
		
	}
	
	
	

}
