package comp559.a2;

import java.util.ArrayList;

import javax.vecmath.Vector2d;

public class BodyContact {
	
	public RigidBody otherBody;

	public ArrayList<Double> relativeVelHistory = new ArrayList<Double>();
	
	boolean updatedThisTimeStep = false;
	
	public BodyContact(RigidBody body) {
		otherBody = body;
		//updatedThisTimeStep = true;
	}


	
	public BodyContact alreadyExists(ArrayList<BodyContact> list) {
		//returns the BodyContact in the list, if it exists already
		
	
		for (BodyContact c : list) {
			if (c.otherBody.equals(this.otherBody)){
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
	
	
	

}
