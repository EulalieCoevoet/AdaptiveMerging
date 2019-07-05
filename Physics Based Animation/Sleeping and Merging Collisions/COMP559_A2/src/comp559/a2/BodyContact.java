package comp559.a2;

import java.util.ArrayList;

import javax.vecmath.Vector2d;

public class BodyContact {
	
	public RigidBody otherBody;

	
	
	public ArrayList<Double> relativeVelHistory = new ArrayList<Double>();
	
	
	public BodyContact(RigidBody body) {
		otherBody = body;
	}
	
	public boolean alreadyExists(ArrayList<BodyContact> list) {
		//returns true if contact with the other body exists in this list.
		boolean count = false;
		for (BodyContact c : list) {
			if (c.otherBody.equals(this.otherBody)){
				count = true;
			}
			
		}
		return count;
		
	}
	

}
