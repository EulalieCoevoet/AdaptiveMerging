package mergingBodies;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Util class to hold an mouse impulse 
 *
 */
public class Impulse {

	RigidBody pickedBody = new RigidBody();
	Point2d pickedPoint = new Point2d();
	Vector2d force = new Vector2d();
	boolean holdingForce = false;
	
	public boolean isHoldingForce() {
		return holdingForce;
	}
	
	public void set(RigidBody pickedBody, Point2d pickedPoint, Vector2d force){
		this.pickedBody = pickedBody;
		this.pickedPoint.set(pickedPoint);
		this.force.set(force);
	}
	
	public void set(RigidBody pickedBody){
		this.pickedBody = pickedBody;
	}
	
	public void set(Point2d pickedPoint){
		this.pickedPoint.set(pickedPoint);
	}
	
	public void set(Vector2d force){
		
		if(pickedBody == null) {
			clear();
			return;
		}

		this.force.set(force);
		holdingForce = true;
	}
	
	public void clear(){
		pickedBody = null;
		pickedPoint.set(0.,0.);
		force.set(0.,0.);
		holdingForce = false;
	}
}
