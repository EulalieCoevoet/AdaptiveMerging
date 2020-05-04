package mergingBodies2D;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Util class to hold an mouse impulse 
 *
 */
public class Impulse {

	RigidBody pickedBody = new RigidBody();
	/** Picked point in body coordinate */
	Point2d pickedPointB = new Point2d();
	/** Picked point in world coordinate */
	Point2d pickedPointW = new Point2d();
	Vector2d force = new Vector2d();
	boolean holdingForce = false;
	
	public boolean isHoldingForce() {
		return holdingForce;
	}
	
	public void set(RigidBody pickedBody, Point2d pickedPointB, Vector2d force){
		this.pickedBody = pickedBody;
		this.pickedPointB.set(pickedPointB);
        pickedBody.transformB2W.transform(pickedPointB, pickedPointW);
		this.force.set(force);
	}
	
	public void set(RigidBody pickedBody){
		this.pickedBody = pickedBody;
	}
	
	public void set(Point2d pickedPoint){
		this.pickedPointB.set(pickedPoint);
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
		pickedPointB.set(0.,0.);
		pickedPointW.set(0.,0.);
		force.set(0.,0.);
		holdingForce = false;
	}
}
