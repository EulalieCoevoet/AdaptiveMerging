package mergingBodies3D;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Useful class to hold a mouse impulse (typically in case of merging event)
 * @author eulalie coevoet
 */
public class Impulse {

	RigidBody pickedBody = new RigidBody();
	/** Picked point in body coordinate */
	Point3d pickedPointB = new Point3d(); 
	/** Picked point in world coordinate */
	Point3d pickedPointW = new Point3d(); 
	Vector3d force = new Vector3d();
	boolean holdingForce = false;
	
	public boolean isHoldingForce() {
		return holdingForce;
	}
	
	public void set(RigidBody pickedBody, Point3d pickedPointB, Vector3d force){
		this.pickedBody = pickedBody;
		this.pickedPointB.set(pickedPointB);
		pickedBody.transformB2W.transform(pickedPointB,pickedPointW);
		this.force.set(force);
	}
	
	public void set(RigidBody pickedBody){
		this.pickedBody = pickedBody;
	}
	
	public void set(Point3d pickedPointB){
		this.pickedPointB.set(pickedPointB);
		pickedBody.transformB2W.transform(pickedPointB,pickedPointW);
	}
	
	public void set(Vector3d force){
		
		if(pickedBody == null) {
			clear();
			return;
		}

		this.force.set(force);
		holdingForce = true;
	}
	
	public void clear(){
		pickedBody = null;
		pickedPointB.set(0.,0.,0.);
		force.set(0.,0.,0.);
		holdingForce = false;
	}
}
