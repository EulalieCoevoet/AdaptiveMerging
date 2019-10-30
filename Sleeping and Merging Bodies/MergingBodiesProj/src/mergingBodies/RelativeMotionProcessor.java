package mergingBodies;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * This class is used for relative motion's calculations
 */
public class RelativeMotionProcessor {	
	
	/**
	 * Compute the relative linear velocity
	 * @param body1
	 * @param body2
	 * @return relative linear velocity
	 */
	public Vector2d getRelativeLinearVelocity(RigidBody body1, RigidBody body2, boolean fromBB) {
		
		if ( body1.pinned || body1.temporarilyPinned ) {
			if(body1.v.x != 0. || body1.v.y != 0.) {
				System.err.print("[getRelativeLinearVelocity] linear velocity of pinned body is not zero: ");
				System.err.println(body1.v);
			} else {
				return body2.v;
			}
		} else if ( body2.pinned || body2.temporarilyPinned ) {
			if(body2.v.x != 0. || body2.v.y != 0.) {
				System.err.print("[getRelativeLinearVelocity] linear velocity of pinned body is not zero: ");
				System.err.println(body2.v);
			} else {
				return body1.v;
			}
		}
		
		Vector2d relativeLinearVelocity = new Vector2d();
		
		Point2d x1;		
		Point2d x2;
		if (fromBB) {
			x1 = new Point2d(body1.bbmax);
			x2 = new Point2d(body2.bbmin);
		} else {
			x1 = new Point2d(body1.x);
			x2 = new Point2d(body2.x);
		}
		
		Point2d massCom1 = new Point2d(x1);
		Point2d massCom2 = new Point2d(x2);
		massCom1.scale( body1.massLinear);
		massCom2.scale( body2.massLinear);
		Point2d newCom = new Point2d();
		newCom.add( massCom1, massCom2 );
		newCom.scale( 1./(body1.massLinear + body2.massLinear) );
	
		if (fromBB) {
			Vector2d v1 = new Vector2d( -(x1.y - body1.x.y), x1.x - body1.x.x );
			v1.scale( body1.omega );
			v1.add(body1.v);
			Vector2d v2 = new Vector2d( -(x2.y - body2.x.y), x2.x - body2.x.x );
			v2.scale( body2.omega );
			v2.add(body2.v);
			relativeLinearVelocity.sub(v2, v1);
		} else {
			relativeLinearVelocity.sub(body2.v, body1.v);
		}

		Vector2d tmp = new Vector2d();
		Vector2d tmp2 = new Vector2d();
		
		tmp.sub( newCom, x2 );
		tmp.scale( body2.omega );
		tmp2.set( -tmp.y, tmp.x );
		relativeLinearVelocity.add( tmp2 );
		
		tmp.sub( newCom, x1 );
		tmp.scale( body1.omega );
		tmp2.set( -tmp.y, tmp.x );
		relativeLinearVelocity.sub( tmp2 );
		
		return relativeLinearVelocity;
	}

	/**
	 * Compute the relative angular velocity
	 * @param body1
	 * @param body2
	 * @return relative angular velocity
	 */
	public double getRelativeAngularVelocity(RigidBody body1, RigidBody body2) {
		
		if ( body1.pinned || body1.temporarilyPinned ) {
			if(body1.omega != 0.) {
				System.err.print("[getRelativeAngularVelocity] angular velocity of pinned body is not zero: ");
				System.err.println(body1.omega);
			} else {
				return body2.omega;
			}
		} else if ( body2.pinned || body2.temporarilyPinned ) {
			if(body2.omega != 0.) {
				System.err.print("[getRelativeAngularVelocity] angular velocity of pinned body is not zero: ");
				System.err.println(body2.omega);
			} else {
				return body1.omega;
			}
		}

		return body2.omega - body1.omega;
	}
	

	/**
	 * Computes and returns the relative kinetic energy normalized by the mass
	 * @return metric
	 */
	public double getRelativeKineticEnergyMassNormalized(Vector2d relativeLinearVelocity, double relativeAngularVelocity) {
		double k = 0.5*relativeLinearVelocity.lengthSquared() + 0.5*relativeAngularVelocity*relativeAngularVelocity;
		return k;
	}
	
	/**
	 * Computes and returns the relative kinetic energy 
	 * @return metric
	 */
	public double getRelativeKineticEnergy(RigidBody body1, RigidBody body2, Vector2d relativeLinearVelocity, double relativeAngularVelocity) {
		double massDifference = Math.abs(body1.massLinear - body2.massLinear);
		double inertiaDifference = Math.abs(body1.massAngular - body2.massAngular);
		double k = 0.5*relativeLinearVelocity.lengthSquared()*massDifference+ 0.5*relativeAngularVelocity*relativeAngularVelocity*inertiaDifference;
		
		return k/massDifference;
	}
}
