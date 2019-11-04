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
	public Vector2d getRelativeLinearVelocity(RigidBody body1, RigidBody body2) {
		
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
	
		Point2d COM = getCommonCOM(body1, body2);
	
		relativeLinearVelocity.sub(body2.v, body1.v);

		Vector2d tmp = new Vector2d();
		Vector2d tmp2 = new Vector2d();
		
		tmp.sub( COM, body2.x );
		tmp.scale( body2.omega );
		tmp2.set( -tmp.y, tmp.x );
		relativeLinearVelocity.add( tmp2 );
		
		tmp.sub( COM, body1.x );
		tmp.scale( body1.omega );
		tmp2.set( -tmp.y, tmp.x );
		relativeLinearVelocity.sub( tmp2 );
		
		return relativeLinearVelocity;
	}
	
	/**
	 * Compute the relative linear velocity
	 * @param body1
	 * @param body2
	 * @return relative linear velocity
	 */
	public double getLargestVelocity(RigidBody body1, RigidBody body2) {
		
		Point2d COM = getCommonCOM(body1, body2);
		double largestVelocity = getLargestVelocity(body1, COM);
		largestVelocity = Math.max(largestVelocity, getLargestVelocity(body2, COM));
		
		return largestVelocity;
	}
	
	/**
	 * Compute the relative linear velocity
	 * @param body1
	 * @param body2
	 * @return relative linear velocity
	 */
	public double getLargestVelocity(RigidBody body, Point2d COM) {
		double largestVelocity = -Double.MAX_VALUE;
		
		for (Point2d point : body.boundingBoxB) {
			final Vector2d rw = new Vector2d( -(point.y - COM.y), point.x - COM.x );
			rw.scale( body.omega );
			largestVelocity = Math.max(largestVelocity, Math.sqrt(rw.lengthSquared()));
		}
		
		return largestVelocity;
	}
	
	protected Point2d getCommonCOM(RigidBody body1, RigidBody body2) {
		Point2d massCOM1 = new Point2d(body1.x);
		Point2d massCOM2 = new Point2d(body2.x);
		massCOM1.scale(body1.massLinear);
		massCOM2.scale(body2.massLinear);
		Point2d newCOM = new Point2d();
		newCOM.add( massCOM1, massCOM2 );
		newCOM.scale( 1./(body1.massLinear + body2.massLinear) );
		return newCOM;
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
	
	public double getRelativeVelocitiesMetric(Vector2d relativeLinearVelocity, double relativeAngularVelocity) {
		double k = 0.5*relativeLinearVelocity.lengthSquared() + 0.5*relativeAngularVelocity*relativeAngularVelocity;
		return k;
	}
	
	public double getRelativeKineticEnergy(RigidBody body1, RigidBody body2, Vector2d relativeLinearVelocity, double relativeAngularVelocity) {
		double massDifference = Math.abs(body1.massLinear - body2.massLinear);
		double inertiaDifference = Math.abs(body1.massAngular - body2.massAngular);
		double k = 0.5*relativeLinearVelocity.lengthSquared()*massDifference+ 0.5*relativeAngularVelocity*relativeAngularVelocity*inertiaDifference;
		
		return k/massDifference;
	}
}
