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
	protected Vector2d getRelativeLinearVelocity(RigidBody body1, RigidBody body2) {
		
		if ( body1.pinned || body1.temporarilyPinned ) {
			// ASSERT that body 1 velocity is zero for this to be correct
			// NOTE this will break if we ever have non zero velocities on pinned bodies!
			return body2.v;
		} else if ( body2.pinned || body2.temporarilyPinned ) {
			// ASSERT that body 2 velocity is zero for this to be correct
			// NOTE this will break if we ever have non zero velocities on pinned bodies!
			return body1.v;
		}

		Vector2d relativeLinearVelocity = new Vector2d();
		
		Point2d massCom1 = new Point2d();
		Point2d massCom2 = new Point2d();
		massCom1.scale( body1.massLinear, body1.x );
		massCom2.scale( body2.massLinear, body2.x );
		Point2d newCom = new Point2d();
		newCom.add( massCom1, massCom2 );
		newCom.scale( 1./(body1.massLinear + body2.massLinear) );
			
		relativeLinearVelocity.sub(body2.v, body1.v);

		Vector2d tmp = new Vector2d();
		Vector2d tmp2 = new Vector2d();
		
		tmp.sub( newCom, body2.x );
		tmp.scale( body2.omega );
		tmp2.set( -tmp.y, tmp.x );
		relativeLinearVelocity.add( tmp2 );
		
		tmp.sub( newCom, body1.x );
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
	protected double getRelativeAngularVelocity(RigidBody body1, RigidBody body2) {
		
		if ( body1.pinned || body1.temporarilyPinned ) {
			// ASSERT that body 1 velocity is zero for this to be correct
			// NOTE this will break if we ever have non zero velocities on pinned bodies!
			return body2.omega;
		} else if ( body2.pinned || body2.temporarilyPinned ) {
			// ASSERT that body 2 velocity is zero for this to be correct
			// NOTE this will break if we ever have non zero velocities on pinned bodies!
			return body1.omega;
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
