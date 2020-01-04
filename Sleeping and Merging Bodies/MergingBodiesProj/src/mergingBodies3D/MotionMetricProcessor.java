package mergingBodies3D;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * This class is used for relative motion's calculations
 */
public class MotionMetricProcessor {	

	enum MotionMetricType {LARGESTVELOCITY, RELATIVEKINETICENERGY, VELOCITIESNORM};
	//private MotionMetricType motionMetricType = MotionMetricType.LARGESTVELOCITY;
	
	public void setMotionMetricType( int type ) {
		if ( type !=  MotionMetricType.LARGESTVELOCITY.ordinal() ) {
			System.err.println("motion metric request ignored... should only use LARGESTVELOCITY");
		}
//		if(type ==  MotionMetricType.LARGESTVELOCITY.ordinal())
//			motionMetricType = MotionMetricType.LARGESTVELOCITY;
//		else if(type ==  MotionMetricType.RELATIVEKINETICENERGY.ordinal())
//			motionMetricType = MotionMetricType.RELATIVEKINETICENERGY;
//		else if(type ==  MotionMetricType.VELOCITIESNORM.ordinal())
//			motionMetricType = MotionMetricType.VELOCITIESNORM;
//		else
//			System.err.println("[getMetric] metric type unknown");
	}
	
	public double getMotionMetric(RigidBody body1, RigidBody body2) {
		return getLargestVelocityNorm(body1, body2);
	
//		double metric = 0.;		
//			if (motionMetricType == MotionMetricType.VELOCITIESNORM)
//				metric = getRelativeVelocitiesNorm(body1, body2);
////			else if (motionMetricType == MotionMetricType.RELATIVEKINETICENERGY) 
////				metric = getRelativeKineticEnergy(body1, body2);
//			else if (motionMetricType == MotionMetricType.LARGESTVELOCITY)
//				metric = getLargestVelocityNorm(body1, body2);
//			else
//				System.err.println("[getMetric] metric type unknown");
//			
//			return metric;
	}
	
//	private double getRelativeVelocitiesNorm(RigidBody body1, RigidBody body2) {
//		
//		Vector3d relativeLinearVelocity = getRelativeLinearVelocity(body1, body2);
//		Vector3d relativeAngularVelocity = getRelativeAngularVelocity(body1, body2);
//		
//		double k = 0.5*relativeLinearVelocity.lengthSquared() + 0.5*relativeAngularVelocity.lengthSquared();
//		return k;
//	}
//	
////	private double getRelativeKineticEnergy(RigidBody body1, RigidBody body2) {
////
////		Vector3d relativeLinearVelocity = getRelativeLinearVelocity(body1, body2);
////		Vector3d relativeAngularVelocity = getRelativeAngularVelocity(body1, body2);
////		
////		double massDifference = Math.abs(body1.massLinear - body2.massLinear);
////		double inertiaDifference = Math.abs(body1.massAngular - body2.massAngular);
////		double k = 0.5*relativeLinearVelocity.lengthSquared()*massDifference+ 0.5*relativeAngularVelocity*relativeAngularVelocity*inertiaDifference;
////		
////		return k/massDifference;
////	}
	
	private Point3d pW = new Point3d();
	private Vector3d v1 = new Vector3d();
	private Vector3d v2 = new Vector3d();

	/**
	 * Compute the magnitude of the largest relative velocity looking at
	 * each bounding box point of each body.
	 * @param body1
	 * @param body2
	 * @return
	 */
	private double getLargestVelocityNorm(RigidBody body1, RigidBody body2) {
		
		// dont'need to compute this in the common frame... can simply ask how fast each of the points
		// of each bounding volume are moving in the other object's moving frame
		
		double largestVelocityNorm = 0;
		for (Point3d point : body1.boundingBoxB) {
			body1.transformB2W.transform( point, pW );
			body1.getSpatialVelocity(pW, v1);
			body2.getSpatialVelocity(pW, v2);
			v1.sub(v2);
			largestVelocityNorm = Math.max( v1.length(), largestVelocityNorm );
		}
		for (Point3d point : body2.boundingBoxB) {
			body2.transformB2W.transform( point, pW );
			body1.getSpatialVelocity(pW, v1);
			body2.getSpatialVelocity(pW, v2);
			v1.sub(v2);
			largestVelocityNorm = Math.max( v1.length(), largestVelocityNorm );
		}
		return largestVelocityNorm;
		
//		Point3d x = getCommonCOM(body1, body2);
//		Vector3d v = getRelativeLinearVelocity(body1, body2);
//		Vector3d omega = getRelativeAngularVelocity(body1, body2);
//		
//		double largestVelocityNorm1 = 0.;
//		for (Point3d point : body1.boundingBoxB) {
//			Vector3d rw = new Vector3d( -(point.y - relative.x.y), point.x - relative.x.x );
//			rw.scale( omega );
//			rw.add( v ); 
//			largestVelocityNorm1 = Math.max(largestVelocityNorm1, rw.lengthSquared());
//		}
//		
//		double largestVelocityNorm2 = 0.;
//		for (Point3d point : body2.boundingBoxB) {
//			Vector3d rw = new Vector3d( -(point.y - relative.x.y), point.x - relative.x.x );
//			rw.scale( omega );
//			rw.add( v ); 
//			largestVelocityNorm2 = Math.max(largestVelocityNorm2, rw.lengthSquared());
//		}
//		
//		return Math.max(largestVelocityNorm1, largestVelocityNorm2);
	}
	
//	/**
//	 * Compute the relative linear velocity
//	 * @param body1
//	 * @param body2
//	 * @return relative linear velocity
//	 */
//	private Vector3d getRelativeLinearVelocity(RigidBody body1, RigidBody body2) {
//		
////		if ( body1.pinned || body1.temporarilyPinned ) {
////			if(body1.v.x != 0. || body1.v.y != 0.) {
////				System.err.print("[getRelativeLinearVelocity] linear velocity of pinned body is not zero: ");
////				System.err.println(body1.v);
////			} else {
////				return body2.v;
////			}
////		} else if ( body2.pinned || body2.temporarilyPinned ) {
////			if(body2.v.x != 0. || body2.v.y != 0.) {
////				System.err.print("[getRelativeLinearVelocity] linear velocity of pinned body is not zero: ");
////				System.err.println(body2.v);
////			} else {
////				return body1.v;
////			}
////		}
//		
//		Vector3d relativeLinearVelocity = new Vector3d();
//	
//		Point3d COM = getCommonCOM(body1, body2);
//	
//		relativeLinearVelocity.sub(body2.v, body1.v);
//
//		Vector3d tmp = new Vector3d();
//		Vector3d tmp2 = new Vector3d();
//		
//		tmp.sub( COM, body2.x );
//		tmp.scale( body2.omega );
//		tmp2.set( -tmp.y, tmp.x );
//		relativeLinearVelocity.add( tmp2 );
//		
//		tmp.sub( COM, body1.x );
//		tmp.scale( body1.omega );
//		tmp2.set( -tmp.y, tmp.x );
//		relativeLinearVelocity.sub( tmp2 );
//		
//		return relativeLinearVelocity;
//	}
//	
//	/**
//	 * Compute common center of mass
//	 * TODO: MEMORY
//	 * @param body1
//	 * @param body2
//	 * @return
//	 */
//	private Point3d getCommonCOM(RigidBody body1, RigidBody body2) {
//		Point3d massCOM1 = new Point3d(body1.x);
//		Point3d massCOM2 = new Point3d(body2.x);
//		massCOM1.scale(body1.massLinear);
//		massCOM2.scale(body2.massLinear);
//		Point3d newCOM = new Point3d();
//		newCOM.add( massCOM1, massCOM2 );
//		newCOM.scale( 1./(body1.massLinear + body2.massLinear) );
//		return newCOM;
//	}
//
//	/**
//	 * Compute the relative angular velocity
//	 * @param body1
//	 * @param body2
//	 * @return relative angular velocity
//	 */
//	private Vector3d getRelativeAngularVelocity( RigidBody body1, RigidBody body2 ) {
//		
////		if ( body1.pinned || body1.temporarilyPinned ) {
////			if(body1.omega != 0.) {
////				System.err.print("[getRelativeAngularVelocity] angular velocity of pinned body is not zero: ");
////				System.err.println(body1.omega);
////			} else {
////				return body2.omega;
////			}
////		} else if ( body2.pinned || body2.temporarilyPinned ) {
////			if(body2.omega != 0.) {
////				System.err.print("[getRelativeAngularVelocity] angular velocity of pinned body is not zero: ");
////				System.err.println(body2.omega);
////			} else {
////				return body1.omega;
////			}
////		}
//		Vector3d deltaOmega = new Vector3d();
//		deltaOmega.sub( body2.omega, body1.omega );
//		return deltaOmega;
//	}
}
