package mergingBodies3D;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * This class is used for relative motion's calculations
 * @author eulalie coevoet
 */
public class MotionMetricProcessor {	

	RigidBody body1Advanced = new RigidBody();
	RigidBody body2Advanced = new RigidBody();
	
	public double getMotionMetric(RigidBody body1, RigidBody body2) {
		return getLargestVelocityNorm(body1, body2);
	}
	
	public double getMotionMetric(RigidBody body) {
		return getLargestVelocityNorm(body);
	}
	
	public double getMotionMetric(RigidBody body1, RigidBody body2, double dt) {
		return getLargestVelocityNorm(body1, body2, dt);
	}
	
	private Point3d pW = new Point3d();
	private Point3d pB = new Point3d();
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
		for(int i=0; i<2; i++) {
			RigidBody body = (i==0)? body1: body2;
			for (Point3d point : body.boundingBoxB) {
				body.transformB2W.transform( point, pW );
				body1.getSpatialVelocity(pW, v1);
				body2.getSpatialVelocity(pW, v2);
				v1.sub(v2);
				largestVelocityNorm = Math.max( v1.length(), largestVelocityNorm );
			}
		}
		return largestVelocityNorm;
	}
	
	/**
	 * Compute the magnitude of the largest relative velocity looking at
	 * each bounding box point the given body.
	 * @param body
	 * @return
	 */
	private double getLargestVelocityNorm(RigidBody body) {
		
		double largestVelocityNorm = 0;
		for (Point3d point : body.boundingBoxB) {
			body.transformB2W.transform( point, pW );
			body.getSpatialVelocity(pW, v1);
			largestVelocityNorm = Math.max( v1.length(), largestVelocityNorm );
		}
		return largestVelocityNorm;
	}
	
	private double getLargestVelocityNorm(RigidBody body1, RigidBody body2, double dt) {
		
		double largestVelocityNorm = 0;
		
		body1Advanced.set(body1);
		body1Advanced.advancePositions(dt);
		
		body2Advanced.set(body2);
		body2Advanced.advancePositions(dt);
		
		for (Point3d point : body1.boundingBoxB) {
			// point from b1 to b2
			body1.transformB2W.transform( point, pW );
			body2.transformB2W.inverseTransform( pW, pB );
			
			// advanced point from b2 to b1
			body2Advanced.transformB2W.transform( pB, pW );
			body1Advanced.transformB2W.inverseTransform( pW, pB );
			
			// effective velocity
			v1.sub(point,pB);
			v1.scale(1./dt);
			largestVelocityNorm = Math.max( v1.length(), largestVelocityNorm );
		}
		
		for (Point3d point : body2.boundingBoxB) {
			// point from b2 to b1
			body2.transformB2W.transform( point, pW );
			body1.transformB2W.inverseTransform( pW, pB );

			// advanced point from b1 to b2
			body1Advanced.transformB2W.transform( pB, pW );
			body2Advanced.transformB2W.inverseTransform( pW, pB );
			
			// effective velocity
			v2.sub(point,pB);
			v2.scale(1./dt);
			largestVelocityNorm = Math.max( v2.length(), largestVelocityNorm );
		}
		
		return largestVelocityNorm;
	}
}
