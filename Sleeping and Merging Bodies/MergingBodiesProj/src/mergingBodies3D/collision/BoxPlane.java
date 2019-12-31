package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import mergingBodies3D.RigidTransform;

public class BoxPlane {

	private static Vector3d p = new Vector3d();
	private static Point3d q = new Point3d();

	/** 
	 * Collides box with plane
	 * @param TB2W  box's transform from body to world
	 * @param size  boxes dimensions
	 * @param n		plane normal
	 * @param d		plane intercept 
	 * @return the number of contact points
	 */
	public static int dBoxPlane( RigidTransform TB2W, Vector3d size, Vector3d n, double d, ArrayList<DContactGeom> contacts ) {
		//RigidTransform TB2W = new RigidTransform();
		p.scale( 0.5, size );
		int numContacts = contacts.size();
		double s;
		// we store the corner ID for warm starts...
		// NOTE: could this be more efficient by transforming the plane to box coordinates?
		q.set(  p.x,  p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,0) );	
		q.set(  p.x,  p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,1) );	
		q.set(  p.x, -p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,2) );	
		q.set(  p.x, -p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,3) );	
		q.set( -p.x,  p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,4) );	
		q.set( -p.x,  p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,5) );	
		q.set( -p.x, -p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,6) );	
		q.set( -p.x, -p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) contacts.add( new DContactGeom(q,n,-s,7) );			
		return contacts.size() - numContacts;
	}
	
	/** 
	 * Because vecmath is stupid sometimes, here is a dot product between tuples
	 * @param a
	 * @param b
	 * @return
	 */
	private static double dot( Tuple3d a, Tuple3d b ) {
		return a.x*b.x + a.y*b.y + a.z*b.z;
	}
		
}
