package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import mergingBodies3D.Contact;
import mergingBodies3D.ContactPool;
import mergingBodies3D.RigidBody;
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
	public static int dBoxPlane( RigidBody body1, Vector3d size, RigidBody body2, Vector3d n, double d, ArrayList<Contact> contacts, ContactPool pool ) {
		// quick test of box origin with plane
		if ( dot( body1.x, n ) + d > body1.radius ) return 0;

		p.scale( 0.5, size );
		int numContacts = contacts.size();
		RigidTransform TB2W = body1.transformB2W;
		double s;
		// we store the corner ID for warm starts...
		// NOTE: could this be more efficient by transforming the plane to box coordinates?
		q.set(  p.x,  p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 0, contacts, pool );	
		q.set(  p.x,  p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 1, contacts, pool );	
		q.set(  p.x, -p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 2, contacts, pool );	
		q.set(  p.x, -p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 3, contacts, pool );	
		q.set( -p.x,  p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 4, contacts, pool );	
		q.set( -p.x,  p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 5, contacts, pool );	
		q.set( -p.x, -p.y,  p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 6, contacts, pool );	
		q.set( -p.x, -p.y, -p.z ); TB2W.transform(q); s = dot(q,n) + d; if ( s < 0 ) addContact( body1, body2, q, n, s, 7, contacts, pool );			
		return contacts.size() - numContacts;
	}
	
	static private void addContact( RigidBody body1, RigidBody body2, Point3d q, Vector3d n, double s, int info, ArrayList<Contact> contacts, ContactPool pool ) {
		Contact c = pool.get();
		c.set( body2, body1, q, n, null, null, info, s ); // plane is always first for the contact normal to be correct
		contacts.add( c );
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
