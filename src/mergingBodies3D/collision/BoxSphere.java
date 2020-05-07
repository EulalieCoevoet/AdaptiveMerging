package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mergingBodies3D.Contact;
import mergingBodies3D.ContactPool;
import mergingBodies3D.RigidBody;

/**
 * Box-Sphere collision test with preallocation of working variables
 * Code is perhaps not so pretty?
 * @author kry
 */
public class BoxSphere {

	/** 
	 * query point, i.e., to hold corners of cube in world coords, and allocated once to reuse	
	 */
	private static Point3d q = new Point3d(); 	
	
	/**
	 * Sphere center in box body coordinates
	 */
	private static Point3d cB = new Point3d();

	/** 
	 * upper right corder of box (half the size)
	 * Could probably just be stored in the geometry as it is recomputed often
	 */
	private static Vector3d p = new Vector3d(); 

	/**
	 * Helper vector for queries and setting the normal
	 */
	private static Vector3d v = new Vector3d();

	private static class TestResult {
		Point3d pos = new Point3d();
		Vector3d normal = new Vector3d();
		double depth;
	}
	
	private static TestResult testResult = new TestResult();
	
	/**
	 * Check for collision between box and sphere and collect contacts
	 * @param TB2W1  box body to world transform
	 * @param TW2B1  box world to body transform
	 * @param size   box size
	 * @param c		 sphere center in world
	 * @param r		 sphere radius
	 * @param contacts 
	 * @return 			number of contacts (zero or one)
	 */
	public static int dBoxSphere( RigidBody body1, Vector3d size, RigidBody body2, Point3d c, double r, ArrayList<Contact> contacts, ContactPool contactPool ) {
		p.scale( 0.5, size );
		
		body1.transformB2W.inverseTransform( c, cB );
		//body1.transformW2B.transform( c, cB );
		testResult.depth = 1;
		
		// checking corners is easy, we'll do it in box coordinates		
		q.set(  p.x,  p.y,  p.z ); test1( q, cB, r );
		q.set(  p.x,  p.y, -p.z ); test1( q, cB, r );
		q.set(  p.x, -p.y,  p.z ); test1( q, cB, r );
		q.set(  p.x, -p.y, -p.z ); test1( q, cB, r );
		q.set( -p.x,  p.y,  p.z ); test1( q, cB, r );
		q.set( -p.x,  p.y, -p.z ); test1( q, cB, r );
		q.set( -p.x, -p.y,  p.z ); test1( q, cB, r );
		q.set( -p.x, -p.y, -p.z ); test1( q, cB, r );

		double s;

		// The tests below are not so pretty compared to the above tests... can it be cleaner / simpler??

		// Do edge distance checks, again in box coordinates
		if ( -p.x <= cB.x && cB.x <= p.x ) {
			if        ( cB.y >=  p.y && cB.z >=  p.z ) { v.set(0, cB.y-p.y, cB.z-p.z); s=v.length(); s-=r; test2( s, cB.x,  p.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.y <= -p.y && cB.z >=  p.z ) { v.set(0, cB.y+p.y, cB.z-p.z); s=v.length(); s-=r; test2( s, cB.x, -p.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.y >=  p.y && cB.z <= -p.z ) { v.set(0, cB.y-p.y, cB.z+p.z); s=v.length(); s-=r; test2( s, cB.x,  p.y, -p.z, v.x, v.y, v.z );				
			} else if ( cB.y <= -p.y && cB.z <= -p.z ) { v.set(0, cB.y+p.y, cB.z+p.z); s=v.length(); s-=r; test2( s, cB.x, -p.y, -p.z, v.x, v.y, v.z );			
			}
		}
		if ( -p.y <= cB.y && cB.y <= p.y ) {
			if        ( cB.x >=  p.x && cB.z >=  p.z ) { v.set(cB.x-p.x, 0, cB.z-p.z); s=v.length(); s-=r; test2( s,  p.x, cB.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.x <= -p.x && cB.z >=  p.z ) { v.set(cB.x+p.x, 0, cB.z-p.z); s=v.length(); s-=r; test2( s, -p.x, cB.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.x >=  p.x && cB.z <= -p.z ) { v.set(cB.x-p.x, 0, cB.z+p.z); s=v.length(); s-=r; test2( s,  p.x, cB.y, -p.z, v.x, v.y, v.z );				
			} else if ( cB.x <= -p.x && cB.z <= -p.z ) { v.set(cB.x+p.x, 0, cB.z+p.z); s=v.length(); s-=r; test2( s, -p.x, cB.y, -p.z, v.x, v.y, v.z );			
			}
		}
		if ( -p.z <= cB.z && cB.z <= p.z ) {
			if        ( cB.x >=  p.x && cB.y >=  p.y ) { v.set(cB.x-p.x, cB.y-p.y, 0); s=v.length(); s-=r; test2( s,  p.x,  p.y, cB.z, v.x, v.y, v.z );
			} else if ( cB.x <= -p.x && cB.y >=  p.y ) { v.set(cB.x+p.x, cB.y-p.y, 0); s=v.length(); s-=r; test2( s, -p.x,  p.y, cB.z, v.x, v.y, v.z );
			} else if ( cB.x >=  p.x && cB.y <= -p.y ) { v.set(cB.x-p.x, cB.y+p.y, 0); s=v.length(); s-=r; test2( s,  p.x, -p.y, cB.z, v.x, v.y, v.z );				
			} else if ( cB.x <= -p.x && cB.y <= -p.y ) { v.set(cB.x+p.x, cB.y+p.y, 0); s=v.length(); s-=r; test2( s, -p.x, -p.y, cB.z, v.x, v.y, v.z );			
			}		
		}
		
		// checking the faces is easy in box coordinates, 
		if ( -p.x <= cB.x && cB.x <= p.x && -p.y <= cB.y && cB.y <= p.y ) {
			if ( cB.z > 0 ) { // check +z normal face
				s =  cB.z - p.z - r;
				test2( s, cB.x, cB.y, p.z, 0, 0, 1 ); 
			} else { // check -z normal face
				s = -p.z - cB.z - r;
				test2( s, cB.x, cB.y, -p.z, 0, 0, -1 ); 				
			}
		}
		if ( -p.x <= cB.x && cB.x <= p.x && -p.z <= cB.z && cB.z <= p.z ) {
			if ( cB.y > 0 ) { // check +y normal face
				s =  cB.y - p.y - r;
				test2( s, cB.x, p.y, cB.z, 0, 1, 0 ); 
			} else { // check -y normal face
				s = -p.y - cB.y - r;
				test2( s, cB.x, -p.y, cB.z, 0, -1, 0 ); 				
			}
		}		 
		if ( -p.y <= cB.y && cB.y <= p.y && -p.z <= cB.z && cB.z <= p.z ) {
			if ( cB.x > 0 ) { // check +x normal face
				s =  cB.x - p.x - r;
				test2( s, p.x, cB.y, cB.z, 1, 0, 0 ); 			
			} else { // check -x normal face
				s =  -p.x - cB.x - r;
				test2( s, -p.x, cB.y, cB.z, -1, 0, 0 ); 						
			}
		}
		
		if ( testResult.depth != 1 ) {
			// if we found a contact, transform the position and normal back to world coordinates 
			body1.transformB2W.transform( testResult.pos );
			body1.transformB2W.transform( testResult.normal );
			testResult.normal.normalize(); // normalize only once here!
			Contact contact = contactPool.get();
			// swap the sign as we did everything as separation distance!
			// info is zero as there is only ever one contact
			contact.set(body1, body2, testResult.pos, testResult.normal, null, null, 0, testResult.depth );
			contacts.add( contact );
			return 1;
		}
		return 0;
	}
	
	/**
	 * Check for collision between box and sphere and collect contacts
	 * @param TB2W1  box body to world transform
	 * @param TW2B1  box world to body transform
	 * @param size   box size
	 * @param c		 sphere center in world
	 * @param r		 sphere radius
	 * @param contacts 
	 * @return 			number of contacts (zero or one)
	 */
	public static boolean dBoxSphereTest( RigidBody body1, Vector3d size, Point3d c, double r ) {
		if ( c.distance(body1.x) > body1.radius + r ) return false; // faster exit... 
		
		p.scale( 0.5, size );
		body1.transformB2W.inverseTransform( c, cB );
		// RigidTransform3D TW2B1 = body1.transformW2B;
		// TW2B1.transform( c, cB );

		// quick exit based on distance to faces
		if ( cB.x -  p.x > r ) return false;
		if ( -p.x - cB.x > r ) return false;
		if ( cB.y -  p.y > r ) return false;
		if ( -p.y - cB.y > r ) return false;
		if ( cB.z -  p.z > r ) return false;
		if ( -p.z - cB.z > r ) return false;

		// we certainly don't need the rest of the checks as much as those above, but we 
		// migth yet catch a few early exits...
		
		// checking corners is easy, we'll do it in box coordinates		
		q.set(  p.x,  p.y,  p.z ); if ( cB.distance(q)-r < 0) return true;
		q.set(  p.x,  p.y, -p.z ); if ( cB.distance(q)-r < 0) return true;
		q.set(  p.x, -p.y,  p.z ); if ( cB.distance(q)-r < 0) return true;
		q.set(  p.x, -p.y, -p.z ); if ( cB.distance(q)-r < 0) return true;
		q.set( -p.x,  p.y,  p.z ); if ( cB.distance(q)-r < 0) return true;
		q.set( -p.x,  p.y, -p.z ); if ( cB.distance(q)-r < 0) return true;
		q.set( -p.x, -p.y,  p.z ); if ( cB.distance(q)-r < 0) return true;
		q.set( -p.x, -p.y, -p.z ); if ( cB.distance(q)-r < 0) return true;

		// The tests below are not so pretty compared to the above tests... can it be cleaner / simpler??

		// Do edge distance checks, again in box coordinates
		if ( -p.x <= cB.x && cB.x <= p.x ) {
			if        ( cB.y >=  p.y && cB.z >=  p.z ) { v.set(0, cB.y-p.y, cB.z-p.z); if ( v.length()-r < 0 ) return true;
			} else if ( cB.y <= -p.y && cB.z >=  p.z ) { v.set(0, cB.y+p.y, cB.z-p.z); if ( v.length()-r < 0 ) return true;
			} else if ( cB.y >=  p.y && cB.z <= -p.z ) { v.set(0, cB.y-p.y, cB.z+p.z); if ( v.length()-r < 0 ) return true;
			} else if ( cB.y <= -p.y && cB.z <= -p.z ) { v.set(0, cB.y+p.y, cB.z+p.z); if ( v.length()-r < 0 ) return true;
			}
		}
		if ( -p.y <= cB.y && cB.y <= p.y ) {
			if        ( cB.x >=  p.x && cB.z >=  p.z ) { v.set(cB.x-p.x, 0, cB.z-p.z); if ( v.length()-r < 0 ) return true;
			} else if ( cB.x <= -p.x && cB.z >=  p.z ) { v.set(cB.x+p.x, 0, cB.z-p.z); if ( v.length()-r < 0 ) return true;
			} else if ( cB.x >=  p.x && cB.z <= -p.z ) { v.set(cB.x-p.x, 0, cB.z+p.z); if ( v.length()-r < 0 ) return true;
			} else if ( cB.x <= -p.x && cB.z <= -p.z ) { v.set(cB.x+p.x, 0, cB.z+p.z); if ( v.length()-r < 0 ) return true;			
			}
		}
		if ( -p.z < cB.z && cB.z < p.z ) {
			if        ( cB.x >=  p.x && cB.y >=  p.y ) { v.set(cB.x-p.x, cB.y-p.y, 0); if ( v.length()-r < 0 ) return true;
			} else if ( cB.x <= -p.x && cB.y >=  p.y ) { v.set(cB.x+p.x, cB.y-p.y, 0); if ( v.length()-r < 0 ) return true;
			} else if ( cB.x >=  p.x && cB.y <= -p.y ) { v.set(cB.x-p.x, cB.y+p.y, 0); if ( v.length()-r < 0 ) return true;
			} else if ( cB.x <= -p.x && cB.y <= -p.y ) { v.set(cB.x+p.x, cB.y+p.y, 0); if ( v.length()-r < 0 ) return true;
			}		
		}
		
		// checking the faces is easy in box coordinates, 
		if ( -p.x <= cB.x && cB.x <= p.x && -p.y <= cB.y && cB.y <= p.y ) {
			if ( cB.z > 0 ) { // check +z normal face
				if ( cB.z - p.z - r < 0 ) return true;
			} else { // check -z normal face
				if ( -p.z - cB.z - r < 0 ) return true;
			}
		}
		if ( -p.x <= cB.x && cB.x <= p.x && -p.z <= cB.z && cB.z <= p.z ) {
			if ( cB.y > 0 ) { // check +y normal face
				if ( cB.y - p.y - r < 0 ) return true;
			} else { // check -y normal face
				if ( -p.y - cB.y - r < 0 ) return true;
			}
		}		 
		if ( -p.y <= cB.y && cB.y <= p.y && -p.z <= cB.z && cB.z <= p.z ) {
			if ( cB.x > 0 ) { // check +x normal face
				if ( cB.x - p.x - r < 0 ) return true;
			} else { // check -x normal face
				if ( -p.x - cB.x - r < 0 ) return true;
			}
		}
		return false;
	}

	/**
	 * Checks if the distance between two points is less than r (i.e., sphere center tested with corners)
	 * Silly to pass testResult in this case, so the static member is modified with the result!
	 * @param q
	 * @param cB
	 * @param r
	 */
	private static void test1( Point3d q, Point3d cB, double r ) {
		double s = cB.distance(q)-r;
		if ( s > 0 ) return;
		if ( s < testResult.depth ) {
			// could set contact point as half way, or for simplicity, just set it to be the corner of the cube...
			testResult.pos.set(q);
			testResult.normal.sub( cB, q );
			testResult.normal.normalize();  // should normalize at end for speed.
			testResult.depth = s;
			// contact info unneeded as can only have one contact between sphere and box
		}
	}

	/**
	 * Checks the provided depth, and stores the given contact point and normal if it is the deepest (most negative) so far.
 	 * Silly to pass testResult in this case, so the static member is modified with the result!
	 * @param depth
	 * @param px
	 * @param py
	 * @param pz
	 * @param nx
	 * @param ny
	 * @param nz
	 */
	private static void test2( double depth, double px, double py, double pz, double nx, double ny, double nz ) {
		if ( depth > 0 ) return;
		if ( depth < testResult.depth ) {
			// set contact point as given (which is on the surface of the box... as opposed to half way).
			testResult.pos.set( px, py, pz );
			testResult.normal.set( nx, ny, nz );
			testResult.depth = depth;
			// contact info not needed as can only have one contact between sphere and box
		}
	}
		
}
