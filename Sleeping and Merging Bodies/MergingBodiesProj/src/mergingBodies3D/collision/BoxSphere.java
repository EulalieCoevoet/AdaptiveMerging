package mergingBodies3D.collision;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mergingBodies3D.RigidTransform;

public class BoxSphere {

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
	public static int dBoxSphere( RigidTransform TB2W1, RigidTransform TW2B1, Vector3d size, Point3d c, double r, ArrayList<DContactGeom> contacts ) {
		Vector3d p = new Vector3d(); 
		p.scale( 0.5, size );
		Point3d q = new Point3d(); // to hold corners of cube in world coords		
		Point3d cB = new Point3d(); // sphere center in the box body coordinates
		TW2B1.transform( c, cB );
		DContactGeom contact = null;
		
		// checking corners is easy, we'll do it in box coordinates		
		q.set(  p.x,  p.y,  p.z ); contact = test1( contact, q, cB, r );
		q.set(  p.x,  p.y, -p.z ); contact = test1( contact, q, cB, r );
		q.set(  p.x, -p.y,  p.z ); contact = test1( contact, q, cB, r );
		q.set(  p.x, -p.y, -p.z ); contact = test1( contact, q, cB, r );
		q.set( -p.x,  p.y,  p.z ); contact = test1( contact, q, cB, r );
		q.set( -p.x,  p.y, -p.z ); contact = test1( contact, q, cB, r );
		q.set( -p.x, -p.y,  p.z ); contact = test1( contact, q, cB, r );
		q.set( -p.x, -p.y, -p.z ); contact = test1( contact, q, cB, r );

		double s;
		Vector3d v = new Vector3d();

		// The tests below are not so pretty compared to the above tests... can it be cleaner / simpler??

		// Do edge distance checks, again in box coordinates
		if ( -p.x <= cB.x && cB.x <= p.x ) {
			if        ( cB.y >=  p.y && cB.z >=  p.z ) { v.set(0, cB.y-p.y, cB.z-p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, cB.x,  p.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.y <= -p.y && cB.z >=  p.z ) { v.set(0, cB.y+p.y, cB.z-p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, cB.x, -p.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.y >=  p.y && cB.z <= -p.z ) { v.set(0, cB.y-p.y, cB.z+p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, cB.x,  p.y, -p.z, v.x, v.y, v.z );				
			} else if ( cB.y <= -p.y && cB.z <= -p.z ) { v.set(0, cB.y+p.y, cB.z+p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, cB.x, -p.y, -p.z, v.x, v.y, v.z );			
			}
		}
		if ( -p.y <= cB.y && cB.y <= p.y ) {
			if        ( cB.x >=  p.x && cB.z >=  p.z ) { v.set(cB.x-p.x, 0, cB.z-p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s,  p.x, cB.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.x <= -p.x && cB.z >=  p.z ) { v.set(cB.x+p.x, 0, cB.z-p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, -p.x, cB.y,  p.z, v.x, v.y, v.z );
			} else if ( cB.x >=  p.x && cB.z <= -p.z ) { v.set(cB.x-p.x, 0, cB.z+p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s,  p.x, cB.y, -p.z, v.x, v.y, v.z );				
			} else if ( cB.x <= -p.x && cB.z <= -p.z ) { v.set(cB.x+p.x, 0, cB.z+p.z); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, -p.x, cB.y, -p.z, v.x, v.y, v.z );			
			}
		}
		if ( -p.z <= cB.z && cB.z <= p.z ) {
			if        ( cB.x >=  p.x && cB.y >=  p.y ) { v.set(cB.x-p.x, cB.y-p.y, 0); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s,  p.x,  p.y, cB.z, v.x, v.y, v.z );
			} else if ( cB.x <= -p.x && cB.y >=  p.y ) { v.set(cB.x+p.x, cB.y-p.y, 0); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, -p.x,  p.y, cB.z, v.x, v.y, v.z );
			} else if ( cB.x >=  p.x && cB.y <= -p.y ) { v.set(cB.x-p.x, cB.y+p.y, 0); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s,  p.x, -p.y, cB.z, v.x, v.y, v.z );				
			} else if ( cB.x <= -p.x && cB.y <= -p.y ) { v.set(cB.x+p.x, cB.y+p.y, 0); s=v.length(); v.scale(1/s); s-=r; contact = test2( contact, s, -p.x, -p.y, cB.z, v.x, v.y, v.z );			
			}		
		}
		
		// checking the faces is easy in box coordinates, 
		if ( -p.x <= cB.x && cB.x <= p.x && -p.y <= cB.y && cB.y <= p.y ) {
			if ( cB.z > 0 ) { // check +z normal face
				s =  cB.z - p.z - r;
				contact = test2( contact, s, cB.x, cB.y, p.z, 0, 0, 1 ); 
			} else { // check -z normal face
				s = -p.z - cB.z - r;
				contact = test2( contact, s, cB.x, cB.y, -p.z, 0, 0, -1 ); 				
			}
		}
		if ( -p.x <= cB.x && cB.x <= p.x && -p.z <= cB.z && cB.z <= p.z ) {
			if ( cB.y > 0 ) { // check +y normal face
				s =  cB.y - p.y - r;
				contact = test2( contact, s, cB.x, p.y, cB.z, 0, 1, 0 ); 
			} else { // check -y normal face
				s = -p.y - cB.y - r;
				contact = test2( contact, s, cB.x, -p.y, cB.z, 0, -1, 0 ); 				
			}
		}		 
		if ( -p.y <= cB.y && cB.y <= p.y && -p.z <= cB.z && cB.z <= p.z ) {
			if ( cB.x > 0 ) { // check +x normal face
				s =  cB.x - p.x - r;
				contact = test2( contact, s, p.x, cB.y, cB.z, 1, 0, 0 ); 			
			} else { // check -x normal face
				s =  -p.x - cB.x - r;
				contact = test2( contact, s, -p.x, cB.y, cB.z, -1, 0, 0 ); 						
			}
		}
		
		if ( contact != null ) {
			// if we found a contact, transform the position and normal back to world coordinates 
			TB2W1.transform( contact.pos );
			TB2W1.transform( contact.normal );
			contacts.add( contact );
			return 1;
		}
		return 0;
	}

	/**
	 * Because vecmath sucks sometimes... here is the distance from p to the origin
	 * @param p
	 * @return
	 */
	private static double distanceToOrigin( Point3d p ) {
		return Math.sqrt( p.x*p.x + p.y*p.y + p.z*p.z ); 
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
	public static boolean dBoxSphereTest( RigidTransform TB2W1, RigidTransform TW2B1, Vector3d size, Point3d c, double r ) {
		Vector3d p = new Vector3d(); 
		p.scale( 0.5, size );
		Point3d q = new Point3d(); // to hold corners of cube in world coords		
		Point3d cB = new Point3d(); // sphere center in the box body coordinates
		TW2B1.transform( c, cB );

		if ( distanceToOrigin(cB) > p.length() + r ) return false; // quick exit
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

		Vector3d v = new Vector3d();

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

	
	private static DContactGeom test1( DContactGeom contact, Point3d q, Point3d cB, double r ) {
		double s = cB.distance(q)-r;
		if ( s > 0 ) return contact;
		if ( contact == null ) {
			contact = new DContactGeom();
			contact.depth = 1; // force code below to trigger given that s is non-positive
		}
		if ( s < contact.depth ) {
			// could set contact point as half way, or for simplicity, just set it to be the corner of the cube...
			contact.pos.set(q);
			contact.normal.sub( cB, q );
			contact.normal.normalize();
			contact.depth = s;
			// contact info unneeded as can only have one contact between sphere and box
		}
		return contact;
	}

	private static DContactGeom test2( DContactGeom contact, double depth, double px, double py, double pz, double nx, double ny, double nz ) {
		if ( depth > 0 ) return contact;
		if ( contact == null ) {
			contact = new DContactGeom();
			contact.depth = 1; // force code below to trigger given that s is non-positive			
		}
		if ( depth < contact.depth ) {
			// set contact point as given (which is on the surface of the box... as opposed to half way).
			contact.pos.set( px, py, pz );
			contact.normal.set( nx, ny, nz );
			contact.depth = depth;
			// contact info not needed as can only have one contact between sphere and box
		}
		return contact;
	}
		
}
