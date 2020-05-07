package mergingBodies3D;

import com.jogamp.opengl.GLAutoDrawable;

import tools.moments.PolygonSoup;

/** 
 * Really just a wrapper for the soup
 * @author kry
 */
public class RigidBodyGeomMesh extends RigidBodyGeom {

	private PolygonSoup soup;
	
	public RigidBodyGeomMesh( PolygonSoup soup ) {
		this.soup = soup;
	}
    
    public void drawGeom( GLAutoDrawable drawable ) {    
    	soup.display(drawable);
    }
}
