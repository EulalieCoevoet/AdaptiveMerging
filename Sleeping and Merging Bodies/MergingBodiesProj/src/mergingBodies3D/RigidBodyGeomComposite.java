package mergingBodies3D;

import java.util.ArrayList;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.BooleanParameter;
import tools.moments.PolygonSoup;

/**
 * A composite body is a permanently merged collection of bodies, and is perhaps useful 
 * for generating interesting geometries from collections of simple primitives without
 * resorting to a poor sphere-tree based collision model.
 * @author kry
 */
public class RigidBodyGeomComposite extends RigidBodyGeom {

	public ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
	
	static private double[] openlGLmatrix = new double[16];
	
	/** optional visual geometry that can be set at load time */
	public PolygonSoup soup = null;
	
	/** Added to interface by the Display class getControls method */
	static BooleanParameter disableDisplaySoup = new BooleanParameter("display composite bodies instead of mesh mesh (if mesh defined)", false );
	
	/**
	 * Updates the composite body positions for collision detection, or for drawing
	 */
	public void updateBodyPositionsFromParent() {
		for ( RigidBody b : bodies ) {
			b.transformB2W.mult( b.compositeBodyParent.transformB2W, b.transformB2C );
		}
	}
	
	/**
	 * We override the display method... don't create a display list for 
	 * the whole body, but instead we'll call the bodies display methods individaully 
	 * ( each with their own display list )
	 * 
	 * 
	 */
	@Override
	public void drawGeom(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		if ( soup != null && ! disableDisplaySoup.getValue() ) {
			soup.display(drawable);
		} else {
			for ( RigidBody b : bodies ) {
				gl.glPushMatrix();        
		        gl.glMultMatrixd( b.transformB2C.getAsArray(openlGLmatrix), 0 );
		        // skip the display list building step that would be at the body level... 
		        b.geom.drawGeom( drawable );	        
		        gl.glPopMatrix();
			}
		}
	}

}
