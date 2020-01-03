package mergingBodies3D;

import java.util.ArrayList;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

/**
 * A composite body is a permanently merged collection of bodies, and is perhaps useful 
 * for generating interesting geometries from collections of simple primitives without
 * resorting to a poor sphere-tree based collision model.
 * @author kry
 */
public class RigidBodyGeomComposite extends RigidBodyGeom {

	public ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
	
	@Override
	public void drawGeom(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
		for ( RigidBody b : bodies ) {
			gl.glPushMatrix();        
	        gl.glMultMatrixd( b.transformB2W.Tflat.asArray(),0 );
	        // skip the display list building step that would be at the body level... 
	        b.geom.drawGeom( drawable );	        
	        gl.glPopMatrix();
		}	
	}

}
