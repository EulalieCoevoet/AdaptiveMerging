package mergingBodies3D;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.EasyViewer;

public class RigidBodyGeomBox extends RigidBodyGeom {

	public Vector3d size = new Vector3d();
	
	public RigidBodyGeomBox( Vector3d size ) {
		this.size.set(size);
	}
    
    /** 
     * Draws the blocks of a rigid body
     * @param drawable
     */
    public void drawGeom( GLAutoDrawable drawable ) {    
    	GL2 gl = drawable.getGL().getGL2();
    	gl.glPushMatrix();
    	gl.glScaled( size.x, size.y, size.z );
    	EasyViewer.glut.glutSolidCube(1);
    	gl.glPopMatrix();
    }
}
