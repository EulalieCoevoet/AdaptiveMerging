package mergingBodies3D;

import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.EasyViewer;

public class RigidBodyGeomBox extends RigidBodyGeom {

	Vector3d size = new Vector3d();
	
	
	public RigidBodyGeomBox( Vector3d size ) {
		this.size.set(size);
		float[] col= new float[] {0.9f,0.9f,0.9f,1};
		this.colour = col;
	}
    
    /** 
     * Draws the blocks of a rigid body
     * @param drawable
     */
    public void drawGeom( GLAutoDrawable drawable ) {    
    	GL2 gl = drawable.getGL().getGL2();
    	gl.glPushMatrix();
    	gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, colour, 0 );
    	gl.glScaled( size.x, size.y, size.z );
    	EasyViewer.glut.glutSolidCube(1);
    	gl.glPopMatrix();
    }
}
