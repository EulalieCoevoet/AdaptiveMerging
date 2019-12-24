package mergingBodies3D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.EasyViewer;

public class RigidBodyGeomSphere extends RigidBodyGeom {

	double r;
	
	

	public RigidBodyGeomSphere( double r ) {
		this.r = r;
		float[] col= new float[] {0.9f,0.9f,0.9f,1};
		this.colour = col;
	}
    
    /** 
     * Draws the blocks of a rigid body
     * @param drawable
     */
    public void drawGeom( GLAutoDrawable drawable ) {    
    	GL2 gl = drawable.getGL().getGL2();
    	gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, colour, 0 );
    	EasyViewer.glut.glutSolidSphere(r, 32, 32);
    }
}
