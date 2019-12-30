package mergingBodies3D;

import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.EasyViewer;

public class RigidBodyGeomSphere extends RigidBodyGeom {

	double r;

	public RigidBodyGeomSphere( double r ) {
		this.r = r;
	}
    
    /** 
     * Draws the blocks of a rigid body
     * @param drawable
     */
    public void drawGeom( GLAutoDrawable drawable ) {    
    	EasyViewer.glut.glutSolidSphere(r, 32, 32);
    }
}
