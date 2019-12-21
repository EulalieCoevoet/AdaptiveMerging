package mergingBodies3D;

import java.util.ArrayList;

import com.jogamp.opengl.GLAutoDrawable;

public class RigidBodyGeomBlocks extends RigidBodyGeom {

	private ArrayList<Block> blocks;
	
	public RigidBodyGeomBlocks( ArrayList<Block> blocks ) {
		this.blocks = blocks;
	}
    
    /** 
     * Draws the blocks of a rigid body
     * @param drawable
     */
    public void drawGeom( GLAutoDrawable drawable ) {    
        for ( Block b : blocks ) {
            b.display( drawable );
        }                
    }
}
