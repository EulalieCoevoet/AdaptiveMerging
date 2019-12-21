package mergingBodies3D;

import java.util.HashMap;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public abstract class RigidBodyGeom {

    /** Map to keep track of display list IDs for drawing our rigid bodies efficiently */
    static protected HashMap<Object,Integer> mapObjectsToDisplayList = new HashMap<Object,Integer>();
    
    /** display list ID for this rigid body */
    protected int myListID = -1;
    
    /** 
     * Draws the rigid body geometry, and uses display lists to make things reasonably fast
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        if ( myListID == -1 ) {
            Integer ID = mapObjectsToDisplayList.get(this);
            if ( ID == null ) {
                myListID = gl.glGenLists(1);
                gl.glNewList( myListID, GL2.GL_COMPILE_AND_EXECUTE );
                
                drawGeom( drawable );
                
                gl.glEndList();
                mapObjectsToDisplayList.put( this, myListID );
            } else {
                myListID = ID;
                gl.glCallList(myListID);
            }
        } else {
            gl.glCallList(myListID);
        }
    }
    
    public abstract void drawGeom( GLAutoDrawable drawable );	
    
    /**
     * Deletes all display lists.
     * This is called when clearing all rigid bodies from the simulation, or when display lists need to be updated due to 
     * changing transparency of the blocks.
     * @param gl
     */
    static public void clearDisplayLists( GL2 gl ) {
        for ( int id : mapObjectsToDisplayList.values() ) {
            gl.glDeleteLists(id, 1);
        }
        mapObjectsToDisplayList.clear();
    }
}
