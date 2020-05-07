package  mergingBodies2D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.vecmath.Color3f;
import javax.vecmath.Point2d;

/**
 * The Block class represents a single pixel in the original image.  A rigid body is
 * made up of a collection of connected blocks.
 * @author kry
 */
public class Block {

    /** Transparency with which to draw all blocks */
    static float alpha;
    
    /** Radius of circle that encloses this block */
    static double radius = Math.sqrt(2) * 0.5;// * 0.9; // 90% of the normal size... allow for some overlap?
    
    /** Block pixel color */
    Color3f c = new Color3f();
    
    /** row index in the original image */
    int i;
    
    /** column index in the original image */
    int j;
    
    /** position of block in the body frame */
    Point2d pB = new Point2d(); 
    
    /** size of pixel */
    double h = 0.5;
 
    /**
     * Creates a new block
     * @param i
     * @param j
     * @param c
     */
    public Block( int i, int j, Color3f c ) {
        this.i = i;
        this.j = j;
        this.c.set( c );
    }
    
    /**
     * Computes a mass based on the intensity, darker is heavier.  Uses standard color to intensity conversion weights.
     * @return a value between 0 and 1
     */
    public double getColorMass() {
        return 1 - (0.3 * c.x + 0.59 * c.y + 0.11 * c.z);
    }
    
    /**
     * Draws the block in its body coordinates.
     * @param drawable
     */
    
    public void display( GLAutoDrawable drawable, Color3f color ) {
        GL2 gl = drawable.getGL().getGL2();
        if(color!=null)
        	gl.glColor4f( color.x, color.y, color.z, alpha );
        else 
        	gl.glColor4f( c.x, c.y, c.z, alpha );
        
        gl.glBegin(GL.GL_TRIANGLE_STRIP);
        gl.glVertex2d( pB.x - h, pB.y - h );
        gl.glVertex2d( pB.x - h, pB.y + h );
        gl.glVertex2d( pB.x + h, pB.y - h );
        gl.glVertex2d( pB.x + h, pB.y + h );
        gl.glEnd();
    }   
}
