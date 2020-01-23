/*
 * Created on 11-Sep-2003
 */
package mintools.viewer;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.glu.GLUquadric;

/**
 * Class for drawing reference frames with arrow geometry.
 * The axis is a grey ball, with red greed and blue arrows pointing 
 * in the x y and z directions.
 * 
 */
public class FancyAxis {
    
	static private FancyAxis defaultFancyAxis = new FancyAxis();
	
	static public void draw( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		float[] vec = new float[4];
		// cache (somewhat) the material properties
		// this will do something reasonable in most cases
        gl.glGetMaterialfv( GL.GL_FRONT, GL2.GL_DIFFUSE, vec, 0 ); 
		defaultFancyAxis.draw( gl );
		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, vec, 0 );		
	}
	
    /** create our own GLU... is there overhead for this? */
    private GLU glu = new GLU();
        
    double size;
    
    /**
     * Creates a new axis of size 1
     */
    public FancyAxis() {
        this( 1 );
    }
    
    /**
     * Creates a new axis of the desired size
     * @param size
     */
    public FancyAxis( double size ) {
        this.size = size;
    }
    
    /**
     * Sets the size of the axis 
     * @param size
     */
    public void setSize( double size ) {
        this.size = size;
    }
        
    /**
     * Draws the axis 
     * @param gl
     */
    public void draw( GL2 gl ) {
    	
    	gl.glPushMatrix();
    	gl.glScaled( size, size, size );
    	
            float [] ballCol = { 0.5f, 0.5f, 0.5f, 1 };            
            
            gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, ballCol, 0 );
            gl.glEnable( GL2.GL_LIGHTING );
            EasyViewer.glut.glutSolidSphere( 0.15, 20, 20 );
            
            gl.glPushMatrix();
            float [] xCol = { 1, 0, 0, 1 };
            gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, xCol, 0 );
            gl.glRotated( 90, 0, 1, 0 );
            drawArrow( gl );
            gl.glPopMatrix();
            
            gl.glPushMatrix();
            float [] yCol = { 0, 1, 0, 1 };
            gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, yCol, 0 );
            gl.glRotated( -90, 1, 0, 0 );
            drawArrow( gl );
            gl.glPopMatrix();
            
            gl.glPushMatrix();
            float [] zCol = { 0, 0, 1, 1 };
            gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, zCol, 0 );
            drawArrow( gl );
            gl.glPopMatrix();
           
        gl.glPopMatrix();
    }
    
    /**
     * Draws an arrow (i.e., one axis)
     * @param gl
     */
    public void drawArrow( GL2 gl ) {
        double r = 0.07;
        double h = 0.8;
        int nAround = 10;
        gl.glPushMatrix();
        
        GLUquadric cylinder = glu.gluNewQuadric();
        glu.gluQuadricNormals( cylinder, GLU.GLU_SMOOTH );
        glu.gluCylinder( cylinder, r, r, h, nAround, 1 );
        glu.gluDeleteQuadric( cylinder );
        
        gl.glTranslated( 0, 0, h );        
        EasyViewer.glut.glutSolidCone( r * 2, 1 - h, nAround, 5 );
        
        GLUquadric disk = glu.gluNewQuadric();
        glu.gluQuadricOrientation( disk, GLU.GLU_INSIDE );
        glu.gluDisk( disk, 0, r * 2, nAround, 1 );
        glu.gluDeleteQuadric( disk );
        
        gl.glPopMatrix();
    }
}
