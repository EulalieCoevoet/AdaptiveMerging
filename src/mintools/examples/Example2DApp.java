package mintools.examples;

import java.awt.Dimension;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.DoubleParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.SceneGraphNode;

public class Example2DApp implements SceneGraphNode {

    @Override
    public void display(GLAutoDrawable drawable) {
        
        GL2 gl = drawable.getGL().getGL2();
        
        // Probably better to completely avoid the 3D setup if we 
        // only want to draw in 2D, but this is a quick and easy way 
        // to draw in 2D
        EasyViewer.beginOverlay(drawable);
        
        gl.glColor4d( 0,1,0,alpha.getValue() );
        gl.glLineWidth(2);        
        gl.glBegin( GL.GL_LINE_STRIP );
        for ( int i = 0; i < 10; i++) {
            gl.glVertex2d( 200 + i*10, 100 +i*i );
        }
        gl.glEnd();
        
        gl.glColor4d( 1,0,0,alpha.getValue() );
        gl.glPointSize(5);
        gl.glBegin( GL.GL_POINTS );
        for ( int i = 0; i < 10; i++) {
            gl.glVertex2d( 200 + i*10, 100+i*i );
        }        
        gl.glEnd();
        
        
        // lets draw some 2D text
        gl.glColor4d( 1,1,1,1 );        
        EasyViewer.printTextLines( drawable,"(100,100)", 100, 100, 12, GLUT.BITMAP_HELVETICA_10 );
        gl.glRasterPos2d( 200, 200 );
        EasyViewer.printTextLines(drawable, "(200,200)\ncan have a second line of text",
                200, 200, 12, GLUT.BITMAP_HELVETICA_10 );
        
        EasyViewer.endOverlay(drawable);
        
    }

    DoubleParameter alpha = new DoubleParameter( "alpha value" , 0.5, 0, 1 );
    
    @Override
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Transparency") );
        vfp.add( alpha.getSliderControls(false) );
        CollapsiblePanel cp = new CollapsiblePanel( vfp.getPanel() );
        //cp.collapse();
        return cp;   
    }

    @Override
    public void init(GLAutoDrawable drawable) {
        // This will set up nice anti aliased lines and points
        GL gl = drawable.getGL();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
    }
    
    /**
     * @param args
     */
    public static void main(String[] args) {
        new EasyViewer("Example 1", new Example2DApp(), new Dimension(640,480), new Dimension(320,480) );
    }
    
}
