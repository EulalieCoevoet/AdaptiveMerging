package mintools.examples;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import mintools.parameters.BooleanParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.BoxRoom;
import mintools.viewer.EasyViewer;
import mintools.viewer.SceneGraphNode;
import mintools.viewer.ShadowMap;
import mintools.viewer.TrackBallCamera;

/**
 * A simple example that draws a teapot in a box room
 * @author kry
 */
public class ExampleShadowApp implements SceneGraphNode {

	EasyViewer ev;
    
    ShadowMap sm = new ShadowMap(1024);
	
    /**
     * main entry point
     * @param args
     */
    public static void main( String args[] ) {
        new ExampleShadowApp();
    }
    
    public ExampleShadowApp() {
    	ev = new EasyViewer("Example 1", this );
	}
    
    @Override
    public void display(GLAutoDrawable drawable) {       
    	TrackBallCamera tbc = ev.trackBall;
    	sm.beginLightPass( drawable, tbc );
    	drawAllObjects( drawable );        
    	sm.endLightPass( drawable, tbc );    	 
    	sm.beginShadowMapping( drawable, tbc ); 
    	drawAllObjects( drawable );
    	sm.endShadowMapping( drawable, tbc );
    }

    public void drawAllObjects(GLAutoDrawable drawable) {        
        GL gl = drawable.getGL();
        if ( drawBoxRoom.getValue() ) {
        	boxRoom.display(drawable);
        }
        if ( cull.getValue() ) {
            gl.glEnable( GL.GL_CULL_FACE );
        } else {
            gl.glDisable( GL.GL_CULL_FACE );
        }   
        gl.glFrontFace( GL.GL_CCW );
        EasyViewer.glut.glutSolidTeapot(1);
    }
    
    BoxRoom boxRoom = new BoxRoom();
    BooleanParameter cull = new BooleanParameter("cull face", false );
    BooleanParameter drawBoxRoom = new BooleanParameter( "draw box room", true );
    
    @Override
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("controls") );
        vfp.add( cull.getControls() );
        vfp.add( drawBoxRoom.getControls() );
        vfp.add( sm.getControls() );
        CollapsiblePanel cp = new CollapsiblePanel( vfp.getPanel() );
        cp.collapse();
        return cp;        
    }

    @Override
    public void init(GLAutoDrawable drawable) {
    	sm.init(drawable);
    }

}
