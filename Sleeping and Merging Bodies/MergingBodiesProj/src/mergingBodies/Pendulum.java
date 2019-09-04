package mergingBodies;

import java.awt.Component;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

public class Pendulum {
	
	Boolean use = false;
	
    RigidBodySystem system;
    
    private ArrayList<RigidBody> pinnedBodies = new ArrayList<RigidBody>();
    
    private ArrayList<RigidBody> unpinnedBodies = new ArrayList<RigidBody>();

    private Random rand = new Random();
    
 
    /** a flag for requesting a new rigid body immediately */
    boolean createBodyRequest = false;
    
    
	


    /**
     * Creates a new pendulum for the provided system
     * @param system
     */
    public Pendulum( RigidBodySystem system ) {
        this.system = system;    
        
    }
    
    /**
     * Resets the system and the pendulum
     */
    public void reset() {
        
        rand.setSeed(0);
        system.clear();
        system.pendulumProcessor.constraints.clear();
        
        for ( RigidBody b : pinnedBodies ) {
            system.bodies.add( new RigidBody(b) );
        }
    }
    /**
     * Sets the image blocker to use with this pendulum.
     *
     * @param blocker
     */
    
    public void setImageBlocker( ImageBlocker blocker ) {
        pinnedBodies.clear();
        unpinnedBodies.clear();
        for ( RigidBody b : blocker.bodies ) {
            if ( b.pinned ) {
                pinnedBodies.add(b);
            } else {
                unpinnedBodies.add(b);
            }
        }
       
    }
    
    //generates a body a distance l away from the pendulum origin.
    public void generateBody() {     
        RigidBody body = new RigidBody( unpinnedBodies.get( rand.nextInt(unpinnedBodies.size())) );
       
        double angle = Math.PI*this.theta.getValue()/180;
        double length = pendulum_length.getValue();
        double o_x = origin_x.getValue();
        double o_y = origin_y.getValue();
        Point2d position = new Point2d(length*Math.cos(angle) + o_x , length*Math.sin(angle) + o_y);
     
        body.x0.set(position);                        
        body.x.set(position);            
        body.theta = 0;
        body.omega = 0;
        body.v.x = 0;
        body.v.y = 0;
        body.updateTransformations();

        system.add( body );
    }
    
	  /** x value of origin of pendulum */
    public static DoubleParameter origin_x = new DoubleParameter("x position of pendulum origin", 40, 10, 100 );
    
    /** y value of origin of pendulum */
    public static DoubleParameter origin_y = new DoubleParameter("y position of pendulum origin", 0, -100, 100 );
    
    /** Pendulum length */
    public static DoubleParameter pendulum_length = new DoubleParameter("length of pendulum", 3, 0, 100 );
    
    /** y value of origin of pendulum in degrees*/
    private DoubleParameter theta = new DoubleParameter("where around the pendulum will this body be created", 0, 0, 360 );
    
    
	public JPanel getControls() {
		  VerticalFlowPanel vfp = new VerticalFlowPanel();
	      vfp.setBorder( new TitledBorder("Pendulum  Controls") );
	      
	      
	      vfp.add( origin_x.getSliderControls(false) );
	      vfp.add( origin_y.getSliderControls(false) );
	      vfp.add( pendulum_length.getSliderControls(false) );
	      
	      vfp.add( theta.getSliderControls(false) );
	        
	      return vfp.getPanel();
	    
	}

	public void displayOrigin(GLAutoDrawable drawable) {
		
		GL2 gl = drawable.getGL().getGL2();
        gl.glPointSize(12);
        gl.glColor3f(1, 0, 0);
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex2d(origin_x.getValue(), origin_y.getValue());
        gl.glEnd();
        gl.glPointSize(4);
        gl.glColor3f(1, 0, 0);
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex2d(origin_x.getValue(), origin_y.getValue());
        gl.glEnd();
	}



}
