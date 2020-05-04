package mergingBodies2D;

import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.swing.VerticalFlowPanel;

/**
 * RigidBody factory for stress testing.
 */
public class Factory {
    
    /** flags if this Factory is in use */
    boolean use = false;
    
    RigidBodySystem system;
    
    public ArrayList<RigidBody> pinnedBodies = new ArrayList<RigidBody>();
    public ArrayList<RigidBody> unpinnedBodies = new ArrayList<RigidBody>();

    /**
     * Creates a new factory for the provided system
     * @param system
     */
    public Factory( RigidBodySystem system ) {
        this.system = system;        
    }

    /**
     * Sets the image blocker to use with this factory.
     * The free bodies will be created at regular intervals from
     * a seed location above the top center of the image.
     * @param blocker
     */
    public void setImageBlocker( ImageBlocker blocker ) {
        pinnedBodies.clear();
        unpinnedBodies.clear();
        for ( RigidBody b : blocker.bodies ) {
            if ( b.pinned || !b.springs.isEmpty() ) {
                pinnedBodies.add(b);
            } else {
                unpinnedBodies.add(b);
            }
        }
        seed.set( blocker.width*0.5, -blocker.height*0.2 );
    }
        
    /**
     * Resets the system and the factory
     */
    public void reset() {
        elapsed = interval.getValue(); // make a body right away!
        rand.setSeed(0);
        system.clear();
        for ( RigidBody b : pinnedBodies ) {
           // system.bodies.add( new RigidBody(b) );
        	system.bodies.add(b);
        }
    }
    
    /** keeps track of elapsed time since last rigid body creation */
    private double elapsed = 0;
    
    private Random rand = new Random();
    
    /** seed location for creating new rigid bodies */
    private Point2d seed = new Point2d(0,0);    
    
    /** a flag for requesting a new rigid body immediately */
    boolean createBodyRequest = false;
    
    /**
     * Advances the state of the factory.  Creates new rigid bodies at the 
     * specified interval in the controls.
     * @param dt
     */
    public void advanceTime( double dt ) {
    	removeLowAltitudeBody();
    	if ( createBodyRequest == true ) {
            createBodyRequest = false;
            generateBody();
        }
        
        if ( ! run.getValue() ) return;
        
        elapsed += dt;       
        
        if ( elapsed >= interval.getValue() ) {
            elapsed = 0;
            generateBody();
        }
    }
    
    /*
     * Removes body from system body list if body is below y threshold.
     */
	private void removeLowAltitudeBody() {
		ArrayList<RigidBody> removalQueue = new ArrayList<RigidBody>();
        for (RigidBody b : system.bodies) {
        	if (b.x.y > minY.getValue() && !b.pinned && b.springs.isEmpty()) {
        		removalQueue.add(b);
        	}
        }
        for (RigidBody b : removalQueue) {
        	system.bodies.remove(b);
        }
	}

    
    /** 
     * Creates a new body by copying a random body among those that were not pinned in the 
     * currently set image blocker.  Some random initial velocity is set.
     */
    private void generateBody() {     
        RigidBody body = new RigidBody( unpinnedBodies.get( rand.nextInt(unpinnedBodies.size())) );
        Vector2d tmp = new Vector2d( seed );
        tmp.x += ((system.bodies.size()*2)%5 - 2) * spread.getValue();
        body.x0.set( tmp );                        
        body.x.set( tmp );            
        body.theta = rand.nextDouble() * Math.PI*2;
        body.omega = (2*rand.nextDouble() - 1) * angularVelocityScale.getValue();
        body.v.x = (2*rand.nextDouble()-1) * linearVelocityScale.getValue();  
        body.v.y =  downVelocity.getValue();
        body.updateTransformations();
        system.bodies.add( body );
    }
    
    /** Specifies the width of the zone from which new objects will be dropped */
    DoubleParameter spread = new DoubleParameter("drop zone width", 15, 0, 60 );

    /** When run is true, the factory will create objects at a regular interval */
    BooleanParameter run = new BooleanParameter("run factory", true );
    
    /** Interval between body creation events */
    DoubleParameter interval = new DoubleParameter("delay", 5, 0.2, 10);

    /** Downward velocity of new bodies */
    DoubleParameter downVelocity = new DoubleParameter( "down velocity", 10, 0.1, 20 );

    /** For controlling the initial linear velocity of new bodies */
    DoubleParameter linearVelocityScale = new DoubleParameter( "linear velocity perturbation scale", 2.5, 0.1, 10 );
    
    /** For controlling the initial angular velocity of new bodies */
    DoubleParameter angularVelocityScale = new DoubleParameter( "angular velocity perturbation scale", 0.3, 0.1, 10 );

    
    /** Clears all bodies below this y value*/
    DoubleParameter minY = new DoubleParameter( "Min y value", 300 , -300, 500 );

    
    /**
     * Gets a control panel for the factory
     * @return control panel
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.add( run.getControls() );
        vfp.add( interval.getSliderControls(true) );
        vfp.add( spread.getSliderControls(false) );
        vfp.add( linearVelocityScale.getSliderControls(true) );
        vfp.add( downVelocity.getSliderControls(true) );
        vfp.add( angularVelocityScale.getSliderControls(true) );
        vfp.add( minY.getSliderControls(false));
        
        return vfp.getPanel();
    }
    
}
