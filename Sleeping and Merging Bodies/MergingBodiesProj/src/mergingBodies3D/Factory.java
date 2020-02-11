package mergingBodies3D;

import java.util.ArrayList;
import java.util.Random;

import javax.swing.JPanel;
import javax.vecmath.Vector3d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.Vec3Parameter;
import mintools.swing.VerticalFlowPanel;

/**
 * RigidBody factory for stress testing.
 * 
 */
public class Factory {
    
    /** flags if this Factory is in use */
    boolean use = false;
    
    RigidBodySystem system;
    
    private ArrayList<RigidBody> pinnedBodies = new ArrayList<RigidBody>();
    
    private ArrayList<RigidBody> unpinnedBodies = new ArrayList<RigidBody>();

    /**
     * Creates a new factory for the provided system
     * @param system
     */
    public Factory( RigidBodySystem system ) {
        this.system = system;        
    }

    /**
     * Sets the system to use with this factory.
     * The free bodies will be created at regular intervals from
     * a seed location.
     * @param system
     */
    public void setSystem( RigidBodySystem system ) {
        pinnedBodies.clear();
        unpinnedBodies.clear();
        for ( RigidBody b : system.bodies ) {
            if ( b.pinned || ! b.factoryPart ) {
                pinnedBodies.add(b);
            } else {
                unpinnedBodies.add(b);
            }
        }
    }
        
    /**
     * Resets the system and the factory
     */
    public void reset() {
        elapsed = interval.getValue(); // make a body right away!
        rand.setSeed(0);
        system.bodies.clear();
    	RigidBody.nextIndex = 0;
        for ( RigidBody b : pinnedBodies ) {
        	system.bodies.add( b );
        }
        system.reset();
    }
    
    /** keeps track of elapsed time since last rigid body creation */
    private double elapsed = 0;
    
    private Random rand = new Random();
        
    /** a flag for requesting a new rigid body immediately */
    boolean createBodyRequest = false;
    
    /**
     * Advances the state of the factory.  Creates new rigid bodies at the 
     * specified interval in the controls.
     * @param dt
     */
    public void advanceTime( double dt ) {
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
    
    /** 
     * Creates a new body by copying a random body among those that were not pinned in the 
     * currently set image blocker.  Some random initial velocity is set.
     */
    private void generateBody() {     
        RigidBody body = new RigidBody( unpinnedBodies.get( rand.nextInt(unpinnedBodies.size())) );
        Vector3d tmp = new Vector3d( seed.x, seed.y, seed.z );
        tmp.x += (rand.nextFloat()*2-1) * spread.getValue();
        tmp.z += (rand.nextFloat()*2-1) * spread.getValue();
        body.x0.set( tmp );                        
        body.x.set( tmp );            
        body.theta.setIdentity(); 
        body.theta.rotZ(rand.nextDouble());
        body.omega.x = (2*rand.nextDouble() - 1) * angularVelocityScale.getValue();
        body.omega.y = (2*rand.nextDouble() - 1) * angularVelocityScale.getValue();
        body.omega.z = (2*rand.nextDouble() - 1) * angularVelocityScale.getValue();
        body.v.x = (2*rand.nextDouble()-1) * linearVelocityScale.getValue();  
        body.v.y =  downVelocity.getValue();
        body.v.z =  (2*rand.nextDouble()-1) * linearVelocityScale.getValue();  
        body.updateRotationalInertiaFromTransformation();
        system.add( body );
    }
    
    /** seed location for creating new rigid bodies */
    Vec3Parameter seed = new Vec3Parameter( "seed location", 0, 50, 0, 150 );    

    /** Specifies the width of the zone from which new objects will be dropped */
    DoubleParameter spread = new DoubleParameter("drop zone width", 3, 0, 60 );

    /** When run is true, the factory will create objects at a regular interval */
    BooleanParameter run = new BooleanParameter("run factory", true );
    
    /** Interval between body creation events */
    DoubleParameter interval = new DoubleParameter("delay", 1, 0.2, 10);

    /** Downward velocity of new bodies */
    DoubleParameter downVelocity = new DoubleParameter( "down velocity", -1, -10, 10 );

    /** For controlling the initial linear velocity of new bodies */
    DoubleParameter linearVelocityScale = new DoubleParameter( "linear velocity perturbation scale", 0.5, 0.1, 5 );
    
    /** For controlling the initial angular velocity of new bodies */
    DoubleParameter angularVelocityScale = new DoubleParameter( "angular velocity perturbation scale", 0.3, 0.1, 10 );

    /**
     * Gets a control panel for the factory
     * @return control panel
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        //vfp.setBorder( new TitledBorder("Factory") );
        vfp.add( run.getControls() );       
        vfp.add( interval.getSliderControls(true) );
        vfp.add( seed );
        vfp.add( spread.getSliderControls(false) );
        vfp.add( linearVelocityScale.getSliderControls(true) );
        vfp.add( downVelocity.getSliderControls(false) );
        vfp.add( angularVelocityScale.getSliderControls(true) );
        //CollapsiblePanel cp = new CollapsiblePanel( vfp.getPanel());
        //cp.collapse();
        //return cp;
        return vfp.getPanel();
    }
    
}
