package mergingBodies2D;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.DoubleParameter;
import mintools.swing.VerticalFlowPanel;

/**
 * Mouse spring for interacting with rigid bodies
 */
public class MouseSpringForce {

    private RigidBody picked = null;
    
    private Point2d grabPointB = new Point2d();
    
    private Point2d point;
    
    /**
     * Creates a new mouse spring, where the provided point will be updated with movement of the mouse
     * @param point
     */
    public MouseSpringForce( Point2d point ) {
        this.point = point;
    }
    
    /**
     * Sets the picked body and the point on that body
     * @param picked
     * @param grabPointB
     */
    public void setPicked( RigidBody picked, Point2d grabPointB ) {
        this.picked = picked;
        this.grabPointB.set( grabPointB );        
    }
    
    /**
     * Gets the current picked body
     * @return
     */
    public RigidBody getPicked() {
    	return picked;
    }
    
    /**
     * Applies the mouse spring force to the picked rigid body, or nohting if no body selected
     */
    public void apply() {
        if ( picked == null ) return;
        
        picked.wake();
        
        Point2d grabPointW = new Point2d();
        Vector2d grabPointV = new Vector2d();
        picked.transformB2W.transform( grabPointB, grabPointW );
        double distance = grabPointW.distance( point );
        double k = stiffness.getValue();
        double c = damping.getValue();
        
        Vector2d force = new Vector2d();
        Vector2d direction = new Vector2d();
        direction.sub( point, grabPointW );
        if ( direction.lengthSquared() < 1e-3 ) return;
        direction.normalize();
        force.scale( distance * k, direction );

    	if (picked.isInCollection()) picked.parent.applyForceW( grabPointW, force );
        picked.applyForceW( grabPointW, force );

        // spring damping forces
        if (picked.isInCollection()) picked.parent.getSpatialVelocity( grabPointW, grabPointV );
        else picked.getSpatialVelocity( grabPointW, grabPointV );
        force.scale( - grabPointV.dot( direction ) * c, direction );
        
        if (picked.isInCollection()) picked.parent.applyForceW( grabPointW, force ); 
        picked.applyForceW( grabPointW, force ); 
    }
    
    /**
     * Stiffness of mouse spring
     */
    public DoubleParameter stiffness = new DoubleParameter("mouse stiffness", 10, 1, 1e4 );
    
    /**
     * Viscous damping coefficient for the mouse spring
     */
    public DoubleParameter damping = new DoubleParameter("mouse spring damping", 0, 0, 100 );
    
    /**
     * @return controls for the collision processor
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Mouse Spring Controls") );
        vfp.add( stiffness.getSliderControls(true) );
        vfp.add( damping.getSliderControls(false) );
        return vfp.getPanel();
    }
        
}
