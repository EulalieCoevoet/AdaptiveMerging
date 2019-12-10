package mergingBodies;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.DoubleParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

public class MouseImpulse {

    private RigidBody pickedBody = null;
    
    private Point2d endPoint = new Point2d();
    
    private Point2d pickedPoint = new Point2d();
    
    public boolean released = false;
    
    /**
     * Creates a new mouse impulse, where the provided point will be updated with movement of the mouse
     * @param pickedPoint
     */
    public MouseImpulse() {
    }
    
    /**
     * Sets the picked body and the point on that body
     * @param picked
     * @param point
     */
    public void grab( RigidBody picked, Point2d point ) {
        pickedBody = picked;  
        pickedPoint = new Point2d(point);
    }
    
    public void release( Point2d point ) {
    	released = true;
    	endPoint = new Point2d(point); 
    }
    
    public boolean isGrabbing() {
    	return (pickedBody != null);
    }
    
    /**
     * Applies the mouse impulse to the picked rigid body, or nothing if no body selected
     */
    public void apply() {
        if ( pickedBody == null || !released) return;
        
        pickedBody.wake();
        
        double distance = endPoint.distance( pickedPoint );
        
        Vector2d force = new Vector2d();
        Vector2d direction = new Vector2d();
        direction.sub( pickedPoint, endPoint );
        if ( direction.lengthSquared() < 1e-3 ) return;
        direction.normalize();
        force.scale( scale.getValue()*distance, direction );
                
    	if (pickedBody.isInCollection()) pickedBody.parent.applyForceW( pickedPoint, force );
        pickedBody.applyForceW( pickedPoint, force );
        
        // Apply only once
        released = false;
        pickedBody = null;
    }
    
    /**
     * Mouse impulse scale
     */
    public DoubleParameter scale = new DoubleParameter("mouse impulse scale", 100, 1, 1e4 );
    
    /**
     * @return controls for the collision processor
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Mouse Impulse Controls") );
        vfp.add( scale.getSliderControls(true) );
        CollapsiblePanel cp = new CollapsiblePanel(vfp.getPanel());
        cp.collapse();
        return cp;
    }
}
