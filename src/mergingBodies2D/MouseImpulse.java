package mergingBodies;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.DoubleParameter;
import mintools.swing.VerticalFlowPanel;

public class MouseImpulse {

    private RigidBody pickedBody = null;
    
    /** End point in world coordinate */
    private Point2d endPointW = new Point2d();
    
    /** Picked point in body coordinate */
    private Point2d pickedPointB = new Point2d();
    
    /** Picked point in world coordinate */
    private Point2d pickedPointW = new Point2d();
    
    private Vector2d force = new Vector2d();
    
    public boolean released = false;
   
    /**
     * Creates a new mouse impulse, where the provided point will be updated with movement of the mouse
     * @param pickedPointB
     */
    public MouseImpulse() {
    }
    
    /**
     * Sets the picked body and the point on that body
     * @param picked
     * @param point
     */
    public void grab( RigidBody picked, Point2d point ) {
    	if(picked != null) {
	        pickedBody = picked;  
	        pickedPointB.set(point);
	        pickedBody.transformB2W.transform(pickedPointB, pickedPointW);
    	}
    }
    
    public void release( Point2d point ) {
    	released = true;
    	endPointW.set(point); 
    }
    
    public boolean isGrabbing() {
    	return (pickedBody != null);
    }
    
    public RigidBody getPickedBody() {
    	return pickedBody;
    }
    
    public Point2d getPickedPoint() {
    	return pickedPointB;
    }
    
    public Vector2d getForce() {
    	return force;
    }
    
    /**
     * Applies the mouse impulse to the picked rigid body, or nothing if no body selected
     */
    public void apply() {
        if ( pickedBody == null || !released) return;
        
        pickedBody.wake();

        pickedBody.transformB2W.transform(pickedPointB, pickedPointW);
        double distance = endPointW.distance( pickedPointW );
        
        Vector2d force = new Vector2d();
        Vector2d direction = new Vector2d();
        direction.sub( pickedPointW, endPointW );
        if ( direction.lengthSquared() < 1e-3 ) return;
        direction.normalize();
        force.scale( scale.getValue()*distance, direction );
                
    	if (pickedBody.isInCollection()) pickedBody.parent.applyForceW( pickedPointW, force );
        pickedBody.applyForceW( pickedPointW, force );
        this.force.set(force);
        
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
        return vfp.getPanel();
    }
}
