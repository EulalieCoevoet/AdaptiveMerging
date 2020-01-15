package mergingBodies3D;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;
import mintools.swing.VerticalFlowPanel;

public class MouseImpulse {

    private RigidBody pickedBody = null;
    
    /** End point in world coordinate */
    private Point3d endPointW = new Point3d();
    
    /** Picked point in body coordinate */
    private Point3d pickedPointB = new Point3d();
    
    /** Picked point in world coordinate */
    private Point3d pickedPointW = new Point3d();
    
    private Vector3d force = new Vector3d();
    
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
     * @param pointB in body coordinates
     */
    public void grab( RigidBody picked, Point3d pointB ) {
    	if (picked != null) {
	        pickedBody = picked;  
	        pickedPointB.set(pointB); 
	        pickedBody.transformB2W.transform(pointB, pickedPointW);
    	}
    }
    
    /**
     * Sets the end point
     * @param pointW in world coordinate
     */
    public void hold( Point3d pointW ) {
    	endPointW.set(pointW);
    }
    
    public void release() {
    	released = true;
    }
    
    public boolean isGrabbing() {
    	return (pickedBody != null);
    }
    
    public RigidBody getPickedBody() {
    	return pickedBody;
    }
    
    /** Get picked point in body coordinate */
    public Point3d getPickedPoint() {
    	return pickedPointB;
    }
    
    /** Get picked point in world coordinate */
    public Point3d getPickedPointW() {
    	pickedBody.transformB2W.transform(pickedPointB, pickedPointW);
    	return pickedPointW;
    }
    
    public Point3d getEndPoint() {
    	return endPointW;
    }
    
    public Vector3d getForce() {
    	return force;
    }
    
    /**
     * Applies the mouse impulse to the picked rigid body, or nothing if no body selected
     */
    public void apply() {
        if ( pickedBody == null || !released) return;
        
        pickedBody.wake();
        pickedBody.picked = true;
        
        pickedBody.transformB2W.transform(pickedPointB, pickedPointW);
        double distance = endPointW.distance( pickedPointW );
        
        Vector3d force = new Vector3d();
        Vector3d direction = new Vector3d();
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
     * Draws a transparent line between the points connected by this spring.
     * @param drawable
     */
    public void display(GLAutoDrawable drawable) {
    	GL2 gl = drawable.getGL().getGL2();
    	gl.glDisable( GL2.GL_LIGHTING );
    	gl.glColor4d( 1, 0.5,0, 1 );
    	gl.glLineWidth(3);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d( endPointW.x, endPointW.y, endPointW.z );
        pickedBody.transformB2W.transform(pickedPointB, pickedPointW);
        gl.glVertex3d( pickedPointW.x, pickedPointW.y, pickedPointW.z);
        gl.glEnd();
        gl.glEnable( GL2.GL_LIGHTING );
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
