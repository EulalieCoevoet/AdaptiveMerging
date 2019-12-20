package mergingBodies3D;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.DoubleParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

/**
 * Mouse spring for interacting with rigid bodies
 * @author kry
 */
public class MouseSpringForce {

	/** The selected body will be null if there is no mouse spring active */
    RigidBody picked = null;
    
    /** Grab point on body in body coords */
    Point3d grabPointB = new Point3d();
    
    /** Grab point on body in world coords */
    Point3d grabPointBW = new Point3d();    
    
    /** Mouse point in world */
    Point3d pointW = new Point3d();
    
    /**
     * Creates a new mouse spring, where the provided point will be updated with movement of the mouse
     * @param point
     */
    public MouseSpringForce(  ) {
    }
    
    /**
     * Sets the picked body and the point on that body
     * @param picked
     * @param grabPointB
     */
    public void setPicked( RigidBody picked, Point3d grabPointB ) {
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
        
        Vector3d grabPointV = new Vector3d();
        picked.transformB2W.transform( grabPointB, grabPointBW );
        double distance = grabPointBW.distance( pointW );
        double k = stiffness.getValue();
        double c = damping.getValue();
        
        Vector3d force = new Vector3d();
        Vector3d direction = new Vector3d();
        direction.sub( pointW, grabPointBW );
        if ( direction.lengthSquared() < 1e-3 ) return;
        direction.normalize();
        force.scale( distance * k, direction );
        
        picked.applyContactForceW( grabPointBW, force );
        
        // spring damping forces
        picked.getSpatialVelocity( grabPointBW, grabPointV );
        force.scale( - grabPointV.dot( direction ) * c, direction );
        
        picked.applyContactForceW( grabPointBW, force );        
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
        CollapsiblePanel cp = new CollapsiblePanel(vfp.getPanel());
        cp.collapse();
        return cp;
    }
    
    /**
     * Draws a transparent line between the points connected by this spring.
     * @param drawable
     */
    public void display(GLAutoDrawable drawable) {
    	GL2 gl = drawable.getGL().getGL2();
    	gl.glDisable( GL2.GL_LIGHTING );
    	gl.glColor4d( 1, 0,0, 0.5 );
    	gl.glLineWidth(3);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d( pointW.x, pointW.y, pointW.z );
        picked.transformB2W.transform( grabPointB, grabPointBW );
        gl.glVertex3d( grabPointBW.x, grabPointBW.y, grabPointBW.z);
        gl.glEnd();
        gl.glEnable( GL2.GL_LIGHTING );
    }
        
}
