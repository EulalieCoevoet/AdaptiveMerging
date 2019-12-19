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

    RigidBody picked = null;
    
    private Point3d grabPointB = new Point3d();
    
    private Point3d grabPointW = new Point3d();

    Point3d point;
    
    /**
     * Creates a new mouse spring, where the provided point will be updated with movement of the mouse
     * @param point
     */
    public MouseSpringForce( Point3d point ) {
        this.point = point;
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
        picked.transformB2W.transform( grabPointB, grabPointW );
        double distance = grabPointW.distance( point );
        double k = stiffness.getValue();
        double c = damping.getValue();
        
        Vector3d force = new Vector3d();
        Vector3d direction = new Vector3d();
        direction.sub( point, grabPointW );
        if ( direction.lengthSquared() < 1e-3 ) return;
        direction.normalize();
        force.scale( distance * k, direction );
        
        picked.applyContactForceW( grabPointW, force );
        
        // spring damping forces
        picked.getSpatialVelocity( grabPointW, grabPointV );
        force.scale( - grabPointV.dot( direction ) * c, direction );
        picked.applyContactForceW( grabPointW, force );        
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
    	gl.glColor4d( 1, 1, 1, 0.5 );
    	gl.glLineWidth(3);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(point.x, point.y, point.z);
        gl.glVertex3d( grabPointW.x, grabPointW.y, grabPointW.z);
        gl.glEnd();
        gl.glEnable( GL2.GL_LIGHTING );
    }
        
}
