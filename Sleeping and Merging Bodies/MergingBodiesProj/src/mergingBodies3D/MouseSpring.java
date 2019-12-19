package mergingBodies3D;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.viewer.FlatMatrix4d;

/**
 * Spring to pull an ODE body towards a desired position controlled by the mouse
 * @author kry
 */
public class MouseSpring {
    
    /** Spring stiffness, sometimes written k_s in equations */
    public static double k;
        
    /** Spring damping (along spring direction), sometimes written k_d in equations */
    public static double c;

    /**
     * selected body which is being pulled by this mouse spring 
     */
    RigidBody body;
    
    /** End of spring in world, in world coordinates */
    Point3d point = new Point3d();
        
    /** End of spring on the body in world coordinates */
    private Point3d bodyPoint = new Point3d();
    
    /**
     * Creates a spring between a point in the world and a selected body
     */
    public MouseSpring( ) {
    	// do nothing
    }
    
    /**
     * computes the ends of the spring in the world
     */
    void updateSpringEnds() {
    	final FlatMatrix4d EwfromRB = new FlatMatrix4d();
    	//ODETools.setFlatMatrix( body.body, EwfromRB );
    	Matrix4d A = EwfromRB.getBackingMatrix();
    	//A.transform( body.selectedPoint, bodyPoint );
    }
        
    /**
     * Applies the spring force by adding a force to the body
     */
    public void apply() {
    	if ( body == null ) return;
    	updateSpringEnds();
    	final Vector3d L = new Vector3d();
    	final Vector3d v = new Vector3d();
    	L.sub( point, bodyPoint );
    	double springF = k * L.length(); // zero rest length spring
        L.normalize();
        //ODETools.setVecmathFromODE(v, body.body.getLinearVel() );
        double springFd = - c * L.dot(v);
        L.scale( springF + springFd );
        //final DVector3 f = new DVector3();
        //f.set( L.x, L.y, L.z );
    	//body.body.addForce(f);
    	
        // should also have a torque on the body based on the grab position!
    	// but perhaps doesn't matter so much in the context of a mouse interface??
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
        gl.glVertex3d(bodyPoint.x, bodyPoint.y, bodyPoint.z);
        gl.glEnd();
        gl.glEnable( GL2.GL_LIGHTING );
    }
}
