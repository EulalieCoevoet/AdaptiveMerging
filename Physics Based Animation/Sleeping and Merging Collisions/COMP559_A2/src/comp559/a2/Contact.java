package comp559.a2;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import no.uib.cipr.matrix.DenseVector;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Implementation of a contact constraint.
 * @author kry
 */
public class Contact {

    /** Next available contact index, used for determining which rows of the jacobian a contact uses */
    static public int nextContactIndex = 0;
    
    /** Index of this contact, determines its rows in the jacobian */
    int index;
    
    /** visitID of this contact at this timestep. */
    boolean visited = false;
    
    /**    If the contact is between two sleeping bodies or not  */
    boolean passive = false;
    
    boolean precomputer = false;
    /** First RigidBody in contact */
    RigidBody body1;
    
    /** Second RigidBody in contact */
    RigidBody body2;
    
    //The two blocks within the bodies that caused the collision
    Block block1;
    
    Block block2;
    
    /** Contact normal in world coordinates. GOES FROM BODY1 to BODY2*/
    Vector2d normal = new Vector2d();
    
    /** Position of contact point in world coordinates */
    Point2d contactW = new Point2d();
    // for normal
    DenseVector j_1 = new DenseVector(6);
    //for tangential
    DenseVector j_2 = new DenseVector(6);
    
    BVNode bvn1;
    
    BVNode bvn2;
    //vector points from body 2 to body 1, magnitude is the amount of overlap.
    double constraint_violation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
    
    Double[] lamdas = new Double[2];
    
    Vector2d relativeVelocity = new Vector2d();
    
    double relativeAngularVelocity = 0;
    
    BodyContact b1;
    
    BodyContact b2;
    /**
     * Creates a new contact, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW
     * @param normal
     */
    public Contact( RigidBody body1, RigidBody body2, Point2d contactW, Vector2d normal, Block b1, Block b2, double distance) {
        this.body1 = body1;
        this.body2 = body2;
        this.contactW.set( contactW );
        this.normal.set( normal );        
        index = nextContactIndex++;        
      
        Vector2d contact_point = new Vector2d(contactW);
   
		Point2d radius_i_body_1 = new Point2d(body1.x);
		Point2d radius_i_body_2 = new Point2d(body2.x);
		//if body is a merged one, it's x is in the collection frame... must transform into world
		if (body1.parent != null) {
			radius_i_body_1.set(body1.parent.x);//body1.parent.transformB2W.transform(radius_i_body_1);
			
		}
		if (body2.parent != null) {
			radius_i_body_2.set(body2.parent.x);//body2.parent.transformB2W.transform(radius_i_body_2);
		}
		radius_i_body_1.sub(contact_point, radius_i_body_1);
		radius_i_body_2.sub(contact_point, radius_i_body_2);
		
		Vector2d tangeant = new Vector2d(-normal.y, normal.x);
		
		Vector2d r1 = new Vector2d(-radius_i_body_1.y, radius_i_body_1.x);
		Vector2d r2 = new Vector2d(-radius_i_body_2.y, radius_i_body_2.x);
		j_1.set(0, -normal.x);
		j_1.set(1, -normal.y);
		j_1.set(2, - r1.dot(normal));
		j_1.set(3, normal.x);
		j_1.set(4, normal.y);
		j_1.set(5, r2.dot(normal));
		
		j_2.set(0, -tangeant.x);
		j_2.set(1, -tangeant.y);
		j_2.set(2, - r1.dot(tangeant));
		j_2.set(3, tangeant.x);
		j_2.set(4, tangeant.y);
		j_2.set(5, r2.dot(tangeant));
		
		
		block1 = b1;
		block2 = b2;
		
	  
		constraint_violation =  distance - 2*Block.radius;
		
		relativeVelocity.sub(body2.v, body1.v);
		relativeAngularVelocity = body2.omega - body1.omega;
    }
    
    public double getRelativeMetric() {
    	double k = 0.5*relativeVelocity.lengthSquared() + 0.5*relativeAngularVelocity*relativeAngularVelocity;
    	return k;
    }
    
    
    /**
     * Draws the contact points
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glPointSize(3);
        gl.glColor3f(.7f,0,0);
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex2d(contactW.x, contactW.y);
        gl.glEnd();
    }
    
    /**
     * Draws the connections between bodies to visualize the 
     * the adjacency structure of the matrix as a graph.
     * @param drawable
     */
    public void displayConnection( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        // draw a line between the two bodies but only if they're both not pinned
        if ( !body1.pinned && ! body2.pinned ) {
            gl.glLineWidth(2);
            gl.glColor4f(0,.3f,0, 0.5f);
            gl.glBegin( GL.GL_LINES );
            Point2d p1 = new Point2d(body1.x);
            Point2d p2 = new Point2d(body2.x);
            if (body1.parent!=null) {
            	p1.set(body1.parent.x);//body1.parent.transformB2W.transform(p1);
            }
           	if (body2.parent!=null) {
           		p2.set(body2.parent.x);//body2.parent.transformB2W.transform(p2);
           	}
            gl.glVertex2d(p1.x, p1.y);
            gl.glVertex2d(p2.x, p2.y);
            gl.glEnd();
        }
    }
    

    
}
