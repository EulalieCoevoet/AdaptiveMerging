package src;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import no.uib.cipr.matrix.DenseVector;

import java.util.ArrayList;

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
    
    /** First RigidBody in contact, only relevant if body1 is a parent. this is the child of that parent that is doing the real collision */
    RigidBody subBody1;
    
    /** Second RigidBody in contact, only relevant if body1 is a parent. this is the child of that parent that is doing the real collision */
    RigidBody subBody2;
    
    //The two blocks within the bodies that caused the collision
    Block block1;
    
    Block block2;
    
    /** Contact normal in world coordinates. GOES FROM BODY1 to BODY2*/
    Vector2d normal = new Vector2d();
    
    
    /** Contact normal in Body1 coordinates. GOES FROM BODY1 to BODY2*/
    Vector2d normalB1 = new Vector2d();
    
    /** Contact normal in Body2 coordinates. GOES FROM BODY1 to BODY2*/
    Vector2d normalB2 = new Vector2d();
    
    /**contact force being applied by this contact on body1*/
    Vector2d contactForceB1 = new Vector2d();
    
    /**contact torque being applied by this contact on body1*/
    double contactTorqueB1 = 0;
    
    /** Contact force being applied by this contact on body2*/
    Vector2d contactForceB2 = new Vector2d();
   
    /**contact torque being applied by this contact on body2*/
    double contactTorqueB2 = 0;
    
    /** Position of contact point in world coordinates */
    Point2d contactW = new Point2d();
    // for normal
    DenseVector j_1 = new DenseVector(6);
    //for tangential
    DenseVector j_2 = new DenseVector(6);
    
    Vector2d lamda = new Vector2d();
    BVNode bvn1;
    
    BVNode bvn2;
    
    //vector points from body 2 to body 1, magnitude is the amount of overlap.
    double constraint_violation; // in this case the constraint violation is the amount of overlap two bodies have when they are determined to be in contact
 
    Vector2d relativeVelocity = new Vector2d();
    
    double relativeAngularVelocity = 0;
    
    //Pointer to the BodyContact this contact is a part of. 
    BodyContact bc;
    
    //history of the last (max N) timesteps for this contact.
    public ArrayList<Vector2d> body1ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body1ContactTorqueHistory = new ArrayList<Double>();
	
	public ArrayList<Vector2d> body2ContactForceHistory = new ArrayList<Vector2d>();
	public ArrayList<Double> body2ContactTorqueHistory = new ArrayList<Double>();
	
	
	
    /**
     * Creates a new contact, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW
     * @param normal
     */
    public Contact( RigidBody body1, RigidBody body2, Point2d contactW, Vector2d normal, Block b1, Block b2, double distance,  RigidBody sBody1, RigidBody sBody2 ) {
        this.body1 = body1;
        this.body2 = body2;
        this.contactW.set( contactW );
        this.normal.set( normal );        
        index = nextContactIndex++;        
      
        Vector2d contact_point = new Vector2d(contactW);
   
		Point2d radius_i_body_1 = new Point2d(body1.x);
		Point2d radius_i_body_2 = new Point2d(body2.x);
	
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
		subBody1 = sBody1;
		subBody2 = sBody2;
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
            if (body1.parent != null) body1.parent.transformB2W.transform(p1);
            // advance the system by the given time step
            if (body2.parent != null) body2.parent.transformB2W.transform(p2);
            
     
            gl.glVertex2d(p1.x, p1.y);
            gl.glVertex2d(p2.x, p2.y);
            gl.glEnd();
        }
    }
    
    /**
     * Draws the connections between bodies to visualize the 
     * the adjacency structure of the matrix as a graph.
     * @param drawable
     */
    public void drawContactForce( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        // draw a line between the two bodies but only if they're both not pinned
        Point2d p1 = new Point2d(block1.pB);
        Point2d p2 = new Point2d(block2.pB);
      
        subBody1.transformB2W.transform(p1); 
        subBody2.transformB2W.transform(p2);
        
        subBody1.transformB2W.transform(contactForceB1);
        subBody2.transformB2W.transform(contactForceB2);

            double scale = 0.05;
 
            
            
            
          //  drawArrowHeads(gl, p2, normal2);
            gl.glLineWidth(2);
            gl.glColor4f(1, 0, 0, 1);
            gl.glBegin( GL.GL_LINES );

            gl.glVertex2d(p2.x, p2.y);
 
            gl.glVertex2d(p2.x + scale*contactForceB2.x, p2.y+scale*contactForceB2.y);
            gl.glEnd();
            
         //   drawArrowHeads(gl, p2, normal2);
            
            gl.glLineWidth(2);
            gl.glColor4f(1 , 0, 0, 1);
            gl.glBegin( GL.GL_LINES );

            gl.glVertex2d(p1.x, p1.y);

      
            gl.glVertex2d(p1.x + scale*contactForceB1.x, p1.y+scale*contactForceB1.y);
            gl.glEnd();
            
            subBody1.transformW2B.transform(contactForceB1);
            subBody2.transformW2B.transform(contactForceB2);

            
    }

    
    /**
     * Draws the connections between bodies to visualize the 
     * the adjacency structure of the matrix as a graph.
     * @param drawable
     */
    public void drawInternalContactForce( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        // draw a line between the two bodies but only if they're both not pinned
        Point2d p1 = new Point2d(block1.pB);
        Point2d p2 = new Point2d(block2.pB);
      
        subBody1.transformB2W.transform(p1); 
        subBody2.transformB2W.transform(p2);
        subBody1.transformB2W.transform(contactForceB1);
        subBody2.transformB2W.transform(contactForceB2);

            gl.glLineWidth(2);
            gl.glColor4f(0, 0, 1, 1);
            gl.glBegin( GL.GL_LINES );

            gl.glVertex2d(p2.x, p2.y);
            double scale = 0.05;
     
         
            gl.glVertex2d(p2.x + scale*contactForceB2.x, p2.y+scale*contactForceB2.y);
            gl.glEnd();
            
         //   drawArrowHeads(gl, p2, normal2);
            
            gl.glLineWidth(2);
            gl.glColor4f(0, 0, 1, 1);
            gl.glBegin( GL.GL_LINES );

            gl.glVertex2d(p1.x, p1.y);
    
          
      
            gl.glVertex2d(p1.x + scale*contactForceB1.x, p1.y+scale*contactForceB1.y);
            gl.glEnd();
            
            subBody1.transformW2B.transform(contactForceB1);
            subBody2.transformW2B.transform(contactForceB2);

          //  drawArrowHeads(gl, p2, normal2);
            
  
    }
    
    /**
     * Draws the connections between bodies to visualize the 
     * the adjacency structure of the matrix as a graph.
     * @param drawable
     */
    public void drawInternalContactHistory( GLAutoDrawable drawable ) {
    	
    	// start with body1.
    	Vector2d body1Force= getAverageContactForce(body1ContactForceHistory);
    //	double body1Torque = getAverageContactTorque(body1ContactTorqueHistory);
    	Vector2d body1ForceVar= getContactForceVar(body1ContactForceHistory, body1Force);
    	//double body1TorqueVar = getContactTorqueVar(body1ContactTorqueHistory, body1Torque);
    	
    	//get average body2.
    	Vector2d body2Force= getAverageContactForce(body2ContactForceHistory);
    //	double body2Torque = getAverageContactTorque(body2ContactTorqueHistory);
    	Vector2d body2ForceVar= getContactForceVar(body2ContactForceHistory, body2Force);
    //	double body2TorqueVar = getContactTorqueVar(body2ContactTorqueHistory,body2Torque);
    	
        GL2 gl = drawable.getGL().getGL2();
        // draw a line between the two bodies but only if they're both not pinned
        Point2d p1 = new Point2d(block1.pB);
        Point2d p2 = new Point2d(block2.pB);
      
        subBody1.transformB2W.transform(p1); 
        subBody2.transformB2W.transform(p2);
        subBody1.transformB2W.transform(body1Force);
        subBody2.transformB2W.transform(body2Force);


            
            double scale = 0.05;

            gl.glLineWidth(1);
            gl.glColor4f(0, 1, 1, 1);
            gl.glBegin( GL.GL_LINES );
            gl.glVertex2d(p2.x + scale*body2Force.x + body2ForceVar.x, p2.y+scale*body2Force.y);
            gl.glVertex2d(p2.x + scale*body2Force.x - body2ForceVar.x, p2.y+ scale*body2Force.y);
            gl.glEnd();
         
            gl.glBegin( GL.GL_LINES );
            gl.glVertex2d(p2.x + scale*body2Force.x, p2.y+scale*body2Force.y + body2ForceVar.y);
            gl.glVertex2d(p2.x + scale*body2Force.x, p2.y+scale*body2Force.y -  body2ForceVar.y);
            gl.glEnd();
            
            gl.glBegin( GL.GL_LINES );
            gl.glVertex2d(p1.x + scale*body1Force.x + body1ForceVar.x, p1.y+scale*body1Force.y);
            gl.glVertex2d(p1.x + scale*body1Force.x - body1ForceVar.x, p1.y+scale*body1Force.y);
            gl.glEnd();
         
      
            gl.glBegin( GL.GL_LINES );
            gl.glVertex2d(p1.x + scale*body1Force.x, p1.y+scale*body1Force.y + body1ForceVar.y);
            gl.glVertex2d(p1.x + scale*body1Force.x, p1.y+scale*body1Force.y -  body1ForceVar.y);
            gl.glEnd();
            
          //  subBody1.transformW2B.transform(contactForceB1);
           // subBody2.transformW2B.transform(contactForceB2);

          //  drawArrowHeads(gl, p2, normal2);
            
  
    }

	private double getContactTorqueVar(ArrayList<Double> list, double avg) {
		double sum = 0;
		for (Double dub : list) {
			sum+= (dub -avg)*(dub -avg);
		}
		sum/=(list.size() - 1);
		
		return sum;
	}

	private Vector2d getContactForceVar(ArrayList<Vector2d> list, Vector2d avg) {
	
		double sum1 = 0;
		double sum2 = 0;
		for (Vector2d vec : list) {
			sum1+= (vec.x - avg.x)*(vec.x - avg.x);
			sum2+= (vec.y - avg.y)*(vec.y - avg.y);
		}
		sum1/=(list.size() - 1);
		sum2 /= (list.size() - 1);
		
		return new Vector2d(sum1, sum2);
	}

	private double getAverageContactTorque(ArrayList<Double> list) {
		double x = 0;
		for (Double dub: list) {
			x += dub;
		}
		x/=list.size();
		return  x;
		
	}

	private Vector2d getAverageContactForce(ArrayList<Vector2d> list) {
			double x = 0;
			double y = 0;
			for (Vector2d vec: list) {
				x += vec.x;
				y += vec.y;
			}
			x/=list.size();
			y/= list.size();
			return new Vector2d(x, y);
	}
    
}
