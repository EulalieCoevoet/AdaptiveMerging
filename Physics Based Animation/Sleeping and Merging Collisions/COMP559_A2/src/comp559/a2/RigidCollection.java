package comp559.a2;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class RigidCollection extends RigidBody{

	ArrayList<RigidBody> collectionBodies = new ArrayList<RigidBody>();
	

	
	public RigidCollection(RigidBody body) {
		super(body);
	}
	
	public void addBody(RigidBody body) {
		collectionBodies.add(body);
		//TODO: how to determine everything else: speed velocity etc...COM, mass, inertia
	}
	
	public void calculateMass() {
		double mass = 0;
		for(RigidBody b: collectionBodies) {
			mass += b.massLinear;
		}
		this.massLinear = mass;
		this.minv = 1/mass;
	}
	
	public void calculateCOM() {
		final Point2d com = new Point2d();
		double totalMass = this.massLinear;
		for (RigidBody b: collectionBodies) {
			Point2d bCOM = new Point2d(b.x);
			double bRatio = b.massLinear/totalMass;
			bCOM.scale(bRatio);
			com.add(bCOM);
		}
		this.x0 = new Point2d(com);
		this.x = new Point2d(com);
	}
	/**
	 * Calculates the angular inertia. 
	 */
	public void calculateInertia() {
		double inertia = 0;
		for (RigidBody b: collectionBodies) {
			Point2d r = b.x;
			double distance = r.distance(this.x);
			double mass = b.massLinear*distance;
			inertia+=mass;
			
		}
		this.massAngular = inertia;
		this.jinv = 1/inertia;
		
	}

    /**
     * Draws the center of mass position with a circle.  
     * @param drawable
     */
    public void displayCOM( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
          
	    gl.glPointSize(8);
	    gl.glColor3f(0,0,0.7f);
	    gl.glBegin( GL.GL_POINTS );
	    gl.glVertex2d(x.x, x.y);
	    gl.glEnd();
	    gl.glPointSize(4);
	    gl.glColor3f(1,1,1);
	    gl.glBegin( GL.GL_POINTS );
	    gl.glVertex2d(x.x, x.y);
	    gl.glEnd();

    }
    
    /**
     * displays the Body Collection as lines between the center of masses of each rigid body to the other. 
     * Uses a string arrayList to check if a connection has already been drawn.
     * @param drawable
     */
    
    public void displayCollection( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        ArrayList<String> connections = new ArrayList<String>();
        // draw a line between the two bodies but only if they're both not pinned
        for (RigidBody b1: collectionBodies) {
        	for (RigidBody b2: collectionBodies) {
        		String option1 = Integer.toString(b1.index) + "_" + Integer.toString(b2.index);
        		String option2 = Integer.toString(b2.index) + "_" + Integer.toString(b1.index);
        		if (!b1.equals(b2) && (!(connections.contains(option1)||connections.contains(option2)))) {
        			gl.glLineWidth(1);
        			gl.glColor4f(1, 0,0, 0.5f);
        			gl.glBegin( GL.GL_LINES );
        			gl.glVertex2d(b1.x.x, b1.x.y);
        			gl.glVertex2d(b2.x.x, b2.x.y);
        			gl.glEnd();
        			connections.add(Integer.toString(b1.index) + "_" + Integer.toString(b2.index));
        		}
        	}
        }

        
    }
}
