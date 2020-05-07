package mergingBodies3D;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

/**
 * A rigid body like all the rest, but always pinned, and its geometry defined by a plane equation. 
 * @author kry
  */
public class PlaneRigidBody extends RigidBody {

	/**
	 * Normal for the plane
	 */
	Vector3d n;
	
	/**
	 * Point for the plane
	 */
	Point3d p; 
	
	/**
	 * Size for the plane
	 */
	float radius = 200; 
	
	/**
	 * d parameter of plane equation (computed in constructor, i.e., n_x x + n_y y + n_z z + d = 0)
	 */
	double d;
	
	/** Used for warm starting contacts with this plane */
	BVSphere dummyBV = new BVSphere( new Point3d(), 1, null );
	
	public PlaneRigidBody() {
		this(new Point3d(0,40,0), new Vector3d(0, -1, 0));
	}
	
	public PlaneRigidBody( PlaneRigidBody body ) {
		super( body );
		this.n = body.n;
		this.p = body.p;
		d = - (p.x*n.x + p.y*n.y + p.z*n.z);
	}
	
	public PlaneRigidBody( Point3d p, Vector3d n ) {
		super( 0, null, true, null );
		n.normalize();
		this.n = new Vector3d(n);
		this.p = new Point3d(p);
		pinned = true;
		d = - (p.x*n.x + p.y*n.y + p.z*n.z);
		geom = new PlaneRigidBodyGeom();
	}
	
    /** number of points in the circle for drawing bounding circles */
    static private final int size = 30;
    static private FloatBuffer vertexBuffer;    
    static private ShortBuffer indexBuffer;
    
    /**
     * We'll use a vertex buffer and index buffer for drawing a unit circle.
     * Using a draw elements call will avoid the overhead of a using OpenGL
     * immediate mode, and likewise we'll not need to call Math.cos and Math.sin
     * when drawing.
     */
    static {
        int numVertFloats = size * 2;
        ByteBuffer vbb = ByteBuffer.allocateDirect( numVertFloats * 4 );
        vbb.order( ByteOrder.nativeOrder() );
        vertexBuffer = vbb.asFloatBuffer();
        ByteBuffer ibb = ByteBuffer.allocateDirect( size * 2 ); // size of short is 2     
        ibb.order(ByteOrder.nativeOrder());
        indexBuffer = ibb.asShortBuffer();
        vertexBuffer.put( 0 );
        vertexBuffer.put( 0 );
        indexBuffer.put( (short) 0 );
        for ( int i = 0 ; i < size-1; i++ ) {
            double theta = 2.0 * i / (size-2) * Math.PI;
            vertexBuffer.put( (float) Math.cos( theta ) );
            vertexBuffer.put( (float) Math.sin( theta ) );
            indexBuffer.put( (short) (i+1) );
        }
        vertexBuffer.position(0);               
        indexBuffer.position(0);
    }
	
	private class PlaneRigidBodyGeom extends RigidBodyGeom {
			    
		@Override
		public void drawGeom(GLAutoDrawable drawable) {
			GL2 gl = drawable.getGL().getGL2();
			
			float s = radius;		
			
			// this is really dumb and slow... but at least it will get baked into a display list
			// If it is ever the case we get rid of display lists, this code will certainly go too!
			Vector3d tmp = new Vector3d(0,0,1);
			Vector3d axis = new Vector3d();
			axis.cross( tmp, n );
			double angle = tmp.angle(n);
			
			gl.glPushMatrix();
			gl.glTranslated(p.x, p.y, p.z);
			gl.glRotated(angle*180/Math.PI, axis.x, axis.y, axis.z);
			gl.glScaled(s, s, 1);
			
			gl.glNormal3f(0,0,1);
			
			gl.glEnableClientState( GL2.GL_VERTEX_ARRAY );     
	        gl.glVertexPointer( 2, GL.GL_FLOAT, 0, vertexBuffer );      
	        gl.glDrawElements( GL.GL_TRIANGLE_FAN, size, GL.GL_UNSIGNED_SHORT, indexBuffer );         

			gl.glPopMatrix();			
		}
	}
	
}
