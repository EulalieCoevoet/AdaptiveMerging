package mergingBodies3D;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.Collection;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.vecmath.Point3d;

/**
 * Circular disc for collision processing
 * @author kry
 */
public class Disc {

    /** center of disc in body coordinates */
    Point3d cB = new Point3d();
    
    /** center of disc in world coordinates */
    Point3d cW = new Point3d();
    
    /** radius */
    double r;
    
    /** body to which this disc is associated */
    RigidBody body;
    
    /**
     * Builds a Disc that encloses the given blocks
     * @param blocks
     * @param body
     */
    public Disc( Collection<Block> blocks, RigidBody body ) {
        this.body = body;
        // We'll choose the minimum disc enclosing the centers and then add the block radius
        ArrayList<Point3d> points = new ArrayList<Point3d>();
        for ( Block b : blocks ) {
            points.add( new Point3d( b.pB.x, b.pB.y, b.pB.z ) );            
        }
        MinimumEnclosingCircle mec = new MinimumEnclosingCircle( points );
        cB.set( mec.answer.centre );
        body.transformB2W.transform(cB, cW);
        r = mec.answer.radius + Block.radius;
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
        for ( int i = 0 ; i < size; i++ ) {
            double theta = 2.0 * i / size * Math.PI;
            vertexBuffer.put( (float) Math.cos( theta ) );
            vertexBuffer.put( (float) Math.sin( theta ) );
            indexBuffer.put( (short) i );
        }
        vertexBuffer.position(0);               
        indexBuffer.position(0);
    }
    
    /**
     * Draws the disc, using its associated body's current position
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        body.transformB2W.transform(cB, cW);
        GL2 gl = drawable.getGL().getGL2();
        gl.glPushMatrix();
        gl.glTranslated( cW.x, cW.y ,cW.z );
        gl.glScaled( r, r, r );
        gl.glColor4f(0.5f,0.0f,0.0f,0.5f);
        gl.glEnableClientState( GL2.GL_VERTEX_ARRAY );     
        gl.glVertexPointer( 2, GL.GL_FLOAT, 0, vertexBuffer );      
        gl.glDrawElements( GL.GL_LINE_LOOP, size, GL.GL_UNSIGNED_SHORT, indexBuffer );            
        gl.glDisableClientState( GL2.GL_VERTEX_ARRAY );        
        gl.glPopMatrix();
    }
    
    /**
     * @param pW query point in world coordinates
     * @return true if point is in the disc
     */
    public boolean isInDisc( Point3d pW ) {
        body.transformB2W.transform(cB, cW);
        return ( pW.distanceSquared( cW ) < r*r );
    }
    
    /**
     * Updates the center in world coordinates.
     * This must be called before doing a call to intersects() if the body has moved
     */
    public void updatecW() {
        body.transformB2W.transform( cB, cW );
    }
    
    /**
     * Checks for intersection of this disc with another using their currently 
     * computed center location in world coordinates (cW) 
     * @param d
     * @return true if intersection
     */
    public boolean intersects( Disc d ) {
        return cW.distanceSquared(d.cW) < (r + d.r)*(r + d.r);
    }
    
}
