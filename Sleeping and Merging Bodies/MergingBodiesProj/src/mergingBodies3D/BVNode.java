package mergingBodies3D;

import java.util.ArrayList;
import java.util.List;

import com.jogamp.opengl.GLAutoDrawable;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Bounding volume node used to build bounding volume trees.
 * The tree is constructed such that every interior node has two children.
 * Either there are two children, or this is a leaf node and the leafBlock 
 * member stores the block at this leaf.
 * (Note that it is actually a bounding area, but it is probably easier to 
 * stick with the 3D terminology)
 * @author kry
 */
public class BVNode {

    /** Bounding disc for all leaves in this subtree */
    Disc boundingDisc;
    
    BVNode child1;
    
    BVNode child2;
    
    /** The visitID keeps track of when this node was last visited */
    int visitID;
    
    /** 
     * Copies the subtree and sets the given body as the associated rigid body.
     * @param n
     * @param body
     */
    public BVNode( BVNode n, RigidBody body ) {
    	boundingDisc = new Disc( n.boundingDisc, body );
    	if ( n.child1 != null ) child1 = new BVNode( child1, body );
    	if ( n.child2 != null ) child2 = new BVNode( child2, body );
    }
    
    /** 
     * Creates a BVNode leaf with the provided disc
     * @param r radius of bouding disc
     * @param b
     */
    public BVNode( Disc d ) {
    	this.boundingDisc = d;    	
    }
    
    /**
     * Create a bounding volume node from a list of blocks. An axis aligned
     * bounding box is computed for the blocks, and the blocks are split in 
     * the axis aligned direction in which they are most spread out.
     * @param blocks
     * @param body
     */
    public BVNode( List<Block> blocks, RigidBody body ) {
        
        // create our own bounding disc
        boundingDisc = new Disc(blocks, body);
        
        if ( blocks.size() > 1 ) {
            // find the distribution       
            Block b0 = blocks.get(0);
            Point3d max = new Point3d( b0.pB );
            Point3d min = new Point3d( b0.pB );            
            for ( Block b : blocks ) {
                max.x = Math.max( max.x, b.pB.x );
                max.y = Math.max( max.y, b.pB.y );
                max.z = Math.max( max.z, b.pB.z );
                min.x = Math.min( min.x, b.pB.x );
                min.y = Math.min( min.y, b.pB.y );
                min.z = Math.min( min.z, b.pB.z );
            }
            Vector3d diff = new Vector3d();
            Point3d centre = new Point3d();
            diff.sub( max, min );
            centre.interpolate( max, min, 0.5 );
            ArrayList<Block> L1 = new ArrayList<Block>();
            ArrayList<Block> L2 = new ArrayList<Block>();
            
            int axis = 0;
            if ( diff.y > diff.x && diff.y > diff.z ) {
            	axis = 1;
            } else if ( diff.z > diff.y && diff.z > diff.x ) {
            	axis = 2;
            }
            double[] bpB = new double[3];
            double[] c = new double[3];
            centre.get(c);
            
            // TODO: check the third direction
            for ( Block b : blocks ) {
            	b.pB.get( bpB );
            	if ( bpB[axis] < c[axis] ) {
            		L1.add(b);
            	} else {
            		L2.add(b);
            	}
//            	
//            	if ( diff.y > diff.x ) {
//                    if ( b.pB.y < centre.y ) {
//                        L1.add( b );
//                    } else {
//                        L2.add( b );
//                    }
//                } else {
//                    if ( b.pB.x < centre.x ) {
//                        L1.add( b );
//                    } else {
//                        L2.add( b );
//                    }
//                }
            }
            child1 = new BVNode(L1, body);
            child2 = new BVNode(L2, body);            
        }
    }

    /**
     * @return true if this node is a leaf
     */    
    public boolean isLeaf() {
    	return (child1 == null) && (child2 == null ); 
    }
    
    /**
     * Draws the bounding volume spheres
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        boundingDisc.display(drawable);
        if ( child1 != null ) child1.display(drawable);
        if ( child2 != null ) child2.display(drawable);
    }
    
    /**
     * Draws the bounding volume spheres at the boundary where the sphere was visited by its children were not.
     * @param drawable
     * @param visit
     */
    public void displayVisitBoundary( GLAutoDrawable drawable, int visit ) {
        if ( isLeaf() ) {
            boundingDisc.display(drawable);    
        } else if ( child1.visitID != visit ) { // both children are visited, or not, never one or the other
            boundingDisc.display(drawable);
        } else {
            child1.displayVisitBoundary(drawable, visit);
            child2.displayVisitBoundary(drawable, visit);
        }
    }
    
}
