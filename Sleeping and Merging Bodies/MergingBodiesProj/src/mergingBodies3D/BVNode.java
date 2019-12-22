package mergingBodies3D;

import java.util.ArrayList;
import java.util.List;

import com.jogamp.opengl.GLAutoDrawable;

import javax.management.RuntimeErrorException;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Bounding volume node used to build bounding volume trees.
 * The tree is constructed such that every interior node has two children.
 * Either there are two children, or this is a leaf node.
 * @author kry
 */
public class BVNode {

    /** Bounding disc for all leaves in this subtree */
    BVSphere boundingSphere;
    
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
    	boundingSphere = new BVSphere( n.boundingSphere, body );
    	if ( n.child1 != null ) child1 = new BVNode( child1, body );
    	if ( n.child2 != null ) child2 = new BVNode( child2, body );
    }
    
    /** 
     * Creates a BVNode leaf with the provided disc
     * @param r radius of bouding disc
     * @param b
     */
    public BVNode( BVSphere d ) {
    	this.boundingSphere = d;    	
    }
    
    /**
     * Create a bounding volume node from a list of blocks. An axis aligned
     * bounding box is computed for the blocks, and the blocks are split in 
     * the axis aligned direction in which they are most spread out.
     * @param blocks
     * @param body
     */
    public BVNode( List<BVSphere> discs, RigidBody body ) {
        if ( discs.size() == 0 ) {
        	throw new RuntimeErrorException(new Error("can't build a BVTree from nothing"));
        } else if ( discs.size() == 1 ) {
        	boundingSphere = discs.get(0);
        } else if ( discs.size() == 2 ) {
        	boundingSphere = new BVSphere(discs);
        	child1 = new BVNode( discs.get(0) );
        	child2 = new BVNode( discs.get(1) );        	
        } else { // if ( blocks.size() > 2 ) {        
            // find the distribution     
        	boundingSphere = new BVSphere(discs);
            BVSphere b0 = discs.get(0);
            Point3d max = new Point3d( b0.cB );
            Point3d min = new Point3d( b0.cB );            
            for ( BVSphere b : discs ) {
                max.x = Math.max( max.x, b.cB.x );
                max.y = Math.max( max.y, b.cB.y );
                max.z = Math.max( max.z, b.cB.z );
                min.x = Math.min( min.x, b.cB.x );
                min.y = Math.min( min.y, b.cB.y );
                min.z = Math.min( min.z, b.cB.z );
            }
            Vector3d diff = new Vector3d();
            Point3d centre = new Point3d();
            diff.sub( max, min );
            centre.interpolate( max, min, 0.5 );
            ArrayList<BVSphere> L1 = new ArrayList<BVSphere>();
            ArrayList<BVSphere> L2 = new ArrayList<BVSphere>();
            
            int axis = 0;
            if ( diff.y >= diff.x && diff.y >= diff.z ) {
            	axis = 1;
            } else if ( diff.z >= diff.y && diff.z >= diff.x ) {
            	axis = 2;
            }
            double[] bpB = new double[3];
            double[] c = new double[3];
            centre.get(c);
            
            for ( BVSphere b : discs ) {
            	b.cB.get( bpB );
            	if ( bpB[axis] < c[axis] ) {
            		L1.add(b);
            	} else {
            		L2.add(b);
            	}
            }
            if ( L1.size() * L2.size() == 0 ) {
            	System.out.println(" is this happening?  If so this is bad!");
            }
            if ( L1.size() > 0 ) child1 = new BVNode(L1, body);
            if ( L2.size() > 0 ) child2 = new BVNode(L2, body);            
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
        boundingSphere.display(drawable);
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
            boundingSphere.display(drawable);    
        } else if ( child1.visitID != visit ) { // both children are visited, or not, never one or the other
            boundingSphere.display(drawable);
        } else {
            child1.displayVisitBoundary(drawable, visit);
            child2.displayVisitBoundary(drawable, visit);
        }
    }
    
}
