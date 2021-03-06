package mergingBodies3D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;

import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.viewer.EasyViewer;


/**
 * Bounding volume node used to build bounding volume trees.
 * The tree is constructed such that every interior node has two children.
 * Either there are two children, or this is a leaf node.
 * 
 */
public class BVNode {

    /** Bounding disc for all leaves in this subtree */
    BVSphere boundingSphere;
    
    /**
     * In many cases we will have binary trees, but in some cases we will not
     */
    BVNode[] children;
    
    /** The visitID keeps track of when this node was last visited */
    int visitID;
    
    /** children depth */
    int depth = 0;
    
    public BVNode () {
    	
    }
    
    /** 
     * Copies the subtree and sets the given body as the associated rigid body.
     * @param n
     * @param body
     */
    public BVNode( BVNode n ) {
    	boundingSphere = new BVSphere( n.boundingSphere, n.boundingSphere.body );
    	if ( n.children != null ) {
    		children = new BVNode[n.children.length];
    		for ( int i = 0; i < n.children.length; i++ ) {
    			children[i] = new BVNode( n.children[i] );
    		}
    	}
    }
    
    /** 
     * Copies the subtree and sets the given body as the associated rigid body.
     * @param n
     * @param body
     */
    public BVNode( BVNode n, RigidBody body ) {
    	boundingSphere = new BVSphere( n.boundingSphere, body );
    	if ( n.children != null ) {
    		children = new BVNode[n.children.length];
    		for ( int i = 0; i < n.children.length; i++ ) {
    			children[i] = new BVNode( n.children[i], body );
    		}
    	}
    }
    
    /** 
     * Creates a BVNode leaf with the provided disc
     * @param r radius of bounding disc
     * @param b
     */
    public BVNode( BVSphere d ) {
    	this.boundingSphere = d;    	
    }
    
//    /**
//     * Create a bounding volume node from a list of blocks. An axis aligned
//     * bounding box is computed for the blocks, and the blocks are split in 
//     * the axis aligned direction in which they are most spread out.
//     * This method builds a binary (mostly-balanced) tree.
//     * @param blocks
//     * @param body
//     */
//    public BVNode( List<BVSphere> discs, RigidBody body ) {
//        if ( discs.size() == 0 ) {
//        	throw new RuntimeErrorException(new Error("can't build a BVTree from nothing"));
//        } else if ( discs.size() == 1 ) {
//        	boundingSphere = discs.get(0);
//        } else if ( discs.size() == 2 ) {
//        	boundingSphere = new BVSphere(discs);
//        	children = new BVNode[2];
//        	children[0] = new BVNode( discs.get(0) );
//        	children[1] = new BVNode( discs.get(1) );          			
//        } else { // if ( blocks.size() > 2 ) {        
//            // find the distribution     
//        	boundingSphere = new BVSphere(discs);
//            BVSphere b0 = discs.get(0);
//            Point3d max = new Point3d( b0.cB );
//            Point3d min = new Point3d( b0.cB );            
//            for ( BVSphere b : discs ) {
//                max.x = Math.max( max.x, b.cB.x );
//                max.y = Math.max( max.y, b.cB.y );
//                max.z = Math.max( max.z, b.cB.z );
//                min.x = Math.min( min.x, b.cB.x );
//                min.y = Math.min( min.y, b.cB.y );
//                min.z = Math.min( min.z, b.cB.z );
//            }
//            Vector3d diff = new Vector3d();
//            Point3d centre = new Point3d();
//            diff.sub( max, min );
//            centre.interpolate( max, min, 0.5 );
//            ArrayList<BVSphere> L1 = new ArrayList<BVSphere>();
//            ArrayList<BVSphere> L2 = new ArrayList<BVSphere>();
//            
//            int axis = 0;
//            if ( diff.y >= diff.x && diff.y >= diff.z ) {
//            	axis = 1;
//            } else if ( diff.z >= diff.y && diff.z >= diff.x ) {
//            	axis = 2;
//            }
//            double[] bpB = new double[3];
//            double[] c = new double[3];
//            centre.get(c);
//            
//            for ( BVSphere b : discs ) {
//            	b.cB.get( bpB );
//            	if ( bpB[axis] < c[axis] ) {
//            		L1.add(b);
//            	} else {
//            		L2.add(b);
//            	}
//            }
//            if ( L1.size() * L2.size() == 0 ) {
//            	System.err.println(" is this happening?  If so this is bad!  Both lists should be non-empty!");
//            }
//            children = new BVNode[2];
//            children[0] = new BVNode(L1, body);
//            children[1] = new BVNode(L2, body);            
//        }
//    }

    /**
     * @return true if this node is a leaf
     */    
    public boolean isLeaf() {
    	return ( children == null );
    }
    

    private static final float[] colText = new float[] { 0,0,0, 0.9f };
    
    /**
     * Draws the bounding volume spheres
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
    	if ( boundingSphere == null ) {
    		System.out.println("[BVNode] display: not supposed to happen");
    		return;
    	}
    	
        boundingSphere.display(drawable);
        
        boolean debugDisplay = true;
        if(debugDisplay) {
	    	boundingSphere.updatecW();
	    	GL2 gl = drawable.getGL().getGL2();
	    	gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, colText, 0 );
		    gl.glRasterPos3d( boundingSphere.cW.x+boundingSphere.r, boundingSphere.cW.y, boundingSphere.cW.z );
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "  " + depth );
        }
        
        if ( children == null ) return;
        for ( BVNode c : children ) {
        	c.display(drawable);
        }
    }
    
    /**
     * Draws the bounding volume spheres at the boundary where the sphere was visited by its children were not.
     * @param drawable
     * @param visit
     */
    public void displayVisitBoundary( GLAutoDrawable drawable, int visit ) {
        if ( isLeaf() ) {
            boundingSphere.display(drawable);    
        } else if ( children[0].visitID != visit ) { // ALL children are visited or not, not just one or some...
            boundingSphere.display(drawable);        	
        } else {
            boundingSphere.display(drawable);   
        	for ( BVNode c : children ) {
        		c.displayVisitBoundary(drawable, visit);
        	}
        }        
    }
    
}
