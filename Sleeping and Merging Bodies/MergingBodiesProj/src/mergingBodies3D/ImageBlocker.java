package mergingBodies3D;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import javax.imageio.ImageIO;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;


/**
 * Creates rigid bodies from an image
 * @author kry
 */
public class ImageBlocker {

    /** image data in format ARGB */
    private int[] imageData;
    
    /** keeps track of which pixels we've visited in searching for connected components */
    private boolean[][] visited;
    
    /** image width */
    int width;
    
    /** image height */
    int height;
    
    /**
     * the bodies identified in the image
     */
    public ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();

    /**
     * Creates a set of rigid bodies form the given image
     * @param filename
     * @param epsilon  threshold for detecting white, zero means white must be white
     */
    public ImageBlocker( String filename, float epsilon ) {        
        this.epsilon = epsilon;
        try {
            BufferedImage img = ImageIO.read( new File(filename) );
            width = img.getWidth();
            height = img.getHeight();
            imageData = new int[width*height];
            img.getRGB( 0, 0, width, height, imageData, 0, width );
            visited = new boolean[width][height];
            Color3f colour = new Color3f();
            ArrayList<Point3d> cyanBlockList = new ArrayList<Point3d>();
            // sweep all pixels
            for ( int x = 0; x < width; x++ ) {
                for ( int y = 0; y < height; y++ ) {
                    if ( visited[x][y] ) continue;
                    visited[x][y] = true;
                    getColor( colour, x, y );
                    if ( isWhite( colour ) ) continue;
                   
	                // this is part of a new body!
                	ArrayList<Block> blocks = new ArrayList<Block>();
                	ArrayList<Block> boundaryBlocks = new ArrayList<Block>();
                	searchConnected( x, y, blocks, boundaryBlocks );
                	if ( blocks.size() == 1 && isCyan( colour ) ) {
                		// put this aside and make a PlaneRigidBody later...
                		cyanBlockList.add( new Point3d(x,height - y,0) );
                	} else {
                		RigidBody body = createRigidBodyFromBlocks( blocks, boundaryBlocks );   
//	                	for ( Block b : blocks ) {
//	                		if ( isRed( b.c ) ) {
//	                			Spring s = new Spring( b.pB, body );
//	                			body.springs.add(s);                				
//	                		}
//	                		if ( isMagenta( b.c ) ) {
//	                			Spring s = new Spring( b.pB, body );
//	                			body.springs.add(s);
//	                			controllableSprings.add(s);
//	                			body.magneticBody = true;
//	                		}
//	                	}                	
	                	bodies.add( body );
                	}
                }
            }
            // if we had cyan isolate blocks, then make a plane for each pair
            for ( int i = 0; i < cyanBlockList.size(); i+=2 ) {
            	Point3d p1 = cyanBlockList.get(i);
            	Point3d p2 = cyanBlockList.get(i+1);
            	
            	Vector3d diff = new Vector3d();
            	Vector3d z = new Vector3d(0,0,1);
            	diff.sub(p2,p1);
            	Vector3d n = new Vector3d();
            	n.cross(diff, z);
            	
            	if ( n.y > 0 ) {
            		n.scale(-1); // make it a bottom surface, regardless the order of the points above
            	}
            	n.normalize();
            	bodies.add( new PlaneRigidBody(p1, n) );
            }
            
        } 
        catch ( Exception e ) {
            System.err.println("Problems processing image "+ filename );
            e.printStackTrace();
        }
    }

    /** Helper class to keep track of coordinate locations */
    class Coord {
        int x, y;
        Coord( int x, int y ) {
            this.x = x;
            this.y = y;
        }
    }
    
    /**
     * Searches for all pixels connected to given block using grass fire.
     * Uses a queue rather than recursion to avoid stack overflow in large images
     */
    private void searchConnected( int x, int y, ArrayList<Block> blocks, ArrayList<Block> boundaryBlocks ) {
        List<Coord> Q = new LinkedList<Coord>();
        visited[x][y] = true;
        Q.add( new Coord(x,y) );
        Color3f colour = new Color3f();
        while ( ! Q.isEmpty() ) {            
            Coord p = Q.remove(0);
            x = p.x;
            y = p.y; // for nicer 3D things, with y up
            getColor( colour, x, y );
            if ( isWhite(colour) ) continue;
            Block b = new Block( height - y, x, 0, colour );
            blocks.add( b );
            if ( isBoundary( x, y ) ) {
                boundaryBlocks.add( b );                
            }
            // search our 8 neighbors for connected components
            for ( int i = -1; i < 2; i++ ) {
                for ( int j = -1; j < 2; j++ ) {
                    if ( x+i >= 0 && x+i < width && y+j >= 0 && y+j < height && ! visited[x+i][y+j] ) {
                        Q.add( new Coord(x+i, y+j) );  
                        visited[x+i][y+j] = true;
                    }
                }
            }            
        }        
    }
    
    private RigidBody createRigidBodyFromBlocks( ArrayList<Block> blocks, ArrayList<Block> boundaryBlocks ) {
		Point3d bbmaxB = new Point3d(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
		Point3d bbminB = new Point3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);     
        // compute the mass and center of mass position   
		double massLinear = 0;
		Point3d x0 = new Point3d();
		Matrix3d theta = new Matrix3d();
        for ( Block b : blocks ) {
            double mass = b.getColourMass();
            massLinear += mass;            
            x0.x += b.j * mass;
			bbmaxB.x = Math.max(bbmaxB.x, b.j + Block.h);
			bbminB.x = Math.min(bbminB.x, b.j - Block.h);
            x0.y += b.i * mass; 
			bbmaxB.y = Math.max(bbmaxB.y, b.i + Block.h);
			bbminB.y = Math.min(bbminB.y, b.i - Block.h);
            x0.z += b.k * mass;
			bbmaxB.z = Math.max(bbmaxB.z, b.k + Block.h);
			bbminB.z = Math.min(bbminB.z, b.k - Block.h);
        }

        x0.scale ( 1 / massLinear );
        // set block positions in world and body coordinates 
        for ( Block b : blocks ) {
            b.pB.x = b.j - x0.x;
            b.pB.y = b.i - x0.y;
            b.pB.z = b.k - x0.z;
        }
        
        theta.setIdentity();
        RigidTransform transformB2W = new RigidTransform();
        RigidTransform transformW2B = new RigidTransform();
        
        transformB2W.set( theta, x0 );
        transformW2B.set( theta, x0 );
        transformW2B.invert();		
        
        transformW2B.transform(bbmaxB);
		transformW2B.transform(bbminB);
		ArrayList<Point3d> boundingBoxB = new ArrayList<Point3d>();
		boundingBoxB.add(bbmaxB);
		boundingBoxB.add(new Point3d(bbminB.x,bbmaxB.y,bbmaxB.z));
		boundingBoxB.add(new Point3d(bbmaxB.x,bbminB.y,bbmaxB.z));
		boundingBoxB.add(new Point3d(bbmaxB.x,bbmaxB.y,bbminB.z));
		boundingBoxB.add(bbminB);
		boundingBoxB.add(new Point3d(bbmaxB.x,bbminB.y,bbminB.z));
		boundingBoxB.add(new Point3d(bbminB.x,bbmaxB.y,bbminB.z));
		boundingBoxB.add(new Point3d(bbminB.x,bbminB.y,bbmaxB.z));
                
		Matrix3d massAngular0 = new Matrix3d();
        // prevent zero angular inertia in the case of a single block
        if ( blocks.size() == 1 ) {
            Block b = blocks.get(0);
            double mass = b.getColourMass();
            massAngular0.m00 = mass * (1+1)/12;
            massAngular0.m11 = mass * (1+1)/12;
            massAngular0.m22 = mass * (1+1)/12;
        } else {
        	// always prevent a zero angular inertia in 
        	// whatever the smallest axis, if this is a
        	// single row or column of blocks...
        	massAngular0.m00 = (1+1)/12.0;
            massAngular0.m11 = (1+1)/12.0;
            massAngular0.m22 = (1+1)/12.0;
        	for ( Block b : blocks ) {
                double mass = b.getColourMass();
        		double x = b.pB.x;
        		double y = b.pB.y;
        		double z = b.pB.z;
           		massAngular0.m00 += mass * ( y*y + z*z );
        		massAngular0.m11 += mass * ( x*x + z*z );
        		massAngular0.m22 += mass * ( x*x + y*y );        	
        		massAngular0.m01 += mass * x*y;
        		massAngular0.m10 += mass * x*y;        		
        		massAngular0.m02 += mass * x*z;
        		massAngular0.m20 += mass * x*z;        		
        		massAngular0.m12 += mass * y*z;
        		massAngular0.m21 += mass * y*z;
        	}
	    }

        boolean pinned = isAllBlueBlocks( blocks );
        RigidBody body = new RigidBody( massLinear, massAngular0, pinned, boundingBoxB );
        body.x0.set( x0 );
        body.x.set( x0 );
        body.updateTransformations();
        
    	// first create leaf Discs and put them in a list
    	ArrayList<BVSphere> L = new ArrayList<BVSphere>();
    	for ( Block b : blocks ) {
    		L.add( new BVSphere( b.pB, Block.radius, body ) );
    	}
         
        body.root = new BVNode( L, body );
        body.geom = new RigidBodyGeomBlocks( blocks );
        
        return body;
    }
    
    /**
     * Checks if all blocks are shades of blue
     * @return true if all blue
     */
    private boolean isAllBlueBlocks( ArrayList<Block> blocks ) {
        for ( Block b : blocks ) {
            if ( ! (b.c.x == b.c.y && b.c.x < b.c.z) ) return false;
        }
        return true;
    }
    /**
     * Checks if this block is on the boundary.  
     * Boundary blocks have at least one white neighbor. 
     * @param x
     * @param y
     * @return true if on boundary of a rigid body
     */
    private boolean isBoundary( int x, int y ) {
        if ( x == 0 || x == width-1 || y == 0 || y == height - 1 ) return true;
        Color3f color = new Color3f();
        for ( int i = -1; i < 2; i++ ) {
            for ( int j = -1; j < 2; j++ ) {
                if ( i==j ) continue;
                getColor( color, x+i, y+j );
                if ( isWhite(color) ) return true;
            }
        }   
        return false;
    }
    
    /** 
     * Gets the colour of the specified pixel
     * @param x
     * @param y
     */
    private void getColor( Color3f colour, int x, int y ) {
        int data = imageData[y*width+x];
        colour.x = ((data >> 16) & 0x0ff) / 255.0f;
        colour.y = ((data >> 8) & 0x0ff) / 255.0f;
        colour.z = ((data >> 0) & 0x0ff) / 255.0f;        
    }

    /** epsilon for checking for white pixels */
    private float epsilon = 0;
    
    /**
     * @param colour
     * @return true if the colour provided is white
     */
    private boolean isWhite( Color3f colour ) {
        final Color3f white = new Color3f(1,1,1);
        return colour.epsilonEquals(white, epsilon );
    }

    /**
     * @param colour
     * @return true if the colour provided is white
     */
    private boolean isCyan( Color3f colour ) {
        final Color3f cyan = new Color3f(0,1,1);
        return colour.epsilonEquals(cyan, epsilon );
    }
    
//    /**
//     * @param colour
//     * @return true if the colour provided is white
//     */
//    private boolean isYellow( Color3f colour ) {
//        final Color3f white = new Color3f(1,1,0);
//        return colour.epsilonEquals(white, epsilon );
//    }
//
//	private boolean isRed(Color3f colour) {
//        final Color3f red= new Color3f(1,0,0);
//        return colour.epsilonEquals(red, epsilon );
//	}
//
//	private boolean isMagenta(Color3f colour) {
//        final Color3f red= new Color3f(1,0,1);
//        return colour.epsilonEquals(red, epsilon );
//	}
    
}
