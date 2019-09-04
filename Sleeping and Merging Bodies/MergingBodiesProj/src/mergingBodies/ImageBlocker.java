package mergingBodies;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import javax.imageio.ImageIO;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;

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
            // sweep all pixels
            for ( int x = 0; x < width; x++ ) {
                for ( int y = 0; y < height; y++ ) {
                    if ( visited[x][y] ) continue;
                    visited[x][y] = true;
                    getColour( colour, x, y );
                    if ( isWhite( colour ) ) continue;
                   
	                // this is part of a new body!
                	ArrayList<Block> blocks = new ArrayList<Block>();
                	ArrayList<Block> boundaryBlocks = new ArrayList<Block>();
                	ArrayList<Point2d> springEndpoints = new ArrayList<Point2d>();
                	searchConnected( x, y, blocks, boundaryBlocks, springEndpoints );
                	RigidBody body = new RigidBody( blocks, boundaryBlocks );
                	
                	for (Point2d p: springEndpoints) {
                		Spring s = new Spring(p, body);
                   		body.springs.add(s);
                	}
                	
                	bodies.add( body );
                }
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
     * @param springEndpoints 
     */
    private void searchConnected( int x, int y, ArrayList<Block> blocks, ArrayList<Block> boundaryBlocks, ArrayList<Point2d> springEndpoints ) {
        List<Coord> Q = new LinkedList<Coord>();
        visited[x][y] = true;
        Q.add( new Coord(x,y) );
        Color3f colour = new Color3f();
        while ( ! Q.isEmpty() ) {            
            Coord p = Q.remove(0);
            x = p.x;
            y = p.y;
            getColour( colour, x, y );
            if ( isWhite(colour)) continue;
            if ( isRed(colour)) springEndpoints.add(new Point2d(x, y));
       
            Block b = new Block( y, x, colour );
            blocks.add( b );
            if ( isBoundary( x, y ) ) {
                boundaryBlocks.add( b );                
            }
            // search our 8 neighbours for connected components
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


	private boolean isRed(Color3f colour) {
        final Color3f red= new Color3f(1,0,0);
        return colour.epsilonEquals(red, epsilon );
	}

	/**
     * Checks if this block is on the boundary.  
     * Boundary blocks have at least one white neighbour. 
     * @param x
     * @param y
     * @return true if on boundary of a rigid body
     */
    private boolean isBoundary( int x, int y ) {
        if ( x == 0 || x == width-1 || y == 0 || y == height - 1 ) return true;
        Color3f colour = new Color3f();
        for ( int i = -1; i < 2; i++ ) {
            for ( int j = -1; j < 2; j++ ) {
                if ( i==j ) continue;
                getColour( colour, x+i, y+j );
                if ( isWhite(colour) ) return true;
            }
        }   
        return false;
    }
    
    /** 
     * Gets the colour of the specified pixel
     * @param x
     * @param y
     */
    private void getColour( Color3f colour, int x, int y ) {
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
    
}
