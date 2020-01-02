package mintools.viewer;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.DisplayMode;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.Toolkit;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.io.IOException;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.util.StringTokenizer;

import javax.imageio.ImageIO;
import com.jogamp.opengl.DebugGL2;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.awt.GLCanvas;
import com.jogamp.opengl.glu.GLU;
import javax.swing.JFrame;
import javax.vecmath.Point3f;

import com.jogamp.opengl.util.FPSAnimator;
import com.jogamp.opengl.util.awt.ImageUtil;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.swing.ControlFrame;

/**
 * Simple viewer glue for very quickly drawing a scene
 * @author Paul Kry
 */
public class EasyViewerAnim implements GLEventListener {
        
    /**
     * The trackball and camera
     */
    public TrackBallCameraAnim trackBall;
    
    /**
     * The scene, needed in the loaded architecture when loading cameras
     */
    private SceneGraphNode scene;
    
    /**
     * The frame containing all controls for this viewing application
     */
    public ControlFrame controlFrame;
    
    /**
     * The dimension of the display screen
     */
    public Dimension size;
   
    public JFrame frame;
    
    public GLCanvas glCanvas;
    
    /**
     * Note: fullscreen mode has issues with respect to getting
     * the size of the drawable.  This should most certainly not be used!
     */
    private boolean fullscreen = false;

    /**
     * Enables activation of multisampling capabilities on creation of a new easy viewer
     */
    public static boolean antialiasing = false;
    
    public static int antialiasingSamples = 4;

    /**
     * Device used for full screen mode
     */
    private GraphicsDevice usedDevice;

    private FPSAnimator animator;
    
    /**
     * Creates a viewer for the given scene
     * @param name
     * @param scene
     */
    public EasyViewerAnim( String name, SceneGraphNode scene ) {
        this( name, scene, new Dimension(500,500), new Dimension(600, 750) );
    }
        
    /**
     * Creates a new easy viewer with given sizes for display and controls
     * @param name
     * @param scene
     * @param size
     * @param controlSize
     */
    public EasyViewerAnim( String name, SceneGraphNode scene, Dimension size, Dimension controlSize ) {
        this(name,scene,size,controlSize, new Point3f(10,10,10), new Point3f(-10,10,10) );
    }    
    
    /**
     * Creates a new easy viewer with given sizes for display and controls
     * @param name
     * @param scene
     * @param size
     * @param controlSize
     * @param light1Pos
     * @param light2Pos
     */
    public EasyViewerAnim( String name, SceneGraphNode scene, Dimension size, Dimension controlSize, Point3f light1Pos, Point3f light2Pos ) {
        
        this.scene = scene;
        this.size = size;

        this.light1Pos.set( light1Pos );
        this.light2Pos.set( light2Pos );
        
        trackBall = new TrackBallCameraAnim();
                                      
        controlFrame = new ControlFrame("Controls");
        // We'll disable the camera tab for now, as nobody should need it until assignment 3
        // well, then again, perhaps assignment 2??
        controlFrame.add("Camera", trackBall.getControls());
        controlFrame.add("Scene", scene.getControls());
        controlFrame.setSelectedTab("Scene");
                                
        controlFrame.setSize(controlSize.width, controlSize.height);
        controlFrame.setLocation(size.width + 20, 0);
        controlFrame.setVisible(true);    
                
        GLProfile glp = GLProfile.getDefault();        
        GLCapabilities glcap = new GLCapabilities(glp);
        
        // NOTE: you can turn on full screen (window) anti-aliasing with the following
        // lines, but it may be very slow depending on your hardware!
        if ( antialiasing ) {
	        glcap.setHardwareAccelerated(true);
	        glcap.setNumSamples(antialiasingSamples);
	        glcap.setSampleBuffers(true);
        }
        
        glCanvas = new GLCanvas(glcap);
        glCanvas.setSize( size.width, size.height );
        glCanvas.setIgnoreRepaint( true );
        glCanvas.addGLEventListener( this );

        frame = new JFrame( name );
        frame.getContentPane().setLayout( new BorderLayout() );
        frame.getContentPane().add( glCanvas, BorderLayout.CENTER );
        frame.setLocation(0,0);
                
        animator = new FPSAnimator( glCanvas, 60 );
        //animator.setRunAsFastAsPossible(false);
        
        addInteractor(trackBall);        
        
        start();
    }
    
    /**
     * Starts the viewer
     */
    public void start() {
        try {
            Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
            frame.setUndecorated( fullscreen );

            frame.addWindowListener( new WindowAdapter() {
                @Override
                public void windowClosing( WindowEvent e ) {
                    stop();
                }
            });

            if ( fullscreen ) {
                usedDevice = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice();
                usedDevice.setFullScreenWindow( frame );
                usedDevice.setDisplayMode(
                        findDisplayMode(
                                usedDevice.getDisplayModes(),
                                screenSize.width, screenSize.height, //size.width, size.height,
                                usedDevice.getDisplayMode().getBitDepth(),
                                usedDevice.getDisplayMode().getRefreshRate()
                        )
                );
            } else {
                frame.setVisible( true );
                glCanvas.setSize( size.width, size.height );
            	frame.pack();
                //frame.setSize( frame.getPreferredSize() );
                //frame.setSize( frame.getContentPane().getPreferredSize() );
                
//                frame.setLocation(
//                        ( screenSize.width - frame.getWidth() ) / 2,
//                        ( screenSize.height - frame.getHeight() ) / 2
//                );
            }

            glCanvas.requestFocus();

            if ( animator != null ) animator.start();
            
        } catch ( Exception e ) {
            e.printStackTrace();
        }
    }
    
    /**
     * Stops the viewer
     */
    public void stop() {
        try {
            if ( animator != null ) animator.stop();
            if ( fullscreen ) {
                usedDevice.setFullScreenWindow( null );
                usedDevice = null;
            }
            frame.dispose();
        } catch ( Exception e ) {
            e.printStackTrace();
        } finally {
            System.exit( 0 );
        }
    }
    
    @Override
    public void dispose(GLAutoDrawable drawable) {
    	// We don't need to do anything here as we're not creating any GL memory buffers
    }
    
    private static final int DONT_CARE = -1;

    private DisplayMode findDisplayMode( DisplayMode[] displayModes, int requestedWidth, int requestedHeight, int requestedDepth, int requestedRefreshRate ) {
        // Try to find an exact match
        DisplayMode displayMode = findDisplayModeInternal( displayModes, requestedWidth, requestedHeight, requestedDepth, requestedRefreshRate );

        // Try again, ignoring the requested bit depth
        if ( displayMode == null )
            displayMode = findDisplayModeInternal( displayModes, requestedWidth, requestedHeight, DONT_CARE, DONT_CARE );

        // Try again, and again ignoring the requested bit depth and height
        if ( displayMode == null )
            displayMode = findDisplayModeInternal( displayModes, requestedWidth, DONT_CARE, DONT_CARE, DONT_CARE );

        // If all else fails try to get any display mode
        if ( displayMode == null )
            displayMode = findDisplayModeInternal( displayModes, DONT_CARE, DONT_CARE, DONT_CARE, DONT_CARE );

        return displayMode;
    }

    private DisplayMode findDisplayModeInternal( DisplayMode[] displayModes, int requestedWidth, int requestedHeight, int requestedDepth, int requestedRefreshRate ) {
        DisplayMode displayModeToUse = null;
        for ( int i = 0; i < displayModes.length; i++ ) {
            DisplayMode displayMode = displayModes[i];
            if ( ( requestedWidth == DONT_CARE || displayMode.getWidth() == requestedWidth ) &&
                    ( requestedHeight == DONT_CARE || displayMode.getHeight() == requestedHeight ) &&
                    ( requestedHeight == DONT_CARE || displayMode.getRefreshRate() == requestedRefreshRate ) &&
                    ( requestedDepth == DONT_CARE || displayMode.getBitDepth() == requestedDepth ) )
                displayModeToUse = displayMode;
        }

        return displayModeToUse;
    }
    
    /**
     * Causes the 3D viewer to be repainted.
     * Currently, this is ignored as we're using an FPSanimator 
     */
    public void redisplay() {        
        //frame.repaint();
        //glCanvas.display();
    }
    
    /**
     * Attaches the provided interactor to this viewer's 3D canvas
     * @param interactor
     */
    public void addInteractor( Interactor interactor ) {
        interactor.attach(glCanvas);
    }

    public Point3f light1Pos = new Point3f( 0, 50, 0 );
    public Point3f light2Pos = new Point3f( 0, 2, -2.5f );
    
    @Override
    public void display(GLAutoDrawable drawable) {
    	trackBall.stepAnimation(false);
        GL2 gl = drawable.getGL().getGL2();
        gl.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT );
        
        gl.glMatrixMode( GL2.GL_MODELVIEW );
        gl.glLoadIdentity();
        
        {
            // main light is at the top front of the room.
            int lightNumber = 1;
            float[] position = { light1Pos.x, light1Pos.y, light1Pos.z, 1 };
            float[] colour = { .8f, .8f, .8f, 1 };
            float[] acolour = {0,0,0,1};//{ .05f, .05f, .05f, 1 };
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_SPECULAR, colour, 0);
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_DIFFUSE, colour, 0);
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_AMBIENT, acolour, 0);
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_POSITION, position, 0);
            gl.glEnable( GL2.GL_LIGHT0 + lightNumber );
        }
        
        {
            // put a dim light at the back of the room, in case anyone wants to 
            // look at the back side of objects
            int lightNumber = 0;
            float[] position = { light2Pos.x, light2Pos.y, light2Pos.z, 1 };
            float[] colour = { .2f, .2f, .2f, 1 };
            float[] acolour = { .0f, .0f, .0f, 1 };
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_SPECULAR, colour, 0);
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_DIFFUSE, colour, 0);
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_AMBIENT, acolour, 0);
            gl.glLightfv(GL2.GL_LIGHT0 + lightNumber, GL2.GL_POSITION, position, 0);
            gl.glEnable( GL2.GL_LIGHT0 + lightNumber );
        }
        
        trackBall.prepareForDisplay(drawable);

        gl.glEnable( GL2.GL_LIGHTING );
        gl.glEnable( GL2.GL_NORMALIZE );
        
        scene.display(drawable);
        
    }
    
    /**
     * Saves a snapshot of the current canvas to a file.
     * The image is saved in png format and will be of the same size as the canvas.
     * Note that if you are assembling frames saved in this way into a video, 
     * for instance, using virtualdub, then you'll need to take care that the 
     * canvas size is nice (i.e., a multiple of 16 in each dimension), or add 
     * a filter in virtualdub to resize the image to be a codec friendly size.
     * @param drawable
     * @param file
     * @return true on success
     */
    public boolean snapshot( GLAutoDrawable drawable, File file ) {
        GL2 gl = drawable.getGL().getGL2();
        int width = drawable.getSurfaceWidth();
        int height = drawable.getSurfaceHeight();
        //gl.glReadPixels( 0, 0, width, height, GL2.GL_ABGR_EXT, GL.GL_UNSIGNED_BYTE, imageBuffer );            
        gl.glReadPixels( 0, 0, width, height, GL2.GL_BGR, GL.GL_UNSIGNED_BYTE, imageBuffer );
        ImageUtil.flipImageVertically(image);
        
        try {
            if ( ! ImageIO.write( image, "png", file) ) {
                System.err.println("Error writing file using ImageIO (unsupported file format?)");
                return false;
            }
        } catch (IOException e) {    
            System.err.println("trouble writing " + file );
            e.printStackTrace();
            return false;
        }
        
        // print a message in the display window
        beginOverlay( drawable );
        String text =  "RECORDED: "+ file.toString();
        gl.glDisable( GL2.GL_LIGHTING );
        gl.glColor4f( 1, 0, 0, 1 );           
        printTextLines( drawable, text, 10, drawable.getSurfaceHeight()-20, 10, GLUT.BITMAP_HELVETICA_10 );
        gl.glEnable( GL2.GL_LIGHTING );
        endOverlay(drawable);
        return true;
    }    

    /** Image for sending to the image processor */
    private BufferedImage image;
    
    /** Image Buffer for reading pixels */
    private Buffer imageBuffer;
    
    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {           
        //image = new BufferedImage( width, height, BufferedImage.TYPE_4BYTE_ABGR );            
        image = new BufferedImage( width, height, BufferedImage.TYPE_3BYTE_BGR );
        imageBuffer = ByteBuffer.wrap(((DataBufferByte)image.getRaster().getDataBuffer()).getData());
    }
    
    @Override
    public void init(GLAutoDrawable drawable) {
        drawable.setGL( new DebugGL2(drawable.getGL().getGL2()) );
        GL2 gl = drawable.getGL().getGL2();
        gl.glShadeModel(GL2.GL_SMOOTH);              // Enable Smooth Shading
        gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);    // Black Background
        gl.glClearDepth(1.0f);                      // Depth Buffer Setup
        gl.glEnable(GL.GL_DEPTH_TEST);              // Enables Depth Testing
        gl.glDepthFunc(GL.GL_LEQUAL);               // The Type Of Depth Testing To Do
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
        //gl.glEnable( GL2.GL_MULTISAMPLE );
        scene.init(drawable);
    }
 
    /**
     * GLUT object to be shared (though perhaps not threadsafe)
     */
    static public GLUT glut = new GLUT();     
    
    /**
     * A <code>GLU</code> object (like a <code>GLUT</code> object) should
     * only be used by one thread at a time. So create your own if you need one.
     */
    private static GLU glu = new GLU();
    
    /**
     * Begin drawing overlay (e.g., text, screen pixel coordinate points and 
     * lines)
     * @param drawable
     */
    static public void beginOverlay( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glPushAttrib( GL.GL_DEPTH_BUFFER_BIT | GL2.GL_ENABLE_BIT | 
                GL2.GL_FOG_BIT | GL2.GL_LIGHTING_BIT | GL.GL_DEPTH_BUFFER_BIT );
        gl.glPushMatrix();
        gl.glLoadIdentity();
        gl.glMatrixMode( GL2.GL_PROJECTION );
        gl.glPushMatrix();
        gl.glLoadIdentity ();
        int width = drawable.getSurfaceWidth();
        int height = drawable.getSurfaceHeight();
        glu.gluOrtho2D( 0, width, height, 0 );
        gl.glDisable(GL.GL_DEPTH_TEST);
        gl.glDisable(GL.GL_TEXTURE_2D);
        gl.glDisable(GL2.GL_LIGHTING);
        gl.glMatrixMode( GL2.GL_MODELVIEW );
    }
    
    /**
     * Draws multi-line text.
     * @param drawable
     * @param text Text lines to draw, delimited by '\n'.
     */
    static public void printTextLines( GLAutoDrawable drawable, String text ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glColor3f(1,1,1);
        printTextLines( drawable, text, 20, 20, 26, GLUT.BITMAP_TIMES_ROMAN_24 );
    }
    
    /**
     * Draws text.
     * @param drawable
     * @param text The String to draw.
     * @param x    The starting x raster position.
     * @param y    The starting y raster position.
     * @param h    The height of each line of text.
     * @param font The font to use (e.g. GLUT.BITMAP_HELVETICA_10).
     */
    static public void printTextLines(GLAutoDrawable drawable, String text, double x, double y, double h, int font)
    {
        GL2 gl = drawable.getGL().getGL2();
        StringTokenizer st = new StringTokenizer( text, "\n" );
        int line = 0;
        while ( st.hasMoreTokens() ) {
            String tok = st.nextToken();
            gl.glRasterPos2d( x, y + line * h );            
            glut.glutBitmapString(font, tok);
            line++;
        }        
    }
    
    /**
     * End drawing overlay.
     * @param drawable
     */
    static public void endOverlay( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glMatrixMode( GL2.GL_PROJECTION );
        gl.glPopMatrix();
        gl.glMatrixMode(GL2.GL_MODELVIEW);
        gl.glPopMatrix();
        gl.glPopAttrib();        
    }    
    
}
