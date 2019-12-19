package mergingBodies3D;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.File;
import java.io.FilenameFilter;
import java.nio.IntBuffer;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.FileSelect;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.FancyAxis;
import mintools.viewer.FlatMatrix4d;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;
import mintools.viewer.ShadowMap;

/**
 * Main entry point for the application
 * @author kry
 */
public class LCPApp implements SceneGraphNode, Interactor {

    private EasyViewer ev;

    private RigidBodySystem system = new RigidBodySystem();
    
    private Factory factory = new Factory( system );

    private CollisionComputationMonitor ccm = new CollisionComputationMonitor();
    
    /**
     * Creates a shadow map with a square image, e.g., 1024x1024.
     * This number can be reduced to improve performance.
     */
    private ShadowMap shadowMap = new ShadowMap( 2048 );
    
    /**
     * Entry point for application
     * @param args
     */
    public static void main(String[] args) {
        new LCPApp();        
    }
    
    /**
     * Creates the application / scene instance
     */
    public LCPApp() {
        system.mouseSpring = mouseSpring;
        loadSystem("datalcp/lcp.png"); // good default scene
        T.getBackingMatrix().setIdentity();
        ev = new EasyViewer( "2D Rigid Body Collision Processing", this, new Dimension(540,480), new Dimension(640,480) );
        ev.addInteractor(this);        
    }
     
    @Override
    public void init(GLAutoDrawable drawable) {
        GL gl = drawable.getGL();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
        shadowMap.init(drawable); 
        gl.glClearColor(1,1,1,1);
    }
                
    private FlatMatrix4d T = new FlatMatrix4d();
    
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        
        if ( selectRequest ) {
        	selectRequest = false;
    		select(drawable, mousePressedPoint );
        }
        
        if ( deleteDisplayListRequest ) {
            deleteDisplayListRequest = false;
            RigidBody.clearDisplayLists( gl );
            // if by chance, this request is made when there are bodies,
            // the bodies will need their listsIDs reset
            for ( RigidBody b : system.bodies ) {
                b.myListID = -1;
            }
        }
        
        if ( run.getValue() ) {
            double dt = stepsize.getValue() / (int)substeps.getValue();
            for ( int i = 0; i < substeps.getValue(); i++ ) {
                if ( factory.use ) factory.advanceTime( dt );
                system.advanceTime( dt );                
            }
        }
 
        if ( ! drawWithShadows.getValue() ) {
            drawAllObjects(drawable);
        } else {                    
            shadowMap.beginLightPass( drawable, ev.trackBall );
            drawAllObjects(drawable);        
            shadowMap.endLightPass( drawable, ev.trackBall );
            shadowMap.beginShadowMapping(drawable, ev.trackBall); 
            drawAllObjects( drawable);
            shadowMap.endShadowMapping( drawable, ev.trackBall );
        }        
       if ( picked != null ) {
    	   int h = drawable.getSurfaceHeight();
			Point3d p = mouseSpring.point;
			unproject( drawable, mousePressedPoint.x, h-mousePressedPoint.y, zClosest, p );
			gl.glPushMatrix();
			gl.glTranslated( p.x, p.y, p.z );
			gl.glColor3f(1, 0, 0);
			EasyViewer.glut.glutSolidSphere(0.025,16,16);
			gl.glPopMatrix();
			mouseSpring.display(drawable);
        }

        EasyViewer.beginOverlay(drawable);

        final SimpleDateFormat formatter = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
        gl.glColor4f(0,0,0,1);
        String text = system.name + "\n";
        text += "bodies = " + system.bodies.size() + "\n";        
        text += "contacts = " + system.collisionProcessor.contacts.size() + "\n";
        text += formatter.format( new Date() ) + "\n";
        text += "simulation time = " + system.simulationTime + "\n";
        text += "total compute time = " + system.totalAccumulatedComputeTime + "\n";
        text += "compute time = " + system.computeTime + "\n";
        text += "collision detection = " + system.collisionProcessor.collisionDetectTime + "\n";
        text += "collision processing = " + system.collisionProcessor.collisionSolveTime + "\n";
        text += "h = " + stepsize.getValue() + " (with " + substeps.getValue() + " substeps)\n";
        text += "PGS iterations = " + system.collisionProcessor.iterations.getValue() + "\n";
        text += "mu = " + system.collisionProcessor.friction.getValue() + "\n";
        text += "r = " + system.collisionProcessor.restitution.getValue() +"\n";
        
        if ( ! hideOverlay.getValue() ) {
        	EasyViewer.printTextLines( drawable, text, 10, 10, 12, GLUT.BITMAP_HELVETICA_10 );
        }
    	EasyViewer.endOverlay(drawable);

        if ( drawGraphs.getValue() ) {
        	ccm.draw(drawable);
        }
        
        if ( run.getValue() || stepped ) {
            ccm.monitor(system);
            stepped = false;        
            if ( record.getValue() ) {
                // write the frame
                File file = new File( "stills/" + dumpName + format.format(nextFrameNum) + ".png" );                                             
                nextFrameNum++;
                file = new File(file.getAbsolutePath().trim());
                ev.snapshot(drawable, file);
            }
        }
    }
    
	private final FancyAxis fa = new FancyAxis();

    private void drawAllObjects( GLAutoDrawable drawable ) {
    	GL2 gl = drawable.getGL().getGL2();
        
        gl.glPushMatrix();
        double s = 1e-1; // guess how to get images to show up nicely in 3D in a first approximation...
        gl.glScaled( s,s,s );
        system.display( drawable );

        gl.glScaled( 5,5,5 );
        fa.draw(gl);
        gl.glPopMatrix();
    }

    private void drawNonShadowable( GLAutoDrawable drawable ) {
        // perhaps want to do something here? 
    }
    
    /**
     * Selects the skeleton ODE body at the given mouse point
     * The mouse spring is updated with the selected body, and the 
     * selected body has a selected point unprojected from screen space 
     * mapped to body coordinates.
     * Selection only works for one skeleton!
     * @param drawable
     * @param p
     */
    private void select( GLAutoDrawable drawable, Point p ) {

    	GL2 gl = drawable.getGL().getGL2();

    	int pickWindowRadius = 1;
    	int h = drawable.getSurfaceHeight(); // should really be viewport
		int xl = p.x - pickWindowRadius;
		int xh = p.x + pickWindowRadius;
		int yl = (h - p.y) - pickWindowRadius;
		int yh = (h - p.y) + pickWindowRadius;
		
    	// clear selection
    	for ( RigidBody b : system.bodies ) {
    		b.selected = false;
    	}
    	
    	if ( selectionBuffer == null ) {
	        int pickbufferSize = 10000;
	        selectionBuffer = Buffers.newDirectIntBuffer(pickbufferSize);	        
	        if ( selectionBuffer == null ) {
	            System.out.println("problems allocating pick buffer :-(");
	            return;
	        }
	    }
	    
	    final int[] viewport = new int[4];
	    gl.glGetIntegerv(GL.GL_VIEWPORT, viewport, 0);
	    //System.out.println( "viewport = " + viewport[0] + " "+ viewport[1] + " "+ viewport[2] + " "+ viewport[3] );
	    final double[] currentProjectionMatrix = new double[16];
	    gl.glGetDoublev( GL2.GL_PROJECTION_MATRIX, currentProjectionMatrix, 0 );
	    
	    gl.glSelectBuffer( selectionBuffer.capacity(), selectionBuffer );
	    gl.glRenderMode( GL2.GL_SELECT );
	    gl.glInitNames(); // clear name stack
	    gl.glPushName(-1);  // I don't think we want this, do we? safety!

	    // set up the projection matrix for picking
	    gl.glMatrixMode( GL2.GL_PROJECTION );
	    gl.glLoadIdentity();
	    glu.gluPickMatrix((xl+xh)/2f, (yl+yh)/2f, xh-xl, yh-yl, viewport, 0);
	    gl.glMultMatrixd(currentProjectionMatrix, 0);
	    gl.glMatrixMode(GL2.GL_MODELVIEW); // Draw the model vertices
    
	    // DRAW IT!
        gl.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT );

        drawAllObjects(drawable);
	       
        // Restore OpenGL state
	    gl.glMatrixMode(GL2.GL_PROJECTION);
	    gl.glLoadMatrixd(currentProjectionMatrix, 0);
	    gl.glMatrixMode(GL2.GL_MODELVIEW); // back again in default mode
	
	    // Restores the rendering mode back to GL_RENDER mode
	    int selectionHits = gl.glRenderMode(GL2.GL_RENDER);

	    // process hits
	    // System.out.println("---------------------------------");
	    //System.out.println(" HITS: " + selectionHits);
	    if ( selectionHits !=0 ) {
	    	
	        // Buffer:
	        // 0=#names, 1=z1, 2=z2, 3=name[1], 4=name[2], ... name[#names]
	    	
    		//int nnames = selectionBuffer.get(0);
    		//System.out.println( "#names = " + nnames );
	        zClosest = 2; // anything past 1 is farther than far
	        int pickIndex = -1; // -1 should not be an assigned name!
	        for (int i = 0; i < selectionHits; i++) {
	        	int index = selectionBuffer.get(4 * i + 3);
		        long zL = Integer.toUnsignedLong( selectionBuffer.get(4 * i + 1) );
		        float z = ( (float) zL ) / 0xffffffffL; // convert to [0,1] interval for comparison and unproject
	        	//System.out.println(" hit: " + selectionBuffer.get(4 * i + 3) + " distance " + selectionBuffer.get(4 + i + 1) );
		        if ( z < zClosest ) { 
		        	pickIndex = index;
	                zClosest = z;
	            }
	        }
	        //System.out.println( "Picked name = " + pickedIndex );
	        if ( pickIndex == -1 ) {
	        	System.out.println("seemed to pick something else?");
	        } else {
	        	unproject( drawable, p.x, h-p.y, zClosest, mouseSpring.point );
                mouseSpring.picked = system.bodies.get(pickIndex);
                mouseSpring.picked.selected = true;
                Matrix4d A = mouseSpring.picked.transformW2B.T;
                A.transform(mouseSpring.point,  mouseSpring.picked.selectedPoint ) ;
		        System.out.println(" unprojected on body = " +  mouseSpring.picked.selectedPoint  + " index " +  pickIndex );
	        }
        }
    }

    private void unproject( GLAutoDrawable drawable, float x, float y, float z, Point3d p ) {
	    final int[] viewport = new int[4];
    	final float[] modelview = new float[16];
    	final float[] projection = new float[16];
        final float[] position = new float[4];
    	GL2 gl = drawable.getGL().getGL2();
	    gl.glGetIntegerv(GL.GL_VIEWPORT, viewport, 0);
        gl.glGetFloatv( GL2.GL_MODELVIEW_MATRIX, modelview, 0 );
        gl.glGetFloatv( GL2.GL_PROJECTION_MATRIX, projection, 0 );
        glu.gluUnProject( x, y, z, modelview, 0, projection, 0, viewport, 0, position, 0 );
        p.set( position[0], position[1], position[2] );
    }
    
    private BooleanParameter record = new BooleanParameter( "record (ENTER in canvas)", false );
    
    /** 
     * boolean to signal that the system was stepped and that a 
     * frame should be recorded if recording is enabled
     */
    private boolean stepped = false;
        
    private String dumpName = "dump";
    
    private int nextFrameNum = 0;
    
    private NumberFormat format = new DecimalFormat("00000");

    private BooleanParameter hideOverlay = new BooleanParameter( "hide overlay", false );
    private BooleanParameter drawGraphs = new BooleanParameter("draw performance graphs", false );
    private BooleanParameter run = new BooleanParameter( "simulate", false );
    private DoubleParameter stepsize = new DoubleParameter( "step size", 0.05, 1e-5, 1 );
    private IntParameter substeps = new IntParameter( "sub steps (integer)", 1, 1, 100);
    
    /**
     * Creates a control panel for changing visualization and simulation parameters
     */
    @Override
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        
        JPanel basicControls = new JPanel( new GridLayout(1,3));
        JButton reset = new JButton("Reset");
        basicControls.add( reset );
        reset.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                systemReset();
            }
        });
        
        JButton clear = new JButton("Clear");
        basicControls.add( clear );
        clear.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                factory.use = false;
                systemClear();
            }
        });
        
        JButton load = new JButton("Load");
        basicControls.add( load );
        load.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                File f = FileSelect.select("png", "image", "load", "datalcp/", true );
                if ( f != null ) {
                    loadSystem( f.getPath() );
                }
            }
        });
        vfp.add( basicControls );
        
        HorizontalFlowPanel hfp2 = new HorizontalFlowPanel();
        hfp2.add( record.getControls() );
        JButton res1 = new JButton("640x360");
        hfp2.add( res1);
        res1.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                ev.glCanvas.setSize( 640, 360 );
                ev.frame.setSize( ev.frame.getPreferredSize() );
            }
        });        
        JButton res2 = new JButton("1280x720");
        hfp2.add( res2);
        res2.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {                
                ev.glCanvas.setSize( 1280, 720 );
                ev.frame.setSize( ev.frame.getPreferredSize() );

            }
        });                
        vfp.add( hfp2.getPanel() );
        
        vfp.add( hideOverlay.getControls() );
        vfp.add( drawGraphs.getControls() );
        
        vfp.add( run.getControls() );
        vfp.add( stepsize.getSliderControls(true) );
        vfp.add( substeps.getControls() );
        vfp.add( system.getControls() );
        
        vfp.add( mouseSpring.getControls() );
        
        VerticalFlowPanel vfpv = new VerticalFlowPanel();
        vfpv.setBorder( new TitledBorder("window content scaling controls") );
        vfpv.add( scale.getSliderControls(true) );
        vfpv.add( posx.getSliderControls(false) );
        vfpv.add( posy.getSliderControls(false) );
        CollapsiblePanel vcp = new CollapsiblePanel(vfpv.getPanel());
        vcp.collapse();
        vfp.add( vcp );
        
        vfp.add( whiteEpsilon.getSliderControls(false) );
        vfp.add( factory.getControls() );
        
        vfp.add(drawWithShadows.getControls() );
        return vfp.getPanel();
    }
    
    private BooleanParameter drawWithShadows = new BooleanParameter( "draw with shadows", true );

    DoubleParameter whiteEpsilon = new DoubleParameter( "white epsilon", 0.05, 0, 1 );
        
    // parameters and variables for for scaling and translating the window
    private DoubleParameter scale = new DoubleParameter("scale scene",.9, 0.1, 10);
    private DoubleParameter posx = new DoubleParameter("x translation", 0, -1000, 1000 );
    private DoubleParameter posy = new DoubleParameter("y translation", 0, -1000, 1000 );
//    private double windowWidth = 1.0;
//    private double windowHeight= 1.0;
//    private double imageWidth = 1.0;
//    private double imageHeight = 1.0;

    // variables and objects for picking rigid body with the mouse
    
    
    private RigidBody picked = null;
//    private Point2d grabPointB = new Point2d();
   
    // TODO: UPDATE WITH the PD Control
    private Point3d mousePoint3d = new Point3d();      
    private MouseSpringForce mouseSpring = new MouseSpringForce( mousePoint3d );
    
    /**
     * Loads the specified image, clearing the old system, and resets viewing parameters.
     * @param filename
     */
    private void loadSystem( String filename ) {
        factory.use = false;        
        systemClear();
        system.name = filename;
        ImageBlocker blocker = new ImageBlocker( filename, (float) (double) whiteEpsilon.getValue() );
        system.bodies.addAll(blocker.bodies);
    }
    
    /**
     * Loads the specified image as a factory, clearing the old system, and resets viewing parameters.
     * @param filename
     */
    private void loadFactorySystem( String filename ) {              
        factory.use = false;        
        systemClear();
        system.name = filename + " factory";
        ImageBlocker blocker = new ImageBlocker( filename, (float) (double) whiteEpsilon.getValue() );
        factory.setImageBlocker(blocker);
        factory.use = true;
        factory.reset();        
    }
    
    /**
     * Resets the rigid body system, and factory if it is currently being used.  
     * When the factory is reset, all non pinned rigid bodies are removed from the system.
     */
    private void systemReset() {
        if ( factory.use ) {
            factory.reset();
        } else {
            system.reset();
        }
    }
    
    private boolean deleteDisplayListRequest = false;
    
    /**
     * Clears the system, and ensures that any display lists that were created
     * to draw the various rigid bodies are cleaned up on the next display call.
     */
    private void systemClear() {
        posx.setValue(0.0);
        posy.setValue(0.0);
        scale.setValue(0.9);
        deleteDisplayListRequest = true;
        system.clear();
    }
    
    /** List of png files in the data folder which can be loaded with left and right arrow */
    private File[] files = null;
    
    /** current index in the files list */
    private int whichFile = 0;
    
    private Point mousePressedPoint = null;
    private boolean selectRequest = false;
    
    /** [0,1] z position from opengl picking to use with unproject */
    float zClosest;
   
    /** Selection buffer for openGL picking */
    private IntBuffer selectionBuffer;
   
    private GLU glu = new GLU();
    
    /**
     * Attaches mouse and keyboard listeners to the canvas.
     */
    @Override
    public void attach(Component component) {
    
        File directory = new File("datalcp");
        files = directory.listFiles(new FilenameFilter() {            
            @Override
            public boolean accept( File dir, String name ) {
                return name.endsWith(".png");                
            }
        });
        java.util.Arrays.sort(files);
        
        component.addMouseMotionListener( new MouseMotionListener() {
			@Override
			public void mouseMoved(MouseEvent e) {}
			@Override
			public void mouseDragged(MouseEvent e) {
				mousePressedPoint = e.getPoint();
			}
		});
    	component.addMouseListener( new MouseListener() {
			@Override
			public void mouseReleased(MouseEvent e) {
				if ( mouseSpring.picked != null ) {
					mouseSpring.picked.selected = false;
					mouseSpring.picked = null;
				}
			}
			@Override
			public void mousePressed(MouseEvent e) {
				if ( e.getButton() == 1 && e.isShiftDown() ) {
					mousePressedPoint = e.getPoint();
					selectRequest = true;
				}
			}
			@Override
			public void mouseExited(MouseEvent e) {}
			@Override
			public void mouseEntered(MouseEvent e) {}
			@Override
			public void mouseClicked(MouseEvent e) {}
		});
        
        component.addKeyListener( new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                if ( e.getKeyCode() == KeyEvent.VK_SPACE ) {
                    run.setValue( ! run.getValue() ); 
                } else if ( e.getKeyCode() == KeyEvent.VK_S ) {
                    double dt = stepsize.getValue() / (int)substeps.getValue();
                    for ( int i = 0; i < substeps.getValue(); i++ ) {
                        if ( factory.use ) factory.advanceTime( dt );
                        system.advanceTime( dt );                
                    }
                    stepped = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_R ) {                    
                    systemReset();      
                } else if ( e.getKeyCode() == KeyEvent.VK_C ) {                   
                    systemClear();
                    factory.use = false;
                } else if ( e.getKeyCode() == KeyEvent.VK_J ) {                   
                    system.jiggle();                                        
                } else if ( e.getKeyCode() == KeyEvent.VK_G ) {                   
                    File f = FileSelect.select("png", "image for factory", "load", "datalcp/", true );
                    if ( f != null ) {
                        loadFactorySystem( f.getPath() );
                    }   
                } else if ( e.getKeyCode() == KeyEvent.VK_F ) {                                       
                    loadFactorySystem( "datalcp/tetrisTube.png" );
                    factory.spread.setValue(30);
                    factory.interval.setValue(0.4);
                    factory.downVelocity.setValue(10.0);
                    factory.angularVelocityScale.setValue(0.5);
                    factory.linearVelocityScale.setValue(2.5);
                } else if ( e.getKeyCode() == KeyEvent.VK_PERIOD ) {                                       
                    factory.run.setValue ( ! factory.run.getValue() );
                } else if ( e.getKeyCode() == KeyEvent.VK_COMMA ) {
                    factory.createBodyRequest = true;
                } else if (  e.getKeyCode() == KeyEvent.VK_L ) {                    
                    File f = FileSelect.select("png", "image", "load", "datalcp/", true );
                    if ( f != null ) {
                        loadSystem( f.getPath() );
                    }
                } else if ( e.getKeyCode() == KeyEvent.VK_LEFT) {
                    if ( files != null && files.length >= 0 ) {
                        whichFile --;
                        if ( whichFile < 0 ) whichFile = files.length-1;
                        loadSystem( files[whichFile].getPath() );                        
                    }
                } else if ( e.getKeyCode() == KeyEvent.VK_RIGHT ) {
                    if ( files != null && files.length >= 0 ) {
                        whichFile ++;
                        if ( whichFile >= files.length ) whichFile = 0;
                        loadSystem( files[whichFile].getPath() );
                    }
                } else if ( e.getKeyCode() == KeyEvent.VK_ESCAPE ) {
                    ev.stop();
                } else if ( e.getKeyCode() == KeyEvent.VK_ENTER ) {
                    record.setValue( ! record.getValue() );
                } else if ( e.getKeyCode() == KeyEvent.VK_UP ) {
                    int ss = substeps.getValue();
                    if ( ss == substeps.getMaximum() ) return;
                    substeps.setValue( ss + 1 );
                    stepsize.setValue( stepsize.getValue() * (ss+1)/ss );
                } else if ( e.getKeyCode() == KeyEvent.VK_DOWN ) {
                    int ss = substeps.getValue();
                    if ( ss == substeps.getMinimum() ) return;
                    substeps.setValue( ss - 1 );
                    stepsize.setValue( stepsize.getValue() *(ss-1)/ss );
                }
            }
        } );
    }
    
}
