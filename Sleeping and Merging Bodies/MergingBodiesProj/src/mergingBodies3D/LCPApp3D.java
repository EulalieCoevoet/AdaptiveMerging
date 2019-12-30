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
import java.nio.ByteBuffer;
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
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.parameters.Vec3Parameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.FileSelect;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.FlatMatrix4d;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;
import mintools.viewer.ShadowMap;

/**
 * Main entry point for the application
 * @author kry
 */
public class LCPApp3D implements SceneGraphNode, Interactor {

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
        new LCPApp3D();    
    }
    
    /**
     * Creates the application / scene instance
     */
    public LCPApp3D() {
        system.mouseSpring = mouseSpring;
        loadXMLSystem("scenes3D/boxesOnPlane.xml");
        //System("datalcp/lcp.png"); // good default scene
        T.getBackingMatrix().setIdentity();
        ev = new EasyViewer( "Adaptive Merging 3D Rigid Body Simulation", this, new Dimension(540,480), new Dimension(640,480) );
        ev.addInteractor(this);       
        
        ev.trackBall.setFocalDistance(10);
        //ev.trackBall.focalPointY.setValue(1);
        ev.trackBall.near.setValue(2);
        //ev.trackBall.far.setValue(15);
    }
     
    @Override
    public void init(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
        shadowMap.init(drawable); 
        gl.glClearColor(1,1,1,1);
        gl.glEnable( GL.GL_CULL_FACE );
    }
                
    private FlatMatrix4d T = new FlatMatrix4d();
    
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        	
        if ( selectRequest ) {
        	selectRequest = false;
    		select(drawable, mousePressedPoint );
        	gl.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT );
        }
        
        if ( deleteDisplayListRequest ) {
            deleteDisplayListRequest = false;
            RigidBodyGeom.clearDisplayLists( gl );
            // if by chance, this request is made when there are bodies,
            // the bodies will need their listsIDs reset
            for ( RigidBody b : system.bodies ) {
                b.geom.myListID = -1;
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
            drawAllObjects( drawable, false );
        } else {                    
            shadowMap.beginLightPass( drawable, ev.trackBall );
            drawAllObjects( drawable, false );        
            shadowMap.endLightPass( drawable, ev.trackBall );
            shadowMap.beginShadowMapping(drawable, ev.trackBall); 
            drawAllObjects( drawable, false );
            shadowMap.endShadowMapping( drawable, ev.trackBall );
        }
        displayAllObjectsNonShadowable( drawable );
        if ( mouseSpring.picked != null ) {
        	gl.glPushMatrix();
        	gl.glScaled( sceneScale,sceneScale,sceneScale );
            gl.glTranslated( sceneTranslation.x, sceneTranslation.y, sceneTranslation.z);

        	int h = drawable.getSurfaceHeight();
        	Point3d p1 = mouseSpring.pointW;
        	Point3d p2 = mouseSpring.grabPointBW;
        	unproject( drawable, mousePressedPoint.x, h-mousePressedPoint.y, zClosest, p1 );

        	mouseSpring.display(drawable);

    		float[] red = new float[] { 1, 0, 0, 1 };		
    		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, red, 0 );
        	gl.glPushMatrix();
        	gl.glTranslated( p1.x, p1.y, p1.z );
        	gl.glColor3f(1, 0, 0);
        	EasyViewer.glut.glutSolidSphere(0.1,16,16);        
        	gl.glPopMatrix();

        	gl.glPushMatrix();
        	gl.glTranslated( p2.x, p2.y, p2.z );
        	gl.glColor3f(1, 0, 0);
        	EasyViewer.glut.glutSolidSphere(0.1,16,16);        
        	gl.glPopMatrix();

        	
        	gl.glPopMatrix();
        }
       
        EasyViewer.beginOverlay(drawable);

        final SimpleDateFormat formatter = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
        gl.glColor4f(0,0,0,1);
        String text = system.name + "\n";
        text += "bodies = " + system.bodies.size() + "\n";        
        text += "contacts = " + system.collision.contacts.size() + "\n";
        text += formatter.format( new Date() ) + "\n";
        text += "simulation time = " + system.simulationTime + "\n";
        text += "total compute time = " + system.totalAccumulatedComputeTime + "\n";
        text += "compute time = " + system.computeTime + "\n";
        text += "collision detection = " + system.collision.collisionDetectTime + "\n";
        text += "collision processing = " + system.collision.collisionSolveTime + "\n";
        text += "h = " + stepsize.getValue() + " (with " + substeps.getValue() + " substeps)\n";
        text += "PGS iterations = " + system.collision.iterations.getValue() + "\n";
        text += "mu = " + system.collision.friction.getValue() + "\n";
        text += "r = " + system.collision.restitution.getValue() +"\n";
        
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
    
	//private final FancyAxis fa = new FancyAxis();

    private void drawAllObjects( GLAutoDrawable drawable, boolean picking ) {
    	GL2 gl = drawable.getGL().getGL2();
        
        gl.glPushMatrix();
        gl.glScaled( sceneScale,sceneScale,sceneScale );
        gl.glTranslated( sceneTranslation.x, sceneTranslation.y, sceneTranslation.z);
        
        if ( useClipPlane.getValue() ) {
        	Vector3d cpn = new Vector3d( clipPlaneNormal.x, clipPlaneNormal.y, clipPlaneNormal.z );
        	cpn.normalize();
	        double[] planeEquation = new double[] { cpn.x, cpn.y, cpn.z, clipPlaneD.getValue() }; 
	        gl.glClipPlane( GL2.GL_CLIP_PLANE0, planeEquation, 0 );
	        gl.glEnable( GL2.GL_CLIP_PLANE0 );
        }
        
        system.display( drawable, picking );

        if ( useClipPlane.getValue() ) {
            gl.glDisable( GL2.GL_CLIP_PLANE0 );
        }
        
        //gl.glScaled( 5,5,5 );
        //fa.draw(gl);
        gl.glPopMatrix();
    }
    
    private void displayAllObjectsNonShadowable( GLAutoDrawable drawable ) {
    	GL2 gl = drawable.getGL().getGL2();
        gl.glPushMatrix();
        gl.glScaled( sceneScale,sceneScale,sceneScale );
        gl.glTranslated( sceneTranslation.x, sceneTranslation.y, sceneTranslation.z);
        system.displayNonShadowable(drawable);
        gl.glPopMatrix();
    }
    
    
    /** For reading the depth of a visible pixel */
    private IntBuffer depthPixels = IntBuffer.allocate( 1 );
	private static ByteBuffer colorPixels = ByteBuffer.allocate( 4 );

	static public void setColorWithID( GL2 gl, int ID ) {
    	gl.glColor3f( (ID & 0xff)/255f, ((ID >> 8) & 0xff)/255f, ((ID>> 16) & 0xff)/255f);
	}
	
	static public int getIDFromColor( ByteBuffer colorPixels ) {
		int r, g, b;
		colorPixels.rewind();
		r = colorPixels.get() & 0xff;
		g = colorPixels.get() & 0xff;
		b = colorPixels.get() & 0xff;
		colorPixels.rewind();
		return r + (g << 8) + (b << 16);		
	}
	
    private void select( GLAutoDrawable drawable, Point p ) {
    	GL2 gl = drawable.getGL().getGL2();

    	gl.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT );
    	gl.glDisable( GL2.GL_LIGHTING );
    	gl.glDisable( GL.GL_BLEND );
    	drawAllObjects( drawable, true );
    	gl.glEnable( GL2.GL_LIGHTING );
    	gl.glEnable( GL.GL_BLEND );
    	
    	int h = drawable.getSurfaceHeight();
		int x = p.x;
		int y = h - p.y;

		colorPixels.rewind();
		gl.glReadPixels( x, y, 1, 1, GL2.GL_RGB, GL.GL_UNSIGNED_BYTE, colorPixels );
		colorPixels.rewind();		
		int ID = getIDFromColor( colorPixels );
		if ( ID > system.bodies.size() ) {
//			System.out.println("picked non-body ID " + ID );
			return;
		}
		
		depthPixels.rewind();
		gl.glReadPixels( x, y, 1, 1, GL2.GL_DEPTH_COMPONENT, GL.GL_UNSIGNED_INT, depthPixels );
		depthPixels.rewind();
		int zint = depthPixels.get(0);
		// System.out.println(Integer.toHexString(zint));
		// convert to [0,1] interval for unproject
        long zL = Integer.toUnsignedLong( zint );
        zClosest = ( (float) zL ) / 0xffffffffL; 
        unproject( drawable, p.x, h-p.y, zClosest, mouseSpring.pointW );
    	mouseSpring.picked = system.bodies.get( ID );
    	mouseSpring.picked.selected = true;
    	mouseSpring.pointW.scale( 1/sceneScale ); // gross to undo the other drawing stuff here... 
    	mouseSpring.pointW.sub( sceneTranslation);
    	
    	Matrix4d A = mouseSpring.picked.transformW2B.T;
    	A.transform(mouseSpring.pointW,  mouseSpring.grabPointB ) ;
//    	System.out.println(" unprojected in world = " +  mouseSpring.pointW  + " index " +  ID );
//    	System.out.println(" unprojected on body = " +  mouseSpring.grabPointB  + " index " +  ID );
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
    
    private BooleanParameter useClipPlane = new BooleanParameter( "use clip plane", false );
    private Vec3Parameter clipPlaneNormal = new Vec3Parameter( "clip plane normal", 0, 1, 1);
    private DoubleParameter clipPlaneD = new DoubleParameter( " clip plane D", 0, -10, 10 );
    
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
        
        JButton load = new JButton("Load PNG");
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
        JButton loadxml = new JButton("Load XML");
        basicControls.add( loadxml );
        loadxml.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                File f = FileSelect.select("xml", "xml", "load", "scenes3d/", true );
                if ( f != null ) {
                    loadXMLSystem( f.getPath() );
                }
            }
        });

        vfp.add( basicControls );
       
        vfp.add( useClipPlane.getControls() );
        vfp.add( clipPlaneNormal );
        vfp.add( clipPlaneD.getSliderControls(false) );
       
        
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
        
        vfp.add( drawWithShadows.getControls() );
        vfp.add( shadowMap.getControls() );
        return vfp.getPanel();
    }
    
    private BooleanParameter drawWithShadows = new BooleanParameter( "draw with shadows", true );

    DoubleParameter whiteEpsilon = new DoubleParameter( "white epsilon", 0.05, 0, 1 );
        
    // parameters and variables for for scaling and translating the window
    private DoubleParameter scale = new DoubleParameter("scale scene",.9, 0.1, 10);
    private DoubleParameter posx = new DoubleParameter("x translation", 0, -1000, 1000 );
    private DoubleParameter posy = new DoubleParameter("y translation", 0, -1000, 1000 );

    // variables and objects for picking rigid body with the mouse
    private MouseSpringForce mouseSpring = new MouseSpringForce();
    
    private double sceneScale = 0.1; // guess how to get images to show up nicely in 3D in a first approximation...
    private Vector3d sceneTranslation = new Vector3d();
    
    /**
     * Loads the specified image, clearing the old system, and resets viewing parameters.
     * @param filename
     */
    private void loadSystem( String filename ) {
        factory.use = false;        
        systemClear();
        system.name = filename;
        ImageBlocker blocker = new ImageBlocker( filename, (float) (double) whiteEpsilon.getValue() );
        sceneTranslation.set( - blocker.width/2, - blocker.height, 0 );
        system.bodies.addAll(blocker.bodies);
        
        for (RigidBody body: system.bodies) {
        	body.friction = system.collision.friction.getValue();
        	body.restitution = system.collision.restitution.getValue();
        }
    }
    
    /**
     * Loads the specified image, clearing the old system, and resets viewing parameters.
     * @param filename
     */
    private void loadXMLSystem( String filename ) {
        factory.use = false;        
        systemClear();
        system.name = filename;
        XMLParser parser = new XMLParser();
        parser.parse( system, filename );
        sceneTranslation.set( 0 ,0, 0 );
        // TODO: dont'override these values later... for now set!
        for (RigidBody body: system.bodies) {
        	body.friction = system.collision.friction.getValue();
        	body.restitution = system.collision.restitution.getValue();
        }
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
    private float zClosest;
   
   
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
