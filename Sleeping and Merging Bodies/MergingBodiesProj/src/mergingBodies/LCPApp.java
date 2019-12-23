package mergingBodies;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.io.File;
import java.io.FilenameFilter;
import java.io.PrintStream;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GL4;
import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JButton;
import javax.swing.JEditorPane;
import javax.swing.JPanel;
import javax.swing.JToggleButton;
import javax.swing.border.TitledBorder;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.FileSelect;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.FlatMatrix4d;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;

/**
 * Main entry point for the application
 * @author kry
 */
public class LCPApp implements SceneGraphNode, Interactor {

    private EasyViewer ev;

    protected RigidBodySystem system = new RigidBodySystem();
    
    private Factory factory = new Factory( system );
   
    private CollisionComputationMonitor ccm = new CollisionComputationMonitor();
    
    private XMLParser xmlParser= new XMLParser(); 
    
    /**
     * Entry point for application
     * @param args
     */
    public static void main(String[] args) {
        new LCPApp();        
    }
    
    private String systemDir;
    /** Creates the application / scene instance */
    public LCPApp() {
    	setUp();
    }
    
    public void setUp() {
        system.mouseSpring = mouseSpring;
        system.mouseImpulse = mouseImpulse;
        String filename = "datalcp/line.png";
       	loadSystem(filename); 

        T.getBackingMatrix().setIdentity();
        ev = new EasyViewer( "Adaptive Merging 3D Rigid Body Simulation", this, new Dimension(1280, 720), new Dimension(640,640) );
        ev.controlFrame.add("Display", system.display.getControls());
        ev.controlFrame.add("Merging", system.merging.getControls());
        ev.controlFrame.add("Sleeping", system.sleeping.getControls());
        ev.controlFrame.add("Collision", system.collision.getControls());
        ev.controlFrame.add("Factory", factory.getControls());
        ev.addInteractor(this); 
    }
    
    @Override
    public void init(GLAutoDrawable drawable) {
        GL gl = drawable.getGL();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
        gl.glDisable( GL2.GL_LIGHTING );
        gl.glClearColor(1,1,1,1);
    }
                
    private FlatMatrix4d T = new FlatMatrix4d();
    
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        
        if ( deleteDisplayListRequest ) {
            deleteDisplayListRequest = false;
            RigidBody.clearDisplayLists( gl );
            // if by chance, this request is made when there are bodies,
            // the bodies will need their listsIDs reset
            for ( RigidBody b : system.bodies ) {
                b.myListID = -1;
            }
        }
        
        windowWidth = drawable.getSurfaceWidth();
        windowHeight = drawable.getSurfaceHeight();
        if ( run.getValue() && (!system.merging.mergingEvent || !system.merging.triggerMergingEvent)) {
            double dt = stepsize.getValue() / (int)substeps.getValue();
            for ( int i = 0; i < substeps.getValue(); i++ ) {
                if ( factory.use ) factory.advanceTime( dt );
                system.advanceTime( dt );                
            }
        }
        
        gl.glDisable( GL.GL_DEPTH_TEST );
        EasyViewer.beginOverlay(drawable);

        gl.glPushMatrix();
        double sw = scale.getValue() * windowWidth / imageWidth;
        double sh = scale.getValue() * windowHeight / imageHeight;
        double s = Math.min(sw,sh);        
        double x = posx.getValue(); // extra translation
        double y = posy.getValue();        
        
        // create this as a transform to use for mouse picking        
        Matrix4d M = T.getBackingMatrix();
        M.setIdentity();
        M.m00 = s;
        M.m11 = s;        
        M.m03 = x + windowWidth /2 - s * (imageWidth-1)/2 ;
        M.m13 = y + windowHeight/2 - s * (imageHeight-1)/2;
        
        gl.glMultMatrixd( T.asArray(),0 );
        
        system.display.display( drawable );
        //clears bodies when using contact graph heuristics
        
        if ( picked != null ) {
            // draw a line from the mouse to the body point
            Point2d tmp = new Point2d();
            picked.transformB2W.transform( grabPointB, tmp );
            if(mouseImpulse.isGrabbing())
            	gl.glColor4f( 1,0.5f,0,0.5f);
            else
            	gl.glColor4f( 1,0,0,0.5f);
            gl.glLineWidth( 5 );
            gl.glBegin( GL.GL_LINES );
            gl.glVertex2d( mousePoint.x, mousePoint.y );
            gl.glVertex2d( tmp.x, tmp.y );
            gl.glEnd();
            gl.glLineWidth(1);
        }

        gl.glPopMatrix();

        final SimpleDateFormat formatter = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
        gl.glColor4f(0,0,0,1);
        String text = "filename = " + system.name + "\n";
        text += "bodies = " + system.bodies.size() + "\n";        
        text += "contacts = " + system.collision.contacts.size() + "\n";
        text += formatter.format( new Date() ) + "\n";
        text += "simulation time = " + system.simulationTime + "\n";
        text += "total compute time = " + system.totalComputeTime + "\n";
        text += "total steps = " + system.totalSteps + "\n";
        text += "compute time = " + system.computeTime + "\n";
        text += "collision detection = " + system.collision.collisionDetectTime + "\n";
        text += "LCP solve = " + system.collision.collisionSolveTime + "\n";
        text += "Single iteration PGS = " + system.collision.collectionUpdateTime + "\n";
        text += "h = " + stepsize.getValue() + " (with " + substeps.getValue() + " substeps)\n";
        text += "PGS iterations = " + system.collision.iterations.getValue() + "\n";
        text += "PGS iterations in collection = " + system.collision.iterationsInCollection.getValue() + "\n";
        text += "mu = " + system.collision.friction.getValue() + "\n";
        text += "r = " + system.collision.restitution.getValue() +"\n";
        
        if ( ! system.display.params.hideOverlay.getValue() ) {
        	EasyViewer.printTextLines( drawable, text, 10, 10, 12, GLUT.BITMAP_HELVETICA_10 );
        }
        
        if (system.saveCSV.getValue()) {
            gl.glColor4f(1,0,0,1);
        	text = "Saving datas to " + system.sceneName + ".csv";
			if (system.merging.params.enableMerging.getValue())
				text = "Saving datas to " + system.sceneName + "_merged.csv";
			EasyViewer.printTextLines( drawable, text, 10, drawable.getSurfaceHeight()-40, 10, GLUT.BITMAP_HELVETICA_10 );
        }
        
    	EasyViewer.endOverlay(drawable);

        if ( system.display.params.drawGraphs.getValue() ) {
        	ccm.draw(drawable);
        }
        
        if ( run.getValue() || stepped ) {
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
    
    /**
     * Converts from screen coordinates to image coordinates
     * @param x
     * @param y
     */
    private void setPoint( int x, int y ) {
        Point3d tmp = new Point3d(x,y,0);
        Matrix4d M = new Matrix4d();
        M.invert(T.getBackingMatrix());
        M.transform( tmp );
        mousePoint.x = tmp.x;
        mousePoint.y = tmp.y;    
    }
    
    private BooleanParameter record = new BooleanParameter( "record", false );
    
    /** 
     * boolean to signal that the system was stepped and that a 
     * frame should be recorded if recording is enabled
     */
    private boolean stepped = false;
        
    private String dumpName = "dump";
    
    private int nextFrameNum = 0;
    
    private NumberFormat format = new DecimalFormat("00000");

    private BooleanParameter run = new BooleanParameter( "simulate", false );
    private DoubleParameter stepsize = new DoubleParameter( "step size", 0.05, 1e-5, 1 );
    private IntParameter substeps = new IntParameter( " sub steps (integer)", 1, 1, 100);
    public BooleanParameter loadXML = new BooleanParameter( "load XML", true);
    
    /** Creates a control panel for changing visualization and simulation parameters */
    @Override
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        
        JEditorPane ta = new JEditorPane("text/html", 
        		"   <b>KEYBOARD</b><br/>" +          		
        		"   space : start and stop simulation <br/>" +
        		"   enter : start and stop recording <br/> " +
        		"   m : simulate up to next merge or unmerge <br/> " +
        		"   l : load a file <br/> " +
        		"   &larr : previous scene<br/> " +
        		"   &rarr : next scene<br/> " +
        		"   1-7 : left right up down spring up down and toggle magnet<br/> "+
        		"   <b>MOUSE </b><br/>" +
        		"   left drag : mouse spring<br/> " +
                "   shift + left drag : mouse impulse<br/> " +
                "   middle drag : translate scene <br/> " +
        		"   right drag : zoom in and out <br/> ");    
        ta.setEditable(false);
        ta.setBorder( new TitledBorder("Keyboard and Mouse controls") );
        vfp.add( ta );
        
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
        
		JToggleButton save = new JToggleButton("save CSV");
        vfp.add( save);
        save.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	system.saveCSV.setValue(!system.saveCSV.getValue());
            }
        });

        JPanel windowsSizeControls = new JPanel( new GridLayout(1,3));
        JButton res1 = new JButton("640x360");
        windowsSizeControls.add( res1);
        res1.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                ev.glCanvas.setSize( 640, 360 );
                ev.frame.setSize( ev.frame.getPreferredSize() );
            }
        });        
        JButton res2 = new JButton("1280x720");
        windowsSizeControls.add( res2);
        res2.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {                
                ev.glCanvas.setSize( 1280, 720 );
                ev.frame.setSize( ev.frame.getPreferredSize() );

            }
        });                
        vfp.add( windowsSizeControls );
        JButton res3 = new JButton("2240x1280");
        windowsSizeControls.add( res3);
        res3.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {                
                ev.glCanvas.setSize( 2240, 1280 );
                ev.frame.setSize( ev.frame.getPreferredSize() );

            }
        });                
        vfp.add( windowsSizeControls );
        
        HorizontalFlowPanel hfp2 = new HorizontalFlowPanel();
        hfp2.add( record.getControls() );
        vfp.add( hfp2.getPanel() );
        
        vfp.add( loadXML.getControls() );
        
        vfp.add( run.getControls() );
        vfp.add( stepsize.getSliderControls(true) );
        vfp.add( substeps.getControls() );
        vfp.add( system.getControls() );
        
        vfp.add( mouseSpring.getControls() );
        vfp.add( mouseImpulse.getControls() );
        
        VerticalFlowPanel vfpv = new VerticalFlowPanel();
        vfpv.setBorder( new TitledBorder("Window content scaling controls") );
        vfpv.add( scale.getSliderControls(true) );
        vfpv.add( posx.getSliderControls(false) );
        vfpv.add( posy.getSliderControls(false) );
        vfp.add( vfpv.getPanel() );
        
        vfp.add( whiteEpsilon.getSliderControls(false) );
        
        return vfp.getPanel();
    }
    
    protected DoubleParameter whiteEpsilon = new DoubleParameter( "white epsilon", 0.05, 0, 1 );
        
    // parameters and variables for for scaling and translating the window
    private DoubleParameter scale = new DoubleParameter( "scale scene", 0.9, 0.1, 10 );
    private DoubleParameter posx = new DoubleParameter( "x translation", 0, -1000, 1000 );
    private DoubleParameter posy = new DoubleParameter( "y translation", 0, -1000, 1000 );
    private double windowWidth = 1.0;
    private double windowHeight= 1.0;
    private double imageWidth = 1.0;
    private double imageHeight= 1.0;
    private Point prevMousePoint = new Point();

    // variables and objects for picking rigid body with the mouse
    private RigidBody picked = null;
    private Point2d grabPointB = new Point2d();
    private Point2d mousePoint = new Point2d();    
    private MouseSpringForce mouseSpring = new MouseSpringForce( mousePoint );
    private MouseImpulse mouseImpulse = new MouseImpulse();
	PrintStream stream = null;
	
    /**
     * Loads the specified image, clearing the old system, and resets viewing parameters.
     * @param filename
     */
    protected void loadSystem( String filename ) {
        factory.use = false;        
        systemClear();
        system.name = filename;
        systemDir = filename;
        ImageBlocker blocker = new ImageBlocker( filename, (float) (double) whiteEpsilon.getValue() );
        imageWidth  = blocker.width;
        imageHeight = blocker.height;
        system.bodies.clear();
        system.bodies.addAll(blocker.bodies);
        system.controllableSprings = blocker.controllableSprings;
    	system.collision.bodyPairContacts.clear();
    	system.collision.contacts.clear();
    	system.sceneName = filename.substring(0, filename.length() - 4);
    	for (RigidBody body: system.bodies) {
    		body.restitution = system.collision.restitution.getValue();
    		body.friction = system.collision.friction.getValue();
    	}
    	if(loadXML.getValue())
    		xmlParser.parse(system, systemDir);
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
        imageWidth = blocker.width;
        imageHeight= blocker.height;
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
            system.reset(); // eulalie: I'm not sure why we do that?
        }
        
        loadSystem(systemDir);
    	if(loadXML.getValue())
    		xmlParser.parse(system, systemDir);
        nextFrameNum = 0;
    }
    
    private boolean deleteDisplayListRequest = false;
    
    /**
     * Clears the system, and ensures that any display lists that were created
     * to draw the various rigid bodies are cleaned up on the next display call.
     */
    protected void systemClear() {
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
    
    private boolean shiftPressed = false;
    
    /** Attaches mouse and keyboard listeners to the canvas. */
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
        
        component.addMouseMotionListener( new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                setPoint( e.getPoint().x, e.getPoint().y );
                if ( picked == null ) {
                    if ( (e.getModifiers() & InputEvent.BUTTON2_MASK) != 0) { // button3 ) {
                        posx.setValue( posx.getValue() + e.getPoint().x - prevMousePoint.x );
                        posy.setValue( posy.getValue() + e.getPoint().y - prevMousePoint.y );
                    }
                    if ( (e.getModifiers() & InputEvent.BUTTON3_MASK) != 0 ) {
                        scale.setValue( scale.getValue() * Math.pow(1.002, e.getPoint().y - prevMousePoint.y ));
                    }
                }
                prevMousePoint.setLocation( e.getPoint() );
            }           
        } );
        
        component.addMouseListener( new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {     
            	
                prevMousePoint.setLocation( e.getPoint() );
                setPoint( e.getPoint().x, e.getPoint().y );
                if ( (e.getModifiers() & InputEvent.BUTTON1_MASK) != 0 ) {
                    picked = system.pickBody( mousePoint );                
                    if ( picked != null ) {
                        picked.transformW2B.transform( mousePoint, grabPointB );                
                    } 
                }
                
                if(shiftPressed) {
                	if(!mouseImpulse.isGrabbing()) {
                		mouseImpulse.grab( picked, mousePoint ); 
                	}
            	} else {
            		mouseSpring.setPicked( picked, grabPointB );    
            	}
            }
            
            @Override
            public void mouseReleased(MouseEvent e) {
                picked = null; 
        		if(mouseImpulse.isGrabbing()) {
            		mouseImpulse.release( mousePoint );
            	} else {               
                    mouseSpring.setPicked( picked, grabPointB );	
            	}
            }
        } );
        
        component.addKeyListener( new KeyAdapter() {
        	
        	@Override
        	public void keyReleased(KeyEvent e) {
        		if ( e.getKeyCode() == KeyEvent.VK_SHIFT ) {
                	shiftPressed = false;
	            } 
        	}
        	
            @Override
            public void keyPressed(KeyEvent e) {
            	double ds = 0.5; // step size for moving controllable springs (might want to expose this as a parameter!)
            	
                if ( e.getKeyCode() == KeyEvent.VK_SHIFT ) {
                	shiftPressed = true;
	            }
            	
                if ( e.getKeyCode() == KeyEvent.VK_SPACE ) {

                	if(system.merging.triggerMergingEvent) {
                    	system.merging.triggerMergingEvent = false;
                        run.setValue(true); 
                	} else {
                		run.setValue( ! run.getValue() ); 
                	}
                }  
                else if ( e.getKeyCode() == KeyEvent.VK_S ) {
                    double dt = stepsize.getValue() / (int)substeps.getValue();
                    for ( int i = 0; i < substeps.getValue(); i++ ) {
                        if ( factory.use ) factory.advanceTime( dt );
                        system.advanceTime( dt );                
                    }
                    stepped = true;
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_M ) {
                	system.merging.triggerMergingEvent = true;
                	system.merging.mergingEvent = false;
                    run.setValue( true );       
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_R ) {                    
                	double px = posx.getValue();
                    double py = posy.getValue();
                    double s = scale.getValue();
                    systemReset();
                	posx.setValue(px);
                    posy.setValue(py);
                    scale.setValue(s);
                    nextFrameNum = 0;
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_A ) {
                	scale.setValue( imageWidth / windowWidth );
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_C ) {                   
                    systemClear();
                    factory.use = false;
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_J ) {                   
                    system.jiggle();                                        
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_G ) {                   
                   File f = FileSelect.select("png", "image for factory", "load", "datalcp/", true );
                    if ( f != null ) {
                        loadFactorySystem( f.getPath() );
                    }   
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_F ) {                                       
                    loadFactorySystem( "datalcp/tetrisTube.png" );
                    factory.spread.setValue(30);
                    factory.interval.setValue(0.4);
                    factory.downVelocity.setValue(10.0);
                    factory.angularVelocityScale.setValue(0.5);
                    factory.linearVelocityScale.setValue(2.5);
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_PERIOD ) {                                       
                    factory.run.setValue ( ! factory.run.getValue() );
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_COMMA ) {
                    factory.createBodyRequest = true;
                }
                else if ( e.getKeyCode() == KeyEvent.VK_N) {                   
                    	system.generateBody= true;
                }
                else if (  e.getKeyCode() == KeyEvent.VK_L ) {                    
                    File f = FileSelect.select("png", "image", "load", "datalcp/", true );
                    if ( f != null ) {
                        loadSystem( f.getPath() );
                    }
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_LEFT) {
                    if ( files != null && files.length >= 0 ) {
                        whichFile --;
                        if ( whichFile < 0 ) whichFile = files.length-1;
                        loadSystem( files[whichFile].getPath() );                        
                    }
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_RIGHT ) {
                    if ( files != null && files.length >= 0 ) {
                        whichFile ++;
                        if ( whichFile >= files.length ) whichFile = 0;
                        loadSystem( files[whichFile].getPath() );
                    }
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_ESCAPE ) {
                    ev.stop();
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_ENTER ) {
                    record.setValue( ! record.getValue() );
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_UP ) {
                    int ss = substeps.getValue();
                    if ( ss == substeps.getMaximum() ) return;
                    substeps.setValue( ss + 1 );
                    stepsize.setValue( stepsize.getValue() * (ss+1)/ss );
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_DOWN ) {
                    int ss = substeps.getValue();
                    if ( ss == substeps.getMinimum() ) return;
                    substeps.setValue( ss - 1 );
                    stepsize.setValue( stepsize.getValue() *(ss-1)/ss );
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_1 ) {
                	for ( Spring s : system.controllableSprings ) {
                		s.moveWorldAttachmentAndRestLength( -ds, 0, 0 );
                	}
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_2 ) {
                	for ( Spring s : system.controllableSprings ) {
                		s.moveWorldAttachmentAndRestLength( ds, 0, 0 );
                	}
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_3 ) {
                	for ( Spring s : system.controllableSprings ) {
                		s.moveWorldAttachmentAndRestLength( 0, -ds, 0 );
                	}
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_4 ) {
                	for ( Spring s : system.controllableSprings ) {
                		s.moveWorldAttachmentAndRestLength( 0, ds, 0 );
                	}
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_5 ) {
                	for ( Spring s : system.controllableSprings ) {
                		s.moveWorldAttachmentAndRestLength( 0, 0, -ds );
                	}
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_6 ) {
                	for ( Spring s : system.controllableSprings ) {
                		s.moveWorldAttachmentAndRestLength( 0, 0, ds );
                	}                    	
                } 
                else if ( e.getKeyCode() == KeyEvent.VK_7 ) {
                	for ( RigidBody b : system.bodies ) {
              
                		if (b instanceof RigidCollection) {
                			RigidCollection collection = (RigidCollection)b;
                			for ( RigidBody sb : collection.bodies ) {
                				if(sb.magnetic)
                					sb.activateMagnet = !sb.activateMagnet;
                			}
                		}
                		
                		if(b.magnetic)
                			b.activateMagnet = !b.activateMagnet;
                	}                    	
                } 
            }
        } );
    }
    
}
