package mergingBodies3D;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.Font;
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
import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.JButton;
import javax.swing.JEditorPane;
import javax.swing.JPanel;
import javax.swing.JToggleButton;
import javax.swing.border.TitledBorder;
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
import mintools.viewer.EasyViewerAnim;
import mintools.viewer.FlatMatrix4d;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;
import mintools.viewer.ShadowMap;

/**
 * Main entry point for the application
 * @author kry
 */
public class LCPApp3D implements SceneGraphNode, Interactor {

	public boolean osx;
	
    private EasyViewerAnim eva;

    private RigidBodySystem system = new RigidBodySystem();
    
    private Factory factory = new Factory( system );

    private CollisionComputationMonitor ccm = new CollisionComputationMonitor();
    private PGSConvergenceMonitor pgsm = new PGSConvergenceMonitor();
    
    private String sceneFilename = "scenes3D/factoryTest2.xml";
    
    /**
     * Creates a shadow map with a square image, e.g., 1024x1024.
     * This number can be reduced to improve performance.
     */
    private ShadowMap shadowMap;
    
    private boolean loadSystemRequest = false;
    private boolean loadFactoryRequest = false;
    private boolean clearRequest = false;
    
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
    	
    	osx = (System.getProperty("os.name").contains("Mac"));
    	if (!osx) 
    		shadowMap = new ShadowMap( 2048 );
    	else {
    		drawWithShadows.setValue(false);
    		drawWithShadows.setDefaultValue(false);
    	}
    	
        system.mouseSpring = mouseSpring;
        system.mouseImpulse = mouseImpulse;
        loadXMLSystem(sceneFilename);
        T.getBackingMatrix().setIdentity();
        EasyViewerAnim.antialiasing = true;        
        eva = new EasyViewerAnim( "Adaptive Merging 3D Rigid Body Simulation", this, new Dimension(640,360), new Dimension(640,480) );
        eva.controlFrame.add("Display", system.display.getControls());
        eva.controlFrame.add("Merging", system.merging.getControls());
        eva.controlFrame.add("Sleeping", system.sleeping.getControls());
        eva.controlFrame.add("Factory", factory.getControls());
        eva.controlFrame.add("Animation", system.animation.getControls());
        eva.controlFrame.add("Help", getHelpPanel() );
        eva.controlFrame.setSelectedTab("Help");

        eva.addInteractor(this);       
        
        eva.trackBall.setFocalDistance(10);
        eva.trackBall.near.setValue(5);
    }
     
    @Override
    public void init(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
        if (!osx)
        	shadowMap.init(drawable); 
        gl.glClearColor(1,1,1,1);
        gl.glEnable( GL.GL_CULL_FACE );
    }
                
    private FlatMatrix4d T = new FlatMatrix4d();
    
    @Override
    public void display(GLAutoDrawable drawable) {
    	
    	if(clearRequest) {
            clearRequest = false;
    		systemClear();
        	system.collision.contactPool.clear(); 
            System.gc();
            if (factory.use)
        		loadFactoryRequest = false;
            else
            	loadSystemRequest = true;
            factory.use = false;
    	}
    	
    	if(loadSystemRequest) {
    		loadSystemRequest = false;
    		loadXMLSystem(sceneFilename);
    	}
    	
    	if(loadFactoryRequest) {
    		loadFactoryRequest = false;
    		loadFactorySystem(sceneFilename);
    	}
    	
        GL2 gl = drawable.getGL().getGL2();
                
        if ( selectRequest ) {
        	selectRequest = false;
    		select(drawable, mousePressedPoint );
    		if ( system.display.params.enableQMG.getValue() ) {
    			lastSelectedBody = mouseSpring.picked;
    			if ( lastSelectedBody != null ) {
    				qmgMetric.setNameAndClear( "BPC metric on " + lastSelectedBody.name );
    				qmgContact.setNameAndClear( "contact slip on " + lastSelectedBody.name );    				
    			}
    		}
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
        
        double dt = stepsize.getValue() / substeps.getValue();
        if ( run.getValue() && (!system.merging.mergingEvent || !system.merging.triggerMergingEvent)) {
            for ( int i = 0; i < substeps.getValue(); i++ ) {
                if ( factory.use ) factory.advanceTime( dt );
                system.advanceTime( dt );                
            }

            if ( system.display.params.enableQMG.getValue() ) {
            	if ( lastSelectedBody != null ) {
            		for ( BodyPairContact bpc : lastSelectedBody.bodyPairContacts ) {
            			if ( bpc.motionMetricHist.size() > 0 ) {
            				qmgMetric.add( bpc.motionMetricHist.get( bpc.motionMetricHist.size()-1 ) );
            			}
            			for ( Contact c : bpc.contactList ) {
            				qmgContact.add( Math.abs( c.w1 ) );
            				qmgContact.add( Math.abs( c.w2 ) );
            			}
            		}
            		qmgMetric.step();
            		qmgContact.step();
            	}
            }
        }
        
        

        if ( ! drawWithShadows.getValue() ) {
            drawAllObjects( drawable, false );
        } else {                    
            shadowMap.beginLightPass( drawable, eva.trackBall );
            drawAllObjects( drawable, false );        
            shadowMap.endLightPass( drawable, eva.trackBall );
            shadowMap.beginShadowMapping(drawable, eva.trackBall); 
            drawAllObjects( drawable, false );
            shadowMap.endShadowMapping( drawable, eva.trackBall );
        }
        displayAllObjectsNonShadowable( drawable,dt );
        if ( mouseSpring.picked != null || mouseImpulse.isGrabbing() ) {
        	gl.glPushMatrix();
        	gl.glScaled( sceneScale,sceneScale,sceneScale );
            gl.glTranslated( sceneTranslation.x, sceneTranslation.y, sceneTranslation.z);

        	int h = drawable.getSurfaceHeight();
        	Point3d p1 = (selectRequestType == SelectRequestType.SPRING)? mouseSpring.pointW : mouseImpulse.getEndPoint();
        	Point3d p2 = (selectRequestType == SelectRequestType.SPRING)? mouseSpring.grabPointBW : mouseImpulse.getPickedPointW();
        	unproject( drawable, mousePressedPoint.x, h-mousePressedPoint.y, zClosest, p1 );

        	if (selectRequestType == SelectRequestType.SPRING)
        		mouseSpring.display(drawable);
        	else {
        		mouseImpulse.display(drawable);
        	}

    		float[] color = (selectRequestType == SelectRequestType.SPRING)? new float[] { 1, 0, 0, 1 }: new float[] { 1, 0.5f, 0, 1 };		
    		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, color, 0 );
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

        gl.glColor4f(0,0,0,1);
        // perhaps goofy to do this as it is just going to get tokenized afterwards to put it on the screen.
        stringBuilder.delete(0, stringBuilder.length());
        stringBuilder.append( system.name ); stringBuilder.append( '\n' );
        stringBuilder.append( "bodies = "); stringBuilder.append( system.bodies.size()); stringBuilder.append( '\n' );        
        stringBuilder.append( "contacts = "); stringBuilder.append( system.collision.contacts.size()); stringBuilder.append( '\n' );
        stringBuilder.append( formatter.format( new Date() )); stringBuilder.append( '\n' );
        stringBuilder.append( "simulation time = "); stringBuilder.append( system.simulationTime); stringBuilder.append( '\n' );
        stringBuilder.append( "total compute time = "); stringBuilder.append( system.totalAccumulatedComputeTime); stringBuilder.append( '\n' );
        stringBuilder.append( "compute time = "); stringBuilder.append( system.computeTime); stringBuilder.append( '\n' );
        stringBuilder.append( "collision detection = "); stringBuilder.append( system.collision.collisionDetectTime); stringBuilder.append( '\n' );
        stringBuilder.append( "LCP solve = "); stringBuilder.append( system.collision.collisionSolveTime); stringBuilder.append( '\n' );
        stringBuilder.append( "Single iteration solve = "); stringBuilder.append( system.collision.collectionUpdateTime); stringBuilder.append( '\n' );
        stringBuilder.append( "Merging = "); stringBuilder.append( system.mergingTime); stringBuilder.append( '\n' );
        stringBuilder.append( "Unmerging = "); stringBuilder.append( system.unmergingTime); stringBuilder.append( '\n' );
        stringBuilder.append( "h = "); 
        stringBuilder.append( stepsize.getValue()); 
        stringBuilder.append( " (with "); 
        stringBuilder.append( substeps.getValue()); 
        stringBuilder.append( " substeps)\n" );
        stringBuilder.append( "PGS iterations = "); stringBuilder.append( system.collision.iterations.getValue()); stringBuilder.append( '\n' );
        //stringBuilder.append( "mu = "); stringBuilder.append( system.collision.friction.getValue()); stringBuilder.append( '\n' );
        //stringBuilder.append( "r = "); stringBuilder.append( system.collision.restitution.getValue()); stringBuilder.append( '\n' );
        stringBuilder.append( "(box-box) badWarmStarts = "); stringBuilder.append( system.collision.badWarmStarts); stringBuilder.append( '\n' );
        stringBuilder.append( "(box-box) badWarmStartsRepaired = "); stringBuilder.append( system.collision.badWarmStartsRepaired); stringBuilder.append( '\n' );
        stringBuilder.append( "timestep =  "); stringBuilder.append( system.timestep); stringBuilder.append( '\n' );
        Runtime rt = Runtime.getRuntime();
        long usedMem = (rt.totalMemory() - rt.freeMemory()) / 1024; // / 1024;
        double alpha = 0.01;
        memDelta = 0;
        if ( lastUsedMem != -1) {
        	memDelta = usedMem - lastUsedMem;
        	filteredUsedMem = alpha * ( usedMem - lastUsedMem ) + (1-alpha)*filteredUsedMem;
        }
        lastUsedMem = usedMem;
        long f2fTime = 0;
		long now = System.nanoTime();
    	if ( lastFrameTime != -1 ) {
    		f2fTime = now-lastFrameTime;
    	}
		lastFrameTime = now;
    	
		memMonitor.add( memDelta );
    	//computeTimeMonitor.add( system.computeTime * 1000 );
		computeTimeMonitor.add( f2fTime / 1e6 );
        
        stringBuilder.append( "MEMORY USED = "); stringBuilder.append( usedMem ); stringBuilder.append( " KB \n" );
                stringBuilder.append( "MEMORY DELTA = "); stringBuilder.append( memDelta ); stringBuilder.append( " KB \n" );
        stringBuilder.append( "FILTERED DELTA = "); stringBuilder.append( (int) filteredUsedMem ); stringBuilder.append( " KB \n" );
        
        if ( ! system.display.params.hideOverlay.getValue() ) {
        	EasyViewer.printTextLines( drawable, stringBuilder.toString(), 10, 10, 12, GLUT.BITMAP_HELVETICA_10 );
        }
        
        if (system.saveCSV.getValue()) {
            gl.glColor4f(1,0,0,1);
            String text = "Saving datas to " + system.sceneName + ".csv";
			if (system.merging.params.enableMerging.getValue())
				text = "Saving datas to " + system.sceneName + "_merged.csv";
			EasyViewer.printTextLines( drawable, text, 10, drawable.getSurfaceHeight()-40, 10, GLUT.BITMAP_HELVETICA_10 );
        }
        
    	EasyViewer.endOverlay(drawable);

        if ( system.display.params.drawGraphs.getValue() ) {
        	ccm.draw(drawable);
        }
        
        if ( system.display.params.drawPGSGraph.getValue() ) {
        	pgsm.draw(drawable);
        }
        
        if ( system.display.params.drawMemGraphs.getValue() ) {
        	memMonitor.draw( drawable, 0 );
        	computeTimeMonitor.draw( drawable, 1);
        }
        if ( system.display.params.drawQMG.getValue() ) {
        	qmgMetric.draw(drawable, 0);
        	qmgContact.draw(drawable, 1);
        }
        
        if ( run.getValue() || stepped ) {
        	if ( system.display.params.drawGraphs.getValue() ) {
	            ccm.monitor(system);
        	}
        	if ( system.display.params.drawPGSGraph.getValue() ) {
	            pgsm.monitor(system);
        	}
        	stepped = false;        
            if ( record.getValue() ) {
                // write the frame
                File file = new File( "stills/" + dumpName + format.format(nextFrameNum) + ".png" );                                             
                nextFrameNum++;
                file = new File(file.getAbsolutePath().trim());
                eva.snapshot(drawable, file);
            }
        }
    }
    
    /** Last selected body is used for the quantity harvesting */
    RigidBody lastSelectedBody;
    private QuantityMonitorGraph qmgMetric = new QuantityMonitorGraph("Metric","units");
    private QuantityMonitorGraph qmgContact = new QuantityMonitorGraph("Metric","units");
    
    private MemoryMonitor memMonitor = new MemoryMonitor( "memory", "step");
    private MemoryMonitor computeTimeMonitor = new MemoryMonitor( "compute time", "step");
    private long lastUsedMem = -1;
    private double filteredUsedMem = 0;
    private long memDelta = 0;
    private long lastFrameTime = -1;
    private final SimpleDateFormat formatter = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
    private final StringBuilder stringBuilder = new StringBuilder();
    
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
        
        system.display.display( drawable, picking );

        if ( useClipPlane.getValue() ) {
            gl.glDisable( GL2.GL_CLIP_PLANE0 );
        }
        
        //gl.glScaled( 5,5,5 );
        //fa.draw(gl);
        gl.glPopMatrix();
    }
    
    /**
     * Non shadowable objects include contact force visualizaitons (which need dt) 
     * @param drawable
     * @param dt
     */
    private void displayAllObjectsNonShadowable( GLAutoDrawable drawable, double dt ) {
    	GL2 gl = drawable.getGL().getGL2();
        gl.glPushMatrix();
        gl.glScaled( sceneScale,sceneScale,sceneScale );
        gl.glTranslated( sceneTranslation.x, sceneTranslation.y, sceneTranslation.z);
        system.display.displayNonShadowable(drawable, dt);
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
		if ( ID >= system.display.bodiesDrawnForPicking.size() ) {
			System.err.println("picking problm: ID too large! " + ID );
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
        
        Point3d pointW = new Point3d();
        Point3d grabPointB = new Point3d();
        RigidBody body;
        
        unproject( drawable, p.x, h-p.y, zClosest, pointW );
        body = system.display.bodiesDrawnForPicking.get( ID );
    	pointW.scale( 1/sceneScale ); // gross to undo the other drawing stuff here... 
    	pointW.sub( sceneTranslation);
    	
//    	Matrix4d A = body.transformW2B.T;
//    	A.transform(pointW,  grabPointB ) ;
        
    	body.transformB2W.inverseTransform( pointW, grabPointB );
    	
        if (selectRequestType == SelectRequestType.SPRING) {
	    	mouseSpring.picked = body;
	    	mouseSpring.pointW.set(pointW); 
	    	mouseSpring.grabPointB.set(grabPointB);
        } else if (selectRequestType == SelectRequestType.IMPULSE) {
        	mouseImpulse.grab(body, grabPointB);
        	mouseImpulse.hold(pointW);
        } else if (selectRequestType == SelectRequestType.ANIMATION) {
        	system.animation.setSelection(body.name);
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
    
    private BooleanParameter run = new BooleanParameter( "simulate", false );
    private DoubleParameter stepsize = new DoubleParameter( "step size", 0.05, 1e-5, 1 );
    private IntParameter substeps = new IntParameter( "sub steps (integer)", 1, 1, 100);
    
    
    private BooleanParameter useClipPlane = new BooleanParameter( "use clip plane", false );
    private Vec3Parameter clipPlaneNormal = new Vec3Parameter( "clip plane normal", 0, 1, 1);
    private DoubleParameter clipPlaneD = new DoubleParameter( " clip plane D", 0, -10, 10 );
    
    public JPanel getHelpPanel() {
    	VerticalFlowPanel vfp = new VerticalFlowPanel();
        
    	VerticalFlowPanel vfp1 = new VerticalFlowPanel();
    	try {
        	URL urlXMLSpec  = getClass().getResource( "resources/interfaceHelp.html" );
    		JEditorPane ta = new JEditorPane( urlXMLSpec );        
    		ta.setEditable(false);
    		vfp1.add( ta );
    	} catch ( Exception e ) {
    		e.printStackTrace();
    	}
        vfp1.setBorder( new TitledBorder("Keyboard and Mouse controls") );
        ((TitledBorder) vfp1.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));
        CollapsiblePanel cp = new CollapsiblePanel( vfp1.getPanel() );        
        vfp.add( cp );
        
    	VerticalFlowPanel vfp2 = new VerticalFlowPanel();
    	try {
    		URL urlXMLSpec  = getClass().getResource( "resources/xmlSpec.html" );
        	JEditorPane ta2 = new JEditorPane( urlXMLSpec );        
    		ta2.setEditable(false);
    		vfp2.add( ta2 );
    	} catch ( Exception e ) {
    		e.printStackTrace();
    	}
        vfp2.setBorder( new TitledBorder("XML documentation") );
        ((TitledBorder) vfp2.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));
        CollapsiblePanel cp2 = new CollapsiblePanel( vfp2.getPanel() );
        vfp.add( cp2 );
        
        return vfp.getPanel();
    }
    

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
                clearRequest = true;
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
        
        JToggleButton save = new JToggleButton("save CSV");
        vfp.add( save);
        save.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	system.saveCSV.setValue(!system.saveCSV.getValue());
            }
        });
        
        HorizontalFlowPanel hfp2 = new HorizontalFlowPanel();
        hfp2.add( record.getControls() );
        JButton res1 = new JButton("640x360");
        hfp2.add( res1);
        res1.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                eva.glCanvas.setSize( 640, 360 );
                eva.frame.setSize( eva.frame.getPreferredSize() );
            }
        });        
        JButton res2 = new JButton("1280x720");
        hfp2.add( res2);
        res2.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {                
                eva.glCanvas.setSize( 1280, 720 );
                eva.frame.setSize( eva.frame.getPreferredSize() );

            }
        });     
        JButton res3 = new JButton("1920x1080");
        hfp2.add( res3);
        res3.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {                
                eva.glCanvas.setSize( 1920, 1080 );
                eva.frame.setSize( eva.frame.getPreferredSize() );

            }
        });  
        vfp.add( hfp2.getPanel() );
        
        vfp.add( run.getControls() );
        vfp.add( stepsize.getSliderControls(true) );
        vfp.add( substeps.getControls() );
        vfp.add( system.collision.getControls() );
        vfp.add( system.getControls() );
       
        vfp.add( mouseSpring.getControls() );
        vfp.add( mouseImpulse.getControls() );
        vfp.add( system.getMoveableSpringControls());
                
        VerticalFlowPanel vfpsm = new VerticalFlowPanel();
        vfpsm.setBorder(new TitledBorder("Shadow map controls") );
        ((TitledBorder) vfpsm.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));

        vfpsm.add( drawWithShadows.getControls() );
        if (!osx)
        	vfpsm.add( shadowMap.getControls() );
        CollapsiblePanel smcp = new CollapsiblePanel(vfpsm.getPanel());
        vfp.add( smcp );
        
        VerticalFlowPanel vfpclip = new VerticalFlowPanel();
        vfpclip.setBorder(new TitledBorder("Clip plane controls (turn off shadows to use)") );
        ((TitledBorder) vfpclip.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));
        vfpclip.add( useClipPlane.getControls() );
        vfpclip.add( clipPlaneNormal );
        vfpclip.add( clipPlaneD.getSliderControls(false) );
        CollapsiblePanel cpclip = new CollapsiblePanel( vfpclip.getPanel() );
        
        vfp.add( cpclip );
        
        return vfp.getPanel();
    }
    
    private BooleanParameter drawWithShadows = new BooleanParameter( "draw with shadows", true );
        

    // variables and objects for picking rigid body with the mouse
    private MouseSpringForce mouseSpring = new MouseSpringForce();
    private MouseImpulse mouseImpulse = new MouseImpulse();
    
    private double sceneScale = 0.1; // guess how to get images to show up nicely in 3D in a first approximation...
    private Vector3d sceneTranslation = new Vector3d();
    
    /**
     * Loads the specified image, clearing the old system, and resets viewing parameters.
     * @param filename
     */
    private void loadXMLSystem( String filename ) {
    	this.sceneFilename = filename;
    	system.sceneName = filename.substring(0, filename.length() - 4);
    	System.out.println("loading " + filename );
        factory.use = false;
        systemClear();
        system.name = filename;
        system.timestep = 0;
        XMLParser parser = new XMLParser();
        parser.parse( system, filename );
        system.animation.init(system.bodies);
        sceneTranslation.set( 0 ,0, 0 );
    }
    
    /**
     * Loads the specified image as a factory, clearing the old system, and resets viewing parameters.
     * @param filename
     */
    private void loadFactorySystem( String filename ) {    
    	this.sceneFilename = filename;
    	system.sceneName = filename.substring(0, filename.length() - 4);
    	factory.use = false; 
    	systemClear();
        loadXMLSystem( filename );
        system.name = filename + " factory";
        factory.setSystem(system);
        factory.use = true;
        factory.reset();        
    }
    
    /**
     * Clears the system, and ensures that any display lists that were created
     * to draw the various rigid bodies are cleaned up on the next display call.
     */
    private void systemClear() {
    	// TODO: avoid rebuilding display lists if only resetting a factory
    	// also... awkward way to try to delete the display lists as the 
    	// display loop is not really going to get a chance to be called before
    	// the system.clear call...   In any case, not a big deal if we only 
    	// load systems occasionally.
    	deleteDisplayListRequest = true;  
    	system.clear();
    	nextFrameNum=0;
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
    
    /** List of png files in the data folder which can be loaded with left and right arrow */
    private File[] files = null;
    
    /** current index in the files list */
    private int whichFile = 0;
    
    private Point mousePressedPoint = null;
    private boolean selectRequest = false;
    private enum SelectRequestType {SPRING, IMPULSE, ANIMATION};
    private SelectRequestType selectRequestType;
    
    /** [0,1] z position from opengl picking to use with unproject */
    private float zClosest;
   
    private GLU glu = new GLU();
    
    /**
     * Attaches mouse and keyboard listeners to the canvas.
     */
    @Override
    public void attach(Component component) {
    
        File directory = new File("scenes3D");
        files = directory.listFiles(new FilenameFilter() {            
            @Override
            public boolean accept( File dir, String name ) {
                return name.endsWith(".xml");                
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
        		if(mouseImpulse.isGrabbing()) {
            		mouseImpulse.release();
            	} 
        		
        		if ( mouseSpring.picked != null ) {
					mouseSpring.picked = null;
				}
            }
			
			@Override
			public void mousePressed(MouseEvent e) {
				if ( e.getButton() == 1 && e.isShiftDown() ) {
					mousePressedPoint = e.getPoint();
					selectRequest = true;
					selectRequestType = SelectRequestType.SPRING;
				}
				
				if( e.getButton() == 2 && e.isControlDown() ) {
					mousePressedPoint = e.getPoint();
					selectRequest = true;
					selectRequestType = SelectRequestType.IMPULSE;
            	}
				
				if( e.getButton() == 1 && e.isControlDown() ) {
					mousePressedPoint = e.getPoint();
					selectRequest = true;
					selectRequestType = SelectRequestType.ANIMATION;
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
                	if(system.merging.triggerMergingEvent) {
                    	system.merging.triggerMergingEvent = false;
                        run.setValue(true); 
                	} else {
                		run.setValue( ! run.getValue() ); 
                	}
                } else if ( e.getKeyCode() == KeyEvent.VK_S  && !e.isAltDown()) {
                	double dt = stepsize.getValue() / (int)substeps.getValue();
                    for ( int i = 0; i < substeps.getValue(); i++ ) {
                        if ( factory.use ) factory.advanceTime( dt );
                        system.advanceTime( dt );                
                    }
                    stepped = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_R ) {                    
                	if ( e.isShiftDown() ) {
                		if ( factory.use ) {
                			loadFactoryRequest = true;
                		} else {
                			loadSystemRequest = true;
                		}                    
                	} else {
                		systemReset();
                	}
                } else if ( e.getKeyCode() == KeyEvent.VK_H ) { // tab is for changing focus, so a pain to capture here
                	system.display.params.hideOverlay.setValue( ! system.display.params.hideOverlay.getValue() );
                } else if ( e.getKeyCode() == KeyEvent.VK_C ) { 
                    clearRequest = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_J ) {                   
                    system.jiggle();                                        
                } else if ( e.getKeyCode() == KeyEvent.VK_F ) {                                       
                    File f = FileSelect.select("xml", "xml scene for factory", "load", "scenes3D/", true );
                    if ( f != null ) {
                    	sceneFilename = f.getPath();
                        loadFactoryRequest = true;   
                    }   
                } else if ( e.getKeyCode() == KeyEvent.VK_PERIOD ) {                                       
                    factory.run.setValue ( ! factory.run.getValue() );
                } else if ( e.getKeyCode() == KeyEvent.VK_COMMA ) {
                    factory.createBodyRequest = true;
                } else if (  e.getKeyCode() == KeyEvent.VK_L ) {                    
                    File f = FileSelect.select( "xml", "scene", "load", "scenes3d/", true );
                    if ( f != null ) {
                    	sceneFilename = f.getPath();
                        loadSystemRequest = true;       
                    }
                } else if ( e.getKeyCode() == KeyEvent.VK_M ) {
                	system.merging.triggerMergingEvent = true;
                	system.merging.mergingEvent = false;
                    run.setValue( true );   
                } else if ( e.getKeyCode() == KeyEvent.VK_G) {                   
                        	system.generateBody= true;
                } else if ( e.getKeyCode() == KeyEvent.VK_O ) {
                	system.display.params.hideOverlay.setValue(!system.display.params.hideOverlay.getValue()); 
                } else if ( e.getKeyCode() == KeyEvent.VK_LEFT) {
                    if ( files != null && files.length >= 0 ) {
                        whichFile --;
                        if ( whichFile < 0 ) whichFile = files.length-1;

                    	sceneFilename = files[whichFile].getPath();
                        loadSystemRequest = true;                        
                    }
                } else if ( e.getKeyCode() == KeyEvent.VK_RIGHT ) {
                    if ( files != null && files.length >= 0 ) {
                        whichFile ++;
                        if ( whichFile >= files.length ) whichFile = 0;
                        
                    	sceneFilename = files[whichFile].getPath();
                        loadSystemRequest = true;       
                    }
                } else if ( e.getKeyCode() == KeyEvent.VK_ESCAPE ) {
                    eva.stop();
                } else if ( e.getKeyCode() == KeyEvent.VK_ENTER ) {
                    record.setValue( ! record.getValue() );
                } else if ( e.getKeyCode() == KeyEvent.VK_UP && !e.isAltDown() ) {
                    int ss = substeps.getValue();
                    if ( ss == substeps.getMaximum() ) return;
                    substeps.setValue( ss + 1 );
                    stepsize.setValue( stepsize.getValue() * (ss+1)/ss );
                } else if ( e.getKeyCode() == KeyEvent.VK_DOWN && !e.isAltDown() ) {
                    int ss = substeps.getValue();
                    if ( ss == substeps.getMinimum() ) return;
                    substeps.setValue( ss - 1 );
                    stepsize.setValue( stepsize.getValue() *(ss-1)/ss );
                }  else if ( e.getKeyCode() == KeyEvent.VK_Q  && !e.isAltDown()) {
                	float scale = 1;
                    for (RigidBody b: system.bodies) {
                    	if (b.canSpin) {
                    		b.tmpSpinner.set(b.spinner);
                    		b.tmpSpinner.scale(scale);
                    		b.omega.add(b.tmpSpinner);
                    	}
                    }
                } else if ( e.getKeyCode() == KeyEvent.VK_W && !e.isAltDown()) {
                	float scale = 1;
                    for (RigidBody b: system.bodies) {
                    	if (b.canSpin) {
                    		b.tmpSpinner.set(b.spinner);
                    		b.tmpSpinner.scale(-scale);
                    		b.omega.add(b.tmpSpinner);
                    	}
                    }
                }else if ( e.getKeyCode() == KeyEvent.VK_7 ) {
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
                } else if (moveControlSpringsTrigger(e)) {
                	//move all controllable springs forward z
                	Vector3d direction = new Vector3d();
                	double length = 0;
                	boolean moveEvent = true;
                	if (e.getKeyCode() == KeyEvent.VK_W) {
                		direction.set(0, 0, -1);
                	}else if (e.getKeyCode() == KeyEvent.VK_S) {
                		direction.set(0, 0, 1);
                	}else if (e.getKeyCode() == KeyEvent.VK_A) {
                		direction.set(-1, 0, 0);
                	}else if (e.getKeyCode() == KeyEvent.VK_D) {
                		direction.set(1, 0, 0);
                	}else if (e.getKeyCode() == KeyEvent.VK_Q) {
                		direction.set(0, 1, 0);
                	}else if (e.getKeyCode() == KeyEvent.VK_E) {
                		direction.set(0, -1, 0);
                	}
                	else if (e.getKeyCode() == KeyEvent.VK_UP) {
                		length = -1;
                	}
                	else if (e.getKeyCode() == KeyEvent.VK_DOWN) {
                		length = 1;
                	}else {
                		moveEvent = false;
                	}
                	if (moveEvent) {
                	}
                		for( Spring s : system.springs) {
                    		if (s.controllable) {
                    			s.moveTargetpW(direction, length);
                    		}
                	}
                	
                }
            }

			private boolean moveControlSpringsTrigger(KeyEvent e) {
				if (e.isAltDown()) {
					return true;
				}
				return false;
			}
        } );
    }
}
