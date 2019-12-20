package mergingBodies3D;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.File;
import java.nio.IntBuffer;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Locale;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.border.TitledBorder;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.glu.GLU;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.FancyAxis;
import mintools.viewer.FlatMatrix4d;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;
import mintools.viewer.ShadowMap;

/**
 * Physically based character animation and PD control assignment
 * @author kry
 */
public class PDControlApp implements SceneGraphNode, Interactor {

    /**
     * Entry point for application
     * @param args
     */
    public static void main(String[] args) {
    	Locale.setDefault(Locale.ENGLISH);  // To ensure BVH load works for international students
        new PDControlApp();
    }
    
    private EasyViewer ev;
              
    private boolean stepRequest = false;    
    
    private boolean resetRequest = false;    
    
    private boolean throwSphereRequest = false;
    
    private boolean clearProjectilsRequest = false;
    
    private boolean setCurrentPoseRequest = false;
        
    /** space bar in the display window will toggle this. */
    private BooleanParameter runSimulation = new BooleanParameter( "simulate", false );
            
    Point mousePressedPoint = null;

    boolean selectRequest = false;
    
    /** [0,1] z position from opengl picking to use with unproject */
    float zClosest;
   
    /** Selection buffer for openGL picking */
    private IntBuffer selectionBuffer;
   
    private GLU glu = new GLU();
    
    /**
     * Creates a shadow map with a square image, e.g., 1024x1024.
     * This number can be reduced to improve performance.
     */
    private ShadowMap shadowMap = new ShadowMap( 2048 );

    /**
     * Creates the application / scene instance
     */
    public PDControlApp() {
        
        ev = new EasyViewer( "3d shadow example and picking", this, new Dimension(640,480), new Dimension(640,640) );
        ev.trackBall.setFocalDistance(5);
        ev.trackBall.focalPointY.setValue(1);  // might need a new mintools?
        ev.trackBall.near.setValue(2);
        ev.trackBall.far.setValue(15);
        
        ev.addInteractor(this);
        ev.controlFrame.add("Shadow", shadowMap.getControls() );        
    }
    
    @Override
    public void init(GLAutoDrawable drawable) {
        GL gl = drawable.getGL();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
        shadowMap.init(drawable);        
    }
    
    /**
     * Reset the animation
     */
    public void reset() {
        // do nothing in this example
    }

    MouseSpring mouseSpring = new MouseSpring();
    
    @Override
    public void display(GLAutoDrawable drawable) {
        if ( resetRequest ) {
            resetRequest = false;
            reset();
        }       
        if ( throwSphereRequest ) {
        	throwSphereRequest = false;
        }
        if ( clearProjectilsRequest ) {
        	clearProjectilsRequest = false;
        }
        
        
        if ( runSimulation.getValue() || stepRequest ) {
            stepRequest = false;
            // could do a sim step here
        }        

        GL2 gl = drawable.getGL().getGL2();

        if ( selectRequest ) {
        	selectRequest = false;
    		select(drawable, mousePressedPoint );
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
        
        // Spring for 
        if ( mouseSpring.body != null ) {
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
        
        drawNonShadowable( drawable );
        
        EasyViewer.beginOverlay( drawable );
        EasyViewer.printTextLines( drawable, "Message goes here" );
        
        int w = drawable.getSurfaceWidth();
        int h = drawable.getSurfaceHeight();
         
        EasyViewer.endOverlay(drawable);    
        
//        if ( play.getValue() || stepped ) {
//            stepped = false;        
//            if ( record.getValue() ) {
//                // write the frame
//                File file = new File( "stills/" + dumpName + format.format(nextFrameNum) + ".png" );                                             
//                nextFrameNum++;
//                file = new File(file.getAbsolutePath().trim());
//                ev.snapshot(drawable, file);
//            }
//        }
    }
    
    private void drawAllObjects( GLAutoDrawable drawable ) {
    	final FancyAxis fa = new FancyAxis();
    	GL2 gl = drawable.getGL().getGL2();
        
    	//odeSim.display(drawable);
        
        // draw the world reference frame
        gl.glPushMatrix();
        //gl.glTranslated( 0, odeSim.floorOffset, 0 );
        fa.setSize(0.25);
        fa.draw(gl);
        gl.glPopMatrix();
    }
    
    private void drawNonShadowable( GLAutoDrawable drawable ) {
        //odeSim.displayNonShadowable( drawable );        
    }

    /** TODO: to move this or use a list of bodies elsewhere... */
    ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
    
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
    	for ( RigidBody b : bodies ) {
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
                mouseSpring.body = bodies.get(pickIndex);
                mouseSpring.body.selected = true;
                final FlatMatrix4d ERBfromw = new FlatMatrix4d();
                //ODETools.setFlatMatrix(  odeSim.mouseSpring.body.body, ERBfromw );
                Matrix4d A = ERBfromw.getBackingMatrix();
                A.invert();
                //A.transform(mouseSpring.point,  mouseSpring.body.selectedPoint ) ;
		        //System.out.println(" unprojected on body = " +  odeSim.mouseSpring.body.selectedPoint );
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
    
    /** 
     * boolean to signal that the system was stepped and that a 
     * frame should be recorded if recording is enabled
     */
    private boolean stepped = false;
    
    private BooleanParameter record = new BooleanParameter( "record (press ENTER in canvas to toggle)", false );
    
    private String dumpName = "dump";
    
    private int nextFrameNum = 0;
    
    private NumberFormat format = new DecimalFormat("00000");
    
    /**
     * For enabling shadows and per pixel lighting
     */
    private BooleanParameter drawWithShadows = new BooleanParameter( "draw with shadows", true );

    @Override
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        
        JTextArea ta = new JTextArea(
        		"   space - start/stop simulation \n" +
        		"   s - step simulation \n" +
        		"   p - start/stop bvh animation \n" +
        		"   f - toggle follow bvh animation \n" +
        		"   e - set simulation to bvh pose \n" +
        		"   m - toggle marionette springs \n" +
        		"   c - clear all projectiles \n" +
        		"   z - throw projectile \n" +
        		"   left - rewind bvh one frame \n" +
        		"   right - advance bvh one frame \n" +
        		"   esc - quit" 
        		);                  
        ta.setEditable(false);
        VerticalFlowPanel vfpta = new VerticalFlowPanel();
        vfpta.add( ta );
        vfpta.setBorder( new TitledBorder("Keyboard controls") );
        CollapsiblePanel tacp = new CollapsiblePanel( vfpta.getPanel() );
        vfp.add( tacp );
        
        vfp.add( record.getControls() );
        
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
        
        vfp.add( runSimulation.getControls() );
        
        JButton fire = new JButton("throw projectile (z in window)");
        fire.addActionListener( new ActionListener() {
           @Override
            public void actionPerformed(ActionEvent e) {
        	   throwSphereRequest = true;
            } 
        });
        vfp.add(fire);
        
        
        vfp.add( drawWithShadows.getControls() );
        
        return vfp.getPanel();
    }
        
    @Override
    public void attach(Component component) {
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
				if ( mouseSpring.body != null ) {
					mouseSpring.body.selected = false;
					mouseSpring.body = null;
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
                    runSimulation.setValue( ! runSimulation.getValue() );
                } else if ( e.getKeyCode() == KeyEvent.VK_R ) {
                    resetRequest = true;
                    setCurrentPoseRequest = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_S ) {
                    stepRequest = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_E ) {
                    setCurrentPoseRequest = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_C ) {
                    clearProjectilsRequest = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_Z ) {
                    throwSphereRequest = true;
                } if ( e.getKeyCode() == KeyEvent.VK_ESCAPE ) {
                    ev.stop(); // quit
                }
            }
        } );
    }
}