package mintools.viewer;

import java.awt.Dimension;

import javax.swing.JPanel;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.DebugGL2;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GL2ES2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLException;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.util.glsl.ShaderCode;
import com.jogamp.opengl.util.glsl.ShaderProgram;
import com.jogamp.opengl.util.glsl.ShaderState;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.Vec3Parameter;
import mintools.swing.VerticalFlowPanel;

/**
 * Creates a frame buffer object for a shadow map and permits simple scenes to be drawn with shadows.
 * Note that limited fixed functionality graphics pipeline will work with this!  Only one light
 * can be used.
 * 
 * typical usage:
 * 
 * shadowMap.beginLightPass( drawable, tbc );
 * drawAllObjects( drawable );        
 * shadowMap.endLightPass( drawable, tbc );
 * 
 * shadowMap.beginShadowMapping( drawable, tbc ); 
 * drawAllObjects( drawable );
 * shadowMap.endShadowMapping( drawable, tbc );
 *
 * While the trackball camera is not needed for all calls, it is included for symmetry 
 *
 * @author kry
 */
public class ShadowMap {

	/** Position of the light */
	private Vec3Parameter lightPos = new Vec3Parameter("light position", 5, 10, 5 );
    
    private BooleanParameter debugLightFrustum = new BooleanParameter( "debug light frustum" , false );
    private DoubleParameter near = new DoubleParameter( "near plane for light", 6.5, 0.5, 8 );    
    private DoubleParameter far = new DoubleParameter( "far plane for light" , 25, 8, 50 );    
    private DoubleParameter fov = new DoubleParameter( "light fov" , 30, 25, 90 );    
    private DoubleParameter sigma = new DoubleParameter( "self shadowing offset", 0.0015, 0, 0.1 );
	
    private ShaderState pflShaderState;

    private ShaderState depthShaderState;

	private int[] depthTexture = new int[1];

	private int[] depthFBO = new int[1];

	private final Dimension depthFBOSize;

	/**
	 * The light projection must be provided to the per fragment lighting program 
	 * so that we can look up the depth of the closest surface in the light depth map.  
	 */
	public FlatMatrix4f lightProjectionMatrix = new FlatMatrix4f();        
	
	/**
	 * Inverse of light viewing transformation
	 */
	FlatMatrix4f VLinv = new FlatMatrix4f(); 
	
	/** 
	 * Inverse of light viewing and projection 
	 */
	FlatMatrix4f VLinvPLinv = new FlatMatrix4f();      
	
	private GLU glu = new GLU();
    	
    /**
     * Creates a shadow map of the given size
     * @param size must be a power of 2
     */
    public ShadowMap( int size ) {        
        depthFBOSize = new Dimension(size,size);
    }
    
    //BooleanParameter useClipPlane = new BooleanParameter( "use clip plane", false );
    //Vec3Parameter np = new Vec3Parameter( "clipPlaneNormal", 0, 0, 1 );
    //DoubleParameter planez = new DoubleParameter( "plane goes throuhg this z in eye coords", 10, 0, 100 );
        
    /**
     * @return controls for the shadow mapped light
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.add( lightPos );        
        vfp.add( lightIntensity.getSliderControls(false) );
        vfp.add( ambientIntensity.getSliderControls(false) );
        vfp.add( sigma.getSliderControls(false) );
        vfp.add( near.getSliderControls(false) );
        vfp.add( far.getSliderControls(false) );
        vfp.add( fov.getSliderControls(false) );
        vfp.add( debugLightFrustum.getControls() ); 
        //vfp.add( useClipPlane.getControls() );
        //vfp.add( np );
        //vfp.add( planez.getSliderControls(false));
        return vfp.getPanel();
    }
    
    /** 
     * Creates a frame buffer object and sets it up as a depth texture for shadow mapping
     * @param drawable
     */
    public void init( GLAutoDrawable drawable ) {
        drawable.setGL( new DebugGL2( drawable.getGL().getGL2() ) );
		GL2 gl = drawable.getGL().getGL2();

		// Not needed for per fragment lighting program but enable for when PFL is not enabled
		gl.glEnable( GL2.GL_NORMALIZE );

		gl.glClearColor(0.0f, 0.0f, 0.0f, 0.5f);    // Black Background
		gl.glClearDepth(1.0f);                      // Depth Buffer Setup
		gl.glEnable(GL.GL_DEPTH_TEST);
		gl.glEnable(GL2.GL_LIGHTING);
		gl.glEnable( GL.GL_BLEND );
		gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
		gl.glEnable( GL.GL_LINE_SMOOTH );
		gl.glEnable( GL2.GL_POINT_SMOOTH );

		// no extra ambient light by default !
		gl.glLightModelfv( GL2.GL_LIGHT_MODEL_AMBIENT, new float[] {0,0,0,0}, 0);

		// Set some default material parameters
		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, new float[] {1,1,1,1}, 0 );
		gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_SPECULAR, new float[] {1,1,1,0}, 0 );
		gl.glMaterialf( GL.GL_FRONT_AND_BACK, GL2.GL_SHININESS, 50 );

		// CREATE THE FRAGMENT PROGRAM FOR DRAWING DEPTH
		depthShaderState = createProgram( drawable, "depthDraw" );

		// CREATE THE FRAGMENT PROGRAM FOR PER FRAGMENT LIGHTING
		pflShaderState = createProgram( drawable, "perFragmentLighting" );

		// SET UP RENDER TO TEXTURE FOR LIGHT DEPTH OFF SCREEN RENDERING
		gl.glGenTextures( 1, depthTexture, 0 );
		gl.glBindTexture( GL.GL_TEXTURE_2D, depthTexture[0] );
		// By clamping texture lookups to the border, we can force the use of an arbitrary depth value
		// on the edge and outside of our depth map. {1,1,1,1} is max depth, while {0,0,0,0} is min depth
		// Ultimately, you may alternatively want to deal with clamping issues in a fragment program.
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL2.GL_CLAMP_TO_BORDER);
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL2.GL_CLAMP_TO_BORDER);
		gl.glTexParameterfv(GL.GL_TEXTURE_2D, GL2.GL_TEXTURE_BORDER_COLOR, new float[] {1,1,1,1}, 0 );
		// The default filtering parameters not appropriate for depth maps, so we set them here! 
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST);  
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST);
		// You can also try GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT24 for the internal format.
		// Alternatively GL_DEPTH24_STENCIL8_EXT can be used (GL_EXT_packed_depth_stencil).
		// Here, null means reserve texture memory without initializing the contents.
		gl.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_DEPTH_COMPONENT32, depthFBOSize.width, depthFBOSize.height, 0, GL2.GL_DEPTH_COMPONENT, GL.GL_UNSIGNED_INT, null);
		gl.glGenFramebuffers( 1, depthFBO, 0);
		gl.glBindFramebuffer( GL.GL_FRAMEBUFFER, depthFBO[0] );
		gl.glFramebufferTexture2D( GL.GL_FRAMEBUFFER, GL.GL_DEPTH_ATTACHMENT, GL.GL_TEXTURE_2D, depthTexture[0], 0);
		gl.glDrawBuffer(GL.GL_NONE);
		gl.glReadBuffer(GL.GL_NONE);
		StringBuilder status = new StringBuilder();
        checkFramebufferStatus(gl, status );
        System.out.println( status );
        
		// Restore the original screen rendering frame buffer binding
		gl.glBindFramebuffer( GL.GL_FRAMEBUFFER, 0 );
    }       

    /**
	 * Creates a GLSL program from the .vp and .fp code provided in the shader directory 
	 * @param drawable
	 * @param name
	 * @return
	 */
	private ShaderState createProgram( GLAutoDrawable drawable, String name ) {
		GL2 gl = drawable.getGL().getGL2();
		
		ShaderCode vsCode = ShaderCode.create( gl, GL2ES2.GL_VERTEX_SHADER, this.getClass(), "shaderSol", "shader/bin", name, false );
		ShaderCode fsCode = ShaderCode.create( gl, GL2ES2.GL_FRAGMENT_SHADER, this.getClass(), "shaderSol", "shader/bin", name, false );
		ShaderProgram shaderProgram = new ShaderProgram();
		shaderProgram.add( vsCode );
		shaderProgram.add( fsCode );
		if (!shaderProgram.link(gl, System.err)) {
			throw new GLException("Couldn't link program: " + shaderProgram);
		}	
		ShaderState shaderState = new ShaderState();
		shaderState.attachShaderProgram( gl, shaderProgram, false );	
		return shaderState;
	}

	DoubleParameter lightIntensity = new DoubleParameter( "light intensity", 0.75, 0, 1 );
	DoubleParameter ambientIntensity = new DoubleParameter( "ambient intensity", 0.3, 0, 1 );
    
	public void setupLightsInWorld( GLAutoDrawable drawable ) {
		GL2 gl = drawable.getGL().getGL2();
		float[] position = { lightPos.x, lightPos.y, lightPos.z, 1 };
		float I = lightIntensity.getFloatValue();
		float A = ambientIntensity.getFloatValue();
		float[] colour = { I, I, I, 1 };
		float[] acolour = { A, A, A, 0 };
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_SPECULAR, colour, 0 );
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_DIFFUSE, colour, 0 );
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_AMBIENT, acolour, 0 );
		gl.glLightfv( GL2.GL_LIGHT0, GL2.GL_POSITION, position, 0 );
		gl.glEnable( GL2.GL_LIGHT0 );
	}
	
    /**
     * Prepares for drawing the light view
     * @param drawable
     */
    public void beginLightPass( GLAutoDrawable drawable, Camera tbc ) {        
        GL2 gl = drawable.getGL().getGL2(); 
        
		//////////////////////////////////////////////////////////////////
		// Render to our off-screen depth frame buffer object (render to texture)
		gl.glBindFramebuffer( GL.GL_FRAMEBUFFER, depthFBO[0] );
		gl.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT );
		gl.glViewport( 0, 0, depthFBOSize.width, depthFBOSize.height ); 
		
		Point3d lPos = new Point3d( lightPos.x, lightPos.y, lightPos.z );
		Point3d at = new Point3d( 0,0,0 );
		//double distance = lPos.distance( at );
		// Could use the distance as near and far... e.g., distance - 2, distance + 2
		
		gl.glMatrixMode( GL2.GL_PROJECTION );		
		gl.glLoadIdentity();        
		glu.gluLookAt( lPos.x, lPos.y, lPos.z, at.x, at.y, at.z, 1, 0, 0 );   		
		gl.glGetFloatv( GL2.GL_PROJECTION_MATRIX, VLinv.asArray(), 0 );        
		VLinv.reconstitute();
		VLinv.getBackingMatrix().invert();
		
		gl.glLoadIdentity();		
		glu.gluPerspective(fov.getFloatValue(), 1, near.getFloatValue(), far.getFloatValue() );
		glu.gluLookAt( lPos.x, lPos.y, lPos.z, at.x, at.y, at.z, 1, 0, 0 );		
		gl.glGetFloatv( GL2.GL_PROJECTION_MATRIX, VLinvPLinv.asArray(), 0 );        
		VLinvPLinv.reconstitute();
		VLinvPLinv.getBackingMatrix().invert();
		
		gl.glMatrixMode( GL2.GL_MODELVIEW );
		gl.glLoadIdentity();
    }
    
    /**
     * Finishes the light view drawing and prepares for normal camera view
     * @param drawable
     */
    public void endLightPass( GLAutoDrawable drawable, Camera tbc ) {
        GL2 gl = drawable.getGL().getGL2();

        // grab a copy of the light projection matrix... could have been done elsewhere
		gl.glMatrixMode( GL2.GL_PROJECTION );	
		tbc.applyInverseViewTransformation(drawable);
		gl.glGetFloatv( GL2.GL_PROJECTION_MATRIX, lightProjectionMatrix.asArray(), 0 );        
		lightProjectionMatrix.reconstitute();		
		gl.glMatrixMode( GL2.GL_MODELVIEW );	
    }
    
    /**
     * Prepares for drawing with the shadow map depth test
     * @param drawable
     */
    public void beginShadowMapping(GLAutoDrawable drawable, Camera tbc ) {
        GL2 gl = drawable.getGL().getGL2();
        
		//////////////////////////////////////////////////////////////////
		// Render to the screen
		gl.glBindFramebuffer( GL.GL_FRAMEBUFFER, 0 );
		gl.glClear( GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT );
		gl.glViewport( 0, 0, drawable.getSurfaceWidth(), drawable.getSurfaceHeight() ); 
		
		tbc.prepareForDisplay(drawable);
		setupLightsInWorld(drawable);
		
		pflShaderState.useProgram(gl, true);
		int shadowMapID = pflShaderState.getUniformLocation( gl, "shadowMap");
		int clipPlaneID = pflShaderState.getUniformLocation( gl, "u_clipPlane");
		int lightProjectionID = pflShaderState.getUniformLocation( gl, "lightProjection");
		int sigmaID = pflShaderState.getUniformLocation( gl, "sigma");			
		gl.glUniform1i( shadowMapID, 0 ); // use texture unit zero!
		
		/*Vector3d n = new Vector3d( np.x, np.y, np.z );
		n.normalize();
		float[] clipPlane = new float[] { 0, 0, 0, 1 }; 
		if ( useClipPlane.getValue()) {
				clipPlane = new float[] { (float)n.x, (float)n.y, (float)n.z, (float)(-n.z*planez.getValue()) };
		}
		gl.glUniform4fv( clipPlaneID, 1, clipPlane, 0 );*/
		
		// Don't screw up the numbers here!  Texture unit ID
		gl.glUniformMatrix4fv( lightProjectionID, 1, false, lightProjectionMatrix.asArray(), 0 );
		gl.glUniform1f( sigmaID, sigma.getFloatValue() ); // use texture unit zero!
    }

    /**
     * Finishes drawing with the shadow map depth test
     * @param drawable
     */
    public void endShadowMapping( GLAutoDrawable drawable, Camera tbc ) {
        GL2 gl = drawable.getGL().getGL2();
		pflShaderState.useProgram(gl, false);
		displayDebug(drawable);
    }
    
    /**
     * Draws the light frustum and light position and the texture 
     * @param drawable
     */
    public void debugLightFrustum( GLAutoDrawable drawable ) {
    	GL2 gl = drawable.getGL().getGL2();

    	final FancyAxis fa = new FancyAxis();
    	
    	gl.glPushMatrix();    	
		// Set up the appropriate transformations for drawing the light's camera frame			
		gl.glMultMatrixf( VLinv.asArray(), 0 );	
		gl.glLightModelfv( GL2.GL_LIGHT_MODEL_AMBIENT, new float[] {.5f,.5f,.5f,1}, 0);
		fa.draw(gl);
		gl.glLightModelfv( GL2.GL_LIGHT_MODEL_AMBIENT, new float[] {0,0,0,1}, 0);		
		gl.glPopMatrix();
		
		gl.glPushMatrix();
		// Set up the appropriate matrices to draw the light frustum and the depth map texture on its near plane
		gl.glMultMatrixf(VLinvPLinv.asArray(),0);
		gl.glDisable( GL2.GL_LIGHTING );
		gl.glColor4f( 1, 1, 1, 0.5f );
		gl.glLineWidth( 3 );       
		EasyViewer.glut.glutWireCube( 2 );
		gl.glEnable( GL2.GL_LIGHTING );
		// draw the depth map on the near plane of the light view frustum
		depthShaderState.useProgram(gl, true);
		gl.glBindTexture( GL.GL_TEXTURE_2D, depthTexture[0] );
		int textureUnitID = depthShaderState.getUniformLocation( gl, "depthTexture" );
		int alphaID = depthShaderState.getUniformLocation( gl, "alpha" );  // unused... can remove
		// Note that we need to specify the texture unit ID, as opposed to the texture ID.
		// the texture ID is bound to the current texture unit with the bind call above, while
		// other calls would be necessary to use additional texture units.  
		gl.glUniform1i( textureUnitID, 0 ); 
		gl.glUniform1f( alphaID,  0.5f ); 		
		gl.glDisable( GL2.GL_LIGHTING );
		gl.glEnable( GL.GL_TEXTURE_2D );
		// Draw a quad with texture coordinates that span the entire texture.  
		// Note that we put the quad on the near plane by providing z = -1
		gl.glBegin( GL2.GL_QUADS );
		gl.glTexCoord2d( 0, 0 ); gl.glVertex3f(-1, -1, -1 );
		gl.glTexCoord2d( 1, 0 ); gl.glVertex3f( 1, -1, -1 );
		gl.glTexCoord2d( 1, 1 ); gl.glVertex3f( 1,  1, -1 );
		gl.glTexCoord2d( 0, 1 ); gl.glVertex3f(-1,  1, -1 );
		gl.glEnd();
		gl.glDisable( GL.GL_TEXTURE_2D );
		gl.glEnable( GL2.GL_LIGHTING );
		depthShaderState.useProgram( gl, false );
		gl.glPopMatrix();
    }
    
    /** 
     * Displays debug information for the shadow mapping, if appropriate check-boxes are selected.
     * @param drawable
     */
    public void displayDebug( GLAutoDrawable drawable ) {
        GL gl = drawable.getGL();
        gl.glEnable(GL.GL_BLEND);
        gl.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);
        gl.glLineWidth( 2 );
        // Check to see if the light frustum makes sense        
        if ( debugLightFrustum.getValue() ) {
            debugLightFrustum( drawable );
        }                
    }
    
    private int checkFramebufferStatus(GL gl, StringBuilder statusString) {
        int framebufferStatus = gl.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER);
        switch (framebufferStatus) {
            case GL.GL_FRAMEBUFFER_COMPLETE:
                statusString.append("GL_FRAMEBUFFER_COMPLETE");
                break;
            case GL.GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                statusString.append("GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENTS");
                break;
            case GL.GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                statusString.append("GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
                break;
            case GL.GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
                statusString.append("GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS");
                break;
            case GL.GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
                statusString.append("GL_FRAMEBUFFER_INCOMPLETE_FORMATS");
                break;
            case GL2.GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
                statusString.append("GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER");
                break;
            case GL2.GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
                statusString.append("GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER");
                break;
            case GL.GL_FRAMEBUFFER_UNSUPPORTED:
                statusString.append("GL_FRAMEBUFFER_UNSUPPORTED");
                break;
        }
        return framebufferStatus;
    }
    
}
