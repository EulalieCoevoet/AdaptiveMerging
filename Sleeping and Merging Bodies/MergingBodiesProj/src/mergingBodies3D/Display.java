package mergingBodies3D;

import java.util.ArrayList;

import javax.swing.JPanel;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;

public class Display {
	
	public class DisplayParameters {

	    public BooleanParameter hideOverlay = new BooleanParameter( "hide overlay", false );
	    public BooleanParameter drawGraphs = new BooleanParameter( "draw performance graphs", false );
	    public BooleanParameter drawMemGraphs = new BooleanParameter( "draw memory graphs", false );
	    
		private DoubleParameter transparency = new DoubleParameter("body block transparency", .5, 0, 1 );
		private BooleanParameter drawBodies = new BooleanParameter( "draw bodies", true );
		private BooleanParameter drawCollections = new BooleanParameter( "draw collections with different colors", true );
		
		private BooleanParameter drawBoundingVolumes = new BooleanParameter( "draw root bounding volumes", false );
		private BooleanParameter drawBoundingVolumesUsed = new BooleanParameter( "draw bounding volumes used", false );
		private BooleanParameter drawAllBoundingVolumes = new BooleanParameter( "draw ALL bounding volumes", false );
		
		private BooleanParameter drawContactForces = new BooleanParameter("draw contact forces", false );
		private BooleanParameter drawContactForcesInCollection = new BooleanParameter("draw contact forces in collections", false );
		private BooleanParameter drawContactLocations = new BooleanParameter( "draw contact locations", false );
		private IntParameter contactLocationSize = new IntParameter( "contact point size ", 5, 5, 20);
		private BooleanParameter drawContactGraph = new BooleanParameter( "draw contact graph", false );
		private BooleanParameter drawCollectionContactGraph = new BooleanParameter( "draw collections' contact graph", false );
		private BooleanParameter drawCycles = new BooleanParameter( "draw cycles", true );
		
		private BooleanParameter drawCOMs = new BooleanParameter( "draw COM", false );
		private BooleanParameter drawSpeedCOMs = new BooleanParameter( "draw speed COM", false );
		private BooleanParameter drawBB = new BooleanParameter( "draw bounding box", false );
		public BooleanParameter drawIndex = new BooleanParameter( "dawIndex", false );
	}
	public DisplayParameters params = new DisplayParameters();
	
	ArrayList<RigidBody> bodies;
	CollisionProcessor collision; // TODO: rename 
	
	Display(ArrayList<RigidBody> bodies, CollisionProcessor collisionProcessor) {
		this.bodies = bodies;
		this.collision = collisionProcessor;
	}
	
    /** Might want to allow for different coloured blocks?? but for now, in 3D this is easiest */
    private float[] green = new float[] { 0, 1, 0, 0.25f };
    private float[] colourPinned = new float[] { 0.75f,0.75f,1, 1 };		        			
	private float[] colour = new float[] { 0.9f,0.9f,0.9f, 1 };        			
    private float[] red = new float[] { 1, 0, 0, 0.5f };
    private float[] blue = new float[] { 0, 0, 1, 0.25f };
    
    public void displayNonShadowable( GLAutoDrawable drawable, double dt ) {
    	GL2 gl = drawable.getGL().getGL2();
        gl.glDisable(GL2.GL_DEPTH_TEST);

    	// Should move this stuff below to a display non-shadowable function
        gl.glMaterialfv(GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, green, 0);
        if ( params.drawContactGraph.getValue() ) {
            for ( Contact c : collision.contacts ) {
                c.displayConnection(drawable);
            }
        }
        
    	if (params.drawContactLocations.getValue() || params.drawContactForces.getValue()) {
			for ( Contact c : collision.contacts ) {
				if (params.drawContactLocations.getValue()) {
					c.display(drawable, false); 
				}
				if (params.drawContactForces.getValue())
					c.displayContactForce(drawable, false, dt );
			}
		}
		
		if ( params.drawContactLocations.getValue() || params.drawContactForcesInCollection.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					RigidCollection collection = (RigidCollection)b;
					if (params.drawContactLocations.getValue())
						collection.displayInternalContactLocations(drawable, params.contactLocationSize.getValue());
					if (params.drawContactForcesInCollection.getValue())
						collection.displayInternalContactForces( drawable, dt );
				}
			}
		}
        
        
        
        if ( params.drawBB.getValue() ) {
        	 for ( RigidBody b : bodies ) {
               b.displayBB(drawable);
           }
        }
        gl.glEnable(GL2.GL_DEPTH_TEST);
    }
	
	/**
	 * Draws all rigid bodies
	 * @param drawable
	 */
	public void display( GLAutoDrawable drawable, boolean picking ) {
		
		GL2 gl = drawable.getGL().getGL2();
		
		if ( params.drawCOMs.getValue() && ! picking ) {
        	for ( RigidBody b : bodies ) {
        		b.displayFrame(drawable);
        	}
        }
        
        // TODO: perhaps do clipping planes here instead to only clip bodies?
        // TODO: perhaps also have cull face options?  think we want to cull to better see contacts after clipping
        if ( params.drawBodies.getValue() ) {
        	int i = 0;
        	for ( RigidBody b : bodies ) {
        		if ( picking ) {
        			LCPApp3D.setColorWithID( gl, i++ );
        		} else {
        			// let's control the colour of geometry here as it will let us 
        			// decide when we want to override this colour (e.g., if we have a 
        			// rigid body collection)
        			float[] c = colour;
        			if ( b.pinned ) {
        				c = colourPinned;
        			} else if ( b.col != null ) {
        				c = b.col;
        			}
					if( b instanceof RigidCollection && ! params.drawCollections.getValue() ) {
						// TODO: make sure the bodies draw with their proper colours!
					}
    				c[3] = params.transparency.getFloatValue();         			
        			gl.glMaterialfv( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, c, 0 );
        		}
                b.display( drawable );
        	}
        }
       
        
        // TODO: end clipping here to continue to see other debug visualizations un-clipped..
        // again, perhaps an option to end clipping here or at the end of all of this!
        
        if ( ! picking ) {
	        gl.glMaterialfv(GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, red, 0);
	        gl.glNormal3f(0,0,1);
	    	for ( RigidBody b : bodies ) {
	    		for (Spring s : b.springs) {
					s.displaySpring(drawable);
				}
	    	}        
	        gl.glLineWidth(1);
	        gl.glMaterialfv(GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE, blue, 0);
	        if ( params.drawBoundingVolumes.getValue() ) {
	            for ( RigidBody b : bodies ) {
	            	if ( b.root == null ) continue; // rigid body planes don't have a BVH
	            	if (!(b instanceof RigidCollection)) {
	            		b.root.boundingSphere.display(drawable);
	            	}
	            }
	        }
	        if ( params.drawAllBoundingVolumes.getValue() ) {
	            for ( RigidBody b : bodies ) {
	            	if ( b.root == null ) continue; // rigid body planes don't have a BVH
	            	if (!(b instanceof RigidCollection)) {
	            		b.root.display( drawable );
		            } else {
	            		displayCollectionBV((RigidCollection) b, drawable);
	            	}
	            }
	        }        
	        if ( params.drawBoundingVolumesUsed.getValue() ) {
	            for ( RigidBody b : bodies ) {
	            	if ( b.root == null ) continue; // rigid body planes don't have a BVH
	            	if (!(b instanceof RigidCollection)) {
	            		b.root.displayVisitBoundary( drawable, collision.visitID );
	            	} else {
	            		displayVisitBoundaryCollection((RigidCollection) b, drawable);
	            	}
	            }
	        }	        
        }
		
		// TODO: finish updating this display stuff...
		
//		if ( params.drawContactGraph.getValue() ) {
//			if (collisionProcessor.useContactGraph.getValue()) {
//				for (BodyPairContact bpc : collisionProcessor.bodyPairContacts) {
//					for (Contact c : bpc.contactList) {
//						c.displayContactGraph(drawable);
//					}
//				} 
//			} 
//			else {
//				for ( Contact c : collisionProcessor.contacts ) {
//					c.displayContactGraph(drawable);
//				}
//			}
//		}
//		
//		if (params.drawCollectionContactGraph.getValue()) {
//			for (RigidBody b : bodies) {
//				if (b instanceof RigidCollection) {
//					((RigidCollection)b).displayContactGraph(drawable);
//				}
//			}
//		}
//		
//		if ( params.drawSpeedCOMs.getValue() ) {
//			for (RigidBody b: bodies) {
//				b.displaySpeedCOM(drawable);
//			}
//		}
//
//	
//		
//		if ( params.drawCycles.getValue() ) {
//			for (RigidBody b : bodies) {
//				if (b instanceof RigidCollection) {
//					((RigidCollection)b).displayCycles(drawable, params.contactLocationSize.getValue());
//				}
//			}
//		}
		
		
		if ( params.drawIndex.getValue()) {
			for (RigidBody b : bodies) {
				displayIndex(b, bodies.indexOf(b), drawable, GLUT.BITMAP_8_BY_13);
			}
		}
	}

	private void displayVisitBoundaryCollection(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.bodies) {
			body.root.displayVisitBoundary(drawable, collision.visitID);
		}
	}

	private void displayCollectionBV(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.bodies) {
			body.root.display(drawable);
		}
	}
	
	public void displayIndex(RigidBody body, int index, GLAutoDrawable drawable, int font) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glColor3f(1, 0, 0);
		gl.glRasterPos2d(body.x.x, body.x.y);

		EasyViewer.glut.glutBitmapString(font, Integer.toString(index));
	}

	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {

		VerticalFlowPanel vfp = new VerticalFlowPanel();
		
        vfp.add( params.hideOverlay.getControls() );
        vfp.add( params.drawGraphs.getControls() );
        vfp.add( params.drawMemGraphs.getControls() );
        
		vfp.add( params.transparency.getSliderControls(false));
		vfp.add( params.drawBodies.getControls() );
		vfp.add( params.drawCollections.getControls() );
		
		vfp.add( params.drawBoundingVolumes.getControls() );
		vfp.add( params.drawBoundingVolumesUsed.getControls() );
		vfp.add( params.drawAllBoundingVolumes.getControls() );
		
		vfp.add( params.drawContactForces.getControls() );
		vfp.add( params.drawContactForcesInCollection.getControls() );
		vfp.add( params.drawContactLocations.getControls() );
		vfp.add( params.contactLocationSize.getSliderControls());
		vfp.add( params.drawContactGraph.getControls() );
		vfp.add( params.drawCollectionContactGraph.getControls() );
		vfp.add( params.drawCycles.getControls() );

		vfp.add( params.drawCOMs.getControls() );
		vfp.add( params.drawSpeedCOMs.getControls() );
		vfp.add( params.drawBB.getControls() );
		vfp.add( params.drawIndex.getControls() );
		vfp.add( Contact.forceVizScale.getSliderControls(true) ); // Gross?

		return vfp.getPanel();
	}
}
