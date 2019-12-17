package mergingBodies;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.vecmath.Color3f;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;

public class Display {
	
	public class DisplayParameters {
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
	CollisionProcessor collisionProcessor;
	
	Display(ArrayList<RigidBody> bodies, CollisionProcessor collisionProcessor) {
		this.bodies = bodies;
		this.collisionProcessor = collisionProcessor;
	}
	
	/**
	 * Draws all rigid bodies
	 * @param drawable
	 */
	public void display( GLAutoDrawable drawable ) {
		
		GL2 gl = drawable.getGL().getGL2();
		if ( Block.alpha != (float) (double) params.transparency.getValue()) {
			Block.alpha = (float) (double) params.transparency.getValue();
			// gross... need to rebuild display lists for the currently set transparency
			// which is potentially slow and bad news (memory thrashing) for openGL if we do this excessively!
			RigidBody.clearDisplayLists( gl );
			for ( RigidBody b : bodies ) {
				b.myListID = -1;

				if (b instanceof RigidCollection)
					for (RigidBody sb : ((RigidCollection)b).bodies)
						sb.myListID = -1;
			}
		}  
		
		if ( params.drawBodies.getValue() ) {
			
			for ( RigidBody b : bodies ) {
				
				if ( b instanceof RigidCollection) {
					RigidCollection collection = (RigidCollection)b;
					if(params.drawCollections.getValue()) {
						collection.displayCollection(drawable);
					} else {
						for (RigidBody sb : collection.bodies)
							sb.display(drawable, null);
					}
				} else {
					b.display(drawable, null);
				}
				
				for (Spring s : b.springs) {
					s.displaySpring(drawable);
				}
			}
		}

		gl.glLineWidth(1);
		if ( params.drawBoundingVolumes.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection)) {
					b.root.boundingDisc.display(drawable);
				}
			}
		}
		
		if ( params.drawAllBoundingVolumes.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection)) {
					b.root.display( drawable );
				} else {
					displayCollectionBV((RigidCollection) b, drawable);
				}
			}
		}      
		
		if ( params.drawBoundingVolumesUsed.getValue() ) {
			for ( RigidBody b : bodies ) {
				if (!(b instanceof RigidCollection)) {
					b.root.displayVisitBoundary( drawable, collisionProcessor.visitID );
				} else {
					displayVisitBoundaryCollection((RigidCollection) b, drawable);
				}
			}
		}
		
		if ( params.drawContactGraph.getValue() ) {
			if (CollisionProcessor.useContactGraph.getValue()) {
				for (BodyPairContact bpc : collisionProcessor.bodyPairContacts) {
					for (Contact c : bpc.contactList) {
						c.displayContactGraph(drawable);
					}
				} 
			} 
			else {
				for ( Contact c : collisionProcessor.contacts ) {
					c.displayContactGraph(drawable);
				}
			}
		}
		
		if (params.drawCollectionContactGraph.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					((RigidCollection)b).displayContactGraph(drawable);
				}
			}
		}
		
		if ( params.drawSpeedCOMs.getValue() ) {
			for (RigidBody b: bodies) {
				b.displaySpeedCOM(drawable);
			}
		}

		if (params.drawContactLocations.getValue() || params.drawContactForces.getValue()) {
			for ( Contact c : collisionProcessor.contacts ) {
				if (params.drawContactLocations.getValue()) {
					c.displayContactLocation(drawable, new Color3f(1,0,0), params.contactLocationSize.getValue()); 
				}
				if (params.drawContactForces.getValue())
					c.displayContactForce(drawable, new Color3f(1,0,0));
			}
		}
		
		if ( params.drawContactLocations.getValue() || params.drawContactForcesInCollection.getValue()) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					RigidCollection collection = (RigidCollection)b;
					if (params.drawContactLocations.getValue())
						collection.displayInternalContactLocations(drawable, params.contactLocationSize.getValue());
					if (params.drawContactForcesInCollection.getValue())
						collection.displayInternalContactForces(drawable);
				}
			}
		}
		
		if ( params.drawCycles.getValue() ) {
			for (RigidBody b : bodies) {
				if (b instanceof RigidCollection) {
					((RigidCollection)b).displayCycles(drawable, params.contactLocationSize.getValue());
				}
			}
		}
		
		if ( params.drawCOMs.getValue() ) {
			for ( RigidBody b : bodies ) {
				b.displayCOMs(drawable);
			}
		}
		
		if ( params.drawBB.getValue() ) {
			for ( RigidBody b : bodies ) {
				b.displayBB(drawable);
			}
		}
		
		if ( params.drawIndex.getValue()) {
			for (RigidBody b : bodies) {
				b.displayIndex(drawable, GLUT.BITMAP_8_BY_13);
			}
		}
	}

	private void displayVisitBoundaryCollection(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.bodies) {
			body.root.displayVisitBoundary(drawable, collisionProcessor.visitID);
		}
	}

	private void displayCollectionBV(RigidCollection b, GLAutoDrawable drawable) {
		for (RigidBody body: b.bodies) {
			body.root.display(drawable);
		}
	}

	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {

		VerticalFlowPanel vfp = new VerticalFlowPanel();
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
