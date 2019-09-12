package mergingBodies;

import java.util.ArrayList;
import java.util.LinkedList;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import com.jogamp.opengl.util.gl2.GLUT;

import mintools.viewer.EasyViewer;

public class CollisionComputationMonitor {

	private static class Pair {
		int n;
		double v;
		Pair(int n, double v) {
			this.n =n; 
			this.v = v;
		}
	}
	
	private class MonitorData {
		
		final int binSize = 30;
		final int binCount = 30;

		ArrayList<LinkedList<Pair>> dataPoints = new ArrayList<LinkedList<Pair>>();

		double max = 0;

		String title;
		String xLabel;
		
		MonitorData( String title, String xLabel ) {
			this.title = title;
			this.xLabel = xLabel;
		}
		
		public void add( int n, double v ) {
			int index = n / binSize;
			if ( index >= dataPoints.size() ) {
				while ( index >= dataPoints.size() ) {
					dataPoints.add( new LinkedList<Pair>() );
				}
			}
			max = Math.max( max, v );
			LinkedList<Pair> L = dataPoints.get(index);
			if ( L.size() > binCount ) {
				Pair p = L.remove(0);
				p.n = n;
				p.v = v;
				L.add( p );
			} else {
				L.add( new Pair(n,v) );
			}
		}
		
		public void draw( GLAutoDrawable drawable, int where ) {
			int w = drawable.getSurfaceWidth();
			int h = drawable.getSurfaceHeight();
			EasyViewer.beginOverlay(drawable);

			GL2 gl = drawable.getGL().getGL2();
			gl.glPushMatrix();
			
			gl.glScaled( w, h, 1 );
			if ( (where%2) == 0 ) {
				gl.glTranslated( 0.25, 0, 0);
			} else {
				gl.glTranslated( -0.25, 0, 0);
			}
			gl.glTranslated( 0.5, 0.5, 0 );
			gl.glScaled( 0.4, -0.4, 1 );
			gl.glTranslated( -0.5, -0.5, 0 );
			
			
			gl.glColor3f( 0,0,0 );
			gl.glBegin( GL.GL_LINE_STRIP );
			gl.glVertex2d( 1,0 );
			gl.glVertex2d( 0,0 );
			gl.glVertex2d( 0,1 );
			gl.glEnd();
			
			double maxY = (Math.floor(max*1000)+1)/1000;
			gl.glRasterPos2d(-0.05,0.5);
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "t" );
			gl.glRasterPos2d(0,1);
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, ""+maxY );

			double maxX = dataPoints.size()*binSize;
			gl.glRasterPos2d(0.3,-0.1);
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, xLabel );
			gl.glRasterPos2d(0.8,-0.1);
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, ""+maxX );
			gl.glRasterPos2d(0.3,1.0);
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, title );
	
	        gl.glPointSize(3);

			gl.glScaled( 1/maxX, 1/maxY, 1 );
			gl.glBegin( GL.GL_POINTS );
			for ( LinkedList<Pair> L : dataPoints ) {
				for ( Pair p : L ) {
					gl.glVertex2d(p.n, p.v);
				}
			}
			gl.glEnd();
			
			
			gl.glPopMatrix();
			
			EasyViewer.endOverlay(drawable);
		}
		
	}
	
	private MonitorData CD = new MonitorData("collision detection", "bodies");
	private MonitorData CP = new MonitorData("collision processing", "contacts");
	private MonitorData CU = new MonitorData("collision update in collections", "tests");
	
	public void monitor( RigidBodySystem system ) {
		int nb = system.bodies.size(); 
		int nc = system.collisionProcessor.contacts.size();
		double tcs = system.collisionProcessor.collisionSolveTime;
		double tcd = system.collisionProcessor.collisionDetectTime;
		double tcu = system.collisionProcessor.collectionUpdateTime;
	
		CD.add( nb, tcd );
		CP.add( nc, tcs );
		CU.add( nc, tcs );
	}
	
	public void draw( GLAutoDrawable drawable) {
		CD.draw(drawable, 0);
		CP.draw(drawable, 1);
		CU.draw(drawable, 1);
	}
}
