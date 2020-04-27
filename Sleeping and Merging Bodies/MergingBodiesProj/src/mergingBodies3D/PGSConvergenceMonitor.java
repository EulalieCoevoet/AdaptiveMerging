package mergingBodies3D;

import java.util.ArrayList;
import java.util.LinkedList;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import com.jogamp.opengl.util.gl2.GLUT;

import mintools.viewer.EasyViewer;

public class PGSConvergenceMonitor {

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

		double max = Double.MIN_VALUE;
		double min = Double.MAX_VALUE;

		String title;
		String xLabel;
		
		MonitorData( String title, String xLabel ) {
			this.title = title;
			this.xLabel = xLabel;
		}
		
		public void clear() {
			dataPoints.clear();
			max = Double.MIN_VALUE;
			min = Double.MAX_VALUE;
		}
		
		public void add( int n, double v ) {
			int index = n / binSize;
			if ( index >= dataPoints.size() ) {
				while ( index >= dataPoints.size() ) {
					dataPoints.add( new LinkedList<Pair>() );
				}
			}
			max = Math.max( max, v );
			min = Math.min( min, v );
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
			
			double maxY = (Math.floor(max*1e7)+1)/1e7;
			double minY = (Math.floor(min*1e7)+1)/1e7;
			gl.glRasterPos2d(-0.1,1);
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, ""+maxY );
			gl.glRasterPos2d(-0.1,0);
			EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, ""+minY );

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
	
	private MonitorData PGSM = new MonitorData("PGS convergence graph", "nb iterations");
	
	public void monitor( RigidBodySystem system ) {
	
		PGSM.clear();
		ArrayList<Double> changes = system.collision.solver.changes;
		int iterations = changes.size();
		for (int i=0; i<iterations; i++) {
			PGSM.add( i, changes.get(i) );
		}
	}
	
	public void draw( GLAutoDrawable drawable) {
		PGSM.draw(drawable, 0);
	}
}
