package mergingBodies3D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.viewer.EasyViewer;

/**
 * Basicaly like the memory monitor and to a smaller extent the collsion computation monitor,
 * but this one will keep a vector of up to 20 quantities over the last 1000 time steps, and plot
 * them.
 * @author kry
 *
 */
public class QuantityMonitorGraph {

	/** current index for the circular buffer */
	int pos = 0;
	/** next component of data to set */
	int index = 0;
	
	/** Number of frames to keep */
	final int N = 1000; 
	final int M = 20;
	double[][] data = new double[N][M];
	int[] dataCount = new int[N];

	String title;
	String xLabel;
		
	QuantityMonitorGraph( String title, String xLabel ) {
		this.title = title;
		this.xLabel = xLabel;
	}
	
	public void setNameAndClear( String name ) {
		this.title = name;
		pos = 0; // doesn't really matter... but why not
		for ( int i = 0; i < N; i++ ) {
			dataCount[i] = 0;
		}
	}
	
	public void add( double v ) {
		if ( index == M ) return; // ignore
		data[pos][index++] = v;
		dataCount[pos] = index;
	}
	public void step() {
		pos = (pos+1)%N;
		index = 0;
	}
	
	
	public double getMax() {
		double max = 0;
		for ( int i = 0; i < N; i++ ) {
			for ( int j = 0; j < dataCount[i]; j++) {
				max = Math.max( max, data[i][j] );
			}
		}
		return max;
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
		
		double maxY = getMax();//(Math.floor(getMax()*1000)+1)/1000;
		gl.glRasterPos2d(-0.05,0.5);
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "t" );
		gl.glRasterPos2d(0,1);
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "" + maxY ); // ((int)maxY) );

		gl.glRasterPos2d(0.3,-0.1);
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, xLabel );
		gl.glRasterPos2d(0.3,1.0);
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, title );

        gl.glPointSize(2);
        gl.glLineWidth( 0.75f );
        gl.glColor4f( 0,0,0, 0.75f );

		gl.glScaled( 1.0/N, 1/maxY, 1 );
		gl.glBegin( GL.GL_POINTS );
		for ( int n = 0; n < N; n++ ) {
			int i = (n+pos)%N;
			for ( int k = 0; k < dataCount[i]; k++ ) {
				gl.glVertex2d( n, data[i][k]);
			}
		}
		gl.glEnd();
		
		
		gl.glPopMatrix();
		
		EasyViewer.endOverlay(drawable);
	}
	
}
