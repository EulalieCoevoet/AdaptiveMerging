package mergingBodies3D;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import mintools.viewer.EasyViewer;

public class MemoryMonitor {

	int pos = 0;
	/** Number of frames to keep */
	final int N = 1000; 
	double[] data = new double[N];

	String title;
	String xLabel;
	double max = 0;
		
	MemoryMonitor( String title, String xLabel ) {
		this.title = title;
		this.xLabel = xLabel;
	}
	
	public void add( double v ) {
		data[pos] = v;
		pos = (pos+1)%N;
		max = Math.max( max,  v );
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
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, "" + ((int)maxY) );

		gl.glRasterPos2d(0.3,-0.1);
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, xLabel );
		gl.glRasterPos2d(0.3,1.0);
		EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, title );

        gl.glPointSize(3);
        gl.glLineWidth( 0.75f );
        gl.glColor4f( 0,0,0, 0.75f );

		gl.glScaled( 1.0/N, 1/maxY, 1 );
		gl.glBegin( GL.GL_LINE_STRIP );
		for ( int n = 0; n < N; n++ ) {
			gl.glVertex2d( n, data[(n+pos)%N]);			
		}
		gl.glEnd();
		
		
		gl.glPopMatrix();
		
		EasyViewer.endOverlay(drawable);
	}
	
}
