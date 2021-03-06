package mintools.examples;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.GraphicsEnvironment;
import java.awt.Shape;
import java.awt.font.FontRenderContext;
import java.awt.font.GlyphVector;
import java.awt.geom.AffineTransform;
import java.awt.geom.PathIterator;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.SceneGraphNode;

/**
 * Demonstration of how to get stroke information from fonts.
 * @author kry
 */
public class Example2DFontStrokesApp implements SceneGraphNode {

    IntParameter which = new IntParameter( "which font", 3, 0, 73);
    IntParameter size = new IntParameter( "size", 500, 10, 1000 );
    IntParameter curveResolution = new IntParameter( "curve resolution", 10, 1, 20);
    BooleanParameter bold = new BooleanParameter( "bold", false );
    BooleanParameter italic = new BooleanParameter( "italic", false );
    JTextField text = new JTextField("B");
    JLabel fname = new JLabel();
    
    String[] fontNames;
    
    /**
     * Initialize the font list
     */
    public Example2DFontStrokesApp() {
        // Get the local graphics environment
        GraphicsEnvironment env = GraphicsEnvironment.getLocalGraphicsEnvironment();      
        //Get the font names from the graphics environment
        fontNames = env.getAvailableFontFamilyNames();        
        which.setMaximum(fontNames.length);
        fname.setText(fontNames[(int)which.getValue()]);
    }
    
    @Override
    public void init(GLAutoDrawable drawable) {
        // This will set up nice anti aliased lines and points
        GL gl = drawable.getGL();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );        
    }
    
    @Override
    public void display(GLAutoDrawable drawable) {
        
        GL2 gl = drawable.getGL().getGL2();
        
        // Probably better to completely avoid the 3D setup if we 
        // only want to draw in 2D, but this is a quick and easy way 
        // to draw in 2D
        EasyViewer.beginOverlay(drawable);
        
        gl.glColor4d( 0,1,0,alpha.getValue() );
        gl.glLineWidth(2);        
        
        // Note this is horribly inefficiant...
        // we'll get the stroke information for the given text and the given 
        // font on every display call
        int style = 0;
        style |= bold.getValue() ? Font.BOLD : 0;
        style |= italic.getValue() ? Font.ITALIC : 0;                       
        fname.setText(fontNames[(int)which.getValue()]);        
        Font f = new Font( fname.getText(), style, (int)size.getValue() );        
        FontRenderContext defaultFRC = new FontRenderContext(null, false, false);
        GlyphVector gv = f.createGlyphVector(defaultFRC, text.getText() );        
        Shape shape = gv.getOutline();
        AffineTransform at = new AffineTransform();
        at.setToIdentity();
        PathIterator pit = shape.getPathIterator( at );
        
        int n = (int) curveResolution.getValue();
        double[] c = new double[6];
        Point2d p = new Point2d(0,0);        
        gl.glPushMatrix();
        gl.glTranslated( 50, 400, 0);
        while ( !pit.isDone() ) {            
            int type = pit.currentSegment(c);
            if ( type == PathIterator.SEG_LINETO ) {
                gl.glBegin( GL.GL_LINES );
                gl.glVertex2d( p.x, p.y );
                gl.glVertex2d( c[0], c[1] );
                gl.glEnd();
                p.set( c[0], c[1]);                
            } else if ( type == PathIterator.SEG_MOVETO ) {
                p.set( c[0], c[1]);
            } else if ( type == PathIterator.SEG_CLOSE ) {
                p.set( c[0], c[1]);             
            } else if ( type == PathIterator.SEG_CUBICTO ) {
                //  P(t) = B(3,0)*CP + B(3,1)*P1 + B(3,2)*P2 + B(3,3)*P3,  0 <= t <= 1
                //         B(n,m) = mth coefficient of nth degree Bernstein polynomial
                //                = C(n,m) * t^(m) * (1 - t)^(n-m)
                //         C(n,m) = Combinations of n things, taken m at a time
                //                = n! / (m! * (n-m)!)
                gl.glBegin( GL.GL_LINE_STRIP );
                double x = 0;
                double y = 0;
                for ( int i = 0; i <= n; i++) {
                    double t = i/(double)n;                    
                    double b0 = 1 * t*t*t;
                    double b1 = 3 * t*t*(1-t);
                    double b2 = 3 * t*(1-t)*(1-t);
                    double b3 = 1 * (1-t)*(1-t)*(1-t);                    
                    x = p.x*b3 + c[0]*b2 + c[2]*b1 + c[4]*b0;
                    y = p.y*b3 + c[1]*b2 + c[3]*b1 + c[5]*b0;
                    gl.glVertex2d( x, y );
                }
                gl.glEnd();
                p.set( x, y );                
            } else if ( type == PathIterator.SEG_QUADTO ) {
                // P(t) = B(2,0)*CP + B(2,1)*P1 + B(2,2)*P2, 0 <= t <= 1
                //        B(n,m) = mth coefficient of nth degree Bernstein polynomial
                //               = C(n,m) * t^(m) * (1 - t)^(n-m)
                //        C(n,m) = Combinations of n things, taken m at a time
                //               = n! / (m! * (n-m)!)
                gl.glBegin( GL.GL_LINE_STRIP );
                double x = 0;
                double y = 0;
                for ( int i = 0; i <= n; i++) {
                    double t = i/(double)n;                    
                    double b0 = 1 * t*t;
                    double b1 = 2 * t*(1-t);
                    double b2 = 1 * (1-t)*(1-t);                                       
                    x = p.x*b2 + c[0]*b1 + c[2]*b0;
                    y = p.y*b2 + c[1]*b1 + c[3]*b0;
                    gl.glVertex2d( x, y );
                }
                gl.glEnd();
                p.set( x, y );                
            }
            pit.next();
        }
        gl.glPopMatrix();
                
        EasyViewer.endOverlay(drawable);
        
    }

    DoubleParameter alpha = new DoubleParameter( "alpha value" , 0.5, 0, 1 );
    
    @Override
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Transparency") );
        vfp.add( alpha.getControls() );
        vfp.add( which.getSliderControls() );
        vfp.add( fname );
        vfp.add( size.getSliderControls());
        vfp.add( curveResolution.getSliderControls());
        vfp.add( bold.getControls() );
        vfp.add( italic.getControls() );
        vfp.add( text );
        CollapsiblePanel cp = new CollapsiblePanel( vfp.getPanel() );
        return cp;   
    }

    /**
     * @param args
     */
    public static void main(String[] args) {
        new EasyViewer("Font Stroke Data Example", new Example2DFontStrokesApp(), new Dimension(640,480), new Dimension(320,480) );
    }
    
}
