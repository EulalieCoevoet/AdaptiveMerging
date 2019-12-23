/**
 * Colour.java
 *
 * Static definition of some useful colours...
 *
 * @author Paul Kry
 */

package mintools.viewer;

import javax.vecmath.Color3f;

/**
 * This is a convenience class which contains predefined colours.
 * @author kry
 */
public class Colour
{
    /** 1f, 1f, 1f */
    public static Color3f white   = new Color3f( 1f, 1f, 1f );
    /** 0f, 0f, 0f */
    public static Color3f black   = new Color3f( 0f, 0f, 0f );
    /** 1f, 0f, 0f */
    public static Color3f red     = new Color3f( 1f, 0f, 0f );
    /** 1f,0.5f,0f */
    public static Color3f orange  = new Color3f( 1f, 0.5f, 0f );
    /** 1f, 1f, 0f */
    public static Color3f yellow  = new Color3f( 1f, 1f, 0f );
    /** 0f,0.75f, 0f */
    public static Color3f green   = new Color3f( 0f, 0.75f, 0f); //(0f, 1f, 0f );
    /** 0f, 0f, 1.0f */
    public static Color3f blue    = new Color3f( 0f, 0f, 1.0f ); //(0f, 0f, 0.75f );
    /** 0.25f, 0f, 0.75f */
    public static Color3f indigo  = new Color3f( 0.25f, 0f, 0.75f );
    /** 0.5f, 0f, 0.5f */
    public static Color3f violet  = new Color3f( 0.5f, 0f, 0.5f );
    /** 1f, 0f, 1f */
    public static Color3f magenta = new Color3f( 1f, 0f, 1f );
    /** 0f, 1f, 1f */
    public static Color3f cyan    = new Color3f( 0f, 1f, 1f );
    
    /** 0.1f, 0.1f, 0.1f */
    public static Color3f grey1   = new Color3f( 0.1f, 0.1f, 0.1f );
    /** 0.2f, 0.2f, 0.2f */
    public static Color3f grey2   = new Color3f( 0.2f, 0.2f, 0.2f );
    /** 0.3f, 0.3f, 0.3f */
    public static Color3f grey3   = new Color3f( 0.3f, 0.3f, 0.3f );
    /** 0.4f, 0.4f, 0.4f */
    public static Color3f grey4   = new Color3f( 0.4f, 0.4f, 0.4f );
    /** 0.5f, 0.5f, 0.5f */
    public static Color3f grey5   = new Color3f( 0.5f, 0.5f, 0.5f );
    /** 0.6f, 0.6f, 0.6f */
    public static Color3f grey6   = new Color3f( 0.6f, 0.6f, 0.6f );
    /** 0.7f, 0.7f, 0.7f */
    public static Color3f grey7   = new Color3f( 0.7f, 0.7f, 0.7f );
    /** 0.8f, 0.8f, 0.8f */
    public static Color3f grey8   = new Color3f( 0.8f, 0.8f, 0.8f );
    /**  0.9f, 0.9f, 0.9f */
    public static Color3f grey9   = new Color3f( 0.9f, 0.9f, 0.9f );
    
    /** 0,0,143f/256f */
    public static Color3f darkBlue = new Color3f( 0,0,143f/256f );
    /** 128f/256f, 0, 0 */
    public static Color3f darkRed  = new Color3f( 128f/256f, 0, 0 );
    
    /** skin colour */
    public static Color3f skin = new Color3f(0.941125f, 0.75f, 0.617625f);
    
    private static final double [] colorMapJetIndex = { 0, 8, 24, 40, 56, 64 };
    private static final Color3f [] colorMapJetColor = { darkBlue,
                                                            blue,
                                                            cyan,
                                                            yellow,
                                                            red,
                                                            darkRed };
    
    /**
     * Create a colour using a scheme similar to the matlab jet colour map.
     * This takes a number between 0 and 1 and maps it to a colour between 
     * blue and red
     * @param v a number between 0 and 1
     * @param c return parameter set to the corresponding jet colour
     */
    static public void getJetColor( double v, Color3f c ) {
        double vs = v * colorMapJetIndex[colorMapJetIndex.length-1];
        if( vs < colorMapJetIndex[0] ) {
            c.set( colorMapJetColor[0]);
            return;
        }
        if ( vs > colorMapJetIndex[colorMapJetIndex.length-1] ){
            c.set( colorMapJetColor[colorMapJetColor.length-1] );
            return;
        }
        
        for ( int i = 0; i < colorMapJetIndex.length - 1; i++ ) {
            if ( vs >= colorMapJetIndex[i] && vs <= colorMapJetIndex[i+1] ) {
                double t = ( vs - colorMapJetIndex[i] ) / ( colorMapJetIndex[i+1] - colorMapJetIndex[i] );
                c.interpolate( colorMapJetColor[i], colorMapJetColor[i+1], (float) t );
                return;
            }
        }
    }
}

