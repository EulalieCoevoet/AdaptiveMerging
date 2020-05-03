/*
 * Created on 1-Oct-2003
 */
package mintools.swing;

import static javax.swing.ScrollPaneConstants.HORIZONTAL_SCROLLBAR_AS_NEEDED;
import static javax.swing.ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Point;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.SwingConstants;

/**
 * A helper class for making nice control panels. <p>
 * Use with {@link VerticalFlowPanel}, or panels with custom layout. <p>
 * <p>
 * The control panel consists of tabs, the contents of which should
 * flow vertically.  Each content panel will be given a vertical scrollbar, and
 * horizontal scrollbars will appear as necessary.<p>
 * <p>
 * Use add( name, yourPanel) to add new controls in a named tab. <p>
 * Use getContentPanel.add( yourPanel , BorderLayout.NORTH ) <p> 
 * <p>
 * The frame is not visible by default, and setVisible(true) must be 
 * called. <p>
 * <p>
 * Note that although this is based off the old JoglGraphics ControlFrame, 
 * it doesn't have the same exception throwing bad behaviour. 
 * 
 * 
 */
public class ControlFrame {

    private JFrame frame;
    private JPanel content;
    private JTabbedPane tabs;
    
    /**
     * Make a new control frame with default size and placement
     * 400x600 at 600,100 
     * @param title 
     */
    public ControlFrame( String title ) {
        this( title, new Dimension( 400,600 ), new Point(600,100) );
    }
    
    /**
     * Make a new control frame with given size and placement
     * @param title 
     * @param d size
     * @param p location
     */
    public ControlFrame( String title, Dimension d, Point p ) {    
        frame = new JFrame();
        frame.setTitle( title );
        frame.setSize(d);
        frame.setLocation(p);
        
        // JFrame.setDefaultLookAndFeelDecorated( true );
        
        tabs = new JTabbedPane();
        tabs.setTabPlacement( SwingConstants.TOP );
        tabs.setPreferredSize( d );
        
        content = new JPanel();
        content.setLayout( new BorderLayout() );
        content.add(tabs, BorderLayout.CENTER );
        
        frame.getContentPane().add( content );        
    }
    
    /**
     * Adds a JPanel with the given name to this control frame.
     * @param panelName
     * @param panel
     */
    public void add( String panelName, JPanel panel ) {
        
        JScrollPane scroller = new JScrollPane(panel,VERTICAL_SCROLLBAR_ALWAYS, HORIZONTAL_SCROLLBAR_AS_NEEDED );
        scroller.getVerticalScrollBar().setUnitIncrement(10);
        scroller.getHorizontalScrollBar().setUnitIncrement(10);
        tabs.add( scroller, panelName );
    }
    
    /**
     * Adds a JPanel with the given name to this control frame, optionally without a scroll bar
     * @param panelName
     * @param panel
     * @param scrollbar
     */
    public void add( String panelName, JPanel panel, boolean scrollbar ) {
        if ( ! scrollbar ) {
            tabs.add( panel, panelName );
        } else {
            JScrollPane scroller = new JScrollPane(panel,VERTICAL_SCROLLBAR_ALWAYS, HORIZONTAL_SCROLLBAR_AS_NEEDED );
            scroller.getVerticalScrollBar().setUnitIncrement(10);
            scroller.getHorizontalScrollBar().setUnitIncrement(10);
            tabs.add( scroller, panelName );
        }
    }
    
    /**
     * Set the size of this control frame
     * @param d
     */
    public void setSize( Dimension d ) {
        frame.setSize(d);
    }
    
    /**
     * Set the size of this control frame
     * @param width
     * @param height
     */
    public void setSize( int width, int height ) {
        frame.setSize( width, height );
    }
    
    /**
     * Set the screen location of this control frame
     * @param p
     */
    public void setLocation( Point p ) {
        frame.setLocation(p);
    }
    
    /**
     * Set the screen location of this control frame
     * @param x
     * @param y
     */
    public void setLocation( int x, int y ) {
        frame.setLocation( x, y );
    }

    /**
     * Set the visibility of this control frame
     * @param visible
     */
    public void setVisible( boolean visible ) {
        frame.setVisible( visible );
    }
       
    /**
     * Get the content panel. <p>
     * Note that this panel uses a BorderLayout.  Use BorderLayout.NORTH etc.
     * to add universal controls that should be visible at all times.
     * @return content panel with BorderLayout
     */
    public JPanel getContentPanel() {
        return content;
    }
    
    /**
     * Gets the frame.
     * Useful for setting up a shutdown event on closing the window, e.g. <p><p>
     * 
     * getJFrame().addWindowListener( new WindowAdapter() {
     *       public void windowClosing( WindowEvent evt ) {
     *           System.exit(1);
     *       }});
     * 
     * <p><p>
     * @return the JFrame 
     */
    public JFrame getJFrame() {
        return frame;
    }
    
    /**
     * Set the named panel as the visible panel
     * @param panelName
     */
    public void setSelectedTab( String panelName ) {
        //frame.setVisible(true);
        int panelIndex = tabs.indexOfTab( panelName );
        if ( panelIndex != -1 ) {
            tabs.setSelectedIndex( panelIndex );
        }
    }
}
