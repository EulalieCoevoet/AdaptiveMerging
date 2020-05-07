/*
 * Created on 23-Feb-2005
 */
package mintools.swing;

import java.awt.Component;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JPanel;
import javax.swing.border.Border;
import javax.swing.border.TitledBorder;

/**
 * Wrapper for a JPanel that enforces a simple vertical flow layout.<p>
 * <p>
 * Components will fill the horizontal space while padding will prevent the
 * items from growing vertically. 
 * 
 * @author kry
 */
public class HorizontalFlowPanel {

    private JPanel panel;
    
    private GridBagConstraints gbc;
    
    private JPanel empty;
    
    /**
     * Create a new vertical control panel where items added will stack
     * on top of one another vertically in a nice manner.     
     */
    public HorizontalFlowPanel() {
        panel = new JPanel();
        panel.setLayout( new GridBagLayout() );
        gbc = new GridBagConstraints();
        gbc.anchor = GridBagConstraints.NORTHWEST;
        gbc.fill = GridBagConstraints.VERTICAL;
        gbc.gridheight = 1;
        gbc.gridwidth = 1;
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.ipadx = 0;
        gbc.ipady = 0;
        gbc.weightx = 0; 
        gbc.weighty = 1;
    
        empty = new JPanel();
        gbc.weightx = 1;
        panel.add( empty );
        gbc.weightx = 0;        
    }
    
    /**
     * Set the name of this control panel (note that this is normally not visible)
     * @param name
     */
    public void setName( String name ) {
        panel.setName( name );
    }
    
    /**
     * Set a border for this control panel.  If the border is a titled border
     * the name of this control panel will be set automatically.
     * @param b
     */
    public void setBorder( Border b ) {
        panel.setBorder( b );
        if( b instanceof TitledBorder ) {
            setName( ((TitledBorder) b).getTitle() );
        }
    }
    
    /**
     * Add a component to this control panel.  Note that components stack 
     * vertically in a nice manner 
     * @param c
     * @return the component
     */
    public Component add(Component c) {
        if (c == null)
        {
            return null;
        }
        panel.remove(empty);
        panel.add( c, gbc ); gbc.gridx++;
        gbc.weightx = 1;
        panel.add( empty, gbc );
        gbc.weightx = 0;
        return c;
    }

    /**
     * @return the actual JPanel with the contents of this control panel
     */
    public JPanel getPanel() {
        return panel;
    }
    
}
