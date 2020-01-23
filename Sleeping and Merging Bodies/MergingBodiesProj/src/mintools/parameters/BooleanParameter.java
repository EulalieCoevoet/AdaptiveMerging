package mintools.parameters;

import java.awt.FlowLayout;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;

import javax.swing.JCheckBox;
import javax.swing.JPanel;


/**
 * Simple boolean parameter class
 * 
 */
public class BooleanParameter extends Parameter<Boolean> implements ItemListener {

    /**
     * Creates a new <code>BooleanParameter</code> with the specified name and
     * default value.
     * @param name the name of this parameter
     * @param defaultValue the default value of this parameter
     */
    public BooleanParameter(String name, boolean defaultValue) {
        super(name, defaultValue);
    }

    private JCheckBox checkBox;

    private JPanel panel = null;
    
    /**
     * Gets a swing panel with a check box to control this parameter
     * @return the control panel
     */
    public JPanel getControls() {
        if ( panel != null ) return panel;
        checkBox = new JCheckBox( getName(), getValue() );
        checkBox.addItemListener(this);
        //parameter.addView(this);
        panel = new JPanel();
        FlowLayout layout = new FlowLayout();
        layout.setAlignment(FlowLayout.LEFT);
        panel.setLayout(layout);
        panel.add(checkBox);
        return panel;    
    }

    /**
     * Updates this checkBox's <code>JCheckBox</code> to reflect the current
     * value of this checkBox's parameter.
     */
    protected void updateView() {
        if ( checkBox == null ) return;
        checkBox.setSelected( getValue() );
    }

    /**
     * Called whenever the state of this checkBox's <code>JCheckBox</code> is
     * changed.  The change in state is passed down to this check box's
     * <code>BooleanParameter</code>.
     * 
     * @param arg0 the change event (ignored)
     */
    public void itemStateChanged(ItemEvent arg0) {
        setValue( checkBox.isSelected() );
    }

}
