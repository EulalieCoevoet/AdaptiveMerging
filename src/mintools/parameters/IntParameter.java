/*******************************************************************************
 * DoubleParameter.java
 * 
 * Created on 21-Sep-2003
 * Created by tedmunds
 * 
 ******************************************************************************/
package mintools.parameters;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.swing.JFormattedTextField;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import mintools.swing.HorizontalFlowPanel;

/**
 * This {@link Parameter}subclass represents a int-valued parameter that can
 * take on values within a specified range.
 * 
 * @author tedmunds 
 */
public class IntParameter extends BoundedParameter<Integer> implements PropertyChangeListener, ChangeListener {
    
    /**
     * Creates a new <code>DoubleParameter</code> with the specified name,
     * default value, minimum value, and maximum value.
     * 
     * @param name
     *        the name of this parameter
     * @param defaultValue
     *        the default value of this parameter
     * @param minValue
     *        the minimum value of this parameter
     * @param maxValue
     *        the maximum value of this parameter
     */
    public IntParameter(String name, int defaultValue, int minValue, int maxValue) {        
        super(name, defaultValue, minValue, maxValue);
    }
    
    /**
     * The slider that displays/controls the parameter.
     */
    private JSlider slider;
    
    /**
     * Default floating point format as a <code>NumberFormat</code>.
     */
    public static NumberFormat DEFAULT_NUMBER_FORMAT = new DecimalFormat("0");
    
    /**
     * Default width in pixels of the label to the left of a slider in a
     * slideNText panel. If the label doesn't fit in this width a tool tip with
     * the parameter name will be set.
     */
    public static int DEFAULT_SLIDER_LABEL_WIDTH = 160;
    public static int DEFAULT_SLIDER_TEXT_WIDTH = 160;
    
    /**
     * The text field that displays this parameter's value.
     */
    private JFormattedTextField textField;
    
    private JPanel panel = null;
    
    /**
     * Gets a slider and text box control for this int parameter.
     * @return controls
     */
    public JPanel getSliderControls() {
        if ( panel != null ) return panel;
        // Create the ui components
        JLabel label = new JLabel( getName() );
        slider = new JSlider( SwingConstants.HORIZONTAL, minValue, maxValue, value );
        textField = new JFormattedTextField( DEFAULT_NUMBER_FORMAT );
        
        textField.setText( "" + value );

        slider.addChangeListener(this);
        textField.addPropertyChangeListener("value", this);
        
        Dimension d;
        
        d = new Dimension(textField.getPreferredSize());
        d.width = DEFAULT_SLIDER_TEXT_WIDTH;
        textField.setPreferredSize(d);
        textField.setMinimumSize(d);

        d = new Dimension(label.getPreferredSize());
        if (d.width > DEFAULT_SLIDER_LABEL_WIDTH) {
            label.setToolTipText( getName() );
        }
        d.width = DEFAULT_SLIDER_LABEL_WIDTH;
        label.setPreferredSize(d);
        label.setMinimumSize(d);
        
        // Create the panel that holds the ui component
        panel = new JPanel();

        GridBagLayout layout = new GridBagLayout();
        GridBagConstraints gbc = new GridBagConstraints();
        gbc.anchor = GridBagConstraints.NORTHWEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        gbc.gridheight = 1;
        gbc.gridwidth = 1;
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.insets = new Insets(2, 4, 2, 4);
        gbc.ipadx = 0;
        gbc.ipady = 0;
        gbc.weightx = 0;
        gbc.weighty = 0;


        panel.setLayout(layout);
        panel.add(label, gbc);
        gbc.gridx++;
        gbc.weightx = 1;
        panel.add(slider, gbc);
        gbc.gridx++;
        gbc.weightx = 0;
        panel.add(textField, gbc);
        gbc.gridx++;

        setValue(defaultValue);

        return panel;
    }

    /**
     * Gets a  text box control for this int parameter.
     * @return controls
     */
    public JPanel getControls() {
        if ( panel != null ) return panel;
        // Create the ui components
        JLabel label = new JLabel( getName() );
        textField = new JFormattedTextField( DEFAULT_NUMBER_FORMAT );
        
        textField.setText( "" + value );

        textField.addPropertyChangeListener("value", this);
        
        Dimension d;
        
        d = new Dimension(textField.getPreferredSize());
        d.width = DEFAULT_SLIDER_LABEL_WIDTH;
        textField.setPreferredSize(d);
        textField.setMinimumSize(d);

        d = new Dimension(label.getPreferredSize());
        if (d.width > DEFAULT_SLIDER_LABEL_WIDTH) {
            label.setToolTipText( getName() );
        }
        d.width = DEFAULT_SLIDER_LABEL_WIDTH;
        label.setPreferredSize(d);
        label.setMinimumSize(d);
        
        // Create the panel that holds the ui component
        HorizontalFlowPanel hfp = new HorizontalFlowPanel();
        hfp.add( label );
        hfp.add( textField );
        panel = hfp.getPanel();
        
        return panel;
    }

    
    /**
	 * Updates this parameter's <code>JSlider</code> and <code>JFormattedTextField</code>
	 * to reflect the range and
	 * current value of this slider's parameter.
	 */
	protected void updateView() {
	    if ( slider != null ) {
	        slider.setValue( value );
	    }
	    if (textField != null) textField.setValue(getValue());        
	}

	/**
     * Called by the text field when a new value is committed. The updated value
     * of the text field is passed down to the represented parameter.
     * 
     * @param evt
     *        ignored
     * @see java.beans.PropertyChangeListener#propertyChange(PropertyChangeEvent)
     */
    public void propertyChange(PropertyChangeEvent evt) {
        Object valueO = textField.getValue();
        if (valueO instanceof Number) {
            int v = ((Number)valueO).intValue();
            setValue(v);
            return;
        }
    }

    /**
     * Called whenever the state of this slider's <code>JSlider</code> is
     * changed. The change in state is passed down to this slider's
     * <code>Parameter</code>
     * 
     * @param event
     *        the change event (ignored)
     */
    public void stateChanged(ChangeEvent event) {
        value = slider.getValue();
        setValue(value);
    }
    
}
