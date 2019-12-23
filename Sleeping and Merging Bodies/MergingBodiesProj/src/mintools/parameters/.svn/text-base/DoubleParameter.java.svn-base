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
 * This {@link Parameter}subclass represents a double-valued parameter that can
 * take on values within a specified range.
 * 
 * @author tedmunds (kry: modified and simplified for 599)
 */
public class DoubleParameter extends BoundedParameter<Double> implements PropertyChangeListener, ChangeListener {
    
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
    public DoubleParameter(String name, double defaultValue, double minValue, double maxValue) {        
        super(name, defaultValue, minValue, maxValue);
    }
    
    /**
     * Returns the double value stored in this parameter as a float.
     * @return The current value of this <code>DoubleParameter</code> as a float.
     */
    public float getFloatValue() {
    	/* 
    	 * A simple cast to float of the value returned by getValue.
    	 * This helper function is needed because generics don't handle
    	 * primitives and the Double object can't be cast directly to
    	 * float in Java6.
    	 * Having a helper function is nicer than having (float)(double)
    	 * casts all over the code in my opinion.
    	 */
    	return (float)(double)this.getValue();
    }

    /**
     * Sets the value
     * This helps type promotion and auto boxing to occur
     * @param value
     */
    public void setValue(double value) {
    	super.setValue(value);
    }
    
    /**
     * The slider that displays/controls the parameter.
     */
    private JSlider slider;
    
    /**
     * Whether this slider is a logarithmic scale slider.
     */
    private boolean isLogarithmic;

    /**
     * Default number of ticks for number sliders.
     */
    public static int DEFAULT_SLIDER_TICKS = 2000;
    
    /**
     * Default floating point format for a floating point text field.
     */
    public static String DEFAULT_FLOATING_POINT_FORMAT = "0.0000";
    
    /**
     * Default floating point format as a <code>NumberFormat</code>.
     */
    public static NumberFormat DEFAULT_FLOATING_POINT_NUMBER_FORMAT = new DecimalFormat(DEFAULT_FLOATING_POINT_FORMAT);
    
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
     * Gets a slider and text box control for this double parameter.
     * @param isLog set to true to have log behaviour with slider
     * @return controls
     */
    public JPanel getSliderControls( boolean isLog ) {
        if ( panel != null ) return panel;
        this.isLogarithmic = isLog;
        // Create the ui components
        JLabel label = new JLabel( getName() );
        slider = new JSlider(SwingConstants.HORIZONTAL, 0, DEFAULT_SLIDER_TICKS, DEFAULT_SLIDER_TICKS / 2);
        if ( !isLog ) {
        	textField = new JFormattedTextField( DEFAULT_FLOATING_POINT_NUMBER_FORMAT );
        } else {
        	textField = new JFormattedTextField( new DecimalFormat( "0.000E0") );
        }
        
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
     * Gets a  text box control for this double parameter.
     * @return controls
     */
    public JPanel getControls() {
        if ( panel != null ) return panel;
        this.isLogarithmic = false;
        // Create the ui components
        JLabel label = new JLabel( getName() );
        textField = new JFormattedTextField( DEFAULT_FLOATING_POINT_NUMBER_FORMAT );
        
        textField.setText( "" + value );

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
        HorizontalFlowPanel hfp = new HorizontalFlowPanel();
        hfp.add( label );
        hfp.add( textField );
        panel = hfp.getPanel();
        
        setValue(defaultValue);
        
        return panel;
    }

    
    /**
     * Updates this parameter's <code>JSlider</code> and <code>JFormattedTextField</code>
     * to reflect the range and
     * current value of this slider's parameter.
     */
    protected void updateView() {
        if ( slider != null ) {
            int sliderRange = slider.getMaximum() - slider.getMinimum();
            int ivalue;
            double min = getMinimum();
            double max = getMaximum();
            double v = getValue();
            if (isLogarithmic)
            {
                min = Math.log(min);
                max = Math.log(max);
                v = Math.log(v);
            }
            ivalue = slider.getMinimum()
                     + (int)(Math.round((v - min) * sliderRange / (max - min)));
            slider.setValue(ivalue);
        }
        // update the text field
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
	        double v = ((Number)valueO).doubleValue();
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
        int ivalue = slider.getValue();
        int sliderRange = slider.getMaximum() - slider.getMinimum();
        double min = getMinimum();
        double max = getMaximum();
        double v;
        if (isLogarithmic) {
            min = Math.log(min);
            max = Math.log(max);
        }
        v = min + ivalue * (max - min) / sliderRange;
        if (isLogarithmic) {
            v = Math.exp(v);
        }        
        setValue(v);
    }
    
}
