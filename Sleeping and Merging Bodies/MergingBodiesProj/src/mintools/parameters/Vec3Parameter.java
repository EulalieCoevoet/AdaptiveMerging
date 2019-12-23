package mintools.parameters;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Tuple3d;

import mintools.parameters.DoubleParameter;
import mintools.parameters.Parameter;
import mintools.parameters.ParameterListener;
import mintools.swing.VerticalFlowPanel;

public class Vec3Parameter extends JPanel {

	private static final long serialVersionUID = -7176729017675559468L;

	/** These values are updated when the paramter is changed */
	public float x,y,z;
	
	public DoubleParameter xp = new DoubleParameter( "x", 0, -10, 10 );
	public DoubleParameter yp = new DoubleParameter( "y", 0, -10, 10 );
	public DoubleParameter zp = new DoubleParameter( "z", 0, -10, 10 );
	
	/**
	 * Creates a new vec3 parameter control
	 * The default range is -10 to 10.  To change you must access the members.
	 * Members x y and z are updated when the paramter changes.
	 * Because this object extends JPanel, it can be directly added to your controls.
	 * @param name
	 * @param xDefault
	 * @param yDefault
	 * @param zDefault
	 */
	public Vec3Parameter( String name, double xDefault, double yDefault, double zDefault ) {
		setLayout( new GridBagLayout() );
		GridBagConstraints gbc = new GridBagConstraints();
        gbc.anchor = GridBagConstraints.NORTHWEST;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        gbc.gridheight = 1;
        gbc.gridwidth = 1;
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.ipadx = 0;
        gbc.ipady = 0;
        gbc.weightx = 1; 
        gbc.weighty = 0;
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.setBorder( new TitledBorder(name) );
		vfp.add( xp.getSliderControls(false) );
		vfp.add( yp.getSliderControls(false) );
		vfp.add( zp.getSliderControls(false) );
		this.add( vfp.getPanel(), gbc );
		gbc.gridy++;
        gbc.weighty = 1;
        add( new JPanel(), gbc );
        gbc.weighty = 0;		
        xp.addParameterListener( new ParameterListener<Double>() {
			@Override
			public void parameterChanged(Parameter<Double> parameter) {
				x = xp.getFloatValue();			
			}
		});
		yp.addParameterListener( new ParameterListener<Double>() {
			@Override
			public void parameterChanged(Parameter<Double> parameter) {
				y = yp.getFloatValue();			
			}
		});
		zp.addParameterListener( new ParameterListener<Double>() {
			@Override
			public void parameterChanged(Parameter<Double> parameter) {
				z = zp.getFloatValue();			
			}
		});
		xp.setValue(xDefault);		
		yp.setValue(yDefault);		
		zp.setValue(zDefault);
	}
	
	public void set( Tuple3d p ) {
		xp.setValue( p.x );
		yp.setValue( p.y );
		zp.setValue( p.z );
	}
	
	public void get( Tuple3d p ) {
		p.x = xp.getValue();
		p.y = yp.getValue();
		p.z = zp.getValue();
	}
}
