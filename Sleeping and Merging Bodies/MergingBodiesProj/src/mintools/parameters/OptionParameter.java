package mintools.parameters;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * A <code>Parameter</code> with multiple options.
 * Controllable by a set of radio buttons.
 * 
 * @author Gabriel Charette
 *
 */
public class OptionParameter extends Parameter<Integer> {
	
//	private final JPanel radioPanel;
	private final JPanel panel;
	private JComboBox comboBox = new JComboBox();

	/**
	 * Creates a new <code>OptionParameter</code>.
	 * 
	 * @param name
	 * 		  the name of this parameter
	 * @param defaultValue
	 * 		  the index of the default option
	 * @param options
	 * 		  options (default is the first one)
	 * @throws IllegalArgumentException
	 * 		  if options.length < 2
	 */
	public OptionParameter(String name, int defaultValue, String... options) throws IllegalArgumentException{
		super(name, defaultValue);
		
		if(options.length < 2) throw new IllegalArgumentException("An OptionParameter requires at least 2 options.");
		
		for (String s : options ) {
			comboBox.addItem(s);
		}
		comboBox.addActionListener( new ActionListener() {	
			@Override
			public void actionPerformed(ActionEvent e) {
				setValue( comboBox.getSelectedIndex() );
			}
		});
		comboBox.setSelectedIndex(defaultValue);
		
		panel = new JPanel( new GridLayout(1,2) );
		panel.add( new JLabel(name) );
		panel.add( comboBox );
		
//		ButtonGroup optionsGroup = new ButtonGroup();
//		radioPanel = new JPanel( new GridLayout(0, 1) );
//		int i = 0;
//		for(String opt : options) {
//			final JRadioButton rb = new JRadioButton(opt);
//			optionsGroup.add(rb);
//			radioPanel.add(rb);
//			
//			//add an action listener to each button to update the value of this OptionParameter
//			final int k = i++;
//			rb.addActionListener( new ActionListener() {
//				@Override
//				public void actionPerformed(ActionEvent e) {
//					if( rb.isSelected() ) setValue(k);
//				}
//			} );
//		}
//		this.setValue(defaultValue);
//		panel = new JPanel( new GridLayout(1,0) );
//		panel.setBorder( new TitledBorder(name) );
//		panel.add(radioPanel);
	}

	@Override
	public JPanel getControls() {
		return panel;
	}

	@Override
	protected void updateView() {
		comboBox.setSelectedIndex( getValue() );
//		if(radioPanel == null) return;
//		JRadioButton selection = (JRadioButton)radioPanel.getComponent( getValue() );
//		if( !selection.isSelected() ) selection.setSelected(true);
	}
}
