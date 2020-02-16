package mergingBodies3D;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JPanel;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.OptionParameter;
import mintools.swing.VerticalFlowPanel;

/**
 * Animate body selected by user
 * TODO: add to XML parser
 *
 */
public class Animation {

	protected ArrayList<RigidBody> bodies;
	protected RigidBody body;
	protected int selection=0;
	protected String [] options;
	protected double time;
	protected boolean play = false;
	protected boolean reset = false;
	
	public class AnimationParameters {
		public OptionParameter bodyName;

		public BooleanParameter persistant = new BooleanParameter("persistant (if true will keep the velocities after 'time')", false );
		
		public DoubleParameter time = new DoubleParameter("time (to reach velocity)", 1, 0, 100 );
		
		public DoubleParameter vx = new DoubleParameter("vx", 0, -100, 100 );
		public DoubleParameter vy = new DoubleParameter("vy", 0, -100, 100 );
		public DoubleParameter vz = new DoubleParameter("vz", 0, -100, 100 );
		
		public DoubleParameter wx = new DoubleParameter("wx", 0, -100, 100 );
		public DoubleParameter wy = new DoubleParameter("wy", 0, -100, 100 );
		public DoubleParameter wz = new DoubleParameter("wz", 0, -100, 100 );
	}
	public AnimationParameters params = new AnimationParameters();
	
	public Animation() {
	}
	
	public void init(ArrayList<RigidBody> bodies) {	
		
		this.bodies = bodies;
		selection = 0;
		body = null;
		time = 0.;
		play=false;
		reset=false;
		
		if (bodies.isEmpty())
			return;
		
		options = new String[bodies.size()+1];
		
		int i = 0;
		options[i++] = "";
		for (RigidBody body: bodies) {
			options[i++] = body.name;
		}
		
		if (params.bodyName == null)
			params.bodyName = new OptionParameter("selected body (ctrl+click left on body)", 0, options);
		else
			params.bodyName.setOptions(options);
	}
	
	public void reset() {		
		time = 0.;
		play = false;
	}
	
	public void apply(double dt) {
		updateSelection();
		
		if (reset) {
			resetVelocities();
			time = 0;
			reset = false;
			play = false;
		}
		
		if (!play)
			return;
		
		if (body != null) {
			
			if(!params.persistant.getValue()) {
				applyNonPersistant(dt);
			}
			
			if(params.persistant.getValue()) 
				applyPersistant(dt);
		}
	}
	
	protected void applyNonPersistant(double dt) {

		double ratio = (params.time.getValue()<1e-10)? 1. : time/params.time.getValue(); 
		
		if(params.vx.getValue() != 0.) {
			time+=dt;
			this.body.sleeping = false;
			body.v.x = params.vx.getValue()*ratio;
			if (time > params.time.getValue()) {
				params.vx.setValue(0.);
				reset();
			}
		}
		if(params.vy.getValue() != 0.) {
			time+=dt;
			this.body.sleeping = false;
			body.v.y = params.vy.getValue()*ratio;
			if (time > params.time.getValue()) {
				params.vy.setValue(0.);
				reset();
			}
		}
		if(params.vz.getValue() != 0.) {
			time+=dt;
			this.body.sleeping = false;
			body.v.z = params.vz.getValue()*ratio;
			if (time > params.time.getValue()) {
				params.vz.setValue(0.);
				reset();
			}
		}
		if(params.wx.getValue() != 0.) {
			time+=dt;
			this.body.sleeping = false;
			body.omega.x = params.wx.getValue()*ratio;
			if (time > params.time.getValue()) {
				params.wx.setValue(0.);
				reset();
			}
		}
		if(params.wy.getValue() != 0.) {
			time+=dt;
			this.body.sleeping = false;
			body.omega.y = params.wy.getValue()*ratio;
			if (time > params.time.getValue()) {
				params.wy.setValue(0.);
				reset();
			}
		}
		if(params.wz.getValue() != 0.) {
			time+=dt;
			this.body.sleeping = false;
			body.omega.z = params.wz.getValue()*ratio;
			if (time > params.time.getValue()) {
				params.wz.setValue(0.);
				reset();
			}
		}
	}
	
	protected void applyPersistant(double dt) {
		
		double ratio = (params.time.getValue()<1e-10)? 1. : time/params.time.getValue();
		if (time < params.time.getValue())
			time+=dt;
		else
			ratio = 1.;
		
		this.body.sleeping = false;
		body.v.x = params.vx.getValue()*ratio;
		body.v.y = params.vy.getValue()*ratio;
		body.v.z = params.vz.getValue()*ratio;
		body.omega.x = params.wx.getValue()*ratio;
		body.omega.y = params.wy.getValue()*ratio;
		body.omega.z = params.wz.getValue()*ratio;
	}
	
	protected void updateSelection() {
		if (params.bodyName.getValue()!=selection) {
			reset=true;
			selection = params.bodyName.getValue();
			for (RigidBody body: bodies) {
				
				if (body instanceof RigidCollection) {
					for (RigidBody b: ((RigidCollection)body).bodies) {
						if (options[selection].contentEquals(b.name)) {
							this.body = b;
							return;
						}
					}
					continue;
				}
				
				if ( options[selection].contentEquals(body.name)) {
					this.body = body;
					return;
				}
			}
		}
	}
	
	protected void resetVelocities() {
		params.vx.setValue(0.);
		params.vy.setValue(0.);
		params.vz.setValue(0.);
		params.wx.setValue(0.);
		params.wy.setValue(0.);
		params.wz.setValue(0.);
	}
	
	protected void setSelection(String name) {
		for (int i=0; i<options.length; i++) {
			if (options[i].contentEquals(name)) {
				params.bodyName.setValue(i);
				return;
			}
		}
	}
	
	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		
		JPanel controls = new JPanel( new GridLayout(1,2));
		JButton playButton = new JButton("play");
		controls.add(playButton);
        playButton.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	play = true;
            }
        });
        JButton resetButton = new JButton("reset");
        controls.add(resetButton);
        resetButton.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	reset = true;
            }
        });
        vfp.add(controls);
		
		if (params.bodyName != null)
			vfp.add( params.bodyName.getControls() );
		vfp.add( params.persistant.getControls() );
		vfp.add( params.time.getSliderControls(false) );
		vfp.add( params.vx.getSliderControls(false) );
		vfp.add( params.vy.getSliderControls(false) );
		vfp.add( params.vz.getSliderControls(false) );
		vfp.add( params.wx.getSliderControls(false) );
		vfp.add( params.wy.getSliderControls(false) );
		vfp.add( params.wz.getSliderControls(false) );
		return vfp.getPanel();
	}
}
