package mergingBodies3D;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JPanel;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;

public class Sleeping {
	
	ArrayList<RigidBody> bodies;
	
	public class SleepParameters {
		public BooleanParameter enableSleeping = new BooleanParameter( "sleeping", true);
		public IntParameter stepAccum = new IntParameter("check threshold over N number of time steps", 10, 0, 200 );
		public DoubleParameter threshold = new DoubleParameter("sleeping threshold", 1e-7, 1e-10, 1 );
		public BooleanParameter wakeAll = new BooleanParameter("wake all", false);
	}
	
	public SleepParameters params = new SleepParameters();
	
	MotionMetricProcessor motionMetricProcessor = new MotionMetricProcessor();
	
	public Sleeping(ArrayList<RigidBody> bodies) {
		this.bodies = bodies;
	}
	
	/**
	 * Checks if we should put these bodies to sleep.
	 * Conditions for sleeping are:
	 * <p><ul>
	 * <li>1. All velocities in velocity history are below threshold. 
	 * <li>2. Velocities in history are monotonically decreasing. 
	 * </ul><p>
	 */
	public void sleep() {
		
		if(!params.enableSleeping.getValue())
			return;
		
		double threshold = params.threshold.getValue();
		for (RigidBody body : bodies) {
			if (body.isSleeping)
				continue;
			
			if (body.pinned) {
				body.isSleeping = true;
				continue;
			}
			
			boolean externalContact = false;
			for (BodyPairContact bpc : body.bodyPairContacts) {
				if (!bpc.inCollection && !(bpc.body1.pinned || bpc.body2.pinned)) {
					externalContact = true;
					break;
				}
			}
			if (externalContact)
				continue;
			
			accumulate(body);
			
			boolean sleep = true;
			double prevMetric = Double.MAX_VALUE; 
			double epsilon = 5e-5;
			if (body.metricHistory.size() < params.stepAccum.getValue()) {
				sleep = false;
			} else {
				for (Double metric : body.metricHistory) {
					if (metric > prevMetric+epsilon) {
						sleep = false;
						break;
					}
					if (metric > threshold) {
						sleep = false;
						break;
					}
					prevMetric = metric;
				}
			}
			
			body.isSleeping = sleep;
		}
	}

	/**
	 * Checks if a body should wake up
	 * conditions for waking:
	 * If total force metric acting on body is above the forceMetric threshold.
	 * total Force metric = totalForce^2/Mass
	 * total Force = sum of all forces (including contact forces)
	 */
	public void wake() {
		
		if(!params.enableSleeping.getValue())
			return;
		
		for (RigidBody body: bodies) {
			if (!body.isSleeping) 
				continue;
			
			for (BodyPairContact bpc : body.bodyPairContacts) {
				if (!bpc.inCollection && !(bpc.body1.pinned || bpc.body2.pinned)) {
					bpc.body1.wake();
					bpc.body2.wake();
				}
			}
		}
	}	
	
	/**
	 * Wake all bodies
	 */
	public void wakeAll() {
		
		for (RigidBody body: bodies)
			body.wake();

		params.wakeAll.setValue(false);
	}
	
	/**
	 * Track metric over time steps
	 */
	protected void accumulate(RigidBody body) {		
		body.metricHistory.add(motionMetricProcessor.getMotionMetric(body));
		if (body.metricHistory.size() > params.stepAccum.getValue()) {
			body.metricHistory.remove(0);	
		}
	}
	
	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.add( params.enableSleeping.getControls() );
		vfp.add( params.threshold.getSliderControls(true) );
		vfp.add( params.stepAccum.getSliderControls() );
        JButton wakeButton = new JButton("wake all");
        vfp.add( wakeButton);
        wakeButton.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	params.wakeAll.setValue(true);
            }
        });
		return vfp.getPanel();
	}
}
