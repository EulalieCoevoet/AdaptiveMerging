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
	
	protected ArrayList<RigidBody> bodies;
	protected ArrayList<Spring> springs;
	
	public class SleepParameters {
		public BooleanParameter enableSleeping = new BooleanParameter( "sleeping", false);
		public IntParameter stepAccum = new IntParameter("check threshold over N number of time steps", 10, 0, 200 );
		public DoubleParameter threshold = new DoubleParameter("sleeping threshold", 1e-5, 1e-10, 1 );
		public BooleanParameter wakeAll = new BooleanParameter("wake all", false);
	}
	
	public SleepParameters params = new SleepParameters();
	
	MotionMetricProcessor motionMetricProcessor = new MotionMetricProcessor();
	
	public Sleeping(ArrayList<RigidBody> bodies, ArrayList<Spring> springs) {
		this.bodies = bodies;
		this.springs = springs;
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
			
			//This is not safe, single iteration PGS may take multiple time steps to detect something...
			/*if (body.pinned) {
				body.isSleeping = true;
				continue;
			}*/
			
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
		
		for (Spring spring: springs) {
			
			if (spring.body2 == null)
				continue;
			
			boolean sleeping1 = (spring.body1.isInCollection())? spring.body1.parent.isSleeping: spring.body1.isSleeping;
			boolean sleeping2 = (spring.body2.isInCollection())? spring.body2.parent.isSleeping: spring.body2.isSleeping; 			
			boolean pinned1   = (spring.body1.isInCollection())? spring.body1.parent.pinned: spring.body1.pinned;
			boolean pinned2   = (spring.body2.isInCollection())? spring.body2.parent.pinned: spring.body2.pinned; 
			
			if(sleeping1!=sleeping2 && !(pinned1 || pinned2)) {
				spring.body1.wake();
				spring.body2.wake();
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
