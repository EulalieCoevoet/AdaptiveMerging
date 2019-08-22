package src;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.vecmath.Point2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.IntParameter;
import no.uib.cipr.matrix.DenseVector;

public class PendulumProcessor {
	
	private List<RigidBody> bodies =new ArrayList<RigidBody>();;

    public ArrayList<Constraint> constraints = new ArrayList<Constraint>();
    
    public Point2d pendulum_origin = new Point2d();
    
    public double pendulum_length;
    
	public PendulumProcessor(ArrayList<RigidBody> bodies) {
		
		this.bodies = bodies;
	}

	public void processPendulum(double dt, Point2d pendulum_origin, double pendulum_length) {
	     

        long now = System.nanoTime();
        this.pendulum_origin.set(pendulum_origin);
        this.pendulum_length = pendulum_length;
        find_constraints(pendulum_origin, pendulum_length);
        solve_system(dt);
       
	}
	
	//goes through each body attached to the pendulum, and fills out the constraint jacobian for each one
	private void find_constraints(Point2d pendulum_origin, double pendulum_length) {
		//every rigid body in the system has a constraint
		for (RigidBody b: bodies) {
			//if (b.pendulum_body){
				Constraint c = new Constraint(b, b.x, pendulum_origin, pendulum_length );
				constraints.add(c);
			//}
		}
	}

	private void solve_system(double dt) {
		// assumes all bodies in the system are subject to the pendulum constraints
		DenseVector delta_V = new DenseVector(3*bodies.size());
        delta_V.zero();
        DenseVector lamda = new DenseVector(constraints.size());
        lamda.zero();
        
        organize();
        knuth_shuffle();
        for (int i = 0; i < constraints.size(); i++) {
        	
        	
        	
        	Constraint constraint_i = constraints.get(i);
        	DenseVector j = constraint_i.j;
        	int index = 3*constraint_i.body.index;
        	
        	// get d_i for this constraint
        	double d_i = 0;
    		d_i+= Math.pow(j.get(0), 2) * constraint_i.body.minv;
    		d_i+= Math.pow(j.get(1), 2) * constraint_i.body.minv;
    		d_i+= Math.pow(j.get(2), 2) * constraint_i.body.jinv;
   		
    		//set up components for b' vector
        	double u_x, u_y, u_theta;
        	u_x = constraint_i.body.v.x + dt*constraint_i.body.minv*constraint_i.body.force.x;
        	u_y = constraint_i.body.v.y + dt*constraint_i.body.minv*constraint_i.body.force.y;
        	u_theta = constraint_i.body.omega + dt*constraint_i.body.jinv*constraint_i.body.torque;
        	
        	u_x *= j.get(index);
        	u_y *= j.get(index + 1);
        	u_theta *= j.get(index + 2);
        	
        	double constraint_violation = constraint_i.constraint_violation;
        	
        	// create b' vector  ( will not change through iteration i)
        	double b = (u_x + u_y + u_theta + CollisionProcessor.feedback_stiffness.getValue() *constraint_violation)/d_i;
        	
        	
        	// DenseVector J'
        	DenseVector j_prime = new DenseVector(j);
        	j_prime.scale(1/d_i);
        	
        	
        	//assemble T matrix
        	DenseVector T = new DenseVector(j);
        	T.set(0, j.get(0)*constraint_i.body.minv);
        	T.set(1, j.get(1)* constraint_i.body.minv);
        	T.set(2, j.get(2)*constraint_i.body.jinv);
        	int iteration = iterations.getValue();
        	while(iteration > 0) {
        		
        		//update lamda
        		double lamda_i = lamda.get(i);
        		double j_prime_delta_V = j_prime.get(0)*delta_V.get(index) + j_prime.get(1)*delta_V.get(index+1) + j_prime.get(2)*delta_V.get(index+2);
        		double new_lamda_i = lamda_i - b - j_prime_delta_V;
        		double delta_lamda = new_lamda_i - lamda_i;
        		lamda.set(i, new_lamda_i);
        		
        		
        		//update V
        		T.scale(delta_lamda);
        		delta_V.set(index + 0, delta_V.get(index+0) + T.get(index+0));
        		delta_V.set(index + 1, delta_V.get(index+1) + T.get(index+1));
        		delta_V.set(index + 2, delta_V.get(index+2) + T.get(index+2));
        		
        		
        		
        		
        		
        		
       		/*
       		//if we are looking at a normal component of lamda
        		double lamda_i = lamda.get(i);
        		Constraint constraint_i = constraints.get(i);
        		DenseVector j = new DenseVector(constraint_i.j);
        		
        		
        		double d_i = 0;
        		d_i+= Math.pow(j.get(0), 2) * constraint_i.body.minv;
        		d_i+= Math.pow(j.get(1), 2) * constraint_i.body.minv;
        		d_i+= Math.pow(j.get(2), 2) * constraint_i.body.jinv;
       		
        		//get J Row i Delta V term first
        		//multiply the 3 J values with the appropriate delta V values
        		int first_body_index = 3*constraint_i.body.index;
        		
        		double j_row_i_delta_V = 0;
        		
        		//first body
        		j_row_i_delta_V += j.get(0) * delta_V.get(first_body_index);
        		j_row_i_delta_V += j.get(1) * delta_V.get(first_body_index + 1);
        		j_row_i_delta_V += j.get(2) * delta_V.get(first_body_index + 2);
        		j_row_i_delta_V /= d_i;
        		
        		
        		//now take care of assembling b
        		// find all relevant values of u.
        		double u_x = constraint_i.body.v.x + constraint_i.body.force.x * constraint_i.body.minv*dt;
        		double u_y = constraint_i.body.v.y+ constraint_i.body.force.y * constraint_i.body.minv*dt;
        		double u_omega = constraint_i.body.omega + constraint_i.body.torque * constraint_i.body.jinv*dt;
        		u_x *= j.get(0);
        		u_y *= j.get(1);
        		u_omega *= j.get(2);
        		
        		//assemble b vector
        		double b = (u_x + u_y + u_omega + CollisionProcessor.feedback_stiffness.getValue() * constraint_i.constraint_violation)/d_i;
        		
        		double prev_lamda = lamda_i;
        		//putting everything  for lamda together
        		lamda_i= prev_lamda-b - j_row_i_delta_V;
        		
        		double delta_lamda = lamda_i - prev_lamda;
        		lamda.set(constraint_i.index,  lamda_i);
        		
        		//Now we still need to do the velocity update.
        		// for that we must compute T = MinvJT TIMES deltaLamda
        		double t_x, t_y, t_omega, t_2_x;
        		
        		//first body
    			t_x = j.get(0) * constraint_i.body.minv*delta_lamda;
    			t_y = j.get(1)* constraint_i.body.minv*delta_lamda;
    			t_omega = j.get(2)* constraint_i.body.jinv*delta_lamda;
    			
    			//update delta V;
        		delta_V.set(first_body_index + 0, delta_V.get(first_body_index + 0) + t_x );
        		delta_V.set(first_body_index + 1, delta_V.get(first_body_index + 1) + t_y );
        		delta_V.set(first_body_index + 2, delta_V.get(first_body_index + 2) + t_omega );
        		*/
        		iteration--;
       		}
       		
       	}
       	for(RigidBody body:bodies) {
	    	int index = body.index;
	    	body.v.x += delta_V.get(3*index);
	    	body.v.y += delta_V.get(3*index+1);
	    	body.omega += delta_V.get(3*index+2);
	    
    	}	
        
       
		
	}
	
	private void organize() {
		for (Constraint c : constraints) {
			c.index = constraints.indexOf(c);
		}
		
	}
	
	private void knuth_shuffle() {
		//go through each element in contacts 2.
    	//at each element, swap it for another random member of contacts2. 
    	//at each element, get a random index from i to contacts.size.
    	//swap this element with that element at that index. 
    	// go to next element in contacts 2
    	
		Collections.shuffle(constraints);
		for (Constraint c : constraints) {
			c.index = constraints.indexOf(c);
		}
		
	}
	
	 /** Number of iterations to use in projected Gauss Seidel solve */
    public IntParameter iterations = new IntParameter("iterations for GS solve", 10, 1, 500);
}
