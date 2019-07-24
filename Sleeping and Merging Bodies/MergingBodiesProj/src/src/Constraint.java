package src;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import no.uib.cipr.matrix.DenseVector;

public class Constraint {
	
	/** Next available contact index, used for determining which rows of the jacobian a contact uses */
    static public int nextContactIndex = 0;
    
	int index;
	
	RigidBody body;
	
	Point2d contact_point = new Point2d();
	Vector2d constraint_direction = new Vector2d();
	DenseVector j = new DenseVector(3);
	
	
	double constraint_violation;
	
	 /**
     * Creates a new constraint, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW
     * @param normal
     */
    public Constraint( RigidBody body, Point2d point_contact, Point2d point_origin, double length) {
        this.body = body;
        this.contact_point.set(point_contact);       
        index = nextContactIndex++;        
       
        constraint_direction.sub(point_origin, point_contact);
        constraint_direction.normalize();
        
        Vector2d contact_p = new Vector2d(this.contact_point);
		Vector2d radius = new Vector2d(body.x);
		
		radius.sub(contact_p, radius);
		
		radius = new Vector2d(-radius.y, radius.x);

		j.set(0, constraint_direction.x);
		j.set(1, constraint_direction.y);
		j.set(2, radius.dot(constraint_direction));
		
		double distance = point_contact.distance(point_origin);
		constraint_violation = distance-length;
    }
    
}
