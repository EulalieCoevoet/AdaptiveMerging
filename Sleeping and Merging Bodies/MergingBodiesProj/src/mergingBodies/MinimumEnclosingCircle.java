package mergingBodies;
/*
 * Mec.java
 *
 * Created on 5. Mai 2008, 10:56
 *
 * Calculates the minimal closing circle of an set of given points in 2D
 * with Welzl's algorithm. Not sure if bugfree.
 *
 * @author Sunshine
 * web: www.sunshine2k.de
 *
 * Personal note: when using vectors instead of arrays in the recursive function
 * mec, the algo does not work. What's the problem?
 * 
 * Simplifed slightly, and modified to use vecmath
 */

import java.util.Collection;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * Algorithm for computing Minimum Enclosing Circle using Welzl's algorithm based on Sunshine's implementation (www.sunshine2k.de)
 */
public class MinimumEnclosingCircle {
    
    private Point2d[] points;
    
    private Point2d[] boundary = new Point2d[3];
        
    /**
     * Container for a circle
     */
    public class Circle {
        /** radius */
        public double radius;
        /** centre */
        public Point2d centre = new Point2d();
        /** empty constructor */
        public Circle() { /* do nothing */ }
        /** 
         * simple constructor 
         * @param radius 
         * @param centre 
         */
        public Circle(double radius, Point2d centre ) {
            this.radius = radius;
            this.centre.set( centre );
        }
        /**
         * @param p query point
         * @return true if point is in the circle
         */
        public boolean isInCircle( Point2d p ) {
            return ( p.distanceSquared( centre ) <= radius * radius );
        }
    }
    
    /** the answer */
    public Circle answer = new Circle();
    
    /** 
     * Creates a local copy of the collection of points 
     * @param points 
     */
    public MinimumEnclosingCircle( Collection<Point2d> points ) {
        try {
            int count = 0;
            this.points = new Point2d[points.size()];
            for ( Point2d p : points ) {
                this.points[count++] = p;            
            }
            calcMec();
        } catch ( StackOverflowError e ) {
            System.err.println("Welzl algo blowing the stack away (" + points.size() + " points), so building conservative AABB instead.");
            Point2d min = new Point2d(Double.MAX_VALUE, Double.MAX_VALUE);
            Point2d max = new Point2d(Double.MIN_VALUE, Double.MIN_VALUE);
            for ( Point2d p : points ) {
                min.x = Math.min( p.x, min.x );
                min.y = Math.min( p.y, min.y );
                max.x = Math.max( p.x, max.x );
                max.y = Math.max( p.y, max.y );
            }            
            answer = new Circle();
            answer.centre.interpolate( max, min, 0.5);
            Vector2d tmp = new Vector2d();
            tmp.sub( max, min );
            answer.radius = 0.5 * Math.max( tmp.x, tmp.y ) * Math.sqrt(2);
        }
    }
        
    /**
     * Calculates the minimal enclosing circle. Prepares data and calls the recursive function.
     */
    private void calcMec() {
        if (points.length == 0) return;        
        // random permutation of point set (perhaps unnecessary?)
        int pos;
        Point2d temp;
        for (int i = 0; i < points.length; i++) {
            pos = (int)(Math.random() * points.length);
            temp = points[i];
            points[i] = points[pos];
            points[pos] = temp;
        }
        answer = mec( points.length, 0 );        
    }

    /**
     * Calculates the minimal enclosing circle recursively
     * Call initially with boundary array empty and b = 0.
     * NOTE: stack overflow will occur for a large number of points!
     * @param n number of points to work on
     * @param b number of boundary points identified
     */
    private Circle mec( int n, int b ) {
        Circle localCircle = null;
        // terminal cases
        if (b == 3) {
            localCircle = calcCircle3( boundary[0], boundary[1], boundary[2] );
        } else if (n == 1 && b == 0) {
            localCircle = new Circle( 0, points[0] );
        } else if (n == 0 && b == 2) {
            localCircle = calcCircle2( boundary[0], boundary[1] );
        } else if (n == 1 && b == 1) {
            localCircle = calcCircle2( boundary[0], points[0] );
        } else {
            localCircle = mec( n-1, b );
            if ( ! localCircle.isInCircle( points[n-1] ) ) {
                //System.out.println("Not in circle");
                boundary[b++] = points[n-1];
                localCircle = mec( n-1, b );
            }
        }
        return localCircle;
    }
  
    /**
     * Calculates the circle through the given 3 points. Should be robust to colinear points
     * @param p1 
     * @param p2 
     * @param p3 
     * @return The circle enclosing the three points.
     */
    public Circle calcCircle3(Point2d p1, Point2d p2, Point2d p3) {
        double a = p2.x - p1.x;
        double b = p2.y - p1.y;     
        double c = p3.x - p1.x;
        double d = p3.y - p1.y;
        double e = a * (p2.x + p1.x) * 0.5 + b * (p2.y + p1.y) * 0.5;
        double f = c * (p3.x + p1.x) * 0.5 + d * (p3.y + p1.y) * 0.5;
        double det = a*d - b*c;    

        if ( det == 0 ) {
            double d12 = p1.distance(p2);
            double d23 = p2.distance(p3);
            double d13 = p1.distance(p3);
            if ( d12 > d23 && d12 > d13 ) {
                return calcCircle2( p1, p2 );
            } else if ( d23 > d13 ) {
                return calcCircle2( p2, p3 );
            } 
            return calcCircle2( p1, p3 );
        }
        
        Circle circle = new Circle();
        circle.centre.x = (d*e - b*f) / det;
        circle.centre.y = (-c*e + a*f) / det;
        circle.radius = p1.distance(circle.centre);
        return circle;
    }

    /**
     * Calculates a circle through the given two points.
     * @param p1 
     * @param p2 
     * @return The circle enclosing the two points
     */
    public Circle calcCircle2(Point2d p1, Point2d p2) {
        Circle circle = new Circle();
        circle.centre.x = 0.5*(p1.x + p2.x);
        circle.centre.y = 0.5*(p1.y + p2.y);
        circle.radius = circle.centre.distance(p1);
        return circle;
    }
}
