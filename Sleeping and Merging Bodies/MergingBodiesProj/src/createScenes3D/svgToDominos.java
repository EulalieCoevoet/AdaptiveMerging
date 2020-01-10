package createScenes3D;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/**
 * The suggestion here is to use SVG to create strokes to convert into collections of blocks
 * - Use the spiral tool
 * - Draw Curves
 * - Or convert an image to a set of strokes!
 * 
 * TODO: Doesn't work with g tags created by inkscape (groups? layers?), so best to drill down to find 
 *       all paths and process them all??
 *       
 * TODO: Only does ONE path... the first path... Make it work for all paths!
 * 
 * Easy to update to do more tricks!  Consider likewise creating floating spring supported platforms to have
 * objects not merge all into the inertial floor plane... or consider making bridges for dominoes to climb some stairs and 
 * go over some others ??  Lots of options for scene building here!!!
 * 
 * @author kry
 *
 */
public class svgToDominos {

	public static void main( String[] args ) {
		new svgToDominos();
	}
	
	public svgToDominos() {
		load("scenes3D/drawing2.svg");
		write("scenes3D/dom2.xml");
		System.out.println("blocks written = " + blockID );
	}
	
	public void load( String fileName ) {
		try {
			InputStream inputStream = new FileInputStream(new File(fileName));
			DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
			DocumentBuilder builder = factory.newDocumentBuilder();
			Document document = builder.parse(inputStream);
			parse( document.getDocumentElement() );
		} catch ( Exception e ) {
			e.printStackTrace();
		}
		
	}
	
	/**
	 * Build blocks along all paths in the SVG
	 * @param fname
	 */
	public void write( String fname ) {
		// now build blocks along all paths in the SVG
		
		try {
			PrintStream ps = new PrintStream( fname );
			ps.println("<root>");
			createPlane( ps, 0 );
			
			// choose COM of commands
			// perhaps find bounding box too to scale if the SVG is a bad size?
			// choose first point for center.
			Cmd c = commands.get(0);
			offset.x = c.L.get(0);
			offset.y = c.L.get(1);
			
			strokesToBlocks( ps );
			
			ps.println("</root>");			
			ps.close();
		} catch ( Exception e ) {
			e.printStackTrace();
		}
	}
	
	double arcLength = 0;
	double spacing;
	double eps = 1e-5; 
	Vector3d size = new Vector3d();
	Point2d pos = new Point2d();
	
	Point2d offset = new Point2d(); // not sure where the center is!
	
	private void processSegment( PrintStream ps, Point2d prev, Point2d p ) {
		double d = prev.distance(p);
		if ( d < eps ) return; 
		if ( arcLength + d < spacing ) {
			arcLength += d;
			return;
		}
		while ( arcLength + d > spacing ) {
			double alpha = 1 - (arcLength+d-spacing)/d;
			pos.interpolate( prev, p, alpha );
			double angle = Math.atan2( p.y-prev.y, p.x-prev.x );
			createBlock( ps, pos.x - offset.x, size.y/2, pos.y-offset.y, -angle, size );
			arcLength += - spacing; //d * (alpha);
		}
	}
	
	public void strokesToBlocks( PrintStream ps ) {
		int n = 64; // number of segments for discretizing cubic curves
		
		arcLength = 0;
		spacing = 1.5;
		eps = 1e-5;
		size.set( 0.5, 3, 1.5 );
		
		Point2d p0 = new Point2d();
		Point2d p1 = new Point2d();
		Point2d p2 = new Point2d();
		Point2d p3 = new Point2d();
		Point2d p = new Point2d();
		Point2d prev = new Point2d();
		Point2d tmp = new Point2d();
		int index = 0;

		for (int k = 0; k < commands.size(); k++) {
				Cmd c = commands.get(k);
				if (c.s.startsWith("M")) {
					p0.x = c.L.get(0);
					p0.y = c.L.get(1);
					p.set(p0);
					if (c.L.size() > 2) {
//						gl.glBegin(GL.GL_LINE_STRIP);
//						gl.glVertex2d(p0.x, p0.y);
						
						prev.set( p0 );
						
						for (int j = 2; j < c.L.size(); j += 2) {
							p.x = c.L.get(j);
							p.y = c.L.get(j + 1);
							
							processSegment( ps, prev, p ); prev.set(p);
							
//							gl.glVertex2d(p.x, p.y);
						}
//						gl.glEnd();
					}
					
				} else if (c.s.startsWith("m")) {
					p0.x += c.L.get(0);
					p0.y += c.L.get(1);
					p.set(p0);

					if (c.L.size() > 2) { // draw the rest as lines
//						gl.glBegin(GL.GL_LINE_STRIP);
//						gl.glVertex2d(p0.x, p0.y);
						
						prev.set( p0 );

						for (int j = 2; j < c.L.size(); j += 2) {
							p.x += c.L.get(j);
							p.y += c.L.get(j + 1);

							processSegment( ps, prev, p ); prev.set(p);

//							gl.glVertex2d(p.x, p.y);
						}
//						gl.glEnd();
					}
				} else if (c.s.startsWith("l")) {
//					gl.glBegin(GL.GL_LINE_STRIP);
//					gl.glVertex2d(p.x, p.y);
					
					prev.set( p );

					for (int i = 0; i < c.L.size(); i += 2) {
						p.x += c.L.get(i);
						p.y += c.L.get(i + 1);
						
						processSegment( ps, prev, p ); prev.set(p);

//						gl.glVertex2d(p.x, p.y);
					}
//					gl.glEnd();
				} else if (c.s.startsWith("L")) {
//					gl.glBegin(GL.GL_LINE_STRIP);
//					gl.glVertex2d(p.x, p.y);
					
					prev.set( p );

					for (int i = 0; i < c.L.size(); i += 2) {
						p.x = c.L.get(i);
						p.y = c.L.get(i + 1);
						
						processSegment( ps, prev, p ); prev.set(p);

//						gl.glVertex2d(p.x, p.y);
					}
//					gl.glEnd();
				} else if (c.s.startsWith("h")) {
//					gl.glBegin(GL.GL_LINE_STRIP);
//					gl.glVertex2d(p.x, p.y);
					
					prev.set( p );

					p.x += c.L.get(0);
					
					processSegment( ps, prev, p ); prev.set(p);
					
//					gl.glVertex2d(p.x, p.y);
//					gl.glEnd();
				} else if (c.s.startsWith("v")) {
//					gl.glBegin(GL.GL_LINE_STRIP);
//					gl.glVertex2d(p.x, p.y);

					prev.set( p );

					p.y += c.L.get(0);
					
					processSegment( ps, prev, p ); prev.set(p);
					
//					gl.glVertex2d(p.x, p.y);
//					gl.glEnd();
				} else if (c.s.startsWith("C")) {
					// Bezier curve
//					gl.glBegin(GL.GL_LINE_STRIP);
//					gl.glVertex2d(p.x, p.y);
					
					prev.set( p );
					
					for (int j = 0; j < c.L.size(); j += 6) {
						p1.x = c.L.get(j + 0);
						p1.y = c.L.get(j + 1);
						p2.x = c.L.get(j + 2);
						p2.y = c.L.get(j + 3);
						p3.x = c.L.get(j + 4);
						p3.y = c.L.get(j + 5);
						for (int i = 0; i <= n; i++) {
							double t = i / (double) n;
							double b0 = 1 * t * t * t;
							double b1 = 3 * t * t * (1 - t);
							double b2 = 3 * t * (1 - t) * (1 - t);
							double b3 = 1 * (1 - t) * (1 - t) * (1 - t);
							double x = p.x * b3 + p1.x * b2 + p2.x * b1 + p3.x * b0;
							double y = p.y * b3 + p1.y * b2 + p2.y * b1 + p3.y * b0;

							tmp.set(x,y);
							processSegment( ps, prev, tmp ); prev.set(tmp);

//							gl.glVertex2d(x, y);
						}
						p.set(p3);
					}
//					gl.glEnd();
				} else if (c.s.startsWith("c")) {
					// Bezier curve
//					gl.glBegin(GL.GL_LINE_STRIP);
//					gl.glVertex2d(p.x, p.y);
					
					prev.set( p );
					
					for (int j = 0; j < c.L.size(); j += 6) {
						p1.x = p.x + c.L.get(j + 0);
						p1.y = p.y + c.L.get(j + 1);
						p2.x = p.x + c.L.get(j + 2);
						p2.y = p.y + c.L.get(j + 3);
						p3.x = p.x + c.L.get(j + 4);
						p3.y = p.y + c.L.get(j + 5);
						for (int i = 0; i <= n; i++) {
							double t = i / (double) n;
							double b0 = 1 * t * t * t;
							double b1 = 3 * t * t * (1 - t);
							double b2 = 3 * t * (1 - t) * (1 - t);
							double b3 = 1 * (1 - t) * (1 - t) * (1 - t);
							double x = p.x * b3 + p1.x * b2 + p2.x * b1 + p3.x * b0;
							double y = p.y * b3 + p1.y * b2 + p2.y * b1 + p3.y * b0;
							
							tmp.set(x,y);
							processSegment( ps, prev, tmp ); prev.set(tmp);

//							gl.glVertex2d(x, y);
						}
						p.set(p3);
					}
//					gl.glEnd();
				} else if (c.s.startsWith("z")) {
					
					// can ignore close commands if we don't want to seal up a beginning end end?
					
//					gl.glBegin(GL.GL_LINE_STRIP);
//					gl.glVertex2d(p.x, p.y);
//					gl.glVertex2d(p0.x, p0.y);
//					gl.glEnd();

				} else {
					throw new RuntimeException("unimplemented command " + c.s + " at index " + index);
				}
				index++;
			}
	}
	
	private int blockID = 0;
	
	public void createBlock( PrintStream ps, double px, double py, double pz, double angle, Tuple3d s) {
		String name = "B" + blockID++;
		ps.println("<body type=\"box\" name=\""+name+"\" dim=\"" + asString(s.x, s.y, s.z) + "\">");
		ps.println("    <x> " + asString(px, py, pz) + " </x>" );
		ps.println("    <R> 0 1 0 " + angle + "</R>" );
		addRandCol(ps);			
		ps.println("</body>");
	}
	
	private static Color3f colour = new Color3f();
	private static Random rand = new Random();
	static double hue = 0;
	
	public static void addRandCol( PrintStream ps ) {
		//colour.set( java.awt.Color.getHSBColor( rand.nextFloat(), .2f, .75f ) );
		colour.set( java.awt.Color.getHSBColor( (float) hue, 1, .75f ) );
		hue += 0.01;
		if ( hue >= 1.0 ) hue = 0;
		ps.println("   <col> " + asString(colour.x, colour.y, colour.z) + " </col>");
	}
	
	public static String asString( double sx, double sy, double sz ) {
		return  sx + "  " + sy + " " + sz; 
	}
	
	public static String asString( int sx, int sy, int sz ) {
		return  sx + "  " + sy + " " + sz; 
	}
	
	public void createPlane( PrintStream ps, double floory ) {
		ps.println("<body type=\"plane\" name=\"plane1\" p=\"0 "+(floory)+" 0\" n=\"0 1 0\"></body>");
	}
	
	private void parse( Node dataNode ) {
		//Node gNode = getFirstChild( dataNode, "g" );
		//Node pathNode = getFirstChild( gNode, "path" );
		Node pathNode = getFirstChild( dataNode, "path" );
		String path = pathNode.getAttributes().getNamedItem("d").getNodeValue();
		Scanner s = new Scanner( path );
		s.useDelimiter("[, \\t\\n]");
		while ( s.hasNext() ) {
			Cmd c = new Cmd( s.next(), getFloatList( s ) );
			commands.add( c );
			System.out.print( c.s + " " );
		}
	}
	
	private Node getFirstChild( Node node, String name ) {
		NodeList nodeList = node.getChildNodes();
		for ( int i = 0; i < nodeList.getLength(); i++ ) {
            Node n = nodeList.item(i);
            String nodeName = n.getNodeName();
            if ( nodeName.compareToIgnoreCase( name ) == 0 ) {
            	return n;
            }
		}
		return null;
	}
	
	private List<Float> getFloatList( Scanner s ) {
		List<Float> L = new ArrayList<Float>(); 
		while ( s.hasNextFloat() ) {
			L.add( s.nextFloat() );
		}
		return L;
	}
	
	final class Cmd {
		String s;
		List<Float> L;
		public Cmd( String s, List<Float> L ) {
			this.s = s;
			this.L = L;
		}
	}
	
	private static List<Cmd> commands = new ArrayList<Cmd>();
	
}
