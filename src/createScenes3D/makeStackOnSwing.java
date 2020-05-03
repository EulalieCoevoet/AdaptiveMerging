package createScenes3D;

import java.io.PrintStream;
import java.util.Random;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class makeStackOnSwing {

	public static void main( String[] args ) {		
		try {
			PrintStream ps = new PrintStream("scenes3D/stackOnSwing.xml");
			ps.println("<root>");
			createPlane( ps, -10 );
			
			double eps = -1e-5;
			int stackID = 0;
			int platformID = 0;
			
			Point3d pos = new Point3d( 0, 0, 0 );
			Vector3d brickSize = new Vector3d( 8,1,8 );					
			createMessyStack(ps, stackID++, 20, -eps, brickSize, pos, 0.02);
					
			double standWidth = 20;
			createPlatform(ps, platformID++, 10, pos, 50, 500, 200, standWidth );
			
			ps.println("</root>");			
			ps.close();

			
		} catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	public static void createVenus( PrintStream ps, int venusID, double size, Point3d pos ) {
		ps.println("<body type=\"mesh\" name=\"torso"+venusID+"\" scale=\""+size+"\" obj=\"data/scaledtorso10.obj\" st=\"data/torso_flux.sph\">");
		ps.println("    <x> " + asString(pos.x,pos.y,pos.z) + " </x>" );
		ps.println("    <R> 1 0 0 -1.757 </R>" );		
		ps.println("</body>");
	}
	
	public static void createPlane( PrintStream ps, double floory ) {
		int space = 0;
		ps.println("<body type=\"plane\" name=\"plane1\" p=\"0 "+(floory-space)+" 0\" n=\"0 1 0\"></body>");
	}

	/** 
	 * Varying box sizes would make this interesting too...
	 * @param ps
	 * @param stackID
	 * @param layers
	 * @param brickSize
	 * @param pos
	 */
	public static void createMessyStack( PrintStream ps, int stackID, int layers, double gap, Vector3d brickSize, Point3d pos, double deltaRad ) {
		double sx = brickSize.x;
		double sy = brickSize.y;
		double sz = brickSize.z;
		
		double x = pos.x;
		double y = pos.y + sy/2 + gap;
		double z = pos.z;
		

		for ( int j = 0; j < layers; j++ ) {
			double radians =  rand.nextDouble() * 0.5 - 0.25;

			String name = "S" + stackID + "B" + j;
			ps.println("<body type=\"box\" name=\""+name+"\" dim=\"" + asString(sx, sy, sz) + "\">");
			ps.println("    <x> " + asString(x,y,z) + " </x>" );
			ps.println("    <R> 0 -1 0 " + radians + "</R>" );
			addRandCol(ps);			
			ps.println("</body>");
			y += sy/2 + gap;
//			sx = brickSize.x * rand.nextDouble()*0.5 + 1;
//			sy = brickSize.y* rand.nextDouble()*0.5 + 1;
//			sz = brickSize.z * rand.nextDouble()*0.5 + 1;
			y += sy/2 + gap;		
			//radians += deltaRad * ( (j%2)==0 ? -1 : 1 );
		}
	}
	
	static int ballID = 0;
	public static void createBallDrop( PrintStream ps, int count, double r, Point3d pos ) {
		double y = pos.y+2*r;		
		for ( int i = 0; i < count; i++ ) {
			double x = pos.x + ( -1. + (i%3)) * r*4  + rand.nextDouble()*r/2-r/4;
			double z = pos.y + ( -1. + ((i/3)%3)) * r*4 + rand.nextDouble()*r/2-r/4;			 
			ps.println("<body type=\"sphere\" name=\"S"+(ballID++)+"\" r=\"" + r * (rand.nextDouble()+1) + "\">");			
			ps.println("    <x> " + asString(x,y+r*i*.4,z) + " </x>" );
			addRandCol(ps);			
			ps.println("</body>");
		}
	}
	
	/**
	 * 
	 * @param ps
	 * @param platformID
	 * @param r
	 * @param pos
	 * @param springLength
	 * @param offx			for an initial swing
	 * @param k
	 * @param d
	 */
	public static void createPlatform( PrintStream ps, int platformID, double r, Point3d pos, double springLength, double k, double d, double standWidth ) {
		
		ps.println("<body type=\"composite\" name=\"stand\">"); 
		ps.println("    <x> 0. 0. 0. </x>" );
		ps.println("    <body type=\"box\" name=\"compositBox1 \" dim=\" " + (2*r+2+standWidth*2) + " 2 " + 2 + "\">");
		ps.println("         <x> "+ asString( 0., springLength, pos.z )+" </x>" );
		ps.println("         <R>  0 -1 0 0. </R>" );		
		ps.println("    </body>");
		ps.println("    <body type=\"box\" name=\"compositBox2 \" dim=\"2 " + (pos.y+10+springLength) + " 2 \">");
		ps.println("         <x> "+ asString( (r+standWidth), (pos.y+springLength-10)/2., pos.z )+" </x>" );
		ps.println("         <R>  0 -1 0 0. </R>" );		
		ps.println("    </body>");
		ps.println("    <body type=\"box\" name=\"compositBox3 \" dim=\"2 " + (pos.y+10+springLength) + " 2 \">");
		ps.println("         <x> "+ asString( (-r-standWidth), (pos.y+springLength-10)/2., pos.z )+" </x>" );
		ps.println("         <R>  0 -1 0 0. </R>" );		
		ps.println("    </body>");
		ps.println("    <body type=\"box\" name=\"compositBox4 \" dim=\" " + 2 + " 2 " + 10 + "\">");
		ps.println("         <x> "+ asString( (r+standWidth), -9, pos.z )+" </x>" );
		ps.println("         <R>  0 -1 0 0. </R>" );		
		ps.println("    </body>");
		ps.println("    <body type=\"box\" name=\"compositBox5 \" dim=\" " + 2 + " 2 " + 10 + "\">");
		ps.println("         <x> "+ asString( (-r-standWidth), -9, pos.z )+" </x>" );
		ps.println("         <R>  0 -1 0 0. </R>" );		
		ps.println("    </body>");
		ps.println("</body>");	
		
		ps.println("<body type=\"box\" name=\"platform"+platformID+"\" dim=\"" + 2*r + " 1 " + 2*r + "\">"); 
		ps.println("    <x> " + asString( pos.x, pos.y-0.5, pos.z ) + " </x>" );
		ps.println("    <spring pB=\"" + asString( -r, +0.5, -r ) + "\" pB2=\" 0. " + (springLength/2-2) + " 0. \" body2= \"stand\" k=\""+k+"\" d=\""+d+"\"/>");
		ps.println("    <spring pB=\"" + asString( -r, +0.5, +r ) + "\" pB2=\" 0. " + (springLength/2-2) + " 0. \" body2= \"stand\" k=\""+k+"\" d=\""+d+"\"/>");
		ps.println("    <spring pB=\"" + asString( +r, +0.5, -r ) + "\" pB2=\" 0. " + (springLength/2-2) + " 0. \" body2= \"stand\" k=\""+k+"\" d=\""+d+"\"/>");
		ps.println("    <spring pB=\"" + asString( +r, +0.5, +r ) + "\" pB2=\" 0. " + (springLength/2-2) + " 0. \" body2= \"stand\" k=\""+k+"\" d=\""+d+"\"/>");
		ps.println("</body>");		
	}
	
	/** 
	 * creates a tower, with blocks rotated about y as we go around a circle of a given radius
	 * @param ps
	 * @param r			radius of circle
	 * @param layers	number of layers
	 * @param gap		gap between layers
	 * @param brickSize 		
	 * @param pos		location of center bottom of tower
	 */
	public static void createTower( PrintStream ps, int towerID, double r, int layers, double gap, Vector3d brickSize, Point3d pos ) {
		double sx = brickSize.x;
		double sy = brickSize.y;
		double sz = brickSize.z; 
		String dim = "dim=\"" + asString(sx, sy, sz) + "\"";

		double y = pos.y + 0.5* sy + gap; 
		// number of blocks that can fit around the circumference with a small safety factor?  
		int N = (int) Math.floor( Math.PI * 2 * r / (sz * 1.2) );			
		for ( int j = 0; j < layers; j++ ) {
			for ( int i = 0; i < N; i++ ) {
				double radians = 2.0 * Math.PI * i / N;
				if ( j%2 == 1 ) {
					radians += Math.PI / N;
				}
				double x = Math.cos( radians ) * r + pos.x;
				double z = Math.sin( radians ) * r + pos.z;
				String name = "T" + towerID + "B" + i + "L" + j;
				ps.println("<body type=\"box\" name=\""+name+"\" " + dim + ">" );
				ps.println("    <x> " + asString(x,y,z) + " </x>" );
				ps.println("    <R> 0 -1 0 " + radians + "</R>" );
				int ind = rand.nextInt( col.length );
				ps.println("    <col> " + col[ind] + "</col>" );
				ps.println("</body>");
			}
			y = y + (float)(sy + gap);
		}	
	}
	
	static Random rand = new Random();
	static String[] col = new String[] {
		"0.8 0.5 0.5",
		"0.5 0.5 0.8",
		"0.5 0.8 0.5",
		"0.8 0.8 0.5",
		"0.5 0.8 0.8",
		"0.8 0.5 0.8",
	};
	
	private static Color3f colour = new Color3f();
	
	public static void addRandCol( PrintStream ps ) {
		colour.set( java.awt.Color.getHSBColor( rand.nextFloat(), .2f, .75f ) );
		ps.println("   <col> " + asString(colour.x, colour.y, colour.z) + " </col>");
	}
	
	public static String asString( double sx, double sy, double sz ) {
		return  sx + "  " + sy + " " + sz; 
	}
	
	public static String asString( int sx, int sy, int sz ) {
		return  sx + "  " + sy + " " + sz; 
	}
}
