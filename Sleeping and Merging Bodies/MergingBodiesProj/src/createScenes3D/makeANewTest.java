package createScenes3D;

import java.io.PrintStream;
import java.util.Random;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class makeANewTest {

	public static void main( String[] args ) {		
		try {
			PrintStream ps = new PrintStream("scenes3D/aNewTestVenus.xml");
			ps.println("<root>");
			createPlane( ps, -10 );
			
			double eps = 1e-4;
			
			int towerID = 0;
			int platformID = 0;
			int stackID = 0;
			int venusID = 0;

			double bs = 4;
			double platformRadius = bs * 2;
			double dx = bs*5;
			for ( int i = -1; i <= 1; i++ ) {
				for ( int j = -1; j <= 1; j++ ) {
					double offx = 0;//rand.nextDouble()*2-1;
					double offz = 0;//rand.nextDouble()*2-1;
//					Point3d pos = new Point3d( dx*i - offx, rand.nextDouble() * 8, dx*j - offz );
					Point3d pos = new Point3d( dx*i - offx, 0, dx*j - offz );
					createPlatform( ps, platformID++, platformRadius, pos, 25, offx, offz, 500, 200, 3 * 0.017 );//rand.nextDouble()*2*0.017*2-0.017*2 );
					if ( i == 0 && j == 0 ) {
						Vector3d size = new Vector3d( 2, 2, 4 ); // brick size in towers			
						createTower( ps, towerID++, platformRadius*0.9 , 3, -eps, size, pos );
						
						//createBallDrop(ps, 200, 0.35, pos);
						createVenus(ps, venusID++, 1, new Point3d( pos.x+3, pos.y + 8, pos.z-3 ) );
						createVenus(ps, venusID++, 1, new Point3d( pos.x-3, pos.y + 8, pos.z ) );
						createVenus(ps, venusID++, 1, new Point3d( pos.x+1, pos.y + 8, pos.z+2 ) );
					} else {
						Vector3d brickSize = new Vector3d( bs,bs,bs );					
						createMessyStack(ps, stackID++, 10, -eps, brickSize, pos);
//						createMessyStack(ps, stackID++, 5, -eps, brickSize, pos);
					}
				}
			}
			
						
//			
			
			
			ps.println("</root>");			
			ps.close();
		} catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	public static void createVenus( PrintStream ps, int venusID, double size, Point3d pos) {
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
	public static void createMessyStack( PrintStream ps, int stackID, int layers, double gap, Vector3d brickSize, Point3d pos ) {
		double sx = brickSize.x;
		double sy = brickSize.y;
		double sz = brickSize.z;
		
		double x = pos.x;
		double y = pos.y + sy/2 + gap;
		double z = pos.z;
		
		for ( int j = 0; j < layers; j++ ) {
			double radians = rand.nextDouble() * 0.5 - 0.25;
			String name = "S" + stackID + "B" + j;
			ps.println("<body type=\"box\" name=\""+name+"\" dim=\"" + asString(sx, sy, sz) + "\">");
			ps.println("    <x> " + asString(x,y,z) + " </x>" );
			ps.println("    <R> 0 -1 0 " + radians + "</R>" );
			addRandCol(ps);			
			ps.println("</body>");
			y += sy/2 + gap;
			sx = brickSize.x * rand.nextDouble()*0.5 + 1;
			sy = brickSize.y* rand.nextDouble()*0.5 + 1;
			sz = brickSize.z * rand.nextDouble()*0.5 + 1;
			y += sy/2 + gap;			
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
	public static void createPlatform( PrintStream ps, int platformID, double r, Point3d pos, double springLength, double offx, double offz, double k, double d, double yrot ) {
		ps.println("<body type=\"box\" name=\"platform"+platformID+"\" dim=\"" + 2*r + " 1 " + 2*r + "\">"); //pW="4. 10. 0." 
		ps.println("    <x> " + asString( pos.x, pos.y-0.5, pos.z ) + " </x>" );
		ps.println("    <R> 0 -1 0 " + yrot + "</R>" );
		ps.println("    <spring pB=\"" + asString( -r, +0.5, -r ) + "\" pW=\"" + asString( pos.x-r+offx, pos.y+springLength, pos.z-r+offz ) + "\" k=\""+k+"\" d=\""+d+"\"/>");
		ps.println("    <spring pB=\"" + asString( -r, +0.5, +r ) + "\" pW=\"" + asString( pos.x-r+offx, pos.y+springLength, pos.z+r+offz ) + "\" k=\""+k+"\" d=\""+d+"\"/>");
		ps.println("    <spring pB=\"" + asString( +r, +0.5, -r ) + "\" pW=\"" + asString( pos.x+r+offx, pos.y+springLength, pos.z-r+offz ) + "\" k=\""+k+"\" d=\""+d+"\"/>");
		ps.println("    <spring pB=\"" + asString( +r, +0.5, +r ) + "\" pW=\"" + asString( pos.x+r+offx, pos.y+springLength, pos.z+r+offz ) + "\" k=\""+k+"\" d=\""+d+"\"/>");
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
