package createScenes3D;
import java.io.PrintStream;
import java.util.Random;

import javax.vecmath.Point3d;


public class makeDominoPlatformPyramid {

	public static void main( String[] args ) {		
		try {
			PrintStream ps = new PrintStream("scenes3D/dominoSpiralPlatformPyramid.xml");
			ps.println("<root>");
			//make floor plane
			int floory = -100;
	
			
			// leave room to add a bouncy plane by hand...  ?
 			int space = 0;

			ps.println("<body type=\"plane\" name=\"plane1\" p=\"0 "+(floory-space)+" 0\" n=\"0 1 0\"></body>");
			
			
		
		
			int layers = 4; //how many layers in the pyramid?
			int firstLayerY = 0; //firstplatform will be built at y = 0;
			double platformY = firstLayerY;
			int spaceBetweenLayers = (firstLayerY-floory)/layers;
			
			
			
			//dimensions of domino
			double sx = 2;
			double sy = 4;
			double sz = 0.5; 
			
			
			//first spiral dimesnios... will be increased as we go down the pyramid.
			int numLoops = 2; //how many times iwll the dominos do a full 360 loop?
			double rStart = 10;	// given initial radius... will be made gradually smaller for spiralling 
			double rEnd = 2; //when r reaches this value, terminate the domino generation??? not using this stopping condition but may be better
			double r = rStart; // current radius

			// first layer of blocks above the floor, taking into account
			// the sqrt2 bounding sphere that goes slightly outside blocks
			float y = (float) (firstLayerY+ sy*0.5 + (Math.sqrt(2)-1) );
			for (int n = 0; n < layers; n++) { // for each layer, build a spiral of dominos. 
			
				//build platform the dominos will be on top of. 
				double bs = 4;
				double platformRadius = rStart;
				double dx = bs*5;
				Point3d pos = new Point3d( 0, platformY, 0 );
				//build platform (comment if you dont want to)
				createPlatform(ps, 0, platformRadius, pos, 1, 0, 0, 1000, 10, 0);
				
				String dim = "dim=\"" + asString(sx, sy, sz) + "\"";
				
				for(int j = 0; j < numLoops; j++) {
					// number of blocks that can fit around the circumference
					// we want 1/4 that much usually so hence the divide by 2
					int N = (int) Math.floor( Math.PI  * r / (2*sz * 1.2) );	
					double deltaR = (rStart - rEnd)/(N * numLoops);
					for ( int i = 0; i < N; i++ ) {
						double radians = 2.0 * Math.PI * i / N;
						double x = Math.cos( radians ) * r;
						double z = Math.sin( radians ) * r;
						String name = "B" + i;
						ps.println("<body type=\"box\" name=\""+name+"\" " + dim + ">" );
						ps.println("    <x> " + asString(x,y,z) + " </x>" );
						ps.println("    <R> 0 -1 0 " + radians + "</R>" );
						int ind = rand.nextInt( col.length );
						ps.println("    <col> " + col[ind] + "</col>" );
						ps.println("</body>");
						r -= deltaR;
						
					}
				}
				platformY -= spaceBetweenLayers;
				y -= spaceBetweenLayers; // move down y position of platform along with dominos
				numLoops *= 2; //doubles every time we go bigger in next layer
				rStart += 3*rStart/layers ;	// make radius of spiral bigger for next layer
				rEnd += 3*rEnd/layers; 
				r = rStart; // set current radius to initial
		
				
			}
			

			
		
			ps.println("</root>");			
			ps.close();
		} catch (Exception e ) {
			e.printStackTrace();
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
	
	public static String asString( double sx, double sy, double sz ) {
		return  sx + "  " + sy + " " + sz; 
	}
	
	public static String asString( int sx, int sy, int sz ) {
		return  sx + "  " + sy + " " + sz; 
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
}
