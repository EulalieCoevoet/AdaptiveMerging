package createScenes3D;

import java.io.PrintStream;

public class makeCicularTowerApp {

	public static void main( String[] args ) {		
		try {
			PrintStream ps = new PrintStream("scenes3D/tower.xml");
			ps.println("<root>");
			// rotate blocks about y as we go around a circle of a 
			// given radius
			double r = 10;
			int layers = 4;
			int sx = 2;
			int sy = 2;
			int sz = 4; 
			int floory = -10;
			String dim = "dim=\"" + asString(sx, sy, sz) + "\"";
			ps.println("<body type=\"plane\" name=\"plane1\" p=\"0 "+floory+" 0\" n=\"0 1 0\"></body>");
			// first layer of blocks above the floor, taking into account
			// the sqrt2 bounding sphere that goes slightly outside blocks
			float y = (float) ( floory + sy*0.5 + (Math.sqrt(2)-1) );
			// number of blocks that can fit around the circumference
			// with a small safety factor?  
			int N = (int) Math.floor( Math.PI * 2 * r / (sz * 1.2) );			
			for ( int j = 0; j < layers; j++ ) {
				for ( int i = 0; i < N; i++ ) {
					double radians = 2.0 * Math.PI * i / N;
					if ( j%2 == 1 ) {
						radians += Math.PI / N;
					}
					double x = Math.cos( radians ) * r;
					double z = Math.sin( radians ) * r;
					String name = "B" + i + "L" + j;
					ps.println("<body type=\"box\" name=\""+name+"\" " + dim + ">" );
					ps.println("    <x> " + asString(x,y,z) + " </x>" );
					ps.println("    <R> 0 -1 0 " + radians + "</R>" );
					ps.println("</body>");
				}
				y = y + (float)(sy + (Math.sqrt(2)-1));
			}			
			ps.println("</root>");			
			ps.close();
		} catch (Exception e ) {
			e.printStackTrace();
		}
	}
	
	public static String asString( double sx, double sy, double sz ) {
		return  sx + "  " + sy + " " + sz; 
	}
	
	public static String asString( int sx, int sy, int sz ) {
		return  sx + "  " + sy + " " + sz; 
	}
}
