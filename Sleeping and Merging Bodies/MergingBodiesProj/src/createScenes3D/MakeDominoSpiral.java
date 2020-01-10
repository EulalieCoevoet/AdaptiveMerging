package createScenes3D;
import java.io.PrintStream;
import java.util.Random;


public class MakeDominoSpiral {

	public static void main( String[] args ) {		
		try {
			PrintStream ps = new PrintStream("scenes3D/dominoSpiral.xml");
			ps.println("<root>");
			
			int numLoops = 10;
			double rStart = 100;	// given initial radius... will be made gradually smaller
			double rEnd = 4; //when r reaches this value, terminate the domino generation
			double r = rStart;
//			int sx = 3;
//			int sy = 2;
//			int sz = 5; 
	
			double sx = 2;
			double sy = 4;
			double sz = 0.5; 

			int floory = -10;
			String dim = "dim=\"" + asString(sx, sy, sz) + "\"";
			// leave room to add a bouncy plane by hand...  ?
 			int space = 0;
			ps.println("<body type=\"plane\" name=\"plane1\" p=\"0 "+(floory-space)+" 0\" n=\"0 1 0\"></body>");
			// first layer of blocks above the floor, taking into account
			// the sqrt2 bounding sphere that goes slightly outside blocks
			float y = (float) ( floory + sy*0.5 + (Math.sqrt(2)-1) );
			// number of blocks that can fit around the circumference
			// we want half that much usually. 
				
			
			
			for(int j = 0; j < numLoops; j++) {
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
}
