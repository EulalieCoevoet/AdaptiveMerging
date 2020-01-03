package mergingBodies;
import java.util.Random;

import javax.vecmath.Color3f;

/**
 * This class extends Color3f to allow features like the generation of a random color
 * @author eulalie
 */
public class Color extends Color3f {

	private static final long serialVersionUID = 6836205431065116946L;

	public Color(Color color) {
		super(color);
	}

	public Color() {
		super();
	}

	private static Random rand = new Random();
	
	/**
	 * Return a random color (not gray)
	 * @return Color3f
	 */
	public void setRandomColor() {
		this.set( java.awt.Color.getHSBColor( rand.nextFloat(), 1f, 1f ) );

//		Float r = rand.nextFloat();
//		Float g = rand.nextFloat();
//		Float b = rand.nextFloat();
//		
//		this.set( java.awt.Color.getHSBColor( rand.nextFloat(), 1f, 1f ) );
//		
//		if (Math.abs(r - g) < 1.5e-1 && Math.abs(g - b) < 1.5e-1) // avoid gray
//			this.setRandomColor();
//		
//		this.x = r;
//		this.y = g;
//		this.z = b;
	}
}
