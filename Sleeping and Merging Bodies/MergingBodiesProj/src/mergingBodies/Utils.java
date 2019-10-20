package mergingBodies;

import java.util.Random;
import javax.vecmath.Color3f;

public class Utils {

	public Color3f getRandomColor() {
		
		Float r = (new Random()).nextFloat();
		Float g = (new Random()).nextFloat();
		Float b = (new Random()).nextFloat();
		
		if (Math.abs(r - g) < 1.5e-1 && Math.abs(g - b) < 1.5e-1) // avoid gray
			return getRandomColor();
		
		return new Color3f(r, g, b);
	}
}
