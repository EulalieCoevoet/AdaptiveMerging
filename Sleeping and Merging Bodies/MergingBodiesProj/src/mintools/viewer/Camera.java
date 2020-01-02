package mintools.viewer;

import com.jogamp.opengl.GLAutoDrawable;

public interface Camera {
	
    public void applyInverseViewTransformation(GLAutoDrawable drawable);

    public void prepareForDisplay(GLAutoDrawable drawable);

}
