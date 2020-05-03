package mintools.viewer;

import java.awt.Component;

/**
 * Interaction interface to allow mouse and key listeners to be attached to 
 * a component, namely the canvas.  You would addMouse and addKey listeners
 * in the attach call of your implementation.  
 * 
 * @author tedmunds
 */
public interface Interactor
{
    /**
     * Attaches listeners to the provided component (the 3D canvas)
     * @param component
     */
    public void attach( Component component );
}