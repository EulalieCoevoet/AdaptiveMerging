package mintools.parameters;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;


/**
 * This abstract class is the parent class for a variety of parameters.  A
 * <code>Parameter</code> is meant to represent a varying application parameter
 * whose value may be displayed/changed through a variety of views.
 * 
 * @author tedmunds (charette:modified to make it generic)
 * 
 * @param <T> The type of the values stored in this parameter.
 */
public abstract class Parameter<T>
{
    /**
     * The name of this parameter.
     */
    protected final String name;
    
    /**
     * The default value of this parameter.
     */
    protected T defaultValue;
    
    /**
     * The current value of this parameter.
     */
    protected T value;
    
    /**
     * The <code>List</code> of {@link ParameterListener}s registered to receive
     * notification whenever this parameter's value is changed.
     */
    private List<ParameterListener<T>> parameterListeners = new ArrayList<ParameterListener<T>>();
    
    /**
     * An indicator of whether a set of this parameter's value is currently
     * in progress.
     */
    private boolean isBeingSet;

//    /**
//     * The list of {@link ParameterView}s registered as views of this
//     * parameter.
//     */
//    private List<ParameterView> views;
    
    /**
     * Constructs a parameter of type <code>T</code> with the specified name
     * and default value.
     * 
     * @param name the name of the parameter
     * @param defaultValue the default value of this parameter
     */
    public Parameter(String name, T defaultValue) {
        super();        
        this.name = name;
        this.defaultValue = defaultValue;
        this.value = defaultValue;
    }

    /**
     * Sets the value of this parameter.  Registered
     * <code>ParameterListener</code>s are notified of the change.
     * @param value the new value of this parameter
     */
    public void setValue(T value)
    {
        if (testBeingSet()) return;
        this.value = value;
        updateView();
        this.finishSetting();
    }

    /**
     * Sets the default value of this parameter.
     * @param value the new default value for this parameter
     */
    public void setDefaultValue( T value ) {
    	this.defaultValue = value;
    }
    
    /**
     * Gets the value of this parameter.
     * @return the current value of this parameter.
     */
    public T getValue() {
        return value;
    }
    
    /**
     * Gets the default value of this parameter.
     * 
     * @return the default value of this parameter.
     */
    public T getDefaultValue()
    {
        return defaultValue;
    }
    
    /**
     * Gets the name of this parameter.
     * 
     * @return the name of this parameter
     */
    public String getName() {
        return name;
    }

    /**
     * Registers the specified {@link ParameterListener} to receive
     * notification whenever this parameter changes.
     * 
     * @param listener the listener to register
     */
    public void addParameterListener(ParameterListener<T> listener) {
        parameterListeners.add(listener);
    }
    
    /**
     * Returns the list of all the <code>ParameterListener</code> s added to
     * this <code>Parameter</code> with
     * {@link #addParameterListener(ParameterListener)}.
     * 
     * @return all of the <code>ParameterListener</code> s added or an empty
     *         list if no listeners have been added
     */
    public List<ParameterListener<T>> getParameterListeners()
    {
        return parameterListeners;
    }
    
    /**
     * Removes a <code>ParameterListener</code>.
     * @param listener the <code>ParameterListener</code> to remove
     */
    public void removeParameterListener(ParameterListener<T> listener)
    {
        parameterListeners.remove(listener);
    }

//    /**
//     * Registers the provided <code>ParameterView</code> as a view of this
//     * parameter.
//     * 
//     * @param view
//     *        the view to register
//     */
//    public void addView(ParameterView view)
//    {
//        views.add(view);
//        notifyView(view);
//    }

    /**
     * If this parameter is not already in the process of being set, it is
     * marked as being set.
     * 
     * @return whether this parameter was already in the process of being
     *         set.
     */
    protected synchronized boolean testBeingSet() {
        boolean value = isBeingSet;
        isBeingSet = true;
        return value;
    }

    /**
     * Marks this parameter as having finished its set.
     * Notifies all listeners about the recent set.
     */
    protected void finishSetting() {
    	synchronized(this) {
    		isBeingSet = false;
    	}
        notifyListeners();
    }

    /**
     * Notifies all registered <code>ParameterListener</code>s of a change
     * in this parameter's value.
     */
    protected void notifyListeners() {
        for ( ParameterListener<T> l : parameterListeners ) {
            l.parameterChanged(this);
        }
    }
    
    /**
     * @return A panel allowing the control of this parameter via the GUI.
     */
    public abstract JPanel getControls();

    /**
     * Asks the parameter to update its view after an update.
     */
	protected abstract void updateView();
    
//    /**
//     * Notifies all of the registered <code>ParameterView</code> s of this
//     * parameter's condition.
//     */
//    protected void notifyViews() {
//        for (int i = 0; i < views.size(); i++)
//        {
//            ParameterView view = views.get(i);
//            notifyView(view);
//        }
//    }
//
//    /**
//     * Notifies the specified view of this parameter's condition.
//     * 
//     * @param view
//     *        the view to be notified
//     */
//    protected void notifyView(ParameterView view)
//    {
//        view.updateView(this);
//    }
       
}
