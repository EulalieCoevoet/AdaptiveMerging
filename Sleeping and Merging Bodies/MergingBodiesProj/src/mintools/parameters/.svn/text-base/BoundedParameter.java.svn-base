package mintools.parameters;

/**
 * This abstract class is an extension of <code>Parameter</code> that allows
 * a generic parameter to have a minimum and maximum bound.
 * 
 * @author Gabriel Charette
 *
 * @param <T> The type of the values stored in this parameter.
 */
public abstract class BoundedParameter<T extends Comparable<T>> extends Parameter<T> {

	/**
	 * The minimum value this parameter can take.
	 */
    protected T minValue;
    
    /**
     * The maximum value this parameter can take.
     */
    protected T maxValue;
	
    /**
     * Creates a new <code>BoundedParameter</code> with the specified name,
     * default value, minimum value, and maximum value.
     * 
     * @param name
     *        the name of this parameter
     * @param defaultValue
     *        the default value of this parameter
     * @param minValue
     *        the minimum value of this parameter
     * @param maxValue
     *        the maximum value of this parameter
     * @throws IllegalArgumentException
     * 		  if minValue > maxValue
     */
	public BoundedParameter(String name, T defaultValue, T minValue, T maxValue) throws IllegalArgumentException{
		super(name, defaultValue);
		
		if(minValue.compareTo(maxValue) > 0) throw new IllegalArgumentException("minValue must be less than or equal to maxValue");
		
		 // Make sure default and current values are within range
        if (defaultValue.compareTo(minValue) < 0) {
        	this.defaultValue = minValue;
        	this.value = minValue;
        } else if (defaultValue.compareTo(maxValue) > 0) {
        	this.defaultValue = maxValue;
        	this.value = minValue;
        }
        this.minValue = minValue;
        this.maxValue = maxValue;
	}

    /**
     * Sets the value of this parameter. Registered
     * <code>ParameterListener</code>'s are notified of the change. The value
     * is clamped to [minValue, maxValue] where minValue and maxValue are the
     * parameter's minimum and maximum allowable values respectively.
     * @param value
     *        the new value of this parameter
     */
	@Override
    public void setValue(T value) {
        if (testBeingSet()) return;
        this.value = value;
        if (this.value.compareTo(minValue) < 0) this.value = minValue;
        else if (this.value.compareTo(maxValue) > 0) this.value = maxValue;
        updateView();
        this.finishSetting();
    }
	
    /**
     * Sets the minimum allowed value of this parameter. The default and current
     * value may be adjusted to comply.
     * @param minValue
     *        the new minimum value of this parameter
     * @throws IllegalArgumentException
     * 		  if the new minValue is greater than the current maxValue
     */
    public void setMinimum(T minValue) throws IllegalArgumentException{
        if (testBeingSet()) return;
        
        if(minValue.compareTo(maxValue) > 0) throw new IllegalArgumentException("minValue must be less than or equal to maxValue");
        
        this.minValue = minValue;
        if (defaultValue.compareTo(minValue) < 0) this.defaultValue = minValue;
        if (value.compareTo(minValue) < 0) value = minValue;
        updateView();
        this.finishSetting();
    }

	/**
	 * Sets the maximum allowed value of this parameter. The default and current
	 * value may be adjusted to comply.
	 * @param maxValue
	 *        the new maximum value of this parameter
	 * @throws IllegalArgumentException
	 * 		  if the new maxValue is less than the current minValue
	 */
	public void setMaximum(T maxValue) throws IllegalArgumentException{
	    if (testBeingSet()) return;
	    
	    if(maxValue.compareTo(minValue) < 0) throw new IllegalArgumentException("maxValue must be greater than or equal to minValue");
	    
	    this.maxValue = maxValue;
	    if (defaultValue.compareTo(maxValue) > 0) this.defaultValue = maxValue;
	    if (value.compareTo(maxValue) > 0) value = maxValue;
	    updateView();
	    this.finishSetting();
	}
	
    /**
     * Gets the minimum value that this parameter may take on.
     * @return the minimum value of this parameter
     */
    public T getMinimum() {
        return minValue;
    }

    /**
     * Gets the maximum value of this parameter.
     * @return the maximum value that this parameter may take on
     */
    public T getMaximum() {
        return maxValue;
    }
}
