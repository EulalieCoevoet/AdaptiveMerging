package mergingBodies3D;

import java.util.ArrayList;

/**
 * Arrays of pre-allocated contacts to save on memory allocation and garbage 
 * collection of contacts.  Two pools are maintained so that there can be
 * a set of contacts from last time step used for warm starts while new ones
 * are allocated in the primary pool.  A swap will clear the old pool for
 * new allocations while leaving the recently allocated conatcts untouched
 * for the next warm start.  Memory is allocated in two mostly continuous
 * blocks of memory of contact structures to help with cache locality in 
 * the solver (probably would want to organize memory differently if we 
 * really cared).
 * 
 *   TODO: Consider a 3rd pool for contacts that are persistant within 
 *   merged collections... should probably make a copy of the contact when 
 *   merging so that the primary pool memory doesn't get disturbed?  This
 *   might be a bit tricky to figure out...
 *   
 * @author kry
 */
public class ContactPool {

	/**
	 * Initial size of each pool, and also the rate at which they grow
	 */
	private int initialSize = 1024;
	
	private ArrayList<Contact> pool1 = new ArrayList<Contact>(initialSize);
	
	private ArrayList<Contact> pool2 = new ArrayList<Contact>(initialSize);

	private int nextInPool1 = 0;
	
	public ContactPool() {
		for ( int i = 0; i < initialSize; i++ ) {
			pool1.add( new Contact() );			
		}
		for ( int i = 0; i < initialSize; i++ ) {
			pool2.add( new Contact() );			
		}
		nextInPool1 = 0;
	}
	
	public Contact get() {
		if ( nextInPool1 == pool1.size() ) {
			for ( int i = 0; i < initialSize; i++ ) {
				pool1.add( new Contact() );			
			}
		}
		return pool1.get( nextInPool1++ );
	}
	
	/**
	 * Swaps the primary and secondary pools, leaving the most recently
	 * allocated contacts unmodified in the secondary pool while new 
	 * contacts can be created in the primary.
	 */
	public void swapPools() {
		ArrayList<Contact> tmp = pool1;
		pool1 = pool2;
		pool2 = tmp;
		nextInPool1 = 0;
	}
	
	public void clear() {
		// no need to force reallocation... 
		// if this is really the fix, then there is really something weird going on....
//		pool1.clear();
//		pool2.clear();
		nextInPool1 = 0;
	}
	
}
