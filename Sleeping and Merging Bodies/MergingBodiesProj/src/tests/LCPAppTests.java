package tests;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import mergingBodies.LCPApp;
import mergingBodies.RigidBodySystem;
import mergingBodies.RigidBody;
import mergingBodies.RigidCollection;
import mergingBodies.Contact;
import javax.vecmath.Vector2d;

public class LCPAppTests extends LCPApp {
	
	double dt = 0.05;
	
	@Override
	public void setUp() {
		// Do nothing, we don't want to launch the app (2D viewer)
	}
	
    @Test
    public void collisionWithCollectionTest() {
    	loadSystem("datalcp/twoStacksTest.png");

    	RigidBodySystem.enableMerging.setValue(true);
    	for (int i=0; i<122; i++)
    		system.advanceTime(dt);
    	
    	boolean hasCollection = false;
		for (RigidBody body: system.bodies)
			if (body instanceof RigidCollection)
				hasCollection = true;
		assertTrue(hasCollection);
    	
    	for (Contact contact : system.collisionProcessor.contacts) {
    		assertFalse(contact.body1 instanceof RigidCollection);
    		assertFalse(contact.body2 instanceof RigidCollection);
    	}
    }
	
    @Test
    public void mergingTest() {
		loadSystem("datalcp/twoStacksTest.png");
		
    	assertEquals(system.bodies.size(),6); // should have 6 bodies when loading the scene
    	
    	RigidBodySystem.enableMerging.setValue(true);
    	for (int i=0; i<450; i++)
    		system.advanceTime(dt);
    	assertEquals(system.bodies.size(),2); // with the rule that pinned objects should not been merged, we should get 2 bodies at the end of the simulation
    }
    
    @Test
    public void mergingAndSleepingTest() {  
    	
    	// TODO: This test failed after changing the unmerging rule to relative velocity check
    	// Need to be fixed
    	
    	assertTrue(false);
		/*loadSystem("datalcp/twoStacksTest.png");
		
    	for (RigidBody body: system.bodies)
			assertEquals(ObjectState.ACTIVE, body.state); // every bodies should be active at beginning of the simulation

    	RigidBodySystem.enableMerging.setValue(true);
    	RigidBodySystem.enableSleeping.setValue(true);
    	for (int i=0; i<400; i++)
    		system.advanceTime(dt);
    	for (RigidBody body: system.bodies)
    		if (body instanceof RigidCollection)
    			assertEquals(ObjectState.SLEEPING, body.state);*/ 
    }
    
    @Test
    public void temporarilyPinnedObjectTest() {
    	loadSystem("datalcp/twoStacksTest.png");
    	
    	for (int i=0; i<10; i++)
    		system.advanceTime(dt);
    	
    	RigidBody tempPinnedBody = null;
    	Vector2d zeroVelocity = new Vector2d(0.,0.);
    	for (RigidBody body: system.bodies)
    		if (body.temporarilyPinned) { 
    			tempPinnedBody = body;
    			assertEquals(zeroVelocity, tempPinnedBody.v); 
    		}
    	
    	for (int i=0; i<190; i++)
    		system.advanceTime(dt);

		assertNotEquals(null, tempPinnedBody); 
		assertNotEquals(zeroVelocity, tempPinnedBody.v); 
		assertFalse(tempPinnedBody.temporarilyPinned); 
    }
    
    @Test
    /**
     * Checks that the merging is working with a temporarily pinned object.
     */
    public void twoStackTest() {
		loadSystem("datalcp/twoStacksTest.png");
    	
    	for (int i=0; i<450; i++)
    		system.advanceTime(dt);
    	assertEquals(system.collisionProcessor.contacts.size(),2); 
    }
    
    @Test
    /**
     * Failing example with current pruning technique combined with update of contacts in collection
     */
    public void inactiveContactsTest() {
		loadSystem("datalcp/unstableStackTest.png");
    	
    	for (int i=0; i<200; i++)
    		system.advanceTime(dt);
    	assertEquals(system.bodies.size(),2); 
    }
}