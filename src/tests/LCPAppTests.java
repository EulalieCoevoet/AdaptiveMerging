package tests;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import mergingBodies2D.Contact;
import mergingBodies2D.LCPApp2D;
import mergingBodies2D.RigidBody;
import mergingBodies2D.RigidBodySystem;
import mergingBodies2D.RigidCollection;
import mergingBodies2D.Merging.MergeParameters;
import mergingBodies2D.Sleeping.SleepParameters;

import javax.vecmath.Vector2d;

public class LCPAppTests extends LCPApp2D {

	double dt = 0.05;
	MergeParameters mergeParams;
	SleepParameters sleepParams;

	@Override
	public void setUp() {
		// Do nothing, we don't want to launch the app (2D viewer)
		mergeParams = system.merging.params;
		sleepParams = system.sleeping.params;
	}

	@Test
	/**
	 * Simple test for merge condition
	 */
	public void simpleMerging() {
		loadSystem("scenes2D/twoStacksTest.png");

		assertEquals(6, system.bodies.size()); // should have 6 bodies when loading the scene

		mergeParams.enableMerging.setValue(true);
		mergeParams.enableMergePinned.setValue(false);
		mergeParams.enableMergeLetItBreathe.setValue(false);
		for (int i=0; i<RigidBodySystem.tempSleepCount.getValue()*2; i++)
			system.advanceTime(dt);
		assertEquals(2, system.bodies.size());
		
		mergeParams.enableMergePinned.setValue(true);
		for (int i=0; i<RigidBodySystem.tempSleepCount.getValue()*2; i++)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size());
	}

	//@Test
	/**
	 * Simple test for sleeping condition 
	 * With the new rule that pinned body can be merged: this test is no longer relevant as pinned body are not put to SLEEPING mode
	 */
	public void mergingAndSleeping() {  

		loadSystem("scenes2D/twoStacksTest.png");

		for (RigidBody body: system.bodies)
			assertEquals(false, body.isSleeping); // every bodies should be active at beginning of the simulation

		mergeParams.enableMerging.setValue(true);
		sleepParams.enableSleeping.setValue(true);
		for (int i=0; i<RigidBodySystem.tempSleepCount.getValue()*2; i++)
			system.advanceTime(dt);
		for (RigidBody body: system.bodies)
			if (body instanceof RigidCollection)
				assertEquals(true, body.isSleeping);
	}

	@Test
	/**
	 * Test the behavior of temporarily pinned bodies
	 */
	public void tempPinnedObject() {
		loadSystem("scenes2D/twoStacksTest.png");
		
		for (int i=0; i<10; i++)
			system.advanceTime(dt);

		RigidBody tempPinnedBody = null;
		Vector2d zeroVelocity = new Vector2d(0.,0.);
		for (RigidBody body: system.bodies)
			if (body.temporarilyPinned) { 
				tempPinnedBody = body;
				assertEquals(zeroVelocity, tempPinnedBody.v); 
			}

		for (int i=0; i<RigidBodySystem.tempSleepCount.getValue(); i++)
			system.advanceTime(dt);

		assertNotEquals(null, tempPinnedBody); 
		assertNotEquals(zeroVelocity, tempPinnedBody.v); 
		assertFalse(tempPinnedBody.temporarilyPinned); 
	}

	@Test
	/**
	 * Checks that the merging is working with a temporarily pinned body.
	 */
	public void mergingWithTempPinnedObject() {
		loadSystem("scenes2D/twoStacksTest.png");

		mergeParams.enableMerging.setValue(true);
		mergeParams.enableMergePinned.setValue(true);
		mergeParams.enableMergeLetItBreathe.setValue(false);
		for (int i=0; i<RigidBodySystem.tempSleepCount.getValue()*2; i++)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size()); 
	}

	@Test
	/**
	 * Simple test for one iteration PGS in collection
	 */
	public void updateContactInCollection() {
		loadSystem("scenes2D/twoStacksTest.png");

		mergeParams.enableMerging.setValue(true);
		mergeParams.enableUnmerging.setValue(false);
		mergeParams.updateContactsInCollections.setValue(true);
		mergeParams.enableMergePinned.setValue(true);
		while (!system.merging.mergingEvent) 
			system.advanceTime(dt);

		RigidCollection collection = null;
		for (RigidBody body : system.bodies)
			if (body instanceof RigidCollection) {
				collection = (RigidCollection)body;
				break;
			}

		assertTrue(collection != null); 
		
		Contact contact = collection.getInternalContacts().get(0);
		Vector2d lambda = new Vector2d(contact.getLambda());

		for (int i=0; i<RigidBodySystem.tempSleepCount.getValue(); i++)
			system.advanceTime(dt);

		assertTrue(lambda.x < contact.getLambda().x);
	}

	@Test
	/**
	 * Critical test for one iteration PGS in collection
	 * After merging, if the bodies are stable/static, the internal contacts should remain the same
	 */
	public void updateContactInCollectionConsistency() {
		loadSystem("scenes2D/doubleStackUnmergeTest.png");

		mergeParams.enableMerging.setValue(true);
		mergeParams.enableUnmerging.setValue(false);
		mergeParams.updateContactsInCollections.setValue(true);
		mergeParams.enableMergeLetItBreathe.setValue(true);
		mergeParams.enableMergePinned.setValue(true);
		while (!system.merging.mergingEvent) 
			system.advanceTime(dt);

		RigidCollection collection = null;
		for (RigidBody body : system.bodies)
			if (body instanceof RigidCollection) {
				collection = (RigidCollection)body;
				break;
			}
				
		assertTrue(collection != null); 
		
		Contact contact = collection.getInternalContacts().get(0);
		Vector2d lambda = new Vector2d(contact.getLambda());

		system.advanceTime(dt);

		assertTrue(Math.abs(lambda.x - contact.getLambda().x) < 1e-3); 
		assertTrue(Math.abs(lambda.y - contact.getLambda().y) < 1e-3); 
	}

	@Test
	/**
	 * Critical test for one iteration PGS in collection
	 * After merging, if the bodies are stable/static, the internal contacts should remain the same 
	 */
	public void updateContactInCollectionConsistencyPinned() {
		loadSystem("scenes2D/singleBlockSmallTest.png");

		mergeParams.enableMerging.setValue(true);
		mergeParams.updateContactsInCollections.setValue(true);
		mergeParams.enableMergeLetItBreathe.setValue(true);
		mergeParams.enableMergePinned.setValue(true);
		while (!system.merging.mergingEvent) 
			system.advanceTime(dt);

		RigidCollection collection = null;
		for (RigidBody body : system.bodies)
			if (body instanceof RigidCollection) {
				collection = (RigidCollection)body;
				break;
			}
				
		assertTrue(collection != null); 
		Contact contact = collection.getInternalContacts().get(0);
		Vector2d lambda = new Vector2d(contact.getLambda());

		system.advanceTime(dt);

		assertTrue(Math.abs(lambda.x - contact.getLambda().x) < 1e-3); 
		assertTrue(Math.abs(lambda.y - contact.getLambda().y) < 1e-3); 
	}

	@Test
	/**
	 * Critical test : this test will fail if the external contacts are not updated correctly
	 */
	public void externalContacts() {
		loadSystem("scenes2D/unstableStackTest.png");

		mergeParams.enableMerging.setValue(true);
		mergeParams.enableUnmerging.setValue(true);
		mergeParams.updateContactsInCollections.setValue(true);
		mergeParams.enableMergePinned.setValue(true);
		for (int i=0; i<400; i++)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size()); 
	}

	@Test
	/**
	 * Critical test for multiple bodies merge during single time step
	 */
	public void mergeMultipleBodiesSingleStep() {
		loadSystem("scenes2D/jamTest.png");

		mergeParams.enableMerging.setValue(false);
		for (int i=0; i<400; i++)
			system.advanceTime(dt);
		assertEquals(4, system.bodies.size()); 

		mergeParams.enableMerging.setValue(true);
		mergeParams.enableUnmerging.setValue(false);
		mergeParams.enableMergePinned.setValue(true);
		mergeParams.enableMergeCycleCondition.setValue(false);
		while(!system.merging.mergingEvent)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size()); 
	}
	
	@Test
	/**
	 * Test collisionProcessor bodyPairContact list
	 */
	public void storeBodyPairContactInCollisionProcessor() {
		loadSystem("scenes2D/doubleStackUnmergeTest.png");
		
		mergeParams.enableMerging.setValue(false);
		
		for (int i=0; i<23; i++)
			system.advanceTime(dt);
		
		assertEquals(4, system.collision.contacts.size()); 
		system.collision.updateBodyPairContacts();
		system.collision.clearBodyPairContacts();
		assertEquals(2, system.collision.bodyPairContacts.size()); 
		
		for (int i=0; i<9; i++)
			system.advanceTime(dt);

		system.collision.updateBodyPairContacts();
		system.collision.clearBodyPairContacts();
		assertEquals(4, system.collision.bodyPairContacts.size()); 
		
		for (int i=0; i<7; i++)
			system.advanceTime(dt);

		system.collision.updateBodyPairContacts();
		system.collision.clearBodyPairContacts();
		assertEquals(6, system.collision.bodyPairContacts.size()); 
	}
	
	@Test
	/**
	 * Test that the bodyPairContact list of collection is correctly being updated
	 */
	public void storeBodyPairContactInCollection() {
		loadSystem("scenes2D/doubleStackUnmergeTest.png");
		
		mergeParams.enableMerging.setValue(true);
		mergeParams.enableMergePinned.setValue(false);
		
		while(!system.merging.mergingEvent)
			system.advanceTime(dt);
		
		RigidCollection collection = null;
		for (RigidBody body : system.bodies)
			if (body instanceof RigidCollection) {
				collection = (RigidCollection)body;
				break;
			}
				
		assertTrue(collection != null); 		
		assertEquals(3, collection.bodyPairContacts.size()); 
	}
}













