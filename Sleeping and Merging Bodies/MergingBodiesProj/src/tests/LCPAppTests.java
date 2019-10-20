package tests;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import mergingBodies.LCPApp;
import mergingBodies.RigidBody;
import mergingBodies.RigidBody.ObjectState;
import mergingBodies.RigidBodySystem;
import mergingBodies.RigidCollection;
import mergingBodies.CollisionProcessor;
import mergingBodies.Contact;
import javax.vecmath.Vector2d;

public class LCPAppTests extends LCPApp {

	double dt = 0.05;

	@Override
	public void setUp() {
		// Do nothing, we don't want to launch the app (2D viewer)
	}

	@Test
	/**
	 * Simple test for merge condition
	 */
	public void simpleMerging() {
		loadSystem("datalcp/twoStacksTest.png");

		assertEquals(6, system.bodies.size()); // should have 6 bodies when loading the scene

		system.enableMerging.setValue(true);
		for (int i=0; i<system.tempSleepCount.getValue()*2; i++)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size());
	}

	//@Test
	/**
	 * Simple test for sleeping condition 
	 * With the new rule that pinned body can be merged: this test is no longer relevant as pinned body are not put to SLEEPING mode
	 */
	public void mergingAndSleeping() {  

		loadSystem("datalcp/twoStacksTest.png");

		for (RigidBody body: system.bodies)
			assertEquals(ObjectState.ACTIVE, body.state); // every bodies should be active at beginning of the simulation

		system.enableMerging.setValue(true);
		RigidBodySystem.enableSleeping.setValue(true);
		for (int i=0; i<system.tempSleepCount.getValue()*2; i++)
			system.advanceTime(dt);
		for (RigidBody body: system.bodies)
			if (body instanceof RigidCollection)
				assertEquals(ObjectState.SLEEPING, body.state);
	}

	@Test
	/**
	 * Test the behavior of temporarily pinned bodies
	 */
	public void tempPinnedObject() {
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

		for (int i=0; i<system.tempSleepCount.getValue(); i++)
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
		loadSystem("datalcp/twoStacksTest.png");

		system.enableMerging.setValue(true);
		system.enableMergePinned.setValue(true);
		for (int i=0; i<system.tempSleepCount.getValue()*2; i++)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size()); 
	}

	@Test
	/**
	 * Simple test for one iteration PGS in collection
	 */
	public void updateContactInCollection() {
		loadSystem("datalcp/twoStacksTest.png");

		system.enableMerging.setValue(true);
		system.enableUnmerging.setValue(false);
		system.enableUpdateContactsInCollections.setValue(true);
		for (int i=0; i<200; i++)
			system.advanceTime(dt);

		RigidCollection collection = (RigidCollection)system.bodies.get(1);
		Contact contact = collection.getInternalContacts().get(0);
		Vector2d lambda = new Vector2d(contact.getLambda());

		for (int i=0; i<system.tempSleepCount.getValue(); i++)
			system.advanceTime(dt);

		assertTrue(lambda.x < contact.getLambda().x);
	}

	@Test
	/**
	 * Critical test for one iteration PGS in collection
	 * After merging, if the bodies are stable/static, the internal contacts should remain the same
	 * failing : (eulalie) since the use of compliance and stiffness feedback... 
	 */
	public void updateContactInCollectionConsistency() {
		loadSystem("datalcp/doubleStackUnmergeTest.png");

		system.enableMerging.setValue(true);
		system.enableUnmerging.setValue(false);
		system.enableUpdateContactsInCollections.setValue(true);
		system.enableMergePinned.setValue(true);
		for (int i=0; i<40+CollisionProcessor.sleepAccum.getValue(); i++)
			system.advanceTime(dt);

		RigidCollection collection = (RigidCollection)system.bodies.get(0);
		Contact contact = collection.getInternalContacts().get(0);
		Vector2d lambda = new Vector2d(contact.getLambda());

		system.advanceTime(dt);

		assertTrue(Math.abs(lambda.x - contact.getLambda().x) < 1e-14); 
		assertTrue(Math.abs(lambda.y - contact.getLambda().y) < 1e-14); 
	}

	@Test
	/**
	 * Critical test for one iteration PGS in collection
	 * After merging, if the bodies are stable/static, the internal contacts should remain the same 
	 * failing : (eulalie) since the use of compliance and stiffness feedback... 
	 */
	public void updateContactInCollectionConsistencyPinned() {
		loadSystem("datalcp/singleBlockSmallTest.png");

		system.enableMerging.setValue(true);
		system.enableUpdateContactsInCollections.setValue(true);
		system.enableMergePinned.setValue(true);
		for (int i=0; i<23+CollisionProcessor.sleepAccum.getValue(); i++)
			system.advanceTime(dt);

		RigidCollection collection = (RigidCollection)system.bodies.get(0);
		Contact contact = collection.getInternalContacts().get(0);
		Vector2d lambda = new Vector2d(contact.getLambda());

		system.advanceTime(dt);

		assertTrue(Math.abs(lambda.x - contact.getLambda().x) < 1e-14); 
		assertTrue(Math.abs(lambda.y - contact.getLambda().y) < 1e-14); 
	}

	@Test
	/**
	 * Critical test : this test will fail if the external contacts are not updated correctly
	 */
	public void externalContacts() {
		loadSystem("datalcp/unstableStackTest.png");

		system.enableMerging.setValue(true);
		system.enableUnmerging.setValue(true);
		system.enableUpdateContactsInCollections.setValue(true);
		system.enableMergePinned.setValue(true);
		for (int i=0; i<400; i++)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size()); 
	}

	@Test
	/**
	 * Critical test for multiple bodies merge during single time step
	 */
	public void mergeMultipleBodiesSingleStep() {
		loadSystem("datalcp/jamTest.png");

		system.enableMerging.setValue(false);
		for (int i=0; i<100; i++)
			system.advanceTime(dt);
		assertEquals(4, system.bodies.size()); 

		system.enableMerging.setValue(true);
		system.enableUnmerging.setValue(false);
		system.enableMergeCheckCycleCondition.setValue(false);
		for (int i=0; i<CollisionProcessor.sleepAccum.getValue(); i++)
			system.advanceTime(dt);
		assertEquals(1, system.bodies.size()); 
	}
	
	@Test
	/**
	 * Test collisionProcessor bodyPairContact list
	 */
	public void storeBodyPairContactInCollisionProcessor() {
		loadSystem("datalcp/doubleStackUnmergeTest.png");
		
		system.enableMerging.setValue(false);
		
		for (int i=0; i<23; i++)
			system.advanceTime(dt);
		
		assertEquals(4, system.collisionProcessor.contacts.size()); 
		system.collisionProcessor.processBodyPairContacts();
		assertEquals(2, system.collisionProcessor.bodyPairContacts.size()); 
		
		for (int i=0; i<9; i++)
			system.advanceTime(dt);

		system.collisionProcessor.processBodyPairContacts();
		assertEquals(4, system.collisionProcessor.bodyPairContacts.size()); 
		
		for (int i=0; i<7; i++)
			system.advanceTime(dt);

		system.collisionProcessor.processBodyPairContacts();
		assertEquals(6, system.collisionProcessor.bodyPairContacts.size()); 
	}
	
	@Test
	/**
	 * Test that the bodyPairContact list of collection is correctly being updated
	 */
	public void storeBodyPairContactInCollection() {
		loadSystem("datalcp/doubleStackUnmergeTest.png");
		RigidCollection collection;
		
		system.enableMerging.setValue(true);
		system.enableMergePinned.setValue(false);
		
		system.mergingEvent=false;
		while(!system.mergingEvent)
			system.advanceTime(dt);
		
		collection = (RigidCollection)system.bodies.get(2);		
		assertEquals(3, collection.bodyPairContactList.size()); 
		
		system.mergingEvent=false;
		while(!system.mergingEvent)
			system.advanceTime(dt);
		
		collection = (RigidCollection)system.bodies.get(1);		
		assertEquals(6, collection.bodyPairContactList.size()); 
	}
}













