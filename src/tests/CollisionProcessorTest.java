package tests;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.Test;

import mergingBodies2D.CollisionProcessor;
import mergingBodies2D.ImageBlocker;
import mergingBodies2D.RigidBody;

class CollisionProcessorTest extends CollisionProcessor {

	String defaultFile = "scenes2D/twoStacksTest.png";
	public List<RigidBody> defaultBodies;
	
	public CollisionProcessorTest() {
		load(defaultFile);
	}
	
	void load(String filename) {
		ImageBlocker blocker = new ImageBlocker(filename, (float)0.05);
		bodies = blocker.bodies;
	}
	
	void clear() {
		load(defaultFile);
		contacts.clear();
	}
	
	@Test
	void test() {
		assertNotEquals(null, bodies);
		assertNotEquals(0, bodies.size());
	}
	
	@Test
	void processCollisionsTest() {
		collisionDetection(0.05);
		assertEquals(0, contacts.size());

		for (RigidBody body: bodies) {
			if (body.temporarilyPinned == false && body.pinned == false) {
				body.x.x -= 3;
				body.x.y -= 3;
				body.updateTransformations();
			}
		}
		
		collisionDetection(0.05);
		assertNotEquals(0, contacts.size());
	}
	
	@Test
	void pruneTest() {
		assertEquals(0, contacts.size());
		
		pruneContacts.setValue(true);
		for (RigidBody body: bodies) {
			if (body.temporarilyPinned == false && body.pinned == false) {
				body.x.x -= 3;
				body.x.y -= 3;
				body.updateTransformations();
			}
		}
	
		collisionDetection(0.05);
		assertEquals(2, contacts.size());
		
		clear();
		
		assertEquals(0, contacts.size());
		
		pruneContacts.setValue(false);
		for (RigidBody body: bodies) {
			if (body.temporarilyPinned == false && body.pinned == false) {
				body.x.x -= 3;
				body.x.y -= 3;
				body.updateTransformations();
			}
		}
	
		collisionDetection(0.05);
		assertNotEquals(0, contacts.size());
		assertTrue(2 < contacts.size());
		clear();
	}
}
