package mergingBodies3D;

import java.util.ArrayList;
import java.util.HashSet;

import mergingBodies3D.Contact.ContactState;
import mergingBodies3D.Merging.MergeParameters;

/**
 * This class holds info about two colliding bodies (list of contacts, relative velocity, if they are merged...).
 * @author eulalie coevoet
 */

public class BodyPairContact {
	
	public RigidBody body1; 
	public RigidBody body2;
	
	public RigidBody getBody( int i ) { return i==0 ? body1 : body2; }
	
	public ArrayList<Contact> contactList = new ArrayList<Contact>();
	
	// Utils for cycle merge/unmerge condition.
	 /** merge condition : by using this tag we avoid checking the same cycle twice*/
	public boolean inCycle = false;
	/** store the bpc which are part of the cycle */
	public ArrayList<BodyPairContact> cycle = null; 
	public Color cycleColor = null;

	MotionMetricProcessor motionMetricProcessor = new MotionMetricProcessor();
	
	/** TODO: make these circular buffers for efficiency!
	 * That is the remove(0) is linear time, while a linked list would be constant time it would thrash the GC */
	public ArrayList<Double> motionMetricHist = new ArrayList<Double>();
	public ArrayList<Contact.ContactState> contactStateHist = new ArrayList<Contact.ContactState>();
		
	public boolean inCollection = false;
	
	/** Used for contact ordering, i.e., in CollisionProcessor.getOrganizedContacts() */
	public boolean checked = false;
	
	public BodyPairContact(RigidBody body1, RigidBody body2) {
		this.body1 = body1;
		this.body2 = body2;
	}

	/**
	 * Check if two bodies are in a list of BodyContact.
	 * @param body1
	 * @param body2
	 * @param bodyPairContacts
	 * @return the corresponding BodyContact object
	 */
	public static BodyPairContact checkExists(RigidBody body1, RigidBody body2, HashSet<BodyPairContact> bodyPairContacts) {
		for (BodyPairContact bpc: bodyPairContacts) {
			if ((bpc.body1.equals(body1) && bpc.body2.equals(body2))
				|| (bpc.body1.equals(body2) && bpc.body2.equals(body1))) {
				return bpc;
			}
		}
		return null;
	}

	/**
	 * Check if BodyPairContact is in a list of BodyPairContacts
	 * @param bodyPairContacts
	 * @return true of false
	 */
	public boolean isIn(ArrayList<BodyPairContact> bodyPairContacts) {
		
		for (BodyPairContact c : bodyPairContacts) {
			if (c.body1.equals(this.body1) && c.body2.equals(this.body2)) { 
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Accumulate criteria for merging
	 * @param contact
	 */
	public void accumulateForMerging(MergeParameters mergeParams, double dt) {

		RigidBody body1 = (this.body1.isInCollection())? this.body1.parent: this.body1;
		RigidBody body2 = (this.body2.isInCollection())? this.body2.parent: this.body2;
		
		if (body1.pinned)
			motionMetricHist.add(motionMetricProcessor.getMotionMetric(body2));
		else if (body2.pinned)
			motionMetricHist.add(motionMetricProcessor.getMotionMetric(body1));
		else if (mergeParams.metricPositionLevel.getValue())
			motionMetricHist.add(motionMetricProcessor.getMotionMetric(body1, body2, dt));
		else
			motionMetricHist.add(motionMetricProcessor.getMotionMetric(body1, body2));
		
		if (motionMetricHist.size() > mergeParams.stepAccumMerging.getValue())
			motionMetricHist.remove(0);
	
		ContactState state = ContactState.CLEAR;
		// If only have one contact, then our state can come from the only contact
		// If we have two contacts, then again, if one of the two is on edge, we could have spinning
		// HOWEVER... for 3 contacts, if we see 2 that are not on edge, that is good enough. 
		int notOnEdgeCount = 0;
		for (Contact contact : contactList)
			if (contact.state != ContactState.ONEDGE)
				notOnEdgeCount++;
		
		if (notOnEdgeCount == 0) // Note: this call cannot be made on an empty bodyPairContact
			state = ContactState.ONEDGE;
		else if (notOnEdgeCount >= 2) // even if we only have 2, consider it good, as the cycle detection's job will do 3 point stability.
			state = ContactState.CLEAR;
		else if (contactList.size() == 1) 
			state = ContactState.CLEAR;
		else // we get here by having only 1 notOnEdge, and 2 or more contacts... this is a onEdge case
			state = ContactState.ONEDGE;
		
		contactStateHist.add(state);
		if (contactStateHist.size() > mergeParams.stepAccumMerging.getValue())
			contactStateHist.remove(0);
	}
	
	/**
	 * Accumulate motion metric for unmerging
	 * @param contact
	 */
	public void accumulateForUnmerging(MergeParameters mergeParams, double dt) {
		if(body1.isInSameCollection(body2)) {			
			double metric;
			if (body1.pinned)
				metric = motionMetricProcessor.getMotionMetric(body2);
			else if (body2.pinned)
				metric = motionMetricProcessor.getMotionMetric(body1);
			else if (mergeParams.metricPositionLevel.getValue())
				metric = motionMetricProcessor.getMotionMetric(body1, body2, dt);
			else
				metric = motionMetricProcessor.getMotionMetric(body1, body2);
			
			if (metric>mergeParams.thresholdUnmerge.getValue()) {
				motionMetricHist.add(metric);
			} else {
				motionMetricHist.clear();
			}
		}
	}
	
	/**
	 * Check merge condition
	 * @return
	 */
	protected boolean checkMergeCondition(MergeParameters mergeParams, boolean checkCycle) {
		
		if (body1.sleeping && body2.sleeping) 
			return true;

		if (mergeParams.enableMergeLetItBreathe.getValue())
			for (Contact contact: contactList)
				if (Math.abs(contact.prevConstraintViolation - contact.constraintViolation)>mergeParams.thresholdBreath.getValue()) 
					return false;
		
		if (!mergeParams.enableMergePinned.getValue() && (body1.pinned || body2.pinned)) 
			return false;
		
		if (body1.isInSameCollection(body2)) 
			return false;
		
		if (!checkMotionMetricForMerging(mergeParams))
			return false;

		if (mergeParams.enableMergeStableContactCondition.getValue() && !areContactsStable(mergeParams))
			return false;
		
		if (mergeParams.enableMergeCycleCondition.getValue() && checkCycle && !checkContactsCycle(mergeParams))
			return false;
			
		return true;
	}
	
	/**
	 * Check if:
	 * <p><ul> 
	 * <li> 1. the two bodies have been in contact for at least "stepAccum" number of time steps
	 * <li> 2. and lower than a threshold over "stepAccum" time steps.
	 * </ul><p>
	 * @return true or false
	 */
	public boolean checkMotionMetricForMerging(MergeParameters mergeParams) {

		if ((motionMetricHist.size() == mergeParams.stepAccumMerging.getValue())) {
			for (Double metric : motionMetricHist) {
				if (metric > mergeParams.thresholdMerge.getValue())
					return false;
			}
		} else {
			return false;
		}
		return true;
	}  
	
	public boolean checkMotionMetricForUnmerging(MergeParameters mergeParams) {
		if ((motionMetricHist.size() >= mergeParams.stepAccumUnmerging.getValue()) && contactList.size()<3) // only reason for unstable config
			return true;
		
		return false;
	}

	/**
	 * Check if contacts have been stable over CollisionProcessor.sleepAccum time steps 
	 * (i.e not on edge of friction cone during the entire time)
	 * @return true or false
	 */
	public boolean areContactsStable(MergeParameters mergeParams) {

		if ((contactStateHist.size() == mergeParams.stepAccumMerging.getValue())) {
			for (Contact.ContactState state : contactStateHist) {
				if (state == Contact.ContactState.ONEDGE)
					return false;
			}
		} else {
			return false;
		}
		
		return true;
	}
	
	/**
	 * Check if it satisfies the cycle criteria: only bodies that share two
	 * contacts, or cycles formed by three bodies with one contact between each.
	 * @return true if the criteria is satisfied
	 */
	public boolean checkContactsCycle(MergeParameters mergeParams) {	
		int nbActiveContact = 0;
		for (Contact contact : contactList)
			if (contact.state != ContactState.BROKEN)
				nbActiveContact += 1;
		
		// if there are more than two active contact within the bpc, we can merge
		if (nbActiveContact>2) 
			return true;

		// if the bpc has already been identified as being part of a cycle, we can merge
		if(inCycle) 
			return true;
		
		// otherwise check if this bpc is in a cycle formed by three bodies with one contact between each		
		return body1.checkCycle(1, body2, this, mergeParams); 
	}
	
	/**
	 * Check the contacts' state for unmerge
	 * @param body
	 * @param dt
	 * @return true if should unmerge
	 */
	public boolean checkContactsState(double dt, MergeParameters mergeParams) {		
				
		for (Contact contact : contactList) { 
			if (contact.state == ContactState.BROKEN && mergeParams.enableUnmergeNormalCondition.getValue()) 
				return true; // rule 1. if one contact has broken
			
			if (contact.state == ContactState.ONEDGE && mergeParams.enableUnmergeFrictionCondition.getValue()) 
				return true; // rule 2. if one contact is on the edge of friction cone
		}

		return false;
	}
	
	/**
	 * Given a RigidBody sB in a BodyPairContact, return the adjacent RigidBody in the BodyPairContact
	 * @param body
	 * @return
	 */
	public RigidBody getOtherBody(RigidBody body) {
		if (body1 == body) return body2;
		if (body2 == body) return body1;
		return null;
	}
	
	/**
	 * Given a RigidBody or RigidCollection sB in a BodyPairContact, return the adjacent RigidBody in the BodyPairContact
	 * @param body
	 * @return
	 */
	public RigidBody getOtherBodyWithCollectionPerspective(RigidBody body) {
		if (body.isInCollection()) 
			return (body1.isInSameCollection(body))? body2 : body1;
		else 
			return getOtherBody(body);
	}
	
	/**
	 * Add the BodyContact to bodyContactList of members body1 and body2  
	 */
	public void addBpcToUnmerge(HashSet<BodyPairContact> bpcsToUnmerge) {
		if(!body1.isInCollection() || !body2.isInCollection() || !body1.isInSameCollection(body2)) {
			System.err.println("[addBodiesToUnmerge] trying to unmerge a body that is not in a collection. This shouldn't happen?");
			return;
		}

		if(!bpcsToUnmerge.contains(this))
			bpcsToUnmerge.add(this);
	}

	/**
	 * Add the BodyContact to bodyContactList of members body1 and body2  
	 */
	public void addToBodyLists() {
		if (!body1.bodyPairContacts.contains(this)) {
			body1.bodyPairContacts.add(this);
		}
		if (!body2.bodyPairContacts.contains(this)) {
			body2.bodyPairContacts.add(this);
		}
	}
	
	/**
	 * Add the BodyContact to bodyContactList of parent members body1 and body2  
	 */
	public void addToBodyListsParent() {		
		if (body1.isInCollection() && !body1.parent.bodyPairContacts.contains(this)) {
			body1.parent.bodyPairContacts.add(this);
		}
		if (body2.isInCollection() && !body2.parent.bodyPairContacts.contains(this)) {
			body2.parent.bodyPairContacts.add(this);
		}
	}
	
	/**
	 * Remove the BodyContact from bodyContactList of members body1 and body2
	 */
	public void removeFromBodyLists() {
		if (body1.bodyPairContacts.contains(this))
			body1.bodyPairContacts.remove(this);
		if (body2.bodyPairContacts.contains(this))
			body2.bodyPairContacts.remove(this);
	}
	
	/**
	 * Remove the BodyContact from bodyContactList of parent of members body1 and body2
	 */
	public void removeFromBodyListsParent() {
		if (body1.isInCollection() && body1.parent.bodyPairContacts.contains(this)) {
			body1.parent.bodyPairContacts.remove(this);
		}
		if (body2.isInCollection() && body2.parent.bodyPairContacts.contains(this)) {
			body2.parent.bodyPairContacts.remove(this);
		}
	}
	
	/**
	 * Clear datas related to the cycle, and tag the other bpc in the cycle with input unmerge value 
	 * @param unmerge
	 */
	public void clearCycle() {
		if (cycle != null) {
			for (BodyPairContact bpc : cycle) {	
				if (bpc != this) {
					bpc.inCycle = false;
					bpc.cycleColor = null;
					bpc.cycle.clear();
				}
			}
			inCycle = false;
			cycleColor = null;
			cycle.clear();
		}
	}
	
	private static HashSet<BodyPairContact> bpcToCheck = new HashSet<BodyPairContact>();

	/**
	 * Uses static internal memory to do the job
	 * @param bpcsToUnmerge
	 */
	public void checkCyclesToUnmerge(HashSet<BodyPairContact> bpcsToUnmerge) {

		for (int i=0; i<2; i++) { 
			RigidBody body = getBody(i);
			for (BodyPairContact bpc: body.bodyPairContacts) { // check if body was part of a cycle
				if (bpc.inCycle) {
					bpc.addBpcToUnmerge(bpcsToUnmerge);
					bpcToCheck.add(bpc);
					 for (BodyPairContact bpcToUnmerge : bpc.cycle) { // unmerge the others bodies
						bpcToUnmerge.addBpcToUnmerge(bpcsToUnmerge);
						bpcToCheck.add(bpcToUnmerge);
					 }
					bpc.clearCycle();
				}
			}
		}
		
		for (BodyPairContact bpc: bpcToCheck)
			bpc.checkCyclesToUnmerge(bpcsToUnmerge);
	}
		
}
