package mergingBodies3D;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.LinkedList;

import javax.swing.JButton;
import javax.swing.JPanel;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

public class Merging {
	
	public boolean mergingEvent = false;
	public boolean triggerMergingEvent = false;

	public enum MergeConditions {RELATIVEMOTION, CONTACTS;}
	
	protected ArrayList<RigidBody> bodies;
	protected CollisionProcessor collision;
	
	public class MergeParameters {
		public MergeParameters() {}
		
		public BooleanParameter enableMerging = new BooleanParameter( "merging", true);
		public BooleanParameter enableMergePinned = new BooleanParameter( "merging - pinned body", true);
		public BooleanParameter enableMergeCycleCondition = new BooleanParameter( "merging - check cycle condition", false);
		public BooleanParameter enableMergeStableContactCondition = new BooleanParameter( "merging - stable contact condition", true);
		public BooleanParameter enableMergeLetItBreathe = new BooleanParameter( "merging - let it breathe condition", true);
		public BooleanParameter enableUnmerging = new BooleanParameter( "unmerging", true );
		public BooleanParameter enableUnmergeFrictionCondition = new BooleanParameter( "unmerging - friction condition", true);
		public BooleanParameter enableUnmergeNormalCondition = new BooleanParameter( "unmerging - contact normal condition", true);
		public BooleanParameter enableUnmergeRelativeMotionCondition = new BooleanParameter( "unmerging - relative motion condition", false);
		public BooleanParameter updateContactsInCollections = new BooleanParameter( "update contact in collection", true);
		public BooleanParameter organizeContacts = new BooleanParameter( "organize contacts (FIXE ME)", false);
		public IntParameter stepAccum = new IntParameter("check threshold over N number of time steps", 10, 0, 200 );
		public DoubleParameter thresholdMerge = new DoubleParameter("merging threshold", 1e-3, 1e-10, 100 );
		public DoubleParameter thresholdUnmerge = new DoubleParameter("unmerging threshold", 10, 1e-10, 100 );
		public DoubleParameter thresholdBreath = new DoubleParameter("breathing threshold", 1e-5, 1e-10, 1e0 );
		public BooleanParameter unmergeAll = new BooleanParameter("unmerge all", false);
	}
	public MergeParameters params = new MergeParameters();
	
	Merging(ArrayList<RigidBody> bodies, CollisionProcessor CP) {
		this.bodies = bodies;
		this.collision = CP;
	}

	/**
	 * Merges all rigidBodies in the system that fit the appropriate criteria: 
	 * <p><ul>
	 * <li> 1. They have been in contact for at least "mergingAccum" number of time steps
	 * <li> 2. The "metric" of the two bodies in contact has been below the "mergingThreshold"
	 * 	  value for the ENTIRETY of the contact.
	 * <li> 3. The contacts have been stable for "mergingAccum" number of time steps
	 * <li> 4. Satisfies the conservative force closure: only bodies that share two
     * contacts, or cycles formed by 3 bodies with one contact between each
	 * </ul><p>
	 */
	public void merge() {
		
		if (!params.enableMerging.getValue())
			return;
			
		LinkedList<BodyPairContact> removalQueue = new LinkedList<BodyPairContact>();

		for (BodyPairContact bpc : collision.bodyPairContacts) {
			
			if (!bpc.inCollection && bpc.checkMergeCondition(params, true)) {
				mergingEvent = true;
				bpc.inCollection = true;
				
				// the body pair contact is now going to be internal, so let's migrate it's contacts over to
				// a more permanent list... for now allocate new memory, but it could be pooled.
				// It seems that the body pair contacts should persist at the subbody level, while
				// anything in the parent collection's BPC list should only involve external contacts.
				// This would be logical.
				
				// The complication is with BPC that are between bodies and collections when those bodies
				// merge, because the contact was previously with the parent and needs to be updated to be
				// with the subbody. (correction: contacts are always between bodies, no collection involved)
				
				removalQueue.add(bpc); // bpc in now exclusively part of the collection
				if(!bpc.body1.isInCollection() && !bpc.body2.isInCollection()) {
					//both are not collections: make a new collection
					bodies.remove(bpc.body1); 
					bodies.remove(bpc.body2);
					RigidCollection collection = new RigidCollection(bpc.body1, bpc.body2);
					collection.addToInternalContact(bpc);
					collection.addBPCsToCollection(bpc);
					bodies.add(collection);
				} else if (bpc.body1.isInCollection() && bpc.body2.isInCollection() ) {
					if ( bpc.body1.isInSameCollection(bpc.body2)) {
						System.err.println("merge() had bodies from the same collection merging");
					}
					//both are collections: take the smaller number of bodies and add them to the other collection
					if (bpc.body1.parent.bodies.size() > bpc.body2.parent.bodies.size()) {
						bodies.remove(bpc.body2.parent);
						for (BodyPairContact bpccollection : bpc.body2.parent.bodyPairContacts)
							if(!bpc.body1.parent.bodyPairContacts.contains(bpccollection))
								bpc.body1.parent.bodyPairContacts.add(bpccollection);
						bpc.body1.parent.internalContacts.addAll(bpc.body2.parent.internalContacts);
						bpc.body1.parent.addCollection(bpc.body2.parent);
						bpc.body1.parent.addToInternalContact(bpc);
						bpc.body1.parent.addIncompleteCollectionContacts(bpc.body2.parent, removalQueue);
						bpc.body1.parent.addBPCsToCollection(bpc);
					} else {
						bodies.remove(bpc.body1.parent);
						for (BodyPairContact bpccollection : bpc.body1.parent.bodyPairContacts)
							if(!bpc.body2.parent.bodyPairContacts.contains(bpccollection))
								bpc.body2.parent.bodyPairContacts.add(bpccollection);
						bpc.body2.parent.internalContacts.addAll(bpc.body1.parent.internalContacts);
						bpc.body2.parent.addCollection(bpc.body1.parent);
						bpc.body2.parent.addToInternalContact(bpc);
						bpc.body2.parent.addIncompleteCollectionContacts(bpc.body1.parent, removalQueue);
						bpc.body2.parent.addBPCsToCollection(bpc);
					}
				}
				else if (bpc.body1.isInCollection()) {
					//body1 is in a collection, body2 is not
					bodies.remove(bpc.body2);
					bpc.body1.parent.addBody(bpc.body2);
					bpc.body1.parent.addToInternalContact(bpc);
					bpc.body1.parent.addIncompleteContacts(bpc.body2, removalQueue);
					bpc.body1.parent.addBPCsToCollection(bpc);
				}
				else if (bpc.body2.isInCollection()) {
					//body2 is in a collection, body1 is not
					bodies.remove(bpc.body1);
					bpc.body2.parent.addBody(bpc.body1);
					bpc.body2.parent.addToInternalContact(bpc);
					bpc.body2.parent.addIncompleteContacts(bpc.body1, removalQueue);
					bpc.body2.parent.addBPCsToCollection(bpc);
				}
			}
		}
		
		collision.bodyPairContacts.removeAll(removalQueue);
	}

	/**
	 * Unmerge all bodies
	 */
	public void unmergeAll() {
		
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
		LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
		
		for(RigidBody body : bodies) {
			
			if (body instanceof RigidCollection) {
				
				RigidCollection collection = (RigidCollection) body;
				removalQueue.add(collection);
				
				for (BodyPairContact bpc: collection.bodyPairContacts)
					unmergeBodyPairContact(bpc);
							
				for (RigidBody b: collection.bodies) {
					collection.unmergeBody(b);
					additionQueue.add(b);
				}
			}
		}

		bodies.removeAll(removalQueue);
		bodies.addAll(additionQueue);
		
		params.unmergeAll.setValue(false);
	}
	
	/**
	 * Unmerge BodyPairContacts that satisfy condition
	 */
	public void unmerge(MergeConditions condition, double dt) {
		
		if (!params.enableUnmerging.getValue())
			return;
		
		if (condition == MergeConditions.RELATIVEMOTION && !params.enableUnmergeRelativeMotionCondition.getValue())
			return;
		
		if (condition == MergeConditions.CONTACTS && !(params.enableUnmergeNormalCondition.getValue() || params.enableUnmergeFrictionCondition.getValue()))
			return;
		
		LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
		LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
		
		for(RigidBody body : bodies) {
			
			if(body.isSleeping )//|| body.temporarilyPinned)
				continue;
			
			if (body instanceof RigidCollection) {
				
				RigidCollection collection = (RigidCollection) body;
				ArrayList<BodyPairContact> bpcsToUnmerge = new ArrayList<BodyPairContact>();
				
				for (BodyPairContact bpc: collection.bodyPairContacts) {
					if (!bpc.inCollection)
						continue;
					
					for (int i=0; i<2; i++) { 
						RigidBody b = bpc.getBody(i);
						if (!bpcsToUnmerge.contains(bpc)) {
							
							if (condition == MergeConditions.CONTACTS && !bpc.checkContactsState(dt, params))
								continue;
							
							if (condition == MergeConditions.RELATIVEMOTION && !collection.isMovingAway(b, params))
								continue;
							
							bpc.checkCyclesToUnmerge(bpcsToUnmerge);
							bpc.addBpcToUnmerge(bpcsToUnmerge);
						}
					}
				}
				
				ArrayList<RigidBody> newBodies = new ArrayList<RigidBody>();
				if (!bpcsToUnmerge.isEmpty()) 
					unmergeSelectedBpcs(collection, bpcsToUnmerge, newBodies, dt);	

				if (!newBodies.isEmpty()) {
					additionQueue.addAll(newBodies);
					removalQueue.add(collection);
					if (newBodies.size()>1)
						mergingEvent = true;
				}
			}
		}
		
		bodies.addAll(additionQueue);
		bodies.removeAll(removalQueue);

		processCollectionsColor(bodies);
	}

	private void unmergeSelectedBpcs(RigidCollection collection, ArrayList<BodyPairContact> bpcsToUnmerge, ArrayList<RigidBody> newBodies, double dt) {
			
		// Check for unstable configurations
		ArrayList<BodyPairContact> unstableBpcsToUnmerge = new ArrayList<BodyPairContact>();
		ArrayList<BodyPairContact> bpcs = new ArrayList<BodyPairContact>();
		for (BodyPairContact bpc : bpcsToUnmerge) {
			for (int i=0; i<2; i++) { 
				RigidBody body = bpc.getBody(i);
				bpcs.clear();
				for (BodyPairContact newBpc : body.bodyPairContacts) 
					if (newBpc.contactList.size()<2 && newBpc.inCollection && !bpcsToUnmerge.contains(newBpc) && !unstableBpcsToUnmerge.contains(newBpc)) 
						bpcs.add(newBpc);
				if (bpcs.size()==1)
					unstableBpcsToUnmerge.add(bpcs.get(0));
			}
		}
		bpcsToUnmerge.addAll(unstableBpcsToUnmerge);
		
		// Cut connections
		for (BodyPairContact bpc: bpcsToUnmerge)
			unmergeBodyPairContact(bpc);
		
		// Compute resulting new collections/bodies
		ArrayList<RigidBody> handledBodies = new ArrayList<RigidBody>();
		ArrayList<RigidBody> subbodies = new ArrayList<RigidBody>();
		
		for (BodyPairContact bpc: bpcsToUnmerge) {
			for (int i=0; i<2; i++) { 
				RigidBody body = bpc.getBody(i);
				
				if (!handledBodies.contains(body)) {
					
					subbodies.add(body);
					buildNeighborBody(body, subbodies, handledBodies);
					handledBodies.addAll(subbodies);
					
					for (RigidBody b: subbodies)
						collection.unmergeBody(b);

					if (subbodies.size() > 1) {
						RigidCollection newCollection = new RigidCollection(subbodies.remove(0), subbodies.remove(0));
						newCollection.addBodies(subbodies);
						newCollection.fillInternalBodyContacts();
						newCollection.color = new Color(collection.color);
						newCollection.col = new float[] { newCollection.color.x, newCollection.color.y, newCollection.color.z, 1 };
						collection.applyVelocitiesTo(newCollection);
						newBodies.add(newCollection);
					} else if (subbodies.size() == 1){ 
						newBodies.add(subbodies.get(0));
					}
					
					subbodies.clear();
				}
			}	
			
			if (bpc.body1.isInSameCollection(bpc.body2))
				mergeBodyPairContact(bpc);
		}	
		
		if(handledBodies.size() != collection.bodies.size()) {
			for (RigidBody body : collection.bodies) {
				if (body.isInCollection(collection)) {
					collection.unmergeBody(body);
					newBodies.add(body);
					handledBodies.add(body);
				}
			}
		}
		
		if(handledBodies.size() != collection.bodies.size())
			System.err.println("[unmergeSelectedBpcs] Something is wrong");
	}
	
	/**
	 * Unmerge body pair contact
	 * @param bpc
	 */
	protected void unmergeBodyPairContact(BodyPairContact bpc) {
		if (!collision.bodyPairContacts.contains(bpc)) {
			collision.bodyPairContacts.add(bpc);
			bpc.inCollection = false;
			bpc.contactStateHist.clear();
			bpc.motionMetricHist.clear();
			for (Contact contact : bpc.contactList) 
				if (!collision.contacts.contains(contact))
					collision.contacts.add(contact);
		}
	}
	
	/**
	 * Unmerge body pair contact
	 * @param bpc
	 */
	protected void mergeBodyPairContact(BodyPairContact bpc) {
		if (collision.bodyPairContacts.contains(bpc)) {
			collision.bodyPairContacts.remove(bpc);
			bpc.inCollection = true;
			for (Contact contact : bpc.contactList) {
				if (collision.contacts.contains(contact))
					collision.contacts.remove(contact);
				if (!bpc.body1.parent.internalContacts.contains(contact))
					bpc.body1.parent.internalContacts.add(contact);
			}
		}
	}

	/**
	 * Fills subBodies list with bodies connected to given body in a same collection 
	 * @param body
	 * @param bodies
	 */
	private void buildNeighborBody(RigidBody body, ArrayList<RigidBody> bodies, ArrayList<RigidBody> handledBodies) {

		for (BodyPairContact bpc : body.bodyPairContacts) {
			if (!bpc.inCollection) 
				continue;
			
			RigidBody otherBody = bpc.getOtherBody(body);
			if (!bodies.contains(otherBody) && !handledBodies.contains(otherBody)) {
				bodies.add(otherBody);
				buildNeighborBody(otherBody, bodies, handledBodies);
			}
		}
	}
	
	/**
	 * Process collections color:
	 * <p><ul>
	 * <li> if two collections have the same color, the most massive one will keep it
	 * </ul><p>
	 */
	protected void processCollectionsColor(ArrayList<RigidBody> bodies) {
		ArrayList<Color> colors = new ArrayList<Color>();
		ArrayList<RigidCollection> collections = new ArrayList<RigidCollection>();
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection)body;
				if (colors.contains(collection.color)) {
					RigidCollection sameColorCollection = collections.get(colors.indexOf(collection.color));
					if(sameColorCollection.bodies.size()>collection.bodies.size()) 
						collection.color.setRandomColor();
					else 
						sameColorCollection.color.setRandomColor();
				}
				colors.add(collection.color);
				collections.add(collection);
			}
		}
	}
	
	/**
	 * @return control panel for the system
	 */
	public JPanel getControls() {
	
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.add( params.enableMerging.getControls() );
		vfp.add( params.enableMergePinned.getControls() );
		vfp.add( params.enableMergeCycleCondition.getControls() );
		vfp.add( params.enableMergeStableContactCondition.getControls() );
		vfp.add( params.enableMergeLetItBreathe.getControls() );
		vfp.add( params.enableUnmerging.getControls() );
		vfp.add( params.enableUnmergeFrictionCondition.getControls() );
		vfp.add( params.enableUnmergeNormalCondition.getControls() );
		vfp.add( params.enableUnmergeRelativeMotionCondition.getControls() );
		vfp.add( params.updateContactsInCollections.getControls() );
		vfp.add( params.organizeContacts.getControls() );
		vfp.add( params.stepAccum.getSliderControls() );
		vfp.add( params.thresholdMerge.getSliderControls(false) );
		vfp.add( params.thresholdUnmerge.getSliderControls(false) );
		vfp.add( params.thresholdBreath.getSliderControls(true) );
        JButton umergeButton = new JButton("unmerge all");
        vfp.add( umergeButton);
        umergeButton.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	params.unmergeAll.setValue(true);
            }
        });
		CollapsiblePanel cpm = new CollapsiblePanel(vfp.getPanel());
		cpm.collapse();

		return vfp.getPanel();
	}
}
