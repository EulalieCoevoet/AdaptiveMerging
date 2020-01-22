package mergingBodies3D;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;

import javax.swing.JButton;
import javax.swing.JPanel;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;

public class Merging {
	
	public boolean mergingEvent = false;
	public boolean triggerMergingEvent = false;
	
	public enum UnmergingCondition {RELATIVEMOTION, CONTACTS;}
	
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
		public BooleanParameter enableUnmergeRelativeMotionCondition = new BooleanParameter( "unmerging - relative motion condition", true);
		public BooleanParameter updateContactsInCollections = new BooleanParameter( "update contact in collection", true);
		public BooleanParameter organizeContacts = new BooleanParameter( "organize contacts", true);
		public IntParameter stepAccumMerging = new IntParameter("check threshold over N number of time steps for merging", 3, 0, 200 );
		public IntParameter stepAccumUnmerging = new IntParameter("check threshold over N number of time steps for unmerging", 3, 0, 200 ); 
		public DoubleParameter thresholdMerge = new DoubleParameter("merging threshold", 1e-2, 1e-10, 100 );
		public DoubleParameter thresholdUnmerge = new DoubleParameter("unmerging threshold", 2e-2, 1e-10, 100 );
		public DoubleParameter thresholdBreath = new DoubleParameter("breathing threshold", 1e-5, 1e-10, 1e0 );
		public BooleanParameter unmergeAll = new BooleanParameter("unmerge all", false);
		public BooleanParameter changeColors = new BooleanParameter("change colors", false);
		
	    public double mergingCheckContactTime = 0;
	    public double mergingCheckCycleTime = 0;
	    public double mergingCheckMotionTime = 0;
	    public double mergingCheckViolationTime = 0;
	    public double mergingBuildTime = 0;
	    public double unmergingCheckContactTime = 0;
	    public double unmergingCheckCycleTime = 0;
	    public double unmergingCheckMotionTime = 0;
	    public double unmergingBuildTime = 0;
	}
	public MergeParameters params = new MergeParameters();
	
	Merging(ArrayList<RigidBody> bodies, CollisionProcessor CP) {
		this.bodies = bodies;
		this.collision = CP;
	}

	protected void resetMergingTime() {
	    params.mergingCheckContactTime = 0;
	    params.mergingCheckCycleTime = 0;
	    params.mergingCheckMotionTime = 0;
	    params.mergingCheckViolationTime = 0;
	    params.mergingBuildTime = 0;
	}
	
	protected void resetUnmergingTime(UnmergingCondition condition) {
		if (condition == UnmergingCondition.CONTACTS) {
			params.unmergingCheckContactTime = 0;
			
			// during a time step, contacts for unmerging is the first thing we check...
			params.unmergingCheckCycleTime = 0;
			params.unmergingCheckMotionTime = 0;
			params.unmergingBuildTime = 0;
		}
		
		if (condition == UnmergingCondition.RELATIVEMOTION)
			params.unmergingCheckContactTime = 0;
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
		
		resetMergingTime();
		
		for (BodyPairContact bpc : collision.bodyPairContacts) {
			
			if (!bpc.inCollection && bpc.checkMergeCondition(params, true)) {

				long now = System.nanoTime();
				mergingEvent = true;
				bpc.inCollection = true;
				bpc.motionMetricHist.clear();
				bpc.contactStateHist.clear();
				
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

				params.mergingBuildTime += (System.nanoTime() - now) / 1e9;
			}
		}
		
		for (BodyPairContact bpc: removalQueue) {
			collision.bodyPairContacts.remove(bpc);
			//collision.contacts.removeAll(bpc.contactList);
		}
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
				
				for (BodyPairContact bpc: collection.bodyPairContacts) {
					if (!collision.bodyPairContacts.contains(bpc))
						collision.bodyPairContacts.add(bpc);
					bpc.inCollection = false;
					if(bpc.inCollection) {
						for (Contact contact : bpc.contactList) {
							contact.lambda0warm = contact.lambda0;
							contact.lambda1warm = contact.lambda1;
							contact.lambda2warm = contact.lambda2;
							collision.contacts.add(contact);
						}
					}
				}
							
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
	
	private LinkedList<RigidBody> removalQueue = new LinkedList<RigidBody>();
	private LinkedList<RigidBody> additionQueue = new LinkedList<RigidBody>();
	private HashSet<BodyPairContact> bpcsToUnmerge = new HashSet<BodyPairContact>();
	private ArrayList<RigidBody> newBodies = new ArrayList<RigidBody>();
	/**
	 * Unmerge BodyPairContacts that satisfy condition
	 */	
	public void unmerge(UnmergingCondition condition, double dt) {
		if (!params.enableUnmerging.getValue())
			return;

		resetUnmergingTime(condition);
		
		if (condition == UnmergingCondition.RELATIVEMOTION && !params.enableUnmergeRelativeMotionCondition.getValue())
			return;
		
		if (condition == UnmergingCondition.CONTACTS && !(params.enableUnmergeNormalCondition.getValue() || params.enableUnmergeFrictionCondition.getValue()))
			return;
		
		removalQueue.clear();
		additionQueue.clear();
		
		for(RigidBody body : bodies) {
			
			if(body.isSleeping )
				continue;
			
			if (body instanceof RigidCollection) {
				
				RigidCollection collection = (RigidCollection) body;
				bpcsToUnmerge.clear();
				
				for (BodyPairContact bpc: collection.bodyPairContacts) {
					if (!bpc.inCollection)
						continue;
					
					if (!bpcsToUnmerge.contains(bpc)) {
						
						if (condition == UnmergingCondition.CONTACTS && !bpc.checkContactsState(dt, params))
							continue;

						if (condition == UnmergingCondition.RELATIVEMOTION && !bpc.checkMotionMetricForUnmerging(params))
							continue;

						long now = System.nanoTime();
						if (params.enableMergeCycleCondition.getValue())
							bpc.checkCyclesToUnmerge(bpcsToUnmerge);
						params.unmergingCheckCycleTime += (System.nanoTime() - now) / 1e9;
						
						bpc.addBpcToUnmerge(bpcsToUnmerge);
					}
				}
				
				long now = System.nanoTime();
				newBodies.clear();
				boolean removeCollection = false;
				if (!bpcsToUnmerge.isEmpty()) 
					removeCollection = unmergeSelectedBpcs(collection, bpcsToUnmerge, newBodies, dt);	

				if (!newBodies.isEmpty()) {
					mergingEvent = true;
					additionQueue.addAll(newBodies);
					if (removeCollection)
						removalQueue.add(collection);
				}
				params.unmergingBuildTime += (System.nanoTime() - now) / 1e9;
			}
		}
		
		bodies.addAll(additionQueue);
		bodies.removeAll(removalQueue);

		processCollectionsColor(bodies);
	}

	private HashSet<RigidBody> handledBodies = new HashSet<RigidBody>();
	private HashSet<RigidBody> subbodies = new HashSet<RigidBody>();
	private HashSet<RigidBody> remainedBodies = new HashSet<RigidBody>();
	/**
	 * Unmerge given bpcs
	 * @param collection
	 * @param bpcsToUnmerge
	 * @param newBodies
	 * @param dt
	 * @return true if collection should be removed from the system
	 */
	private boolean unmergeSelectedBpcs(RigidCollection collection, HashSet<BodyPairContact> bpcsToUnmerge, ArrayList<RigidBody> newBodies, double dt) {
			
		boolean removeCollection = true;
		
		// Check for unstable configurations
		// TODO: eulalie: don't know about this strategy
		/*ArrayList<BodyPairContact> unstableBpcsToUnmerge = new ArrayList<BodyPairContact>();
		ArrayList<BodyPairContact> bpcs = new ArrayList<BodyPairContact>();
		for (BodyPairContact bpc : bpcsToUnmerge) {
			for (int i=0; i<2; i++) { 
				RigidBody body = bpc.getBody(i);
				bpcs.clear();
				for (BodyPairContact newBpc : body.bodyPairContacts) 
					if (newBpc.contactList.size()<3 && newBpc.inCollection && !bpcsToUnmerge.contains(newBpc) && !unstableBpcsToUnmerge.contains(newBpc)) 
						bpcs.add(newBpc);
				if (bpcs.size()==1)
					unstableBpcsToUnmerge.add(bpcs.get(0));
			}
		}
		bpcsToUnmerge.addAll(unstableBpcsToUnmerge);*/
		
		// Cut connections
		for (BodyPairContact bpc: bpcsToUnmerge)
			bpc.inCollection = false;
		
		// Compute resulting new collections/bodies
		handledBodies.clear();
		subbodies.clear();
		remainedBodies.clear();
		
		for (RigidBody body: collection.bodies) {
			if (!handledBodies.contains(body)) {
				
				subbodies.add(body);
				buildNeighborBody(body, subbodies, handledBodies);
				handledBodies.addAll(subbodies);
				
				if (collection.bodies.size() != subbodies.size()) { 
					if (subbodies.size() < collection.bodies.size()/2+1) { 
						for (RigidBody b: subbodies)
							collection.unmergeBody(b);
	
						if (subbodies.size() > 1) { // new collection
							Iterator<RigidBody> iter = subbodies.iterator();
							
							RigidBody sb1 = iter.next();
							RigidBody sb2 = iter.next();
							subbodies.remove(sb1);
							subbodies.remove(sb2);
							
							RigidCollection newCollection = new RigidCollection(sb1,sb2);
							newCollection.addBodies(subbodies);
							newCollection.fillInternalBodyContacts();
							newCollection.color = new Color(collection.color);
							newCollection.col = new float[] { newCollection.color.x, newCollection.color.y, newCollection.color.z, 1 };
							collection.applyVelocitiesTo(newCollection); //eulalie: the velocities are updated in call to addBodies...
							newBodies.add(newCollection);
						} else if (subbodies.size() == 1){ // single body
							newBodies.add(subbodies.iterator().next());
						}
					} else { // collection is only cut in half or less
						removeCollection = false;
						remainedBodies.addAll(subbodies);
					}
				} else { // collection remains entirely the same, happens if the bpc doesn't actually cut the collection in pieces
					removeCollection = false;
					remainedBodies.addAll(subbodies);
					break;
				}
				
				subbodies.clear();
			}	
		}	

		// Reconnect if necessary, if not clean cut 
		for (BodyPairContact bpc: bpcsToUnmerge) {
			if (bpc.body1.isInSameCollection(bpc.body2)) {
				bpc.inCollection = true;
			} else {
				if (!collision.bodyPairContacts.contains(bpc)) {
					collision.bodyPairContacts.add(bpc);
					for (Contact contact : bpc.contactList) {
						contact.lambda0warm = contact.lambda0;
						contact.lambda1warm = contact.lambda1;
						contact.lambda2warm = contact.lambda2;
						collision.contacts.add(contact);
					}
				}
				bpc.motionMetricHist.clear();
				bpc.contactStateHist.clear();
			}
		}
		
		if(handledBodies.size() != collection.bodies.size())
			System.err.println("[unmergeSelectedBpcs] Something is wrong");
		
		if (!removeCollection) {
			if (remainedBodies.size() != collection.bodies.size()) {
				handledBodies.removeAll(remainedBodies);
				collection.removeBodies(handledBodies);
				collection.fillInternalBodyContacts();
			}
		}
		
		return removeCollection;
	}

	/**
	 * Fills subBodies list with bodies connected to given body in a same collection 
	 * @param body
	 * @param bodies
	 */
	private void buildNeighborBody(RigidBody body, HashSet<RigidBody> bodies, HashSet<RigidBody> handledBodies) {

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
	
	private ArrayList<Color> colors = new ArrayList<Color>();
	private ArrayList<RigidCollection> collections = new ArrayList<RigidCollection>();
	
	/**
	 * Process collections color:
	 * <p><ul>
	 * <li> if two collections have the same color, the most massive one will keep it
	 * </ul><p>
	 */
	protected void processCollectionsColor(ArrayList<RigidBody> bodies) {
		colors.clear();
		collections.clear();
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection) {
				RigidCollection collection = (RigidCollection)body;
				if (colors.contains(collection.color)) {
					RigidCollection sameColorCollection = collections.get(colors.indexOf(collection.color));
					if(sameColorCollection.bodies.size()>collection.bodies.size()) {
						collection.generateColor();
					} else {
						sameColorCollection.generateColor();
					}
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
		vfp.add( params.stepAccumMerging.getSliderControls() );
		vfp.add( params.stepAccumUnmerging.getSliderControls() );
		vfp.add( params.thresholdMerge.getSliderControls(false) );
		vfp.add( params.thresholdUnmerge.getSliderControls(false) );
		vfp.add( params.thresholdBreath.getSliderControls(true) );
		vfp.add( Contact.slidingThreshold.getSliderControls(true) ); // Gross?
        JButton umergeButton = new JButton("unmerge all");
        vfp.add( umergeButton);
        umergeButton.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	params.unmergeAll.setValue(true);
            }
        });
        JButton changeColors = new JButton("change colors");
        vfp.add( changeColors);
        changeColors.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	params.changeColors.setValue(true);
            }
        });

		return vfp.getPanel();
	}
}
