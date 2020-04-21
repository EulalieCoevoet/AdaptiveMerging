package mergingBodies3D;

import java.awt.Font;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import mergingBodies3D.Merging.MergeParameters;
import mergingBodies3D.collision.BoxBox;
import mergingBodies3D.collision.BoxPlane;
import mergingBodies3D.collision.BoxSphere;
import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.parameters.OptionParameter;
import mintools.swing.VerticalFlowPanel;

/**
 * Class for detecting and resolving collisions.  Currently this class uses penalty forces between rigid bodies.
 * 
 */
public class CollisionProcessor {

    private ArrayList<RigidBody> bodies;

    /**
     * We want warm starts to be very efficient so let's hope that the Contact hashing function is good
     * enough to avoid collisions, and likewise that the hash table for this map is large enough
     * to also avoid collisions.  This will grow dynamically, but we could likewise set the number
     * higher by default if we are running large simulations.
     */
    private HashMap<Contact,Contact> lastTimeStepContacts = new HashMap<Contact,Contact>(8192);
	
    /**
     * The current contacts that resulted in the last call to process collisions
     */
    ArrayList<Contact> contacts = new ArrayList<Contact>();

    /**
     * Array of contacts used for the single cycle update, kept separate 
     * from the main solve contact list
     */
	ArrayList<Contact> altContactList = new ArrayList<Contact>(); 


    public ContactPool contactPool = new ContactPool();
    
	/** PGS solver */
    private PGS solver = new PGS();
	
    /**
     * Creates this collision processor with the provided set of bodies
     * @param bodies
     */
    public CollisionProcessor( ArrayList<RigidBody> bodies ) {
        this.bodies = bodies;
    }
    
    /** keeps track of the time used for collision detection on the last call */
    double collisionDetectTime = 0;
    
    /** keeps track of the time used to solve the LCP based velocity update on the last call */
    double collisionSolveTime = 0;
   
	/** keeps track of the time used to update the contacts inside the collections on the last call */
	double collectionUpdateTime = 0;
	
	double contactOrderingTime = 0;
	
	double singleItPGSTime = 0;
	
	// TODO: MEMORY POOLING? might want a memory pool for BodyPairContact too?  Could be complicated since bodyPairContacts have some dynamic internal memory??  
	/**list that keeps track of all the body pair contacts that occurred in this time step */
	public HashSet<BodyPairContact> bodyPairContacts = new HashSet<BodyPairContact>();
	/**used to update bodyPairContacts*/
	private ArrayList<BodyPairContact> tmpBodyPairContacts = new ArrayList<BodyPairContact>();
	private ArrayList<BodyPairContact> tmpBodyPairContacts2 = new ArrayList<BodyPairContact>();
	
    /**
     * Processes all collisions 
     * @param dt time step
     */
    public void collisionDetection( double dt ) {
        contacts.clear();
        contactPool.swapPools();
        
        long now = System.nanoTime();
        broadPhase();
        collisionDetectTime = ( System.nanoTime() - now ) * 1e-9;
                
        if (contacts.isEmpty()) {
        	lastTimeStepContacts.clear();        	
        }
    }
    
	/**
	 * Solve LCP
	 * @param dt time step
	 */
	public void solveLCP( double dt, boolean postStabilization ) {
		
		if(!contacts.isEmpty()) {
	    	
	    	if (shuffle.getValue()) 
	    		knuthShuffle(contacts);
	    	
	    	// set up contacts solver 
			solver.init(iterations.getValue());
			solver.feedbackStiffness = (!enablePostStabilization.getValue() || postStabilization)? feedbackStiffness.getValue(): 0.;
			solver.postStabilization = postStabilization;
			solver.compliance = (enableCompliance.getValue())? compliance.getValue() : 0.;
			solver.warmStart = true;
			solver.contacts = contacts;

			// if we did a single cycle update, then some of our contacts will have had their
			// jacobians computed for acting on bodies rather than collections... so update 
			// those again so that we can compute the solution for the full system.
			updateJacobiansThatNeedUpdating( solver.contacts, false );
			
			long now = System.nanoTime();
			solver.solve( dt, restitutionOverride.getValue(), restitution.getValue(), frictionOverride.getValue(), friction.getValue() );
			collisionSolveTime = (System.nanoTime() - now) * 1e-9;
			
		} else {
			collisionSolveTime = 0.;
		}
	}
	
	/**
	 * Updates bodyPairContacts list
	 * It seems that the main thing that might happen is that a BPC could end up with an empty 
	 * contact list...  Feels like there should be another way to deal with this...   but I can't 
	 * see it right now... 
	 */
	public void updateBodyPairContacts() {
		
		for (BodyPairContact bpc : bodyPairContacts) { // clear contactList of existing bpc
			bpc.checked = false;
			bpc.contactList.clear();
		}
		
		for (RigidBody body: bodies) {
			if (body instanceof RigidCollection) {
				for (BodyPairContact bpc: body.bodyPairContacts) {
					bpc.checked = false;
				}
			}
		}

		for (Contact contact : contacts) // loop over all contacts between bodies
			storeInBodyPairContacts(contact);
		
		removeEmptyBodyPairContacts();
	}

	/**
	 * Store contact in bodyPairContacts.
	 * @param contact newly detected contact
	 */
	private void storeInBodyPairContacts(Contact contact) {

		// TODO : I think this will never happen as we don't detect collision between two pinned bodies
		if (contact.body1.pinned && contact.body2.pinned) 
			return;
		
		BodyPairContact bpc = BodyPairContact.checkExists(contact.body1, contact.body2, bodyPairContacts);

		if (bpc == null) { // body contact did not exist in previous list
			bpc = new BodyPairContact(contact.body1, contact.body2);
			bodyPairContacts.add(bpc);
		} 
		
		bpc.addToBodyLists();
		bpc.addToBodyListsParent();
		bpc.contactList.add(contact);
	}
	
	/**
	 * Remove empty body pair contacts	 
	 */
	protected void removeEmptyBodyPairContacts() {
		tmpBodyPairContacts.clear();
		boolean updated = false;
		for (BodyPairContact bpc : bodyPairContacts) {
			if(!bpc.contactList.isEmpty())
				tmpBodyPairContacts.add(bpc);
			else {
				bpc.removeFromBodyLists();
				bpc.removeFromBodyListsParent();
				updated = true;
			}
		}
		
		if (updated) {
			bodyPairContacts.clear();
			bodyPairContacts.addAll(tmpBodyPairContacts);
		}
	}
	
	/**
	 * Remove inactive contact from bodyPairContacts list
	 */
	public void clearBodyPairContacts() {

		ArrayList<Contact> tmpContacts = new ArrayList<Contact>();
		for (BodyPairContact bpc : bodyPairContacts) {
			tmpContacts.clear();
			for (Contact contact : bpc.contactList) 
				if (Math.abs(contact.lambda0) > 1e-14) // (math.abs is for magnet)
					tmpContacts.add(contact);
			bpc.contactList.clear();
			bpc.contactList.addAll(tmpContacts);
		}

		removeEmptyBodyPairContacts();
	}
	
	/**
	 * Does one iteration of PGS to update contacts inside collections
	 * @param dt
	 */
	public void updateInCollections(double dt, MergeParameters mergeParams) {

		if (!mergeParams.updateContactsInCollections.getValue()) {
			collectionUpdateTime = 0.;
			return;
		}
		
		long now = System.nanoTime();
		
		solver.init(iterationsInCollection.getValue());
		solver.computeInCollection = true;
		solver.feedbackStiffness = feedbackStiffness.getValue();
		solver.compliance = (enableCompliance.getValue())? compliance.getValue() : 0.;
		solver.warmStart = true;
		
		solver.contacts = altContactList;
		solver.contacts.clear();

		if ( !hasCollections() ) {
			collectionUpdateTime = 0.;
			return;
		}
		
		if (mergeParams.organizeContacts.getValue()) {	
			long now1 = System.nanoTime();	
			getOrganizedContacts(solver.contacts);
			contactOrderingTime = ( System.nanoTime() - now1 ) * 1e-9;
		} else {	
			// This resolution is done with the Jacobians of the bodies (not the collection) 
			// so not a good warm start for the LCP solve (we don't want to keep these values).
			// we will redo the warm start after
			solver.contacts.addAll(contacts);
			
			for (RigidBody body : bodies) {
				if (body instanceof RigidCollection && !body.sleeping) {
					RigidCollection collection = (RigidCollection)body;
					solver.contacts.addAll(collection.internalContacts);
				}
			}
			
			if (shuffle.getValue()) 
	    		knuthShuffle(solver.contacts);
		}
		
		// We may need to update select jacobians 
		// - contacts that are internal (i.e., their frames moved)
		// - contacts that are external but touch a collection (i.e., we need the jacobian to the 
		//   rather than the collection, and while this would be a partial update of the jacobian if
		//   there is only a collection on one side, it is easier just to ask again that the whole
		//   jacobian be computed
		updateJacobiansThatNeedUpdating( solver.contacts, true );

		long now2 = System.nanoTime();	
		solver.postStabilization = false;
		solver.solve(dt, restitutionOverride.getValue(), restitution.getValue(), frictionOverride.getValue(), friction.getValue() );
		singleItPGSTime = ( System.nanoTime() - now2 ) * 1e-9;
		
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection && !body.sleeping) {
				RigidCollection collection = (RigidCollection)body;
			
				// Update the bodies velocities for unmerge condition (relative motion)
				for (RigidBody b : collection.bodies)
					if(!b.pinned ) 
						b.advanceVelocities(dt);	
			}
			// Reset deltaV for LCP solve
			body.deltaV.setZero();
		}
		
		collectionUpdateTime = ( System.nanoTime() - now ) * 1e-9;
	}
	
	/**
	 * Check to see if we have collections, as we can save some work if we dont
	 * @return
	 */
	protected boolean hasCollections() {
		boolean hasCollection = false;		
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection) {
				hasCollection = true;
			}
		}
		return hasCollection;
	}
	
	/** 
	 * Given a list of contacts, recompute jacobians, and use the computeInCollection flag to determine if
	 * the jacobians should be computed based on subbody or parent
	 * @param list
	 * @param computeInCollection
	 * @return
	 */
	protected void updateJacobiansThatNeedUpdating( Collection<Contact> list, boolean computeInCollection) {
		for ( Contact c : list ) {
			// if either body parent is non null then one side or both is a collection
			// and possibly these bodies are even within the same collection.
			if ( c.body1.parent != null || c.body2.parent != null ) {
				c.computeJacobian(computeInCollection);
			}
		}
	}
	
	private ArrayList<BodyPairContact> orderedBpcs = new ArrayList<BodyPairContact>();
	
	/**
	 * Reordering of the contacts list for the one iteration PGS as follows:
	 * <p><ul>
	 * <li> 1. New contacts 
	 * <li> 2. For each new contact, add one level of contact neighbors
	 * <li> 3. Continue until the end
	 * </ul><p>
	 */
	protected void getOrganizedContacts(ArrayList<Contact> contacts) {

		orderedBpcs.clear();
		
		// first add all bpc containing new contacts
		for ( BodyPairContact bpc : bodyPairContacts ) {
			for ( Contact contact : bpc.contactList ) {
				if (contact.newThisTimeStep) {
					orderedBpcs.add(bpc);
					bpc.checked = true;
					break;
				}
			}
		}
		
		// second add all bpc of bodies the user interacted with
		for ( RigidBody body : bodies ) {
			if (body instanceof RigidCollection) {
				for ( RigidBody b : ((RigidCollection)body).bodies ) {
					if ( b.picked ) {
						for ( BodyPairContact bpc : b.bodyPairContacts )
							if (!bpc.checked) {
								orderedBpcs.add(bpc);
								bpc.checked=true;
							}
						b.picked = false;
					}
				}
			} else if ( body.picked ) {
				for ( BodyPairContact bpc : body.bodyPairContacts )
					if (!bpc.checked) {
						orderedBpcs.add(bpc);
						bpc.checked=true;
					}
				body.picked = false;
			}
		}

		// then add bpc layer by layer by looping over the "new bpcs" 
		if (!orderedBpcs.isEmpty()) {
			ArrayList<BodyPairContact> tmpBodyPairContacts = new ArrayList<BodyPairContact>();
			tmpBodyPairContacts.addAll(orderedBpcs);
			getNextLayer(tmpBodyPairContacts, orderedBpcs);
		}
		
		// add missing bpc from bodyPairContacts (non-connected to the new contacts)
		for ( BodyPairContact bpc : bodyPairContacts ) {
			if (!bpc.checked) {
				orderedBpcs.add(bpc);
				bpc.checked = true;
			}
		}

		// add missing bpc from collections (non-connected to the new contacts)
		for (RigidBody body : bodies) {
			if (body instanceof RigidCollection && !body.sleeping) {
				RigidCollection collection = (RigidCollection)body;
				for ( BodyPairContact bpc : collection.bodyPairContacts ) {
					if (!bpc.checked) {
						orderedBpcs.add(bpc);
						bpc.checked = true;
					}
				}
			}
		}
		
		// build the final organized contact list
		for ( BodyPairContact bpc : orderedBpcs ) {
			// This resolution is done with the Jacobians of the bodies (not the collection) 
			// so not a good warm start for the LCP solve (we don't want to keep these values).
			// we will redo the warm start after
			contacts.addAll( bpc.contactList );			
		}
	}	
	
	protected void getNextLayer(ArrayList<BodyPairContact> bodyPairContacts, ArrayList<BodyPairContact> orderedBpcs) {

		tmpBodyPairContacts.clear();
		for ( BodyPairContact bpc : bodyPairContacts ) {
			for (int i=0; i<2; i++) { 
				RigidBody body = bpc.getBody(i);
				for ( BodyPairContact otherBpc : body.bodyPairContacts ) {
					if (!otherBpc.checked) {
						tmpBodyPairContacts.add(otherBpc);
						otherBpc.checked = true;
					}
				}
			}
		}			
		if (!tmpBodyPairContacts.isEmpty()) {
			orderedBpcs.addAll(tmpBodyPairContacts);
			tmpBodyPairContacts2.clear();
			tmpBodyPairContacts2.addAll(tmpBodyPairContacts);
			getNextLayer(tmpBodyPairContacts2, orderedBpcs);
		}
	}
	
	protected void updateContactsMap() {
		lastTimeStepContacts.clear();
		for ( Contact contact : contacts ) {
			lastTimeStepContacts.put( contact, contact );
		}
	}
	
	/**
	 * Re-initialized lambdas with values identified from the warm start
	 */
	protected void redoWarmStart() {
		for (Contact c : contacts) {
			c.lambda0 = c.lambda0warm;
			c.lambda1 = c.lambda1warm;
			c.lambda2 = c.lambda2warm;
		}
	}
	
	int badWarmStarts = 0;
	int badWarmStartsRepaired = 0;
	
	/**
	 * Fills lambdas with values from previous time step.
	 * 
	 * Would be nice to just loop over all contacts, but this uses the BPCs to treat box-box warm starts
	 * slightly differently given that they don't always end up preserving their "info" number across time steps.
	 */
	protected void warmStart(boolean postStabilization) {
		
		badWarmStarts = 0;
		badWarmStartsRepaired = 0;
		for ( BodyPairContact bpc : bodyPairContacts ) {
			if ( bpc.inCollection ) continue;
			
			boolean b1IsBox = bpc.body1.geom instanceof RigidBodyGeomBox;
			boolean b2IsBox = bpc.body2.geom instanceof RigidBodyGeomBox;
			boolean b1IsComp = bpc.body1.geom instanceof RigidBodyGeomComposite;
			boolean b2IsComp = bpc.body2.geom instanceof RigidBodyGeomComposite;
			
			if ( b1IsBox && b2IsBox || b1IsComp && b2IsBox || b1IsBox && b2IsComp || b1IsComp && b2IsComp ) {
				for ( Contact contact : bpc.contactList ) {
					// could end up here with composite bodies... so be sure to drop out into a vanilla
					// warm start if it isn't a box box collision.
					if ( contact.csb1 != null && !(contact.csb1.geom instanceof RigidBodyGeomBox) ) {
						vanillaWarmStart( contact, postStabilization );
						continue; 
					}
					if ( contact.csb2 != null && !(contact.csb2.geom instanceof RigidBodyGeomBox)) {
						vanillaWarmStart( contact, postStabilization );
					}
					
					// note that body1 and body 2 can swap across time steps... so just compare in the current world!
					// note also that nothing fancy is needed for composite bodies... the contact csb1 and csb2 
					// (composite sub bodies) will match given the hashcode lookup, so we can search for our warm start
					// by changing the info number, running info from 0 to 7 
					
					// TODO: this threshold for fixing box-boxy warm starts is perhaps delicate??
					// Perhaps 0.1 or 0.05 is far enough to say it is a bad match?
					// should also perhaps make sure that we don't match the same more than once??
					// which would give a bit of a boost to the lagrange multipliers in the warm start.
					
					Point3d pNew = new Point3d();
					Point3d pOld = new Point3d();
					contact.body1.transformB2W.transform( contact.contactB1, pNew );

					Contact oldContact = lastTimeStepContacts.get( contact );
					if (oldContact != null) {
						oldContact.body1.transformB2W.transform( oldContact.contactB1, pOld );
						double dist = pNew.distance( pOld );
						if (  dist > 0.05 ) {
							badWarmStarts++;
							// try all info numbers on this contact (other than our current info number) to find a better match,
							// then put our info number back to what it is suppose to be.
							int myInfo = contact.info;
							double bestMatchDist = dist;
							int bestMatchInfo = myInfo;
							Contact bestMatchOldContact = oldContact;
							for ( int info = 0; info < 9; info++ ) {
								if ( info == myInfo ) continue; // we've already got this as the best so far
								contact.info = info;
								oldContact = lastTimeStepContacts.get( contact );
								if ( oldContact == null ) break; // no more contacts to check
								if ( info == 8 ) {
									System.err.println("warm start: unexpected number of box-box contacts!  what's the max number?");
								}
								oldContact.body1.transformB2W.transform( oldContact.contactB1, pOld );
								dist = pNew.distance( pOld );
								if ( dist < bestMatchDist ) {
									bestMatchDist = dist;
									bestMatchInfo = info;
									bestMatchOldContact = oldContact;
								}
							}
							if ( bestMatchInfo != myInfo ) {
								badWarmStartsRepaired++;
							}
							oldContact = bestMatchOldContact;
							contact.info = myInfo; 
							dist = bestMatchDist;
							// restoring the info of the contact in this step means that if this was just
							// an order swap that happens when rotating from 44 to 46 degrees, for instance,
							// then we won't need this search for the next warm start.
						}
						
						if ( dist < 0.05 ) {
					
							contact.newThisTimeStep = false; // not new, even if we failed to do a good warm start
							contact.lambda0 = oldContact.lambda0;
							contact.lambda1 = oldContact.lambda1;
							contact.lambda2 = oldContact.lambda2;
	
							contact.lambda0warm = oldContact.lambda0;
							contact.lambda1warm = oldContact.lambda1;
							contact.lambda2warm = oldContact.lambda2;
							
							// we are taking these values... don't let anyone else have them!
							oldContact.lambda0 = 0;
							oldContact.lambda1 = 0;
							oldContact.lambda2 = 0;
	
							if(!postStabilization)
								contact.prevConstraintViolation = oldContact.constraintViolation;
							else
								contact.prevConstraintViolation = oldContact.prevConstraintViolation;
						} else {
							// may as well treat as new as didn't find anything close.
							contact.newThisTimeStep = true;
						}
					} else {
						// Just because we didn't find ourselves, doesn't mean that we're not there!
						// start searching from zero
						int myInfo = contact.info;
						double bestMatchDist = Double.MAX_VALUE;
						int bestMatchInfo = -1;
						Contact bestMatchOldContact = null;
						for ( int info = 0; info < 9; info++ ) {
							contact.info = info;
							oldContact = lastTimeStepContacts.get( contact );
							if ( oldContact == null ) break; // no more contacts to check
							oldContact.body1.transformB2W.transform( oldContact.contactB1, pOld );
							double dist = pNew.distance( pOld );
							if ( dist < bestMatchDist ) {
								bestMatchDist = dist;
								bestMatchInfo = info;
								bestMatchOldContact = oldContact;
							}
						}
						contact.info = myInfo; // put my info back!!
						if ( bestMatchInfo != -1 ) {
							badWarmStartsRepaired++;
							// found something... but might still be large
							if ( bestMatchDist < 0.05 ) {
								oldContact = bestMatchOldContact;
								
								contact.newThisTimeStep = false; // not new, even if we failed to do a good warm start
								contact.lambda0 = oldContact.lambda0;
								contact.lambda1 = oldContact.lambda1;
								contact.lambda2 = oldContact.lambda2;
		
								contact.lambda0warm = oldContact.lambda0;
								contact.lambda1warm = oldContact.lambda1;
								contact.lambda2warm = oldContact.lambda2;
								
								// we are taking these values... don't let anyone else have them!
								oldContact.lambda0 = 0;
								oldContact.lambda1 = 0;
								oldContact.lambda2 = 0;
								
								if(!postStabilization)
									contact.prevConstraintViolation = oldContact.constraintViolation;
								else
									contact.prevConstraintViolation = oldContact.prevConstraintViolation;
							}

						} else {
							contact.newThisTimeStep = true;
						}
					}
				}
					
			} else {
				for ( Contact contact : bpc.contactList ) {
					vanillaWarmStart( contact, postStabilization );
				}
			}
		}
	}
	
	/** 
	 * Do the basic warm start of taking whatever contact matches the hash.
	 * @param contact
	 */
	private void vanillaWarmStart( Contact contact, boolean postStabilization ) {
		Contact oldContact = lastTimeStepContacts.get( contact );
		if (oldContact != null) {
			contact.newThisTimeStep = false;
			contact.lambda0 = oldContact.lambda0;
			contact.lambda1 = oldContact.lambda1;
			contact.lambda2 = oldContact.lambda2;
			
			contact.lambda0warm = oldContact.lambda0;
			contact.lambda1warm = oldContact.lambda1;
			contact.lambda2warm = oldContact.lambda2;

			if(!postStabilization)
				contact.prevConstraintViolation = oldContact.constraintViolation;
			else
				contact.prevConstraintViolation = oldContact.prevConstraintViolation;
		} else {
			contact.newThisTimeStep = true;
		}					
	}
	

	/**go through each element in contacts 2.
	* at each element, swap it for another random member of contacts2. 
	* at each element, get a random index from i to contacts.size.
	* swap this element with that element at that index. 
	* go to next element in contacts 2
	*/
	private void knuthShuffle(ArrayList<Contact> contacts) {
		Collections.shuffle(contacts);
	}
    
    /**
     * Checks for collisions between bodies.  Note that you can optionaly implement some broad
     * phase test such as spatial hashing to reduce the n squared body-body tests.
     * Currently this does the naive n squared collision check.
     */
    private void broadPhase() {    	
        // Naive n squared body test.. might not be that bad for even a larger number of bodies
    	// as the LCP solve for large number of contacts is really the killer!
        visitID++;
        int N = bodies.size();
        for ( int i = 0; i < N-1; i++ ) {
        	RigidBody b1 = bodies.get(i);
        	for ( int j = i+1; j < N; j++ ) {
        		RigidBody b2 = bodies.get( j );
                if ( b1.pinned && b2.pinned ) continue;
                if ( (b1.pinned && b2.sleeping) || (b2.pinned && b1.sleeping) ) continue;
                narrowPhase( b1, b2 );                
        	}
        }    
    }

    double[] depth = new double[1];
    int [] rc = new int[1];

    /**
     * Checks for collision between boundary blocks on two rigid bodies.
     * @param body1
     * @param body2
     */
	private void narrowPhase( RigidBody body1, RigidBody body2 ) {
				
		if ( body1 instanceof RigidCollection || body2 instanceof RigidCollection) { 	
			narrowPhaseCollection(body1, body2);
		} else if ( body1.geom instanceof RigidBodyGeomComposite ) {
			RigidBodyGeomComposite g1 = (RigidBodyGeomComposite) body1.geom;
			g1.updateBodyPositionsFromParent();
			for ( RigidBody b : g1.bodies ) {
				narrowPhase( b, body2 );
			}
		} else if ( body2.geom instanceof RigidBodyGeomComposite ) {
			RigidBodyGeomComposite g2 = (RigidBodyGeomComposite) body2.geom;
			g2.updateBodyPositionsFromParent();
			for ( RigidBody b : g2.bodies ) {
				narrowPhase( body1, b );
			}
		} else if ( body1 instanceof PlaneRigidBody ) {
			PlaneRigidBody b1 = (PlaneRigidBody) body1;
			if ( body2 instanceof PlaneRigidBody ) {
				System.err.println("plane plane collision is impossible!");
			} else if ( body2.geom instanceof RigidBodyGeomBox ) { // box plane
				RigidBodyGeomBox g2 = (RigidBodyGeomBox) body2.geom;
				BoxPlane.dBoxPlane(body2, g2.size, b1, b1.n, b1.d, contacts, contactPool );
			} else { 
				collideSphereTreeAndPlane( body2.root, body2, (PlaneRigidBody) body1 ); 
			}
		} else if ( body1.geom instanceof RigidBodyGeomBox ) {
			RigidBodyGeomBox g1 = (RigidBodyGeomBox) body1.geom;
			if ( body2 instanceof PlaneRigidBody ) { 
				PlaneRigidBody b2 = (PlaneRigidBody) body2;
				BoxPlane.dBoxPlane(body1, g1.size, b2, b2.n, b2.d, contacts, contactPool );
			} else if ( body2.geom instanceof RigidBodyGeomBox ) {
				RigidBodyGeomBox g2 = (RigidBodyGeomBox) body2.geom;
				BoxBox.dBoxBox(body1,g1.size,body2, g2.size, normal, depth, rc, contacts, contactPool );
			} else { 
				collideBoxAndSphereTree( g1, body2.root, body1, body2 );
			}
		} else { // all others have a sphere tree
			if ( body2 instanceof PlaneRigidBody ) {
				collideSphereTreeAndPlane( body1.root, body1, (PlaneRigidBody) body2 );
			} else if ( body2.geom instanceof RigidBodyGeomBox ) {
				collideBoxAndSphereTree( (RigidBodyGeomBox) body2.geom, body1.root, body2, body1 );
			} else {
				collideSphereTrees(body1.root, body2.root, body1, body2);
			}
		}
		// NOTE: might be easier to handle this flow control structure with a check on the first parameter 
		// and calling appropriate functions that check the second parameter, filtering the result down
		// to the appropriate pair of primitives?	
		// That said.... this is certainly faster with if/else than with function/method calls
	}
	
    /** 
     * The visitID is used to tag boundary volumes that are visited in 
     * a given time step.  Marking boundary volume nodes as visited during
     * a time step allows for a visualization of those used, but it can also
     * be used to more efficiently update the centers of bounding volumes
     * (i.e., call a BVNode's updatecW method at most once on any given time step)
     */
    int visitID = 0;
    
    private void narrowPhaseCollection(RigidBody body1, RigidBody body2) {    	
    	if (collectionCD.getValue() == 0) { // Brute force
			if (body1 instanceof RigidCollection ) {
				for (RigidBody body: ((RigidCollection)body1).bodies)
					narrowPhase(body, body2);
			} else if (body2 instanceof RigidCollection ) {
				for (RigidBody body: ((RigidCollection)body2).bodies)
					narrowPhase(body1, body);
			}
		} else if (collectionCD.getValue() == 1) { // BVH
			if (body1.root == null) {
				BVSphere sphere = (body1 instanceof PlaneRigidBody)? new BVSphere(new Point3d(0.,0.,0.), Double.MAX_VALUE, body1): new BVSphere(body1);
				body1.root = new BVNode(sphere);
			} else if (body2.root == null) {
				BVSphere sphere = (body2 instanceof PlaneRigidBody)? new BVSphere(new Point3d(0.,0.,0.), Double.MAX_VALUE, body2): new BVSphere(body2);
				body2.root = new BVNode(sphere);
			}
			collideCollectionBVH(body1.root, body2.root, body1, body2);
		} else if (collectionCD.getValue() == 2) { // Sweep and prune
			collideCollectionSP(body1, body2);				
		} else {
			System.err.println("[narrowPhaseCollection] Unsupported collision detection method.");
		}
    }
	
	private void collideSphereTreeAndPlane( BVNode node1, RigidBody body1, PlaneRigidBody planeBody ) {
		if ( node1.visitID != visitID ) {
			node1.visitID = visitID;
			node1.boundingSphere.updatecW();
		}
		// check bounding disc with plane
		Tuple3d c = node1.boundingSphere.cW;
		Tuple3d n = planeBody.n;
		double d = n.x*c.x + n.y*c.y + n.z*c.z + planeBody.d - node1.boundingSphere.r;
		if ( d<0 ) {
			if ( node1.isLeaf() ) {
				// create a collision here!
				
				// contact position in world coordinates will be the closest point on the plane
				// using the temp working variable normal, but don't get confused by the name!!!
				// just computing p = c-n (n \cdot (c-x))
				normal.sub( c, planeBody.p ); 
				double val = normal.dot( planeBody.n );
				normal.scale( val, planeBody.n );
				contactW.sub( c, normal );
				normal.scale( -1, planeBody.n );
				
				// only 1 contact (per leaf) if there is a contact, so we set info to zero
				Contact contact = contactPool.get(); 
	            contact.set( body1, planeBody, contactW, normal, node1.boundingSphere, planeBody.dummyBV, 0, d);
	            // simple option... add to contact list...
	            contacts.add( contact );
			} else {
				for ( BVNode child : node1.children ) {
					collideSphereTreeAndPlane( child, body1,planeBody );					
				}
			}
		}		
	}
	
	private boolean intersectPlaneSphere(BVNode node1, PlaneRigidBody planeBody) {

		Tuple3d c = node1.boundingSphere.cW;
		Tuple3d n = planeBody.n;
		double d = n.x*c.x + n.y*c.y + n.z*c.z + planeBody.d - node1.boundingSphere.r;
		return d < 0;
		
	}
	
	private void collideBoxAndSphereTree( RigidBodyGeomBox geom1, BVNode node2, RigidBody body1, RigidBody body2 ) {
		if ( node2.visitID != visitID ) {
			node2.visitID = visitID;
			node2.boundingSphere.updatecW();
		}
		if ( !BoxSphere.dBoxSphereTest( body1, geom1.size, node2.boundingSphere.cW, node2.boundingSphere.r ) ) return;
		if ( !node2.isLeaf() ) {
			for ( BVNode child : node2.children ) {
				collideBoxAndSphereTree( geom1, child, body1, body2 );
			}
		} else { // this is a leaf! we are colliding for real!
			BoxSphere.dBoxSphere(body1, geom1.size, body2, node2.boundingSphere.cW, node2.boundingSphere.r, contacts, contactPool );
		}
	}
	
	/**
	 * Narrow phase for collections (BVH method), at least one of the two given bodies should be a collection
	 * @param node1
	 * @param node2
	 * @param body1
	 * @param body2
	 */
	private void collideCollectionBVH(BVNode node1, BVNode node2, RigidBody body1, RigidBody body2) {

		if (node1.visitID != visitID) {
			node1.visitID = visitID;
			node1.boundingSphere.updatecW();
		}	
		if (node2.visitID != visitID) {
			node2.visitID = visitID;
			node2.boundingSphere.updatecW();
		}
		
		boolean intersect = false;
		if (node1.boundingSphere.body instanceof PlaneRigidBody)
			intersect = intersectPlaneSphere(node2, (PlaneRigidBody) node1.boundingSphere.body);
		else if (node2.boundingSphere.body instanceof PlaneRigidBody)
			intersect = intersectPlaneSphere(node1, (PlaneRigidBody) node2.boundingSphere.body);
		else 
			intersect = node1.boundingSphere.intersects(node2.boundingSphere);
		
		if ( intersect ) {
			
			if ( body1 instanceof RigidCollection && body2 instanceof RigidCollection ) { // both are collections
				if ( isLeaf(node1) && isLeaf(node2) )
					narrowPhase(node1.boundingSphere.body, node2.boundingSphere.body);
				else if ( isLeaf(node1) ) {
					for ( BVNode child : node2.children ) 
						collideCollectionBVH(node1, child, body1, body2);		
				} else if ( isLeaf(node2) ) {
					for ( BVNode child : node1.children ) 
						collideCollectionBVH(child, node2, body1, body2);					
				} else if ( node1.boundingSphere.r <= node2.boundingSphere.r ) {
					for ( BVNode child : node2.children ) 
						collideCollectionBVH(node1, child, body1, body2);					
				} else {
					for ( BVNode child : node1.children )
						collideCollectionBVH(child, node2, body1, body2);					
				}
			} else if ( body1 instanceof RigidCollection ) { // only body1 is a collection
				if ( isLeaf(node1) ) 
					narrowPhase(node1.boundingSphere.body, body2);
				else {
					for ( BVNode child : node1.children ) 
						collideCollectionBVH(child, node2, body1, body2);					
				}
			} else { // only body2 is a collection
				if ( isLeaf(node2) ) 
					narrowPhase(body1, node2.boundingSphere.body); 
				else {
					for ( BVNode child : node2.children )
						collideCollectionBVH(node1, child, body1, body2);					
				}
			}
		}
	}
	
	/**
	 * Narrow phase for collections (sweep and prune method), at least one of the two given bodies should be a collection 
	 * @param node1
	 * @param node2
	 * @param body1
	 * @param body2
	 */
	private void collideCollectionSP(RigidBody body1, RigidBody body2) {
		if (body1 instanceof RigidCollection ) {
			for (RigidBody body: ((RigidCollection)body1).bodies)
				collideCollectionSP(body, body2);
		} else if (body2 instanceof RigidCollection ) {
			for (RigidBody body: ((RigidCollection)body2).bodies)
				collideCollectionSP(body1, body);
		} else {			
			if (sweepCollide(body1, body2))
				narrowPhase(body1, body2);
		}
	}

	protected boolean sweepCollide(RigidBody body1, RigidBody body2) {
		
		// TODO: make it work... optimize... use a strategy...	
		if(body1 instanceof PlaneRigidBody || body2 instanceof PlaneRigidBody)
			return true;
		
		if(body1.bbMinW.x>body2.bbMaxW.x) // min > max
			return false;
		if(body1.bbMaxW.x<body2.bbMinW.x) // max < min
			return false;
		
		if(body1.bbMinW.y>body2.bbMaxW.y)
			return false;
		if(body1.bbMaxW.y<body2.bbMinW.y) 
			return false;
		
		if(body1.bbMinW.z>body2.bbMaxW.z)
			return false;
		if(body1.bbMaxW.z<body2.bbMinW.z)
			return false;
		
		return true;
	}
	
	protected boolean isLeaf(BVNode node) {
		if(node.isLeaf())
			return true;
		
		if(!(node.boundingSphere.body instanceof RigidCollection))
			return true;
		
		return false;
	}
	
	/** 
	 * Recurses through all of body_1, then body_2
	 * @param node1
	 * @param node2
	 * @param body1
	 * @param body2
	 */
	private void collideSphereTrees(BVNode node1, BVNode node2, RigidBody body1, RigidBody body2) {
		if (node1.visitID != visitID) {
			node1.visitID = visitID;
			node1.boundingSphere.updatecW();
		}	
		if (node2.visitID != visitID) {
			node2.visitID = visitID;
			node2.boundingSphere.updatecW();
		}

		if (node1.boundingSphere.intersects(node2.boundingSphere)) {
			if (node1.isLeaf() && node2.isLeaf()) {
				BVSphere leafBV1 = node1.boundingSphere;
				BVSphere leafBV2 = node2.boundingSphere;
				processCollision(body1, leafBV1, body2, leafBV2);
			} else if ( node1.isLeaf() ) {
				for ( BVNode child : node2.children ) {
					collideSphereTrees(node1, child, body1, body2);					
				}
			} else if ( node2.isLeaf() ) {
				for ( BVNode child : node1.children ) {
					collideSphereTrees(child, node2, body1, body2);					
				}
			} else if ( node1.boundingSphere.r <= node2.boundingSphere.r ) {
				// if we have the choice, descend subtree with larger sphere
				for ( BVNode child : node2.children ) {
					collideSphereTrees(node1, child, body1, body2);					
				}
			} else {
				for ( BVNode child : node1.children ) {
					collideSphereTrees(child, node2, body1, body2);					
				}
			}
		}
	}
    
    /**
     * Resets the state of the collision processor by clearing all
     * currently identified contacts, and reseting the visitID for
     * tracking the bounding volumes used
     */
    public void reset() {
        contacts.clear();
		bodyPairContacts.clear();
		lastTimeStepContacts.clear();

        visitID = 0;            
    }
    
    // some working variables for processing collisions
    private Point3d contactW = new Point3d();
    private Vector3d normal = new Vector3d();
        
    /**
     * Processes a collision between two bodies for two given blocks that are colliding.
     * Currently this implements a penalty force
     * @param body1
     * @param bv1
     * @param body2
     * @param bv2
     */
    private void processCollision( RigidBody body1, BVSphere bv1, RigidBody body2, BVSphere bv2 ) {        
        double distance = bv1.cW.distance( bv2.cW );
        double distanceBetweenCenters = bv2.r + bv1.r;
        
        if ( distance < distanceBetweenCenters ) {
            // contact point at halfway between points 
            // NOTE: this assumes that the two blocks have the same radius!
        	
        	double alpha = (bv1.r - bv2.r + distance)/(2*distance);
        	
            contactW.interpolate( bv1.cW, bv2.cW, alpha );
            // contact normal
            normal.sub( bv2.cW, bv1.cW );
            normal.normalize();
            // create the contact, and again info is zero as we only generate one contact per pair of leaves
            Contact contact = contactPool.get(); 
            contact.set( body1, body2, contactW, normal, bv1, bv2, 0, distance - distanceBetweenCenters);
            // simple option... add to contact list...
            contacts.add( contact );
        }
    }

	public BooleanParameter shuffle = new BooleanParameter( "shuffle", false);
	public DoubleParameter feedbackStiffness = new DoubleParameter("feedback coefficient", 0.5, 0, 50 );
	public BooleanParameter enablePostStabilization = new BooleanParameter("use post stabilization", false );
	public BooleanParameter enableCompliance = new BooleanParameter("enable compliance", true );
	public DoubleParameter compliance = new DoubleParameter("compliance", 1e-3, 1e-10, 1  );
	public static OptionParameter collectionCD = new OptionParameter("collision detection method", 0, "brute force", "bounding sphere hierarchy");
	
    /** Override restitution parameters when set true */
    public BooleanParameter restitutionOverride = new BooleanParameter( "restitution override, otherwise default 0", false );

    /** Restitution parameter for contact constraints */
    public DoubleParameter restitution = new DoubleParameter( "restitution (bounce), if override enabled ", 0.5, 0, 2 );

    /** Override friction parameters when set true */
    public BooleanParameter frictionOverride = new BooleanParameter( "Coulomb friction override, otherwise default 0.8", false );
    
    /** Coulomb friction coefficient for contact constraint */
    public DoubleParameter friction = new DoubleParameter("Coulomb friction, if override enabled", 0.1, 0, 2 );
    
    /** Number of iterations to use in projected Gauss Seidel solve */
    public IntParameter iterations = new IntParameter("iterations for PGS solve", 200, 1, 5000);

	public IntParameter iterationsInCollection = new IntParameter("iterations for PGS solve in collection", 1, 1, 5000);
        
    
    /**
     * @return controls for the collision processor
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Collision Processing Controls") );
        ((TitledBorder) vfp.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));
        
		vfp.add( shuffle.getControls() );
		
        vfp.add( iterations.getSliderControls() );
		vfp.add( iterationsInCollection.getSliderControls() );

		vfp.add( restitutionOverride.getControls() );
        vfp.add( restitution.getSliderControls(false) );
        
        vfp.add( frictionOverride.getControls() );
        vfp.add( friction.getSliderControls(false) );
        
		vfp.add( feedbackStiffness.getSliderControls(false) );
		vfp.add( enablePostStabilization.getControls() );
		vfp.add( enableCompliance.getControls() );
		vfp.add( compliance.getSliderControls(true) );
        return vfp.getPanel();
    }
}
