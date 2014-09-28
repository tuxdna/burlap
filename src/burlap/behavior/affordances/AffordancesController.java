package burlap.behavior.affordances;

import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;
import burlap.oomdp.logicalexpressions.LogicalExpression;
import burlap.oomdp.singleagent.Action;
import cc.mallet.types.Dirichlet;
import cc.mallet.types.Multinomial;

public class AffordancesController {

	private List<AffordanceDelegate> affordances = new ArrayList<AffordanceDelegate>();
	public List<AbstractGroundedAction> allActions = new ArrayList<AbstractGroundedAction>();
	public LogicalExpression currentGoal; 
	private boolean threshold = false; // True when we threshold probabilities to determine actions
	private boolean expertFlag = false;
	private double hardThreshold; 
	
	/**
	 * Create an affordances controller with the list of affordance delegates to use for planning.
	 * @param affs
	 */
	public AffordancesController(List<AffordanceDelegate> affs) {
		// Add each manually so that we add all relevant actions, too.
		for(AffordanceDelegate affDel : affs) {
			this.addAffordanceDelegate(affDel);
		}
	}
	
	/**
	 * Create an affordances controller with the list of affordance delegates to use for planning.
	 * @param affs
	 * @param hardFlag: a boolean indicating if the affordances should be treated in a crisp way
	 */
	public AffordancesController(List<AffordanceDelegate> affs, boolean hardFlag, boolean expertFlag) {
		// Add each manually so that we add all relevant actions, too.
		for(AffordanceDelegate affDel : affs) {
			this.addAffordanceDelegate(affDel);
		}
		this.threshold = hardFlag;
		this.expertFlag = expertFlag;
	}
	
	/**
	 * Update the current goal, and change the state of each affordance accordingly.
	 * @param currentGoal
	 */
	public void setCurrentGoal(LogicalExpression currentGoal){
		this.currentGoal = currentGoal;
		for(AffordanceDelegate aff : this.affordances){
			aff.setCurrentGoal(currentGoal);
		}
	}
	
	/**
	 * Add the affordance delegate to the controller. If it has any actions we don't have yet, add them.
	 * @param aff
	 */
	public void addAffordanceDelegate(AffordanceDelegate aff) {
		if(!this.affordances.contains(aff)) {
			this.affordances.add(aff);
		}
		for(AbstractGroundedAction action : aff.getAffordance().getActionOptimalAffActiveCounts().keySet()) {
			if(!this.allActions.contains(action)) {
				this.allActions.add(action);
			}
		}
		this.hardThreshold = 0.2 / allActions.size();
	}
	
	public void removeAffordance(AffordanceDelegate aff) {
		this.affordances.remove(aff);
	}
	
	// --- NEW STUFF ---
	
	/**
	 * Prunes actions according the affordances in the controller, and returns
	 * the action set suggested by the affordances. If the set is empty, return
	 * the full action set.
	 * @param s
	 * @return
	 */
	public List<AbstractGroundedAction> getPrunedActionsForState(State s) {
		Map<AbstractGroundedAction,Double> posterior = computePosterior(s);
		List<AbstractGroundedAction> result = new ArrayList<AbstractGroundedAction>();
		Random r = new SecureRandom();

		// If we are thresholding probabilities to determine the action set
		if(threshold && !expertFlag) {
			for(AbstractGroundedAction aga : posterior.keySet()) {
				if (posterior.get(aga) > this.hardThreshold) {
					result.add(aga);
				}
			}
		}
		else if(expertFlag) {
			// Expert action pruning (union of all active affordances are good actions)
			for(AbstractGroundedAction aga : posterior.keySet()) {
				if (posterior.get(aga) > 0.0) {
					result.add(aga);
				}
			}
		}
		// Otherwise, we sample from the posterior
		else {
			for(AbstractGroundedAction aga : posterior.keySet()) {
				if (posterior.get(aga) > (r.nextDouble())) {
					result.add(aga);
				}
			}
		}
		// If the set is empty, return the full action set.
		if(result.isEmpty()) {
			return this.allActions;
		}
		
		return result;
	}
	
	/**
	 * Computes the posterior distribution over the probability that each action is optimal
	 * in the given state @param s and the set of affordances in the controller.
	 * @param s
	 * @return
	 */
	private Map<AbstractGroundedAction,Double> computePosterior(State s) {
		Map<AbstractGroundedAction,Double> posterior = new HashMap<AbstractGroundedAction,Double>();
		
		// Compute probability that each action is optimal;
		for(AbstractGroundedAction action_i : this.allActions) {
			double numerator = computeNumerator(s, action_i);
			double denominator = computeDenominator(s, action_i);
			
			posterior.put(action_i, numerator / denominator);
		}
		return posterior;
	}
	
	/**
	 * Computes the numerator of the posterior
	 * @param s: State
	 * @param action: Action
	 * @return
	 */
	private double computeNumerator(State s, AbstractGroundedAction action) {
		double numerator = 1;
		for(AffordanceDelegate affDel : this.affordances) {
			if(affDel.isActive(s)) {
				// Prob active when action is optimal
				numerator *= affDel.probActionIsRelevant(action);
			}
			else {
				// Prob inactive when action is optimal
				numerator *= (1 - affDel.probActionIsRelevant(action));
			}
		}
		return numerator * computePrior(s, action);
	}
	
	/**
	 * Computes the denominator of the posterior
	 * @param s: State
	 * @param action: Action
	 * @return
	 */
	private double computeDenominator(State s, AbstractGroundedAction action) {
		// POSITIVE HYPOTHESIS (action is optimal)
		double positiveHypothesis = 1;
		for(AffordanceDelegate affDel : this.affordances) {
			if(affDel.isActive(s)) {
				// Prob active when action is optimal
				positiveHypothesis *= affDel.probActionIsRelevant(action);
			}
			else {
				// Prob inactive when action is optimal
				positiveHypothesis *= (1 - affDel.probActionIsRelevant(action));
			}
		}
		
		// NEGATIVE HYPOTHESIS (action is not optimal)
		double negativeHypothesis = 1;
		for(AffordanceDelegate affDel : this.affordances) {	
			
			// Count number times affordance was active and action was not optimal
			double countsOfAff = 0;
			for(AbstractGroundedAction aga : affDel.getActionCounts().keySet()) {
				countsOfAff += affDel.getActionCounts().get(aga);
			}
			double countsAffActionNotOptimal = countsOfAff - affDel.getActionCounts().get(action);
			
			// Count number of times the action was NOT optimal
			double totalStatesVisited = 0; 
			for(AbstractGroundedAction aga : affDel.getTotalActionCounts().keySet()) {
				totalStatesVisited += affDel.getTotalActionCounts().get(aga);
			}
			double countsActionNotOptimal = totalStatesVisited - affDel.getTotalActionCounts().get(action);
			
			if(affDel.isActive(s)) {
				negativeHypothesis *= countsAffActionNotOptimal / countsActionNotOptimal;
			}
			else {
				negativeHypothesis *= (1 - (countsAffActionNotOptimal / countsActionNotOptimal));
			}
		}

		return (positiveHypothesis * computePrior(s, action)) + (negativeHypothesis * (1 - computePrior(s, action)));
	}
	
	/**
	 * Computes the prior over action optimality
	 * @param s
	 * @param action_i
	 * @return
	 */
	private double computePrior(State s, AbstractGroundedAction action_i) {
		double totalStatesVisited = 0;
		Map<AbstractGroundedAction,Integer> totalActionCounts = this.affordances.get(0).getTotalActionCounts();
		for(AbstractGroundedAction aga : totalActionCounts.keySet()) {
			totalStatesVisited += totalActionCounts.get(aga);
		}

		return totalActionCounts.get(action_i) / totalStatesVisited;
	}
}
