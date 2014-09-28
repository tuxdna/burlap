/**
 * 
 */
package burlap.behavior.affordances;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.State;
import burlap.oomdp.logicalexpressions.LogicalExpression;
import cc.mallet.types.Dirichlet;

/**
 * @author dabel
 *
 */
public class Affordance {

	private Map<AbstractGroundedAction, Integer> 	actionCounts;
	private Map<AbstractGroundedAction,Integer>		totalActionCounts;
	public int numActivations;
	public LogicalExpression preCondition;
	public LogicalExpression goalDescription;
	
	/**
	 * Constructor Affordances. Maps a <Predicate,GoalDescription> pair to a subset of the action space
	 * @param preCond: the precondition for the affordance
	 * @param goalDescr: the goal description for the affordance
	 * @param actions: the list of all actions used for the affordance
	 */
	public Affordance(LogicalExpression preCond, LogicalExpression goalDescr, List<AbstractGroundedAction> actions) {
			this.preCondition = preCond;
			this.goalDescription = goalDescr;
			
			initCounts(actions);
	}
		
	/**
	 * Initiliazes counts for the dirichlet multinomial and dirichlet process.
	 */
	private void initCounts(List<AbstractGroundedAction> fullActionSet) {
		this.actionCounts = new HashMap<AbstractGroundedAction,Integer>();
		for (AbstractGroundedAction a: fullActionSet) {
			this.actionCounts.put(a, 0);
		}
		
	}

	/**
	 * Computes the maximum likelihood estimate for the probability that this action is relevant
	 * @param action
	 * @return
	 */
	public double probActionIsRelevant(AbstractGroundedAction action) {
		// TODO: add smoothing, dirichlet, etc.
		double counts = (double) this.actionCounts.get(action);
		double totalCounts = (double) this.totalActionCounts.get(action);

		return counts / totalCounts;
	}
	
	// --- Accessors ---
	
	public Map<AbstractGroundedAction, Integer> getActionCounts() {
		return actionCounts;
	}
	
	public Map<AbstractGroundedAction, Integer> getTotalActionCounts() {
		return totalActionCounts;
	}

	// --- Mutators ---
	
	public void setActionCounts(Map<AbstractGroundedAction, Integer> actionCounts) {
		this.actionCounts = actionCounts;
		for(AbstractGroundedAction aga : actionCounts.keySet()) {
			this.numActivations += actionCounts.get(aga);
		}
	}
	
	public void setTotalActionCountMap(Map<AbstractGroundedAction,Integer> totalActionCounts) {
		this.totalActionCounts = totalActionCounts;
	}

	public void incrementActionCount(AbstractGroundedAction a) {
		Integer count = this.actionCounts.get(a);
		this.actionCounts.put(a, count + 1);
	}
	
	public String toString() {
		return this.preCondition.toString() + "," + this.goalDescription.toString();
	}

	
	
}
