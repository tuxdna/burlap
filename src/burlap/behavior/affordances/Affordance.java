/**
 * 
 */
package burlap.behavior.affordances;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.logicalexpressions.LogicalExpression;

/**
 * @author dabel
 *
 */
public class Affordance {

	private Map<AbstractGroundedAction, Integer> 	actionOptimalAffActiveCounts;
	private Map<AbstractGroundedAction,Integer>		totalActionOptimalCounts;
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
		this.actionOptimalAffActiveCounts = new HashMap<AbstractGroundedAction,Integer>();
		this.totalActionOptimalCounts = new HashMap<AbstractGroundedAction,Integer>();
		for (AbstractGroundedAction a: fullActionSet) {
			this.actionOptimalAffActiveCounts.put(a, 0);
			this.totalActionOptimalCounts.put(a, 0);
		}	
	}

	/**
	 * Computes the maximum likelihood estimate for the probability that this action is relevant
	 * @param action
	 * @return
	 */
	public double probActionIsRelevant(AbstractGroundedAction action) {
		double optimalActAffActive = (double) this.actionOptimalAffActiveCounts.get(action);
		double totalActOptimal = (double) this.totalActionOptimalCounts.get(action);

		return optimalActAffActive / totalActOptimal;
	}
	
	// --- Accessors ---
	
	public Map<AbstractGroundedAction, Integer> getActionOptimalAffActiveCounts() {
		return actionOptimalAffActiveCounts;
	}
	
	public Map<AbstractGroundedAction, Integer> getTotalActionOptimalCounts() {
		return totalActionOptimalCounts;
	}

	// --- Mutators ---
	
	public void setOptimalActionAffActiveCountMap(Map<AbstractGroundedAction, Integer> actionCounts) {
		this.actionOptimalAffActiveCounts = actionCounts;
	}
	
	public void setTotalOptimalActionCountMap(Map<AbstractGroundedAction,Integer> totalActionCounts) {
		this.totalActionOptimalCounts = totalActionCounts;
	}

	public void incrementActionOptimalAffActive(AbstractGroundedAction a) {
		Integer count = this.actionOptimalAffActiveCounts.get(a);
		this.actionOptimalAffActiveCounts.put(a, count + 1);
	}
	
	public void incrementTotalActionOptimal(AbstractGroundedAction a) {
		Integer count = this.totalActionOptimalCounts.get(a);
		this.totalActionOptimalCounts.put(a, count + 1);
	}
	
	public String toString() {
		return this.preCondition.toString() + "," + this.goalDescription.toString();
	}	
	
}
