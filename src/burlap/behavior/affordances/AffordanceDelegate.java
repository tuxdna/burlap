package burlap.behavior.affordances;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.GroundedProp;
import burlap.oomdp.core.PropositionalFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.logicalexpressions.LogicalExpression;
import burlap.oomdp.logicalexpressions.PFAtom;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;


public class AffordanceDelegate {

	 protected Affordance								affordance;
	 protected Map<String,AbstractGroundedAction>		fullActionSet;
	 protected boolean									active = false;
	 protected boolean									goalActive = false;

	 public AffordanceDelegate(Affordance affordance){
		 this.affordance = affordance;
	 }
	 
	 /**
	  * Updates the affordance to the current goal.
	  * @param currentGoal
	  */
	 public void setCurrentGoal(LogicalExpression currentGoal){
		 if(this.affordance.goalDescription.toString().equals(currentGoal.toString())) {
			 this.goalActive = true;
		 }
		 else {
			 this.goalActive = false;
		 }
	 }
	 
	 public void setOptimalActionAffActiveCountMap(Map<AbstractGroundedAction,Integer> totalActionCounts) {
		 this.affordance.setOptimalActionAffActiveCountMap(totalActionCounts);
	 }
	 
	 public void setTotalOptimalActionCountMap(Map<AbstractGroundedAction,Integer> totalActionCounts) {
		 this.affordance.setTotalOptimalActionCountMap(totalActionCounts);
	 }
	 
	 
	 // --- ACCESSORS ---
	
	 public Map<AbstractGroundedAction,Integer> getActionCounts() {
		 return this.affordance.getActionOptimalAffActiveCounts();
	 }
	 
	 public Map<AbstractGroundedAction,Integer> getTotalActionCounts() {
		 return this.affordance.getTotalActionOptimalCounts();
	 }
	 
	 /**
	  * Checks to see if the affordance is active
	  * @param s
	  * @return
	  */
	 public boolean isActive(State s) {
		 if(this.goalActive && this.affordance.preCondition.evaluateIn(s)) {
			 return true;
		 }
		 return false;
	 }
	 
	 public double probActionIsRelevant(AbstractGroundedAction action) {
		 return this.affordance.probActionIsRelevant(action);
	 }
	 
	 public Affordance getAffordance() {
		 return this.affordance;
	 }
	 
	 /**
	  * Loads a single affordance delegate given a string of the affordance data from a knowledge base file
	  * @param d: the domain in which to apply the affordances
	  * @param tempExtActions: a map from action names to action objects, containing the non-domain referenced actions (e.g. options, macroactions) 
	  * @param affString: a string containing the affordance data from the knowledge base file
	  * @param expertFlag: a boolean indicating whether or not this delegate should be treated as an expert affordance.	
	  * @return
	  */
	public static AffordanceDelegate load(Domain d, Map<String,Action> tempExtActions, String affString, boolean expertFlag) {
		String[] affLines = affString.split("\n");
		boolean readHeader = true;
		boolean readActCounts = true;
		
		LogicalExpression preCondition = null;
		LogicalExpression goal = null;
		
		int[] actionNumCounts = null;
		Map<AbstractGroundedAction,Integer> actionOptimalAffActiveCounts = new HashMap<AbstractGroundedAction,Integer>();
		Map<AbstractGroundedAction,Integer> actionTotalOptimalCounts = new HashMap<AbstractGroundedAction,Integer>();

		for(String line : affLines) {
			if(line.isEmpty()) {
				continue;
			}
			
			if (line.equals("===")) {
				// Reached the end of an affordance definition
				break;
			}
			
			if (line.equals("---")) {
				// Finished reading action counts -- ready to start reading action set sizes
				readActCounts = false;
				actionNumCounts = new int[actionOptimalAffActiveCounts.size()];
				continue;
			}
			
			String[] info = line.split(",");
			
			if (readHeader) {
				// We haven't read the header yet, so do that
				
				// TODO: Change to parsing a logical expression (instead of assuming a single pf)
				String preCondLE= info[0];
				String goalName = info[1];
				
				// -- Create PRECONDITION -- 
				PropositionalFunction preCondPF = d.getPropFunction(preCondLE);
				
				// Get grounded prop free variables
				String[] groundedPropPreCondFreeVars = makeFreeVarListFromObjectClasses(preCondPF.getParameterClasses());
				GroundedProp preCondGroundedProp = new GroundedProp(preCondPF, groundedPropPreCondFreeVars);
				preCondition = new PFAtom(preCondGroundedProp);
				
				// -- Create GOAL --
				PropositionalFunction goalPF = d.getPropFunction(goalName);
				
				// Get grounded prop free variables
				String[] groundedPropGoalFreeVars = makeFreeVarListFromObjectClasses(preCondPF.getParameterClasses());
				GroundedProp goalGroundedProp = new GroundedProp(goalPF, groundedPropGoalFreeVars);
				goal = new PFAtom(goalGroundedProp);
				
				readHeader = false;
				continue;
			}
			
			if (readActCounts) {
				// Read the action counts
				String actName = info[0];
				Integer count = Integer.parseInt(info[1]);
				
				Integer totalOptimalActCount;
				if(expertFlag) totalOptimalActCount = 900;
				totalOptimalActCount = Integer.parseInt(info[2]);
				
				// Get action free variables
				Action act = d.getAction(actName);
				// act is a temporally extended action, fetch it from the hashmap provided.
				if(act == null) {
					act = tempExtActions.get(actName);
				}
				
				assert(act!=null);
				String[] actionParams = makeFreeVarListFromObjectClasses(act.getParameterClasses());
				
				GroundedAction ga = new GroundedAction(act, actionParams);
				actionOptimalAffActiveCounts.put(ga, count);
				actionTotalOptimalCounts.put(ga, totalOptimalActCount);
			} 
		}
		
		// Create the Affordance
		List<AbstractGroundedAction> allActions = new ArrayList<AbstractGroundedAction>(actionOptimalAffActiveCounts.keySet()); 
		Affordance aff = new Affordance(preCondition, goal, allActions);
		aff.setOptimalActionAffActiveCountMap(actionOptimalAffActiveCounts);
		aff.setTotalOptimalActionCountMap(actionTotalOptimalCounts);
		
		AffordanceDelegate affDelegate = new AffordanceDelegate(aff);
		
		return affDelegate;
	}
	
	/**
	 * Gets a list of free variables given an OOMDP object's parameter object classes and order groups
	 * @param orderGroups
	 * @param objectClasses
	 * @return: String[] - a list of free variables
	 */
	public static String[] makeFreeVarListFromObjectClasses(String[] objectClasses){
		List<String> groundedPropFreeVariablesList = new ArrayList<String>();
		
		// TODO: improve variable binding stuff
		// Make variables free
		for(String objectClass : objectClasses){
			String freeVar = "?" + objectClass.charAt(0);
			groundedPropFreeVariablesList.add(freeVar);
		}
		String[] groundedPropFreeVars = new String[groundedPropFreeVariablesList.size()];
		groundedPropFreeVars = groundedPropFreeVariablesList.toArray(groundedPropFreeVars);
		
		return groundedPropFreeVars;
	}
	
	/**
	 * Prints the action counts for debugging purposes
	 */
	public void printCounts() {
		System.out.println("Affordance pred: " + this.affordance.preCondition.toString());
		for (AbstractGroundedAction a: this.affordance.getActionOptimalAffActiveCounts().keySet()) {
			System.out.println(a.toString() + ": " + this.affordance.getActionOptimalAffActiveCounts().get(a));
		}
	}
	
	
	/**
	 * Creates a string appropriate for writing the affordance to a file
	 * @return
	 */
	public String toFile() {
		String out = "";

		// Header information (what the affordance's PF and LGD are)
		out += this.affordance.preCondition.toString() + "," + this.affordance.goalDescription.toString() + "\n";
				
		// Add action counts (w/ total action counts
		
		for (AbstractGroundedAction a: this.affordance.getActionOptimalAffActiveCounts().keySet()) {
			out += a.actionName() + "," + this.affordance.getActionOptimalAffActiveCounts().get(a) + "," + this.affordance.getTotalActionOptimalCounts().get(a) + "\n";
		}

		out += "===\n";
		return out;
	}
	
	public String toString() {
		return "[" + this.affordance.preCondition + "," + this.affordance.goalDescription + "]";
	}
	 
}
