/**
 * 
 */
package burlap.behavior.singleagent.planning.stochastic.rtdp;

import java.util.ArrayList;
import java.util.List;

import burlap.behavior.affordances.AffordancesController;
import burlap.behavior.singleagent.ValueFunctionInitialization;
import burlap.behavior.singleagent.planning.ValueFunctionPlanner;
import burlap.behavior.singleagent.planning.commonpolicies.AffordanceGreedyQPolicy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.statehashing.StateHashFactory;
import burlap.behavior.statehashing.StateHashTuple;
import burlap.debugtools.DPrint;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.logicalexpressions.LogicalExpression;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

/**
 * Implementation of Affordance-Aware [2] Real-time dynamic programming [1]. The planning algorithm uses a Q-value derived policy to sample rollouts in the domain. During
 * each step of the rollout, the current state has its value updated using the Bellman operator and the action for the current state
 * is selected using a greedy Q policy in which ties are randomly broken. Affordances are used to prune away irrelevant actions from the considered action set in each state.
 * <p/>
 * To ensure optimality, an optimistic value function initialization should be used. However, RTDP excels when a good value function initialization
 * (e.g., an admissible heuristic) can be provided.
 * 
 * Note: SoftAffordances are (as of yet) required to work with ARTDP - HardAffordances may be used only two value functions, where the first is initialized optimistically
 * to encourage exploration, and the second is initialized pessimistically. During rollouts, the optimistic value function is used, but during each bellman update, both
 * value functions are updated. This is not currently implemented, but is an option for those interested in HardAffordances. 
 * 
 * 1. Barto, Andrew G., Steven J. Bradtke, and Satinder P. Singh. "Learning to act using real-time dynamic programming." Artificial Intelligence 72.1 (1995): 81-138.
 * 2. David Abel & Gabriel Barth-Maron, James MacGlashan, Stefanie Tellex. "Toward Affordance-Aware Planning." Affordances in Vision for Cognitive Robotics. RSS '14.
 * 
 * @author James MacGlashan, David Abel
 *
 */
public class AffordanceRTDP extends RTDP {

	private AffordancesController affController;
	
	public AffordanceRTDP(Domain domain, RewardFunction rf, TerminalFunction tf, double gamma, StateHashFactory hashingFactory, double vInit, int numRollouts, double maxDelta, int maxDepth, AffordancesController affController, int minRolloutsRequiredForConvergance){
		super(domain, rf, tf, gamma,hashingFactory, vInit, numRollouts, maxDelta, maxDepth);
		this.VFPInit(domain, rf, tf, gamma, hashingFactory);
		this.affController = affController;
		this.numRollouts = numRollouts;
		this.maxDelta = maxDelta;
		this.maxDepth = maxDepth;
		this.rollOutPolicy = new AffordanceGreedyQPolicy(affController, this);
		this.valueInitializer = new ValueFunctionInitialization.ConstantValueFunctionInitialization(vInit);
		this.minNumRolloutsWithSmallValueChange = minRolloutsRequiredForConvergance;
	}
	
	public int planFromStateAndCount(State initialState) {
		return this.affordanceRTDP(initialState);
	}

	/**
	 * Runs Affordance Aware RTDP
	 * @param initialState
	 */
	private int affordanceRTDP(State initialState) {

		int totalStates = 0;
		int consecutiveSmallDeltas = 0;
		int numBellmanUpdates = 0;
		for(int i = 0; i < numRollouts; i++){
			List<GroundedAction> rolloutActions = new ArrayList<GroundedAction>();
			State curState = initialState;

			int nSteps = 0;
			double delta = 0;

			while(!this.tf.isTerminal(curState) && nSteps < this.maxDepth){
				StateHashTuple sh = this.hashingFactory.hashState(curState);
				
				// Select an action
				GroundedAction ga = (GroundedAction)this.rollOutPolicy.getAction(curState);
				rolloutActions.add(ga);

//				System.out.println("(affRTDP)Action : " + ga.actionName());
				
				// Update this state's value
				double curV = this.value(sh);
				double nV = ((ValueFunctionPlanner)this).performAffordanceBellmanUpdateOn(sh, affController);
				numBellmanUpdates++;
				delta = Math.max(Math.abs(nV - curV), delta); 

				// Take the action
				curState = ga.executeIn(curState);
				nSteps++;
			}

			totalStates += nSteps;
			
//			DPrint.cl(debugCode, "Pass: " + i + "; Num states: " + nSteps + " (total: " + totalStates + ")");
			
//			System.out.print("[affordanceRTDP] Action sequence: ");
//			for(GroundedAction ga : rolloutActions) {
//				System.out.print(ga + ",");
//			}
//			System.out.print("\n\n");
			
			if(delta < this.maxDelta){
				consecutiveSmallDeltas++;
				if(consecutiveSmallDeltas >= this.minNumRolloutsWithSmallValueChange){
					break;
				}
			}
			else{
				consecutiveSmallDeltas = 0;
			}
		}
		
		return numBellmanUpdates;
	}
	
}
