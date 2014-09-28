package burlap.behavior.singleagent.planning.stochastic.rtdp;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import burlap.behavior.affordances.AffordancesController;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.singleagent.ValueFunctionInitialization;
import burlap.behavior.singleagent.planning.ValueFunctionPlanner;
import burlap.behavior.statehashing.StateHashFactory;
import burlap.behavior.statehashing.StateHashTuple;
import burlap.debugtools.DPrint;
import burlap.debugtools.RandomFactory;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;


/**
 * An extension of BoundedRTDP that prunes actions using affordances
 */
public class AffordanceBoundedRTDP extends BoundedRTDP {

	private AffordancesController affController;
	
	/**
	 * Initializes.
	 * @param domain the domain in which to plan
	 * @param rf the reward function
	 * @param tf the terminal state function
	 * @param gamma the discount factor
	 * @param hashingFactory the state hashing factor to use
	 * @param lowerVInit the value function lower bound initialization
	 * @param upperVInit the value function upper bound initialization
	 * @param maxDiff the max permitted difference in value function margin to permit planning termination. This value is also used to prematurely stop a rollout if the next state's margin is under this value.
	 * @param maxRollouts the maximum number of rollouts permitted before planning is forced to terminate. If set to -1 then there is no limit.
	 * @param affController the set of affordances to use for pruning.
	 */
	public AffordanceBoundedRTDP(Domain domain, RewardFunction rf, TerminalFunction tf, double gamma, StateHashFactory hashingFactory, 
			ValueFunctionInitialization lowerVInit, ValueFunctionInitialization upperVInit, double maxDiff, int maxRollouts, AffordancesController affController){
		super(domain, rf, tf, gamma, hashingFactory, lowerVInit, upperVInit, maxDiff, maxRollouts);
		this.affController = affController;

	}
	
	/**
	 * Returns the maximum Q-value entry for the given state with ties broken randomly. 
	 * @param s the query state for the Q-value
	 * @return the maximum Q-value entry for the given state with ties broken randomly. 
	 */
	protected QValue maxQ(State s){
		
		List<QValue> qs = this.getAffordanceQs(s, this.affController);
		double max = Double.NEGATIVE_INFINITY;
		List<QValue> maxQs = new ArrayList<QValue>(qs.size());
		
		for(QValue q : qs){
			if(q.q == max){
				maxQs.add(q);
			}
			else if(q.q > max){
				max = q.q;
				maxQs.clear();
				maxQs.add(q);
			}
		}
		
		//return random max
		int rint = RandomFactory.getMapped(0).nextInt(maxQs.size());
		
		return maxQs.get(rint);
	}
	
	
	
	/**
	 * A tuple class for a hashed state and the expected value function margin/gap of a the source transition.
	 * @author James MacGlashan
	 *
	 */
	protected static class StateSelectionAndExpectedGap{
		
		/**
		 * The selected state
		 */
		public StateHashTuple sh;
		
		/**
		 * The expected margin/gap of the value function from the source transition
		 */
		public double expectedGap;
		
		/**
		 * Initializes.
		 * @param sh The selected state
		 * @param expectedGap The expected margin/gap of the value function from the source transition
		 */
		public StateSelectionAndExpectedGap(StateHashTuple sh, double expectedGap){
			this.sh = sh;
			this.expectedGap = expectedGap;
		}
		
	}
	
	

}
