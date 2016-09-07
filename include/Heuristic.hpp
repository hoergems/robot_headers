#ifndef __HEURISTIC_HPP__
#define __HEURISTIC_HPP__
#include <frapu_core/core.hpp>
#include "trajectory.hpp"
#include "RewardModel.hpp"

namespace frapu
{

class HeuristicInfo
{
public:
    HeuristicInfo():
        currentBelief(nullptr),
        currentState(nullptr),
        rewardModel(nullptr) {

    }

    frapu::BeliefSharedPtr currentBelief;

    frapu::RobotStateSharedPtr currentState;

    frapu::RewardModelSharedPtr rewardModel;

};

class RRTHeuristicInfo: public HeuristicInfo
{
public:
    RRTHeuristicInfo():
        HeuristicInfo() {

    }

    double rrtTimeout = 10.0;
};

class Heuristic
{
public:
    Heuristic() {

    }

    virtual double operator()(frapu::HeuristicInfoSharedPtr& heuristicInfo) const = 0;    

};

class RRTHeuristic: public Heuristic
{
public:
    RRTHeuristic(frapu::PathPlannerSharedPtr &pathPlanner):
        Heuristic(),
        pathPlanner_(pathPlanner){

    }

    virtual double operator()(frapu::HeuristicInfoSharedPtr& heuristicInfo) const override {	
	if (!pathPlanner_) {
	    cout << "Path planner is null!!!!" << endl;
	}
	
	if (!heuristicInfo->currentState) {
	    cout << "CURRENT STATE IS NULL!!!" << endl;
	}
	
	if (!heuristicInfo->rewardModel) {
	    cout << "Warning: Reward model is nullptr. Did you forget to set a reward model in your HeuristicInfo? Returning 0.0" << endl;
	    return 0.0;
	} 
	
	frapu::RewardModelSharedPtr rewardModel = heuristicInfo->rewardModel;
        	
        frapu::TrajectorySharedPtr trajectory =
            pathPlanner_->solve(heuristicInfo->currentState, static_cast<RRTHeuristicInfo*>(heuristicInfo.get())->rrtTimeout);
        return 0.0;
    }
    
    
protected:
    frapu::PathPlannerSharedPtr pathPlanner_;
};

}

#endif
