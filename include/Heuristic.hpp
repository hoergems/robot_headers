#ifndef __HEURISTIC_HPP__
#define __HEURISTIC_HPP__
#include "trajectory.hpp"
#include "RewardModel.hpp"
#include <functional>

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
    
    double timeout;
    
    unsigned int currentStep;

};

class Heuristic
{
public:
    Heuristic();    

    virtual double operator()(frapu::HeuristicInfoSharedPtr& heuristicInfo) const = 0;

};

class RRTHeuristic: public Heuristic
{
public:
    RRTHeuristic(frapu::PathPlannerSharedPtr& pathPlanner,
		 frapu::CollisionCheckerSharedPtr &collisionChecker,
		 frapu::EnvironmentInfoSharedPtr &environmentInfo,
	         std::function<bool(frapu::RobotStateSharedPtr)> terminalFunction);

    virtual double operator()(frapu::HeuristicInfoSharedPtr& heuristicInfo) const override;


protected:
    frapu::PathPlannerSharedPtr pathPlanner_;
    
    frapu::CollisionCheckerSharedPtr collisionChecker_;
    
    frapu::EnvironmentInfoSharedPtr environmentInfo_;
    
    std::function<bool(frapu::RobotStateSharedPtr)> terminalFunction_;
    
};

}

#endif
