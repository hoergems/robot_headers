#ifndef __HEURISTIC_HPP__
#define __HEURISTIC_HPP__
#include <frapu_core/core.hpp>

namespace frapu
{

class HeuristicInfo
{
public:
    HeuristicInfo() {
	
    }
    
    frapu::BeliefSharedPtr currentBelief;
    
    frapu::RobotStateSharedPtr currentState;

};

/**class RRTHeuristicInfo: public HeuristicInfo {
public:
    RRTHeuristicInfo() {
	
    }
    
    frapu::RobotStateSharedPtr currentState = nullptr;
    
};*/

class Heuristic
{
public:
    Heuristic() {

    }

    virtual double operator()(frapu::HeuristicInfoSharedPtr &heuristicInfo) const = 0;

};

class RRTHeuristic: public Heuristic {
public:
    RRTHeuristic() {
	
    }
    
    virtual double operator()(frapu::HeuristicInfoSharedPtr &heuristicInfo) const override {
	return 0.0;
    }
};

}

#endif
