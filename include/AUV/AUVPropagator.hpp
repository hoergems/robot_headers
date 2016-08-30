#ifndef __AUV_PROPAGATOR_HPP__
#define __AUV_PROPAGATOR_HPP__
#include <memory>
#include <unistd.h>
#include <robot_headers/propagator.hpp>
#include <robot_headers/DiscreteActionSpace.hpp>

namespace shared
{

class AUVPropagator: public Propagator
{
public:
    AUVPropagator();

    virtual bool propagateState(const std::vector<double>& currentState,
                                const std::vector<double>& control,
                                const std::vector<double>& control_error,
                                const double& duration,
                                const double& simulation_step_size,
                                std::vector<double>& result) override;
				
    void setActionSpace(std::shared_ptr<shared::ActionSpace> &actionSpace);
    
private:
    std::shared_ptr<shared::ActionSpace> actionSpace_;
    
};

}

#endif
