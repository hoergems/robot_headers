#include <robot_headers/propagator.hpp>
#include <Eigen/Dense>

namespace frapu
{
class ManipulatorPropagatorLinear: public Propagator
{
public:
    ManipulatorPropagatorLinear();
    
    
    virtual bool propagateState(const std::vector<double>& currentState,
                                const std::vector<double>& control,
                                const std::vector<double>& control_error,
                                const double& duration,
                                const double& simulation_step_size,
                                std::vector<double>& result) override;
};
}
