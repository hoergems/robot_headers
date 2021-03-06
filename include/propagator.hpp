#ifndef PROPAGATOR_HPP_
#define PROPAGATOR_HPP_
#include <vector>
#include "utils.hpp"

namespace frapu
{
class Propagator
{

public:
    Propagator();

    virtual bool propagateState(const std::vector<double>& currentState,
                                const std::vector<double>& control,
                                const std::vector<double>& control_error,
                                const double& duration,
                                const double& simulation_step_size,
                                std::vector<double>& result) = 0;

};
}

#endif
