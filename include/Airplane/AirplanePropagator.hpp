#ifndef __AIRPLANE_PROPAGATOR_HPP__
#define __AIRPLANE_PROPAGATOR_HPP__
#include <robot_headers/propagator.hpp>
#include <robot_headers/Airplane/AirplaneIntegrator.hpp>

namespace frapu
{
class AirplanePropagator: public Propagator
{
public:
    AirplanePropagator();

    virtual bool propagateState(const std::vector<double>& currentState,
                                const std::vector<double>& control,
                                const std::vector<double>& control_error,
                                const double& duration,
                                const double& simulation_step_size,
                                std::vector<double>& result) override;

    std::shared_ptr<AirplaneIntegrator> getIntegrator() const;

private:
    std::shared_ptr<AirplaneIntegrator> integrator_;

};



}

#endif
