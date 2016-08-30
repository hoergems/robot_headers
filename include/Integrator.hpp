#ifndef __INTEGRATOR_HPP__
#define __INTEGRATOR_HPP__
#include <vector>

namespace shared
{

typedef std::vector<double> state_type;
class Integrator
{
public:
    Integrator() {}

    virtual void do_integration(const state_type& x,
                                const state_type& control,
                                const state_type& control_error,
                                const state_type& int_times,
                                state_type& result) = 0;

    virtual void ode(const state_type& x , state_type& dxdt , double t) const = 0;

};
}

#endif
