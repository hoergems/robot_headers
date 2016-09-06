#ifndef __AIRPLANE_INTEGRATOR_HPP__
#define __AIRPLANE_INTEGRATOR_HPP__
#include <Eigen/Dense>
#include <list>
#include <boost/timer.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/stepper/adams_bashforth.hpp>
#include <boost/numeric/odeint/stepper/adams_bashforth_moulton.hpp>

//#include <boost/numeric/odeint/stepper/runge_kutta_fehlberg78.hpp>

#include <boost/numeric/odeint/stepper/euler.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <rbdl_interface/rbdl_interface.hpp>
#include <robot_headers/Integrator.hpp>

namespace pl = std::placeholders;
using namespace Eigen;
using namespace boost::numeric::odeint;

namespace frapu
{
class AirplaneIntegrator: public Integrator
{
public:
    AirplaneIntegrator();
    
    virtual void do_integration(const state_type& x,
                                const state_type& control,
                                const state_type& control_error,
                                const state_type& int_times,
                                state_type& result) override;

    virtual void ode(const state_type& x , state_type& dxdt , double t) const override;
    
    void setup(double cl, double cd, double k, double g);
    
private:
    double tauDest_;
    double alphaDest_;
    double betaDest_;
    
    double cl_;
    double cd_;
    double k_;
    double g_;

};
}

#endif
