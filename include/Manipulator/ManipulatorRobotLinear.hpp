#ifndef __MANIPULATOR_ROBOT_LINEAR_HPP__
#define __MANIPULATOR_ROBOT_LINEAR_HPP__
#include "ManipulatorRobot.hpp"
#include "ManipulatorPropagatorLinear.hpp"

namespace frapu
{

class ManipulatorRobotLinear: public ManipulatorRobot
{
public:
    ManipulatorRobotLinear(std::string robotFile, std::string configFile);

    virtual void getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                          const frapu::ActionSharedPtr& control,
                                          double& duration,
                                          std::vector<Eigen::MatrixXd>& matrices) const override;

    virtual void makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
            const frapu::RobotStateSharedPtr& collidingState,
            frapu::RobotStateSharedPtr& nextState) override;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                frapu::ObservationSharedPtr& observation) const override;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                std::vector<double>& observationError,
                                frapu::ObservationSharedPtr& observation) const override;

    virtual bool makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo) override;

    virtual void transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
            frapu::ObservationSharedPtr& res) const override;

    virtual void getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
            Eigen::MatrixXd& H,
            Eigen::MatrixXd& W) const override;
            
    virtual void updateViewer(const frapu::RobotStateSharedPtr& state,
                      std::vector<frapu::RobotStateSharedPtr>& particles,
                      std::vector<std::vector<double>>& particleColors) override;

    virtual frapu::RobotStateSharedPtr sampleInitialState() const override;
    
    virtual void setNewtonModel() override;
    
    virtual void setGravityConstant(double gravity_constant) override;

};



}

#endif
