#ifndef _AUV_ROBOT_
#define _AUV_ROBOT_
#include <string>
#include <iostream>
#include <assert.h>
#include <robot_headers/robot.hpp>
#include "AUVPropagator.hpp"
#include "AUVSerializer.hpp"

namespace frapu
{
class AUV: public Robot
{
public:
    AUV(std::string robotFile, std::string configFile);

    void createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
                                     std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const override;    

    int getDOF() const override;

    bool isTerminal(const frapu::RobotStateSharedPtr& state) const override;

    double distanceGoal(const frapu::RobotStateSharedPtr& state) const override;

    bool getObservation(const frapu::RobotStateSharedPtr& state,
                        frapu::ObservationSharedPtr& observation) const override;

    bool getObservation(const frapu::RobotStateSharedPtr& state,
                        std::vector<double>& observationError,
                        frapu::ObservationSharedPtr& observation) const override;

    void transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
                                     frapu::ObservationSharedPtr& res) const override;

    bool makeStateSpace() override;
    
    void makeGoal() override;

    bool makeActionSpace(const frapu::ActionSpaceInfo& actionSpaceInfo) override;

    bool makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo) override;

    void getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                  const frapu::ActionSharedPtr& control,
                                  double& duration,
                                  std::vector<Eigen::MatrixXd>& matrices) const override;

    void getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
                                      Eigen::MatrixXd& H,
                                      Eigen::MatrixXd& W) const override;

    void makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
                                     const frapu::RobotStateSharedPtr& collidingState,
                                     frapu::RobotStateSharedPtr& nextState) override;


    void updateViewer(const frapu::RobotStateSharedPtr& state,
                      std::vector<frapu::RobotStateSharedPtr>& particles,
                      std::vector<std::vector<double>>& particleColors) override;

    void setGravityConstant(double gravity_constant) override;

    virtual void makeProcessDistribution(Eigen::MatrixXd& mean,
                                         Eigen::MatrixXd& covariance_matrix,
                                         unsigned long seed) override;

    virtual void makeObservationDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covariance_matrix,
            unsigned long seed) override;

    double calcLikelihood(const frapu::RobotStateSharedPtr& state, 
                          const frapu::ObservationSharedPtr& observation) const override;

    void updateRobot(const frapu::RobotStateSharedPtr& state) override;
    
    frapu::RobotStateSharedPtr sampleInitialState() const override;
    
    virtual frapu::HeuristicFunctionSharedPtr makeHeuristicFunction() const override;

private:
    double dim_x_;
    double dim_y_;
    double dim_z_;

    std::vector<double> lowerStateLimits_;
    std::vector<double> upperStateLimits_;
    std::vector<double> lowerControlLimits_;
    std::vector<double> upperControlLimits_;
    
    frapu::RobotStateSharedPtr initialState_;

};
}

#endif
