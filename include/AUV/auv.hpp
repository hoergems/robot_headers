#ifndef _AUV_ROBOT_
#define _AUV_ROBOT_
#include <string>
#include <iostream>
#include <assert.h>
#include <robot_headers/robot.hpp>
#include "AUVPropagator.hpp"

namespace shared
{
class AUV: public Robot
{
public:
    AUV(std::string robot_file);

    void createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
                                     std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const override;

    int getStateSpaceDimension() const override;

    int getDOF() const override;

    bool isTerminal(const frapu::RobotStateSharedPtr& state) const override;

    double distanceGoal(const frapu::RobotStateSharedPtr& state) const override;

    bool getObservation(const frapu::RobotStateSharedPtr& state,
                        std::vector<double>& observation) const override;

    bool getObservation(const frapu::RobotStateSharedPtr& state,
                        std::vector<double>& observationError,
                        std::vector<double>& observation) const override;

    void transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
                                     std::vector<double>& res) const override;

    bool makeActionSpace(bool normalizedActionSpace) override;

    bool makeObservationSpace(const shared::ObservationSpaceInfo& observationSpaceInfo) override;

    void getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                  std::vector<double>& control,
                                  double& duration,
                                  std::vector<Eigen::MatrixXd>& matrices) const override;

    void getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
                                      Eigen::MatrixXd& H,
                                      Eigen::MatrixXd& W) const override;

    void makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
                                     const frapu::RobotStateSharedPtr& collidingState,
                                     frapu::RobotStateSharedPtr& nextState) override;


    void updateViewer(const frapu::RobotStateSharedPtr& state,
                      std::vector<std::vector<double>>& particles,
                      std::vector<std::vector<double>>& particleColors) override;

    void setGravityConstant(double gravity_constant) override;

    virtual void makeProcessDistribution(Eigen::MatrixXd& mean,
                                         Eigen::MatrixXd& covariance_matrix,
                                         unsigned long seed) override;

    virtual void makeObservationDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covariance_matrix,
            unsigned long seed) override;

    double calcLikelihood(const frapu::RobotStateSharedPtr& state, std::vector<double>& observation) override;

    void updateRobot(const frapu::RobotStateSharedPtr& state) override;

private:
    double dim_x_;
    double dim_y_;
    double dim_z_;

};
}

#endif
