#ifndef _DUBIN_ROBOT_HPP_
#define _DUBIN_ROBOT_HPP_
#include <string>
#include <iostream>
#include <assert.h>
#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <tinyxml.h>
#include <rbdl_interface/rbdl_interface.hpp>
#include <robot_headers/robot.hpp>
#include <robot_headers/Dubin/DubinPropagator.hpp>
#include "DubinSerializer.hpp"

using std::cout;
using std::endl;

namespace frapu
{

struct Beacon {
public:
    Beacon():
        x_(0.0),
        y_(0.0) {

    }

    Beacon(double x, double y):
        x_(x),
        y_(y) {

    }

    double x_;
    double y_;
};

class DubinRobot: public Robot
{
public:
    DubinRobot(std::string robotFile, std::string configFile);
    
    frapu::RobotStateSharedPtr sampleInitialState() const override;

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
	    
    virtual void setupHeuristic(frapu::RewardModelSharedPtr &rewardModel) override;

private:

    double dim_x_;
    double dim_y_;
    double dim_z_;
    std::vector<Beacon> beacons_;
    double d_;

    std::vector<double> lowerStateLimits_;
    std::vector<double> upperStateLimits_;
    std::vector<double> lowerControlLimits_;
    std::vector<double> upperControlLimits_;
    
    frapu::RobotStateSharedPtr initialState_;

    void getLinearObservationMatrix(const std::vector<double>& state, Eigen::MatrixXd& H) const;

};

}

#endif
