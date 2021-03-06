#ifndef ROBOT_INTERFACE_HPP_
#define ROBOT_INTERFACE_HPP_
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "propagator.hpp"
#include <random>
#include "Distributions.hpp"
#include "DiscreteObservationSpace.hpp"
#include "ContinuousObservationSpace.hpp"
#include "ActionSpace.hpp"
#include "Action.hpp"
#include <frapu_core/core.hpp>
#include "RobotState.hpp"
#include "StateSpace.hpp"
#include "Serializer.hpp"
#include "propagator.hpp"

#ifdef USE_OPENRAVE
#include <viewer_interface/viewer_interface.hpp>
#endif

using std::cout;
using std::endl;

namespace frapu
{

class Robot: public frapu::InterfaceBase
{
public:
    Robot(std::string robotFile, std::string configFile);

    virtual ~Robot() = default;

    virtual bool propagateState(const frapu::RobotStateSharedPtr& state,
                                const frapu::ActionSharedPtr& action,
                                double duration,
                                double simulationStepSize,
                                frapu::RobotStateSharedPtr& result);

    virtual bool propagateState(const frapu::RobotStateSharedPtr& state,
                                const frapu::ActionSharedPtr& action,
                                const std::vector<double> controlError,
                                double duration,
                                double simulationStepSize,
                                frapu::RobotStateSharedPtr& result);

    // ******************** Virtual methods **************************
    virtual std::string getName() const = 0;
    
    virtual bool makeStateSpace() = 0;

    virtual bool makeActionSpace(const frapu::ActionSpaceInfo& actionSpaceInfo) = 0;

    virtual bool makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo) = 0;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                frapu::ObservationSharedPtr& observation) const = 0;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                std::vector<double>& observationError,
                                frapu::ObservationSharedPtr& observation) const = 0;

    virtual void createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
            std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const = 0;

    virtual int getDOF() const = 0;

    virtual void makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
            const frapu::RobotStateSharedPtr& collidingState,
            frapu::RobotStateSharedPtr& nextState) = 0;

    virtual void getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                          const frapu::ActionSharedPtr& control,
                                          double& duration,
                                          std::vector<Eigen::MatrixXd>& matrices) const = 0;

    virtual void getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
            Eigen::MatrixXd& H,
            Eigen::MatrixXd& W) const = 0;

    virtual bool isTerminal(const frapu::RobotStateSharedPtr& state) const = 0;    

    virtual double distanceGoal(const frapu::RobotStateSharedPtr& state) const = 0;

    virtual void makeProcessDistribution(Eigen::MatrixXd& mean,
                                         Eigen::MatrixXd& covariance_matrix) = 0;

    virtual void makeObservationDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covariance_matrix) = 0;

    virtual void transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
            frapu::ObservationSharedPtr& res) const = 0;

    virtual void updateViewer(const frapu::RobotStateSharedPtr& state,
                              std::vector<frapu::RobotStateSharedPtr>& particles,
                              std::vector<std::vector<double>>& particleColors) = 0;
			      
    virtual frapu::RobotStateSharedPtr sampleInitialState() const = 0;
    
    virtual frapu::HeuristicFunctionSharedPtr makeHeuristicFunction() const = 0;

    //****************** End of virtual methods ***********************
    virtual bool isValid(const frapu::RobotStateSharedPtr& state) const;
    
    virtual void makeGoal();
    
    frapu::GoalSharedPtr getGoal() const;

    virtual void updateRobot(const frapu::RobotStateSharedPtr& state);

    virtual void setGravityConstant(double gravity_constant);

    void setEnvironmentInfo(frapu::EnvironmentInfoSharedPtr& environmentInfo);

    virtual void setGoalArea(std::vector<double>& goal_position, double& goal_radius);
    
    virtual void getGoalArea(std::vector<double> &goalArea) const;

    virtual void setNewtonModel();

    virtual void enforceConstraints(bool enforce);

    virtual bool constraintsEnforced();

    virtual bool checkSelfCollision(std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const;

    virtual bool checkSelfCollision(const frapu::RobotStateSharedPtr& state) const;

    std::shared_ptr<Eigen::Distribution<double>> getProcessDistribution() const;

    std::shared_ptr<Eigen::Distribution<double>> getObservationDistribution() const;

    /**
     * Calculates the likelihood of 'observation' given 'state'
     */
    virtual double calcLikelihood(const frapu::RobotStateSharedPtr& state,
                                  const frapu::ObservationSharedPtr& observation) const;

    frapu::StateSpaceSharedPtr getStateSpace() const;

    frapu::ObservationSpaceSharedPtr getObservationSpace() const;

    frapu::ActionSpaceSharedPtr getActionSpace() const;
    
    frapu::SerializerSharedPtr getSerializer() const;
    
    std::vector<frapu::RobotStateSharedPtr> getGoalStates() const;
    
    void setGoalStates(std::vector<frapu::RobotStateSharedPtr> &goalStates);
    
    void setControlDuration(double control_duration);
    
    double getControlDuration() const;
    
    std::vector<frapu::RobotStateSharedPtr> loadGoalStatesFromFile(std::string &filename) const;

    /*** Methods for viewer interface ***/
    virtual void setupViewer(std::string model_file, std::string environment_file);

    virtual void resetViewer(std::string model_file, std::string environment_file);

    void getCameraImage(std::vector<uint8_t>& image, int width, int height);

    virtual void setParticlePlotLimit(unsigned int particle_plot_limit);

    virtual void addBox(std::string name, std::vector<double> dims);

    virtual void removeBox(std::string name);
    
    void setRandomEngine(std::default_random_engine &randomEngine);

protected:
    std::default_random_engine randomEngine_;
    
    bool constraints_enforced_;

    std::string robot_file_;

    std::shared_ptr<frapu::Propagator> propagator_;

    std::vector<double> goal_position_;
    
    std::vector<frapu::RobotStateSharedPtr> goalStates_;

    double goal_radius_;
    
    double control_duration_;

    std::shared_ptr<Eigen::Distribution<double>> process_distribution_;

    std::shared_ptr<Eigen::Distribution<double>> observation_distribution_;

    frapu::StateSpaceSharedPtr stateSpace_;

    frapu::ObservationSpaceSharedPtr observationSpace_;

    frapu::ActionSpaceSharedPtr actionSpace_;

    frapu::EnvironmentInfoSharedPtr environmentInfo_;
    
    frapu::SerializerSharedPtr serializer_;
    
    frapu::GoalSharedPtr goal_;

#ifdef USE_OPENRAVE
    std::shared_ptr<frapu::ViewerInterface> viewer_;
#else
    std::shared_ptr<double> viewer_;
#endif

};

/**struct RobotWrapper: Robot, boost::python::wrapper<Robot> {
public:
    RobotWrapper(std::string robotFile, std::string configFile):
        Robot(robotFile, configFile) {

    }

    int getDOF() const {
        return this->get_override("getDOF")();
    }

    int getStateSpaceDimension() const {
        return this->get_override("getStateSpaceDimension")();
    }

    unsigned int getControlSpaceDimension() const {
        return this->get_override("getControlSpaceDimension")();
    }

    void addBox(std::string name, std::vector<double> dims) {
        this->get_override("addBox")(name, dims);
    }

    void removeBox(std::string name) {
        this->get_override("removeBox")(name);
    }

    void createRobotCollisionObjects(const std::vector<double>& state,
                                     std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const {
        this->get_override("createRobotCollisionObjects")(state, collision_objects);
    }

    void getControlLimits(std::vector<double>& lowerLimits, std::vector<double>& upperLimits) const {
        this->get_override("getControlLimits")(lowerLimits, upperLimits);
    }

    void getLinearProcessMatrices(const std::vector<double>& state,
                                  std::vector<double>& control,
                                  double& duration,
                                  std::vector<Eigen::MatrixXd>& matrices) const {
        this->get_override("getLinearProcessMatrices")(state, control, duration, matrices);
    }

    void setStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix) {
        this->get_override("setStateCovarianceMatrix")(state_covariance_matrix);
    }

    void getStateCovarianceMatrix(Eigen::MatrixXd& state_covariance_matrix) const {
        this->get_override("getStateCovarianceMatrix")(state_covariance_matrix);
    }

    void setObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix) {
        this->get_override("setStateCovarianceMatrix")(observation_covariance_matrix);
    }

    void getObservationCovarianceMatrix(Eigen::MatrixXd& observation_covariance_matrix) const {
        this->get_override("getObservationCovarianceMatrix")(observation_covariance_matrix);
    }

    bool getObservation(std::vector<double>& state, std::vector<double>& observation) const {
        this->get_override("getObservation")(state, observation);
    }

    bool isTerminal(std::vector<double>& state) const {
        return this->get_override("isTerminal")(state);
    }

    double distanceGoal(std::vector<double>& state) const {
        return this->get_override("distanceGoal")(state);
    }

    bool enforceConstraints(std::vector<double>& state) const {
        return this->get_override("enforceConstraints")(state);
    }

    void enforceConstraints(bool enforce) {
        this->get_override("enforceConstraints")(enforce);
    }

    void updateViewer(std::vector<double>& state,
                      std::vector<std::vector<double>>& particles,
                      std::vector<std::vector<double>>& particle_colors) {
        this->get_override("updateViewer")(state, particles, particle_colors);
    }

    void makeNextStateAfterCollision(std::vector<double>& previous_state,
                                     std::vector<double>& colliding_state,
                                     std::vector<double>& next_state) {
        this->get_override("makeNextStateAfterCollision")(previous_state, colliding_state, next_state);
    }

    void sampleRandomControl(std::vector<double>& control, std::default_random_engine* randGen) {
        this->get_override("sampleRandomControl")(control, randGen);
    }

    void setGravityConstant(double gravity_constant) {
        this->get_override("setGravityConstant")(gravity_constant);
    }

    void setNewtonModel() {
        this->get_override("setNewtonModel")();
    }

    bool makeActionSpace(bool normalizedActionSpace) {
        this->get_override("makeActionSpace")(normalizedActionSpace);
    }

    bool makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo) {
        this->get_override("makeObservationSpace")(observationSpaceInfo);
    }

    void transformToObservationSpace(std::vector<double>& state, std::vector<double>& res) const {
        this->get_override("transformToObservationSpace")(state, res);
    }

    void getLinearObservationDynamics(const std::vector<double>& state,
                                      Eigen::MatrixXd& H,
                                      Eigen::MatrixXd& W) const {
        this->get_override("getLinearObservationDynamics")(state, H, W);
    }

    bool getObservation(std::vector<double>& state, std::vector<double>& observationError, std::vector<double>& observation) const {
        this->get_override("getObservation")(state, observationError, observation);
    }

    void getCameraImage(std::vector<uint8_t>& image, int width, int height) {
        this->get_override("getCameraImage")(image, width, height);
    }

    void makeProcessDistribution(Eigen::MatrixXd& mean,
                                 Eigen::MatrixXd& covariance_matrix,
                                 unsigned long seed) {
        this->get_override("makeProcessDistribution")(mean, covariance_matrix, seed);
    }

    void makeObservationDistribution(Eigen::MatrixXd& mean,
                                     Eigen::MatrixXd& covariance_matrix,
                                     unsigned long seed) {
        this->get_override("makeObservationDistribution")(mean, covariance_matrix, seed);
    }

    double calcLikelihood(std::vector<double>& state, std::vector<double>& observation) {
        this->get_override("calcLikelihood")(state, observation);
    }

};*/

}

#endif
