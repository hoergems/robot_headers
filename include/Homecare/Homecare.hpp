#ifndef __HOMECARE_HPP__
#define __HOMECARE_HPP__
#include <robot_headers/robot.hpp>

namespace frapu
{
    
class Homecare: public Robot
{
public:
    Homecare(std::string robotFile, std::string configFile);
    
    virtual std::string getName() const override;
    
    virtual bool propagateState(const frapu::RobotStateSharedPtr& state,
                                const frapu::ActionSharedPtr& action,
                                double duration,
                                double simulationStepSize,
                                frapu::RobotStateSharedPtr& result) override;

    virtual bool propagateState(const frapu::RobotStateSharedPtr& state,
                                const frapu::ActionSharedPtr& action,
                                const std::vector<double> controlError,
                                double duration,
                                double simulationStepSize,
                                frapu::RobotStateSharedPtr& result) override;
    
    /****** Virtual functions ******/    
    virtual bool makeStateSpace() override;

    virtual bool makeActionSpace(const frapu::ActionSpaceInfo& actionSpaceInfo) override;

    virtual bool makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo) override;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                frapu::ObservationSharedPtr& observation) const override;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                std::vector<double>& observationError,
                                frapu::ObservationSharedPtr& observation) const override;

    virtual void createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
            std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const override;

    virtual int getDOF() const override;

    virtual void makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
            const frapu::RobotStateSharedPtr& collidingState,
            frapu::RobotStateSharedPtr& nextState) override;

    virtual void getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                          const frapu::ActionSharedPtr& control,
                                          double& duration,
                                          std::vector<Eigen::MatrixXd>& matrices) const override;

    virtual void getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
            Eigen::MatrixXd& H,
            Eigen::MatrixXd& W) const override;

    virtual bool isTerminal(const frapu::RobotStateSharedPtr& state) const override;    

    virtual double distanceGoal(const frapu::RobotStateSharedPtr& state) const override;

    virtual void makeProcessDistribution(Eigen::MatrixXd& mean,
                                         Eigen::MatrixXd& covariance_matrix) override;

    virtual void makeObservationDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covariance_matrix) override;

    virtual void transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
            frapu::ObservationSharedPtr& res) const override;

    virtual void updateViewer(const frapu::RobotStateSharedPtr& state,
                              std::vector<frapu::RobotStateSharedPtr>& particles,
                              std::vector<std::vector<double>>& particleColors) override;
			      
    virtual frapu::RobotStateSharedPtr sampleInitialState() const override;
    
    virtual frapu::HeuristicFunctionSharedPtr makeHeuristicFunction() const override;
    
    /****** End of virtual functions ******/
    
private:
    std::vector<double> lowerStateLimits_;
    std::vector<double> upperStateLimits_;
    std::vector<double> lowerControlLimits_;
    std::vector<double> upperControlLimits_;
    
    frapu::RobotStateSharedPtr initialState_;

};

}


#endif
