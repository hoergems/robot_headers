#ifndef _MANIPULATOR_ROBOT_HPP_
#define _MANIPULATOR_ROBOT_HPP_
#include <string>
#include <iostream>
#include <assert.h>
#include <boost/algorithm/string.hpp>
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <tinyxml.h>
#include <rbdl_interface/rbdl_interface.hpp>
#include <robot_headers/robot.hpp>
#include "ManipulatorPropagator.hpp"
#include "ManipulatorStateLimits.hpp"
#include "Kinematics.hpp"
#include "ManipulatorSerializer.hpp"

using std::cout;
using std::endl;

namespace frapu
{
struct RRTOptions {
    RRTOptions() {
	
    }
    
    bool continuousCollision;
    
    double planningVelocity;

};

struct Link {
    std::string name;

    bool active;

    std::vector<double> link_dimensions;

    std::vector<double> origin;

    double mass;

    std::vector<double> inertia_origin;

    std::vector<double> inertials;

    std::vector<double> com_origin;
};

struct Joint {
    std::string name;

    std::shared_ptr<frapu::Link> parent_link;

    std::shared_ptr<frapu::Link> child_link;
};

class ManipulatorRobot: public Robot
{
public:
    ManipulatorRobot(std::string robotFile, std::string configFile);
    
    virtual std::string getName() const override;

    void getLinkNames(std::vector<std::string>& link_names);

    void getLinkMasses(std::vector<std::string>& link, std::vector<double>& link_masses);

    void getLinkDimension(std::vector<std::string>& link, std::vector<std::vector<double>>& dimension);

    void getActiveLinkDimensions(std::vector<std::vector<double>>& dimensions);

    void getLinkPose(std::vector<std::string>& link, std::vector<std::vector<double>>& pose);

    void getLinkInertialPose(std::vector<std::string>& link, std::vector<std::vector<double>>& pose);

    void getLinkInertias(std::vector<std::string>& link, std::vector<std::vector<double>>& inertias);

    void getJointNames(std::vector<std::string>& joint_names);

    void getActiveJoints(std::vector<std::string>& joints) const;

    void getJointType(std::vector<std::string>& joint, std::vector<std::string>& type);

    void getJointOrigin(std::vector<std::string>& joints, std::vector<std::vector<double>>& origins);

    void getJointAxis(std::vector<std::string>& joints, std::vector<std::vector<int>>& axis);

    virtual void getLinearProcessMatrices(const frapu::RobotStateSharedPtr& state,
                                          const frapu::ActionSharedPtr& control,
                                          double& duration,
                                          std::vector<Eigen::MatrixXd>& matrices) const override;

    virtual bool isTerminal(const frapu::RobotStateSharedPtr& state) const override;

    virtual double distanceGoal(const frapu::RobotStateSharedPtr& state) const override;

    virtual frapu::HeuristicFunctionSharedPtr makeHeuristicFunction() const override;
    
    virtual void makeGoal() override;

    void getJointLowerPositionLimits(std::vector<std::string>& joints, std::vector<double>& joint_limits) const;

    void getJointUpperPositionLimits(std::vector<std::string>& joints, std::vector<double>& joint_limits) const;

    void getJointVelocityLimits(std::vector<std::string>& joints, std::vector<double>& joint_limits) const;

    void getJointTorqueLimits(std::vector<std::string>& joints, std::vector<double>& joint_limits) const;

    void getJointDamping(std::vector<std::string>& joints, std::vector<double>& damping);

    void getPositionOfLinkN(const std::vector<double>& joint_angles, const int& n, std::vector<double>& position);

    void getEndEffectorPosition(const std::vector<double>& joint_angles, std::vector<double>& end_effector_position) const;

    void getEndEffectorJacobian(const std::vector<double>& joint_angles,
                                std::vector<std::vector<double>>& ee_jacobian);

    int getStateSpaceDimension() const;

    virtual int getDOF() const override;

    virtual void makeNextStateAfterCollision(const frapu::RobotStateSharedPtr& previousState,
            const frapu::RobotStateSharedPtr& collidingState,
            frapu::RobotStateSharedPtr& nextState) override;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                frapu::ObservationSharedPtr& observation) const override;

    virtual bool getObservation(const frapu::RobotStateSharedPtr& state,
                                std::vector<double>& observationError,
                                frapu::ObservationSharedPtr& observation) const override;

    virtual bool makeStateSpace() override;

    virtual bool makeActionSpace(const frapu::ActionSpaceInfo& actionSpaceInfo) override;

    virtual bool makeObservationSpace(const frapu::ObservationSpaceInfo& observationSpaceInfo) override;

    virtual void transformToObservationSpace(const frapu::RobotStateSharedPtr& state,
            frapu::ObservationSharedPtr& res) const override;

    virtual void getLinearObservationDynamics(const frapu::RobotStateSharedPtr& state,
            Eigen::MatrixXd& H,
            Eigen::MatrixXd& W) const override;

    virtual void makeProcessDistribution(Eigen::MatrixXd& mean,
                                         Eigen::MatrixXd& covariance_matrix) override;

    virtual void makeObservationDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covariance_matrix) override;

    bool propagate_first_order(std::vector<double>& current_state,
                               std::vector<double>& control_input,
                               std::vector<double>& control_error,
                               std::vector<double>& nominal_state,
                               std::vector<double>& nominal_control,
                               double simulation_step_size,
                               double duration,
                               std::vector<double>& result);

    bool propagate_second_order(std::vector<double>& current_state,
                                std::vector<double>& control_input,
                                std::vector<double>& control_error,
                                std::vector<double>& nominal_state,
                                std::vector<double>& nominal_control,
                                double simulation_step_size,
                                double duration,
                                std::vector<double>& result);

    bool propagate_linear(std::vector<double>& current_state,
                          std::vector<double>& control_input,
                          std::vector<double>& control_error,
                          double duration,
                          std::vector<double>& result);

    //void createRobotCollisionStructures(const std::vector<double> &joint_angles, std::vector<fcl::OBB> &collision_structures);
    /**
     * Creates the collision object for the end effector
     */
    void createEndEffectorCollisionObject(const std::vector<double>& joint_angles,
                                          std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects);

    std::vector<std::shared_ptr<fcl::CollisionObject>>
            createEndEffectorCollisionObjectPy(const std::vector<double>& joint_angles);

    /**
     * Create the robot collision objects
     */
    virtual void createRobotCollisionObjects(const frapu::RobotStateSharedPtr state,
            std::vector<frapu::CollisionObjectSharedPtr>& collision_objects) const override;

    virtual bool checkSelfCollision(std::vector<std::shared_ptr<fcl::CollisionObject>>& collision_objects) const override;

    virtual bool checkSelfCollision(const frapu::RobotStateSharedPtr& state) const override;

    /**
     * Set the gravity constant
     */
    virtual void setGravityConstant(double gravity_constant) override;

    /**
     * Set the external force acting on the end-effector
     */
    void setExternalForce(double f_x,
                          double f_y,
                          double f_z,
                          double f_roll,
                          double f_pitch,
                          double f_yaw);

    /**
     * Set a joint acceleration limit
     */
    void setAccelerationLimit(double accelerationLimit);

    /**
     * Gets the end-effector velocity for a given state
     */
    void getEndEffectorVelocity(std::vector<double>& state,
                                std::vector<double>& ee_velocity);   
    
    virtual void setNewtonModel() override;

    virtual void updateViewer(const frapu::RobotStateSharedPtr& state,
                      std::vector<frapu::RobotStateSharedPtr>& particles,
                      std::vector<std::vector<double>>& particleColors) override;

    virtual frapu::RobotStateSharedPtr sampleInitialState() const override;


#ifdef USE_OPENRAVE
    /**
     * Set the size of the attached viewer
     */
    void setViewerSize(int x, int y);

    /**
     * Set the background color of the viewer
     */
    void setViewerBackgroundColor(double r, double g, double b);

    /**
     * Set the viewer camera transformation
     */
    void setViewerCameraTransform(std::vector<double>& rot, std::vector<double>& trans);

    void updateViewerValues(const std::vector<double>& current_joint_values,
                            const std::vector<double>& current_joint_velocities,
                            const std::vector<std::vector<double>>& particle_joint_values,
                            const std::vector<std::vector<double>>& particle_colors);


    /**
      * Add particles to the viewer which remain when the viewer is updated
    */
    void addPermanentViewerParticles(const std::vector<std::vector<double>>& particle_joint_values,
                                     const std::vector<std::vector<double>>& particle_colors);

    /**
      * Removes any permanent particles
      */
    void removePermanentViewerParticles();

    void setObstacleColor(std::string obstacle_name,
                          std::vector<double>& diffuse_color,
                          std::vector<double>& ambient_color);
#endif
protected:
    std::shared_ptr<Kinematics> kinematics_;
    
    std::vector<frapu::CollisionObjectSharedPtr> collision_objects_;
    
    std::vector<double> lowerStateLimits_;
    std::vector<double> upperStateLimits_;
    std::vector<double> lowerControlLimits_;
    std::vector<double> upperControlLimits_;

private:
    std::vector<frapu::Link> links_;

    std::vector<frapu::Joint> joints_;

    std::vector<std::string> link_names_;

    std::vector<std::string> active_link_names_;

    std::vector<std::string> joint_names_;

    std::vector<std::string> joint_types_;

    std::vector<std::string> active_joints_;

    std::vector<double> joint_dampings_;

    std::vector<std::vector<double>> joint_origins_;

    std::vector<std::vector<double>> link_origins_;

    std::vector<std::vector<double>> active_joint_origins_;

    std::vector<std::vector<int>> joint_axes_;

    std::vector<double> joint_torque_limits_;

    std::vector<double> active_joint_torque_limits_;

    std::vector<double> lower_joint_limits_;

    std::vector<double> upper_joint_limits_;

    std::vector<double> active_lower_joint_limits_;

    std::vector<double> active_upper_joint_limits_;

    std::vector<double> joint_velocity_limits_;

    std::vector<double> active_joint_velocity_limits_;

    std::vector<std::vector<int>> active_joint_axes_;

    std::vector<std::string> active_links_;

    std::vector<double> link_masses_;

    std::vector<std::vector<double>> link_inertia_origins_;

    std::vector<std::vector<double>> link_inertia_matrices_;

    std::vector<std::vector<double>> link_dimensions_;

    std::vector<std::vector<double>> active_link_dimensions_;

    bool initLinks(TiXmlElement* robot_xml);

    bool initJoints(TiXmlElement* robot_xml);

    unsigned int get_link_index(std::string& link_name);

    std::vector<double> process_origin_(TiXmlElement* xml);    

    void quatFromRPY(double& roll, double& pitch, double& y, std::vector<double>& quat);

    /**
     * Initialize the collision objects for the active links
     */
    void initCollisionObjects();

    
    

    std::shared_ptr<frapu::RBDLInterface> rbdl_interface_;

    frapu::RobotStateSharedPtr initialState_;
    
    RRTOptions rrtOptions;
};

}

#endif
