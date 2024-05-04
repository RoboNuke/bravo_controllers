// ros includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <moveit/planning_scene/planning_scene.h>

namespace bravo_controllers{
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
class Robot {
    public:
        Robot();
        ~Robot();
        Eigen::MatrixXd getJacobian();
        Eigen::MatrixXd getJacobian(Eigen::Vector3d ref_pt, std::string link_name);
        Eigen::MatrixXd getAnalyticJacobian(Eigen::MatrixXd j);
        Eigen::MatrixXd getPsudoInv(Eigen::MatrixXd j);
        Vector6d getJntAngles();
        Eigen::VectorXd getJntVels(); 
        std::vector<double> getGravity();
        std::vector<double> getTorques();
        std::vector<double> getTorques(bool with_accel);
        void setState(std::vector<double> jnt_angles, 
                        std::vector<double> jnt_vels); 
        void setState(std::vector<double> jnt_angles, 
                        std::vector<double> jnt_vels, 
                        std::vector<double> jnt_accels);
        void setState(sensor_msgs::JointState msg);
        std::vector<double> torqueToCurrent(std::vector<double> torques);
        std::vector<double> torqueToCurrent(Eigen::VectorXd torques);
        std::vector<double> currentToTorque(std::vector<double> currents);
        std::vector<std::string> joint_names_;

        std::vector<Eigen::Vector3d> getCollisionDir(std::vector<double> jnt_angles);

    private:
        moveit::core::RobotModelPtr model_;
        moveit::core::RobotState* state_;
        moveit::core::JointModelGroup* arm_group_;
        std::string ee_link_name_;

        dynamics_solver::DynamicsSolver* dyn_solver_;
        geometry_msgs::Vector3 grav_;

        // collision checking requirements
        planning_scene::PlanningScene* planning_scene_;
        collision_detection::CollisionResult collision_result_;
        collision_detection::CollisionRequest collision_request_;

};

}; //bravo controller namespace