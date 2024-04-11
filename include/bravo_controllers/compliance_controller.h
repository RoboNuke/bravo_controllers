// ros includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <std_msgs/Float64MultiArray.h>
// custom includes
#include<bravo_controllers/robot.h>

namespace bravo_controllers{

class ComplianceController{
    public:
        ComplianceController(ros::NodeHandle nh);

        // topic callbacks
        void jntStateCallback(sensor_msgs::JointState msg);
        void goalCallback(std_msgs::Float64MultiArray msg);
        void EEPoseCallback(sensor_msgs::JointState msg);

        // services
        bool toggleComplianceControl(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res);

        std_msgs::Float64MultiArray torqueToROSEffort(Vector6d t);


    private:
        // ros crap
        ros::NodeHandle nh_;
        ros::Publisher eff_cmd_pub_;
        ros::Subscriber jnt_state_sub_;
        ros::Subscriber goal_pose_sub_;
        ros::Subscriber ee_pose_sub_;

        // services
        ros::ServiceServer toggle_srv_;
        ros::ServiceClient controller_switch_client_;
        ros::ServiceClient controller_list_client_;

        // actual useful stuff
        Robot robot_;
        bool running_;
        Eigen::MatrixXd kp_;
        Eigen::MatrixXd kd_;
        Vector6d goal_pose_;
        Vector6d ee_pose_;
        Vector6d pose_error_;
        Vector6d dq_;
        Vector6d u_;
        std::string effort_controller_name_;
        std::string old_controller_name_;

}; // ComplianceController class

}; // bravo_controllers ns