// ros includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "bravo_controllers/set_gains.h"
#include <bravo_ft_sensor/FT_Interrupt.h>
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
        bool setGains(set_gains::Request &req,
                            set_gains::Response &res);
        bool demoComplianceControl(std_srvs::SetBool::Request &req,
                                    std_srvs::SetBool::Response &res);

        std_msgs::Float64MultiArray torqueToROSEffort(Vector6d t);
        void SetGains(std::vector<double> k_holder, std::vector<double> kp_holder);
        Eigen::Quaterniond EulerToQuat(double x, double y, double z);
        double clamp(double x, double min, double max);

        // utilities
        void checkSelfCollision(Eigen::MatrixXd Ja);
        void calcPoseError();
        std_msgs::Float64MultiArray toEffortCmd(Vector6d u);
        void ftInterruptCB(bravo_ft_sensor::FT_Interrupt msg);
        Vector6d getVecFromTwist(geometry_msgs::Twist twst);

    private:
        // ros crap
        ros::NodeHandle nh_;
        ros::Publisher eff_cmd_pub_;
        ros::Subscriber jnt_state_sub_;
        ros::Subscriber goal_pose_sub_;
        ros::Subscriber ee_pose_sub_;
        ros::Subscriber ft_interrupt_sub_;

        // services
        ros::ServiceServer toggle_srv_;
        ros::ServiceServer gain_srv_;
        ros::ServiceServer demo_srv_;
        ros::ServiceClient controller_switch_client_;
        ros::ServiceClient controller_list_client_;

        // actual useful stuff
        Robot* robot_;
        bool running_;
        bool sim_;
        bool in_demo_mode_;
        Eigen::MatrixXd kp_;
        Eigen::MatrixXd kd_;
        Vector3d goal_pose_;
        Eigen::Quaterniond goal_orient_;
        Vector3d ee_pose_;
        Eigen::Quaterniond ee_orient_;
        Vector6d pose_error_;
        Eigen::Quaterniond orient_error_;
        Vector6d dq_;
        Vector6d u_;
        std::string effort_controller_name_;
        std::string old_controller_name_;

        // error clipping stuff
        bool clip_error_;
        double trans_max_error_;
        double rot_max_error_;

        // collision ctuff
        bool check_self_collision_;
        bool stop_on_collision_;
        bool stop_till_new_goal_;
        Eigen::MatrixXd kp2d_; // weights on the Ja * dq term in look ahead est
        double look_ahead_dt_;
        double pos_repulse_;
        double rot_repulse_;

        // ft interrupt stuff
        bool with_ft_interrupt_;
        bool is_safe_;
        Vector3d safe_pose_;
        Eigen::Quaterniond safe_orient_;


}; // ComplianceController class

}; // bravo_controllers ns