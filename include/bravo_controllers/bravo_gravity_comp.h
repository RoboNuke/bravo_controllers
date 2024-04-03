// ros includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// custom includes
#include <bravo_controllers/robot.h>

namespace bravo_controllers{

class BravoGravityComp{
    public:
        BravoGravityComp(ros::NodeHandle nh);

        // topic callbacks
        void jntStateCallback(sensor_msgs::JointState msg);

        // services
        bool toggleGravityComp(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res);


    private:
        // ros crap
        ros::NodeHandle nh_;
        ros::Publisher eff_cmd_pub_;
        ros::Subscriber jnt_state_sub_;

        // services
        ros::ServiceServer toggle_srv_;
        ros::ServiceClient controller_switch_client_;
        ros::ServiceClient controller_list_client_;

        // actual useful stuff
        Robot robot_;
        bool running_;
        std::string effort_controller_name_;
        std::string old_controller_name_;

}; // BravoGravityComp class

}; // bravo_controllers ns