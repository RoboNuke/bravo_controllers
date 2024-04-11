#include <bravo_controllers/compliance_controller.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Bravo_Compliance_Control_Node");
    ros::NodeHandle nh;
    bravo_controllers::ComplianceController cc(nh);
    ros::spin();
    return 0;
}