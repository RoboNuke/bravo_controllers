#include <bravo_controllers/bravo_gravity_comp.h>
#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Bravo_Gravity_Comp_Node");
    ros::NodeHandle nh;
    bravo_controllers::BravoGravityComp bgc(nh);
    ros::spin();
    return 0;
}