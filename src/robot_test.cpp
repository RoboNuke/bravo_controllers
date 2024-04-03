#include<bravo_controllers/robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    bravo_controllers::Robot robot = bravo_controllers::Robot();
    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;

    //bravo_controllers::Robot robot();
    std::cout << "Finished main" << std::endl;
    Eigen::MatrixXd jacobian = robot.getJacobian();
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    robot.getGravity();
    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;


    std::vector<double> pos {0.0, 3.14, 1.57, 0.0, 1.57, 0.0};
    std::vector<double> vel(6,0.0);
    std::vector<double> accel(6, 0.0);

    robot.setState(pos, vel, accel);
    robot.getGravity();
    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;


    sensor_msgs::JointState js;
    std::vector<double> vels2 {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    js.name = robot.joint_names_;
    js.position = pos;
    js.velocity = vels2;
    js.effort = accel;
    robot.setState(js);
    robot.getTorques();

    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;
    std::cout << robot.getAnalyticJacobian(jacobian) << std::endl << std::endl;
    Eigen::MatrixXd pJ = robot.getPsudoInv(jacobian);
    std::cout << pJ * jacobian << std::endl;
    return 1;
}