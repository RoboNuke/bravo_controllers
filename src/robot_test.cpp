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
    //std::cout << "Finished main" << std::endl;
    //Eigen::MatrixXd jacobian = robot.getJacobian();
    //ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    //robot.getGravity();
    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;


    //std::vector<double> pos {0.0, 3.14, 1.57, 0.0, 1.57, 0.0};
    //std::vector<double> vel(6,0.0);
   // std::vector<double> accel(6, 0.0);

    //robot.setState(pos, vel, accel);
    //robot.getGravity();
    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;


    sensor_msgs::JointState js;
    std::vector<std::string> joint_names {"bravo_axis_a","bravo_axis_b",
                                          "bravo_axis_c","bravo_axis_d",
                                          "bravo_axis_e","bravo_axis_f",
                                          "bravo_axis_g"};
    js.name = joint_names;
    std::vector<double> pos2 {0.0010152395963668823, 0.15205837786197662, 
                                1.3160640001296997, 
                                3.1499903202056885, 0.6710423231124878, 
                                2.850409984588623, 3.1400632858276367};
    //std::reverse(pos2.begin(), pos2.end());
    std::vector<double> vel2(7,0.0);
    std::vector<double> accel2(7, 0.0);
    js.position = pos2;
    js.velocity = vel2;
    js.effort = accel2;
    robot.setState(js);
    std::vector<double> tout = robot.getTorques();
    std::vector<double> cout = robot.torqueToCurrent(tout);

    std::vector<double> cin {-97.32202911376953, 71.41635131835938, 170.3534393310547, 
                            948.214111328125, 438.65057373046875, 35.26539611816406};
    std::reverse(cin.begin(), cin.end());
    std::vector<double> tin = robot.currentToTorque(cin);

    for( int i = 0; i < 6; i++){
        std::cout << cin[i] << "\t" << cout[i] 
                            << "\t" << tin[i] 
                            << "\t" << tout[i] << std::endl;
    }

    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;
    //std::cout << robot.getAnalyticJacobian(jacobian) << std::endl << std::endl;
    //Eigen::MatrixXd pJ = robot.getPsudoInv(jacobian);
    //std::cout << pJ * jacobian << std::endl;
    return 1;
}