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
    std::vector<double> pos2 {3.13991, 2.8505, 
                                0.6711, 3.1499, 
                                1.3161, 0.1521, 0.0010152395963668823};
    std::reverse(pos2.begin(), pos2.end());
    /*std::vector<double> pos2 {.001, 
                            32.9*3.14/180.0, 
                            85.2*3.14/180.0, 
                            180.4*3.14/180.0,
                            91.6*3.14/180.0,
                            149.3*3.14/180.0,
                            176.8*3.14/180.0};
    std::vector<double> pos2 {0.001, 
                            32.9*3.14/180,
                            162.0*3.14/180.0,
                            180.4*3.14/180.0,
                            167.6*3.14/180.0,
                            3.14,
                            176.9*3.14/180.0};*/
    std::vector<double> vel2(7,0.0);
    std::vector<double> accel2(7, 0.0);
    js.position = pos2;
    js.velocity = vel2;
    js.effort = accel2;
    robot.setState(js);
    std::vector<double> tout = robot.getGravity();
    std::vector<double> cout = robot.torqueToCurrent(tout);

    std::vector<double> cin {-170.26, 367.6, 954.27, 175.97, 38.6, 47.522};
    /*std::vector<double> cin {17.9, -17.2, 12.5, -12.7, 422.3, 1037.0, -14};*/
    /*std::vector<double> cin {18.9, -13.9, -191.7, -14.5, -521.9, 101.2, 0.79};*/
    //std::reverse(cin.begin(), cin.end());
    std::vector<double> tin = robot.currentToTorque(cin);

    std::cout << cin.size() << ", " << cout.size() << ", " << tin.size() << ", " << tout.size() << std::endl;

    for( int i = 0; i < 6; i++){
        std::cout << cin[i] << "\t" << cout[i] 
                            << "\t" << tin[i] 
                            << "\t" << tout[i] << std::endl;
    }

    //std::cout << robot.state_->hasAccelerations() << ", " << robot.state_->hasEffort() << std::endl;
    //std::cout << robot.getAnalyticJacobian(jacobian) << std::endl << std::endl;
    //Eigen::MatrixXd pJ = robot.getPsudoInv(jacobian);
    //std::cout << pJ * jacobian << std::endl;

    bravo_controllers::Vector6d tout_clone(tout.data());
    std::vector<double> tout2 = robot.torqueToCurrent(tout_clone);
    std::cout << "Tout2:";
    for(int i =0; i < 6; i++){
        std::cout << tout2[i] << ", ";
    }
    std::cout << std::endl;

    std::vector<double> vel3{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    js.velocity = vel3;
    robot.setState(js);

    Eigen::VectorXd jntVel = robot.getJntVels();
    std::cout << jntVel << std::endl;

    return 0;
}