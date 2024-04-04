#include <bravo_controllers/robot.h>

namespace bravo_controllers{
Robot::~Robot(){
    delete state_;
    delete arm_group_;
    
}
Robot::Robot()
{
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    
    model_ = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", model_->getModelFrame().c_str());

    state_ = new moveit::core::RobotState(model_);
    state_->setToDefaultValues();
    arm_group_ = model_->getJointModelGroup("arm");

    joint_names_ = arm_group_->getVariableNames();

    std::cout << "Joints in order: " << std::endl;
    for(int i = 0; i < joint_names_.size(); i++){
        std::cout << "\t" << joint_names_[i] << std::endl;
    }
    arm_group_->printGroupInfo();

    ee_link_name_ = arm_group_->getLinkModelNames().back();
    grav_.x = 0.0;
    grav_.y = 0.0;
    grav_.z = -9.8;
    dyn_solver_ = new dynamics_solver::DynamicsSolver(model_, "arm", grav_);
}

Eigen::MatrixXd Robot::getJacobian(){
    Eigen::Vector3d ref_pt(0,0,0);
    return getJacobian(ref_pt, ee_link_name_);
}

Eigen::MatrixXd Robot::getJacobian(Eigen::Vector3d ref_pt, std::string link_name){
    Eigen::MatrixXd jacobian;
    state_->getJacobian(
            arm_group_,
            state_->getLinkModel(link_name),
            ref_pt, 
            jacobian
        );
    return jacobian;
}
void Robot::setState(std::vector<double> jnt_angles, 
                std::vector<double> jnt_vels)
{
    state_->setJointGroupPositions(arm_group_, jnt_angles);
    state_->setJointGroupVelocities(arm_group_, jnt_vels);        
}

void Robot::setState(std::vector<double> jnt_angles, 
                std::vector<double> jnt_vels, 
                std::vector<double> jnt_accels)
{
    state_->setJointGroupPositions(arm_group_, jnt_angles);
    state_->setJointGroupVelocities(arm_group_, jnt_vels);
    state_->setJointGroupAccelerations(arm_group_, jnt_accels);        
}

void Robot::setState(sensor_msgs::JointState msg){
    state_->setVariableValues(msg);
}

std::vector<double> Robot::getGravity(){
    int  n = joint_names_.size();
    std::vector<double> torques(n, 0.0);
    std::vector<double> angles(n, 0.0);
    std::vector<double> vels(n, 0.0);
    std::vector<double> accels(n, 0.0);
    std::vector<geometry_msgs::Wrench> wrenches(8, geometry_msgs::Wrench());
    // copy joint angles
    state_->copyJointGroupPositions("arm", angles);
    dyn_solver_->getTorques(
        angles, vels, accels, wrenches, torques
    );
    std::cout << "Torques: [";
    for(int i = 0; i < n; i++){
        std::cout << torques[i] << ", ";
    }
    std::cout << "\b]" << std::endl;
    return torques;
}
std::vector<double> Robot::getTorques(){
    return getTorques(false);
}

std::vector<double> Robot::getTorques(bool with_accel){
    int  n = joint_names_.size();
    std::vector<double> torques(n, 0.0);
    std::vector<double> angles(n, 0.0);
    std::vector<double> vels(n, 0.0);
    std::vector<double> accels(n, 0.0);
    std::vector<geometry_msgs::Wrench> wrenches(8, geometry_msgs::Wrench());
    // copy joint angles
    state_->copyJointGroupPositions("arm", angles);
    state_->copyJointGroupVelocities("arm", vels);
    if(with_accel){
        state_->copyJointGroupAccelerations("arm", accels);
    }

    dyn_solver_->getTorques(
        angles, vels, accels, wrenches, torques
    );
    /*std::cout << "Torques: [";
    for(int i = 0; i < n; i++){
        std::cout << torques[i] << ", ";
    }
    std::cout << "\b]" << std::endl;*/
    return torques;
}

Eigen::MatrixXd Robot::getAnalyticJacobian(Eigen::MatrixXd Jg)
{
    // get transform
    Eigen::Affine3d R = state_->getFrameTransform(ee_link_name_);
    // transform j_geo
    Eigen::MatrixXd T(6,6);
    T << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    
    T.block<3,3>(3,3) = R.matrix().transpose().block<3,3>(0,0);
    Eigen::MatrixXd Ja = T * Jg;
    return Ja;
}

Eigen::MatrixXd Robot::getPsudoInv(Eigen::MatrixXd j)
{
    Eigen::MatrixXd jT = j.transpose();
    Eigen::MatrixXd pInv = jT * (j * jT).inverse();
    return pInv;
}

std::vector<double> Robot::torqueToCurrent(std::vector<double> torques){
    std::vector<double> currents(torques.size(), 0.0);
    currents[0] = torques[0] / 0.222 / 120.0 * 1000.0; // 1000 * Gr / Kt
    currents[1] = torques[1] / 0.222 / 120.0 * 1000.0; // 1000 * Gr / Kt
    currents[2] = torques[2] / 0.215 / 120.0 * 1000.0; // 1000 * Gr / Kt
    currents[3] = torques[3] / 0.215 / 120.0 * 1000.0; // 1000 * Gr / Kt
    currents[4] = torques[4] / 0.215 / 120.0 * 1000.0; // 1000 * Gr / Kt
    currents[5] = torques[5] / 0.209 / 120.0 * 1000.0; // 1000 * Gr / Kt
    return currents;
}

std::vector<double> Robot::currentToTorque(std::vector<double> currents){
    std::vector<double> torques(currents.size(), 0.0);
    torques[0] = currents[0] * 0.222 * 120.0 / 1000.0; // Kt /(1000* Gr)
    torques[1] = currents[1] * 0.222 * 120.0 / 1000.0; // Kt /(1000* Gr)
    torques[2] = currents[2] * 0.215 * 120.0 / 1000.0; // Kt /(1000* Gr)
    torques[3] = currents[3] * 0.215 * 120.0 / 1000.0; // Kt /(1000* Gr)
    torques[4] = currents[4] * 0.215 * 120.0 / 1000.0; // Kt /(1000* Gr)
    torques[5] = currents[5] * 0.209 * 120.0 / 1000.0; // Kt /(1000* Gr)
    return torques;
}

}; // bravo_controllers ns