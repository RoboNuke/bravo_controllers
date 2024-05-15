#include <bravo_controllers/robot.h>

namespace bravo_controllers{
Robot::~Robot(){
    delete state_;
    delete arm_group_;
    
}
Robot::Robot(){
    Robot("");
}
Robot::Robot(std::string ee_frame)
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
    num_seg_ = arm_group_->getLinkModelNames().size();
    if( ee_frame == "" ){
        ee_link_name_ = arm_group_->getLinkModelNames().back();
    } else{
        ee_link_name_ = ee_frame;
    }
    //std::cout << "Base:" << state_->getLinkModelNames()[0] << std::endl;
    std::cout << "EE Link:" << ee_link_name_ << std::endl;
    grav_.x = 0.0;
    grav_.y = 0.0;
    grav_.z = -9.8;
    dyn_solver_ = new dynamics_solver::DynamicsSolver(model_, "arm", grav_);

    planning_scene_ = new planning_scene::PlanningScene(model_);

    // for collision checking
    std::map<std::string, double> paddings =  planning_scene_->getCollisionEnv()->getLinkPadding();
    for(int i=0; i < joint_names_.size();i++){
        std::cout << "Padding for " << joint_names_[i] << " is: " << paddings[joint_names_[i]] << std::endl;
    }

    std::cout << "Number of fixed joint:" << arm_group_->getFixedJointModels().size() << std::endl;
    std::cout << "Number of link models:" << arm_group_->getLinkModelNames().size() << std::endl;
}

std::vector<Eigen::Vector3d> Robot::getCollisionDir(std::vector<double> jnt_angles){
    std::vector<Eigen::Vector3d> dirs;
    // set state to new joint angle pose
    robot_state::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setJointGroupPositions(arm_group_, jnt_angles);

    // reset collision request
    collision_result_.clear();
    // set request to get contacts
    collision_request_.contacts = true;
    collision_request_.max_contacts = 1000;
    collision_request_.group_name="arm";

    planning_scene_->checkSelfCollision(collision_request_, collision_result_);

    std::cout << "We are " 
            << (collision_result_.collision ? "in ": "not in ") 
            << "collision" << std::endl;

    if( collision_result_.collision ){
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        for( it = collision_result_.contacts.begin(); 
             it != collision_result_.contacts.end();
             ++it)
        {
            std::cout << "Contact between: "<<
                        it->first.first.c_str() << " and " << 
                        it->first.second.c_str() << "  ";
            for(int i = 0; i < it->second.size(); i++){
                dirs.push_back(it->second[i].normal);
                std::cout << it->second[i].normal.transpose() << std::endl;
            }
        }
    }
    return dirs;
}
Eigen::MatrixXd Robot::getJacobian(){
    Eigen::Vector3d ref_pt(0,0,0);
    return getJacobian(ref_pt, ee_link_name_);
}

collision_detection::CollisionResult Robot::getCollisionRes(std::vector<double> jnt_angles){
    // set state to new joint angle pose
    robot_state::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setJointGroupPositions(arm_group_, jnt_angles);

    // reset collision request
    collision_result_.clear();
    // set request to get contacts
    collision_request_.contacts = true;
    collision_request_.max_contacts = 1000;
    collision_request_.group_name="arm";

    planning_scene_->checkSelfCollision(collision_request_, collision_result_);

    return collision_result_;
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
void Robot::setJntVels(Vector6d jnt_vels){
    std::vector<double> vels(jnt_vels.data(), jnt_vels.data() + 
                            jnt_vels.rows() * jnt_vels.cols());
    state_->setJointGroupVelocities(arm_group_, jnt_vels);
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
    geometry_msgs::Wrench holder;
    holder.force.x = 0;
    holder.force.y = 0;
    holder.force.z = 0;
    holder.torque.x = 0;
    holder.torque.y = 0;
    holder.torque.z = 0;
    std::vector<geometry_msgs::Wrench> wrenches(num_seg_, holder);
    // copy joint angles
    state_->copyJointGroupPositions("arm", angles);
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
    //std::cout << "R:\n" << R.matrix() << "\nT:\n" << T << std::endl;
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

Vector6d Robot::getJntAngles(){
    std::vector<double> angles(joint_names_.size(), 0.0);
    state_->copyJointGroupPositions("arm", angles);
    Vector6d jntAngles(angles.data());
    return jntAngles;
}
Eigen::VectorXd Robot::getJntVels(){
    std::vector<double> vels(joint_names_.size(), 0.0);
    state_->copyJointGroupVelocities("arm", vels);
    Vector6d jntVels(vels.data());
    return jntVels;
}

std::vector<double> Robot::torqueToCurrent(Eigen::VectorXd torques){
    std::vector<double> t(torques.data(), torques.data()+torques.rows()*torques.cols());
    return torqueToCurrent(t);
}


}; // bravo_controllers ns