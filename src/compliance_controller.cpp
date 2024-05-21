#include <bravo_controllers/compliance_controller.h>

namespace bravo_controllers{

ComplianceController::ComplianceController(ros::NodeHandle nh):
    nh_(nh), running_(false)
{

    // create client to controller_manager srv for swapping controller
    controller_switch_client_  = nh_.serviceClient
        <controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    controller_list_client_  = nh_.serviceClient
        <controller_manager_msgs::ListControllers>("controller_manager/list_controllers");

    // advertise service to turn on and off this controller
    toggle_srv_ = nh_.advertiseService("compliance_controller/enable_compliance_controller",
                                        &ComplianceController::toggleComplianceControl, this);
    gain_srv_ = nh_.advertiseService("compliance_control/set_gains",
                                        &ComplianceController::setGains, this);
    demo_srv_ = nh_.advertiseService("compliance_controller/enable_demo_mode",
                                        &ComplianceController::demoComplianceControl, this);
    in_demo_mode_ = false;

    // read in error clipping stuff
    nh_.getParam("compliance_controller/clip_error", clip_error_);
    nh_.getParam("compliance_controller/trans_max_error", trans_max_error_);
    nh_.getParam("compliance_controller/rot_max_error", rot_max_error_);

    // read in self collision stuff
    nh_.getParam("compliance_controller/use_self_collision_avoidance", check_self_collision_);
    nh_.getParam("compliance_controller/stop_on_collision", stop_on_collision_);
    nh_.getParam("compliance_controller/look_ahead_dt", look_ahead_dt_);
    nh_.getParam("compliance_controller/pos_repulse", pos_repulse_);
    nh_.getParam("compliance_controller/rot_repulse", rot_repulse_);
    stop_till_new_goal_ = true;

    // read in controller kp and kd from parameter server
    std::vector<double> k_holder, kd_holder;
    nh_.getParam("compliance_controller/kp", k_holder);
    nh_.getParam("compliance_controller/kd", kd_holder);
    kp_ = Eigen::Matrix<double, 6, 6>();
    kd_ = Eigen::Matrix<double, 6, 6>();
    kp2d_ = Eigen::Matrix<double, 6, 6>();
    SetGains(k_holder, kd_holder);

    std::string ee_state_topic, joint_state_topic, effort_cmd_topic, ee_frame;
    nh_.getParam("compliance_controller/joint_state_topic", joint_state_topic);
    nh_.getParam("compliance_controller/ee_state_topic", ee_state_topic);
    nh_.getParam("compliance_controller/effort_cmd_topic", effort_cmd_topic);
    nh_.getParam("compliance_controller/ee_frame", ee_frame);

    robot_ = new Robot(ee_frame);

    std::cout << "ee_state_topic:" << ee_state_topic << std::endl;
    std::cout << "joint_state_topic:" << joint_state_topic << std::endl;
    std::cout << "effort_cmd_topic:" << effort_cmd_topic << std::endl;

    nh_.getParam("compliance_controller/simulate", sim_);
    std::cout << "Sim:  " << (sim_ ? "on":"off") << std::endl;
    std::cout << "Clip_error  " << (clip_error_ ? "on":"off") << std::endl;
    std::cout << "Use self-collision checking  " << (check_self_collision_ ? "on":"off") << std::endl;
    if(check_self_collision_){
        std::cout << "Stop on collision  " << (stop_on_collision_ ? "on":"off") << std::endl;
    }

    // ft interrupt stuff
    nh_.getParam("compliance_controller/with_ft_sensor_interrupt", with_ft_interrupt_);
    is_safe_ = true;
    std::cout << "FT Interrupt:  " << (with_ft_interrupt_ ? "on":"off") << std::endl;
    if( with_ft_interrupt_ ){
        std::string ft_interrupt_topic; 
        nh_.getParam("ft_safety_interrupt/interrupt_topic", ft_interrupt_topic);
        std::cout << "Interrupt topic:" << ft_interrupt_topic << std::endl;
        ft_interrupt_sub_ = nh_.subscribe(ft_interrupt_topic, 1, &ComplianceController::ftInterruptCB, this);
    }

    jnt_state_sub_ = nh_.subscribe(joint_state_topic, 1, &ComplianceController::jntStateCallback, this);
    goal_pose_sub_ = nh_.subscribe("compliance_controller/command", 1, &ComplianceController::goalCallback, this);
    ee_pose_sub_ = nh_.subscribe(ee_state_topic, 1, &ComplianceController::EEPoseCallback, this);
    eff_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(effort_cmd_topic, 1);
    std::cout << "Compliance Controller Initialized" << std::endl;
}

bool ComplianceController::setGains(set_gains::Request &req,
                            set_gains::Response &res){
          
    if(req.new_kps.size() != 6 || req.new_kds.size() != 6){
        res.success=false;
        res.message = "Wrong number of gains";
    }
    SetGains(req.new_kps, req.new_kds);
    res.success = true;
    res.message = "New gains set!";
    return true;                      
}

void ComplianceController::SetGains(std::vector<double> k_holder, std::vector<double> kd_holder){
    std::cout << "Got to printout" << std::endl;
    for(int i = 0; i < 6; i ++){
        for(int j=0; j < 6; j++){
            if( i == j){
                kp_(i,j) = k_holder[i];
                kd_(i,j) = kd_holder[i];
                kp2d_(i,j) = kd_(i,j) / kp_(i,j);
            }else{
                kp_(i,j) = 0.0;
                kd_(i,j) = 0.0;
                kp2d_(i,j) = 0.0;
            }
        }
    }
    std::cout << "Kp:\n" << kp_ << "\n  kd:\n" << kd_ << std::endl;

}

void ComplianceController::goalCallback(std_msgs::Float64MultiArray msg){
    if(in_demo_mode_){
        return;
    }
    for(int i = 0; i < 3; i++){
        goal_pose_[i] = msg.data[i];
    }
    if (msg.data.size() == 6){
        goal_orient_ = EulerToQuat(msg.data[3], msg.data[4], msg.data[5]);
    } else{
        goal_orient_ = Eigen::Quaterniond(msg.data[6], msg.data[3], 
                                    msg.data[4], msg.data[5]); // w, x, y, z
    }
    stop_till_new_goal_ = false;
    std::cout << "New Goal:" << goal_pose_.transpose() << std::endl;

}

double ComplianceController::clamp(double x, double minx, double maxx){
    return std::max( std::min(x, maxx), minx);
}
Eigen::Quaterniond ComplianceController::EulerToQuat(double x, double y, double z){
    return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
}

void ComplianceController::EEPoseCallback(sensor_msgs::JointState msg){
    for(int i = 0; i < 3; i++){
        ee_pose_[i] = msg.position[i];
    }
    if (msg.position.size() == 6){
        ee_orient_ = EulerToQuat(msg.position[3], msg.position[4], msg.position[5]);
    } else{
        ee_orient_ = Eigen::Quaterniond(msg.position[6], msg.position[3], 
                                        msg.position[4], msg.position[5]); // w, x, y, z
    }
    if(!running_ || stop_till_new_goal_ || in_demo_mode_){
        goal_pose_ = ee_pose_;
        goal_orient_ = ee_orient_;
        //std::cout << "Set holding goal: " << goal_pose_.transpose() << std::endl;
    }
        
    //std::cout << "ee_pose:" << ee_pose_.transpose() << std::endl;
}
std_msgs::Float64MultiArray ComplianceController::torqueToROSEffort(Vector6d t){
    std_msgs::Float64MultiArray msg;
    msg.data = robot_->torqueToCurrent(t);
    return msg;
}

void ComplianceController::calcPoseError(){
    if( goal_orient_.coeffs().dot(ee_orient_.coeffs()) < 0.0){
        ee_orient_.coeffs() << -ee_orient_.coeffs();
    }
    //std::cout << "Goal:" << goal_pose_.head(3).transpose() << "  " << goal_orient_.coeffs().transpose() << std::endl;
    //std::cout << "Pose:" << ee_pose_.head(3).transpose() << "  " << ee_orient_.coeffs().transpose() << std::endl;
    //orient_error_ = ee_orient_.inverse() * goal_orient_;
    if(is_safe_){
        orient_error_ = goal_orient_.inverse() * ee_orient_;
        pose_error_.head(3) = ee_pose_.head(3) - goal_pose_.head(3); //goal_pose_ - ee_pose_;
    } else{
        orient_error_ = safe_orient_.inverse() * ee_orient_;
        pose_error_.head(3) = ee_pose_.head(3) - safe_pose_.head(3);
    }
    pose_error_.tail(3) << orient_error_.x(), orient_error_.y(), orient_error_.z();
    if(clip_error_){
        for(int i = 0; i < 3; i++){
            pose_error_[i] = clamp(pose_error_[i], -trans_max_error_, trans_max_error_);
            pose_error_[i+3] = clamp(pose_error_[i+3], -rot_max_error_, rot_max_error_);
        }
    }
    //std::cout << pose_error_.transpose() << std::endl;
    /* below is included in franka controllers but doesn't make sense to me
    pose_error_.tail(3) << -transform.linear() * error_.tail(3); */ 
}

void ComplianceController::checkSelfCollision(Eigen::MatrixXd Ja){
    Vector6d new_angles  = robot_->getJntAngles() - Ja.transpose() * (pose_error_ + kp2d_ * Ja * dq_) * look_ahead_dt_;
    std::vector<double> newJntAngles(new_angles.data(), new_angles.data() + 
                                            new_angles.rows() * new_angles.cols());
    collision_detection::CollisionResult collision_result = robot_->getCollisionRes(newJntAngles);

    //std::cout << "We are " 
    //        << (collision_result.collision ? "in ": "not in ") 
        //       << "collision" << std::endl;

    Vector6d qdot_cc; qdot_cc << 0,0,0,0,0,0;
    if( collision_result.collision && !stop_on_collision_){
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        for( it = collision_result.contacts.begin(); 
            it != collision_result.contacts.end();
            ++it)
        {
            std::cout << "Contact between: "<<
                        it->first.first.c_str() << " and " << 
                        it->first.second.c_str() << "  ";
            for(int i = 0; i < it->second.size(); i++){
                //std::cout << it->second[i].normal.transpose() << std::endl;
                Eigen::MatrixXd Jcc = robot_->getJacobian(it->second[i].pos, 
                                                        it->second[i].body_name_1);
                std::cout << "Jcc:\n" << Jcc << std::endl;
                Eigen::MatrixXd Jcc_inv = robot_->getPsudoInv(Jcc);
                std::cout << "Jcc_inv:\n" << Jcc_inv << std::endl;
                Vector6d xdot_cc;
                xdot_cc.head(3) = it->second[i].normal;
                Vector6d qd_cc = Jcc_inv * xdot_cc;
                for(int i =0; i < 6; i++){
                    if(std::isnan(qd_cc[i])){
                        qd_cc[i] = 0;
                    }
                }
                qdot_cc += qd_cc;

            }
        }
        std::cout << "qdot_cc:" << qdot_cc.transpose() << std::endl;
    }
    if(collision_result.collision && stop_on_collision_){
        stop_till_new_goal_ = true;
        pose_error_ *= 0.0;
    }
}

std_msgs::Float64MultiArray ComplianceController::toEffortCmd(Vector6d u){
    std_msgs::Float64MultiArray effortCmd;
    if(!sim_){
        //std::cout << u << std::endl;
        effortCmd = torqueToROSEffort(u); // g-b
        std::reverse(effortCmd.data.begin(), effortCmd.data.end()); // current b-g
    } else{
        for(int i = 0; i < 6; i++){
            effortCmd.data.push_back(u[i]);
        }
    }
    return effortCmd;
}
Vector6d ComplianceController::getVecFromTwist(geometry_msgs::Twist twst){
    Vector6d out;
    out << twst.linear.x, twst.linear.y, twst.linear.z, twst.angular.x, twst.angular.y, twst.angular.z;
    return out;
}
void ComplianceController::ftInterruptCB(bravo_ft_sensor::FT_Interrupt msg){
    //std::cout << "FT Msg:" << msg.is_safe << std::endl;
    if(!msg.is_safe){
        std::cout << "Not safe!" << std::endl;
        // set this makes it so goal is set to ee pose until new goal comes in
        // this makes it so that as soon as we get into a "safe" position we 
        // don't just instantly move back to the dangerous pose
        stop_till_new_goal_ = true; 
        Vector6d twst = getVecFromTwist(msg.recovery_direction);
        Eigen::Quaterniond w = Eigen::Quaterniond(0.0, twst[3], twst[4], twst[5]); // angular velocity quaternion 
        safe_pose_ = ee_pose_ + look_ahead_dt_ *  twst.head(3);
        // q = q0 + t/2 * w * q0 is how to apply angular velocity to quaternions super intuitive right?
        safe_orient_.coeffs() = ee_orient_.coeffs();// + (look_ahead_dt_ / 2.0) * (w * ee_orient_).coeffs();
        std::cout << "ee_pose_:" << ee_pose_.transpose() << std::endl;
        std::cout << "safe_pose_:" << safe_pose_.transpose() << std::endl;
        //std::cout << "ee_orient_:" << ee_orient_.coeffs() << std::endl;
        //std::cout << "safe_orient_:" << safe_orient_.coeffs() << std::endl;
    } 
    is_safe_ = msg.is_safe;
}

void ComplianceController::jntStateCallback(sensor_msgs::JointState msg){
    // update state
    robot_->setState(msg);
    if( running_ ){
        // get useful values
        Eigen::MatrixXd J = robot_->getJacobian();
        Eigen::MatrixXd Ja = robot_->getAnalyticJacobian(J);
        dq_ = robot_->getJntVels();

        // calculate error
        calcPoseError();

        // check for collision
        if( check_self_collision_){
            checkSelfCollision(Ja);
        }

        // get non-linearities
        Vector6d g(robot_->getGravity().data());
        
        // get control
        u_ = g + Ja.transpose() * (-kp_ * pose_error_ - kd_ * (Ja * dq_));
        //std::cout << "u:" << u_.transpose() << std::endl;
        // convert to effortCmd and publish
        eff_cmd_pub_.publish(toEffortCmd(u_));
    }

}
bool ComplianceController::demoComplianceControl(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res
)
{
    if(!in_demo_mode_ && req.data){
        // turn on demo mode
        res.message = "Successfully enabled demo mode...";
        in_demo_mode_ = true;
    } else if( in_demo_mode_ && !req.data){
        // turn off demo mode
        res.message = "Successfully disabled demo mode...";
        in_demo_mode_ = false;
    }else{
        res.success = false;
        if(req.data){
            res.message = "Demo Mode already running...";
        } else{
            res.message = "Demo Mode already stoppped...";
        }
        return false;
    }
    return true;
}
bool ComplianceController::toggleComplianceControl(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res){
        if(req.data && !running_){
            running_ = true;
            // get list of all loaded controllers
            controller_manager_msgs::ListControllers listSrv;
            if(controller_list_client_.call(listSrv)){
                for(auto const &c : listSrv.response.controller){
                    if( c.name.compare(0, 3, "arm") == 0 ){
                        if( c.state.compare("running") == 0 ){
                            old_controller_name_ = c.name;
                        } else if (c.type.compare("effort_controllers/JointGroupEffortController") == 0){
                            effort_controller_name_ = c.name;
                        }
                    }
                }
                stop_till_new_goal_ = false;
            } else{
                res.success = false;
                res.message = "Controller List not available";
                running_ = false;
                return false;
            }

            // start/stop the controllers
            controller_manager_msgs::SwitchController switchSrv;
            switchSrv.request.start_controllers.push_back(effort_controller_name_);
            switchSrv.request.stop_controllers.push_back(old_controller_name_);
            switchSrv.request.strictness = switchSrv.request.STRICT;
            switchSrv.request.start_asap = true;
            if(controller_switch_client_.call(switchSrv) && switchSrv.response.ok){
                res.success = true;
                res.message = "Controller " + old_controller_name_ + " stopped and " +
                            effort_controller_name_ + " started";
                return true;
            } else{
                res.success = false;
                res.message = "Failed to stop " + old_controller_name_ + 
                                " or start " + effort_controller_name_;
                running_= false;
            }
            /*std::cout << "New Goal:" << goal_pose_.transpose() << std::endl;
            std::cout << "Orient:" << goal_orient_.coeffs() << std::endl;*/
        } else if( !req.data && running_){
            running_ = false;
            // stop controller
            controller_manager_msgs::SwitchController switchSrv;
            switchSrv.request.stop_controllers.push_back(effort_controller_name_);
            switchSrv.request.start_controllers.push_back(old_controller_name_);
            switchSrv.request.strictness = switchSrv.request.STRICT;
            switchSrv.request.start_asap = true;
            if(controller_switch_client_.call(switchSrv) && switchSrv.response.ok){
                res.success = true;
                res.message = "Controller " + effort_controller_name_ + " stopped and " +
                            old_controller_name_ + " started";
                return true;
            } else{
                res.success = false;
                res.message = "Failed to stop " + effort_controller_name_ + 
                                " or start " + old_controller_name_;
            }
        }
        else{
            res.success = false;
            if(req.data){
                res.message = "Controller already running...";
            } else{
                res.message = "Controller already stoppped...";
            }
            return false;
        }
        return true;
}

}; // bravo_controllers ns

