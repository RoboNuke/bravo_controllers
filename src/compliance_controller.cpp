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
    toggle_srv_ = nh_.advertiseService("toggle_compliance_controller",
                                        &ComplianceController::toggleComplianceControl, this);
    gain_srv_ = nh_.advertiseService("compliance_control/set_gains",
                                        &ComplianceController::setGains, this);
    
    // read in error clipping stuff
    nh_.getParam("compliance_controller/clip_error", clip_error_);
    nh_.getParam("compliance_controller/trans_max_error", trans_max_error_);
    nh_.getParam("compliance_controller/rot_max_error", rot_max_error_);

    // read in self collision stuff
    nh_.getParam("compliance_controller/use_self_collision_avoidance", check_self_collision_);
    nh_.getParam("compliance_controller/pos_mult", pos_mult_);
    nh_.getParam("compliance_controller/rot_mult", rot_mult_);
    nh_.getParam("compliance_controller/pos_repulse", pos_repulse_);
    nh_.getParam("compliance_controller/rot_repulse", rot_repulse_);


    // read in controller kp and kd from parameter server
    std::vector<double> k_holder, kd_holder;
    nh_.getParam("compliance_controller/kp", k_holder);
    nh_.getParam("compliance_controller/kd", kd_holder);
    kp_ = Eigen::Matrix<double, 6, 6>();
    kd_ = Eigen::Matrix<double, 6, 6>();
    SetGains(k_holder, kd_holder);

    std::string ee_state_topic, joint_state_topic, effort_cmd_topic;
    nh_.getParam("compliance_controller/joint_state_topic", joint_state_topic);
    nh_.getParam("compliance_controller/ee_state_topic", ee_state_topic);
    nh_.getParam("compliance_controller/effort_cmd_topic", effort_cmd_topic);

    std::cout << "ee_state_topic:" << ee_state_topic << std::endl;
    std::cout << "joint_state_topic:" << joint_state_topic << std::endl;
    std::cout << "effort_cmd_topic:" << effort_cmd_topic << std::endl;

    nh_.getParam("compliance_controller/simulate", sim_);
    std::cout << "Sim:  " << (sim_ ? "on":"off") << std::endl;
    std::cout << "Clip_error  " << (clip_error_ ? "on":"off") << std::endl;

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
            }else{
                kp_(i,j) = 0.0;
                kd_(i,j) = 0.0;
            }
        }
    }
    std::cout << "Kp:\n" << kp_ << "\n  kd:\n" << kd_ << std::endl;

}

void ComplianceController::goalCallback(std_msgs::Float64MultiArray msg){
    for(int i = 0; i < 3; i++){
        goal_pose_[i] = msg.data[i];
    }
    if (msg.data.size() == 6){
        goal_orient_ = EulerToQuat(msg.data[3], msg.data[4], msg.data[5]);
    } else{
        goal_orient_ = Eigen::Quaterniond(msg.data[6], msg.data[3], 
                                    msg.data[4], msg.data[5]); // w, x, y, z
    }

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
    if(!running_){
        goal_pose_ = ee_pose_;
        goal_orient_ = ee_orient_;
    }
        
    //std::cout << "ee_pose:" << ee_pose_.transpose() << std::endl;
}
std_msgs::Float64MultiArray ComplianceController::torqueToROSEffort(Vector6d t){
    std_msgs::Float64MultiArray msg;
    msg.data = robot_.torqueToCurrent(t);
    return msg;
}

void ComplianceController::jntStateCallback(sensor_msgs::JointState msg){
    robot_.setState(msg);
    if( running_ ){
        // get analytical jacobian
        Eigen::MatrixXd J = robot_.getJacobian();
        //std::cout << "J:\n" << J << std::endl;
        Eigen::MatrixXd Ja = robot_.getAnalyticJacobian(J);
        //std::cout << "Ja:\n" << Ja << std::endl;

        // calculate error
        if( goal_orient_.coeffs().dot(ee_orient_.coeffs()) < 0.0){
            ee_orient_.coeffs() << -ee_orient_.coeffs();
        }

        //orient_error_ = ee_orient_.inverse() * goal_orient_;
        orient_error_ = goal_orient_.inverse() * ee_orient_;
        pose_error_.head(3) = ee_pose_.head(3) - goal_pose_.head(3); //goal_pose_ - ee_pose_;
        pose_error_.tail(3) << orient_error_.x(), orient_error_.y(), orient_error_.z();
        if(clip_error_){
            for(int i = 0; i < 3; i++){
                pose_error_[i] = clamp(pose_error_[i], -trans_max_error_, trans_max_error_);
                pose_error_[i+3] = clamp(pose_error_[i+3], -rot_max_error_, rot_max_error_);
            }
        }
        /* below is included in franka controllers but doesn't make sense to me
        pose_error_.tail(3) << -transform.linear() * error_.tail(3); */ 

        // check for collision
        //std::cout << "Pose Error:" << pose_error_.transpose() << std::endl;
        if( check_self_collision_){
            // estimate next joint state
            Vector6d e = pose_error_;
            e.head(3) = e.head(3) * pos_mult_;
            e.tail(3) = e.tail(3) * rot_mult_;
            //std::cout << "E:" << e.transpose() << std::endl;
            Vector6d new_angles  = robot_.getJntAngles() -  Ja.transpose() * ( e );
            std::vector<double> newJntAngles(new_angles.data(), new_angles.data() + 
                                                    new_angles.rows() * new_angles.cols());
            std::vector<Eigen::Vector3d> collision_dirs = robot_.getCollisionDir(newJntAngles);
            
            //std::cout << "Dir Size: " << collision_dirs.size() << std::endl;
            Eigen::Vector3d x = pose_error_.head(3);
            Eigen::Vector3d r = pose_error_.tail(3);
            for(int i=0; i < collision_dirs.size(); i++){
                //std::cout << collision_dirs[i].transpose() << std::endl;
                x = x - pos_repulse_ * x.dot(collision_dirs[i]) * collision_dirs[i];
                r = r - rot_repulse_ * r.dot(collision_dirs[i]) * collision_dirs[i];
            }
            pose_error_.head(3) = x;
            pose_error_.tail(3) = r;
            //std::cout << "After:" << pose_error_.transpose() << std::endl;
            //for(int i=0; i < collision_dirs.size(); i++){
            //    std::cout << x.dot(collision_dirs[i]) << ", " <<
            //                 r.dot(collision_dirs[i]) << std::endl;
            //}
            /*if(collision_dirs.size() > 0){
                std_srvs::SetBool::Request req;
                std_srvs::SetBool::Response res;
                req.data = false;
                toggleComplianceControl(req, res);
                std::cout << "Shutdown" << std::endl;
            }*/
        }
        // calculate u
        dq_ = robot_.getJntVels();

        // publish new command
        Vector6d g(robot_.getGravity().data());
        u_ = g + Ja.transpose() * (-kp_ * pose_error_ - kd_ * (Ja * dq_));
        //std::cout << "u:" << u_ << std::endl;
        
        // convert to effortCmd
        std_msgs::Float64MultiArray effortCmd;
        if(!sim_){
            effortCmd = torqueToROSEffort(u_); // g-b
            std::reverse(effortCmd.data.begin(), effortCmd.data.end()); // current b-g
        } else{
            for(int i = 0; i < 6; i++){
                effortCmd.data.push_back(u_[i]);
            }
        }
        eff_cmd_pub_.publish(effortCmd);
    }

}

bool ComplianceController::toggleComplianceControl(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res){
        if(req.data && !running_){
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
            } else{
                res.success = false;
                res.message = "Controller List not available";
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
                running_ = true;
                return true;
            } else{
                res.success = false;
                res.message = "Failed to stop " + old_controller_name_ + 
                                " or start " + effort_controller_name_;
            }
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

