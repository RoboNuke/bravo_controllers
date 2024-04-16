#include <bravo_controllers/bravo_gravity_comp.h>

namespace bravo_controllers{

BravoGravityComp::BravoGravityComp(ros::NodeHandle nh):
    nh_(nh), running_(false)
{
    jnt_state_sub_ = nh_.subscribe("joint_states", 1, &BravoGravityComp::jntStateCallback, this);
    eff_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("arm_effort_controller", 1);

    // create client to controller_manager srv for swapping controller
    controller_switch_client_  = nh_.serviceClient
        <controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    controller_list_client_  = nh_.serviceClient
        <controller_manager_msgs::ListControllers>("controller_manager/list_controllers");

    // advertise service to turn on and off this controller
    toggle_srv_ = nh_.advertiseService("toggle_gravity_comp_controller",
                                        &BravoGravityComp::toggleGravityComp, this);
    

}

void BravoGravityComp::jntStateCallback(sensor_msgs::JointState msg){
    robot_.setState(msg);
    if( running_ ){
        // get gravity torques
        std::vector<double> g = robot_.getGravity();
        std_msgs::Float64MultiArray effortCmd;
        effortCmd.data = robot_.torqueToCurrent(g);// current g-b
        //effortCmd.data.push_back(0.0); // current g-b
        std::reverse(effortCmd.data.begin(), effortCmd.data.end()); // current b-g

        std::cout << "Current: " << std::endl;
        std::vector<double> cs(6,0.0);
        for(int i = 6; i > 0; i--){
            cs[6-i] = msg.effort[i]; // current g-b
        }
        std::vector<double> ts = robot_.currentToTorque(cs); // torques g-b
        std::reverse(ts.begin(), ts.end()); // torques b-g
        std::reverse(g.begin(), g.end()); // torques b-g
        std::cout << "in A" << "\tout A" << "\tin T" << "\tout T" << std::endl;
         for(int i = 0; i < 6; i++){
            std::cout << msg.effort[i+1] <<"    " << effortCmd.data[i]
                      << "    " << ts[i] <<"    " << g[i] << std::endl;
         }
        std::cout << std::endl;
        
        eff_cmd_pub_.publish(effortCmd);
    }

}


bool BravoGravityComp::toggleGravityComp(
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
