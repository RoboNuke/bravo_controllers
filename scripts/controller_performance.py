import rospy

import numpy as np
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from bravo_controllers.srv import EvalMinErrorMove, EvalPoint2Point, EvalTrajectory

from scipy.spatial.transform import Rotation as R
from motion_primitives.motion import Mover
from math import cos, sin, pi, fabs
import matplotlib.pyplot as plt
import time
import yaml 

class ControllerPerformance():
    def __init__(self):
        #self.center_ws = [0.4170, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0]
        self.robot = Mover()
        self.save_path = rospy.get_param("controller_performance/path_to_save_folder", "/home/hunter/catkin_ws/")
        self.center_ws = np.array(rospy.get_param("controller_performance/center_ws", [0.4170, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0]))
        self.default_orient = np.array(rospy.get_param("controller_performance/default_orient", [1.0,0,0,0]))
        self.state_topic = rospy.get_param("controller_performance/full_state_topic", "/bravo/ee_state")
        self.controller_cmd_topic = rospy.get_param("controller_performance/controller_cmd_topic", "/bravo/compliance_controller/command")
        self.toggle_controller_topic = rospy.get_param("controller_performance/toggle_controller_topic", "toggle_compliance_controller")
        
        print("Saving to:", self.save_path)
        self.trajEval = rospy.Service('controller_eval/eval_traj', EvalTrajectory, self.evalTrajSRV)
        self.p2pEval = rospy.Service('controller_eval/eval_pt_2_pt', EvalPoint2Point, self.evalP2PSRV)
        self.minErrorEval = rospy.Service('controller_eval/eval_min_error_move', EvalMinErrorMove, self.evalMoveSRV)

        print("Waiting for toggle compliance controller service on ", self.toggle_controller_topic)
        rospy.wait_for_service(self.toggle_controller_topic)
        print("Service Found")
        self.toggle_controller_srv = rospy.ServiceProxy(self.toggle_controller_topic, SetBool)
        self.controller_activated = False
        # robot state sub
        self.b7state_sub = rospy.Subscriber(self.state_topic, JointState, self._stateCB)
        
        # robot cmd pub
        self.b7Controller_pub = rospy.Publisher(self.controller_cmd_topic, Float64MultiArray, queue_size=1)

        self.pose = np.zeros((7,))

    def _stateCB(self, msg):
        self.pose = msg.position

    def startController(self):
        """ Start the compliance controller """
        if not self.controller_activated:
            res = self.toggle_controller_srv(True)
            if res.success:
                self.controller_activated = True
            #print(res.message)
        else:
            print("Already Activated")

    def stopController(self):
        """ Stop the compliance controller """
        if self.controller_activated:
            res = self.toggle_controller_srv(False)
            if res.success:
                self.controller_activated = False
            print(res.message)
        else:
            print("Already Deactivated")

    def moveToPose(self, xyz):
        print("stopping")
        self.stopController() # turn off comp controller
        time.sleep(5.0)
        self.robot.arm_move_group_cmdr.stop()
        PS = PoseStamped()
        PS.header.frame_id = "bravo_base_link"
        PS.pose.position.x = xyz[0]
        PS.pose.position.y = xyz[1]
        PS.pose.position.z = xyz[2]
        if len(xyz) == 3:
            PS.pose.orientation.x = self.default_orient[0]
            PS.pose.orientation.y = self.default_orient[1]
            PS.pose.orientation.z = self.default_orient[2]
            PS.pose.orientation.w = self.default_orient[3]
        else:
            PS.pose.orientation.x = xyz[3]
            PS.pose.orientation.y = xyz[4]
            PS.pose.orientation.z = xyz[5]
            PS.pose.orientation.w = xyz[6]

        print("moving")
        success = self.robot.go_ee_pose(pose = PS, wait = True, retries=1)
        if not success:
            print("Failed to move to pose")
            raise RuntimeError
        print("starting")
        self.startController()
    
    def runTrajEval(self, traj, dt=0.001, n=5, reverse=False):
        """ Robot follows trajectory, each waypoint is set to goal every dt,
            error is also taken then. Repeated n times and reversed if reverse=true
            Returns: error at each point in trajector 
        """
        real_trajs = np.zeros((n* (int(reverse)+1), traj.shape[0], len(traj[0])))
        for attemp in range(n):
            # go to starting position
            self.moveToPose(traj[0])
            time.sleep(0.5)
            # publish pose commands
            if len(traj[0]) == 3:
                pose_cmd = [0 if i < 3 else self.default_orient[i-3] for i in range(7)]
            else:
                pose_cmd = [0 for i in range(7)]
            msg = Float64MultiArray()
            real_traj = np.zeros(traj.shape)
            for i, pt in enumerate(traj):
                #print(pt)
                if len(pt) == 3:
                    pose_cmd[:3] = pt
                else:
                    pose_cmd = pt
                msg.data = pose_cmd
                self.b7Controller_pub.publish(msg)
                rospy.timer.sleep(rospy.Duration(dt))
                # store real pose
                #print(self.pose)
                if len(pt) == 3:
                    real_traj[i,:] = self.pose[:3]
                else:
                    real_traj[i,:] = self.pose
            real_trajs[attemp,:,:] = real_traj
        
        if reverse:
            print("Starting Reverse")
            rev_traj = np.flip(traj, axis=0)
            #real_trajs[n:,:,:] = np.flip(self.runTrajEval(rev_traj, dt, n, False), axis=1)
            real_trajs[n:,:,:] = self.runTrajEval(rev_traj, dt, n, False)

        return real_trajs


    def getTrajStats(self, goal_traj, real_trajs):
        """ Calcs stats for a set of trajectories
            - Avg per point
            - std dev per point
            - avg l2 error per pt
            - std l2 error per pt
            - tot avg l2 error 
            - tot std l2 error 
            - tot avg error
            - tot std error
        """
        error = np.zeros(real_trajs[:,:,:3].shape)
        for i in range(real_trajs.shape[0]):
            error[i,:,:] =  real_trajs[i,:,:3] - goal_traj[:,:3]
        stats = []

        # real traj per pt
        stats.append(np.average(real_trajs, axis=0))
        stats.append(np.std(real_trajs, axis=0))

        # l2 error per pt
        l2_error = np.sqrt( error[:,:,0] ** 2 + error[:,:,1]**2 + error[:,:,2] ** 2)
        stats.append(np.average(l2_error, axis=0))
        stats.append(np.std(l2_error, axis=0))

        # total l2 error
        stats.append(np.average(l2_error) )
        stats.append(np.std(l2_error) )

        # error per xyz
        stats.append(np.array([np.average(np.abs(error[:,:,0])),np.average(np.abs(error[:,:,1])),np.average(np.abs(error[:,:,2]))]))
        stats.append(np.array([np.std(np.abs(error[:,:,0])),np.std(np.abs(error[:,:,1])),np.std(np.abs(error[:,:,2]))]))

        if len(goal_traj[0]) == 7:
            # we have orientation error to deal with
            
            ang_error = np.zeros((real_trajs.shape[0], real_trajs.shape[1], ))
            for t_idx, real_traj in enumerate(real_trajs):
                for i, pt in enumerate(real_traj):    
                    ang_error[t_idx, i] = np.linalg.norm(( R.from_quat(pt[3:]) * R.from_quat(goal_traj[i,3:]).inv()).as_rotvec() * 180.0 / np.pi)

            # get per pt error
            stats.append(np.average(ang_error, axis=0))
            stats.append(np.std(ang_error, axis=0) )

            # get total error        
            stats.append(np.average(ang_error))
            stats.append(np.std(ang_error) )


        return stats

    def plotTrajStats(self, traj, stats, exp_name):
        """ Plots the desired trajectory and avg real traj and error """
        has_ang = len(stats) > 8
        real_traj_avg = stats[0]
        real_traj_std_dev = stats[1]
        t = np.linspace(0, len(traj[:,0]), len(traj[:,0]))
        fig, ax = plt.subplots(4 if has_ang else 3, figsize=(12, 10))
        print(exp_name[1:5])
        fig.suptitle("Trajectory Tracking for " + exp_name[1:5])
        labels = ["x", "y", "z"]
        for i in range(4 if has_ang else 3):
            if i < 3:
                print(traj[:,i].shape)
                ax[i].set_title(labels[i] + "-position")
                ax[i].plot(t, traj[:,i], color='blue', label='Goal')
                ax[i].plot(t, real_traj_avg[:,i], color='red', label='Actual')
                ax[i].fill_between(t, real_traj_avg[:,i] - real_traj_std_dev[:,i], real_traj_avg[:,i] + real_traj_std_dev[:,i], color='#888888', alpha=0.4)
                ax[i].set_ylabel("Position (mm)")
                if i == 2 and not has_ang:
                    ax[i].legend(loc='best')
                    ax[i].set_xlabel("Time Step (0.1s/step)")
            else:
                print(stats[8].shape, stats[9].shape)
                ax[i].set_title("Orientation Error")
                ax[i].plot(t, stats[8], color='red', label='Actual')
                ax[i].fill_between(t, stats[8] - stats[9], stats[8] + stats[9], color='#888888', alpha=0.4)
                ax[i].set_ylabel("Angle (deg)")
                ax[i].legend(loc='best')
                ax[i].set_xlabel("Time Step (0.1s/step)")
        if has_ang:
            plt.savefig(self.save_path + exp_name + "_xyzr_pos.png")
        else:
            plt.savefig(self.save_path + exp_name + "_xyz_pos.png")

        """
        fig2, ax2 = plt.subplots(1, figsize=(8,5))
        #fig2.suptitle("Trajectory Tracking for " + exp_name)
        ax2.set_title("Avg L2 Position Error for " + exp_name)
        ax2.plot(t, stats[2], color='red', label='Actual')
        ax2.fill_between(t, stats[2] - stats[3], stats[2] + stats[3], color='#888888', alpha=0.4)
        ax2.set_ylabel("Error (mm)")
        ax2.set_xlabel("Time Step (0.1s/step)")
        plt.savefig(self.save_path + exp_name + "_l2_error.png")
        #plt.show()
        """
    
    def tabTrajStats(self, stats, tag = ""):
        """ Prints out tabulated data on the trajectories """
        """
        out = "\tSum of Error " + tag + "\n"
        out += "Type \tAvg(mm) \tStd(mm)\n"
        out += f"  x \t{'{0:3.4f}'.format(stats[6][0])} \t{'{:3.4f}'.format(stats[7][0])}\n"
        out += f"  y \t{'{0:3.4f}'.format(stats[6][1])} \t{'{:3.4f}'.format(stats[7][1])}\n"
        out += f"  z \t{'{0:3.4f}'.format(stats[6][2])} \t{'{:3.4f}'.format(stats[7][2])}\n"
        out += f" tot \t{'{0:3.4f}'.format(stats[8])} \t{'{:3.4f}'.format(stats[9])}\n" 
        """
        out = "\tAvg Error " + tag + "\n"
        out += "Type \tAvg(mm) \tStd(mm)\n"
        out += f"  x \t{'{0:3.4f}'.format(stats[6][0])} \t{'{:3.4f}'.format(stats[7][0])}\n"
        out += f"  y \t{'{0:3.4f}'.format(stats[6][1])} \t{'{:3.4f}'.format(stats[7][1])}\n"
        out += f"  z \t{'{0:3.4f}'.format(stats[6][2])} \t{'{:3.4f}'.format(stats[7][2])}\n"
        out += f" tot \t{'{0:3.4f}'.format(stats[4])} \t{'{:3.4f}'.format(stats[5])}\n"
        if len(stats) > 8:
            out += f" rot \t{'{0:3.4f}'.format(stats[10])} \t{'{:3.4f}'.format(stats[11])}\n"
        print(out)
        return out
    
    def runPt2PtEval(self, a, b, n=5):
        """ Places the robot at point a, sets b as controller goal;
            then sets a as goal, repeats n times.  If reverse then the
            process is repeated starting at b 
            returns location errors for each trial
        """
        with_rot = len(a) == 7
        a_outs = np.zeros((n, 7))
        b_outs = np.zeros((n, 7))
        if with_rot:
            a_pose_cmd = a
            b_pose_cmd = b
        else:
            a_pose_cmd = [a[i] if i < 3 else self.default_orient[i-3] for i in range(7)]
            b_pose_cmd = [b[i] if i < 3 else self.default_orient[i-3] for i in range(7)]

        msg = Float64MultiArray()
        for attemp in range(n):
            print("Attempt:", attemp)
            self.moveToPose(a)
            msg.data = b_pose_cmd
            self.b7Controller_pub.publish(msg)
            rospy.sleep(rospy.Duration(5.0))
            b_outs[attemp,:] = self.pose
            self.moveToPose(b)
            msg.data = a_pose_cmd
            self.b7Controller_pub.publish(msg)
            rospy.sleep(rospy.Duration(5.0))
            a_outs[attemp,:] = self.pose
        return a_outs, b_outs
    
    def getPt2PtStats(self, a, b, real_as, real_bs):
        """ Calcs stats for a point to point tests  
            - Avg error to A(total and by direction)
            - std error to A
            - Avg error to B
            - std error to B
        """
        n = real_as.shape[0]
        a_error = np.zeros((n, 5)) 
        b_error = np.zeros((n, 5)) 
        a *= 1000
        b *= 1000
        real_as *= 1000
        real_bs *= 1000
        for k in range(3):
            a_error[:,k] = np.array( [ real_as[i][k] - a[k] for i in range(n) ] )
            b_error[:,k] = np.array([real_bs[i][k] - b[k] for i in range(n)])
        a_error[:,3] = [np.sqrt(a_error[i,0]**2 + a_error[i,1]**2 + a_error[i,2]**2) for i in range(n)]
        b_error[:,3] = [np.sqrt(b_error[i,0]**2 + b_error[i,1]**2 + b_error[i,2]**2) for i in range(n)]
        for i in range(n):
            if len(a) == 7:
                a_error[i,4] = np.linalg.norm(( R.from_quat(real_as[i][3:]/1000.0) * R.from_quat(a[i][3:]/1000.0).inv()).as_rotvec() * 180.0 / np.pi)
                b_error[i,4] = np.linalg.norm(( R.from_quat(real_bs[i][3:]/1000.0) * R.from_quat(b[i][3:]/1000.0).inv()).as_rotvec() * 180.0 / np.pi)
            else:
                #print(real_as[i][3:]/1000.0)
                #print(real_bs[i][3:]/1000.0)
                #print(self.default_orient)
                a_error[i,4] = np.linalg.norm(( R.from_quat(real_as[i][3:]/1000.0) * R.from_quat(self.default_orient).inv()).as_rotvec() * 180.0 / np.pi)
                b_error[i,4] = np.linalg.norm(( R.from_quat(real_bs[i][3:]/1000.0) * R.from_quat(self.default_orient).inv()).as_rotvec() * 180.0 / np.pi)
            

        avg_a = np.average(np.abs(a_error), axis=0)
        avg_b = np.average(np.abs(b_error), axis=0)
        std_a = np.std(np.abs(a_error), axis=0)
        std_b = np.std(np.abs(b_error), axis=0)
        tot = np.concatenate((a_error, b_error), axis=0)
        avg_tot = np.average(np.abs(tot), axis=0)
        std_tot = np.std(np.abs(tot), axis=0)

        return np.array([avg_a, std_a, avg_b, std_b, avg_tot, std_tot])
    
    def plotPt2PtStats(self, a, b, avg, std, exp_name, pt_name="A to B"):
        """ Plots the desired trajectory and avg real traj and error """
        n = len(avg)
        t = np.linspace(0, n, n)
        fig, ax = plt.subplots(3, figsize=(12, 10))
        fig.suptitle("Point to Point for " + pt_name)
        labels = ["x", "y", "z"]
        print(avg.shape, std.shape)
        for i in range(3):
            ax[i].set_title(labels[i] + "-position")
            ax[i].plot(t, a[i] * np.ones((n)), color='green', label='Start')
            ax[i].plot(t, b[i] * np.ones((n)), color='blue', label='Goal')
            ax[i].plot(t, avg[:,i], color='red', label='Actual')
            ax[i].fill_between(t, avg[:,i] - std[:,i], avg[:,i] + std[:,i], color='#888888', alpha=0.4)
            ax[i].set_ylabel("Position (mm)")
            if i == 2:
                ax[i].legend(loc='best')
                ax[i].set_xlabel("Time Step (0.1s/step)")
        plt.savefig(self.save_path + exp_name + "_xyz_" + pt_name + ".png")

    def tabPt2PtStats(self, stats):
        """ Prints out tabulated data on the point 2 point evaluation """
        out = "  Moving to Point A \n"
        out += "Type \t Avg(mm) \t Std(mm)\n"
        out += f"  x \t{'{0:3.2f}'.format(stats[0][0])} \t{'{:3.2f}'.format(stats[1][0])}\n"
        out += f"  y \t{'{0:3.2f}'.format(stats[0][1])} \t{'{:3.2f}'.format(stats[1][1])}\n"
        out += f"  z \t {'{0:3.2f}'.format(stats[0][2])} \t {'{:3.2f}'.format(stats[1][2])}\n"
        out += f" rot \t {'{0:3.2f}'.format(stats[0][4])} \t {'{:3.2f}'.format(stats[1][4])}\n"
        out += f" tot \t{'{0:3.2f}'.format(stats[0][3])} \t{'{:3.2f}'.format(stats[1][3])}\n"
        out += "  Moving to Point B\n"
        out += "Type \t Avg(mm) \t Std(mm)\n"
        out += f"  x \t{'{0:3.2f}'.format(stats[2][0])} \t{'{:3.2f}'.format(stats[3][0])}\n"
        out += f"  y \t{'{0:3.2f}'.format(stats[2][1])} \t{'{:3.2f}'.format(stats[3][1])}\n"
        out += f"  z \t{'{0:3.2f}'.format(stats[2][2])} \t{'{:3.2f}'.format(stats[3][2])}\n"
        out += f" rot \t {'{0:3.2f}'.format(stats[2][4])} \t {'{:3.2f}'.format(stats[3][4])}\n"
        out += f" tot \t{'{0:3.2f}'.format(stats[2][3])} \t{'{:3.2f}'.format(stats[3][3])}\n"
        out += "  Totalled for A and B\n"
        out += "Type \t Avg(mm) \t Std(mm)\n"
        out += f"  x \t{'{0:3.2f}'.format(stats[4][0])} \t{'{:3.2f}'.format(stats[5][0])}\n"
        out += f"  y \t{'{0:3.2f}'.format(stats[4][1])} \t{'{:3.2f}'.format(stats[5][1])}\n"
        out += f"  z \t{'{0:3.2f}'.format(stats[4][2])} \t{'{:3.2f}'.format(stats[5][2])}\n"
        out += f" rot \t {'{0:3.2f}'.format(stats[4][4])} \t {'{:3.2f}'.format(stats[5][4])}\n"
        out += f" tot \t{'{0:3.2f}'.format(stats[4][3])} \t{'{:3.2f}'.format(stats[5][3])}\n"
        print(out)
        return out

    def evalP2PSRV(self, srv):
        output = {"params":{}, "results":{}, 'raw':{}}
        a = np.array(srv.a)
        b = np.array(srv.b)
        n_reps = srv.n_reps
        output['units'] = "mm"
        output['params']['a'] = a.tolist()
        output['params']['b'] = b.tolist()
        output['params']['n_reps'] = n_reps
        a_real, b_real, stats, out_msg = self.runFullPt2PtEval(a, b, srv.exp_name, n_reps)
        names = ["avg_a", "std_a", "avg_b", "std_b", "avg_tot", "std_tot"]
        for i, stat in enumerate(stats):
            output['results'][names[i]] = stat.tolist()
        output['raw']['a'] = a_real.tolist()
        output['raw']['b'] = b_real.tolist()
        self.writeResults(output, srv.exp_name)
        return True, out_msg
    
    def writeResults(self, output, fp="results"):
        with open(self.save_path + fp + ".yaml", 'w') as file:
            yaml.dump(output, file)

    def evalMoveSRV(self, srv):
        output = {"params":{}, "results":{}}
        ld = srv.lin_delta
        rd = srv.rot_delta
        output['units'] = "mm"
        output['params']['lin_delta'] = ld
        output['params']['rot_delta'] = rd
        stats, out_msg = self.runFullMinError4MotionEval(ld, rd)
        output['results']['deltas'] = stats.tolist()
        self.writeResults(output, srv.exp_name)
        return True, out_msg
    
    def runFullPt2PtEval(self, a, b, exp_name, n=5):
        """ Runs point to point tests and tabulates data """
        a_real, b_real = self.runPt2PtEval(a, b, n)
        #print(a_real.shape)
        stats = self.getPt2PtStats(a, b, a_real, b_real)
        """self.plotPt2PtStats(a, b, 
                            np.average(a_real, axis=0), 
                            np.std(a_real, axis=0), 
                            exp_name, "A to B")
        self.plotPt2PtStats(b, a, 
                            np.average(b_real, axis=0), 
                            np.std(b_real, axis=0), 
                            exp_name, "B to A")"""
        out_msg = self.tabPt2PtStats(stats)
        self.stopController()
        return a_real, b_real, stats, out_msg
    
    def runMinErr4MotionEval(self, lin_delta, ang_delta):
        """ For each dim (x,y,z,rr,ry,rp) the goal is increased until motion over 
            lin_delta (for x,y,z) or ang_delta (for rr,ry,rp)
            Returns: Min error in each direction 
        """
        msg = Float64MultiArray()
        moveDeltas = np.zeros((6,))
        for i in range(6):
            print("Starting ", i)
            # move to center point
            self.moveToPose(self.center_ws)
            time.sleep(0.5)
            start_pose = np.array(self.pose)
            not_moved = True
            delta = 0.0001
            while not_moved:
                print(delta )
                if i < 3:
                    start_pose[i] += delta
                    msg.data = start_pose
                    self.b7Controller_pub.publish(msg)
                    rospy.sleep(rospy.Duration(0.1))
                    now_pose = np.array(self.pose)
                    if fabs(now_pose[i] - start_pose[i]) >= lin_delta:
                        moveDeltas[i] = delta * 1000
                        not_moved = False
                    else:
                        delta += 0.0001
                    start_pose = now_pose
                else:
                    not_moved = False
                print("next")

        return moveDeltas

    def tabMinErrorEval(self, stats):
        """ Prints out tabulated data on the trajectories """
        out = "  Minimum Error to Move Arm\n"
        out += "Type \tValue (mm)\n"
        out += f"  x \t{'{0:3.2f}'.format(stats[0])}\n"
        out += f"  y \t{'{0:3.2f}'.format(stats[1])}\n"
        out += f"  z \t{'{0:3.2f}'.format(stats[2])}\n"
        #out += f" tot \t{'{0:3.2f}'.format(stats[3])}\n"
        print(out)
        return out
    
    def getCircleTraj(self, r, dy, n = 100):
        """ Returns a trajector of ee poses that form a circle on the 
            z-x plane of radius r, moves in y dy from bottom of the 
            cirlce to top (over a half circle). Trajectory consists of 
            n points
        """
        dtheta = 2.0 * pi / n
        theta = 0.0
        y = 0.0
        y_delta =  2.0 * dy /  n
        traj = np.zeros((n,3))
        for pt in traj:
            pt[0] = r * cos(theta) + self.center_ws[0]
            pt[2] = r * sin(theta) + self.center_ws[2]
            theta += dtheta 
            pt[1] = y + self.center_ws[1]
            y += y_delta
            if fabs(y) >= dy / 2.0:
                if y > 0.0:
                    y = dy/2.0
                else:
                    y = -dy/2.0
                y_delta *= -1
        return traj
    
    def getOvalTraj(self, dx, dy, dz, n = 100):
        """ Returns a trajector of ee poses that form an oval on the 
            z-x plane of size dx by dz, moves in y dy from bottom of the 
            oval to top (over a half oval). Trajectory consists of 
            n points
        """
        #print(dx, dy, dz, n)
        dtheta = 2.0 * pi / n
        theta = 0.0
        y = 0.0
        y_delta =  2.0 * dy /  n
        traj = np.zeros((n, 7))
        for pt in traj:
            pt[0] = dx/2.0 * cos(theta) + self.center_ws[0]
            pt[2] = dz/2.0 * sin(theta) + self.center_ws[2]
            theta += dtheta 
            pt[1] = y + self.center_ws[1]
            y += y_delta 
            if fabs(y) >= dy / 2.0:
                if y > 0.0:
                    y = dy/2.0
                else:
                    y = -dy/2.0
                y_delta *= -1
            
            pt[3:] = R.from_euler("x", pi).as_quat() 
        return traj
    
    def getSquareTraj(self, sx, sy, sz, n=100):
        """ Returns a trajectory of ee poses that form a square projected 
            on the z-x plane of size sz by sx. Moves in y sy from bottom 
            of the square to top (over half the traj). Traj consists of n 
            points
        """
        traj = np.zeros((n,7))
        slen = n // 4
        print(slen)
        dx = sx / slen
        dy = sy / slen
        dz = sz / slen
        dt = 30.0 / slen

        for i in range(slen): 
            # right side
            traj[i][0] = sx/2.0 + self.center_ws[0]
            traj[i][1] = -sy/2.0 + dy * i + self.center_ws[1]
            traj[i][2] = -sz/2.0 + dz * i + self.center_ws[2]
            #traj[i][3:] = [ 0.9659258, 0, 0.258819, 0 ]
            traj[i][3:] = (R.from_euler("x", 0 + dt * i, degrees=True) * 
                           R.from_quat(self.default_orient)).as_quat()
            # top size
            tI = i + slen
            traj[tI][0] = sx/2.0 - dx * i + self.center_ws[0]
            traj[tI][1] = sy/2.0 + self.center_ws[1]
            traj[tI][2] = sz/2.0 + self.center_ws[2]
            #traj[tI][3:] = [ 0.9659258, 0, 0, -0.258819 ]
            traj[tI][3:] = (R.from_euler("y", 0 + dt * i, degrees=True) * 
                            R.from_euler("x", 30.0 - dt * i, degrees=True) * 
                            R.from_quat(self.default_orient)).as_quat()
            
            # left size
            lI = i + 2 * slen
            traj[lI][0] = -sx/2.0 + self.center_ws[0]
            traj[lI][1] = sy/2.0 - dy * i + self.center_ws[1]
            traj[lI][2] = sz/2.0 - dz * i + self.center_ws[2]
            #traj[lI][3:] = [ 0.9659258, 0, -0.258819, 0 ]
            traj[lI][3:] = (R.from_euler("y", 30 - dt * i, degrees=True) * 
                            R.from_euler("x", 0.0 - dt * i, degrees=True) * 
                            R.from_quat(self.default_orient)).as_quat()
            # bot side
            bI = i + 3 * slen
            traj[bI][0] = -sx/2.0 + dx * i + self.center_ws[0]
            traj[bI][1] = -sy/2.0 + self.center_ws[1]
            traj[bI][2] = -sz/2.0 + self.center_ws[2]
            #traj[bI][3:] = [ 0.9659258, 0, 0, 0.258819 ]
            traj[bI][3:] = (R.from_euler("x", -30.0 + dt * i, degrees=True) * 
                            R.from_quat(self.default_orient)).as_quat()
        
        return traj
    
    def runFullMinError4MotionEval(self, lin_delta, rot_delta):
        """ Runs min error for motion eval and tabulates data """
        stats = self.runMinErr4MotionEval(lin_delta, rot_delta)
        out_msg = self.tabMinErrorEval(stats)
        return stats, out_msg
    
    def evalTrajSRV(self, srv):
        ttype = srv.traj_type
        pams = {}
        pams['traj_type'] = ttype
        pams["dt"] = srv.dt
        pams["traj_rev"] = srv.traj_rev
        pams["traj_n"] = srv.traj_n
        pams["n"] = srv.n_reps
        if srv.traj_type == "circle":
            pams["r"] = srv.r_circle
            pams["dy"] = srv.dy
        elif srv.traj_type == "oval":
            pams["dx"] = srv.dx
            pams["dy"] = srv.dy
            pams["dz"] = srv.dz
        elif srv.traj_type == "square":
            pams["sx"] = srv.sx
            pams["sy"] = srv.sy
            pams["sz"] = srv.sz
        else:
            return False, "Trajectory Type Unknowm"
        
        output = {"params":pams, "results":{}, 'raw':{}}
        output['units'] = "mm"
        real_traj, stats, out_msg = self.runFullTrajEvals([ttype], [pams], srv.exp_name)
        output['raw'] = real_traj.tolist() 
        names= ['Avg_per_pt',"std_per_pt", "avg_l2_error_per_pt","std_l2_error_per_pt",
                'tot_avg_l2_error', 'tot_std_l2_error', 'avg_summed_traj_error', 'std_summed_traj_error',
                'l2_avg_sum_traj_error', 'l2_std_sum_traj_error', 'tot_avg_error', 'tot_std_error']
        
        if srv.traj_rev:
            for val in ["forward", "reverse"]:
                output['results'][val] = {}
                for i, stat in enumerate(stats[val]):
                    output['results'][val][names[i]] = stat.tolist()
        else:
            for i, stat in enumerate(stats):
                output['results'][names[i]] = stat.tolist()

        self.writeResults(output, srv.exp_name)
        return True, out_msg
    
    def runFullTrajEvals(self, traj_types, traj_params, exp_name = "traj"):
        """ Runs trajectory evals, and plots and tabulates data 
            - traj_types: ("cirle", "oval" or "square")
            - traj_params: List of parameters for specific types
        """
        for ttype in traj_types:
            pam = traj_params[0]
            if ttype == "circle":
                goal_traj = self.getCircleTraj(pam["r"], pam["dy"], pam["n"])
            elif ttype == "oval":
                goal_traj = self.getOvalTraj(pam["dx"], pam["dy"], pam["dz"], pam["n"])
            elif ttype == "square":
                goal_traj = self.getSquareTraj(pam["sx"], pam["sy"], pam["sz"], pam["n"])
            else:
                raise RuntimeError

            real_trajs = self.runTrajEval(goal_traj, pam['dt'], pam['traj_n'], pam['traj_rev'])
            real_trajs *= 1000
            goal_traj *= 1000
            if pam['traj_rev']:
                n = pam['traj_n']
                forward_real = real_trajs[:n,:,:]
                reverse_real = real_trajs[n:,:,:]
                forward_goal = goal_traj
                reverse_goal = np.flip(goal_traj, axis=0)

                forward_stats = self.getTrajStats(forward_goal, forward_real)
                reverse_stats = self.getTrajStats(reverse_goal, reverse_real)

                self.plotTrajStats(forward_goal, forward_stats, exp_name + "_forward")
                self.plotTrajStats(reverse_goal, reverse_stats, exp_name + "_reverse")

                out_msg = self.tabTrajStats(forward_stats, 'forward')
                out_msg += "\n"
                out_msg += self.tabTrajStats(reverse_stats, "reverse")
                stats = {"forward": forward_stats, "reverse": reverse_stats}
            else:
                stats = self.getTrajStats(goal_traj, real_trajs)
                self.plotTrajStats(goal_traj, stats, exp_name)
                out_msg = self.tabTrajStats(stats)
            return real_trajs, stats, out_msg

    def spin(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__=="__main__":
    rospy.init_node("controller_evaluator")
    CP = ControllerPerformance()
    CP.spin()