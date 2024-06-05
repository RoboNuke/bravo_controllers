import rospy
from std_srvs.srv import Trigger
from bravo_controllers.srv import StartPlayback, ToggleRecording
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import yaml

ee_pose = [0 for i in range(7)]
traj = []
recording = False

def recordSRV(srv):
    global recording, traj
    if recording:
        recording = False
        with open(srv.filepath, 'w') as file:
            yaml.dump(traj, file)
    else:
        traj = []
        recording = True
    return True, f"Recording State: {recording}"

def eeStateCB(msg):
    global ee_pose, recording
    ee_pose = msg.position
    if recording:
        traj.append(list(ee_pose))

def playbackSRV(srv):
    global goalPub
    # load traj
    with open(srv.filepath, 'r') as file:
        traj = yaml.safe_load(file)
    out_traj = Float64MultiArray()
    dim = []
    n = srv.steps if srv.steps > 0 else len(traj)
    dim.append(MultiArrayDimension("pose_list", n, len(traj[0])*n))
    dim.append(MultiArrayDimension("pose", len(traj[0]), 1))
    out_traj.layout.dim = dim
    out_traj.layout.data_offset = 0
    if srv.steps == 0:
        msg = Float64MultiArray()
        for pt in traj:
            print(pt)
            msg.data = pt
            for j in range(len(pt)):
                out_traj.data.append(pt[j])
            if srv.playback:
                goalPub.publish(msg)
                rospy.Rate(1.0/srv.dt).sleep()

        return True, "Trajectory Complete", out_traj
    else:
        msg = Float64MultiArray()
        step = len(traj) // srv.steps
        offset = len(traj) % srv.steps
        for i in range(srv.steps):
            pt = traj[offset + (i+1) * step - 1]
            msg.data = pt
            for j in range(len(pt)):
                out_traj.data.append(pt[j])
            if srv.playback:
                goalPub.publish(msg)
                rospy.Rate(1.0/srv.dt).sleep()
        return True, "Trajectory Completed", out_traj

        


eeSub = rospy.Subscriber("/bravo/ee_state", JointState, eeStateCB)
goalPub = rospy.Publisher("/bravo/compliance_controller/command", Float64MultiArray, queue_size=1)

if __name__=="__main__":
    rospy.init_node("traj_record_n_playback")
    recSRV = rospy.Service('record_ee_state', ToggleRecording, recordSRV)
    pbSRV = rospy.Service('playback_ee_trajectory', StartPlayback, playbackSRV)
    rospy.spin()