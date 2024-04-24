import rospy
from std_srvs.srv import Trigger
from bravo_controllers.srv import StartPlayback
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml

ee_pose = [0 for i in range(7)]
traj = []
recording = False

def recordSRV(srv):
    global recording, traj
    if recording:
        recording = False
        with open('/home/hunter/traj.yaml', 'w') as file:
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
    with open('/home/hunter/traj.yaml', 'r') as file:
        traj = yaml.safe_load(file)
    print(traj)
    msg = Float64MultiArray()
    for pt in traj:
        msg.data = pt
        goalPub.publish(msg)
        rospy.Rate(1.0/srv.dt).sleep()

    return True, "Trajectory Complete"

eeSub = rospy.Subscriber("/bravo/ee_state", JointState, eeStateCB)
goalPub = rospy.Publisher("/bravo/compliance_controller/command", Float64MultiArray, queue_size=1)

if __name__=="__main__":
    rospy.init_node("traj_record_n_playback")
    recSRV = rospy.Service('record_ee_state', Trigger, recordSRV)
    pbSRV = rospy.Service('playback_ee_trajectory', StartPlayback, playbackSRV)
    rospy.spin()