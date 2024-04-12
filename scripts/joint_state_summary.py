import rospy
from sensor_msgs.msg import JointState
import numpy as np
count = 0
max_count = 1000
avgs = {"pos": np.zeros(6), "vel":np.zeros(6), "mA":np.zeros(6)}

def jsCB(msg):
    global count, max_count, avgs
    for i in range(6,0, -1):
        avgs["pos"][6 - i] += msg.position[i] # g-b
        avgs["vel"][6 - i] += msg.velocity[i]
        avgs["mA"][6 - i] += msg.effort[i]  
    count += 1
    if count == max_count - 1:
        print(avgs["pos"] / count)
        print(avgs["vel"] / count)
        print(avgs["mA"] / count)

        count = 0
        avgs = {"pos": np.zeros(6), "vel":np.zeros(6), "mA":np.zeros(6)}

    
if __name__=="__main__":
    rospy.init_node("joint_state_summerizer")
    sub = rospy.Subscriber("/bravo/joint_states", JointState, jsCB)

    rospy.spin()
