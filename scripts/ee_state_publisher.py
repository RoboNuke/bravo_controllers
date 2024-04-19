import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import JointState

from math import pi
class EEStatePubliser:
    def __init__(self):
        self.statePub = rospy.Publisher('/bravo/ee_state', JointState, queue_size=10)

        self.ee_link_name = rospy.get_param("ee_link", "ee_link")
        self.world_frame = rospy.get_param("world_frame", "bravo_base_link")
        self.dt = rospy.get_param("dt",0.001)

        self.pose = []
        self.vels = []
        self.lastTime = None
        self.tDiff = None

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def angleDiff(self, a, b):
        # at b going to a
        diff = a - b
        if abs(diff) > pi:
            if diff < 0:
                diff += 2 * pi
            else:
                diff -= 2 * pi
        return diff
    
    def getFramePose(self):
        try:
            transform = self.tfBuffer.lookup_transform_core(self.world_frame, 
                                self.ee_link_name,
                                rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #print("Failed to get TF")
            print(e)
            return None
        t = transform.header.stamp
        if self.lastTime == None:
            self.lastTime = t
            self.tDiff = 0
        elif (t - self.lastTime).to_sec() < self.dt:
            return False
        else:
            self.tDiff = (t - self.lastTime).to_sec()
            self.lastTime = t

        pose = []
        vels = []
        pose.append(transform.transform.translation.x)
        pose.append(transform.transform.translation.y)
        pose.append(transform.transform.translation.z)
        q = []
        q.append(transform.transform.rotation.x)
        q.append(transform.transform.rotation.y)
        q.append(transform.transform.rotation.z)
        q.append(transform.transform.rotation.w)
        r,p,y = euler_from_quaternion(q)
        pose.append(r)
        pose.append(p)
        pose.append(y)

        if self.tDiff == 0:
            for i in range(6):
                vels.append(0)
        else:
            for i in range(6):
                if i < 3:
                    vels.append( (pose[i] - self.pose[i]) / self.tDiff)
                else:
                    vels.append( self.angleDiff(self.pose[i], pose[i])/ self.tDiff)
        self.pose = pose
        self.vels = vels

        return True
    
    def spin(self):
        eeState = JointState()
        while not rospy.is_shutdown():
            if self.getFramePose():
                eeState.position = self.pose
                eeState.velocity = self.vels
                eeState.header.stamp = self.lastTime
                eeState.header.frame_id = self.world_frame

                self.statePub.publish(eeState)
            rospy.Rate(1.0/self.dt).sleep()

if __name__=="__main__":
    rospy.init_node("ee_state_publisher")
    sp = EEStatePubliser()
    sp.spin()
