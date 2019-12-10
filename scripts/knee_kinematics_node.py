#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse

class KneeKin(object):
    def __init__(self):
        self.startEstimating = False
        self.shank = {'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                     'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        self.thigh = {'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                     'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        """Filter constants"""
        """ROS initialization"""
        self.node_name = 'knee_kinematics'
        rospy.init_node(self.node_name, anonymous=True)
        self.init_servers()
        self.init_subs_()
        self.init_pubs_()

    def svr_handle_(self, req):
        self.startEstimating = True
        return TriggerResponse(True, "Starting to write control outputs...")

    def init_servers(self):
        self.imu_svr = rospy.Service('run_kalman_filter', Trigger, self.svr_handle_)

    def init_subs_(self):
        rospy.Subscriber("shank_imu_data", Imu, self.shank_data_callback_)
        rospy.Subscriber("thigh_imu_data", Imu, self.thigh_data_callback_)

    def shank_data_callback_(self, msg):
        self.shank["gyro"]["x"] = msg.angular_velocity.x
        self.shank["gyro"]["y"] = msg.angular_velocity.y
        self.shank["gyro"]["z"] = msg.angular_velocity.z
        self.shank["accel"]["x"] = msg.linear_acceleration.x
        self.shank["accel"]["y"] = msg.linear_acceleration.y
        self.shank["accel"]["z"] = msg.linear_acceleration.z

    def thigh_data_callback_(self, msg):
        self.thigh["gyro"]["x"] = msg.angular_velocity.x
        self.thigh["gyro"]["y"] = msg.angular_velocity.y
        self.thigh["gyro"]["z"] = msg.angular_velocity.z
        self.thigh["accel"]["x"] = msg.linear_acceleration.x
        self.thigh["accel"]["y"] = msg.linear_acceleration.y
        self.thigh["accel"]["z"] = msg.linear_acceleration.z

    def init_pubs_(self):
        self.knee_angle_pub = rospy.Publisher('knee_angle', Float32, queue_size=100)

    def kalman_filter(self):
        rospy.loginfo("({}) Shank gyro data: {}".format(self.node_name, self.shank["gyro"]))
        rospy.loginfo("({}) Shank accel data: {}".format(self.node_name, self.shank["accel"]))
        rospy.loginfo("({}) Thigh gyro data: {}".format(self.node_name, self.thigh["gyro"]))
        rospy.loginfo("({}) Thigh accel data: {}".format(self.node_name, self.thigh["accel"]))

def main():
    knee_kinem = KneeKin()

    rospy.logwarn("({}) Waiting for client call...".format(knee_kinem.node_name))
    while not knee_kinem.startEstimating: pass
    rospy.loginfo("({}) Server was called".format(knee_kinem.node_name))

    freq = rospy.get_param("kalman/frequency", 50)
    rate = rospy.Rate(freq)       # Hz
    while not rospy.is_shutdown():
        knee_kinem.kalman_filter()
        rate.sleep()

    """Clean up ROS parameter server"""
    try:
        rospy.delete_param("kalman")
    except KeyError:
        pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
