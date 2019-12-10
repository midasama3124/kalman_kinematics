#!/usr/bin/python
import rospy
import rospkg
from scipy import io as scio
from sensor_msgs.msg import Imu

class IMUPub(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.packpath = rospack.get_path('kalman_kinematics')
        self.trial = rospy.get_param("kalman/sim/trial", 1)
        self.verbose = rospy.get_param("kalman/sim/verbose", False)
        self.gyro = {'thigh': [],
                     'shank': []}
        self.accel = {'thigh': [],
                     'shank': []}
        self.data_len = 0
        self.frame = -1
        self.imu_msg = Imu()
        """ROS initialization"""
        self.node_name = 'imu_data_pub_node'
        rospy.init_node(self.node_name, anonymous=True)
        self.load_data()
        self.init_pubs_()
        self.init_clients_()

    """Load mat files containing IMU datasets from experimental trials conducted in Spain
        1: Shank
        2: Thigh
        G: Linear acceleration
        W: Angular velocity
        Components:
            1: x axis
            2: y axis
            3: z axis"""
    def load_data(self):
        datapath = self.packpath + "/log/"
        data = scio.loadmat(datapath + "imu_dataset" + str(self.trial) + ".mat")
        self.gyro['shank'] = data["W1"]
        self.data_len = len(self.gyro['shank'])
        self.gyro['thigh'] = data["W2"]
        self.accel['shank'] = data["G1"]
        self.accel['thigh'] = data["G2"]
        rospy.loginfo("IMU data has been imported from Matlab datasets. Data length: {}".format(self.data_len))

    # TODO: Modify for this node
    def init_clients_(self):
        rospy.logwarn("({}) Waiting for Kalman server...")
        rospy.wait_for_service('run_kalman_filter')
        try:
            self.knee_pos_client = rospy.ServiceProxy("right_knee_pos", TriggerFloat64)
            rospy.loginfo("Clients are ready")
        except rospy.ServiceException, e:
            print "Service calls failed: %s"%e

    def init_pubs_(self):
        self.shank_imu_pub = rospy.Publisher('shank_imu_data', Imu, queue_size=100)
        self.thigh_imu_pub = rospy.Publisher('thigh_imu_data', Imu, queue_size=100)

    '''Publish a single IMU message containing gyro and accel
        gyro: Gyroscope signal (3-vector [x,y,z])
        accel: Accelerometer (3-vector [x,y,z]) '''
    def send_imu_data(self, gyro, accel, sensor):
        self.imu_msg.angular_velocity.x = gyro[0]
        self.imu_msg.angular_velocity.y = gyro[1]
        self.imu_msg.angular_velocity.z = gyro[2]
        self.imu_msg.linear_acceleration.x = accel[0]
        self.imu_msg.linear_acceleration.y = accel[1]
        self.imu_msg.linear_acceleration.z = accel[2]
        if sensor == 'shank':
            self.shank_imu_pub.publish(self.imu_msg)
        elif sensor == 'thigh':
            self.thigh_imu_pub.publish(self.imu_msg)
        else:
            rospy.logerr("{} is not a known sensor.".format(sensor))


    '''Publish a single frame of IMU data encompassing shank and thigh sensors'''
    def publish_single_frame(self):
        self.frame += 1
        if self.frame < self.data_len-1:
            for sensor in ['thigh', 'shank']:
                self.send_imu_data(self.gyro[sensor][self.frame], self.accel[sensor][self.frame],sensor)
        else:
            for sensor in ['thigh', 'shank']:
                self.send_imu_data(self.gyro[sensor][self.data_len-1], self.accel[sensor][self.data_len-1],sensor)
            self.frame = -1

def main():
    imu_pub = IMUPub()

    freq = rospy.get_param("kalman/frequency", 50)
    rate = rospy.Rate(freq)       # Hz
    rospy.logwarn("Publishing IMU data...")
    while not rospy.is_shutdown():
        imu_pub.publish_single_frame()
    	rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
