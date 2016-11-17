import rospy
import math
from math import pi, fmod
import tf.transformations
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
# from read_omni_dataset.msg import BallData, LRMGTData, LRMLandmarksData

HEIGHT = 0.81
BASE_FRAME = 'world'
TWO_PI = 2.0*pi


def normalize_angle(angle):
    # normalize positive
    a = fmod(fmod(angle, TWO_PI) + TWO_PI, TWO_PI)

    # check if over pi
    if a > pi:
        a -= TWO_PI
    return a


class Robot(object):
    # The Robot class holds the various robot components, such as odometry, laser-based observations, etc

    def __init__(self, init_pose, name='OMNI_DEFAULT', freq=10):
        # initial robot pose
        self.pose = init_pose

        # assertions for arguments
        assert isinstance(name, str)
        assert isinstance(self.pose, dict)

        # rate
        self.rate = rospy.Rate(freq)

        # robot name and namespace
        self.name = name
        self.namespace = '/' + name

        # subscribers
        self.sub_odometry = rospy.Subscriber(self.namespace + '/odometry', Odometry, callback=self.odometry_callback, queue_size=10)

        # publishers
        self.pub_gt_rviz = rospy.Publisher(self.namespace + '/gtPose', PointStamped, queue_size=10)

        # set not running
        self.is_running = False

        # GT pose
        self.msg_GT = PoseWithCovariance()
        self.msg_GT_rviz = PointStamped()
        self.msg_GT_rviz.header.frame_id = BASE_FRAME

    def odometry_callback(self, msg):
        # add to current pose
        self.add_odometry(msg)

        # print as debug
        rospy.logdebug(self.pose_to_str())

        # publish current pose
        self.publish_rviz_gt()

    def run(self, flag):
        # check if flag is different from current
        if self.is_running == flag:
            return

            # update state
        self.is_running = flag

    def loop(self):
        # All through callbacks
        rospy.spin()

    def add_odometry(self, msg):

        values = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y}
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)

        # yaw is 3rd values (pos 2)
        values['theta'] = tf.transformations.euler_from_quaternion(quaternion)[2]

        # update pose
        try:
            # calculate initial rotation, translation and final rotation
            initial_rot = math.atan2(values['y'], values['x'])
            translation = math.hypot(values['x'], values['y'])
            final_rot = values['theta'] - initial_rot

            # rotate, translate, rotate
            self.pose['theta'] += initial_rot
            self.pose['x'] += translation*math.cos(self.pose['theta'])
            self.pose['y'] += translation*math.sin(self.pose['theta'])
            self.pose['theta'] = normalize_angle(self.pose['theta'] + final_rot)

        except TypeError, err:
            rospy.logfatal('TypeError: reason - %s', err)
            raise
        except KeyError, err:
            rospy.logfatal('KeyError: variable %s doesnt exist', err)
            raise

    def pose_to_str(self):
        return 'Current pose:\nx={0}\ny={1}\ntheta={2}'.format(self.pose['x'], self.pose['y'], self.pose['theta'])

    def pose_to_omni_gt(self):
        msg = self.msg_GT

        # x and y is copy-paste
        msg.pose.position.x = self.pose['x']
        msg.pose.position.y = self.pose['y']

        # obtain quaternion from theta value (rotation about z axis)
        quaternion = tf.transformations.quaternion_about_axis(self.pose['theta'], [0,0,1])
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        return msg

    def publish_rviz_gt(self, stamp=None):
        if stamp is None:
            stamp = rospy.Time.now()

        assert isinstance(stamp, rospy.Time)

        self.msg_GT_rviz.header.stamp = stamp
        self.msg_GT_rviz.point.x = self.pose['x']
        self.msg_GT_rviz.point.y = self.pose['y']
        self.msg_GT_rviz.point.z = HEIGHT

        self.pub_gt_rviz.publish(self.msg_GT_rviz)
