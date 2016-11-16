import rospy
import math
from randgen_omni_dataset import odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PointStamped
import tf.transformations
#from read_omni_dataset.msg import BallData, LRMGTData, LRMLandmarksData

TWO_PI = math.pi * 2
HEIGHT = 0.81
BASE_FRAME = 'world'

class Robot(object):
    # The Robot class holds the various robot components, such as odometry, laser-based observations, etc

    def __init__(self, init_pose, name='OMNI_DEFAULT'):
        # initial robot pose
        self.pose = init_pose

        # assertions for arguments
        assert isinstance(name, str)
        assert isinstance(self.pose, dict)

        # robot name and namespace
        self.name = name
        self.namespace = '/' + name

        # publishers
        self.pub_gt_rviz = rospy.Publisher(self.namespace + '/gtPose', PointStamped, queue_size=1)

        # odometry object
        self.odometry = odometry.Odometry(topic=self.namespace + '/odometry')

        # initial state is walk forward
        self.odometry.change_state('WalkForward')

        # set not running
        self.is_running = False

        # GT pose
        self.msg_GT = PoseWithCovariance()
        self.msg_GT_rviz = PointStamped()
        self.msg_GT_rviz.header.frame_id = BASE_FRAME

    def run(self, flag):
        # check if flag is different from current
        if self.is_running == flag:
            return

            # update state
        self.is_running = flag

        # start or stop components according to flag
        self.odometry.run(flag)

    def one_loop(self):
        # perform one odometry loop
        self.odometry.one_loop()

        # add odometry to pose
        self.add_odometry()

    def add_odometry(self):

        values = self.odometry.values

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
            self.pose['theta'] = (self.pose['theta'] + final_rot) % (TWO_PI)

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
