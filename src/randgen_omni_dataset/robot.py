import rospy
import math
from math import pi, fmod
import tf.transformations, tf.broadcaster, tf.listener
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, Point, PointStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry as odometryMsg
from randgen_omni_dataset.odometry import customOdometryMsg
from visualization_msgs.msg import MarkerArray, Marker
from read_omni_dataset.msg import BallData, LRMLandmarksData

HEIGHT = 0.81
BASE_FRAME = 'world'
TWO_PI = 2.0 * pi
LAST_TF_TIME = 0


def normalize_angle(angle):
    # normalize positive
    a = fmod(fmod(angle, TWO_PI) + TWO_PI, TWO_PI)

    # check if over pi
    if a > pi:
        a -= TWO_PI
    return a


def build_marker_arrow(head):
    marker = Marker()

    marker.type = Marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.03
    marker.scale.z = 0.05
    marker.color.a = 1
    marker.color.r = marker.color.g = marker.color.b = 0.2

    # tail is 0,0,0
    marker.points.append(Point())

    # head is already a point
    marker.points.append(head)

    return marker


class Robot(object):
    # The Robot class holds the various robot components, such as odometry, laser-based observations, etc

    def __init__(self, init_pose, name='OMNI_DEFAULT', freq=10):
        # type: (dict, str, int) -> None

        # initial robot pose
        self.pose = init_pose

        # assertions for arguments
        assert isinstance(name, str)
        assert isinstance(self.pose, dict)

        # robot name and namespace
        self.name = name
        self.namespace = '/' + name

        # subscribers
        self.sub_odometry = rospy.Subscriber(self.namespace + '/genOdometry', customOdometryMsg, callback=self.odometry_callback,
                                             queue_size=100)
        self.sub_target = rospy.Subscriber('/target/gtPose', PointStamped, callback=self.target_callback,
                                           queue_size=1)

        # publishers
        self.pub_odometry = rospy.Publisher(self.namespace + '/odometry', odometryMsg, queue_size=100)
        self.pub_gt_rviz = rospy.Publisher(self.namespace + '/gtPose', PoseStamped, queue_size=10)
        self.pub_landmark_observations = rospy.Publisher(self.namespace + '/landmarkObs', MarkerArray, queue_size=5)
        self.pub_target_observation = rospy.Publisher(self.namespace + '/targetObs', Marker, queue_size=5)

        # tf broadcaster
        self.broadcaster = tf.TransformBroadcaster()
        self.frame = self.name

        # tf listener
        self.listener = tf.TransformListener()

        # landmarks
        try:
            self.lm_list = rospy.get_param('/landmarks')
        except rospy.ROSException, err:
            rospy.logerr('Error in parameter server - %s', err)
            raise
        except KeyError, err:
            rospy.logerr('Value of %s not set', err)
            raise

        # target pose
        self.target_pose = None

        # set not running
        self.is_running = False

        # odometry msg
        self.msg_odometry = odometryMsg()
        self.msg_odometry.header = Header()
        self.msg_odometry.pose = PoseWithCovariance()

        # GT pose
        self.msg_GT = PoseWithCovariance()
        self.msg_GT_rviz = PoseStamped()
        self.msg_GT_rviz.header.frame_id = self.frame

    def odometry_callback(self, msg):
        # type: (customOdometryMsg) -> None

        # convert to normal odometry msg, self.msg_odometry will be updated
        self.convert_odometry(msg)

        # publish the odometry in standard format
        self.pub_odometry.publish(self.msg_odometry)

        # add to current pose using custom msg type (easier to add)
        self.add_odometry(msg)

        # print as debug
        rospy.logdebug(self.pose_to_str())

        # publish current pose
        self.publish_rviz_gt()

        # generate landmark observations
        self.generate_landmark_observations()

        # generate target observation
        self.generate_target_observation()

    def target_callback(self, msg):
        # save ball pose
        self.target_pose = msg

    def run(self, flag):
        # check if flag is different from current
        if self.is_running == flag:
            return

            # update state
        self.is_running = flag

    def loop(self):
        # All through callbacks
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def convert_odometry(self, msg):
        # type: (customOdometryMsg) -> odometryMsg

        # convert from {translation, rot1, rot2} to our state-space variables {x, y, theta} using previous values
        self.msg_odometry.header.stamp = msg.header.stamp
        self.msg_odometry.pose.pose.position.x = msg.translation * math.cos(msg.rot1)
        self.msg_odometry.pose.pose.position.y = msg.translation * math.sin(msg.rot2)
        delta_theta = msg.rot1 + msg.rot2
        quaternion = tf.transformations.quaternion_about_axis(delta_theta, [0, 0, 1])
        self.msg_odometry.pose.pose.orientation.x = quaternion[0]
        self.msg_odometry.pose.pose.orientation.y = quaternion[1]
        self.msg_odometry.pose.pose.orientation.z = quaternion[2]
        self.msg_odometry.pose.pose.orientation.w = quaternion[3]

        return self.msg_odometry

    def add_odometry(self, msg):
        # type: (customOdometryMsg) -> None

        # update pose
        try:
            # rotate, translate, rotate
            self.pose['theta'] += msg.rot1
            self.pose['x'] += msg.translation * math.cos(self.pose['theta'])
            self.pose['y'] += msg.translation * math.sin(self.pose['theta'])
            self.pose['theta'] = normalize_angle(self.pose['theta'] + msg.rot2)

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
        quaternion = tf.transformations.quaternion_about_axis(self.pose['theta'], [0, 0, 1])
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        return msg

    def publish_rviz_gt(self, stamp=None):
        if stamp is None:
            stamp = rospy.Time.now()

        assert isinstance(stamp, rospy.Time)

        quaternion = tf.transformations.quaternion_about_axis(self.pose['theta'], [0, 0, 1])

        self.broadcaster.sendTransform((self.pose['x'], self.pose['y'], HEIGHT),
                                       quaternion,
                                       stamp,
                                       self.frame,
                                       BASE_FRAME)

        self.msg_GT_rviz.header.stamp = stamp
        # everyhing else is 0 because of TF

        self.pub_gt_rviz.publish(self.msg_GT_rviz)

    def generate_landmark_observations(self):
        marker_id = 0
        stamp = rospy.Time() # last available tf
        markers = MarkerArray()
        lm_point = PointStamped()
        lm_point.header.frame_id = BASE_FRAME
        lm_point.header.stamp = stamp

        # for all landmarks (for now only 1, debug)
        for lm in self.lm_list:
            lm_point.point.x = lm[0]
            lm_point.point.y = lm[1]

            # Calc. the observation in the local frame
            try:
                lm_point_local = tf.TransformerROS.transformPoint(self.listener, self.frame, lm_point)
            except tf.Exception, err:
                rospy.logwarn('TF Error - %s', err)
                return

            # create a marker arrow to connect robot and landmark
            marker = build_marker_arrow(lm_point_local.point)
            marker.header.frame_id = self.frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = self.namespace+'/landmarkObs'
            marker.id = marker_id

            markers.markers.append(marker)
            marker_id += 1

        self.pub_landmark_observations.publish(markers)

    def generate_target_observation(self):
        if self.target_pose is None:
            return

        marker_id = 0

        # Modify the target_pose header stamp to be the time of the latest TF available
        self.target_pose.header.stamp = rospy.Time()

        # Calc. the observation in the local frame
        try:
            target_local = tf.TransformerROS.transformPoint(self.listener, self.frame, self.target_pose)
        except tf.Exception, err:
            rospy.logwarn('TF Error - %s', err)
            return

        # create a marker arrow to connect robot and landmark
        marker = build_marker_arrow(target_local.point)
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.namespace + '/targetObs'
        marker.id = marker_id
        marker.color.g = 1.0

        self.pub_target_observation.publish(marker)