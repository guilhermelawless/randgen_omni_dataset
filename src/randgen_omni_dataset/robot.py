import rospy
import math
import random
from math import pi, fmod
import tf, tf.transformations
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, Point, PointStamped, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Odometry as odometryMsg
from randgen_omni_dataset.odometry import customOdometryMsg
from visualization_msgs.msg import MarkerArray, Marker
from randgen_omni_dataset.srv import *
from read_omni_dataset.msg import BallData, LRMLandmarksData

HEIGHT = 0.81
BASE_FRAME = 'world'
TWO_PI = 2.0 * pi
LAST_TF_TIME = 0
MAX_DIST_FROM_ROBOTS = 1.0
MAX_ANGLE_FROM_ROBOTS = math.radians(25)
MAX_DIST_FROM_WALLS = 0.2
MAX_ANGLE_FROM_WALLS = pi/2.0


def norm2(x, y):
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))


def orientation_to_theta(orientation):
    assert isinstance(orientation, Quaternion)
    q = (orientation.x, orientation.y, orientation.z, orientation.w)

    # return yaw
    return tf.transformations.euler_from_quaternion(q)[2]


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

    def __init__(self, init_pose, name='OMNI_DEFAULT', target_freq = 20, landmark_freq = 20):
        # type: (dict, str, int, int) -> None

        # initial robot pose
        self.pose = init_pose

        # initiate seed with current system time
        random.seed = None
        # jump ahead a number dependant on this robot
        random.jumpahead(sum(ord(c) for c in name))

        # assertions for arguments
        assert isinstance(name, str)
        assert isinstance(self.pose, dict)
        assert isinstance(target_freq, int)
        assert isinstance(landmark_freq, int)

        # timers for observation callbacks
        self.landmark_timer_period = 1.0/landmark_freq
        self.landmark_timer = None
        self.target_timer_period = 1.0/target_freq
        self.target_timer = None

        # robot name and namespace
        self.name = name[1:] # remove /
        self.namespace = name

        # frame
        self.frame = self.name

        # parameters: landmarks, walls, playing robots
        try:
            self.lm_list = rospy.get_param('/landmarks')
            walls = rospy.get_param('/walls')
            playing_robots = rospy.get_param('PLAYING_ROBOTS')
        except rospy.ROSException, err:
            rospy.logerr('Error in parameter server - %s', err)
            raise
        except KeyError, err:
            rospy.logerr('Value of %s not set', err)
            raise

        # build walls list from the dictionary
        # tuple -> (location, variable to check, reference angle)
        try:
            self.walls = list()
            self.walls.append((walls['left'], 'x', pi))
            self.walls.append((walls['right'], 'x', 0.0))
            self.walls.append((walls['down'], 'y', -pi/2.0))
            self.walls.append((walls['up'], 'y', pi/2.0))
        except KeyError:
            rospy.logerr('Parameter /walls does not include left, right, down and up')
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

        # odometry service
        self.service_change_state = rospy.ServiceProxy(self.namespace + '/genOdometry/change_state', SendString)
        self.service_change_state.wait_for_service()
        self.service_change_state('WalkForward')
        self.is_rotating = False
        self.rotating_timer_set = False

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

        # tf listener
        self.listener = tf.TransformListener()

        # other robots
        list_ctr = 0
        self.otherRobots = list()
        for idx, running in enumerate(playing_robots):
            idx += 1
            idx_s = str(idx)
            # add to list if it's running and is not self
            if running == 0 or self.name.endswith(idx_s):
                continue

            # add subscriber to its pose, with an additional argument concerning the list position
            other_name = 'OMNI'+idx_s
            rospy.Subscriber(other_name + '/gtPose', PoseStamped, self.other_robots_callback, list_ctr)
            self.otherRobots.append((other_name, False))
            # wait for odometry service to be available before continue
            rospy.wait_for_service(other_name + '/genOdometry/change_state')
            list_ctr += 1

    def target_callback(self, msg):
        # save ball pose
        self.target_pose = msg

    def run(self, flag):
        # check if flag is different from current
        if self.is_running == flag:
            return

        # update state
        self.is_running = flag

        # set or shutdown timers
        if self.is_running:
            self.landmark_timer = rospy.Timer(rospy.Duration(self.landmark_timer_period),
                                              self.generate_landmark_observations)

            self.target_timer = rospy.Timer(rospy.Duration(self.target_timer_period),
                                            self.generate_target_observation)
        else:
            self.landmark_timer.shutdown()
            self.target_timer.shutdown()

    def loop(self):
        # All through callbacks
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def odometry_callback(self, msg):
        # type: (customOdometryMsg) -> None

        # if not running, do nothing
        if not self.is_running:
            return

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

        # if rotating timer is set, don't even check for collisions
        if self.rotating_timer_set:
            return

        # check for collisions by generating observations to other robots
        collision = self.check_collisions_other_robots()

        # check for collisions with the world
        collision |= self.check_collisions_world()

        # when rotating and no longer flagged for collision, start a timer before walking forward
        if self.is_rotating and not collision:
            # keep rotating for 1 < rand < 5 secs
            rospy.Timer(rospy.Duration(random.randint(1, 4), random.randint(0, 1e9)), self.stop_rotating, oneshot=True)
            self.rotating_timer_set = True

        # when walking forward if collision detected, start rotating
        elif not self.is_rotating and collision:
            self.service_change_state('Rotate')
            self.is_rotating = True

    def stop_rotating(self, event):
        self.service_change_state('WalkForward')
        self.is_rotating = False
        self.rotating_timer_set = False

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

    def generate_landmark_observations(self, event):
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

    def generate_target_observation(self, event):
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

    def other_robots_callback(self, msg, list_id):
        # Replace tuple with a new tuple with the same name and a new PoseStamped
        self.otherRobots[list_id] = (self.otherRobots[list_id][0], msg)

    def check_collisions_other_robots(self):
        # type: () -> boolean
        for robot, msg in self.otherRobots:
            # check if any message has been received
            if msg is False:
                continue

            try:
                # find latest time for transformation
                msg.header.stamp = self.listener.getLatestCommonTime(self.frame, msg.header.frame_id)
                new_pose = self.listener.transformPose(self.frame, msg)
            except tf.Exception, err:
                rospy.logwarn("TF Exception when transforming other robots: %s", err)
                continue

            dist = norm2(new_pose.pose.position.x, new_pose.pose.position.y)
            ang = normalize_angle(math.atan2(new_pose.pose.position.y, new_pose.pose.position.x))

            # if next to other robot and going to walk into it, return collision
            if dist < MAX_DIST_FROM_ROBOTS and abs(ang) < MAX_ANGLE_FROM_ROBOTS:
                # return collision
                return True

        # no collision
        return False

    def check_collisions_world(self):
        # type () -> boolean

        for location, variable, reference_angle in self.walls:
            if abs(self.pose[variable] - location) < MAX_DIST_FROM_WALLS and abs(normalize_angle(self.pose['theta'] - reference_angle)) < MAX_ANGLE_FROM_WALLS:
                # Future collision detected
                return True

        # No collision
        return False

