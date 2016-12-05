import rospy
import random
from math import fabs
from geometry_msgs.msg import PointStamped

ACC_GRAVITY = -9.81  # m.s^-2
PULL_MIN_CHANCE = 0.997
HOVER_MIN_CHANCE = 0.995
HOVER_TIME = 2
HOVER_HEIGHT = 0.8
BASE_FRAME = 'world'
VEL_PULL = 5
FIELD_MAX_X = 5.0
FIELD_MAX_Y = 5.0
MAX_VEL_X = 1.0
MAX_VEL_Y = 1.0


class Ball(object):
    # The ball class holds the pose of a ball in the world, where {0,0,0} is the base frame
    # The model takes into account random acceleration for {x,y}
    # and fixed acceleration with sudden velocity changes for {z}:
    #   - Fixed acceleration (force) of gravity
    #   - Impulse when ball hits the ground
    #   - Impulse by lifting the ball

    def __init__(self, init_pose=None, freq_model=100, freq_pub=10, radius=0.1):

        # initial pose
        if init_pose is None:
            init_pose = {'x': 0, 'y': 0, 'z': 1}
        self.pose = init_pose
        assert len(self.pose) is 3

        # radius
        self.radius = radius
        assert isinstance(radius, float)

        # velocities and flags
        self.pose['vx'] = self.pose['vy'] = self.pose['vz'] = 0.0
        self.flag_hit_ground = False
        self.flag_stop = False
        self.flag_hover = False
        self.virtual_ground = 0.0

        # timer for hovering
        self.timer_hover = None

        # publishers
        self.pub_gt_rviz = rospy.Publisher('/target/gtPose', PointStamped, queue_size=1)

        # model rate
        self.rate_model = rospy.Rate(freq_model)
        self.t = 1.0 / freq_model

        # timer to publish
        self.period_pub = 1.0 / freq_pub
        self.timer_pub = None

        # set not running
        self.is_running = False

        # GT pose
        self.msg_GT_rviz = PointStamped()
        self.msg_GT_rviz.header.frame_id = BASE_FRAME

    def pub_callback(self, event):
        # publish as rviz msg
        self.msg_GT_rviz.header.stamp = rospy.Time.now()
        self.msg_GT_rviz.point.x = self.pose['x']
        self.msg_GT_rviz.point.y = self.pose['y']
        self.msg_GT_rviz.point.z = self.pose['z']

        self.pub_gt_rviz.publish(self.msg_GT_rviz)

    def run(self, flag):
        # check if flag is different from current
        if self.is_running == flag:
            return

            # update state
        self.is_running = flag

        if self.is_running:
            self.timer_pub = rospy.Timer(rospy.Duration(self.period_pub), self.pub_callback)
        else:
            self.timer_pub.shutdown()

    @property
    def ground_hit(self):
        return self.pose['z'] + self.pose['vz'] * self.t - self.radius < self.virtual_ground

    @property
    def above_ground(self):
        return self.pose['vz'] > 0.0 and self.pose['z'] - self.radius > self.virtual_ground

    def hover_callback(self, event):
        self.flag_hover = False
        self.virtual_ground = 0.0
        self.flag_stop = False

    def model_once(self):
        dt = self.t

        # Random acceleration added to velocity of x,y
        ax = random.gauss(0, 2)
        ay = random.gauss(0, 2)
        self.pose['vx'] += ax * dt
        self.pose['vy'] += ay * dt

        # Update x and y with cinematic model
        self.pose['x'] += self.pose['vx'] * dt + 0.5 * ax * dt * dt
        self.pose['y'] += self.pose['vy'] * dt + 0.5 * ay * dt * dt

        # Check if the sides are hit
        if fabs(self.pose['x']) > FIELD_MAX_X:
            self.pose['vx'] *= -1.0

        if fabs(self.pose['y']) > FIELD_MAX_Y:
            self.pose['vy'] *= -1.0

        # Check max velocities
        if fabs(self.pose['vx']) > MAX_VEL_X:
            self.pose['vx'] *= 0.8
        if fabs(self.pose['vy']) > MAX_VEL_Y:
            self.pose['vy'] *= 0.8

        # Should we pull? Low chance
        elif not self.flag_hover and random.random() > PULL_MIN_CHANCE:
            # Pull!
            self.pose['vz'] = VEL_PULL
            self.flag_stop = False
            rospy.logdebug('Ball pulled')

        # Should we hover? Low chance
        elif not self.flag_hover and random.random() > HOVER_MIN_CHANCE:
            # Hover for a while
            self.flag_stop = False
            self.pose['vz'] = VEL_PULL
            self.virtual_ground = HOVER_HEIGHT
            self.flag_hover = True
            self.timer_hover = rospy.Timer(rospy.Duration(HOVER_TIME), self.hover_callback, oneshot=True)
            rospy.logdebug('Hovering for %ds' % HOVER_TIME)

        # Will the ball hit the ground?
        if not self.flag_hit_ground and self.ground_hit:
            # Ball will hit the ground, invert velocity and damp it
            self.pose['vz'] *= -0.7
            self.flag_hit_ground = True
            rospy.logdebug('Hit ground')

        # Should the ball just stop in z?
        elif self.flag_hit_ground and self.pose['vz'] < 0.0 and self.pose['z'] < self.virtual_ground:
            self.pose['vz'] = 0.0
            self.flag_stop = True

        # Update velocity and position
        if not self.flag_stop:
            self.pose['vz'] += ACC_GRAVITY * dt
            self.pose['z'] += self.pose['vz'] * dt + 0.5 * ACC_GRAVITY * dt * dt

        # Check to remove hit ground flag
        if self.flag_hit_ground and self.above_ground:
            self.flag_hit_ground = False

    def loop(self):
        # while ros is running
        while not rospy.is_shutdown():
            # while not running stay here
            while not self.is_running:
                rospy.sleep(self.rate_model)

            self.model_once()
            self.rate_model.sleep()
