import rospy
import threading
import random
import tf.transformations
import std_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Odometry as odometryMsgType


class AbstractOdometryStateVar(object):
    # The AbstractOdometryStateVar establishes the properties and methods for one state space variable
    # Variables include x, y, theta in this case
    # Deriving classes should implement, for instance, based on the rng distribution type

    def __init__(self, name):
        # name of this variable
        self.name = name

    def rng(self):
        raise NotImplementedError('subclasses must override rng()')


class GaussianOdometryStateVar(AbstractOdometryStateVar):
    def __init__(self, name, mean, sigma):
        # Call the base class init
        AbstractOdometryStateVar.__init__(self, name)

        # Save properties of mean and sigma
        self.mu = mean
        self.sigma = sigma

    # Implement the abstract class with gaussian rng
    def rng(self):
        return random.gauss(self.mu, self.sigma)


class Odometry(object):
    # The Odometry class will give a random value of variation for the x, y and theta state-space variables
    # These variables define the robot pose
    # 2 states for the generation are considered:
    #   - Rotate: little variation in x and y, high for theta
    #   - WalkForward: little variation in theta and y, high for x

    stateTypes = dict(WalkForward=0, Rotate=1)
    varTypes = dict(x=0, y=1, theta=2)

    def __init__(self, seed, topic, freq):
        # state of the odometry generation
        self._state = Odometry.stateTypes['WalkForward']

        # initiate random seed with current system time
        random.seed(seed)

        # each variable is a list corresponding to the state
        self.walkForward = [GaussianOdometryStateVar('x_WalkForward', 0, 1),
                            GaussianOdometryStateVar('y_WalkForward', 0, 0.1),
                            GaussianOdometryStateVar('theta_WalkForward', 0, 0.2)]

        self.rotate = [ GaussianOdometryStateVar('x_Rotate', 0, 0.01),
                        GaussianOdometryStateVar('y_Rotate', 0, 0.01),
                        GaussianOdometryStateVar('theta_Rotate', 0, 0.02)]

        # get a list with all variables
        self.var_list = []
        self.var_list.insert(Odometry.stateTypes['WalkForward'], self.walkForward)
        self.var_list.insert(Odometry.stateTypes['Rotate'],self.rotate)

        # list of all values
        self.values = dict()

        # specific robot's odometry topic name
        self.topic = str(topic)

        # publisher of odometry values
        self.publisher = rospy.Publisher(topic, odometryMsgType, queue_size=1)

        # rate to publish odometry
        self.rate = rospy.Rate(freq)

        # initiate the msg to be quicker in the loop
        self.msg = odometryMsgType()
        self.msg.header = std_msgs.msg.Header()
        self.msg.pose = geometry_msgs.msg.PoseWithCovariance()

    # Change state to Rotate or WalkForward
    def change_state(self, new_state):
        try:
            if Odometry.stateTypes[new_state] == self._state:
                rospy.logwarn('Setting new state to the same state %s' % new_state)
            else:
                rospy.logdebug('Changed odometry state to %s' % new_state)
                self._state = Odometry.stateTypes[new_state]
        except KeyError:
            rospy.logfatal('Trying to set odometry state to %s not accepted' % new_state)
            raise

    def get_state(self):
        return self._state

    def get_rand_type(self, rand_type):
        try:
            obj = self.var_list[self._state][Odometry.varTypes[rand_type]]
            ret = obj.rng()
        except KeyError:
            rospy.logfatal('Variable %s does not exist' % rand_type)
            raise
        return ret

    def get_rand_all(self):
        # populate values dictionary
        self.values['x'] = self.get_rand_type('x')
        self.values['y'] = self.get_rand_type('y')
        self.values['theta'] = self.get_rand_type('theta')

        return self.values

    def build_msg(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.pose.pose.position.x = self.values['x']
        self.msg.pose.pose.position.y = self.values['y']

        # obtain quaternion from theta value (rotation about z axis)
        quaternion = tf.transformations.quaternion_about_axis(self.values['theta'], [0,0,1])
        self.msg.pose.pose.orientation.x = quaternion[0]
        self.msg.pose.pose.orientation.y = quaternion[1]
        self.msg.pose.pose.orientation.z = quaternion[2]
        self.msg.pose.pose.orientation.w = quaternion[3]

    def loop(self):
        # wait for at least one subscriber
        while not self.publisher.get_num_connections() > 0:
            rospy.logdebug('Waiting for subscriber on %s' % self.topic)
            rospy.sleep(1)  # 1s
            if rospy.is_shutdown():
                break

        # as long as ROS is running
        while not rospy.is_shutdown():
            # generate new random numbers according to configuration
            self.get_rand_all()

            # build the ROS message in self.msg
            self.build_msg()

            # publish the message to the configured topic
            self.publisher.publish(self.msg)

            # sleep for the rest of the cycle
            self.rate.sleep()