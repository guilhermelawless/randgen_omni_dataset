import rospy
import random
import tf.transformations
import std_msgs.msg
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry as odometryMsgType
from randgen_omni_dataset.srv import SendString, SendStringResponse

MEAN_X_WF = 0.007
STD_X_WF = 0.1*MEAN_X_WF
MEAN_Y_WF = 0.000
STD_Y_WF = 0.00005
MEAN_THETA_WF = 0.000
STD_THETA_WF = 0.0001

MEAN_X_R = 0
STD_X_R = 0.0001
MEAN_Y_R = MEAN_Y_WF
STD_Y_R = STD_Y_WF
MEAN_THETA_R = 0.01
STD_THETA_R = 0.1*MEAN_THETA_R


class AbstractOdometryStateVar(object):
    # The AbstractOdometryStateVar establishes the properties and methods for one state space variable
    # Variables include x, y, theta in this case
    # Deriving classes should implement, for instance, based on the rng distribution type

    def __init__(self, type, name):
        # type of this variable
        self.type = type

        # name of this variable
        self.name = name

    def rng(self):
        raise NotImplementedError('subclasses must override rng()')


class GaussianOdometryStateVar(AbstractOdometryStateVar):
    def __init__(self, type, name, mean, sigma):
        # Call the base class init
        AbstractOdometryStateVar.__init__(self, type, name)

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
    # You can use the Odometry object in 2 ways:
    #   - It implements rate.sleep() in its loop which will block if ran in main thread, or can be multi-threaded
    #   - Using the get_rand_all, build_msg, and publisher.publish methods to do at your own pace

    stateTypes = dict(WalkForward=0, Rotate=1)
    varTypes = dict(x=0, y=1, theta=2)

    def __init__(self, seed=None, topic='/odometry/', service='/odometry/change_state', freq=10):
        # type: (int, str, str, int) -> Odometry
        """

        :param seed: if specified, the RNG seed will be fixed (useful for debugging)
        :param topic: topic to publish odometry to
        :param service: service name to advertise to interface with changing odometry states
        :param freq: if the object loop method is used, this will be the rate of publishing messages
        """

        # state of the odometry generation
        self._state = Odometry.stateTypes['WalkForward']

        # initiate random seed with current system time
        random.seed(seed)

        # each variable is a list corresponding to the state
        self.walkForward = [GaussianOdometryStateVar('x', 'x_WalkForward', MEAN_X_WF, STD_X_WF),
                            GaussianOdometryStateVar('y', 'y_WalkForward', MEAN_Y_WF, STD_Y_WF),
                            GaussianOdometryStateVar('theta', 'theta_WalkForward', MEAN_THETA_WF, STD_THETA_WF)]

        self.rotate = [ GaussianOdometryStateVar('x', 'x_Rotate', MEAN_X_R, STD_X_R),
                        GaussianOdometryStateVar('y', 'y_Rotate', MEAN_Y_R, STD_Y_R),
                        GaussianOdometryStateVar('theta', 'theta_Rotate', MEAN_THETA_R, STD_THETA_R)]

        # get a list with all variables
        self.var_list = []
        self.var_list.insert(Odometry.stateTypes['WalkForward'], self.walkForward)
        self.var_list.insert(Odometry.stateTypes['Rotate'],self.rotate)

        # list of all values
        self.values = dict()

        # specific robot's odometry topic name
        self.topic = str(topic)

        # publisher of odometry values
        self.publisher = rospy.Publisher(topic, odometryMsgType, queue_size=100)

        # service to change state
        self.service = rospy.Service(service, SendString, self.service_callback)

        # rate to publish odometry
        self.rate = rospy.Rate(freq)

        # initiate the msg to be quicker in the loop
        self.msg = odometryMsgType()
        self.msg.header = std_msgs.msg.Header()
        self.msg.pose = PoseWithCovariance()

        # flag for running if loop is used
        self.is_running = False

    def service_callback(self, req):

        res = SendStringResponse()

        try:
            self.change_state(req.data)
            res.success = True
        except KeyError:
            res.success = False
            pass

        return res

    # Change state to Rotate or WalkForward
    def change_state(self, new_state):
        # type: (str) -> None
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
        # type: () -> int
        return self._state

    def get_rand_type(self, rand_type):
        # type: (str) -> float
        try:
            obj = self.var_list[self._state][Odometry.varTypes[rand_type]]
            ret = obj.rng()
        except KeyError:
            rospy.logfatal('Variable %s does not exist' % rand_type)
            raise
        return ret

    def get_rand_all(self, values=None):
        # type: (dictionary) -> dictionary
        if values is None:
            values = self.values

        # populate values dictionary
        try:
            values['x'] = self.get_rand_type('x')
            values['y'] = self.get_rand_type('y')
            values['theta'] = self.get_rand_type('theta')
        except KeyError:
            rospy.logfatal('Dictionary doesnt have x, y or theta')
            raise

        return values

    def build_msg(self, msg=None, values=None, stamp=None):
        # type: (nav_msgs.msg.Odometry, dictionary, Time) -> nav_msgs.msg.Odometry
        if values is None:
            values = self.values

        if msg is None:
            msg = self.msg

        if stamp is None:
            stamp = rospy.Time.now()

        msg.header.stamp = stamp
        msg.pose.pose.position.x = values['x']
        msg.pose.pose.position.y = values['y']

        # obtain quaternion from theta value (rotation about z axis)
        quaternion = tf.transformations.quaternion_about_axis(values['theta'], [0, 0, 1])
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        return msg

    def loop_once(self, stamp=None):
        # perform one loop without sleeping

        # generate new random numbers according to configuration
        self.get_rand_all()

        # build the ROS message in self.msg
        self.build_msg(stamp=stamp)

        try:
            # publish the message to the configured topic
            self.publisher.publish(self.msg)

        except rospy.ROSException:
            rospy.logdebug('ROS shutdown while publishing odometry')
            pass

    def loop(self):
        # as long as ROS is running
        while not rospy.is_shutdown():
            # if not running stay in this loop
            while not self.is_running:
                self.rate.sleep()

            # perform one loop
            self.loop_once()

            # sleep for the rest of the cycle
            self.rate.sleep()

    def run(self, flag):
        # type: (bool) -> None
        self.is_running = flag
        rospy.logdebug('Odometry %s' % 'started' if flag is True else 'stopped')

