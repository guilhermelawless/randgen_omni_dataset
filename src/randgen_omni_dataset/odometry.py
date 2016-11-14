import rospy
import random


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

    def __init__(self, seed):
        # state of the odometry generation
        self._state = Odometry.stateTypes['WalkForward']

        # initiate random seed with current system time
        random.seed(seed)

        # each variable is a list corresponding to the state
        self.walkForward = [GaussianOdometryStateVar('x_WalkForward', 0, 1),
                            GaussianOdometryStateVar('y_WalkForward', 0, 0.1),
                            GaussianOdometryStateVar('theta_WalkForward', 0, 0.2)]

        self.rotate = [ GaussianOdometryStateVar('y_Rotate', 0, 0.01),
                        GaussianOdometryStateVar('x_Rotate', 0, 0.01),
                        GaussianOdometryStateVar('theta_Rotate', 0, 0.02)]

        # get a list with all variables
        self.var_list = []
        self.var_list.insert(Odometry.stateTypes['WalkForward'], self.walkForward)
        self.var_list.insert(Odometry.stateTypes['Rotate'],self.rotate)

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
        ret = -1
        try:
            obj = self.var_list[self._state][Odometry.varTypes[rand_type]]
            ret = obj.rng()
        except KeyError:
            rospy.logfatal('Variable %s does not exist' % rand_type)
            raise
        return ret

    def get_rand_all(self):
        values = []
        for stateVar in self.var_list[self._state]:
            values.append(stateVar.rng())

        return values
