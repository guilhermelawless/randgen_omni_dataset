import rospy
import tf
from read_omni_dataset.msg import *
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PointStamped

GLOBAL_FRAME = 'world'


class OmniCustom():
    # This class will transform messages and TFs to our custom msg format for the OMNI dataset

    def __init__(self, topic_gt='/gtData'):
        # type: (str) -> None

        # initiate main GT message
        self.gt = LRMGTData()
        self.gt.orangeBall3DGTposition.found = False

        # figure out information on existing robots
        try:
            playing_robots = rospy.get_param('PLAYING_ROBOTS')
        except rospy.ROSException, err:
            rospy.logerr('Error in parameter server - %s', err)
            raise
        except KeyError, err:
            rospy.logerr('Value of %s not set', err)
            raise

        # create a tf listener
        self.listener = tf.TransformListener()

        # initiate the publisher for the GT msg
        self.publisher_gt = rospy.Publisher(topic_gt, LRMGTData, queue_size=10)

        # iterate through the playing robots list, building our list of PoseWithCovariance msgs
        list_ctr = 0
        for idx, running in enumerate(playing_robots):

            # robot ID and name
            idx += 1
            idx_s = str(idx)
            name = 'OMNI' + idx_s

            # add subscriber to its pose, with an additional argument concerning the list position
            rospy.Subscriber(name + '/gtPose', PoseStamped, self.robot_pose_callback, list_ctr)

            # add a new PoseWithCovariance object to our poseOMNI list in the GT message
            self.gt.poseOMNI.append(PoseWithCovariance())

            # add a new bool to our foundOMNI list in the GT message
            # will be True when first message comes
            self.gt.foundOMNI.append(False)

            # wait for odometry service to be available before continue
            rospy.wait_for_service(name + '/genOdometry/change_state')

            # increment counter
            list_ctr += 1

        # save number of robots available
        self.numberRobots = list_ctr

        # subscriber to target gt data
        self.sub_target = rospy.Subscriber('/target/gtPose', PointStamped, self.target_pose_callback, queue_size=5)

    def robot_pose_callback(self, msg, list_id):
        # type: (PoseStamped, int) -> None

        # update this robot's information in our GT message
        self.gt.foundOMNI[list_id] = True

        # update time stamp to latest msg time
        self.gt.header.stamp = msg.header.stamp

        # transform the pose from this robot frame to the global frame, using the tf listener
        try:
            # find latest time for transformation
            msg.header.stamp = self.listener.getLatestCommonTime(GLOBAL_FRAME, msg.header.frame_id)
            new_pose = self.listener.transformPose(GLOBAL_FRAME, msg)
        except tf.Exception, err:
            rospy.logwarn("TF Exception when transforming other robots: %s", err)
            return

        # insert new pose in the GT message
        self.gt.poseOMNI[list_id].pose = new_pose.pose

        # if all robots have been found, publish the GT message
        if self.numberRobots == sum(found is True for found in self.gt.foundOMNI):
            self.publisher_gt.publish(self.gt)

    def target_pose_callback(self, msg):
        # type: (PointStamped) -> None

        # update our GT message with the new information
        self.gt.orangeBall3DGTposition.found = True
        self.gt.orangeBall3DGTposition.header.stamp = msg.header.stamp
        self.gt.orangeBall3DGTposition.x = msg.point.x
        self.gt.orangeBall3DGTposition.y = msg.point.y
        self.gt.orangeBall3DGTposition.z = msg.point.z

        # publish this message
        self.publisher_gt.publish(self.gt)
