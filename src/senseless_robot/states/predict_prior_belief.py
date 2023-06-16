from __future__ import annotations
from actionlib.simple_action_client import SimpleActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import rospy
import smach
from senseless_robot.states.utils import (
    heading_to_quat,
    posewithcovar_to_belief2D,
    xyquat_to_ros_pose,
    xyquat_covar_to_ros_posewithcovar,
)
from threading import Lock


class PredictPriorBelief(smach.State):
    OC_SUCCESS = "success"
    OC_FAILURE = "failure"

    IK_NEXT_GOAL = "next_goal"
    OK_PRED_PRIOR_BELIEF = "pred_prior_belief"

    def __init__(self):
        super().__init__(
            outcomes=[
                PredictPriorBelief.OC_SUCCESS,
                PredictPriorBelief.OC_FAILURE,
            ],
            input_keys=[PredictPriorBelief.IK_NEXT_GOAL],
            output_keys=[PredictPriorBelief.OK_PRED_PRIOR_BELIEF],
        )

        # Gather the ROS parameters.
        rospy.logdebug("Gathering the ROS parameters.")
        sim_odom_topic = rospy.get_param("sim_odom_topic")
        sim_controller_name = rospy.get_param("sim_controller_name")
        odom_topic = rospy.get_param("odom_topic")
        sim_set_pose_topic = rospy.get_param("sim_set_pose_topic")

        # Sanity checks.
        rospy.logdebug("Performing the sanity checks.")
        assert isinstance(sim_odom_topic, str)
        assert isinstance(sim_controller_name, str)
        assert isinstance(odom_topic, str)
        assert isinstance(sim_set_pose_topic, str)

        rospy.logdebug(
            f"Setting up motion node publisher. Topic = {sim_set_pose_topic}"
        )
        self.motion_node_set_pose_pub = rospy.Publisher(
            sim_set_pose_topic, Odometry, queue_size=1
        )

        rospy.logdebug(f"Setting up odom estimator. Topic = {odom_topic}")
        self.estimated_odom_lock = Lock()
        self.latest_estimated_odom = Odometry()
        self.ekf_odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self.estimated_odom_callback, queue_size=3
        )

        rospy.logdebug(f"Setting up simulated odom estimator. Topic = {sim_odom_topic}")
        self.simulated_odom_lock = Lock()
        self.latest_simulated_odom = Odometry()
        self.motion_odom_sub = rospy.Subscriber(
            sim_odom_topic, Odometry, self.simulated_odom_callback, queue_size=3
        )

        # Motion client variable.
        rospy.logdebug(
            f"Setting up simulation controller client. Controller name = {sim_controller_name}"
        )
        self.sim_controller_client = SimpleActionClient(
            sim_controller_name, MoveBaseAction
        )
        self.sim_controller_client.wait_for_server()

    def estimated_odom_callback(self, odom_msg: Odometry) -> None:
        """
        Listens for the odometry message from the Kalman filter node. Used to reinitialize
        the simulated EKF for uncertainty propagation.
        """
        with self.estimated_odom_lock:
            self.latest_estimated_odom = odom_msg

    def simulated_odom_callback(self, odom_msg: Odometry) -> None:
        with self.simulated_odom_lock:
            self.latest_simulated_odom = odom_msg

    def execute(self, ud):
        outcome: str = PredictPriorBelief.OC_SUCCESS

        try:
            # Get the latest estimated pose and set the EKF belief with it.
            with self.estimated_odom_lock:
                rospy.logdebug(f"Setting the latest belief for propagation.")
                for _ in range(5):
                    self.motion_node_set_pose_pub.publish(self.latest_estimated_odom)
                    rospy.sleep(0.01)

            # Tell the controller what the new goal is.
            next_goal = ud.next_goal
            x, y, quat = (next_goal[0], next_goal[1], heading_to_quat(next_goal[2]))

            rospy.logdebug("Creating the goal pose.")
            sim_goal = MoveBaseGoal()
            sim_goal.target_pose.header.frame_id = "map"
            sim_goal.target_pose.header.stamp = rospy.get_rostime()
            sim_goal.target_pose.pose = xyquat_to_ros_pose(x=x, y=y, quat=quat)

            duration = 60.0 * 5.0
            rospy.logdebug(
                f"Sending the goal to the simulated controller:\n{sim_goal.target_pose.pose}"
            )
            self.sim_controller_client.send_goal(sim_goal)

            rospy.logdebug(
                f"Waiting for the simulated platform to arrive (timeout = {duration:.1f} s)."
            )
            result = self.sim_controller_client.wait_for_result(
                timeout=rospy.Duration.from_sec(duration)
            )
            if result == False:
                rospy.logerr(
                    f"Simulated controller did NOT succeed within time limit {duration:.1f} s."
                )
                raise Exception("Controller failed.")

            # If move base succeeded, let's get the propagated covariance.
            rospy.sleep(1.0)
            with self.simulated_odom_lock:
                _, P = posewithcovar_to_belief2D(pose=self.latest_simulated_odom.pose)
                rospy.logdebug("Propagation completed!")

            # Create the predicted prior belief message.
            prior_belief = xyquat_covar_to_ros_posewithcovar(
                x=x, y=y, quat=quat, covar=P.flatten().tolist()
            )
            ros_prior_belief = PoseWithCovarianceStamped()
            ros_prior_belief.header.frame_id = "map"
            ros_prior_belief.header.stamp = rospy.get_rostime()
            ros_prior_belief.pose = prior_belief

            ud.pred_prior_belief = ros_prior_belief

        except:
            outcome = PredictPriorBelief.OC_FAILURE

        rospy.loginfo(f"{PredictPriorBelief.__name__} outcome: {outcome}")
        return outcome
