from threading import Lock
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
import rospy
import smach


class MoveToRendezvousPose(smach.State):
    OC_ARRIVED = "arrived"
    OC_FAILURE = "failure"

    OK_PRED_PRIOR_BELIEF = "pred_prior_belief"

    def __init__(self):
        super().__init__(
            outcomes=[
                MoveToRendezvousPose.OC_ARRIVED,
                MoveToRendezvousPose.OC_FAILURE,
            ],
            input_keys=[MoveToRendezvousPose.OK_PRED_PRIOR_BELIEF],
        )

        # Gather the ROS parameters.
        goal_pose_topic = rospy.get_param("goal_pose_topic")
        velocity_cmd_topic = rospy.get_param("velocity_cmd_topic")

        # Sanity checks.
        assert isinstance(goal_pose_topic, str)
        assert isinstance(velocity_cmd_topic, str)

        # Subscribe to velocity command topic.
        self.vel_cmd_sub = rospy.Subscriber(
            velocity_cmd_topic, Twist, self.velocity_command_callback, queue_size=3
        )
        self.last_vel_cmd_lock = Lock()
        self.last_vel_cmd = [0.0, 0.0]

        # Create a publisher that'll tell the platform to move to the goal.
        self.next_goal_pub = rospy.Publisher(goal_pose_topic, PoseStamped, queue_size=1)

    def execute(self, ud):
        outcome: str = MoveToRendezvousPose.OC_ARRIVED

        try:
            # Ensure we received the appropriate message type.
            pred_prior_belief = ud.pred_prior_belief
            if not isinstance(pred_prior_belief, PoseWithCovarianceStamped):
                raise TypeError(
                    f"pred_prior_belief is type {type(pred_prior_belief)}, but expected PoseWithCovarianceStamped."
                )

            # Extract the pose portion of the message.
            rendezvous_pose = PoseStamped()
            rendezvous_pose.header = pred_prior_belief.header

            rendezvous_pose.pose.position.x = pred_prior_belief.pose.pose.position.x
            rendezvous_pose.pose.position.y = pred_prior_belief.pose.pose.position.y
            rendezvous_pose.pose.position.z = pred_prior_belief.pose.pose.position.z

            rendezvous_pose.pose.orientation.x = (
                pred_prior_belief.pose.pose.orientation.x
            )
            rendezvous_pose.pose.orientation.y = (
                pred_prior_belief.pose.pose.orientation.y
            )
            rendezvous_pose.pose.orientation.z = (
                pred_prior_belief.pose.pose.orientation.z
            )
            rendezvous_pose.pose.orientation.w = (
                pred_prior_belief.pose.pose.orientation.w
            )

            # Publish the message.
            rospy.loginfo(
                f"Telling the platform to move to the goal:\n{rendezvous_pose.pose}"
            )
            for _ in range(5):
                self.next_goal_pub.publish(rendezvous_pose)
                rospy.sleep(0.1)

            # Now, wait til the platform arrives.
            rospy.sleep(3.0)
            rospy.loginfo("Waiting for the platform to arrive at the destination.")
            while self.is_moving():
                rospy.sleep(0.25)

        except:
            outcome = MoveToRendezvousPose.OC_FAILURE

        rospy.loginfo(f"{MoveToRendezvousPose.__name__} outcome: {outcome}")
        return outcome

    def is_moving(self) -> bool:
        with self.last_vel_cmd_lock:
            lin_x, ang_z = self.last_vel_cmd

        return abs(lin_x) > 0.0 or abs(ang_z) > 0.0

    def velocity_command_callback(self, cmd_vel: Twist) -> None:
        with self.last_vel_cmd_lock:
            self.last_vel_cmd[0] = cmd_vel.linear.x
            self.last_vel_cmd[1] = cmd_vel.angular.z
