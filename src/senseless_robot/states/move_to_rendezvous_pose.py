from actionlib.simple_action_client import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
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
        rospy.logdebug("Gathering the ROS parameters.")
        controller_name = rospy.get_param("controller_name")

        # Sanity checks.
        rospy.logdebug("Performing the sanity checks.")
        assert controller_name != ""

        # Controller client variable.
        rospy.logdebug(f"Setting up the controller client. Controller name = {controller_name}")
        self.controller_client = SimpleActionClient(controller_name, MoveBaseAction)
        self.controller_client.wait_for_server()

    def execute(self, ud):
        outcome: str = MoveToRendezvousPose.OC_ARRIVED

        try:
            # Ensure we received the appropriate message type.
            pred_prior_belief: PoseWithCovarianceStamped = ud.pred_prior_belief
            if not isinstance(pred_prior_belief, PoseWithCovarianceStamped):
                raise TypeError(
                    f"pred_prior_belief is type {type(pred_prior_belief)}, but expected PoseWithCovarianceStamped."
                )

            rospy.logdebug("Creating the goal pose.")
            goal = MoveBaseGoal()
            goal.target_pose.header = pred_prior_belief.header
            goal.target_pose.pose = pred_prior_belief.pose.pose

            # Use the action client to send the goal.
            rospy.loginfo(f"Sending the goal to the controller:\n{goal.target_pose.pose}")
            self.controller_client.send_goal(goal)
            
            duration = 600.0
            rospy.loginfo(f"Waiting for the platform to arrive (timeout = {duration:.1f} s).")
            result = self.controller_client.wait_for_result(timeout=rospy.Duration.from_sec(duration))
            if result == False:
                rospy.logerr(f"Controller did NOT succeed within time limit {duration:.1f} s.")
                raise Exception("Controller failed.")

        except:
            outcome = MoveToRendezvousPose.OC_FAILURE

        rospy.loginfo(f"{MoveToRendezvousPose.__name__} outcome: {outcome}")
        return outcome
