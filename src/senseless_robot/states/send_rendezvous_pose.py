from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import smach


class SendRendezvousBelief(smach.State):
    OC_SENT = "sent"
    OC_FAILURE = "failure"

    OK_PRED_PRIOR_BELIEF = "pred_prior_belief"

    def __init__(self):
        super().__init__(
            outcomes=[
                SendRendezvousBelief.OC_SENT,
                SendRendezvousBelief.OC_FAILURE,
            ],
            input_keys=[SendRendezvousBelief.OK_PRED_PRIOR_BELIEF],
        )

        # Get ROS params.
        rospy.logdebug("Gathering the ROS parameters.")
        rendezvous_belief_topic = rospy.get_param("rendezvous_belief_topic")

        # Sanity checks.
        rospy.logdebug("Performing the sanity checks.")
        assert isinstance(rendezvous_belief_topic, str)

        # Create a publisher that will send the rendezvous belief.
        rospy.logdebug(f"Setting up the publisher for the rendezvous belief. Topic = {rendezvous_belief_topic}")
        self.pub = rospy.Publisher(
            rendezvous_belief_topic, PoseWithCovarianceStamped, queue_size=1
        )

    def execute(self, ud):
        outcome: str = SendRendezvousBelief.OC_SENT

        try:
            # Sanity check
            pred_prior_belief = ud.pred_prior_belief
            if not isinstance(pred_prior_belief, PoseWithCovarianceStamped):
                raise TypeError(
                    f"{SendRendezvousBelief.__name__}: pred_prior_belief is type {type(pred_prior_belief)} and not PoseWithCovarianceStamped."
                )

            # Publish the rendezvous pose.
            rospy.logdebug(f"Sending the rendezvous belief to the viewer.")
            for _ in range(5):
                self.pub.publish(ud.pred_prior_belief)
                rospy.sleep(0.1)

        # Signal a failure if something goes wrong.
        except:
            outcome = SendRendezvousBelief.OC_FAILURE

        rospy.loginfo(f"{SendRendezvousBelief.__name__} outcome: {outcome}")
        return outcome
