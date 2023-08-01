from std_msgs.msg import String
import rospy
import smach
import traceback


class AcknowledgeMeasurement(smach.State):
    OC_SENT = "sent"
    OC_FAILURE = "failure"

    def __init__(self):
        super().__init__(
            outcomes=[
                AcknowledgeMeasurement.OC_SENT,
                AcknowledgeMeasurement.OC_FAILURE,
            ],
        )

        # Gather the ROS parameters.
        rospy.logdebug("Gathering the ROS parameters.")
        meas_comms_topic = rospy.get_param("meas_comms_topic")

        # Sanity checks.
        rospy.logdebug("Performing the sanity checks.")
        assert isinstance(meas_comms_topic, str)
        
        # Create a publisher for acknowledging measurements.
        self._ack_meas_pub = rospy.Publisher(meas_comms_topic, String, queue_size=1)

    def execute(self, ud):
        outcome: str = AcknowledgeMeasurement.OC_SENT

        try:
            # Publish ACK messages.
            rospy.logdebug("Publishing the ACK message!")
            for _ in range(10):
                self._ack_meas_pub.publish(String("ACK"))
                rospy.sleep(0.01)

        except:
            rospy.logerr(
                f"An error occurred in {__class__.__name__}:\n{traceback.format_exc()}"
            )
            outcome = AcknowledgeMeasurement.OC_FAILURE

        rospy.loginfo(f"{AcknowledgeMeasurement.__name__} outcome: {outcome}")
        return outcome
