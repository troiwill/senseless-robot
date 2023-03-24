from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import smach


class UpdateBelief(smach.State):
    OC_SUCCESS = "success"
    OC_FAILURE = "failure"

    def __init__(self):
        super().__init__(
            outcomes=[
                UpdateBelief.OC_SUCCESS,
                UpdateBelief.OC_FAILURE,
            ],
            input_keys=["pose_measurements"],
        )

        # Gather ROS parameters.
        localizer_measurement_topic: str = rospy.get_param(
            "localizer_measurement_topic"
        )

        # Sanity checks.
        assert isinstance(localizer_measurement_topic, str)

        # Publish to topic that the localizer listens to.
        self.pub = rospy.Publisher(
            localizer_measurement_topic, PoseWithCovarianceStamped, queue_size=1
        )

    def execute(self, ud):
        outcome: str = UpdateBelief.OC_SUCCESS

        try:
            rospy.loginfo("Publishing the pose measurements.")
            for pose_msg in ud.pose_measurements:
                for _ in range(5):
                    self.pub.publish(pose_msg)
                    rospy.sleep(0.1)
                rospy.sleep(0.1)

        except:
            outcome = UpdateBelief.OC_FAILURE

        rospy.loginfo(f"{UpdateBelief.__name__} outcome: {outcome}")
        return outcome
