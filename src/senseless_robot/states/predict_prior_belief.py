from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import numpy as np
import rospy
import smach
from spatialmath import UnitQuaternion


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
        # Sanity checks.

    def execute(self, ud):
        outcome: str = PredictPriorBelief.OC_FAILURE

        # ==================================== TEMP CODE =========================================
        rospy.loginfo("** NOTE: RUNNING TEMP CODE **")
        # TODO: Compute the prior belief.

        # Create a new ROS message with prior belief.
        next_goal = ud.next_goal
        x, y, quat = (
            next_goal[0],
            next_goal[1],
            UnitQuaternion.Rz(next_goal[2], unit="rad"),
        )

        prior_belief = PoseWithCovarianceStamped()
        prior_belief.header.frame_id = "map"
        prior_belief.header.stamp = rospy.get_rostime()

        prior_belief.pose.pose.position = Point(x, y, 0.0)
        prior_belief.pose.pose.orientation = Quaternion(
            quat.v[0], quat.v[1], quat.v[2], quat.s
        )

        prior_belief.pose.covariance = (np.eye(6) * 0.1).flatten().tolist()

        ud.pred_prior_belief = prior_belief
        outcome = PredictPriorBelief.OC_SUCCESS
        # ========================================================================================

        rospy.loginfo(f"{PredictPriorBelief.__name__} outcome: {outcome}")
        return outcome
