import numpy as np
import rospy
import smach


class ComputeNextPose(smach.State):
    OC_HAVE_NEW_GOAL = "have_new_goal"
    OC_FAILURE = "failure"
    OC_DONE = "done"

    OK_NEXT_GOAL = "next_goal"

    def __init__(self):
        super().__init__(
            outcomes=[
                ComputeNextPose.OC_HAVE_NEW_GOAL,
                ComputeNextPose.OC_DONE,
                ComputeNextPose.OC_FAILURE,
            ],
            output_keys=[ComputeNextPose.OK_NEXT_GOAL],
        )

        # Get the ROS parameters.
        rospy.logdebug("Gathering the ROS parameters.")
        waypoints = rospy.get_param("waypoints")
        waypoints = np.array(waypoints)
        rospy.logdebug(f"Got the following waypoints:\n{waypoints}")

        # Sanity checks.
        rospy.logdebug("Performing the sanity checks.")
        assert len(waypoints.shape) == 2
        assert len(waypoints) > 0
        assert waypoints.shape[1] == 3

        self.waypoints: np.ndarray = waypoints.copy()
        self.w_index: int = 0
        rospy.logdebug(f"Loaded {len(self.waypoints)} waypoints.")

    def execute(self, ud):
        try:
            # Get the next goal from the list of waypoints.
            if self.w_index < len(self.waypoints):
                next_goal = self.waypoints[self.w_index].tolist()
                self.w_index += 1
                outcome = ComputeNextPose.OC_HAVE_NEW_GOAL
                rospy.logdebug(
                    f"Next goal: {next_goal}. {len(self.waypoints) - self.w_index} waypoints left."
                )
                ud.next_goal = list(
                    [next_goal[0], next_goal[1], np.radians(next_goal[2])]
                )

            # There are no more waypoints/goals.
            else:
                rospy.logdebug("Completed the path.")
                ud.next_goal = [0.0, 0.0, 0.0]
                outcome = ComputeNextPose.OC_DONE

        except:
            outcome = ComputeNextPose.OC_FAILURE

        rospy.loginfo(f"{ComputeNextPose.__name__} outcome: {outcome}")
        return outcome
