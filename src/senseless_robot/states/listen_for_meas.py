from threading import Lock
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import smach
from std_msgs.msg import String
import traceback


class ListenForMeasurement(smach.State):
    OC_RECEIVED = "received"
    OC_FAILURE = "failure"

    OK_POSE_MEASUREMENTS = "pose_measurements"

    def __init__(self):
        super().__init__(
            outcomes=[
                ListenForMeasurement.OC_RECEIVED,
                ListenForMeasurement.OC_FAILURE,
            ],
            output_keys=[ListenForMeasurement.OK_POSE_MEASUREMENTS],
        )

        # Gather the ROS parameters.
        rospy.logdebug("Gathering the ROS parameters.")
        meas_from_viewer_topic = rospy.get_param("measurement_from_viewer_topic")
        wait_timeout = float(rospy.get_param("listen_wait_timeout", 90.0))
        max_messages = int(rospy.get_param("listen_max_messages", 1))
        listen_delay = float(rospy.get_param("listen_delay", 0.0))
        meas_comms_topic = rospy.get_param("meas_comms_topic")

        # Sanity checks.
        rospy.logdebug("Performing the sanity checks.")
        assert isinstance(meas_from_viewer_topic, str)
        assert isinstance(wait_timeout, (int, float))
        assert isinstance(max_messages, int)
        assert isinstance(listen_delay, float)
        assert isinstance(meas_comms_topic, str)
        assert wait_timeout > 0.0
        assert max_messages > 0
        assert listen_delay >= 0.0

        # Subscribe to topic that the viewer will publish on.
        rospy.logdebug(f"Setting up measurement from viewer message callback. Topic = {meas_from_viewer_topic}")
        self.sub = rospy.Subscriber(
            meas_from_viewer_topic,
            PoseWithCovarianceStamped,
            self.pose_measurement_callback,
        )
        self.new_measurement_lock = Lock()
        self.measurements = list()
        self.is_listening_for_measurements = False
        self.listen_start_time = 0.0

        self.wait_timeout = wait_timeout
        self.max_messages = max_messages
        self.listen_delay = listen_delay

        self._is_ready_pub = rospy.Publisher(meas_comms_topic, String, queue_size=1)

    def execute(self, ud):
        outcome: str = ListenForMeasurement.OC_RECEIVED

        # Delay listening for measurements if necessary.
        if self.listen_delay > 0.0:
            rospy.logdebug(
                f"Delaying listening for measurements for {self.listen_delay} s."
            )
            rospy.sleep(self.listen_delay)

        try:
            # Prepare to get new measurements.
            self.listen_start_time = rospy.get_time()
            rospy.logdebug(
                f"Now listening for pose measurements from time {self.listen_start_time}"
            )

            with self.new_measurement_lock:
                self.measurements.clear()
                self.is_listening_for_measurements = True

            # Wait for measurements.
            rospy.logdebug("Publishing the READY message, and waiting for measurements.")
            while True:
                time_elapsed = rospy.get_time() - self.listen_start_time
                if (
                    self.is_listening_for_measurements == True
                    and time_elapsed < self.wait_timeout
                ):
                    self._is_ready_pub.publish(String("READY"))
                    rospy.sleep(0.1)
                else:
                    break

            # Check if we recevied measurements (and the appropriate number).
            with self.new_measurement_lock:
                if time_elapsed >= self.wait_timeout:
                    raise TimeoutError(
                        f"State timed out after {time_elapsed} secs. Was waiting for at least "
                        + f"{self.max_messages} measurements (timeout limit = {self.wait_timeout} s)."
                    )

                if len(self.measurements) < self.max_messages:
                    raise Exception(
                        f"Expected at least {self.max_messages} measurements, but only received {len(self.measurements)}."
                    )

                # Aggregate all the measurements into a list.
                rospy.logdebug(
                    f"Received {len(self.measurements)} measurements, but only keeping {self.max_messages}."
                )
                ud.pose_measurements = list(
                    [meas_msg for meas_msg in self.measurements[: self.max_messages]]
                )

        except:
            rospy.logerr(
                f"An error occurred in {__class__.__name__}:\n{traceback.format_exc()}"
            )
            outcome = ListenForMeasurement.OC_FAILURE

        rospy.loginfo(f"{ListenForMeasurement.__name__} outcome: {outcome}")
        return outcome

    def pose_measurement_callback(self, pose_msg: PoseWithCovarianceStamped) -> None:
        with self.new_measurement_lock:
            # Ignore messages if we are not able to listen to time.
            if self.is_listening_for_measurements == False:
                return None

            # Ignore messages that were created BEFORE we started listening.
            if pose_msg.header.stamp.to_sec() < self.listen_start_time:
                return None

            # Check we reached our measurement limit.
            if len(self.measurements) >= self.max_messages:
                return None

            # Check if we reached the timeout limit.
            if (rospy.get_time() - self.listen_start_time) > self.wait_timeout:
                return None

            # Record the measurement.
            rospy.logdebug(
                f"Received a pose measurement at time {pose_msg.header.stamp.to_sec()} s."
            )
            self.measurements.append(pose_msg)
            if len(self.measurements) >= self.max_messages:
                self.is_listening_for_measurements = False
