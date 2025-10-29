#!/usr/bin/env python3

import functools
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from trusses_custom_interfaces.msg import SpatialMeasurement

from safe_scout_simulator.srv import SampleGroundTruth


class SpatialMeasurementPublisher(Node):
    """Periodically query the ground truth service and publish SpatialMeasurement data."""

    def __init__(self) -> None:
        super().__init__("spatial_measurement_publisher")

        # Parameters allow topic names and timer interval to be configured
        self.declare_parameter("pose_topic", "spirit/current_pose")
        self.declare_parameter("measurement_topic", "spirit/spatial_measurements")
        self.declare_parameter("publish_interval", 1.0)
        self.declare_parameter("service_name", "sample_ground_truth")
        self.declare_parameter("source_name", "ground_truth")
        self.declare_parameter("unit", "unitless")
        self.declare_parameter("default_uncertainty", 0.0)
        self.declare_parameter("gaussian_noise_stddev", 0.0)

        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        measurement_topic = (
            self.get_parameter("measurement_topic").get_parameter_value().string_value
        )
        service_name = self.get_parameter("service_name").get_parameter_value().string_value
        publish_interval = (
            self.get_parameter("publish_interval").get_parameter_value().double_value
        )

        self.source_name = self.get_parameter("source_name").get_parameter_value().string_value
        self.unit = self.get_parameter("unit").get_parameter_value().string_value
        self.default_uncertainty = (
            self.get_parameter("default_uncertainty").get_parameter_value().double_value
        )

        # Subscriptions and publications
        self.pose_sub = self.create_subscription(Pose, pose_topic, self._pose_callback, 10)
        self.spatial_pub = self.create_publisher(SpatialMeasurement, measurement_topic, 10)

        self.client = self.create_client(SampleGroundTruth, service_name)
        self._reported_waiting = False
        self._pending_future = None
        self._current_pose = None

        self.timer = self.create_timer(publish_interval, self._timer_callback)

    def _pose_callback(self, msg: Pose) -> None:
        # Save the latest pose for use during timer callbacks
        self._current_pose = msg

    def _timer_callback(self) -> None:
        if self._current_pose is None or self._pending_future is not None:
            return

        if not self.client.service_is_ready():
            if not self._reported_waiting:
                self.get_logger().warn("Waiting for ground truth service to become available...")
                self._reported_waiting = True
            return
        self._reported_waiting = False

        pose_snapshot = Pose()
        pose_snapshot.position.x = self._current_pose.position.x
        pose_snapshot.position.y = self._current_pose.position.y
        pose_snapshot.position.z = self._current_pose.position.z
        pose_snapshot.orientation.x = self._current_pose.orientation.x
        pose_snapshot.orientation.y = self._current_pose.orientation.y
        pose_snapshot.orientation.z = self._current_pose.orientation.z
        pose_snapshot.orientation.w = self._current_pose.orientation.w

        request = SampleGroundTruth.Request()
        request.x = pose_snapshot.position.x
        request.y = pose_snapshot.position.y

        self._pending_future = self.client.call_async(request)
        self._pending_future.add_done_callback(
            functools.partial(self._handle_service_response, pose=pose_snapshot)
        )

    def _handle_service_response(self, future, pose: Pose) -> None:
        self._pending_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Service call failed: {exc}")
            return

        measurement = SpatialMeasurement()
        measurement.position.x = pose.position.x
        measurement.position.y = pose.position.y
        measurement.position.z = pose.position.z
        noise_stddev = (
            self.get_parameter("gaussian_noise_stddev").get_parameter_value().double_value
        )
        noise = random.gauss(0.0, noise_stddev) if noise_stddev > 0.0 else 0.0
        measurement.value = response.value + noise
        measurement.unit = self.unit
        measurement.source_name = self.source_name
        measurement.uncertainty = self.default_uncertainty
        measurement.time = self.get_clock().now().to_msg()

        self.spatial_pub.publish(measurement)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpatialMeasurementPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
