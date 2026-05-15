# Copyright 2025 Pavan Kumar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 waypoint navigator node using Nav2 simple commander."""

import time

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult
)


class GoalSender(Node):
    """ROS 2 node that sends sequential waypoints to the Nav2 stack."""

    def __init__(self):
        """Initialize the GoalSender node, parameters, and Nav2 connection."""
        super().__init__('goal_sender')

        # Declare waypoints parameter as a flat list: [x1, y1, x2, y2, ...]
        # FIX: Default waypoints updated to known free-space positions on the
        # TurtleBot3 default map (avoid (0,0) which may fall in an obstacle).
        self.declare_parameter(
            'waypoints',
            [1.0, 0.5, -1.0, 0.5, 0.0, 0.0]
        )

        # Declare initial pose parameters.
        # FIX: Default changed from (0.0, 0.0) to the TurtleBot3 Gazebo
        # default spawn position (-2.0, -0.5), which is a known free cell.
        # Override these in waypoints.yaml if your spawn differs.
        self.declare_parameter('initial_pose_x', -2.0)
        self.declare_parameter('initial_pose_y', -0.5)

        # Declare retry parameters for failed waypoints
        self.declare_parameter('max_retries', 3)

        self.navigator = BasicNavigator()

        # FIX: Wait for TF tree to stabilise before publishing the initial
        # pose. Without this, setInitialPose() races the transform buffer and
        # triggers "Lookup would require extrapolation into the future" warnings
        # that can cause AMCL to silently drop the pose.
        self.get_logger().info('Waiting for TF tree to stabilise...')
        time.sleep(1.0)

        self._set_initial_pose()

        self.get_logger().info(
            'Waiting for Nav2 to become active...'
        )

        self.navigator.waitUntilNav2Active()

        self.get_logger().info(
            'Nav2 is now active!'
        )

    def _set_initial_pose(self):
        """Set the robot's initial pose before starting navigation."""
        x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        y = self.get_parameter('initial_pose_y').get_parameter_value().double_value

        initial_pose = self.create_goal_pose(x, y)
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info(
            f'Initial pose set to ({x}, {y})'
        )

    def create_goal_pose(self, x, y):
        """Create and return a PoseStamped goal message at position (x, y).

        Args:
            x (float): Target X position in the map frame.
            y (float): Target Y position in the map frame.

        Returns:
            PoseStamped: A goal pose stamped message ready for Nav2.

        """
        goal_pose = PoseStamped()

        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        return goal_pose

    def _get_waypoints(self):
        """Retrieve and parse waypoints from ROS 2 parameters.

        Returns:
            list[tuple[float, float]]: List of (x, y) waypoint tuples.

        Raises:
            ValueError: If the waypoints parameter length is not even.

        """
        flat = (
            self.get_parameter('waypoints')
            .get_parameter_value()
            .double_array_value
        )

        if len(flat) % 2 != 0:
            raise ValueError(
                'Waypoints parameter must have an even number of values (x, y pairs).'
            )

        return [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]

    def _navigate_to(self, x, y, waypoint_index, total):
        """Navigate to a single waypoint with retry logic.

        Args:
            x (float): Target X position.
            y (float): Target Y position.
            waypoint_index (int): 1-based index for logging.
            total (int): Total number of waypoints.

        Returns:
            TaskResult: The final result of navigation.

        """
        max_retries = self.get_parameter('max_retries').get_parameter_value().integer_value

        for attempt in range(1, max_retries + 1):

            if attempt > 1:
                self.get_logger().warn(
                    f'Retrying waypoint {waypoint_index} '
                    f'(attempt {attempt}/{max_retries})...'
                )

            goal_pose = self.create_goal_pose(x, y)
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():

                feedback = self.navigator.getFeedback()

                # FIX: Guard against None feedback (returned while Nav2 is
                # still planning or retrying). Log "planning..." instead of
                # printing a misleading 0.00 m distance.
                if feedback is not None:
                    self.get_logger().info(
                        f'Distance remaining: '
                        f'{feedback.distance_remaining:.2f} m'
                    )
                else:
                    self.get_logger().info(
                        'Planning in progress...'
                    )

                rclpy.spin_once(self, timeout_sec=0.5)

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                return result

            if result == TaskResult.CANCELED:
                return result

            # FAILED — retry if attempts remain
            self.get_logger().warn(
                f'Waypoint {waypoint_index} attempt {attempt} failed.'
            )

        return TaskResult.FAILED

    def send_waypoints(self):
        """Navigate the robot through all configured waypoints sequentially."""
        try:
            waypoints = self._get_waypoints()
        except ValueError as e:
            self.get_logger().error(f'Invalid waypoints parameter: {e}')
            return

        for i, (x, y) in enumerate(waypoints):

            self.get_logger().info(
                f'Navigating to waypoint {i + 1}/{len(waypoints)}: ({x}, {y})'
            )

            result = self._navigate_to(x, y, i + 1, len(waypoints))

            if result == TaskResult.SUCCEEDED:

                self.get_logger().info(
                    f'Waypoint {i + 1} reached successfully!'
                )

            elif result == TaskResult.CANCELED:

                self.get_logger().warn(
                    'Navigation was canceled!'
                )

                return

            elif result == TaskResult.FAILED:

                self.get_logger().error(
                    f'Navigation failed at waypoint {i + 1} '
                    f'after all retries. Aborting.'
                )

                return

        self.get_logger().info(
            'All waypoints completed successfully!'
        )


def main(args=None):
    """Entry point for the goal_sender ROS 2 node."""
    rclpy.init(args=args)

    node = GoalSender()

    try:

        node.send_waypoints()

    except KeyboardInterrupt:

        node.get_logger().info(
            'Program interrupted by user.'
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
