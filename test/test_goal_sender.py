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

"""Unit tests for the GoalSender node."""

from unittest.mock import MagicMock, patch

import pytest
import rclpy

from my_navigator.goal_sender import GoalSender


@pytest.fixture(scope='module')
def rclpy_init():
    """Initialize and shutdown rclpy around the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(mock_nav_class):
    """Helper: build a GoalSender with a fully mocked BasicNavigator."""
    mock_nav = MagicMock()
    mock_nav_class.return_value = mock_nav
    mock_nav.waitUntilNav2Active.return_value = None
    mock_nav.setInitialPose.return_value = None
    return GoalSender(), mock_nav


@patch('my_navigator.goal_sender.BasicNavigator')
@patch('my_navigator.goal_sender.time.sleep', return_value=None)
def test_create_goal_pose_values(mock_sleep, mock_nav_class, rclpy_init):
    """Test that create_goal_pose sets x and y correctly."""
    node, _ = _make_node(mock_nav_class)

    pose = node.create_goal_pose(2.5, -1.0)

    assert pose.pose.position.x == pytest.approx(2.5)
    assert pose.pose.position.y == pytest.approx(-1.0)
    assert pose.pose.position.z == pytest.approx(0.0)
    assert pose.pose.orientation.w == pytest.approx(1.0)
    assert pose.header.frame_id == 'map'

    node.destroy_node()


@patch('my_navigator.goal_sender.BasicNavigator')
@patch('my_navigator.goal_sender.time.sleep', return_value=None)
def test_get_waypoints_parsed_correctly(mock_sleep, mock_nav_class, rclpy_init):
    """Test that flat waypoints parameter is parsed into (x, y) tuples."""
    node, _ = _make_node(mock_nav_class)

    node.set_parameters([
        rclpy.parameter.Parameter(
            'waypoints',
            rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
            [1.0, 2.0, 3.0, 4.0]
        )
    ])

    waypoints = node._get_waypoints()

    assert waypoints == [(1.0, 2.0), (3.0, 4.0)]

    node.destroy_node()


@patch('my_navigator.goal_sender.BasicNavigator')
@patch('my_navigator.goal_sender.time.sleep', return_value=None)
def test_get_waypoints_raises_on_odd_count(mock_sleep, mock_nav_class, rclpy_init):
    """Test that an odd-length waypoints list raises ValueError."""
    node, _ = _make_node(mock_nav_class)

    node.set_parameters([
        rclpy.parameter.Parameter(
            'waypoints',
            rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
            [1.0, 2.0, 3.0]  # odd — invalid
        )
    ])

    with pytest.raises(ValueError):
        node._get_waypoints()

    node.destroy_node()


@patch('my_navigator.goal_sender.BasicNavigator')
@patch('my_navigator.goal_sender.time.sleep', return_value=None)
def test_navigate_to_succeeds(mock_sleep, mock_nav_class, rclpy_init):
    """Test that _navigate_to returns SUCCEEDED when Nav2 succeeds first try."""
    from nav2_simple_commander.robot_navigator import TaskResult

    node, mock_nav = _make_node(mock_nav_class)

    # Simulate: task completes immediately, result is SUCCEEDED
    mock_nav.isTaskComplete.return_value = True
    mock_nav.getResult.return_value = TaskResult.SUCCEEDED

    result = node._navigate_to(1.0, 0.5, 1, 1)

    assert result == TaskResult.SUCCEEDED
    mock_nav.goToPose.assert_called_once()

    node.destroy_node()


@patch('my_navigator.goal_sender.BasicNavigator')
@patch('my_navigator.goal_sender.time.sleep', return_value=None)
def test_navigate_to_retries_on_failure(mock_sleep, mock_nav_class, rclpy_init):
    """Test that _navigate_to retries up to max_retries on FAILED result."""
    from nav2_simple_commander.robot_navigator import TaskResult

    node, mock_nav = _make_node(mock_nav_class)

    # All attempts fail
    mock_nav.isTaskComplete.return_value = True
    mock_nav.getResult.return_value = TaskResult.FAILED

    node.set_parameters([
        rclpy.parameter.Parameter(
            'max_retries',
            rclpy.parameter.Parameter.Type.INTEGER,
            2
        )
    ])

    result = node._navigate_to(1.0, 0.5, 1, 1)

    assert result == TaskResult.FAILED
    # goToPose should have been called max_retries times
    assert mock_nav.goToPose.call_count == 2

    node.destroy_node()


@patch('my_navigator.goal_sender.BasicNavigator')
@patch('my_navigator.goal_sender.time.sleep', return_value=None)
def test_feedback_none_does_not_crash(mock_sleep, mock_nav_class, rclpy_init):
    """Test that None feedback during navigation is handled gracefully."""
    from nav2_simple_commander.robot_navigator import TaskResult

    node, mock_nav = _make_node(mock_nav_class)

    # First call: not complete (triggers feedback poll); second: complete
    mock_nav.isTaskComplete.side_effect = [False, True]
    mock_nav.getFeedback.return_value = None   # FIX: None feedback must not crash
    mock_nav.getResult.return_value = TaskResult.SUCCEEDED

    # Should not raise any exception
    with patch('rclpy.spin_once', return_value=None):
        result = node._navigate_to(1.0, 0.5, 1, 1)

    assert result == TaskResult.SUCCEEDED

    node.destroy_node()
