"""Launch test for HardsoftCompensator constructor parameter validation.

Loads the component with deliberately bad calibration parameters and checks the
constructor rejects them (logging an error and throwing) rather than starting up
with a meaningless correction. Each bad node lives in its own container, since a
load failure aborts the rest of a container's load sequence. The constructor's
RCLCPP_ERROR goes to stderr, which is what we wait for.

NOTE: requires a built+sourced ROS 2 workspace (run via ``colcon test``).
"""

import unittest

import launch
import launch_testing.actions
import pytest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _container(name, node_name, scale):
    return ComposableNodeContainer(
        name=name,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="magnetic_compensation",
                plugin="mil::magnetic_compensation::HardsoftCompensator",
                name=node_name,
                parameters=[{"shift": [0.0, 0.0, 0.0]}, {"scale": scale}],
            ),
        ],
    )


@pytest.mark.launch_test
def generate_test_description():
    size_container = _container("size_container", "bad_size", [1.0, 0.0, 0.0])
    singular_container = _container(
        "singular_container",
        "bad_singular",
        [0.0] * 9,  # right size, but det == 0
    )

    return (
        launch.LaunchDescription(
            [
                size_container,
                singular_container,
                launch_testing.actions.ReadyToTest(),
            ],
        ),
        {},
    )


class TestParamValidation(unittest.TestCase):
    def test_invalid_size_is_rejected(self, proc_output):
        # The wrong-size scale (3 elements, not 9) must be rejected.
        proc_output.assertWaitFor(
            "Invalid parameter sizes",
            timeout=15,
            stream="stderr",
        )

    def test_singular_scale_is_rejected(self, proc_output):
        # A 9-element but non-invertible scale must be rejected too.
        proc_output.assertWaitFor(
            "Scale matrix is not invertible",
            timeout=15,
            stream="stderr",
        )
