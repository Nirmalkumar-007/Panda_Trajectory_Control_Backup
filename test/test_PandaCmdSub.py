import rclpy
import pytest
import numpy as np
from unittest.mock import MagicMock, patch
from itertools import cycle
from panda_trajectory.PandaCmdSub import PandaCmdSub

@pytest.fixture(scope="module", autouse=True)
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@patch("builtins.exit")                   # prevent actual exit
@patch("time.sleep", lambda _: None)      # skip delays
@patch("builtins.input", side_effect=cycle(["f", "h"]))  # repeat 'f', then 'h'
def test_pandacmdsub_initialization(mock_input, mock_exit):
    mock_desk = MagicMock()
    mock_panda = MagicMock()
    mock_panda.get_position.return_value = np.array([0.0, 0.0, 0.0])
    mock_panda.get_orientation.return_value = np.array([0.0, 0.0, 0.0, 1.0])
    mock_panda.get_state.return_value.q = np.zeros(7)
    mock_panda.start_controller.return_value = None
    mock_panda.move_to_joint_position.return_value = None

    node = PandaCmdSub(mock_desk, mock_panda)

    np.testing.assert_array_equal(node.home_pos, np.array([0.0, 0.0, 0.0]))
    assert node.tilted_quat.shape == (4,)
    assert len(node.waypoints) == len(node.orientations)
    assert all(np.allclose(o, node.tilted_quat) for o in node.orientations)

    # trajectory assertions
    assert mock_panda.start_controller.call_count >= 1
    assert mock_panda.move_to_joint_position.call_count >= 1
    assert mock_exit.called

"""
# without exit patch, to actually test exit behavior

import rclpy
import pytest
import numpy as np
from unittest.mock import MagicMock
from panda_trajectory.PandaCmdSub import PandaCmdSub

@pytest.fixture(scope="module", autouse=True)
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_pandacmdsub_initialization_real():
    # Create mock desk and panda objects
    mock_desk = MagicMock()
    mock_panda = MagicMock()

    # Return values for Panda methods
    mock_panda.get_position.return_value = np.array([0.0, 0.0, 0.0])
    mock_panda.get_orientation.return_value = np.array([0.0, 0.0, 0.0, 1.0])
    mock_panda.get_state.return_value.q = np.zeros(7)
    mock_panda.start_controller.return_value = None
    mock_panda.move_to_joint_position.return_value = None

    # Initialize node — will wait for real user input at waypoints
    node = PandaCmdSub(mock_desk, mock_panda)

    # Assertions on initialization
    np.testing.assert_array_equal(node.home_pos, np.array([0.0, 0.0, 0.0]))
    assert node.tilted_quat.shape == (4,)
    assert len(node.waypoints) == len(node.orientations)
    assert all(np.allclose(o, node.tilted_quat) for o in node.orientations)

    """