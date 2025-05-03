from robot.misc import speed_ramp
import numpy as np
import pytest

def test_velocity_ramp():
    """Verify trapezoidal velocity profile generation"""

    t, v, a = speed_ramp(2.0, 1.0, np.pi / 2, 100)

    # Check critical profile points
    assert v[0] == 0.0
    assert v[-1] == 0.0
    assert max(v) == pytest.approx(1.0, rel=0.1)