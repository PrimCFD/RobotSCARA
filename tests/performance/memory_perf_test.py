import pytest
import tracemalloc
from robot.kinematics import Compute_Workspace


def test_memory_usage():
    """Detect memory leaks in workspace computation"""
    tracemalloc.start()

    # Record initial memory
    snapshot1 = tracemalloc.take_snapshot()

    # Perform operation multiple times
    for _ in range(10):
        Compute_Workspace((-0.5, 0.5), (-0.5, 0.5), (0, 0.5), 30, 30, 30)

    # Measure memory growth
    snapshot2 = tracemalloc.take_snapshot()
    top_stats = snapshot2.compare_to(snapshot1, 'lineno')

    # Allow 100KB growth max
    assert sum(stat.size_diff for stat in top_stats) < 100 * 1024, \
        "Significant memory leak detected"