import pytest
from robot.kinematics import Compute_Workspace, Compute_kine_traj
from robot.misc import constants


@pytest.mark.benchmark(group="workspace-computation")
def test_workspace_performance(benchmark):
    """Benchmark workspace computation at different resolutions"""

    def setup():
        return (-0.5, 0.5), (-0.5, 0.5), (0, 0.5), 50, 50, 50  # High-res grid

    result = benchmark.pedantic(Compute_Workspace,
                                args=setup(),
                                rounds=3,
                                iterations=1)

    # Assert performance requirements
    assert benchmark.stats['mean'] < 2.0  # Max 2 seconds for 50x50x50 grid


@pytest.mark.benchmark(group="trajectory-calculation")
def test_trajectory_performance(benchmark):
    """Benchmark complex trajectory generation"""

    def setup():
        return (0.15, 0.0, 0.2, 1.57, 3.14, True, True)  # Large angular movement

    result = benchmark.pedantic(Compute_kine_traj,
                                args=setup(),
                                rounds=10,
                                iterations=5)

    assert benchmark.stats['mean'] < 0.5  # 500ms max for complex path