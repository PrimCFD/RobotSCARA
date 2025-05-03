# tests/performance/test_gui.py
import pytest
from PyQt5 import QtWidgets
from robot.gui import MainWindow


@pytest.fixture(scope="function")
def qt_app():
    """Fixture to handle Qt application lifecycle"""
    app = QtWidgets.QApplication([])
    yield app
    app.quit()


def test_gui_refresh_performance(benchmark, qt_app):
    """Measure GUI responsiveness during heavy computation"""
    window = MainWindow((1400, 900), 30)

    # Benchmark just the refresh cycle without setup
    benchmark.pedantic(
        lambda: (
            window.start_computation_in_thread_velo(),
            qt_app.processEvents()
        ),
        rounds=10,
        iterations=5
    )

    # Performance requirement: 30Hz refresh => ~32ms per frame
    assert benchmark.stats['mean'] < 0.040  # 32ms max per refresh cycle + tolerance for testing