from robot.misc import set_config, load_config
import pytest


def test_config_loading(temp_config):
    # Set paths to use temporary directory
    import robot.misc
    robot.misc.CONFIG_ROOT = temp_config / "configs"

    set_config("initial")
    config = load_config()

    assert "geometry" in config
    assert abs(config["geometry"]["d"] - 0.5) < 0.001