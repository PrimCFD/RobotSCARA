import pytest
import json
from pathlib import Path


@pytest.fixture
def temp_config(tmp_path):
    """Create temporary config files for testing"""
    config_dir = tmp_path / "configs"
    config_dir.mkdir()

    # Create initial.json
    initial_config = {
        "geometry": {"d": 0.05},
        "elbow": {"x": 0.3, "y": 0.3, "z": 0.0},
        "initial_pos": {"x": 0.15, "y": 0.0, "z": 0.2}
    }

    (config_dir / "initial.json").write_text(json.dumps(initial_config))

    return tmp_path