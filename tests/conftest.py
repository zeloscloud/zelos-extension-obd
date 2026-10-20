"""Shared pytest setup."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))


def pytest_configure(config):
    """Zelos SDK pytest plugin attributes."""
    config.zelos_local_artifacts_dir = None
    config.zelos_remote_artifacts_dir = None
    config.zelos_device_id = None
