"""OBD-II scan tool extension for Zelos

A Zelos extension for sensor monitoring.
"""

from .extension import SensorMonitor

__all__: list[str] = [
    "SensorMonitor",
]
