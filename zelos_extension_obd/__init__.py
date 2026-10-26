"""Zelos OBD-II

A Zelos extension for OBD-II (SAE J1979, and J1979-2 OBDonUDS) scan-tool
diagnostics over CAN (ISO 15765-4) or DoIP.
"""

#: Must equal `name` in extension.toml: the app and the packager address actions by it.
ACTION_PREFIX = "OBD"

__all__: list[str] = ["ACTION_PREFIX"]
