"""Zelos OBD-II

A Zelos extension for OBD-II (SAE J1979, and J1979-2 OBDonUDS) scan-tool
diagnostics over CAN (ISO 15765-4) or DoIP.
"""

#: Action namespace for `zelos_sdk.init` and the at-rest inventory; matches `extension.toml` name.
ACTION_PREFIX = "OBD"

__all__: list[str] = ["ACTION_PREFIX"]
