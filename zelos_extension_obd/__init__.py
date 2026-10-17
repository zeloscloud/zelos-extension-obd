"""Zelos OBD-II

A Zelos extension for OBD-II (SAE J1979, and J1979-2 OBDonUDS) scan-tool
diagnostics over CAN (ISO 15765-4) or DoIP.
"""

#: Action namespace: `zelos_sdk.init(name=ACTION_PREFIX)` and the at-rest inventory
#: `main.py` re-exports. Matches `name` in `extension.toml`.
ACTION_PREFIX = "OBD"

__all__: list[str] = ["ACTION_PREFIX"]
