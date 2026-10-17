"""The vendored conformance suite is the zelos-examples one, unedited."""

import importlib.util
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent / "conformance"))
import vendor  # noqa: E402


def test_vendored_copy_matches_its_manifest():
    recorded = vendor.manifest()
    assert set(recorded) == set(vendor.FILES)
    for name, sha in recorded.items():
        assert vendor.digest(vendor.HERE / name) == sha, f"{name} edited: change it upstream"


def test_upstream_has_not_moved_on():
    """Against a zelos-examples checkout named by ZELOS_EXAMPLES; skipped without one."""
    source = os.environ.get("ZELOS_EXAMPLES")
    if not source or not all((Path(source) / f).is_file() for f in vendor.FILES):
        pytest.skip("no zelos-examples checkout with the OBD suite (set ZELOS_EXAMPLES)")
    drifted = [f for f, sha in vendor.manifest().items() if vendor.digest(Path(source) / f) != sha]
    assert not drifted, f"run `just sync-conformance {source}`: {drifted}"


def test_replay_fixtures_match_their_manifest():
    """The vendored real-vehicle traces and oracle (tests/fixtures/obd) are zelos-can's."""
    replay = _replay_vendor()
    recorded = replay.manifest()
    assert set(recorded) == set(replay.FILES)
    for name, sha in recorded.items():
        assert replay.digest(replay.path(name)) == sha, f"{name} edited: change it upstream"


def test_replay_upstream_has_not_moved_on():
    """Against a zelos monorepo checkout named by ZELOS_SRC; skipped without one."""
    replay = _replay_vendor()
    source = os.environ.get("ZELOS_SRC")
    files = [Path(source or "") / replay.SOURCE / f for f in replay.FILES]
    if not source or not all(f.is_file() for f in files):
        pytest.skip("no zelos monorepo checkout with the OBD replay tests (set ZELOS_SRC)")
    drifted = [f for f, p in zip(replay.FILES, files, strict=True)
               if replay.digest(p) != replay.manifest()[f]]  # fmt: skip
    assert not drifted, f"run `just sync-replay {source}`: {drifted}"


def _replay_vendor():
    spec = importlib.util.spec_from_file_location(
        "replay_vendor", Path(__file__).parent / "fixtures" / "obd" / "vendor.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
