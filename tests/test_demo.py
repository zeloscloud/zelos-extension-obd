"""The demo's DTC lifecycle and engine data, byte for byte, where the bus suite cannot reach.

The conformance suite drives two-cycle confirmation, healing and permanent release on the
bus; these are the firmware ztest's remaining cases (lib/uds tests) and the engine's own.
"""

import pytest

from zelos_extension_obd.demo import Responder
from zelos_extension_obd.demo import vehicle as demo

U0111, P0A94 = 3, 0


def report(r: Responder, i: int, failed: bool) -> None:
    with r.lock:
        r._report(i, failed)


def test_dcdc_frame_owner_and_aging_follow_the_firmware():
    """CCR 1968.2 (g)(4.3.1)(D), (d)(2.4): frame 0 is the first stored; confirmed ages 40 cycles."""
    r = Responder("dcdc")
    report(r, U0111, True)
    report(r, P0A94, True)
    assert r.request(b"\x02\x02\x00") == b"\x42\x02\x00\xc1\x11"  # U0111 stored first, not index 0
    r.request(b"\x11\x01")
    report(r, P0A94, True)
    report(r, P0A94, False)
    for _ in range(4):  # the failing cycle, then three that pass: MIL off
        r.request(b"\x11\x01")
        report(r, P0A94, False)
    assert (
        r.request(b"\x03") == b"\x43\x01\x0a\x94"
        and r.request(b"\x01\x01") == b"\x41\x01\x01\x04\x00\x00"
    )
    for _ in range(demo.AGING_CYCLES - 1):
        r.request(b"\x11\x01")
    assert r.request(b"\x03") == b"\x43\x01\x0a\x94"
    r.request(b"\x11\x01")
    assert r.request(b"\x03") == b"\x43\x00"
    assert (
        r.request(b"\x19\x04\x0a\x94\x00\x00") == b"\x59\x04\x0a\x94\x00\x60"
    )  # frames went with it


def test_engine_results_and_ratios_follow_a_clear(monkeypatch):
    """CCR 1968.2 (g)(4.5.4), (g)(5.2.1)(B), (g)(4.2.2)(A): results reset by a clear, ratios not."""
    monkeypatch.setattr(demo, "MONITOR_RUN_S", 0.0)
    r = Responder("engine")
    assert r.request(b"\x06\x00") == b"\x46\x00\x80\x00\x00\x01"  # MID 01, and more
    catalyst = r.request(b"\x06\x21")
    assert catalyst == b"\x46\x21\x80\x20\x00\x5a\x00\x00\x00\xc0"  # MID TID UASID value min max
    ratios = r.request(b"\x09\x08")
    assert ratios[:3] == b"\x49\x08\x14" and len(ratios) == 3 + 2 * 20
    r.request(b"\x04")
    assert r.request(b"\x06\x21") == b"\x46\x21\x80\x20" + bytes(6)
    assert r.request(b"\x09\x08") == ratios  # never cleared
    # J1979-2, per DTC: the same result (zeroed) and ratio (kept)
    assert r.request(b"\x19\x06\x04\x20\x00\x92")[-7:] == b"\x20" + bytes(6)
    assert r.request(b"\x19\x06\x04\x20\x00\x91")[-4:] == ratios[7:11]  # CAT1
    monkeypatch.setattr(demo, "MONITOR_RUN_S", 60.0)
    r.request(b"\x11\x01")
    assert (
        r.request(b"\x01\x41") == b"\x41\x41\x00\x07\xe5\xe5"
    )  # enabled, not yet complete this cycle
    monkeypatch.setattr(demo, "MONITOR_RUN_S", 0.0)
    assert r.request(b"\x01\x41") == b"\x41\x41\x00\x07\xe5\x00"
    assert r.request(b"\x06\x21") == catalyst
    assert (
        int.from_bytes(r.request(b"\x09\x08")[5:7], "big") == int.from_bytes(ratios[5:7], "big") + 1
    )  # IGNCNTR


def test_demo_refuses_a_real_bus():
    """Fake emission ECUs on a vehicle bus would answer real testers and ack their clears."""
    for interface, channel in (("socketcan", "can0"), ("pcan", "PCAN_USBBUS1")):
        with pytest.raises(ValueError, match="virtual bus only"):
            demo.DemoVehicle(channel, interface=interface)
