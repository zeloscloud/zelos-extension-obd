"""The settings form's schema, and what create_vehicles refuses."""

import json
from pathlib import Path

import jsonschema
import pytest

from zelos_extension_obd.cli.app import ConfigError, create_vehicles, resolve_advanced

SCHEMA = json.loads((Path(__file__).parent.parent / "config.schema.json").read_text())


def test_schema_takes_each_interface_and_create_vehicles_builds_them():
    config = {
        "vehicles": [
            {"interface": "Demo"},
            {"interface": "PCAN", "channel": "PCAN_USBBUS1", "bitrate": 250000,
             "addressing": "29-bit"},
            {"interface": "DoIP", "doip": {"host": "127.0.0.1"}},
            {"interface": "SocketCAN", "channel": "can0", "protocol": "uds",
             "polling": {"pids": ["EngineRPM"], "period_s": 0.5}},
        ]
    }  # fmt: skip
    jsonschema.Draft7Validator.check_schema(SCHEMA)
    assert list(jsonschema.Draft7Validator(SCHEMA).iter_errors(config)) == []
    vehicles = create_vehicles(config, resolve_advanced(config))
    assert [v.name for v in vehicles] == ["demo", "PCAN_USBBUS1", "doip", "can0"]
    assert vehicles[2].status()["functional_address"] == "0xE400"
    assert vehicles[1].status()["functional_id"] == "0x18DB33F1"


@pytest.mark.parametrize(
    "vehicles, error",
    [
        ([], "No vehicles"),
        ([{"interface": "Demo"}, {"interface": "Demo"}], "Duplicate"),
        ([{"interface": "SocketCAN", "channel": "can0"}] * 2, "Duplicate"),
        ([{"interface": "SocketCAN", "channel": "can0", "name": "a"},
          {"interface": "SocketCAN", "channel": "can0", "name": "b"}], "one bus"),
        ([{"interface": "Demo", "name": "a/b"}], "not allowed"),
        ([{"interface": "Demo", "addressing": "16-bit"}], "addressing"),
        ([{"interface": "PCAN", "channel": "PCAN_USBBUS1",
           "config_json": '{"bitrate": 125000}'}], "use the Bitrate setting"),
    ],
)  # fmt: skip
def test_refused(vehicles, error):
    with pytest.raises(ConfigError, match=error):
        create_vehicles({"vehicles": vehicles}, resolve_advanced({}))
