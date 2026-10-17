# Zelos OBD-II

A Zelos extension that is an OBD-II scan tool: live PIDs, DTCs, readiness monitors, freeze frames, vehicle information and a gated DTC clear, over CAN (ISO 15765-4) or DoIP (ISO 13400-2). Legacy OBD-II (SAE J1979 / ISO 15031-5) and OBDonUDS (SAE J1979-2), from the Zelos App, app extensions or the shell.

Every action except `list_vehicles`, `get_status` and the settings helpers transmits on the vehicle's bus. Requests are functional (0x7DF / 0x18DB33F1): every emissions ECU that supports a request answers it, and results are per ECU.

## Features

- **Scan tool**: detect, supported PIDs, live data with units, MIL and readiness, stored / pending / permanent DTCs, freeze frame, VIN / CALID / CVN / ECU name, monitor test results
- **Gated clear**: `clear_dtcs` clears only with `confirm`; without it, it lists what would be erased and sends no clear
- **Both protocols**: J1979 and J1979-2, probed or configured; 11-bit and 29-bit; 500 and 250 kbit/s
- **Polling**: PIDs read at a fixed period into the trace, plot-ready per ECU
- **CAN adapters**: SocketCAN, SocketCAN over SSH, PCAN, Kvaser, Vector, slcan, any python-can interface
- **Demo vehicle**: three simulated ECUs to try every action without hardware

## Quick Start

```bash
git clone https://github.com/zeloscloud/zelos-extension-obd
cd zelos-extension-obd
just install
uv run main.py --demo
```

## Configuration

All configuration is managed through the Zelos App settings interface.

### Per-vehicle settings

| Setting | Default | Description |
|---|---|---|
| **Interface** | | SocketCAN (Linux), SocketCAN over SSH, PCAN, Kvaser, Vector, slcan (serial), Other (python-can: gs_usb, ...), Demo, DoIP ([below](#doip)) |
| **Name** | CAN channel, `demo` or `doip` | Name the actions take as `vehicle`, and the trace segment |
| **Channel**, **Bitrate**, **Advanced Configuration (JSON)** | as the CAN extension | The adapter. Bitrate 500000 or 250000 where the extension sets it |
| **Addressing** | `11-bit` | `11-bit` (0x7DF, answers on 0x7E8-0x7EF) or `29-bit` (0x18DB33F1, answers on 0x18DAF1xx). `detect` finds it |
| **Protocol** | `auto` | `auto`: 01 00, then 22 F400 if nothing answers. `legacy`: J1979. `uds`: J1979-2 |
| **Polling** | none | `pids` (catalogue names or hex) and `period_s`, read from the start |

A bus is one vehicle: two entries on one interface and channel are refused. PCAN, Kvaser and Vector open a channel once: this extension and the UDS or CAN extension cannot share one adapter channel.

### DoIP

| Setting | Default | Description |
|---|---|---|
| `host` | | IP or hostname of the vehicle's DoIP entity. Empty with Discover: broadcast |
| `port` | `13400` | TCP port |
| `tester_address` | `0x0E80` | Our logical address |
| `ecu_address` | `0xE400` | Target: the functional group (every ECU answers from its own address), or one ECU |
| `activation_type` | `0x00` | Routing activation type |
| `tcp_timeout` | `2.0` | Seconds for the TCP connect and each acknowledgement |
| `discover` | off | Vehicle identification (UDP 13400) first; the answer shows in `get_status` |

### Advanced settings

| Setting | Default | Description |
|---|---|---|
| `prefix` | `OBD` | Trace source every vehicle publishes under; cleared, each vehicle has its own |
| `log_level` | `INFO` | `DEBUG`, `INFO`, `WARNING`, `ERROR` |
| `p2_timeout` | `0.1` | Seconds to wait for the first, and each further, ECU's answer (ISO P2CAN is 0.05). Raise over SSH or a slow gateway |
| `p2_star_timeout` | `5.0` | Seconds after each response-pending (NRC 0x78) answer |
| `isotp_padding` | `204` (0xCC) | Padding byte; every frame is 8 bytes |
| `debug_frames` | off | Log ISO-TP frames and DoIP messages at `DEBUG` |

### Trace layout

| Event | Fields |
|---|---|
| `OBD/<vehicle>/tx` | `service` (value table), `pid` (PID, DID or infotype), `address` (functional id or DoIP target), `data` (after the SID) |
| `OBD/<vehicle>/rx` | `service`, `pid`, `responder` (CAN id or DoIP address), `data`, `success`, `nrc`, `elapsed_ms` |
| `OBD/<vehicle>/<responder>/pids` | one row per answer, one column per catalogue PID column (`EngineRPM` rpm, `MIL`, `DTCCount`, readiness Booleans, ...), `service` |
| `OBD/<vehicle>/<responder>/freeze_frame` | `frame` and the PID columns |
| `OBD/<vehicle>/<responder>/dtc` | one row per DTC: `code` (`P0420`, `P0420-00`), `raw`, `kind` (stored / pending / permanent), `count`; J1979-2: `status`, its 8 ISO 14229-1 bits, `severity`. An empty list is one row with `count` 0 |
| `OBD/<vehicle>/<responder>/vehicle_info` | `infotype`, `vin`, `calid`, `cvn`, `ecu_name` |

`<responder>` is the CAN id as the CAN extension's passive decode names it (`07e8`, `18daf110`), or the DoIP address (`1001`). Columns are the `zelos_can.obd` catalogue's, so active and passive rows line up.

## Actions

Available as `OBD/<action>`. Each takes `vehicle`. A result is `responders`: one entry per ECU that answered. No answer at all is `status: error`.

| Action | Service | Description |
|---|---|---|
| `list_vehicles`, `get_status` | | Vehicles; state, protocol, responders seen, polling tasks, counters. Send nothing |
| `detect` | 01 00 / 22 F400 | ISO 15765-4 initialization: 11-bit and 29-bit at 500 then 250 kbit/s, first answered kept. Bitrates only on adapters the extension sets; a wrong bitrate causes error frames on a running bus |
| `supported_pids` | 01/02/09 00, 20, ... | Bitmap walk per ECU |
| `read_pids` | 01 / 22 F4xx | Up to 6 PIDs per request (J1979-2: one), decoded with units |
| `monitor_status` | 01 01 / 22 F501 | MIL, DTC count, each supported monitor complete / incomplete, `ready` |
| `read_dtcs` | 03 07 0A / 19 42 19 55 | Stored, pending, permanent |
| `freeze_frame` | 02 | Frame 0: its DTC and PIDs (J1979 only) |
| `vehicle_info` | 09 02/04/06/0A / 22 F8xx | VIN, CALID, CVN, ECU name |
| `test_results` | 06 / 22 F6xx | One monitor id, raw |
| `clear_dtcs` | 04 / 14 FFFF33 | `confirm` off (default): reads 03/07, returns `status: refused`, `would_clear` and the warning; no clear is sent. On: clears every ECU, each one's acknowledgement (NRC 0x22: key on, engine off) |
| `start_polling`, `stop_polling` | 01 | PIDs every `period_s` into the trace; `task_id` per PID set |
| `auto_config`, `list_interfaces` | | Settings form helpers; run with the extension stopped |

A clear erases stored and pending DTCs, freeze frames and readiness: every monitor reads not complete until a full drive cycle, and an inspection fails meanwhile. Permanent DTCs remain.

## CLI Usage

```bash
uv run main.py --demo                                   # app mode plus a demo vehicle
uv run main.py read --interface demo --pids EngineRPM,VehicleSpeed
uv run main.py dtcs --interface pcan --channel PCAN_USBBUS1
uv run main.py detect --interface socketcan --channel can0
uv run main.py clear --interface demo                   # dry run: lists, clears nothing
uv run main.py clear --interface demo --yes
uv run main.py info --interface doip --host 192.168.77.2
```

| Subcommand | Action |
|---|---|
| `detect`, `pids` (`--service`), `read` (`--pids`) | `detect`, `supported_pids`, `read_pids` |
| `status`, `dtcs`, `freeze`, `info`, `tests` (`--mid`) | `monitor_status`, `read_dtcs`, `freeze_frame`, `vehicle_info`, `test_results` |
| `clear` (`--yes`) | `clear_dtcs` (`confirm`) |
| `demo-ecu` | Runs the demo vehicle on any python-can bus |

Every subcommand takes `--interface`, `--channel`, `--bitrate`, `--addressing`, `--protocol`, `--p2`; `other` takes `--config-json`, `zelos-ssh-socketcan` `--remote-host`, `doip` `--host`, `--port`, `--tester-address`, `--ecu-address`, `--discover`. Output is JSON; exit 1 on error.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md).

### Demo vehicle

`zelos_extension_obd/demo/` is the bench DC-DC node's OBD-II responder in Python, to its firmware's contract, plus an engine ECU with readiness monitors: 0x7E8 engine (P0420 confirmed and permanent, P0171 pending), 0x7E9 DC-DC, 0x7EA DC-DC with a lost BMS (U0111 active, back after a clear). 29-bit: 0x18DAF110 + n.

The zelos-examples OBD-II conformance suite is vendored in `tests/conformance` and runs in `just test` against one DC-DC responder, on CAN and through the DoIP entity simulator (`tests/doip_entity.py`: tester 0x0E80, ECUs 0x1001 + n, functional 0xE400). Update it with `just sync-conformance ~/zelos/zelos-examples`; `ZELOS_EXAMPLES=<checkout> just test` checks the copy against upstream.

```bash
just sim                       # vcan0, Linux
just sim-doip                  # DoIP entity on 127.0.0.1:13400
uv run main.py demo-ecu --help # --extended, --responders, --uds-only
```

## Links

- **Repository**: [github.com/zeloscloud/zelos-extension-obd](https://github.com/zeloscloud/zelos-extension-obd)
- **Issues**: [Report bugs or request features](https://github.com/zeloscloud/zelos-extension-obd/issues)
- [Zelos Documentation](https://docs.zeloscloud.io)

## License

MIT License - see [LICENSE](LICENSE) for details.
