# Contributing

## Prerequisites

- [Zelos](https://zeloscloud.io) 26.0.10 or later
- [Zelos CLI](https://docs.zeloscloud.io/reference/cli/) 0.1.11 or later
- [just](https://github.com/casey/just)
- [uv](https://docs.astral.sh/uv/)

## Layout

`extension.toml` describes everything the extension contributes:

| Part | Files | Manifest |
| ---- | ----- | -------- |
| Agent runtime | `main.py`, the Python package, `pyproject.toml`, `config.schema.json` | `[agent]`, `[config]` |

| Path | Contents |
| ---- | -------- |
| `zelos_extension_obd/actions.py` | The `OBD/<action>` actions |
| `zelos_extension_obd/constants.py` | Interfaces, settings defaults and trace names |
| `zelos_extension_obd/scan.py` | OBD-II services, J1979 and J1979-2 |
| `zelos_extension_obd/vehicle.py`, `link.py` | One vehicle and its CAN or DoIP link |
| `zelos_extension_obd/trace.py` | Trace events |
| `zelos_extension_obd/cli/` | App mode, and the shell subcommands that run the actions |
| `zelos_extension_obd/demo/` | The demo vehicle |
| `tests/doip_entity.py` | DoIP entity simulator the tests and `just sim-doip` run |

## Commands

| Command                | Description                                        |
| ---------------------- | -------------------------------------------------- |
| `just install`         | Install dependencies and the pre-commit hooks      |
| `just format`          | Format code with ruff                              |
| `just check`           | Check formatting and lint with ruff                |
| `just test`            | Run tests with pytest                              |
| `just package`         | Package for the Zelos marketplace                  |
| `just release VERSION` | Bump version, check, test, commit, tag             |
| `just clean`           | Remove build output, caches and packages           |
| `just run`             | Run the agent runtime locally                      |
| `just sim`             | Run the demo vehicle on vcan0 (Linux)              |
| `just sim-doip`        | Run the DoIP entity simulator on 127.0.0.1:13400   |

`just --list` shows every recipe. `just install` installs the pre-commit hooks in
`.pre-commit-config.yaml`.

## Run it in Zelos

```bash
zelos extensions install-local .
```

A local install runs from your project directory, so you do not install again after a code
change:

- Agent runtime: stop and start the extension. After a dependency change, stop it and run
  `zelos extensions reinstall local.obd`.
- `extension.toml`, or an action that runs while the extension is stopped: run
  `zelos extensions install-local .` again.

## Packaging

```bash
just package
```

The archive lands next to `extension.toml` as `<name>-<version>.tar.gz`, where `<name>` is the
manifest name in lowercase with every other run of characters turned into one hyphen. It holds
`extension.toml`, the README, the icon, the config schema and the paths listed in
`[package] paths`. With an agent runtime it also holds `pyproject.toml`, `uv.lock` and
`actions.json`, the list of actions that run while the extension is stopped.

## Release

```bash
just release 1.0.0
git push --follow-tags
```

The release workflow checks, tests and packages the extension, then attaches the archive to a
GitHub release.
