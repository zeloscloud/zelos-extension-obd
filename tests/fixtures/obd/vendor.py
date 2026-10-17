"""Real-vehicle OBD-II traces and their independent oracle, vendored from zelos-can's
replay tests (api/py/zelos-can/test in the zelos monorepo).

`just sync-replay PATH` copies FILES from a monorepo checkout at PATH and records their
SHA-256 in MANIFEST. Edit them there, never here: the drift test fails on a copy that
differs from the manifest. Licences: LICENSES (MIT, attribution per file).
"""

import hashlib
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).parent
MANIFEST = HERE / "MANIFEST"
SOURCE = "api/py/zelos-can/test"
#: Traces with the tester's requests and the ECUs' answers logged (the client replays them).
TRACES = (
    "canhacker-gm-pids.log",
    "canhacker-gm-dtc.log",
    "canhacker-gm-t0.log",
    "canhacker-gm-olf.log",
    "canhacker-vin.log",
    "comma-ford-f-150-mk14.log",
    "comma-honda-accord-bus1.log",
    "comma-jeep-grand-cherokee-2019.log",
    "comma-mazda-cx5-2022.log",
    "comma-nissan-leaf.log",
    "comma-nissan-rogue.log",
    "comma-ram-1500-5th-gen.log",
    "comma-skoda-kodiaq-mk1-bus1.log",
    "comma-toyota-rav4-tss2.log",
    "comma-volkswagen-atlas-mk1-bus1.log",
    "css-api-examples-7512be4d-00000001.log",
)
FILES = (
    "obd_oracle.py",
    "obd_rules.py",
    "fixtures/obd/LICENSES",
    *(f"fixtures/obd/{t}" for t in TRACES),
)


def path(name: str) -> Path:
    return HERE / Path(name).name


def digest(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def manifest() -> dict[str, str]:
    lines = MANIFEST.read_text().splitlines()
    return {name: sha for sha, name in (line.split("  ", 1) for line in lines if line)}


def sync(source: Path) -> None:
    lines = []
    for name in FILES:
        shutil.copyfile(source / SOURCE / name, path(name))
        lines.append(f"{digest(path(name))}  {name}")
    MANIFEST.write_text("\n".join(lines) + "\n")
    print(f"vendored {len(FILES)} files from {source}")


if __name__ == "__main__":
    sync(Path(sys.argv[1]).expanduser())
