"""Download the binary mesh assets required by the tracked MJCF files.

MuJoCo Menagerie is the upstream source for both the UR10e and Robotiq 2F-85.
The project keeps the integration XML in Git and excludes large meshes, so a
fresh clone should run this script once before launching the simulation.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = ROOT / "models" / "universal_robots_ur10e" / "assets"
BASE_URL = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main"

UR10E_ASSETS = [
    "base_0.obj",
    "base_1.obj",
    "shoulder_0.obj",
    "shoulder_1.obj",
    "shoulder_2.obj",
    "upperarm_0.obj",
    "upperarm_1.obj",
    "upperarm_2.obj",
    "upperarm_3.obj",
    "forearm_0.obj",
    "forearm_1.obj",
    "forearm_2.obj",
    "forearm_3.obj",
    "wrist1_0.obj",
    "wrist1_1.obj",
    "wrist1_2.obj",
    "wrist2_0.obj",
    "wrist2_1.obj",
    "wrist2_2.obj",
    "wrist3.obj",
]
ROBOTIQ_2F85_ASSETS = [
    "base_mount.stl",
    "base.stl",
    "driver.stl",
    "coupler.stl",
    "follower.stl",
    "pad.stl",
    "silicone_pad.stl",
    "spring_link.stl",
]


def source_url(filename: str) -> str:
    if filename in UR10E_ASSETS:
        return f"{BASE_URL}/universal_robots_ur10e/assets/{filename}"
    return f"{BASE_URL}/robotiq_2f85/assets/{filename}"


def fetch(filename: str, force: bool) -> None:
    destination = ASSET_DIR / filename
    if destination.exists() and not force:
        print(f"skip {filename}")
        return

    partial = destination.with_name(destination.name + ".part")
    print(f"download {filename}")
    try:
        with urlopen(source_url(filename), timeout=120) as response:
            with partial.open("wb") as output:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
        partial.replace(destination)
    finally:
        if partial.exists():
            partial.unlink()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--force", action="store_true", help="overwrite existing files")
    args = parser.parse_args()

    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    for filename in UR10E_ASSETS + ROBOTIQ_2F85_ASSETS:
        fetch(filename, force=args.force)
    print(f"assets ready: {ASSET_DIR}")


if __name__ == "__main__":
    main()
