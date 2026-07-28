#!/usr/bin/env python3
"""Static consistency checks for the Demo 0 overlay.

This deliberately avoids ROS imports so it can run before the workspace is built.
It does not replace a real Gazebo/Nav2 runtime test.
"""

from __future__ import annotations

import ast
import math
import sys
import wave
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "socialbot_demo0"


def fail(message: str) -> None:
    raise RuntimeError(message)


def validate_python() -> int:
    files = sorted(PACKAGE.rglob("*.py")) + [ROOT / "demo0" / "static_validate.py"]
    for path in files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return len(files)


def validate_yaml() -> int:
    files = sorted(PACKAGE.rglob("*.yaml")) + [
        ROOT / "docker" / "compose.yaml",
        ROOT / "docker" / "compose.laptop.yaml",
    ]
    for path in files:
        with path.open("r", encoding="utf-8") as stream:
            yaml.safe_load(stream)
    return len(files)


def validate_xml() -> int:
    files = [PACKAGE / "package.xml", PACKAGE / "worlds" / "demo0_world.sdf"]
    for path in files:
        ET.parse(path)
    return len(files)


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            fail(f"Unsupported PGM format in {path}")

        tokens: list[bytes] = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                fail(f"Unexpected end of PGM header in {path}")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())

        width, height, max_value = map(int, tokens[:3])
        if max_value != 255:
            fail(f"Expected 8-bit PGM in {path}, got max value {max_value}")

        pixels = stream.read(width * height)
        if len(pixels) != width * height:
            fail(f"Wrong pixel count in {path}")
        return width, height, pixels


def world_to_grid(
    x: float,
    y: float,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> tuple[int, int]:
    col = int(math.floor((x - origin_x) / resolution))
    row_from_bottom = int(math.floor((y - origin_y) / resolution))
    row = height - 1 - row_from_bottom
    if not (0 <= col < width and 0 <= row < height):
        fail(f"World point ({x}, {y}) lies outside the map")
    return row, col


def inflate_occupied(
    occupied: list[list[bool]],
    radius_cells: int,
) -> list[list[bool]]:
    height = len(occupied)
    width = len(occupied[0])
    inflated = [row[:] for row in occupied]
    offsets = [
        (dr, dc)
        for dr in range(-radius_cells, radius_cells + 1)
        for dc in range(-radius_cells, radius_cells + 1)
        if dr * dr + dc * dc <= radius_cells * radius_cells
    ]

    for row in range(height):
        for col in range(width):
            if not occupied[row][col]:
                continue
            for dr, dc in offsets:
                nr, nc = row + dr, col + dc
                if 0 <= nr < height and 0 <= nc < width:
                    inflated[nr][nc] = True
    return inflated


def path_exists(
    occupied: list[list[bool]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> bool:
    if occupied[start[0]][start[1]] or occupied[goal[0]][goal[1]]:
        return False

    height = len(occupied)
    width = len(occupied[0])
    queue = deque([start])
    visited = {start}
    neighbours = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while queue:
        row, col = queue.popleft()
        if (row, col) == goal:
            return True
        for dr, dc in neighbours:
            nr, nc = row + dr, col + dc
            candidate = (nr, nc)
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if occupied[nr][nc] or candidate in visited:
                continue
            visited.add(candidate)
            queue.append(candidate)
    return False


def validate_map_and_mission() -> None:
    map_yaml_path = PACKAGE / "maps" / "demo0_map.yaml"
    mission_path = PACKAGE / "config" / "interview_script.yaml"

    map_data = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    mission = yaml.safe_load(mission_path.read_text(encoding="utf-8"))

    image_path = map_yaml_path.parent / str(map_data["image"])
    width, height, pixels = read_pgm(image_path)
    resolution = float(map_data["resolution"])
    origin_x, origin_y = map(float, map_data["origin"][:2])

    # Values below 65 are occupied in the generated binary map.
    occupied = [
        [pixels[row * width + col] < 65 for col in range(width)]
        for row in range(height)
    ]

    initial = mission["initial_pose"]
    home = mission["targets"]["home"]
    interview = mission["targets"]["interview_spot"]

    if abs(float(initial["x"]) - float(home["x"])) > 1e-9 or abs(
        float(initial["y"]) - float(home["y"])
    ) > 1e-9:
        fail("Mission initial_pose and home target are inconsistent")

    start = world_to_grid(
        float(home["x"]),
        float(home["y"]),
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
        width=width,
        height=height,
    )
    goal = world_to_grid(
        float(interview["x"]),
        float(interview["y"]),
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
        width=width,
        height=height,
    )

    # Conservatively reserve the robot radius from the supplied Nav2 config.
    nav2 = yaml.safe_load((PACKAGE / "config" / "nav2_params.yaml").read_text(encoding="utf-8"))
    robot_radius = float(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"]
    )
    radius_cells = int(math.ceil(robot_radius / resolution))
    inflated = inflate_occupied(occupied, radius_cells)

    if not path_exists(inflated, start, goal):
        fail("No collision-free grid path exists between home and interview_spot")


def validate_audio() -> int:
    files = sorted((PACKAGE / "assets" / "audio").glob("*.wav"))
    if not files:
        fail("No WAV assets found")

    for path in files:
        with wave.open(str(path), "rb") as audio:
            if audio.getnchannels() not in (1, 2):
                fail(f"Unsupported channel count in {path}")
            if audio.getsampwidth() != 2:
                fail(f"Expected 16-bit PCM in {path}")
            if audio.getnframes() <= 0:
                fail(f"Empty WAV file: {path}")
    return len(files)


def validate_required_files() -> None:
    required = [
        ROOT / "Makefile",
        ROOT / "docker" / "DockerFile",
        ROOT / "docker" / "compose.yaml",
        ROOT / "docker" / "compose.laptop.yaml",
        ROOT / "docker" / ".config" / "entrypoint.sh",
        PACKAGE / "launch" / "simulation.launch.py",
        PACKAGE / "launch" / "mapping.launch.py",
        PACKAGE / "launch" / "navigation.launch.py",
        PACKAGE / "launch" / "demo0.launch.py",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        fail("Missing required files: " + ", ".join(missing))


def main() -> int:
    validate_required_files()
    python_count = validate_python()
    yaml_count = validate_yaml()
    xml_count = validate_xml()
    validate_map_and_mission()
    audio_count = validate_audio()

    print("Demo 0 static validation passed")
    print(f"  Python files parsed: {python_count}")
    print(f"  YAML files parsed:   {yaml_count}")
    print(f"  XML/SDF files parsed:{xml_count:>4}")
    print(f"  WAV files checked:   {audio_count}")
    print("  Home -> interview map path: reachable")
    print("Note: Gazebo, ROS 2 graph, and Nav2 runtime still require a Docker execution test.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 - validation should report all fatal errors uniformly.
        print(f"Demo 0 static validation failed: {exc}", file=sys.stderr)
        sys.exit(1)
