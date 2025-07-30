#!/usr/bin/env python3
"""Scale an SDF/Gazebo world file in X and Y directions.

This utility multiplies the first two numerical components (x, y) of:
  * <size> a b c
  * <pose> x y z roll pitch yaw
by the given scale factor (default 2.0).

Usage:
    python3 scale_world.py path/to/world_file.world [--scale 2.0] [--output scaled.world]

If --output is omitted, a new file named <original>_scaled.world will be created
in the same directory.
"""
import argparse
import pathlib
import re
from typing import List

SIZE_PATTERN = re.compile(r"(<size>)([^<]+)(</size>)")
POSE_PATTERN = re.compile(r"(<pose>)([^<]+)(</pose>)")


def _scale_first_two(values: List[str], factor: float) -> List[str]:
    """Multiply the first two elements in a list of strings by *factor* (in-place)."""
    scaled = []
    for idx, val in enumerate(values):
        if idx < 2:
            try:
                scaled_val = float(val) * factor
                # Preserve integer formatting if possible
                if scaled_val.is_integer():
                    scaled.append(str(int(scaled_val)))
                else:
                    scaled.append(str(scaled_val))
            except ValueError:
                # Non-numeric (should not happen), keep original
                scaled.append(val)
        else:
            scaled.append(val)
    return scaled


def _scale_tag(content: str, pattern: re.Pattern, factor: float) -> str:
    """Scale values inside XML tags matched by *pattern*."""

    def replacer(match: re.Match) -> str:
        opening, numbers, closing = match.groups()
        parts = numbers.strip().split()
        if len(parts) >= 2:
            parts = _scale_first_two(parts, factor)
        return f"{opening}{' '.join(parts)}{closing}"

    return pattern.sub(replacer, content)


def scale_world_file(src: pathlib.Path, dst: pathlib.Path, factor: float) -> None:
    text = src.read_text()
    text = _scale_tag(text, SIZE_PATTERN, factor)
    text = _scale_tag(text, POSE_PATTERN, factor)
    dst.write_text(text)


def main():
    parser = argparse.ArgumentParser(description="Scale Gazebo world in X and Y directions.")
    parser.add_argument("world_file", type=pathlib.Path, help="Path to .world file to scale")
    parser.add_argument("--scale", type=float, default=2.0, help="Scale factor (default: 2.0)")
    parser.add_argument("--output", type=pathlib.Path, help="Output file path")
    args = parser.parse_args()

    src = args.world_file.resolve()
    if not src.exists():
        parser.error(f"Input file {src} does not exist")

    dst = args.output.resolve() if args.output else src.with_name(src.stem + f"_scaled.world")
    scale_world_file(src, dst, args.scale)
    print(f"Scaled world saved to {dst}")


if __name__ == "__main__":
    main()
