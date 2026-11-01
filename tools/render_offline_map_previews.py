#!/usr/bin/env python3
"""Render deterministic 2D occupancy and 3D PCD top-down map previews."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError(f"only binary P5 PGM is supported: {path}")
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = (int(value) for value in line.split())
        maximum = int(stream.readline())
        if maximum != 255:
            raise ValueError(f"unsupported PGM maximum value {maximum}: {path}")
        image = np.frombuffer(stream.read(width * height), dtype=np.uint8)
    if image.size != width * height:
        raise ValueError(f"truncated PGM: {path}")
    return image.reshape(height, width)


def read_binary_xyzi_pcd(path: Path) -> np.ndarray:
    payload = path.read_bytes()
    marker = b"DATA binary\n"
    offset = payload.find(marker)
    if offset < 0:
        raise ValueError(f"only binary XYZI PCD is supported: {path}")
    header = payload[: offset + len(marker)].decode("ascii")
    if "FIELDS x y z intensity" not in header or "SIZE 4 4 4 4" not in header:
        raise ValueError(f"unexpected PCD layout: {path}")
    points_line = next(line for line in header.splitlines() if line.startswith("POINTS "))
    count = int(points_line.split()[1])
    points = np.frombuffer(payload[offset + len(marker):], dtype="<f4")
    if points.size != count * 4:
        raise ValueError(f"truncated PCD: {path}")
    return points.reshape(count, 4)


def render_flat(pgm: Path, output: Path, resolution: float) -> None:
    image = read_pgm(pgm)
    height, width = image.shape
    figure, axis = plt.subplots(figsize=(10, 7), constrained_layout=True)
    axis.imshow(
        image,
        cmap="gray",
        vmin=0,
        vmax=255,
        origin="upper",
        interpolation="nearest",
        extent=(0, width * resolution, 0, height * resolution),
    )
    axis.set_title(f"Flat 2D occupancy map — {width}×{height}, {resolution:.2f} m/cell")
    axis.set_xlabel("map width [m]")
    axis.set_ylabel("map height [m]")
    axis.set_aspect("equal")
    figure.savefig(output, dpi=180)
    plt.close(figure)


def render_track(pcd: Path, output: Path) -> None:
    points = read_binary_xyzi_pcd(pcd)
    finite = np.isfinite(points[:, :3]).all(axis=1)
    points = points[finite]
    figure, axis = plt.subplots(figsize=(10, 7), constrained_layout=True)
    scatter = axis.scatter(
        points[:, 0], points[:, 1], c=points[:, 2], s=2.0,
        cmap="turbo", linewidths=0, rasterized=True,
    )
    figure.colorbar(scatter, ax=axis, label="height z [m]")
    axis.set_title(f"Track 3D map top-down preview — {len(points)} points")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.2)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--flat-pgm", type=Path, required=True)
    parser.add_argument("--track-pcd", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    args = parser.parse_args()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    flat_output = args.output_directory / "flat_2d_preview.png"
    track_output = args.output_directory / "track_3d_preview.png"
    render_flat(args.flat_pgm, flat_output, args.resolution)
    render_track(args.track_pcd, track_output)
    print(f"[MAP-PREVIEW][2D] {flat_output}")
    print(f"[MAP-PREVIEW][3D] {track_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
