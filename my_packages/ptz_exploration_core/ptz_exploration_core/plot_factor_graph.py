#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

NodeKey = Tuple[str, int]
NodePosition = Tuple[float, float]
Edge = Tuple[NodeKey, NodeKey]

DEFAULT_DPI = 400
DEFAULT_FIGSIZE = (8.5, 8.5)
POSE_COLOR = "cyan"
LANDMARK_COLOR = "#0B3D91"
EDGE_COLOR = "#808080"
GRID_COLOR = "#B0B0B0"


def _load_snapshot(snapshot_path: Path) -> Mapping[str, object]:
    with snapshot_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _node_key(node_type: str, node_id: int) -> NodeKey:
    return (str(node_type), int(node_id))


def _node_from_entry(
    entry: Mapping[str, object], expected_type: str
) -> Tuple[NodeKey, NodePosition]:
    node_type = str(entry.get("type", expected_type))
    node_id = int(entry["id"])
    x = float(entry["x"])
    y = float(entry["y"])
    return _node_key(node_type, node_id), (x, y)


def _collect_nodes(snapshot: Mapping[str, object]) -> Dict[NodeKey, NodePosition]:
    nodes: Dict[NodeKey, NodePosition] = {}

    for pose in snapshot.get("poses", []):
        if not isinstance(pose, Mapping):
            continue
        key, position = _node_from_entry(pose, "pose")
        nodes[key] = position

    for landmark in snapshot.get("landmarks", []):
        if not isinstance(landmark, Mapping):
            continue
        key, position = _node_from_entry(landmark, "landmark")
        nodes[key] = position

    return nodes


def _collect_edges(snapshot: Mapping[str, object]) -> List[Edge]:
    edges: List[Edge] = []
    seen: set[frozenset[NodeKey]] = set()

    for entry in snapshot.get("edges", []):
        if not isinstance(entry, Mapping):
            continue

        from_entry = entry.get("from")
        to_entry = entry.get("to")
        if not isinstance(from_entry, Mapping) or not isinstance(to_entry, Mapping):
            continue

        try:
            edge = (
                _node_key(str(from_entry["type"]), int(from_entry["id"])),
                _node_key(str(to_entry["type"]), int(to_entry["id"])),
            )
        except (KeyError, TypeError, ValueError):
            continue

        dedupe_key = frozenset(edge)
        if dedupe_key in seen:
            continue
        seen.add(dedupe_key)
        edges.append(edge)

    return edges


def _extract_positions(
    nodes: Mapping[NodeKey, NodePosition], node_type: str
) -> List[Tuple[int, float, float]]:
    items: List[Tuple[int, float, float]] = []
    for (current_type, node_id), (x, y) in nodes.items():
        if current_type == node_type:
            items.append((node_id, x, y))
    items.sort(key=lambda item: item[0])
    return items


def _draw_edges(ax: plt.Axes, nodes: Mapping[NodeKey, NodePosition], edges: Iterable[Edge]) -> None:
    for from_key, to_key in edges:
        from_position = nodes.get(from_key)
        to_position = nodes.get(to_key)
        if from_position is None or to_position is None:
            continue

        ax.plot(
            [from_position[0], to_position[0]],
            [from_position[1], to_position[1]],
            color=EDGE_COLOR,
            linewidth=0.7,
            alpha=0.65,
            zorder=1,
        )


def _scatter_nodes(
    ax: plt.Axes,
    positions: List[Tuple[int, float, float]],
    *,
    color: str,
    size: float,
    label: str,
) -> None:
    if not positions:
        return

    xs = [item[1] for item in positions]
    ys = [item[2] for item in positions]
    ax.scatter(
        xs,
        ys,
        s=size,
        c=color,
        edgecolors="black",
        linewidths=0.4,
        marker="o",
        label=label,
        zorder=2,
    )


def plot_snapshot(snapshot_path: Path, output_path: Path, dpi: int = DEFAULT_DPI) -> None:
    snapshot = _load_snapshot(snapshot_path)
    nodes = _collect_nodes(snapshot)
    edges = _collect_edges(snapshot)

    pose_positions = _extract_positions(nodes, "pose")
    landmark_positions = _extract_positions(nodes, "landmark")

    if not pose_positions and not landmark_positions:
        raise ValueError(f"No pose or landmark nodes found in {snapshot_path}")

    fig, ax = plt.subplots(figsize=DEFAULT_FIGSIZE)

    _draw_edges(ax, nodes, edges)
    _scatter_nodes(ax, pose_positions, color=POSE_COLOR, size=28, label="Poses")
    _scatter_nodes(ax, landmark_positions, color=LANDMARK_COLOR, size=40, label="Landmarks")

    ax.set_title("Factor Graph")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, color=GRID_COLOR, alpha=0.22, linewidth=0.6)
    ax.set_facecolor("white")
    ax.legend(loc="best", frameon=True, framealpha=0.9)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot a 2D SLAM factor graph from optimizer JSON output."
    )
    parser.add_argument(
        "snapshot",
        type=Path,
        help="Path to the JSON snapshot written by the optimizer.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output image path (default: same path as snapshot with .png suffix).",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=DEFAULT_DPI,
        help=f"Output image DPI (default: {DEFAULT_DPI}).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    snapshot_path = args.snapshot.expanduser().resolve()
    output_path = (
        args.output.expanduser().resolve() if args.output else snapshot_path.with_suffix(".png")
    )
    plot_snapshot(snapshot_path, output_path, dpi=args.dpi)
    print(f"Saved factor graph plot to {output_path}")


if __name__ == "__main__":
    main()
