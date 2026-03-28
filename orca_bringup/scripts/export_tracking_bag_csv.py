#!/usr/bin/env python3
# MIT License — same as orca_bringup
r"""
Export PurePursuit tracking topics from a rosbag2 folder to plain CSV (no ROS needed to analyze CSVs).

Rosbag2 layout (not Markdown): a *bag directory* contains
  - metadata.yaml  — version, duration, topic list, storage backend id
  - data files     — usually ``*.db3`` (SQLite3 plugin) or ``*.mcap`` (MCAP plugin)

This script reads the bag once (needs ``source install/setup.bash`` for rosbag2_py), then writes:

  tracking_errors_long.csv
      Columns: time_sec, topic, data
      One row per message; easy to filter/pivot in pandas/R.

  tracking_errors_wide.csv
      Columns: time_sec, cross_track_xy_m, vertical_error_m, yaw_error_rad
      Rows aligned to cross-track timestamps; vertical/yaw use last sample at or before that time
      (same controller tick → usually identical timestamps).

  tracking_export_README.txt
      Short column reference for copying CSVs into another repo.

Usage:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run orca_bringup export_tracking_bag_csv.py /path/to/rosbag2_folder
  ros2 run orca_bringup export_tracking_bag_csv.py /path/to/bag -o ~/exports/run42

Optional:
  --plot   show matplotlib figure (requires matplotlib)
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import DefaultDict, List, Tuple

TOPIC_CROSS = '/pure_pursuit_cross_track_xy'
TOPIC_VERT = '/pure_pursuit_vertical_error'
TOPIC_YAW = '/pure_pursuit_yaw_error'
TOPICS = (TOPIC_CROSS, TOPIC_VERT, TOPIC_YAW)


def _storage_id_from_metadata(bag_dir: Path) -> str:
    meta = bag_dir / 'metadata.yaml'
    if not meta.is_file():
        return 'sqlite3'
    text = meta.read_text(encoding='utf-8', errors='replace')
    m = re.search(r'storage_identifier:\s*(\S+)', text)
    return m.group(1) if m else 'sqlite3'


def _read_bag_rows(bag_dir: Path) -> List[Tuple[float, str, float]]:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64

    want = set(TOPICS)
    sid = _storage_id_from_metadata(bag_dir)
    storage_options = StorageOptions(uri=str(bag_dir), storage_id=sid)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    rows: List[Tuple[float, str, float]] = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in want:
            continue
        msg = deserialize_message(data, Float64)
        rows.append((t_ns * 1e-9, topic, msg.data))
    rows.sort(key=lambda r: r[0])
    return rows


def _split_by_topic(
    rows: List[Tuple[float, str, float]],
) -> DefaultDict[str, List[Tuple[float, float]]]:
    out: DefaultDict[str, List[Tuple[float, float]]] = defaultdict(list)
    for t_sec, topic, val in rows:
        out[topic].append((t_sec, val))
    return out


def _asof_backward(
    master: List[Tuple[float, float]],
    slave: List[Tuple[float, float]],
) -> List[float]:
    """For each master time t, use last slave value with slave_t <= t (else NaN)."""
    if not master:
        return []
    slave = sorted(slave, key=lambda x: x[0])
    j = 0
    last = float('nan')
    out: List[float] = []
    for t, _ in master:
        while j < len(slave) and slave[j][0] <= t:
            last = slave[j][1]
            j += 1
        out.append(last)
    return out


def _write_long_csv(path: Path, rows: List[Tuple[float, str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['time_sec', 'topic', 'data'])
        w.writerows(rows)


def _write_wide_csv(path: Path, by_topic: DefaultDict[str, List[Tuple[float, float]]]) -> None:
    master = sorted(by_topic.get(TOPIC_CROSS, []), key=lambda x: x[0])
    vert = sorted(by_topic.get(TOPIC_VERT, []), key=lambda x: x[0])
    yaw = sorted(by_topic.get(TOPIC_YAW, []), key=lambda x: x[0])

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['time_sec', 'cross_track_xy_m', 'vertical_error_m', 'yaw_error_rad'])

        if master:
            vcol = _asof_backward(master, vert)
            ycol = _asof_backward(master, yaw)
            for (t, cx), v, y in zip(master, vcol, ycol):
                w.writerow([
                    t,
                    cx,
                    v if not math.isnan(v) else '',
                    y if not math.isnan(y) else '',
                ])
            return

        # No cross-track topic: one row per union timestamp (unusual)
        all_t = sorted({t for s in (vert, yaw) for t, _ in s})
        pseudo = [(t, 0.0) for t in all_t]
        vcol = _asof_backward(pseudo, vert)
        ycol = _asof_backward(pseudo, yaw)
        for t, v, y in zip(all_t, vcol, ycol):
            w.writerow([
                t,
                '',
                v if not math.isnan(v) else '',
                y if not math.isnan(y) else '',
            ])


def _write_readme(path: Path, bag_dir: Path) -> None:
    text = f"""Tracking error CSV export
============================

Source bag directory:
  {bag_dir}

Files in this folder:
  tracking_errors_long.csv
    time_sec   — recorder timestamp when the message was logged (seconds, float)
    topic      — ROS topic name
    data       — std_msgs/Float64 .data field

  tracking_errors_wide.csv
    time_sec             — same as cross-track samples (master clock for alignment)
    cross_track_xy_m     — horizontal distance to path polyline (m)
    vertical_error_m     — robot_z - path_z at closest XY point (m)
    yaw_error_rad        — shortest angle path_heading → robot_yaw (rad), range about [-pi, pi]

These CSV files are plain UTF-8 text. You can copy them to any machine or repo and load with
pandas, R, Julia, Excel, etc. No ROS installation required on the analysis side.

Rosbag2 originals are not CSV; they are metadata.yaml plus a storage file (.db3 / .mcap).
This export was produced by orca_bringup/scripts/export_tracking_bag_csv.py
"""
    path.write_text(text, encoding='utf-8')


def _maybe_plot(by_topic: DefaultDict[str, List[Tuple[float, float]]], title: str) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed; skipping --plot.', file=sys.stderr)
        return

    t0 = min((by_topic[t][0][0] for t in TOPICS if by_topic.get(t)), default=0.0)
    labels = (
        'cross_track_xy (m)',
        'vertical_error (m)',
        'yaw_error (rad)',
    )
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
    for ax, topic, ylab in zip(axes, TOPICS, labels):
        series = by_topic.get(topic, [])
        if series:
            xs = [x - t0 for x, _ in series]
            ys = [y for _, y in series]
            ax.plot(xs, ys, '.', markersize=2)
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('time (s) from first sample')
    fig.suptitle(title)
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Export tracking-error rosbag topics to CSV for offline analysis.',
    )
    parser.add_argument('bag', type=Path, help='rosbag2 directory (contains metadata.yaml)')
    parser.add_argument(
        '-o',
        '--output-dir',
        type=Path,
        default=None,
        help='Directory for CSV + README (default: inside the bag folder)',
    )
    parser.add_argument('--plot', action='store_true', help='Plot with matplotlib')
    args = parser.parse_args()

    bag_dir = args.bag.resolve()
    if not bag_dir.is_dir():
        print(f'Not a directory: {bag_dir}', file=sys.stderr)
        return 1

    try:
        rows = _read_bag_rows(bag_dir)
    except Exception as e:
        print(f'Failed to read bag: {e}', file=sys.stderr)
        print('Source ROS 2 setup (rosbag2_py, rclpy).', file=sys.stderr)
        return 1

    out_dir = (args.output_dir or (bag_dir / 'csv_export')).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    long_path = out_dir / 'tracking_errors_long.csv'
    wide_path = out_dir / 'tracking_errors_wide.csv'
    readme_path = out_dir / 'tracking_export_README.txt'

    _write_long_csv(long_path, rows)
    by_topic = _split_by_topic(rows)
    _write_wide_csv(wide_path, by_topic)
    _write_readme(readme_path, bag_dir)

    print(f'Wrote {len(rows)} samples → {long_path}')
    print(f'Wide CSV → {wide_path}')
    print(f'Notes → {readme_path}')
    print(f'Copy ``{out_dir}`` into another repo for analysis (CSVs only).')

    if args.plot:
        _maybe_plot(by_topic, bag_dir.name)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
