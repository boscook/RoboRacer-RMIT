#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def load_trajectory(path):

    points = []

    with open(path, 'r') as f:

        reader = csv.DictReader(f)

        for row in reader:

            points.append([
                float(row['x']),
                float(row['y'])
            ])

    points = np.asarray(
        points,
        dtype=np.float64
    )

    if len(points) < 10:
        raise RuntimeError(
            'Not enough trajectory points'
        )

    # Remove duplicate closing point.
    if np.linalg.norm(
        points[0] - points[-1]
    ) < 0.10:

        points = points[:-1]

    return points


def resample_closed_path(
    points,
    spacing=0.10
):

    # Close path temporarily.
    closed = np.vstack([
        points,
        points[0]
    ])

    segment_length = np.linalg.norm(
        np.diff(closed, axis=0),
        axis=1
    )

    cumulative = np.concatenate([
        [0.0],
        np.cumsum(segment_length)
    ])

    total_length = cumulative[-1]

    count = max(
        10,
        int(
            round(
                total_length / spacing
            )
        )
    )

    samples = np.linspace(
        0.0,
        total_length,
        count,
        endpoint=False
    )

    x = np.interp(
        samples,
        cumulative,
        closed[:, 0]
    )

    y = np.interp(
        samples,
        cumulative,
        closed[:, 1]
    )

    return (
        np.column_stack([x, y]),
        total_length
    )


def smooth_closed_path(
    points,
    iterations=4
):

    result = points.copy()

    # Circular smoothing keeps the track closed.
    for _ in range(iterations):

        previous = np.roll(
            result,
            1,
            axis=0
        )

        following = np.roll(
            result,
            -1,
            axis=0
        )

        result = (
            0.25 * previous +
            0.50 * result +
            0.25 * following
        )

    return result


def calculate_geometry(points):

    previous = np.roll(
        points,
        1,
        axis=0
    )

    following = np.roll(
        points,
        -1,
        axis=0
    )

    dx = following[:, 0] - previous[:, 0]
    dy = following[:, 1] - previous[:, 1]

    yaw = np.arctan2(
        dy,
        dx
    )

    # Unwrap heading before differentiating.
    yaw_unwrapped = np.unwrap(yaw)

    ds = 0.5 * (
        np.linalg.norm(
            points - previous,
            axis=1
        ) +
        np.linalg.norm(
            following - points,
            axis=1
        )
    )

    ds = np.maximum(
        ds,
        1e-4
    )

    yaw_previous = np.roll(
        yaw_unwrapped,
        1
    )

    yaw_following = np.roll(
        yaw_unwrapped,
        -1
    )

    dyaw = (
        yaw_following -
        yaw_previous
    )

    # Correct wrap at closed-track boundary.
    dyaw = (
        dyaw + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi

    curvature = (
        dyaw /
        (2.0 * ds)
    )

    return (
        yaw,
        curvature
    )


def calculate_speed_profile(
    curvature
):

    # Conservative initial learning values.
    min_speed = 0.50
    max_speed = 1.20

    # Higher curvature -> lower speed.
    curvature_gain = 2.0

    speed = (
        max_speed /
        (
            1.0 +
            curvature_gain *
            np.abs(curvature)
        )
    )

    speed = np.clip(
        speed,
        min_speed,
        max_speed
    )

    # Smooth speed around the closed circuit.
    for _ in range(3):

        speed = (
            0.25 *
            np.roll(speed, 1) +
            0.50 *
            speed +
            0.25 *
            np.roll(speed, -1)
        )

    return speed


def save_track(
    path,
    points,
    yaw,
    curvature,
    speed
):

    with open(
        path,
        'w',
        newline=''
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            'index',
            'x',
            'y',
            'yaw',
            'curvature',
            'target_speed'
        ])

        for i in range(len(points)):

            writer.writerow([
                i,
                f'{points[i,0]:.6f}',
                f'{points[i,1]:.6f}',
                f'{yaw[i]:.6f}',
                f'{curvature[i]:.6f}',
                f'{speed[i]:.6f}'
            ])


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        'trajectory'
    )

    parser.add_argument(
        '--output',
        default=None
    )

    args = parser.parse_args()

    input_path = Path(
        args.trajectory
    ).expanduser().resolve()

    if args.output:

        output_path = Path(
            args.output
        ).expanduser().resolve()

    else:

        output_path = (
            input_path.parent /
            'learned_track.csv'
        )

    points = load_trajectory(
        input_path
    )

    points, length = \
        resample_closed_path(
            points,
            spacing=0.10
        )

    points = smooth_closed_path(
        points,
        iterations=4
    )

    yaw, curvature = \
        calculate_geometry(
            points
        )

    speed = calculate_speed_profile(
        curvature
    )

    save_track(
        output_path,
        points,
        yaw,
        curvature,
        speed
    )

    print()
    print('===== TRACK LEARNING =====')
    print(
        f'Input:          {input_path}'
    )
    print(
        f'Track length:   {length:.2f} m'
    )
    print(
        f'Waypoints:      {len(points)}'
    )
    print(
        f'Max curvature:  '
        f'{np.max(np.abs(curvature)):.3f} 1/m'
    )
    print(
        f'Speed range:    '
        f'{np.min(speed):.2f} - '
        f'{np.max(speed):.2f} m/s'
    )
    print()
    print(
        f'Learned track:  {output_path}'
    )


if __name__ == '__main__':
    main()
