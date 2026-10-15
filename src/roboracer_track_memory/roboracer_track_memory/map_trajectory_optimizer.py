#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


# ============================================================
# MAP
# ============================================================

def load_map(yaml_path):

    yaml_path = Path(
        yaml_path
    ).expanduser().resolve()

    with open(yaml_path, 'r') as f:
        info = yaml.safe_load(f)

    image_path = Path(
        info['image']
    )

    if not image_path.is_absolute():
        image_path = (
            yaml_path.parent /
            image_path
        )

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise RuntimeError(
            f'Cannot load map: {image_path}'
        )

    resolution = float(
        info['resolution']
    )

    origin = info['origin']

    return (
        image,
        resolution,
        origin
    )


def build_clearance_map(
    image,
    resolution
):

    # Only confidently observed free space
    # is considered drivable.
    free = image >= 250

    count, labels, stats, _ = \
        cv2.connectedComponentsWithStats(
            free.astype(np.uint8),
            connectivity=8
        )

    if count <= 1:
        raise RuntimeError(
            'No connected free-space region'
        )

    largest = (
        1 +
        int(
            np.argmax(
                stats[
                    1:,
                    cv2.CC_STAT_AREA
                ]
            )
        )
    )

    drivable = (
        labels == largest
    )

    distance_px = cv2.distanceTransform(
        drivable.astype(np.uint8),
        cv2.DIST_L2,
        5
    )

    clearance = (
        distance_px *
        resolution
    )

    return (
        drivable,
        clearance
    )


# ============================================================
# COORDINATE CONVERSION
# ============================================================

def map_to_pixel(
    x,
    y,
    origin,
    resolution,
    height
):

    px = (
        x - origin[0]
    ) / resolution - 0.5

    py = (
        height -
        (
            (y - origin[1]) /
            resolution
        ) -
        0.5
    )

    return (
        int(round(px)),
        int(round(py))
    )


def pixel_to_map(
    px,
    py,
    origin,
    resolution,
    height
):

    x = (
        origin[0] +
        (px + 0.5) *
        resolution
    )

    y = (
        origin[1] +
        (
            height -
            py -
            0.5
        ) *
        resolution
    )

    return x, y


# ============================================================
# TRAJECTORY
# ============================================================

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

    if np.linalg.norm(
        points[0] -
        points[-1]
    ) < 0.10:

        points = points[:-1]

    return points


# ============================================================
# CLEARANCE OPTIMIZATION
# ============================================================

def optimize_clearance(
    points,
    clearance,
    drivable,
    origin,
    resolution,
    search_radius_m=0.40
):

    height, width = clearance.shape

    radius_px = max(
        1,
        int(
            search_radius_m /
            resolution
        )
    )

    optimized = []

    original_clearances = []
    optimized_clearances = []

    rejected = 0

    for x, y in points:

        px, py = map_to_pixel(
            x,
            y,
            origin,
            resolution,
            height
        )

        # ----------------------------------------
        # Point outside map
        # ----------------------------------------

        if (
            px < 0 or
            px >= width or
            py < 0 or
            py >= height
        ):

            optimized.append([
                x,
                y
            ])

            original_clearances.append(
                0.0
            )

            optimized_clearances.append(
                0.0
            )

            rejected += 1
            continue

        original_clearance = float(
            clearance[py, px]
        )

        # ----------------------------------------
        # Search nearby region for better
        # clearance.
        # ----------------------------------------

        x0 = max(
            0,
            px - radius_px
        )

        x1 = min(
            width,
            px + radius_px + 1
        )

        y0 = max(
            0,
            py - radius_px
        )

        y1 = min(
            height,
            py + radius_px + 1
        )

        local_clearance = (
            clearance[
                y0:y1,
                x0:x1
            ].copy()
        )

        local_drivable = (
            drivable[
                y0:y1,
                x0:x1
            ]
        )

        # Don't allow optimizer to jump
        # arbitrarily across the map.
        yy, xx = np.indices(
            local_clearance.shape
        )

        global_x = xx + x0
        global_y = yy + y0

        distance_from_original = (
            np.sqrt(
                (global_x - px) ** 2 +
                (global_y - py) ** 2
            ) *
            resolution
        )

        valid = (
            local_drivable &
            (
                distance_from_original
                <= search_radius_m
            )
        )

        if not np.any(valid):

            optimized.append([
                x,
                y
            ])

            original_clearances.append(
                original_clearance
            )

            optimized_clearances.append(
                original_clearance
            )

            rejected += 1
            continue

        # Score:
        #
        # prefer high clearance,
        # but penalize moving too far away
        # from the actually driven path.
        score = (
            local_clearance -
            0.35 *
            distance_from_original
        )

        score[~valid] = -1e9

        best_index = np.argmax(
            score
        )

        best_y, best_x = \
            np.unravel_index(
                best_index,
                score.shape
            )

        best_px = (
            best_x + x0
        )

        best_py = (
            best_y + y0
        )

        new_x, new_y = \
            pixel_to_map(
                best_px,
                best_py,
                origin,
                resolution,
                height
            )

        optimized.append([
            new_x,
            new_y
        ])

        original_clearances.append(
            original_clearance
        )

        optimized_clearances.append(
            float(
                clearance[
                    best_py,
                    best_px
                ]
            )
        )

    return (
        np.asarray(optimized),
        np.asarray(original_clearances),
        np.asarray(optimized_clearances),
        rejected
    )


# ============================================================
# SMOOTHING
# ============================================================

def smooth_closed(
    points,
    iterations=5
):

    result = points.copy()

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
            0.20 * previous +
            0.60 * result +
            0.20 * following
        )

    return result


# ============================================================
# GEOMETRY
# ============================================================

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

    dx = (
        following[:, 0] -
        previous[:, 0]
    )

    dy = (
        following[:, 1] -
        previous[:, 1]
    )

    yaw = np.arctan2(
        dy,
        dx
    )

    ds = (
        np.linalg.norm(
            following -
            previous,
            axis=1
        ) /
        2.0
    )

    ds = np.maximum(
        ds,
        0.001
    )

    yaw_previous = np.roll(
        yaw,
        1
    )

    yaw_following = np.roll(
        yaw,
        -1
    )

    dyaw = (
        yaw_following -
        yaw_previous
    )

    dyaw = (
        dyaw +
        math.pi
    ) % (
        2.0 *
        math.pi
    ) - math.pi

    curvature = (
        dyaw /
        (2.0 * ds)
    )

    return (
        yaw,
        curvature
    )


# ============================================================
# SPEED PROFILE
# ============================================================

def speed_profile(
    curvature,
    clearance_values
):

    minimum_speed = 0.40
    maximum_speed = 1.20

    curvature_gain = 2.0

    speed = (
        maximum_speed /
        (
            1.0 +
            curvature_gain *
            np.abs(curvature)
        )
    )

    # Also reduce speed when clearance
    # becomes small.
    clearance_factor = np.clip(
        clearance_values /
        0.60,
        0.35,
        1.0
    )

    speed *= clearance_factor

    speed = np.clip(
        speed,
        minimum_speed,
        maximum_speed
    )

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


# ============================================================
# SAVE
# ============================================================

def save_csv(
    output,
    points,
    yaw,
    curvature,
    clearance_values,
    speed
):

    with open(
        output,
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
            'clearance',
            'target_speed'
        ])

        for i in range(
            len(points)
        ):

            writer.writerow([
                i,
                f'{points[i,0]:.6f}',
                f'{points[i,1]:.6f}',
                f'{yaw[i]:.6f}',
                f'{curvature[i]:.6f}',
                f'{clearance_values[i]:.6f}',
                f'{speed[i]:.6f}'
            ])


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        'trajectory'
    )

    parser.add_argument(
        'map_yaml'
    )

    parser.add_argument(
        '--output',
        default=None
    )

    args = parser.parse_args()

    trajectory_path = Path(
        args.trajectory
    ).expanduser().resolve()

    map_path = Path(
        args.map_yaml
    ).expanduser().resolve()

    if args.output:

        output = Path(
            args.output
        ).expanduser().resolve()

    else:

        output = (
            trajectory_path.parent /
            'learned_track_map.csv'
        )

    image, resolution, origin = \
        load_map(
            map_path
        )

    drivable, clearance = \
        build_clearance_map(
            image,
            resolution
        )

    points = load_trajectory(
        trajectory_path
    )

    (
        optimized,
        old_clearance,
        new_clearance,
        rejected
    ) = optimize_clearance(
        points,
        clearance,
        drivable,
        origin,
        resolution
    )

    optimized = smooth_closed(
        optimized
    )

    # Recalculate actual clearance AFTER
    # smoothing.
    final_clearance = []

    h, w = clearance.shape

    for x, y in optimized:

        px, py = map_to_pixel(
            x,
            y,
            origin,
            resolution,
            h
        )

        if (
            0 <= px < w and
            0 <= py < h
        ):

            final_clearance.append(
                float(
                    clearance[
                        py,
                        px
                    ]
                )
            )

        else:

            final_clearance.append(
                0.0
            )

    final_clearance = np.asarray(
        final_clearance
    )

    yaw, curvature = \
        calculate_geometry(
            optimized
        )

    speed = speed_profile(
        curvature,
        final_clearance
    )

    save_csv(
        output,
        optimized,
        yaw,
        curvature,
        final_clearance,
        speed
    )

    print()
    print(
        '===== MAP TRAJECTORY OPTIMIZER ====='
    )

    print(
        f'Input points:       '
        f'{len(points)}'
    )

    print(
        f'Map resolution:     '
        f'{resolution:.3f} m/pixel'
    )

    print(
        f'Original clearance: '
        f'{np.mean(old_clearance):.3f} m mean'
    )

    print(
        f'Optimized clearance:'
        f' {np.mean(final_clearance):.3f} m mean'
    )

    print(
        f'Min final clearance:'
        f' {np.min(final_clearance):.3f} m'
    )

    print(
        f'Max curvature:      '
        f'{np.max(np.abs(curvature)):.3f} 1/m'
    )

    print(
        f'Speed range:        '
        f'{np.min(speed):.2f} - '
        f'{np.max(speed):.2f} m/s'
    )

    print(
        f'Points outside map: '
        f'{rejected}'
    )

    print()
    print(
        f'Saved: {output}'
    )


if __name__ == '__main__':
    main()
