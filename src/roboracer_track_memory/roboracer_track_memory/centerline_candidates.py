#!/usr/bin/env python3

from pathlib import Path
import argparse

import cv2
import numpy as np
import yaml


def load_map(yaml_file):

    yaml_file = Path(yaml_file).expanduser().resolve()

    with open(yaml_file, 'r') as f:
        info = yaml.safe_load(f)

    image_file = Path(info['image'])

    if not image_file.is_absolute():
        image_file = yaml_file.parent / image_file

    image = cv2.imread(
        str(image_file),
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise RuntimeError(
            f'Cannot read {image_file}'
        )

    return (
        image,
        float(info['resolution']),
        info['origin']
    )


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml')
    args = parser.parse_args()

    yaml_file = Path(
        args.map_yaml
    ).expanduser().resolve()

    image, resolution, origin = load_map(
        yaml_file
    )

    # -------------------------------------------
    # 1. Observed free space
    # -------------------------------------------

    free = image >= 250

    count, labels, stats, _ = \
        cv2.connectedComponentsWithStats(
            free.astype(np.uint8),
            connectivity=8
        )

    if count <= 1:
        raise RuntimeError(
            'No connected free-space region found'
        )

    largest = 1 + int(
        np.argmax(
            stats[1:, cv2.CC_STAT_AREA]
        )
    )

    track = labels == largest

    # -------------------------------------------
    # 2. Clearance field
    # -------------------------------------------

    distance_px = cv2.distanceTransform(
        track.astype(np.uint8),
        cv2.DIST_L2,
        5
    )

    distance_m = (
        distance_px * resolution
    )

    # -------------------------------------------
    # 3. Safe region
    # -------------------------------------------

    safe_clearance = 0.20

    safe = (
        track &
        (distance_m >= safe_clearance)
    )

    # -------------------------------------------
    # 4. Find local clearance maxima
    #
    # A point is a candidate when its clearance
    # is equal to the maximum in its local
    # neighbourhood.
    # -------------------------------------------

    kernel_size = 7

    kernel = np.ones(
        (kernel_size, kernel_size),
        dtype=np.uint8
    )

    local_max = cv2.dilate(
        distance_m.astype(np.float32),
        kernel
    )

    candidates = (
        safe &
        (distance_m >= local_max - 1e-5)
    )

    # Ignore very weak maxima.
    candidates &= (
        distance_m >= 0.25
    )

    # -------------------------------------------
    # 5. Visualization
    # -------------------------------------------

    visual = cv2.cvtColor(
        image,
        cv2.COLOR_GRAY2BGR
    )

    # Safe area = green tint.
    visual[safe] = (
        visual[safe] * 0.4 +
        np.array([0, 150, 0]) * 0.6
    ).astype(np.uint8)

    ys, xs = np.where(candidates)

    for x, y in zip(xs, ys):

        cv2.circle(
            visual,
            (int(x), int(y)),
            2,
            (0, 0, 255),
            -1
        )

    output = (
        yaml_file.parent /
        'centerline_candidates.png'
    )

    cv2.imwrite(
        str(output),
        visual
    )

    # -------------------------------------------
    # 6. Save numerical candidate points
    # -------------------------------------------

    candidate_file = (
        yaml_file.parent /
        'centerline_candidates.csv'
    )

    with open(candidate_file, 'w') as f:

        f.write(
            'pixel_x,pixel_y,map_x,map_y,clearance\n'
        )

        height = image.shape[0]

        for x, y in zip(xs, ys):

            # PGM Y axis points downward.
            # ROS map Y axis points upward.

            map_x = (
                origin[0] +
                (x + 0.5) * resolution
            )

            map_y = (
                origin[1] +
                (
                    height - y - 0.5
                ) * resolution
            )

            clearance = distance_m[
                y,
                x
            ]

            f.write(
                f'{x},'
                f'{y},'
                f'{map_x:.4f},'
                f'{map_y:.4f},'
                f'{clearance:.4f}\n'
            )

    print()
    print('===== CENTERLINE CANDIDATES =====')
    print(
        f'Candidates:       {len(xs)}'
    )
    print(
        f'Safe clearance:   {safe_clearance:.2f} m'
    )
    print(
        f'Max clearance:    {distance_m.max():.3f} m'
    )
    print()
    print('Saved:')
    print(output)
    print(candidate_file)


if __name__ == '__main__':
    main()
