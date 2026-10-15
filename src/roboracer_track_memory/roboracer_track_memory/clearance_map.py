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

    parser.add_argument(
        'map_yaml'
    )

    args = parser.parse_args()

    yaml_file = Path(
        args.map_yaml
    ).expanduser().resolve()

    image, resolution, origin = load_map(
        yaml_file
    )

    # Conservative classification.
    free = image >= 250

    # Keep only largest connected observed-free region.
    count, labels, stats, _ = \
        cv2.connectedComponentsWithStats(
            free.astype(np.uint8),
            connectivity=8
        )

    if count <= 1:
        raise RuntimeError(
            'No connected free-space region found'
        )

    largest = (
        1 +
        np.argmax(
            stats[1:, cv2.CC_STAT_AREA]
        )
    )

    track = labels == largest

    # Distance transform:
    # Each free pixel receives its distance from
    # the nearest non-drivable pixel.
    distance_px = cv2.distanceTransform(
        track.astype(np.uint8),
        cv2.DIST_L2,
        5
    )

    distance_m = distance_px * resolution

    maximum = float(distance_m.max())

    print()
    print('===== CLEARANCE MAP =====')
    print(
        f'Resolution:        '
        f'{resolution:.3f} m/pixel'
    )
    print(
        f'Max clearance:     '
        f'{maximum:.3f} m'
    )

    # --------------------------------------------------
    # Visualisation
    # --------------------------------------------------

    if maximum > 0.0:
        normalized = (
            distance_m / maximum * 255.0
        ).astype(np.uint8)
    else:
        normalized = np.zeros_like(
            image,
            dtype=np.uint8
        )

    heat = cv2.applyColorMap(
        normalized,
        cv2.COLORMAP_JET
    )

    # Unknown/occupied/non-main-region = dark.
    heat[~track] = (0, 0, 0)

    output = (
        yaml_file.parent /
        'clearance_map.png'
    )

    cv2.imwrite(
        str(output),
        heat
    )

    # Save actual numerical clearance values too.
    np.save(
        yaml_file.parent /
        'clearance_map.npy',
        distance_m
    )

    # --------------------------------------------------
    # Generate conservative safe region
    # --------------------------------------------------

    # Temporary development value.
    #
    # This is NOT yet the final physical-car clearance.
    safe_clearance = 0.20

    safe = (
        track &
        (distance_m >= safe_clearance)
    )

    safe_image = np.zeros(
        image.shape,
        dtype=np.uint8
    )

    safe_image[safe] = 255

    safe_output = (
        yaml_file.parent /
        'safe_corridor.png'
    )

    cv2.imwrite(
        str(safe_output),
        safe_image
    )

    print(
        f'Temporary safe clearance: '
        f'{safe_clearance:.2f} m'
    )

    print()
    print(
        'Saved:',
        output
    )

    print(
        'Saved:',
        safe_output
    )

    print(
        'Saved:',
        yaml_file.parent /
        'clearance_map.npy'
    )


if __name__ == '__main__':
    main()
