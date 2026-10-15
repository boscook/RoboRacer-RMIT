#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_map(yaml_path):

    yaml_path = Path(yaml_path).expanduser().resolve()

    with open(yaml_path, 'r') as f:
        info = yaml.safe_load(f)

    image_path = Path(info['image'])

    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise RuntimeError(
            f'Cannot load map image: {image_path}'
        )

    resolution = float(info['resolution'])
    origin = info['origin']

    return image, resolution, origin


def classify_map(image):

    # Standard map_server/SLAM-style PGM:
    #
    # white = free
    # black = occupied
    # grey  = unknown
    #
    # We deliberately use conservative thresholds.
    free = image >= 250
    occupied = image <= 10

    unknown = ~(free | occupied)

    return free, occupied, unknown


def keep_largest_free_region(free):

    mask = free.astype(np.uint8)

    count, labels, stats, _ = \
        cv2.connectedComponentsWithStats(
            mask,
            connectivity=8
        )

    if count <= 1:
        return free

    # Ignore component 0 = background.
    areas = stats[1:, cv2.CC_STAT_AREA]

    largest_label = 1 + int(
        np.argmax(areas)
    )

    return labels == largest_label


def make_diagnostic(
    free,
    occupied,
    unknown,
    connected_free
):

    h, w = free.shape

    out = np.zeros(
        (h, w, 3),
        dtype=np.uint8
    )

    # BGR colours:
    #
    # unknown          = grey
    # occupied/wall    = black
    # disconnected free= blue
    # main free region = green

    out[unknown] = (128, 128, 128)
    out[occupied] = (0, 0, 0)

    disconnected = free & (~connected_free)
    out[disconnected] = (255, 0, 0)

    out[connected_free] = (0, 255, 0)

    return out


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        'map_yaml',
        help='Path to map.yaml'
    )

    parser.add_argument(
        '--output',
        default=None
    )

    args = parser.parse_args()

    yaml_path = Path(
        args.map_yaml
    ).expanduser().resolve()

    image, resolution, origin = \
        load_map(yaml_path)

    free, occupied, unknown = \
        classify_map(image)

    connected_free = \
        keep_largest_free_region(free)

    diagnostic = make_diagnostic(
        free,
        occupied,
        unknown,
        connected_free
    )

    if args.output is None:
        output = (
            yaml_path.parent /
            'track_classification.png'
        )
    else:
        output = Path(
            args.output
        ).expanduser().resolve()

    cv2.imwrite(
        str(output),
        diagnostic
    )

    h, w = image.shape

    print()
    print('===== TRACK MAP INFORMATION =====')
    print(f'Image:       {w} x {h} pixels')
    print(f'Resolution:  {resolution:.4f} m/pixel')
    print(
        f'Physical:    '
        f'{w * resolution:.2f} x '
        f'{h * resolution:.2f} m'
    )
    print(
        f'Origin:      '
        f'x={origin[0]:.3f}, '
        f'y={origin[1]:.3f}'
    )
    print()

    total = image.size

    print(
        f'Free:        '
        f'{np.count_nonzero(free)} '
        f'({100*np.count_nonzero(free)/total:.1f}%)'
    )

    print(
        f'Occupied:    '
        f'{np.count_nonzero(occupied)} '
        f'({100*np.count_nonzero(occupied)/total:.1f}%)'
    )

    print(
        f'Unknown:     '
        f'{np.count_nonzero(unknown)} '
        f'({100*np.count_nonzero(unknown)/total:.1f}%)'
    )

    print(
        f'Main free:   '
        f'{np.count_nonzero(connected_free)} pixels'
    )

    print()
    print(f'Saved: {output}')
    print()
    print('Diagnostic colours:')
    print('  GREEN = main connected free/drivable region')
    print('  BLUE  = other disconnected free regions')
    print('  BLACK = occupied/wall')
    print('  GREY  = unknown - NOT considered drivable')


if __name__ == '__main__':
    main()
