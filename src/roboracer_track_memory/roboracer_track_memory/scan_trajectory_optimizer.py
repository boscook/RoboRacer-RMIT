#!/usr/bin/env python3
"""Racing-line optimiser that needs NO SLAM map (uses lap-1 wall distances)."""
import argparse
import csv
import math
from pathlib import Path

import numpy as np


def load(path):
    rows = list(csv.DictReader(open(path)))
    if len(rows) < 20:
        raise RuntimeError('Not enough trajectory points')
    if 'left' not in rows[0]:
        raise RuntimeError('Trajectory has no left/right wall distances')
    a = np.array([[float(r['x']), float(r['y']), float(r.get('yaw', 0.0)),
                   float(r['left']), float(r['right'])] for r in rows])
    # clean_lap1: keep only forward progress (no roll-backs / reversing).
    keep = [0]
    for i in range(1, len(a)):
        dx = a[i, 0] - a[keep[-1], 0]
        dy = a[i, 1] - a[keep[-1], 1]
        if dx * math.cos(a[i, 2]) + dy * math.sin(a[i, 2]) > 0.03:
            keep.append(i)
    print(f'clean_lap1: kept {len(keep)} of {len(a)} lap-1 points')
    a = a[keep][:, [0, 1, 3, 4]]
    if np.linalg.norm(a[0, :2] - a[-1, :2]) < 0.10:
        a = a[:-1]
    return a


def resample_closed(a, ds):
    closed = np.vstack([a, a[:1]])
    seg = np.hypot(*np.diff(closed[:, :2], axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = max(20, int(round(total / ds)))
    t = np.linspace(0.0, total, n, endpoint=False)
    return np.column_stack([np.interp(t, s, closed[:, k]) for k in range(4)]), total / n


def circ_smooth(v, passes=1):
    for _ in range(passes):
        v = 0.25 * np.roll(v, 1) + 0.5 * v + 0.25 * np.roll(v, -1)
    return v


def curvature(p):
    a, b, c = np.roll(p, 1, axis=0), p, np.roll(p, -1, axis=0)
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = (np.hypot(*ab.T) * np.hypot(*bc.T) * np.hypot(*ac.T)) + 1e-9
    return 2.0 * cross / den


def optimise(a, ds=0.10, margin=0.22, iters=4000, alpha=0.5, kmax=1.0):
    r, step = resample_closed(a, ds)
    p0, left, right = r[:, :2], r[:, 2], r[:, 3]
    left = circ_smooth(left, 2)
    right = circ_smooth(right, 2)

    t = np.roll(p0, -1, axis=0) - np.roll(p0, 1, axis=0)
    t /= (np.linalg.norm(t, axis=1, keepdims=True) + 1e-9)
    n = np.column_stack([-t[:, 1], t[:, 0]])

    lo = -(right - margin)
    hi = left - margin
    centre = 0.5 * (hi + lo)
    bad = lo > hi
    lo[bad] = hi[bad] = centre[bad]
    k = 3
    hi_f = hi.copy()
    lo_f = lo.copy()
    for j in range(1, k + 1):
        hi_f = np.minimum(hi_f, np.minimum(np.roll(hi, j), np.roll(hi, -j)))
        lo_f = np.maximum(lo_f, np.maximum(np.roll(lo, j), np.roll(lo, -j)))
    bad = lo_f > hi_f
    mid = 0.5 * (hi + lo)
    lo_f[bad] = hi_f[bad] = mid[bad]

    q = p0.copy()
    for _ in range(8):
        q = 0.25 * np.roll(q, 1, axis=0) + 0.5 * q + 0.25 * np.roll(q, -1, axis=0)
    k0 = curvature(q)
    rmin = 1.0 / kmax
    room = np.maximum(0.0, 1.0 / np.maximum(np.abs(k0), 1e-6) - rmin)
    hi_f = np.where(k0 > 0, np.minimum(hi_f, room), hi_f)
    lo_f = np.where(k0 < 0, np.maximum(lo_f, -room), lo_f)
    W = 8
    hw, lw = hi_f.copy(), lo_f.copy()
    for j in range(1, W + 1):
        hw = np.minimum(hw, np.minimum(np.roll(hi_f, j), np.roll(hi_f, -j)))
        lw = np.maximum(lw, np.maximum(np.roll(lo_f, j), np.roll(lo_f, -j)))
    hi_f, lo_f = circ_smooth(hw, 6), circ_smooth(lw, 6)
    bad = lo_f > hi_f
    lo_f[bad] = hi_f[bad] = 0.5 * (lo_f[bad] + hi_f[bad])

    d = np.clip(np.zeros(len(p0)), lo_f, hi_f)
    for _ in range(iters):
        p = p0 + d[:, None] * n
        target = 0.5 * (np.roll(p, 1, axis=0) + np.roll(p, -1, axis=0))
        d = np.clip(d + alpha * np.einsum('ij,ij->i', target - p, n), lo_f, hi_f)
    p = p0 + d[:, None] * n
    clearance = np.minimum(left - d, right + d)

    closed = np.vstack([p, p[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    m = len(p)
    u = np.linspace(0.0, s[-1], m, endpoint=False)
    p = np.column_stack([np.interp(u, s, closed[:, 0]),
                         np.interp(u, s, closed[:, 1])])
    clr = np.interp(u, s[:-1], clearance)
    pre = p.copy()
    for _ in range(8):
        p = 0.25 * np.roll(p, 1, axis=0) + 0.5 * p + 0.25 * np.roll(p, -1, axis=0)
    clr = clr - np.hypot(*(p - pre).T)
    return p, clr, s[-1] / m


def speed_profile(kappa, clearance, step, vmax, vmin, a_lat, a_acc, a_brk):
    k = circ_smooth(np.abs(kappa), 4)
    k = np.minimum(k, 2.0)  # fix_lap1: ignore kinks tighter than 0.5 m
    v = np.minimum(vmax, np.sqrt(a_lat / np.maximum(k, 1e-3)))
    v = np.where(clearance < 0.12, np.minimum(v, vmin + 0.1), v)
    n = len(v)
    for _ in range(3):
        for i in range(2 * n):
            j, m = i % n, (i + 1) % n
            v[m] = min(v[m], math.sqrt(v[j] ** 2 + 2.0 * a_acc * step))
        for i in range(2 * n, 0, -1):
            j, m = i % n, (i - 1) % n
            v[m] = min(v[m], math.sqrt(v[j] ** 2 + 2.0 * a_brk * step))
    return np.clip(v, vmin, vmax)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    ap.add_argument('map_yaml', nargs='?', default=None)
    ap.add_argument('--output', default=None)
    ap.add_argument('--vmax', type=float, default=3.5)
    ap.add_argument('--vmin', type=float, default=0.4)
    ap.add_argument('--alat', type=float, default=4.0)
    ap.add_argument('--aacc', type=float, default=2.5)
    ap.add_argument('--abrk', type=float, default=3.0)  # turn_in_v2
    ap.add_argument('--margin', type=float, default=0.22)
    ap.add_argument('--kmax', type=float, default=1.0)
    args = ap.parse_args()

    src = Path(args.trajectory).expanduser().resolve()
    out = Path(args.output).expanduser().resolve() if args.output \
        else src.parent / 'learned_track.csv'

    a = load(src)
    p, clearance, step = optimise(a, margin=args.margin, kmax=args.kmax)
    t = np.roll(p, -1, axis=0) - p
    yaw = np.arctan2(t[:, 1], t[:, 0])
    kappa = curvature(p)
    v = speed_profile(kappa, clearance, step, args.vmax, args.vmin,
                      args.alat, args.aacc, args.abrk)

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['index', 'x', 'y', 'yaw', 'curvature', 'clearance',
                    'target_speed'])
        for i in range(len(p)):
            w.writerow([i, f'{p[i,0]:.6f}', f'{p[i,1]:.6f}', f'{yaw[i]:.6f}',
                        f'{kappa[i]:.6f}', f'{clearance[i]:.6f}',
                        f'{v[i]:.6f}'])
    print(f'Saved: {out}')
    print(f'points={len(p)} length={step*len(p):.1f} m '
          f'min_clear={clearance.min():.2f} '
          f'v=[{v.min():.2f},{v.max():.2f}] max|k|={np.abs(kappa).max():.2f}')


if __name__ == '__main__':
    main()
