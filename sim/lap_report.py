"""Compare the planned lap time with your real lap time.
Run:  docker cp ~/roboracer_twin/lap_report.py rr_twin:/root/ && docker exec rr_twin python3 /root/lap_report.py
"""
import csv, math, os
d = os.path.expanduser('~/roboracer_ws/track_memory/active_track')
for name in ('learned_track_iter.csv', 'learned_track.csv'):
    f = os.path.join(d, name)
    if os.path.exists(f):
        break
else:
    raise SystemExit('No learned track yet: let it finish lap 1 first.')
r = list(csv.DictReader(open(f)))
x = [float(a['x']) for a in r]; y = [float(a['y']) for a in r]
v = [float(a['target_speed']) for a in r]
n = len(r); L = 0.0; T = 0.0; slow = 0.0
for i in range(n):
    j = (i + 1) % n
    ds = math.hypot(x[j] - x[i], y[j] - y[i])
    L += ds; T += ds / max(v[i], 0.05)
    if v[i] < 1.0:
        slow += ds
print(f'File: {name}')
print(f'Track length     : {L:.1f} m')
print(f'Planned lap time : {T:.1f} s   (avg {L / T:.2f} m/s)')
print(f'Planned speed    : min {min(v):.2f}  max {max(v):.2f} m/s')
print(f'Track below 1 m/s: {100 * slow / L:.0f} %')
print(f'For a 15 s lap you need an average of {L / 15:.2f} m/s')
