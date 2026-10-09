#!/usr/bin/env python3
"""
Fake vehicle gateway: pushes steering + distance-sensor data INTO the middleware,
exactly like the real CAN/Ethernet reader will (POST /api/v1/ingest/...).
Python 3.8+, standard library only.

  python tools/fake_vehicle.py                       # scripted parallel-parking, loops
  python tools/fake_vehicle.py --scenario sweep      # steering sweeps +-450 deg
  python tools/fake_vehicle.py --scenario static --steer 200 --rear 0.4 --curb 0.7
  python tools/fake_vehicle.py --host 192.168.1.20 --port 9090 --token SECRET

Start the server WITHOUT -Dwp5.simulate=true, otherwise two sources fight.
"""
import argparse, json, math, sys, time, urllib.request, urllib.error


def post(base, path, obj, token):
    req = urllib.request.Request(base + path, data=json.dumps(obj).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    if token:
        req.add_header("X-Api-Key", token)
    with urllib.request.urlopen(req, timeout=2) as r:
        return r.status


def sensor(pos, d, kind=None, height=None):
    if kind is None:   # zones are CURB-only on the server: LEFT_*/RIGHT_* sensors are curb sensors
        kind = "CURB" if pos.startswith(("LEFT_", "RIGHT_")) else "OBSTACLE"
    s = {"id": pos, "pos": pos, "distanceM": -1 if d is None else round(d, 3),
         "kind": kind, "source": "ULTRASONIC"}
    if height is not None:
        s["heightM"] = height
    return s


def smooth(t, a, b):          # 0..1 ramp between times a and b
    return min(1.0, max(0.0, (t - a) / (b - a)))


def parking(t):
    """~30 s loop: reverse, turn wheel right, then left, rear obstacle + kerb get closer."""
    t = t % 30.0
    if t < 4:    swa = 0
    elif t < 10: swa = -450 * smooth(t, 4, 10)             # full right lock
    elif t < 14: swa = -450
    elif t < 22: swa = -450 + 900 * smooth(t, 14, 22)      # swing to full left lock
    else:        swa = 450 * (1 - smooth(t, 22, 28))       # straighten
    rear = 2.5 - 2.2 * smooth(t, 3, 26)                    # 2.5 m -> 0.3 m
    curb_l = 1.6 - 1.1 * smooth(t, 8, 24)                  # 1.6 m -> 0.5 m
    s = [sensor("REAR_CENTER", rear if rear < 2.5 else None),
         sensor("REAR_LEFT", rear + 0.3 if rear + 0.3 < 2.5 else None),
         sensor("REAR_RIGHT", None),
         sensor("LEFT_REAR", curb_l, "CURB", 0.12),
         sensor("LEFT_FRONT", curb_l + 0.2, "CURB", 0.12),
         sensor("RIGHT_REAR", 1.9), sensor("RIGHT_FRONT", None)]
    return swa, "R", 1.5, s


def sweep(t):
    swa = 450 * math.sin(2 * math.pi * t / 12.0)
    rear = 1.35 + 1.15 * math.sin(2 * math.pi * t / 8.0)
    return swa, "R", 1.5, [sensor("REAR_CENTER", rear), sensor("LEFT_REAR", 0.7 + 0.45 * math.sin(2 * math.pi * t / 6.0 + 1), "CURB", 0.12)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost"); ap.add_argument("--port", type=int, default=9091)
    ap.add_argument("--token", default=""); ap.add_argument("--rate", type=float, default=20.0, help="messages per second")
    ap.add_argument("--scenario", choices=["park", "sweep", "static"], default="park")
    ap.add_argument("--duration", type=float, default=0, help="seconds, 0 = forever")
    ap.add_argument("--steer", type=float, default=0); ap.add_argument("--gear", default="R")
    ap.add_argument("--rear", type=float, default=None); ap.add_argument("--curb", type=float, default=None)
    a = ap.parse_args()
    base = "http://%s:%d" % (a.host, a.port)
    print("Sending to", base, "scenario:", a.scenario, " (Ctrl+C to stop)")
    t0, n = time.time(), 0
    try:
        while a.duration <= 0 or time.time() - t0 < a.duration:
            t = time.time() - t0
            if a.scenario == "park":    swa, gear, spd, s = parking(t)
            elif a.scenario == "sweep": swa, gear, spd, s = sweep(t)
            else:
                swa, gear, spd = a.steer, a.gear, 1.5
                s = [sensor("REAR_CENTER", a.rear), sensor("LEFT_REAR", a.curb, "CURB", 0.12)]
            post(base, "/api/v1/ingest/steering", {"steeringWheelDeg": round(swa, 2), "gear": gear, "speedKph": spd}, a.token)
            post(base, "/api/v1/ingest/range", {"sensors": s}, a.token)
            n += 1
            if n % int(max(1, a.rate)) == 0:
                print("t=%5.1fs  steering %7.1f deg  gear %s  %s" % (t, swa, gear,
                      "  ".join("%s=%s" % (x["pos"], "clear" if x["distanceM"] < 0 else "%.2f" % x["distanceM"]) for x in s[:4])))
            time.sleep(1.0 / a.rate)
    except urllib.error.URLError as e:
        sys.exit("Cannot reach middleware at %s: %s\nIs the server running and the port right?" % (base, e))
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main()
