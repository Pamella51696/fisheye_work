# WP5 vehicle signal source (UDP vs simulator)

Branch: **`upd-inetgration`** (+ `cursor/wp5-signal-source-switch-7e98`).

`VehicleSignalManager` chooses between the existing **`Wp5Simulator`** and **UDP ingress** without changing `SteeringModel`, `CurbModel`, or `SignalHub`.

## Switch

| Property | Values | Default |
|----------|--------|---------|
| `wp5.signalSource` | `AUTO` \| `UDP` \| `SIMULATOR` | `AUTO` (or `SIMULATOR` if `-Dwp5.simulate=true` and property unset) |
| `wp5.udpTimeoutMs` | stale threshold (ms) | `1000` |
| `wp5.autoStartupTimeoutMs` | AUTO: wait before simulator (ms) | `3000` |
| `wp5.udpListenPort` | UDP bind port | `45454` |

Legacy: `-Dwp5.simulate=true` with no `wp5.signalSource` → **SIMULATOR**.

## Behaviour

| Mode | Behaviour |
|------|-----------|
| **SIMULATOR** | `Wp5Simulator` only (same loop as before). |
| **UDP** | 16-byte LE `[steeringDeg, headingDeg]` on `udpListenPort`. No simulator. Stale when packets stop. |
| **AUTO** | Prefer fresh UDP; if none by startup timeout → simulator; if UDP drops → simulator; if UDP returns → stop simulator. |

UDP packet format matches the udp-vehicle-pose middleware: two little-endian `double`s (steering, heading).

## Examples

```bash
# Development (simulator after 3s if no UDP)
java -Dwp5.signalSource=AUTO -jar ...

# Lab replay / real bus
java -Dwp5.signalSource=UDP -Dwp5.udpListenPort=45454 -jar ...

# Explicit simulator (same as old -Dwp5.simulate=true)
java -Dwp5.signalSource=SIMULATOR -jar ...
```

## Debug

`GET /api/v1/signal-source` — configured mode, active source, UDP freshness.

HTTP `POST /api/v1/ingest/steering` still works when simulator is active or UDP is stale (gateway path). It is ignored while **UDP** mode has fresh packets.
