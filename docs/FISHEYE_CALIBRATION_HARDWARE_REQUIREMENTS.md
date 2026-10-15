# Fisheye calibration — extrinsic and geometric hardware requirements

This document defines what **vehicle integrators and camera hardware suppliers** must provide so surround-view fisheye calibration (intrinsics + extrinsics) can reach production accuracy. It aligns with how this middleware loads parameters from **`config/rig.json`** and per-lens files under **`calib/<role>_fisheye.json`**.

**Scope:** mechanical mounting, vehicle coordinate conventions, nominal camera pose, and rig-level geometry. **Out of scope:** intrinsic lens distortion fitting (checkerboard/Charuco procedure), ISP tuning, and Android UI—see `ARCHITECTURE.md` and `calibration/` for those workflows.

---

## 1. Goals and accuracy drivers

Surround processing depends on two geometric layers:

1. **Intrinsics** — how pixels map to rays (focal length, principal point, Kannala–Brandt / equidistant coefficients).
2. **Extrinsics** — how each camera is mounted on the vehicle (orientation; height and lateral position for ground and fusion).

Errors in extrinsics show up as:

| Symptom | Typical cause |
|--------|----------------|
| Visible seams / ghosting between panels | Yaw error between adjacent cameras |
| Horizon or curb “swim” left vs right | Roll mismatch or unequal pitch |
| Wrong distance to ground / curb | Pitch error or incorrect camera height |
| Panorama shifted vs vehicle outline in 3D | Lateral (Y) or forward (X) mount position error |

Nominal hardware data narrows the search space for field calibration; small residual errors are refined into **`yaw_offset_deg`** and **`pitch_offset_deg`** in software.

---

## 2. Coordinate systems (mandatory agreement)

All customer-supplied geometry must be delivered in a **single documented vehicle frame**. The surround pipeline uses:

| Axis | Direction |
|------|-----------|
| **+X** | Forward (toward front of vehicle) |
| **+Y** | Right (passenger side on LHD vehicles) |
| **+Z** | Up |

- **Ground plane** is the horizontal plane **Z = `ground_plane_z`** (normally **0** at nominal ride height on a flat surface).
- **Camera optical frame** follows **OpenCV**: **X** right in the image, **Y** down, **Z** forward along the optical axis. The middleware applies a fixed rotation from camera to vehicle (`surround/geometry/PoseMath.java`).

**WP-5 / parking-guide JSON** uses **+Y left** (ISO-style). When exporting CAD, either convert once to **+Y right** for `rig.json`, or document the conversion matrix supplied with the data.

**Recommended vehicle origin:** **rear axle center**, mid-track, at ground contact (or at a defined height offset). If another origin is used (e.g. vehicle center of gravity), it must be stated explicitly with the transform to rear axle.

---

## 3. Per-camera extrinsic parameters (required from hardware)

For each fisheye module (**`front`**, **`left`**, **`right`**, **`rear`** — roles must match `camera_order` in `rig.json`):

### 3.1 Orientation (required)

Supply **nominal mount orientation** as seen from the vehicle frame:

| Parameter | Definition | Maps to `rig.json` |
|-----------|------------|-------------------|
| **Yaw** | Rotation about vehicle **+Z**: 0° = optical axis horizontal projection aligned with **+X** (forward); +90° = toward **+Y** (right); −90° = toward **−Y** (left); 180° = rear | `yaw_deg` |
| **Pitch** | Downward tilt of optical axis from horizontal in the sagittal plane of the mount. **Negative** values = looking toward the ground (typical for surround: roughly −10° to −25° depending on camera) | `pitch_deg` |
| **Roll** | Twist about the optical axis (clocking of the sensor/module). **0°** if the image row axis is level with vehicle lateral axis | `roll_deg` |

**Rotation order** implemented in software: **yaw (Z) → pitch (Y) → roll (X)**, after the fixed OpenCV-camera-to-mount transform. Customer CAD rotations must be converted to these three angles or delivered as a **3×3 rotation matrix** plus a note that conversion was verified.

**Suggested accuracy for nominal values (design intent):**

| Angle | Target specification | Notes |
|-------|---------------------|--------|
| Yaw (per camera) | ±0.5° | Drives seam alignment |
| Pitch | ±0.5° | Drives ground intersection |
| Roll | ±0.3° | Often neglected in CAD; critical on side cameras |

### 3.2 Position (strongly recommended)

The current `rig.json` schema stores **rotation only** per camera; **translation is approximated** by shared **`camera_height_z`** in the panorama section. For accurate curb ranging and 3D alignment, suppliers should still provide:

| Quantity | Definition | Unit |
|----------|------------|------|
| **x** | Forward offset of **lens entrance pupil / optical center** from vehicle origin | m |
| **y** | Lateral offset (positive = right) | m |
| **z** | Height of optical center above `ground_plane_z` | m |

Document whether the point is **optical center** vs **housing datum**; if housing datum, include the offset to the optical center.

**Minimum for software today:** a single **`camera_height_z`** (m) per rig at the **calibration ride condition** (load, tire pressure, suspension height). Prefer **per-camera z** if mounts differ (e.g. rear bumper camera lower than side mirrors).

### 3.3 Fine trims (software / field calibration, not hardware design)

These are **not** primary hardware deliverables; they are filled after capture:

| Field | Purpose |
|-------|---------|
| `yaw_offset_deg` | Residual horizontal alignment between overlapping panels |
| `pitch_offset_deg` | Residual ground-plane / horizon alignment |

Hardware should be repeatable enough that offsets stay within **±2°** after nominal CAD is loaded.

---

## 4. Rig-level geometric parameters

Provide with the vehicle program or calibration bench:

| Parameter | Description | Maps to `rig.json` |
|-----------|-------------|-------------------|
| **Camera height** | Optical center height above ground at calibration load | `panorama.camera_height_z` |
| **Ground reference** | Z level treated as road surface | `panorama.ground_plane_z` |
| **Wheelbase** | Front–rear axle distance | Validation / 3D model alignment |
| **Track width** (front/rear) | Wheel center separation | Validation |
| **Overhangs** | Bumper to axle distances | Optional seam QA |
| **Ride height specification** | Empty vs GVW; suspension mode if adjustable | Document which condition matches `camera_height_z` |

---

## 5. Mechanical and optical hardware requirements

### 5.1 Camera module

| Requirement | Rationale |
|-------------|-----------|
| **Stable lens–sensor assembly** | Intrinsics must not drift with temperature or vibration |
| **Documented horizontal FOV** (e.g. 180°–190°) | Prior for synthetic / weak calibration |
| **Known active pixel size and resolution** | Must match streamed frames (`width` / `height` in rig) |
| **Principal point offset** from mechanical center | Reduces cx/cy ambiguity during intrinsic fit |
| **Consistent role labeling** | Cable / ECU channel `front`/`left`/`right`/`rear` matches physical location |

### 5.2 Mounting bracket

| Requirement | Rationale |
|-------------|-----------|
| **Repeatable clocking** | Limits roll error |
| **Stiffness** | Pitch/yaw under vibration < 0.2° |
| **Thermal expansion** | Document if bracket material causes seasonal shift |
| **Datum definition** | Single CAD reference for bracket → optical center transform |

### 5.3 Installation tolerances (production)

Define and measure on a sample of vehicles:

| Tolerance | Recommended max |
|-----------|-----------------|
| Position (x, y, z) per camera | ±5 mm (±2 mm preferred for premium surround) |
| Yaw / pitch / roll | ±1° (±0.5° preferred) |
| Pairwise yaw between adjacent cameras | 90° ±1° (for four-camera 360° layouts) |

Out-of-tolerance units should be flagged for **field extrinsic** refinement rather than relying on fleet-average CAD.

### 5.4 Environmental and electrical

| Item | Requirement |
|------|-------------|
| **Synchronization** | Gen-lock or timestamped frames if using motion-based calibration |
| **Image orientation** | No undocumented 90°/180° rotation in ISP; if present, document for intrinsic pipeline |
| **Lens protection** | Note if heater or outer lens shifts effective optical center |

---

## 6. Field calibration support (hardware-enabling)

To run **checkerboard / Charuco** extrinsic refinement (future `calibration_mode: checkerboard`):

| Item | Requirement |
|------|-------------|
| **Flat target** | Known geometry; size suited to fisheye (large board or floor pattern) |
| **Target placement** | Visible in **overlap regions** between adjacent cameras |
| **Ground** | Flat, level area; same ride height as specified in §4 |
| **Lighting** | Even illumination; avoid saturation on fisheye rim |
| **Vehicle position** | Document position of vehicle origin relative to target for absolute pose (optional) |

For **overlap-only** refinement (no full 6-DOF solve), stationary scenes with straight lines (parking structure) are sufficient to tune `yaw_offset_deg` / `pitch_offset_deg` using repository tools (`calibration/optimize_extrinsics.py`, `calibration/ground_plane_align.py`).

---

## 7. Data delivery format

### 7.1 Minimum package (per vehicle program)

1. **Coordinate system diagram** (origin, axes, units).
2. **Table per camera** (see template below).
3. **Ride condition** for height (load, tire pressure, suspension setting).
4. **Channel map**: video stream index / ECU ID → `front` | `left` | `right` | `rear`.
5. **Mechanical drawing** or STEP with optical axis and entrance pupil marked.

### 7.2 Per-camera table template

| Role | x (m) | y (m) | z (m) | yaw (°) | pitch (°) | roll (°) | Notes |
|------|-------|-------|-------|---------|-----------|----------|-------|
| front | | | | 0 | | 0 | |
| left | | | | −90 | | 0 | |
| right | | | | +90 | | 0 | |
| rear | | | | 180 | | 0 | |

Yaw/pitch/roll columns are **examples** for a symmetric four-camera rig; use **CAD-derived** values for the actual program.

### 7.3 Mapping into middleware configuration

**Extrinsics and rig geometry** → `config/rig.json`:

```json
"panorama": {
  "camera_height_z": 1.05,
  "ground_plane_z": 0.0
},
"cameras": {
  "front": {
    "yaw_deg": 0.0,
    "pitch_deg": -12.0,
    "roll_deg": 0.0,
    "yaw_offset_deg": 0.0,
    "pitch_offset_deg": 0.0
  }
}
```

**Lens intrinsics** (after measurement) → `calib/<role>_fisheye.json` (may mirror fx, fy, cx, cy, k1–k4, `projection`).

Set **`calibration_mode`** to `checkerboard` when intrinsics/extrinsics come from a target-based calibration; use `synthetic` only for provisional AI or weak fits.

---

## 8. Verification checklist (acceptance)

Before sign-off between hardware supplier and calibration team:

- [ ] Vehicle frame and units documented and match §2.
- [ ] Four roles identified and wired consistently.
- [ ] Nominal yaw/pitch/roll per camera from CAD (or matrix + conversion note).
- [ ] Optical center position (x, y, z) or per-camera height + bracket offsets.
- [ ] `camera_height_z` agreed for calibration ride condition.
- [ ] Installation tolerances defined and sampled on pilot builds.
- [ ] Stream resolution and orientation match `rig.json` `width` / `height`.
- [ ] After intrinsic fit, residual `yaw_offset_deg` / `pitch_offset_deg` within agreed limits on a golden vehicle.
- [ ] Seam and ground-line QA on flat ground at defined distances (e.g. 0.5 m, 2 m, 5 m).

---

## 9. Related repository artifacts

| Path | Content |
|------|---------|
| `config/rig.json` | Runtime rig: intrinsics + extrinsics + panorama |
| `config/rig.yaml.example` | Annotated example |
| `calib/*_fisheye.json` | Per-camera lens parameters |
| `surround/calibration/CameraExtrinsics.java` | Effective yaw/pitch = nominal + offset |
| `surround/geometry/PoseMath.java` | Camera-to-vehicle rotation |
| `calibration/synthetic_calibrate.py` | Default nominal extrinsics for synthetic mode |
| `ARCHITECTURE.md` | Pipeline overview and calibration modes |
| `CURB_ARCHITECTURE.md` | Vehicle frame usage for distance / zones |

---

## 10. Revision history

| Version | Date | Change |
|---------|------|--------|
| 1.0 | 2026-10-15 | Initial hardware extrinsic / geometric requirements |
