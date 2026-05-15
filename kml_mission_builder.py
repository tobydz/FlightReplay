#!/usr/bin/env python3
"""
kml_mission_builder.py

Reads a Google Earth KML containing named property point markers and area polygons,
then generates one oblique 3D mapping KMZ per property for DJI Pilot 2 / M3E.

Each KMZ contains:
  Pass 1 — Nadir grid     (gimbal -90°)  full top coverage
  Pass 2 — Oblique N→E→S→W (gimbal -45°)  facade/side coverage for 3D reconstruction

Defaults tuned for RealityCapture with M3E wide camera at 50 m AGL:
  80 % frontal overlap, 75 % side overlap, ~1.9 cm/pixel GSD

Usage:
  python kml_mission_builder.py --kml /path/to/file.kml [--altitude 50] [--output ./Missions]
"""

import argparse
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree as ET

# Reuse WPML / KMZ generation from companion script
sys.path.insert(0, str(Path(__file__).parent))
from dji_mission_builder import (
    build_waylines_wpml,
    build_template_kml,
    pack_kmz,
    _normalise_yaw,
)

# ---------------------------------------------------------------------------
# M3E Wide camera  (24 mm equiv, 4/3" sensor, 20 MP)
# ---------------------------------------------------------------------------
M3E_HFOV_DEG = 84.0
M3E_VFOV_DEG = 63.0

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_ALTITUDE      = 50.0
DEFAULT_FRONTAL_OVL   = 0.80
DEFAULT_SIDE_OVL      = 0.75
DEFAULT_OBLIQUE_PITCH = -45.0
DEFAULT_SPEED         = 8.0
DEFAULT_GIMBAL_SETTLE = 3.0
DEFAULT_DWELL         = 2.0
DEFAULT_RTH_HEIGHT    = 100.0

_KNS = "http://www.opengis.net/kml/2.2"


# ===========================================================================
# KML parsing
# ===========================================================================

def parse_kml(kml_path: str) -> dict:
    """
    Return {property_name: {'polygon': [(lat,lon),...], 'ground_alt': float}}.
    Matches 'Name Area' polygons to 'Name' point markers to get ground elevation.
    """
    tree = ET.parse(kml_path)
    root = tree.getroot()

    def _tag(n):
        return f"{{{_KNS}}}{n}"

    def _parse_coords(text):
        pts = []
        for tok in text.strip().split():
            parts = tok.split(",")
            if len(parts) >= 2:
                lon, lat = float(parts[0]), float(parts[1])
                alt = float(parts[2]) if len(parts) > 2 else 0.0
                pts.append((lat, lon, alt))
        return pts

    points: dict = {}
    polygons: dict = {}

    for pm in root.iter(_tag("Placemark")):
        name_el = pm.find(_tag("name"))
        name = (name_el.text or "").strip()

        coord_el = pm.find(f".//{_tag('Point')}/{_tag('coordinates')}")
        if coord_el is not None:
            pts = _parse_coords(coord_el.text)
            if pts:
                points[name] = pts[0]
            continue

        poly_el = pm.find(f".//{_tag('Polygon')}")
        if poly_el is not None:
            c_el = poly_el.find(f".//{_tag('coordinates')}")
            if c_el is not None:
                polygons[name] = _parse_coords(c_el.text)

    properties: dict = {}
    for poly_name, raw in polygons.items():
        prop_name = poly_name.replace(" Area", "").strip()
        pt = points.get(prop_name)
        ground_alt = pt[2] if pt else 0.0
        latlon = [(c[0], c[1]) for c in raw]
        # Drop duplicate closing vertex
        if len(latlon) > 1 and latlon[0] == latlon[-1]:
            latlon = latlon[:-1]
        properties[prop_name] = {"polygon": latlon, "ground_alt": ground_alt}

    return properties


# ===========================================================================
# Coordinate helpers
# ===========================================================================

def _centroid(coords):
    n = len(coords)
    return sum(c[0] for c in coords) / n, sum(c[1] for c in coords) / n


def _to_local(lat, lon, clat, clon):
    x = (lon - clon) * math.cos(math.radians(clat)) * 111_320.0
    y = (lat - clat) * 111_320.0
    return x, y


def _to_latlon(x, y, clat, clon):
    lat = clat + y / 111_320.0
    lon = clon + x / (math.cos(math.radians(clat)) * 111_320.0)
    return lat, lon


def _rotate(pts, angle):
    ca, sa = math.cos(angle), math.sin(angle)
    return [(x * ca - y * sa, x * sa + y * ca) for x, y in pts]


# ===========================================================================
# Camera footprint and spacing
# ===========================================================================

def _footprint(altitude):
    w = 2.0 * altitude * math.tan(math.radians(M3E_HFOV_DEG / 2.0))
    h = 2.0 * altitude * math.tan(math.radians(M3E_VFOV_DEG / 2.0))
    return w, h


def _strip_spacing(altitude, side_overlap):
    w, _ = _footprint(altitude)
    return w * (1.0 - side_overlap)


def _photo_interval(altitude, frontal_overlap):
    _, h = _footprint(altitude)
    return h * (1.0 - frontal_overlap)


# ===========================================================================
# Minimum-area bounding rectangle → optimal grid direction
# ===========================================================================

def _optimal_angle(local_poly):
    best_angle, best_area = 0.0, float("inf")
    n = len(local_poly)
    for i in range(n):
        p1, p2 = local_poly[i], local_poly[(i + 1) % n]
        angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        rot = _rotate(local_poly, -angle)
        w = max(p[0] for p in rot) - min(p[0] for p in rot)
        h = max(p[1] for p in rot) - min(p[1] for p in rot)
        if w * h < best_area:
            best_area, best_angle = w * h, angle
    return best_angle


# ===========================================================================
# Scan-line / polygon intersection
# ===========================================================================

def _scanline_x(local_poly, scan_y):
    xs = []
    n = len(local_poly)
    for i in range(n):
        x1, y1 = local_poly[i]
        x2, y2 = local_poly[(i + 1) % n]
        if (y1 <= scan_y < y2) or (y2 <= scan_y < y1):
            if abs(y2 - y1) > 1e-9:
                t = (scan_y - y1) / (y2 - y1)
                xs.append(x1 + t * (x2 - x1))
    return sorted(xs)


# ===========================================================================
# Waypoint factory
# ===========================================================================

def _wp(lat, lon, rel_alt, ground_alt, yaw, gimbal_pitch):
    return {
        "lat": float(lat),
        "lon": float(lon),
        "relative_altitude": float(rel_alt),
        "absolute_altitude": float(ground_alt + rel_alt),
        "flight_yaw": float(_normalise_yaw(yaw)),
        "gimbal_pitch": float(max(-90.0, min(35.0, gimbal_pitch))),
        "gimbal_yaw": 0.0,
        "rtk_flag": None,
        "datetime": None,
    }


# ===========================================================================
# Pass 1 — Nadir lawnmower grid
# ===========================================================================

def generate_nadir_grid(polygon, altitude, frontal_overlap, side_overlap, ground_alt):
    """Lawnmower grid aligned with the property's longest axis, gimbal -90°."""
    clat, clon = _centroid(polygon)
    local = [_to_local(lat, lon, clat, clon) for lat, lon in polygon]
    angle = _optimal_angle(local)
    rotated = _rotate(local, -angle)

    ss = _strip_spacing(altitude, side_overlap)
    pi = _photo_interval(altitude, frontal_overlap)

    min_y = min(p[1] for p in rotated)
    max_y = max(p[1] for p in rotated)

    heading_fwd = math.degrees(angle)
    heading_rev = heading_fwd + 180.0

    waypoints = []
    strip_idx = 0
    y = min_y + ss / 2.0

    while y <= max_y + 1e-3:
        xs = _scanline_x(rotated, y)
        if len(xs) >= 2:
            x0, x1 = xs[0], xs[-1]
            heading = heading_fwd
            if strip_idx % 2 == 1:
                x0, x1 = x1, x0
                heading = heading_rev

            n_steps = max(1, round(abs(x1 - x0) / pi))
            for step in range(n_steps + 1):
                t = step / n_steps if n_steps else 0.0
                xr = x0 + t * (x1 - x0)
                # Rotate back to world frame
                wx = xr * math.cos(angle) - y * math.sin(angle)
                wy = xr * math.sin(angle) + y * math.cos(angle)
                lat, lon = _to_latlon(wx, wy, clat, clon)
                waypoints.append(_wp(lat, lon, altitude, ground_alt, heading, -90.0))

        strip_idx += 1
        y += ss

    return waypoints


# ===========================================================================
# Pass 2 — Oblique perimeter passes
# ===========================================================================

def generate_oblique_passes(polygon, altitude, oblique_pitch, ground_alt):
    """
    Four passes around the property perimeter at `oblique_pitch`, facing inward.
    Order: North → East → South → West (clockwise to minimise repositioning).
    Standoff = altitude so the camera centre falls on the boundary at -45°.
    """
    clat, clon = _centroid(polygon)
    local = [_to_local(lat, lon, clat, clon) for lat, lon in polygon]

    min_x = min(p[0] for p in local)
    max_x = max(p[0] for p in local)
    min_y = min(p[1] for p in local)
    max_y = max(p[1] for p in local)

    standoff = altitude
    pi = _photo_interval(altitude, DEFAULT_FRONTAL_OVL)

    lx, ly = max_x - min_x, max_y - min_y
    nx = max(2, round(lx / pi) + 1)
    ny = max(2, round(ly / pi) + 1)

    def _pts(x0, x1, y0, y1, n):
        return [(x0 + (x1 - x0) * i / (n - 1),
                 y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]

    def _pass(pts, yaw_deg):
        return [_wp(*_to_latlon(x, y, clat, clon), altitude, ground_alt, yaw_deg, oblique_pitch)
                for x, y in pts]

    wps = []
    wps += _pass(_pts(min_x, max_x, max_y + standoff, max_y + standoff, nx), 180.0)  # N, face S
    wps += _pass(_pts(max_x + standoff, max_x + standoff, max_y, min_y, ny), 270.0)  # E, face W
    wps += _pass(_pts(max_x, min_x, min_y - standoff, min_y - standoff, nx), 0.0)    # S, face N
    wps += _pass(_pts(min_x - standoff, min_x - standoff, min_y, max_y, ny), 90.0)   # W, face E
    return wps


# ===========================================================================
# Summary
# ===========================================================================

def _write_summary(properties, output_dir, missions):
    sep = "─" * 72
    lines = [
        "KML Mission Builder — Oblique 3D Mapping Summary",
        f"Generated  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        sep,
        f"{'Property':<20}  {'Nadir WPs':>9}  {'Oblique WPs':>11}  {'Total WPs':>9}  KMZ",
        sep,
    ]
    for name, info in missions.items():
        lines.append(
            f"{name:<20}  {info['nadir']:>9}  {info['oblique']:>11}  "
            f"{info['total']:>9}  {Path(info['kmz']).name}"
        )
    lines += ["", sep, "PROPERTY DETAILS", sep]
    for name, prop in properties.items():
        clat, clon = _centroid(prop["polygon"])
        lines.append(f"  {name}")
        lines.append(f"    Centre     : {clat:.7f}, {clon:.7f}")
        lines.append(f"    Ground alt : {prop['ground_alt']:.1f} m MSL")
        lines.append(f"    Vertices   : {len(prop['polygon'])}")
    lines.append("")
    path = os.path.join(output_dir, "kml_missions_summary.txt")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    return path


# ===========================================================================
# Orchestration
# ===========================================================================

def run(kml_path, altitude, frontal_overlap, side_overlap, oblique_pitch,
        speed, gimbal_settle, dwell, rth_height, output_dir):

    print(f"[1/3] Parsing {kml_path!r}...")
    properties = parse_kml(kml_path)
    if not properties:
        print("ERROR: No polygon properties found in KML.")
        sys.exit(1)
    print(f"      Found {len(properties)} propert{'y' if len(properties)==1 else 'ies'}: "
          f"{', '.join(properties)}")

    w, h = _footprint(altitude)
    ss = _strip_spacing(altitude, side_overlap)
    pi = _photo_interval(altitude, frontal_overlap)
    print(f"      Footprint: {w:.1f} m × {h:.1f} m  |  "
          f"Strip spacing: {ss:.1f} m  |  Photo interval: {pi:.1f} m")

    os.makedirs(output_dir, exist_ok=True)
    missions: dict = {}

    print(f"[2/3] Generating missions...")
    for name, prop in properties.items():
        nadir_wps = generate_nadir_grid(
            prop["polygon"], altitude, frontal_overlap, side_overlap, prop["ground_alt"])
        oblique_wps = generate_oblique_passes(
            prop["polygon"], altitude, oblique_pitch, prop["ground_alt"])
        all_wps = nadir_wps + oblique_wps

        if len(all_wps) < 2:
            print(f"  WARNING: {name} — fewer than 2 waypoints, skipping.")
            continue

        waylines = build_waylines_wpml(all_wps, speed=speed, rth_height=rth_height,
                                       dwell=dwell, gimbal_settle=gimbal_settle)
        template = build_template_kml(all_wps, speed=speed, rth_height=rth_height)
        kmz_path = os.path.join(output_dir, f"{name}_3d_mission.kmz")
        pack_kmz(template, waylines, kmz_path)

        missions[name] = {
            "nadir": len(nadir_wps),
            "oblique": len(oblique_wps),
            "total": len(all_wps),
            "kmz": kmz_path,
        }
        print(f"  {name}: {len(nadir_wps)} nadir + {len(oblique_wps)} oblique "
              f"= {len(all_wps)} waypoints → {Path(kmz_path).name}")

    print("[3/3] Writing summary...")
    summary = _write_summary(properties, output_dir, missions)
    print(f"      → {summary}")

    print(f"\n{'='*62}")
    print(f"  {len(missions)} mission(s) ready in {output_dir}")
    print(f"  Altitude        : {altitude} m AGL (relativeToStartPoint)")
    fw, _ = _footprint(altitude)
    gsd_cm = fw / 5280.0 * 100.0  # footprint_width / M3E image width (px) → cm/px
    print(f"  GSD (approx)    : {gsd_cm:.1f} cm/pixel")
    print(f"  Gimbal nadir    : -90°   Gimbal oblique: {oblique_pitch}°")
    print(f"  RTH height      : {rth_height} m")
    print(f"\n  RealityCapture: import all KMZ photos into ONE project.")
    print(f"  Nadir + oblique together produce the best 3D geometry.")
    print(f"{'='*62}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Build DJI Pilot 2 oblique 3D mapping missions from a KML polygon file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--kml", required=True, help="KML file with named property polygons")
    parser.add_argument("--altitude", type=float, default=DEFAULT_ALTITUDE,
                        help="Flight altitude in metres AGL")
    parser.add_argument("--frontal-overlap", type=float, default=DEFAULT_FRONTAL_OVL,
                        help="Frontal overlap (0–1)")
    parser.add_argument("--side-overlap", type=float, default=DEFAULT_SIDE_OVL,
                        help="Side overlap (0–1)")
    parser.add_argument("--oblique-pitch", type=float, default=DEFAULT_OBLIQUE_PITCH,
                        help="Gimbal pitch for oblique passes in degrees")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                        help="Flight speed in m/s")
    parser.add_argument("--gimbal-settle", type=float, default=DEFAULT_GIMBAL_SETTLE,
                        help="Seconds for gimbal to settle before shutter")
    parser.add_argument("--dwell", type=float, default=DEFAULT_DWELL,
                        help="Seconds to hover after each photo")
    parser.add_argument("--rth-height", type=float, default=DEFAULT_RTH_HEIGHT,
                        help="Return-to-home altitude in metres")
    parser.add_argument("--output", default="./Missions",
                        help="Output directory for KMZ files")

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(0)

    args = parser.parse_args()
    run(
        kml_path=args.kml,
        altitude=args.altitude,
        frontal_overlap=args.frontal_overlap,
        side_overlap=args.side_overlap,
        oblique_pitch=args.oblique_pitch,
        speed=args.speed,
        gimbal_settle=args.gimbal_settle,
        dwell=args.dwell,
        rth_height=args.rth_height,
        output_dir=args.output,
    )


if __name__ == "__main__":
    main()
