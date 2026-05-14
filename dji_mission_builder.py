#!/usr/bin/env python3
"""
dji_mission_builder.py

Converts a folder of DJI M3E images into a DJI Pilot 2 KMZ waypoint mission,
replaying the original flight as a repeatable autonomous route.

Pipeline:
  1. Extract GPS, altitude, heading, and gimbal angles from each image's DJI XMP
  2. Sort chronologically; skip images < min_delta metres from the previous waypoint
  3. Generate waylines.wpml  — one Placemark per image with gimbalRotate + takePhoto
  4. Generate template.kml   — planning template for DJI Pilot 2 / M3E
  5. Pack both into a .kmz (ZIP archive)

Usage:
  python dji_mission_builder.py --images /path/to/images [--speed 5] [--output ./] [--min-delta 1.0]
"""

import argparse
import math
import os
import sys
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Optional
from xml.dom import minidom
from xml.etree import ElementTree as ET

# ---------------------------------------------------------------------------
# Integrate with existing Pole.IO XMP extraction (extends, does not replace)
# ---------------------------------------------------------------------------
_POLEIO_SRC = Path(
    "/Users/tobydz/Documents/Documents - Tobys MacBook Pro/_CODE/Pole.IO/SRC"
)
if _POLEIO_SRC.exists() and str(_POLEIO_SRC) not in sys.path:
    sys.path.insert(0, str(_POLEIO_SRC))

try:
    from GetEXIFTags import extract_xmp_block as _pole_xmp_block
    from GetEXIFTags import extract_value_from_xmp as _pole_xmp_value
    _POLEIO_AVAILABLE = True
except ImportError:
    _POLEIO_AVAILABLE = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
DJI_NS = "http://www.dji.com/drone-dji/1.0/"
WPML_NS = "http://www.dji.com/wpmz/1.0.2"
KML_NS = "http://www.opengis.net/kml/2.2"

# M3E hardware identifiers (DJI Cloud API WPML spec, droneEnumValue table)
DRONE_ENUM_M3E = "77"
DRONE_SUB_ENUM_M3E = "0"   # 0=M3E, 1=M3T, 2=M3M
PAYLOAD_ENUM_M3E = "66"    # Mavic 3E wide+zoom camera

# DJI XMP fields this tool needs beyond what Pole.IO already reads
_EXTRA_XMP_FIELDS = [
    "RelativeAltitude",
    "AbsoluteAltitude",
    "FlightYawDegree",
    "GimbalYawDegree",
    "RtkFlag",
    # GimbalPitchDegree, GpsLatitude, GpsLongitude already in Pole.IO
]


# ===========================================================================
# STEP 1 — Metadata extraction
# ===========================================================================

def _xmp_block(image_path: str) -> Optional[str]:
    """Return the raw XMP XML string from a DJI image (JPG or DNG)."""
    if _POLEIO_AVAILABLE:
        try:
            result = _pole_xmp_block(image_path)
            if result:
                return result
        except Exception:
            pass
    # Fallback: binary scan — works for both JPG and DNG/TIFF embeds
    try:
        with open(image_path, "rb") as fh:
            data = fh.read()
        start = data.find(b"<x:xmpmeta")
        end = data.find(b"</x:xmpmeta>")
        if start != -1 and end != -1:
            return data[start : end + len(b"</x:xmpmeta>")].decode("utf-8", errors="replace")
    except OSError:
        pass
    return None


def _xmp_value(xmp_block: str, tag: str) -> Optional[str]:
    """
    Extract a single field from the DJI drone-dji XMP namespace.
    Delegates to Pole.IO when available, falls back to inline parsing.
    """
    if _POLEIO_AVAILABLE:
        try:
            val = _pole_xmp_value(xmp_block, tag, lambda *_: None)
            if val is not None:
                return str(val)
        except Exception:
            pass
    # Inline fallback: scan rdf:Description attributes
    try:
        root = ET.fromstring(xmp_block)
        ns_tag = f"{{{DJI_NS}}}{tag}"
        for elem in root.iter():
            if ns_tag in elem.attrib:
                return elem.attrib[ns_tag]
            if elem.tag == ns_tag and elem.text:
                return elem.text.strip()
    except ET.ParseError:
        pass
    return None


def _datetime_original(image_path: str) -> Optional[datetime]:
    """Read DateTimeOriginal (EXIF tag 36867). Falls back to file mtime."""
    try:
        from PIL import Image  # type: ignore
        with Image.open(image_path) as img:
            exif = img._getexif()  # type: ignore[attr-defined]
            if exif:
                raw = exif.get(36867)
                if raw:
                    return datetime.strptime(raw, "%Y:%m:%d %H:%M:%S")
    except Exception:
        pass
    try:
        return datetime.fromtimestamp(os.path.getmtime(image_path))
    except OSError:
        return None


def extract_image_metadata(image_path: str) -> Optional[dict]:
    """
    Extract all mission-relevant fields from a single DJI M3E image.
    Returns None if GPS coordinates are missing (image cannot become a waypoint).

    Returned dict keys:
      path, lat, lon, relative_altitude, absolute_altitude,
      flight_yaw, gimbal_pitch, gimbal_yaw, rtk_flag, datetime
    """
    xmp = _xmp_block(image_path)
    if not xmp:
        return None

    def _f(tag: str, default: float = 0.0) -> float:
        try:
            return float(_xmp_value(xmp, tag) or default)
        except (TypeError, ValueError):
            return default

    lat_raw = _xmp_value(xmp, "GpsLatitude")
    lon_raw = _xmp_value(xmp, "GpsLongitude")
    try:
        lat = float(lat_raw)   # type: ignore[arg-type]
        lon = float(lon_raw)   # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None

    return {
        "path": image_path,
        "lat": lat,
        "lon": lon,
        "relative_altitude": _f("RelativeAltitude", 30.0),
        "absolute_altitude": _f("AbsoluteAltitude", 0.0),
        "flight_yaw": _f("FlightYawDegree", 0.0),
        "gimbal_pitch": _f("GimbalPitchDegree", -90.0),
        "gimbal_yaw": _f("GimbalYawDegree", 0.0),
        "rtk_flag": _xmp_value(xmp, "RtkFlag"),
        "datetime": _datetime_original(image_path),
    }


def collect_images(folder: str) -> list:
    """Recursively collect all JPG and DNG files under folder, sorted by path."""
    exts = {".jpg", ".jpeg", ".dng"}
    paths = []
    for root, _, files in os.walk(folder):
        for f in files:
            if Path(f).suffix.lower() in exts:
                paths.append(os.path.join(root, f))
    return sorted(paths)


# ===========================================================================
# Shared utility
# ===========================================================================

def haversine_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two WGS84 coordinates."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _normalise_yaw(yaw: float) -> float:
    """Wrap yaw into [-180, 180] as required by waypointHeadingAngle."""
    while yaw > 180:
        yaw -= 360
    while yaw < -180:
        yaw += 360
    return yaw


# ===========================================================================
# STEP 2 — Sort and deduplicate waypoints
# ===========================================================================

def build_waypoints(metadata_list: list, min_delta_m: float = 1.0) -> list:
    """
    Sort images by DateTimeOriginal (images without a timestamp go last),
    then drop any image whose position is < min_delta_m from the previous kept image.
    Burst-shoot duplicates and hover shots collapse naturally.
    """
    with_ts = sorted(
        (m for m in metadata_list if m["datetime"] is not None),
        key=lambda m: m["datetime"],
    )
    without_ts = [m for m in metadata_list if m["datetime"] is None]
    ordered = with_ts + without_ts

    waypoints: list = []
    for entry in ordered:
        if not waypoints:
            waypoints.append(entry)
            continue
        prev = waypoints[-1]
        if haversine_meters(prev["lat"], prev["lon"], entry["lat"], entry["lon"]) >= min_delta_m:
            waypoints.append(entry)
    return waypoints


# ===========================================================================
# STEP 3 — XML generation helpers
# ===========================================================================

def _pretty(xml_string: str) -> str:
    """Pretty-print XML, returning UTF-8 text with a proper XML declaration."""
    dom = minidom.parseString(xml_string.encode("utf-8"))
    pretty = dom.toprettyxml(indent="  ", encoding=None)
    # minidom inserts its own declaration; replace with the one we want
    lines = pretty.splitlines()
    if lines and lines[0].startswith("<?xml"):
        lines = lines[1:]
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + "\n".join(lines)


def _sub(parent: ET.Element, tag: str, text: Optional[str] = None) -> ET.Element:
    """Create a child element, optionally setting its text."""
    el = ET.SubElement(parent, tag)
    if text is not None:
        el.text = text
    return el


def _mission_config_block(
    parent: ET.Element,
    transitional_speed: float = 10.0,
    rth_height: float = 100.0,
) -> None:
    """Append a complete <wpml:missionConfig> block (required in both files)."""
    W = f"{{{WPML_NS}}}"
    cfg = _sub(parent, f"{W}missionConfig")
    _sub(cfg, f"{W}flyToWaylineMode", "safely")
    _sub(cfg, f"{W}finishAction", "goHome")
    _sub(cfg, f"{W}exitOnRCLost", "goContinue")
    _sub(cfg, f"{W}executeRCLostAction", "hover")
    _sub(cfg, f"{W}takeOffSecurityHeight", "20")
    _sub(cfg, f"{W}globalTransitionalSpeed", str(round(transitional_speed, 1)))
    # globalRTHHeight is REQUIRED in waylines.wpml; include in template too for safety
    _sub(cfg, f"{W}globalRTHHeight", str(round(rth_height, 1)))
    drone = _sub(cfg, f"{W}droneInfo")
    _sub(drone, f"{W}droneEnumValue", DRONE_ENUM_M3E)
    _sub(drone, f"{W}droneSubEnumValue", DRONE_SUB_ENUM_M3E)
    payload = _sub(cfg, f"{W}payloadInfo")
    _sub(payload, f"{W}payloadEnumValue", PAYLOAD_ENUM_M3E)
    _sub(payload, f"{W}payloadPositionIndex", "0")


def _action_group_block(
    parent: ET.Element,
    wp_index: int,
    group_id: int,
    gimbal_pitch: float,
    gimbal_settle: float = 3.0,
) -> None:
    """
    Append a <wpml:actionGroup> with:
      Action 0: gimbalRotate  (absoluteAngle, pitch from original image)
      Action 1: takePhoto     (wide lens)

    actionGroupId must be unique across the entire KMZ — using wp_index guarantees this
    for single-wayline missions.
    """
    W = f"{{{WPML_NS}}}"
    ag = _sub(parent, f"{W}actionGroup")
    _sub(ag, f"{W}actionGroupId", str(group_id))
    _sub(ag, f"{W}actionGroupStartIndex", str(wp_index))
    _sub(ag, f"{W}actionGroupEndIndex", str(wp_index))
    _sub(ag, f"{W}actionGroupMode", "sequence")
    trigger = _sub(ag, f"{W}actionTrigger")
    _sub(trigger, f"{W}actionTriggerType", "reachPoint")

    # --- gimbalRotate ---
    a0 = _sub(ag, f"{W}action")
    _sub(a0, f"{W}actionId", "0")
    _sub(a0, f"{W}actionActuatorFunc", "gimbalRotate")
    p0 = _sub(a0, f"{W}actionActuatorFuncParam")
    # gimbalHeadingYawBase=north means yaw reference is geographic north
    _sub(p0, f"{W}gimbalHeadingYawBase", "north")
    # absoluteAngle is the only value defined in the WPML spec for gimbalRotateMode
    _sub(p0, f"{W}gimbalRotateMode", "absoluteAngle")
    _sub(p0, f"{W}gimbalPitchRotateEnable", "1")
    # M3E pitch range: [-90, 35] degrees — clamp to be safe
    clamped = max(-90.0, min(35.0, gimbal_pitch))
    _sub(p0, f"{W}gimbalPitchRotateAngle", str(round(clamped, 1)))
    _sub(p0, f"{W}gimbalRollRotateEnable", "0")
    _sub(p0, f"{W}gimbalRollRotateAngle", "0")
    _sub(p0, f"{W}gimbalYawRotateEnable", "0")
    _sub(p0, f"{W}gimbalYawRotateAngle", "0")
    _sub(p0, f"{W}gimbalRotateTimeEnable", "1")
    _sub(p0, f"{W}gimbalRotateTime", str(round(gimbal_settle, 1)))
    _sub(p0, f"{W}payloadPositionIndex", "0")

    # --- takePhoto ---
    a1 = _sub(ag, f"{W}action")
    _sub(a1, f"{W}actionId", "1")
    _sub(a1, f"{W}actionActuatorFunc", "takePhoto")
    p1 = _sub(a1, f"{W}actionActuatorFuncParam")
    _sub(p1, f"{W}fileSuffix", f"wp{wp_index}")
    _sub(p1, f"{W}payloadPositionIndex", "0")
    _sub(p1, f"{W}useGlobalPayloadLensIndex", "0")
    _sub(p1, f"{W}payloadLensIndex", "wide")


# ===========================================================================
# STEP 3a — waylines.wpml
# ===========================================================================

def build_waylines_wpml(
    waypoints: list,
    speed: float = 5.0,
    rth_height: float = 100.0,
    dwell: float = 2.0,
    gimbal_settle: float = 3.0,
) -> str:
    """
    Generate the executable waylines.wpml XML for the given waypoint list.

    Each Placemark gets:
      - executeHeightMode: relativeToStartPoint  (safe for non-RTK missions)
      - executeHeight: RelativeAltitude from original image
      - waypointHeadingMode: smoothTransition with FlightYawDegree
      - waypointTurnMode: toPointAndStopWithDiscontinuityCurvature  (stop and shoot)
      - actionGroup: gimbalRotate → takePhoto
    """
    ET.register_namespace("", KML_NS)
    ET.register_namespace("wpml", WPML_NS)
    W = f"{{{WPML_NS}}}"
    K = f"{{{KML_NS}}}"

    kml = ET.Element(f"{K}kml")
    doc = _sub(kml, f"{K}Document")

    _mission_config_block(
        doc,
        transitional_speed=min(speed * 2, 10.0),
        rth_height=rth_height,
    )

    folder = _sub(doc, f"{K}Folder")
    _sub(folder, f"{W}templateId", "0")
    _sub(folder, f"{W}waylineId", "0")
    # relativeToStartPoint: executeHeight = metres above takeoff point
    # This directly uses RelativeAltitude from DJI XMP — same reference frame
    _sub(folder, f"{W}executeHeightMode", "relativeToStartPoint")
    _sub(folder, f"{W}autoFlightSpeed", str(round(speed, 1)))

    for i, wp in enumerate(waypoints):
        pm = _sub(folder, f"{K}Placemark")
        pt = _sub(pm, f"{K}Point")
        # WPML coordinate order: longitude,latitude  (NOT lat/lon)
        _sub(pt, f"{K}coordinates", f"{wp['lon']},{wp['lat']}")

        _sub(pm, f"{W}index", str(i))
        _sub(pm, f"{W}executeHeight", str(round(wp["relative_altitude"], 2)))
        _sub(pm, f"{W}waypointSpeed", str(round(speed, 1)))

        hdg = _sub(pm, f"{W}waypointHeadingParam")
        # smoothTransition: drone yaw transitions between adjacent waypoints' angles
        _sub(hdg, f"{W}waypointHeadingMode", "smoothTransition")
        _sub(hdg, f"{W}waypointHeadingAngle", str(round(_normalise_yaw(wp["flight_yaw"]), 1)))
        # followBadArc: rotate along the shortest arc (avoids 359°→1° going the long way)
        _sub(hdg, f"{W}waypointHeadingPathMode", "followBadArc")

        turn = _sub(pm, f"{W}waypointTurnParam")
        _sub(turn, f"{W}waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
        _sub(turn, f"{W}waypointTurnDampingDist", "0")

        _sub(pm, f"{W}useStraightLine", "1")
        _sub(pm, f"{W}stayTime", str(round(dwell, 1)))

        _action_group_block(pm, wp_index=i, group_id=i, gimbal_pitch=wp["gimbal_pitch"], gimbal_settle=gimbal_settle)

    return _pretty(ET.tostring(kml, encoding="unicode"))


# ===========================================================================
# STEP 3b — template.kml
# ===========================================================================

def build_template_kml(
    waypoints: list,
    speed: float = 5.0,
    rth_height: float = 100.0,
) -> str:
    """
    Generate the planning template.kml for DJI Pilot 2.
    templateId=0 must match the waylineId=0 in waylines.wpml.
    """
    ET.register_namespace("", KML_NS)
    ET.register_namespace("wpml", WPML_NS)
    W = f"{{{WPML_NS}}}"
    K = f"{{{KML_NS}}}"
    now_ms = str(int(datetime.now().timestamp() * 1000))

    kml = ET.Element(f"{K}kml")
    doc = _sub(kml, f"{K}Document")

    _sub(doc, f"{W}author", "dji_mission_builder")
    _sub(doc, f"{W}createTime", now_ms)
    _sub(doc, f"{W}updateTime", now_ms)

    _mission_config_block(
        doc,
        transitional_speed=min(speed * 2, 10.0),
        rth_height=rth_height,
    )

    folder = _sub(doc, f"{K}Folder")
    _sub(folder, f"{W}templateType", "waypoint")
    # templateId links this folder to the Folder in waylines.wpml
    _sub(folder, f"{W}templateId", "0")

    coord_sys = _sub(folder, f"{W}waylineCoordinateSysParam")
    _sub(coord_sys, f"{W}coordinateMode", "WGS84")
    _sub(coord_sys, f"{W}heightMode", "relativeToStartPoint")
    _sub(coord_sys, f"{W}positioningType", "GPS")

    _sub(folder, f"{W}autoFlightSpeed", str(round(speed, 1)))
    _sub(folder, f"{W}gimbalPitchMode", "usePointSetting")

    g_hdg = _sub(folder, f"{W}globalWaypointHeadingParam")
    _sub(g_hdg, f"{W}waypointHeadingMode", "smoothTransition")
    _sub(g_hdg, f"{W}waypointHeadingAngle", "0")
    _sub(g_hdg, f"{W}waypointHeadingPathMode", "followBadArc")

    _sub(folder, f"{W}globalWaypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
    _sub(folder, f"{W}globalUseStraightLine", "1")

    for i, wp in enumerate(waypoints):
        pm = _sub(folder, f"{K}Placemark")
        pt = _sub(pm, f"{K}Point")
        _sub(pt, f"{K}coordinates", f"{wp['lon']},{wp['lat']}")

        _sub(pm, f"{W}index", str(i))
        h = str(round(wp["relative_altitude"], 2))
        _sub(pm, f"{W}ellipsoidHeight", h)
        _sub(pm, f"{W}height", h)
        _sub(pm, f"{W}useGlobalHeight", "1")
        _sub(pm, f"{W}useGlobalSpeed", "1")
        # Per-point heading overrides the global (useGlobalHeadingParam=0)
        _sub(pm, f"{W}useGlobalHeadingParam", "0")
        _sub(pm, f"{W}useGlobalTurnParam", "1")
        _sub(pm, f"{W}gimbalPitchAngle", str(round(wp["gimbal_pitch"], 1)))

        hdg = _sub(pm, f"{W}waypointHeadingParam")
        _sub(hdg, f"{W}waypointHeadingMode", "smoothTransition")
        _sub(hdg, f"{W}waypointHeadingAngle", str(round(_normalise_yaw(wp["flight_yaw"]), 1)))
        _sub(hdg, f"{W}waypointHeadingPathMode", "followBadArc")

    return _pretty(ET.tostring(kml, encoding="unicode"))


# ===========================================================================
# STEP 4 — Pack KMZ
# ===========================================================================

def pack_kmz(template_kml: str, waylines_wpml: str, output_path: str) -> None:
    """
    Create a DJI Pilot 2-compatible KMZ archive.
    Internal structure must be exactly: wpmz/template.kml, wpmz/waylines.wpml
    """
    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("wpmz/template.kml", template_kml.encode("utf-8"))
        zf.writestr("wpmz/waylines.wpml", waylines_wpml.encode("utf-8"))


# ===========================================================================
# STEP 5 — Summary file
# ===========================================================================

def write_summary(
    images_folder: str,
    all_metadata: list,
    waypoints: list,
    kmz_path: str,
    rtk_active: bool,
    skipped: int,
    output_dir: str,
) -> str:
    """
    Write a human-readable .txt summary of extracted metadata and waypoints.
    Returns the path of the written file.
    """
    folder_name = Path(images_folder).resolve().name
    summary_path = os.path.join(output_dir, f"{folder_name}_summary.txt")

    lines: list = []
    sep = "─" * 72
    lines.append("DJI Mission Builder — Extraction Summary")
    lines.append(f"Generated    : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"Source folder: {Path(images_folder).resolve()}")
    lines.append(f"Mission file : {Path(kmz_path).resolve()}")
    lines.append("")
    lines.append(sep)
    lines.append("STATISTICS")
    lines.append(sep)
    lines.append(f"  Images scanned         : {len(all_metadata) + skipped}")
    lines.append(f"  Metadata extracted     : {len(all_metadata)}")
    lines.append(f"  Skipped (no GPS/XMP)   : {skipped}")
    lines.append(f"  RTK status             : {'active' if rtk_active else 'not active'}")
    lines.append(f"  Waypoints (post-dedup) : {len(waypoints)}")
    lines.append("")
    lines.append(sep)
    lines.append("ALL EXTRACTED IMAGES (chronological order)")
    lines.append(sep)

    # Column header
    hdr = (
        f"{'#':>4}  {'Filename':<36}  {'DateTime':<19}  "
        f"{'Lat':>12}  {'Lon':>13}  {'RelAlt':>7}  "
        f"{'AbsAlt':>7}  {'Yaw':>7}  {'GPitch':>7}  {'GYaw':>7}  {'RTK'}"
    )
    lines.append(hdr)
    lines.append("─" * len(hdr))

    all_sorted = sorted(
        all_metadata,
        key=lambda m: m["datetime"] or datetime.min,
    )
    for idx, m in enumerate(all_sorted):
        dt_str = m["datetime"].strftime("%Y-%m-%d %H:%M:%S") if m["datetime"] else "—"
        fname = Path(m["path"]).name
        rtk = m.get("rtk_flag") or "—"
        lines.append(
            f"{idx:>4}  {fname:<36}  {dt_str:<19}  "
            f"{m['lat']:>12.7f}  {m['lon']:>13.7f}  {m['relative_altitude']:>7.2f}  "
            f"{m['absolute_altitude']:>7.2f}  {m['flight_yaw']:>7.1f}  "
            f"{m['gimbal_pitch']:>7.1f}  {m['gimbal_yaw']:>7.1f}  {rtk}"
        )

    lines.append("")
    lines.append(sep)
    lines.append("WAYPOINTS INCLUDED IN MISSION (after deduplication)")
    lines.append(sep)
    lines.append(hdr)
    lines.append("─" * len(hdr))

    for idx, m in enumerate(waypoints):
        dt_str = m["datetime"].strftime("%Y-%m-%d %H:%M:%S") if m["datetime"] else "—"
        fname = Path(m["path"]).name
        rtk = m.get("rtk_flag") or "—"
        lines.append(
            f"{idx:>4}  {fname:<36}  {dt_str:<19}  "
            f"{m['lat']:>12.7f}  {m['lon']:>13.7f}  {m['relative_altitude']:>7.2f}  "
            f"{m['absolute_altitude']:>7.2f}  {m['flight_yaw']:>7.1f}  "
            f"{m['gimbal_pitch']:>7.1f}  {m['gimbal_yaw']:>7.1f}  {rtk}"
        )

    lines.append("")

    with open(summary_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    return summary_path


# ===========================================================================
# STEP 6 — Deployment instructions
# ===========================================================================

def print_deployment_instructions(
    kmz_path: str,
    waypoint_count: int,
    rtk_active: bool,
) -> None:
    print("\n" + "=" * 62)
    print("  DEPLOYMENT — DJI RC Pro Enterprise / DJI Pilot 2")
    print("=" * 62)
    print(f"\n  Mission file : {kmz_path}")
    print(f"  Waypoints    : {waypoint_count}")
    if rtk_active:
        print("  RTK          : Active during capture — altitude data is reliable.")
    else:
        print("  RTK          : NOT active during capture.")
        print("                 Mission uses relativeToStartPoint altitudes.")
        print("                 IMPORTANT: take off from the same GPS location")
        print("                 as the original flight for altitude to match.")

    print("""
  To import on RC Pro Enterprise
  ─────────────────────────────
  1. Copy the .kmz to a microSD card
  2. Insert the microSD into the RC Pro Enterprise
  3. Open DJI Pilot 2 → Flight Route → "+" → Import Route (KMZ/KML)
  4. Browse to the microSD card and tap the .kmz file

  Pre-flight checklist
  ────────────────────
  □  Take off from the same location as the original flight
     (critical for relativeToStartPoint altitude accuracy)
  □  Confirm obstacle avoidance is enabled
  □  Review the route in Pilot 2 map view before executing

  Firmware note
  ─────────────
  If import fails, ensure firmware is past M3E v07.01.10.xx.
  On older firmware: create and save one dummy mission in Pilot 2
  first, then retry the import.
""")


# ===========================================================================
# Orchestration
# ===========================================================================

def run(
    images_folder: str,
    speed: float = 5.0,
    output_dir: str = "./",
    min_delta: float = 1.0,
    rth_height: float = 100.0,
    dwell: float = 2.0,
    gimbal_settle: float = 3.0,
) -> None:
    print(f"[1/5] Scanning {images_folder!r} for DJI images...")
    image_files = collect_images(images_folder)
    if not image_files:
        print("ERROR: No JPG or DNG files found in the specified folder.")
        sys.exit(1)
    print(f"      Found {len(image_files)} file(s).")

    print("[2/5] Extracting metadata...")
    metadata_list: list = []
    rtk_count = 0
    skipped = 0
    for path in image_files:
        meta = extract_image_metadata(path)
        if meta is None:
            skipped += 1
            continue
        metadata_list.append(meta)
        flag = meta.get("rtk_flag") or ""
        if flag not in ("", "0"):
            rtk_count += 1

    if not metadata_list:
        print("ERROR: Could not extract GPS metadata from any image.")
        sys.exit(1)

    rtk_active = rtk_count > len(metadata_list) * 0.5
    print(f"      Extracted: {len(metadata_list)}  |  Skipped (no GPS): {skipped}")
    print(f"      RTK: {'active' if rtk_active else 'not active'}")

    print(f"[3/5] Building waypoints (min_delta={min_delta} m)...")
    waypoints = build_waypoints(metadata_list, min_delta_m=min_delta)
    print(f"      {len(waypoints)} waypoints after deduplication.")

    if len(waypoints) < 2:
        print("ERROR: Need at least 2 waypoints for a valid mission.")
        sys.exit(1)

    print("[4/5] Generating WPML...")
    waylines = build_waylines_wpml(waypoints, speed=speed, rth_height=rth_height, dwell=dwell, gimbal_settle=gimbal_settle)
    template = build_template_kml(waypoints, speed=speed, rth_height=rth_height)

    folder_name = Path(images_folder).resolve().name
    output_path = os.path.join(output_dir, f"{folder_name}_mission.kmz")

    print(f"[5/5] Packing KMZ → {output_path}")
    pack_kmz(template, waylines, output_path)

    summary_path = write_summary(
        images_folder=images_folder,
        all_metadata=metadata_list,
        waypoints=waypoints,
        kmz_path=output_path,
        rtk_active=rtk_active,
        skipped=skipped,
        output_dir=output_dir,
    )
    print(f"      Summary    → {summary_path}")

    print_deployment_instructions(output_path, len(waypoints), rtk_active)
    print(f"Done — {len(waypoints)} waypoints written to {output_path}\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build a DJI Pilot 2 KMZ waypoint mission from M3E image metadata.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--images", required=True,
        help="Folder containing DJI M3E images (JPG / DNG)",
    )
    parser.add_argument(
        "--speed", type=float, default=5.0,
        help="Waypoint flight speed in m/s",
    )
    parser.add_argument(
        "--output", default="./",
        help="Output directory for the .kmz file",
    )
    parser.add_argument(
        "--min-delta", type=float, default=1.0,
        help="Minimum metres between consecutive waypoints (deduplication threshold)",
    )
    parser.add_argument(
        "--rth-height", type=float, default=100.0,
        help="Return-to-home altitude in metres",
    )
    parser.add_argument(
        "--dwell", type=float, default=2.0,
        help="Seconds drone hovers at each waypoint after actions complete",
    )
    parser.add_argument(
        "--gimbal-settle", type=float, default=3.0,
        help="Seconds allocated for gimbal to reach target angle before shutter fires",
    )
    args = parser.parse_args()
    run(
        images_folder=args.images,
        speed=args.speed,
        output_dir=args.output,
        min_delta=args.min_delta,
        rth_height=args.rth_height,
        dwell=args.dwell,
        gimbal_settle=args.gimbal_settle,
    )


if __name__ == "__main__":
    main()
