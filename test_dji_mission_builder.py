"""
Tests for dji_mission_builder.py

Run with:  python -m pytest test_dji_mission_builder.py -v
"""

import io
import math
import zipfile
from datetime import datetime
from typing import Optional
from unittest.mock import MagicMock, patch
from xml.etree import ElementTree as ET

import pytest

# Module under test
import dji_mission_builder as dmb


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_wp(
    lat: float = 37.0,
    lon: float = -122.0,
    alt: float = 50.0,
    yaw: float = 90.0,
    pitch: float = -60.0,
    dt: Optional[datetime] = None,
) -> dict:
    return {
        "path": "/fake/img.jpg",
        "lat": lat,
        "lon": lon,
        "relative_altitude": alt,
        "absolute_altitude": alt + 10,
        "flight_yaw": yaw,
        "gimbal_pitch": pitch,
        "gimbal_yaw": 0.0,
        "rtk_flag": None,
        "datetime": dt,
    }


SAMPLE_XMP = """<x:xmpmeta xmlns:x="adobe:ns:meta/" xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
  xmlns:drone-dji="http://www.dji.com/drone-dji/1.0/">
  <rdf:RDF>
    <rdf:Description
      drone-dji:GpsLatitude="37.421998"
      drone-dji:GpsLongitude="-122.084058"
      drone-dji:RelativeAltitude="+45.20"
      drone-dji:AbsoluteAltitude="+120.50"
      drone-dji:FlightYawDegree="47.30"
      drone-dji:GimbalPitchDegree="-72.50"
      drone-dji:GimbalYawDegree="2.10"
      drone-dji:RtkFlag="1"
    />
  </rdf:RDF>
</x:xmpmeta>"""


# ===========================================================================
# haversine_meters
# ===========================================================================

class TestHaversineMeters:
    def test_zero_distance(self):
        assert dmb.haversine_meters(37.0, -122.0, 37.0, -122.0) == 0.0

    def test_known_distance(self):
        # Approx 111.32 km per degree of latitude at equator
        dist = dmb.haversine_meters(0.0, 0.0, 1.0, 0.0)
        assert abs(dist - 111_320) < 500

    def test_small_distance(self):
        # Two points ~1 metre apart
        dist = dmb.haversine_meters(37.0, -122.0, 37.000009, -122.0)
        assert 0.9 < dist < 1.1

    def test_symmetry(self):
        d1 = dmb.haversine_meters(37.0, -122.0, 38.0, -121.0)
        d2 = dmb.haversine_meters(38.0, -121.0, 37.0, -122.0)
        assert abs(d1 - d2) < 1e-6


# ===========================================================================
# _normalise_yaw
# ===========================================================================

class TestNormaliseYaw:
    def test_already_in_range(self):
        assert dmb._normalise_yaw(90.0) == 90.0

    def test_over_180(self):
        assert dmb._normalise_yaw(270.0) == -90.0

    def test_under_minus_180(self):
        assert dmb._normalise_yaw(-270.0) == 90.0

    def test_exactly_180(self):
        assert dmb._normalise_yaw(180.0) == 180.0

    def test_exactly_minus_180(self):
        assert dmb._normalise_yaw(-180.0) == -180.0


# ===========================================================================
# XMP extraction
# ===========================================================================

class TestXmpExtraction:
    def _parse(self, tag: str) -> Optional[str]:
        return dmb._xmp_value(SAMPLE_XMP, tag)

    def test_latitude(self):
        assert self._parse("GpsLatitude") == "37.421998"

    def test_longitude(self):
        assert self._parse("GpsLongitude") == "-122.084058"

    def test_relative_altitude(self):
        assert self._parse("RelativeAltitude") == "+45.20"

    def test_absolute_altitude(self):
        assert self._parse("AbsoluteAltitude") == "+120.50"

    def test_flight_yaw(self):
        assert self._parse("FlightYawDegree") == "47.30"

    def test_gimbal_pitch(self):
        assert self._parse("GimbalPitchDegree") == "-72.50"

    def test_rtk_flag(self):
        assert self._parse("RtkFlag") == "1"

    def test_missing_tag_returns_none(self):
        assert self._parse("NonExistentField") is None


# ===========================================================================
# extract_image_metadata (mocked file I/O)
# ===========================================================================

class TestExtractImageMetadata:
    def test_full_extraction(self):
        with patch.object(dmb, "_xmp_block", return_value=SAMPLE_XMP), \
             patch.object(dmb, "_datetime_original", return_value=datetime(2024, 3, 15, 10, 0, 0)):
            meta = dmb.extract_image_metadata("/fake/image.jpg")

        assert meta is not None
        assert abs(meta["lat"] - 37.421998) < 1e-6
        assert abs(meta["lon"] - -122.084058) < 1e-6
        assert abs(meta["relative_altitude"] - 45.20) < 0.01
        assert abs(meta["absolute_altitude"] - 120.50) < 0.01
        assert abs(meta["flight_yaw"] - 47.30) < 0.01
        assert abs(meta["gimbal_pitch"] - -72.50) < 0.01
        assert meta["rtk_flag"] == "1"
        assert meta["datetime"] == datetime(2024, 3, 15, 10, 0, 0)

    def test_missing_gps_returns_none(self):
        xmp_no_gps = SAMPLE_XMP.replace('drone-dji:GpsLatitude="37.421998"', "")
        with patch.object(dmb, "_xmp_block", return_value=xmp_no_gps), \
             patch.object(dmb, "_datetime_original", return_value=None):
            meta = dmb.extract_image_metadata("/fake/image.jpg")
        assert meta is None

    def test_no_xmp_returns_none(self):
        with patch.object(dmb, "_xmp_block", return_value=None):
            meta = dmb.extract_image_metadata("/fake/image.jpg")
        assert meta is None


# ===========================================================================
# build_waypoints
# ===========================================================================

class TestBuildWaypoints:
    def test_chronological_sort(self):
        wps = [
            _make_wp(lat=37.0, dt=datetime(2024, 1, 1, 12, 0, 0)),
            _make_wp(lat=37.1, dt=datetime(2024, 1, 1, 10, 0, 0)),
            _make_wp(lat=37.2, dt=datetime(2024, 1, 1, 11, 0, 0)),
        ]
        result = dmb.build_waypoints(wps, min_delta_m=0.0)
        lats = [w["lat"] for w in result]
        assert lats == [37.1, 37.2, 37.0]

    def test_deduplication_drops_close_points(self):
        # Three points: first two are 0.5 m apart (should drop second), third is far
        wps = [
            _make_wp(lat=37.000000, lon=-122.0, dt=datetime(2024, 1, 1, 10, 0, 0)),
            _make_wp(lat=37.000004, lon=-122.0, dt=datetime(2024, 1, 1, 10, 0, 1)),  # ~0.4 m
            _make_wp(lat=37.001000, lon=-122.0, dt=datetime(2024, 1, 1, 10, 0, 2)),  # ~111 m
        ]
        result = dmb.build_waypoints(wps, min_delta_m=1.0)
        assert len(result) == 2
        assert result[0]["lat"] == 37.000000
        assert result[1]["lat"] == 37.001000

    def test_no_timestamp_goes_last(self):
        wps = [
            _make_wp(lat=37.1, dt=None),
            _make_wp(lat=37.0, lon=-122.002, dt=datetime(2024, 1, 1, 10, 0, 0)),
        ]
        result = dmb.build_waypoints(wps, min_delta_m=0.0)
        assert result[0]["lat"] == 37.0
        assert result[1]["lat"] == 37.1

    def test_first_waypoint_always_kept(self):
        wps = [_make_wp(lat=37.0, dt=datetime(2024, 1, 1, 10, 0, 0))]
        result = dmb.build_waypoints(wps, min_delta_m=100.0)
        assert len(result) == 1

    def test_empty_input(self):
        assert dmb.build_waypoints([], min_delta_m=1.0) == []


# ===========================================================================
# build_waylines_wpml
# ===========================================================================

WPML_NS = dmb.WPML_NS
KML_NS = dmb.KML_NS


def _parse_waylines(wps: list) -> ET.Element:
    xml_str = dmb.build_waylines_wpml(wps, speed=5.0)
    return ET.fromstring(xml_str.split("?>", 1)[-1].strip())


class TestBuildWaylinesWpml:
    def setup_method(self):
        self.wps = [
            _make_wp(lat=37.0, lon=-122.0, alt=50.0, yaw=45.0, pitch=-60.0),
            _make_wp(lat=37.001, lon=-122.001, alt=55.0, yaw=90.0, pitch=-45.0),
        ]

    def test_xml_is_parseable(self):
        xml = dmb.build_waylines_wpml(self.wps, speed=5.0)
        assert xml.startswith("<?xml")
        ET.fromstring(xml.split("?>", 1)[-1].strip())  # must not raise

    def test_waypoint_count(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        placemarks = folder.findall(f"{{{KML_NS}}}Placemark")
        assert len(placemarks) == 2

    def test_coordinate_order_lon_lat(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        coords = pm.find(f"{{{KML_NS}}}Point/{{{KML_NS}}}coordinates").text.strip()
        lon_str, lat_str = coords.split(",")
        assert abs(float(lon_str) - (-122.0)) < 1e-6
        assert abs(float(lat_str) - 37.0) < 1e-6

    def test_execute_height_from_relative_altitude(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        height = pm.find(f"{{{WPML_NS}}}executeHeight").text
        assert abs(float(height) - 50.0) < 0.01

    def test_execute_height_mode_relative(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        mode = folder.find(f"{{{WPML_NS}}}executeHeightMode").text
        assert mode == "relativeToStartPoint"

    def test_heading_mode_smooth_transition(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        mode = pm.find(f"{{{WPML_NS}}}waypointHeadingParam/{{{WPML_NS}}}waypointHeadingMode").text
        assert mode == "smoothTransition"

    def test_turn_mode_stop_and_shoot(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        mode = pm.find(f"{{{WPML_NS}}}waypointTurnParam/{{{WPML_NS}}}waypointTurnMode").text
        assert mode == "toPointAndStopWithDiscontinuityCurvature"

    def test_action_group_present(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        ag = pm.find(f"{{{WPML_NS}}}actionGroup")
        assert ag is not None

    def test_gimbal_rotate_action(self):
        root = _parse_waylines(self.wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        ag = pm.find(f"{{{WPML_NS}}}actionGroup")
        actions = ag.findall(f"{{{WPML_NS}}}action")
        funcs = [a.find(f"{{{WPML_NS}}}actionActuatorFunc").text for a in actions]
        assert "gimbalRotate" in funcs
        assert "takePhoto" in funcs

    def test_gimbal_pitch_clamped_to_m3e_range(self):
        # pitch of -95 should be clamped to -90
        wps = [
            _make_wp(pitch=-95.0),
            _make_wp(lat=37.001, pitch=-45.0),
        ]
        root = _parse_waylines(wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        ag = pm.find(f"{{{WPML_NS}}}actionGroup")
        gimbal_action = next(
            a for a in ag.findall(f"{{{WPML_NS}}}action")
            if a.find(f"{{{WPML_NS}}}actionActuatorFunc").text == "gimbalRotate"
        )
        pitch_angle = float(
            gimbal_action.find(
                f"{{{WPML_NS}}}actionActuatorFuncParam/{{{WPML_NS}}}gimbalPitchRotateAngle"
            ).text
        )
        assert pitch_angle == -90.0

    def test_mission_config_drone_enum(self):
        root = _parse_waylines(self.wps)
        doc = root.find(f"{{{KML_NS}}}Document")
        enum = doc.find(f"{{{WPML_NS}}}missionConfig/{{{WPML_NS}}}droneInfo/{{{WPML_NS}}}droneEnumValue").text
        assert enum == "77"  # M3E

    def test_mission_config_has_rth_height(self):
        root = _parse_waylines(self.wps)
        doc = root.find(f"{{{KML_NS}}}Document")
        rth = doc.find(f"{{{WPML_NS}}}missionConfig/{{{WPML_NS}}}globalRTHHeight")
        assert rth is not None
        assert float(rth.text) > 0

    def test_action_group_ids_unique(self):
        wps = [_make_wp(lat=37.0 + i * 0.001) for i in range(5)]
        root = _parse_waylines(wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        ids = []
        for pm in folder.findall(f"{{{KML_NS}}}Placemark"):
            ag = pm.find(f"{{{WPML_NS}}}actionGroup")
            ids.append(ag.find(f"{{{WPML_NS}}}actionGroupId").text)
        assert len(ids) == len(set(ids))

    def test_waypoint_indices_sequential(self):
        wps = [_make_wp(lat=37.0 + i * 0.001) for i in range(4)]
        root = _parse_waylines(wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        indices = [
            int(pm.find(f"{{{WPML_NS}}}index").text)
            for pm in folder.findall(f"{{{KML_NS}}}Placemark")
        ]
        assert indices == list(range(4))

    def test_yaw_normalised(self):
        # yaw=270 → should appear as -90 in the XML
        wps = [_make_wp(yaw=270.0), _make_wp(lat=37.001, yaw=270.0)]
        root = _parse_waylines(wps)
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        angle = float(
            pm.find(f"{{{WPML_NS}}}waypointHeadingParam/{{{WPML_NS}}}waypointHeadingAngle").text
        )
        assert angle == -90.0


# ===========================================================================
# build_template_kml
# ===========================================================================

class TestBuildTemplateKml:
    def setup_method(self):
        self.wps = [
            _make_wp(lat=37.0, lon=-122.0, alt=50.0, yaw=45.0, pitch=-60.0),
            _make_wp(lat=37.001, lon=-122.001, alt=55.0, yaw=90.0, pitch=-45.0),
        ]

    def _root(self):
        xml = dmb.build_template_kml(self.wps, speed=5.0)
        return ET.fromstring(xml.split("?>", 1)[-1].strip())

    def test_xml_is_parseable(self):
        xml = dmb.build_template_kml(self.wps, speed=5.0)
        ET.fromstring(xml.split("?>", 1)[-1].strip())

    def test_template_id_is_zero(self):
        root = self._root()
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        tid = folder.find(f"{{{WPML_NS}}}templateId").text
        assert tid == "0"

    def test_height_mode_relative(self):
        root = self._root()
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        mode = folder.find(
            f"{{{WPML_NS}}}waylineCoordinateSysParam/{{{WPML_NS}}}heightMode"
        ).text
        assert mode == "relativeToStartPoint"

    def test_waypoint_count(self):
        root = self._root()
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pms = folder.findall(f"{{{KML_NS}}}Placemark")
        assert len(pms) == 2

    def test_coordinate_order_lon_lat(self):
        root = self._root()
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        coords = pm.find(f"{{{KML_NS}}}Point/{{{KML_NS}}}coordinates").text.strip()
        lon_str, lat_str = coords.split(",")
        assert abs(float(lon_str) - (-122.0)) < 1e-6

    def test_gimbal_pitch_in_template(self):
        root = self._root()
        folder = root.find(f"{{{KML_NS}}}Document/{{{KML_NS}}}Folder")
        pm = folder.findall(f"{{{KML_NS}}}Placemark")[0]
        pitch = pm.find(f"{{{WPML_NS}}}gimbalPitchAngle").text
        assert abs(float(pitch) - (-60.0)) < 0.1


# ===========================================================================
# pack_kmz
# ===========================================================================

class TestPackKmz:
    def test_zip_structure(self, tmp_path):
        out = str(tmp_path / "test.kmz")
        dmb.pack_kmz("<template/>", "<waylines/>", out)
        with zipfile.ZipFile(out, "r") as zf:
            names = zf.namelist()
        assert "wpmz/template.kml" in names
        assert "wpmz/waylines.wpml" in names

    def test_content_preserved(self, tmp_path):
        out = str(tmp_path / "test.kmz")
        dmb.pack_kmz("TEMPLATE_CONTENT", "WAYLINES_CONTENT", out)
        with zipfile.ZipFile(out, "r") as zf:
            assert zf.read("wpmz/template.kml") == "TEMPLATE_CONTENT".encode("utf-8")
            assert zf.read("wpmz/waylines.wpml") == "WAYLINES_CONTENT".encode("utf-8")

    def test_only_two_files_in_archive(self, tmp_path):
        out = str(tmp_path / "test.kmz")
        dmb.pack_kmz("<t/>", "<w/>", out)
        with zipfile.ZipFile(out, "r") as zf:
            assert len(zf.namelist()) == 2
