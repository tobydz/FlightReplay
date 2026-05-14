# FlightReplay

Converts a folder of DJI Mavic 3 Enterprise (M3E) images into a DJI Pilot 2 waypoint mission (.kmz) that can be autonomously re-flown. Point it at a folder of images from a previous flight and it reverse-engineers the route — GPS positions, altitudes, headings, and gimbal angles — into a mission file ready to import on your RC Pro Enterprise.

---

## Download

Go to [Releases](../../releases) and download the binary for your platform:

| Platform | File |
|---|---|
| Mac (M1/M2/M3) | `dji_mission_builder-mac-arm64` |
| Mac (Intel) | `dji_mission_builder-mac-intel` |
| Windows | `dji_mission_builder-windows.exe` |
| Linux | `dji_mission_builder-linux` |

**Mac users — first run only:** macOS will block the binary with "developer cannot be verified." Right-click → Open → Open anyway. It won't ask again after that.

---

## Usage

```bash
# Basic — processes all JPG/DNG images in a folder
./dji_mission_builder --images /path/to/images

# Full options
./dji_mission_builder \
  --images   /path/to/images \   # folder of M3E JPG or DNG images (required)
  --output   ./Missions      \   # where to save the .kmz and summary (default: ./)
  --speed    5.0             \   # flight speed in m/s (default: 5.0)
  --min-delta 1.0            \   # min metres between waypoints, filters burst shots (default: 1.0)
  --rth-height 100.0         \   # return-to-home altitude in metres (default: 100.0)
  --gimbal-settle 3.0        \   # seconds to wait for gimbal before firing shutter (default: 3.0)
  --dwell 2.0                    # seconds to hover after each photo before moving on (default: 2.0)
```

### Output

Two files are written to `--output` for each run:

- **`FolderName_mission.kmz`** — import this into DJI Pilot 2
- **`FolderName_summary.txt`** — full table of extracted metadata for every image (lat, lon, altitude, heading, gimbal angles, RTK status)

### Batch processing

To process multiple location folders at once:

```bash
# Mac / Linux
for dir in IMG/Durango IMG/Highland IMG/Pioneer; do
  ./dji_mission_builder --images "$dir" --output Missions/
done

# Windows (PowerShell)
foreach ($dir in @("IMG\Durango","IMG\Highland","IMG\Pioneer")) {
  .\dji_mission_builder.exe --images $dir --output Missions\
}
```

---

## Loading the mission on RC Pro Enterprise

1. Copy the `.kmz` to a microSD card (or connect the controller via USB-C and drag it to internal storage)
2. Open **DJI Pilot 2** → Flight Route → **+** → Import → select the `.kmz`
3. Review the route on the map, confirm waypoint order and altitudes look correct
4. Connect the M3E, tap **Execute**, confirm launch

**Important:** always take off from the same GPS location as the original flight. The mission uses `relativeToStartPoint` altitude mode — the 120m waypoint height is measured from wherever the drone lifts off.

### If import fails

On older firmware, create and save one dummy waypoint mission in Pilot 2 first, then retry the import. This clears a one-time initialization state. Firmware v07.01.10.xx or later is recommended.

---

## What the drone does at each waypoint

1. Flies to waypoint and stops completely
2. Rotates gimbal to the original capture angle (waits up to `--gimbal-settle` seconds)
3. Fires the wide lens shutter
4. Hovers for `--dwell` seconds
5. Flies to the next waypoint

---

## RTK note

If RTK was **not active** during the original flight (shown in the summary file), altitude accuracy depends entirely on matching the original takeoff location. The summary file includes the calculated takeoff elevation (MSL) to help you confirm you're in the right spot.

---

## Building from source

Requires Python 3.9+, no external dependencies.

```bash
python dji_mission_builder.py --images /path/to/images
```

To build the binary yourself:

```bash
pip install pyinstaller
pyinstaller --onefile dji_mission_builder.py
# output: dist/dji_mission_builder
```
