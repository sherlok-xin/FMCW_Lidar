#!/usr/bin/env python3
"""Read-only workbook provenance inventory; no detector or E18 execution."""
from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import platform
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

import build_gt_benchmark_v0 as gt


ROOT = Path(__file__).resolve().parents[1]
WORKBOOK = ROOT / "bag/20260205/飞行轨迹坐标.xlsx"
OUT = ROOT / "results/negative_hover_evidence_audit"
FILES = ["gnss_workbook_inventory.json", "gnss_samples.csv",
         "gnss_adjacent_segments.csv", "gnss_condition_summary.csv",
         "gnss_evidence_execution.json"]
NS = {"s": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
      "r": "http://schemas.openxmlformats.org/package/2006/relationships"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(name, obj):
    (OUT / name).write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n")


def write_csv(name, rows):
    if not rows:
        raise RuntimeError("Inventory CSV must contain observed records")
    with (OUT / name).open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def read_all_cells():
    with zipfile.ZipFile(WORKBOOK) as z:
        shared_root = ET.fromstring(z.read("xl/sharedStrings.xml"))
        shared = ["".join(t.text or "" for t in si.findall(".//s:t", NS))
                  for si in shared_root.findall("s:si", NS)]
        wb = ET.fromstring(z.read("xl/workbook.xml"))
        rels = ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))
        targets = {r.attrib["Id"]: r.attrib["Target"] for r in rels}
        cells, sheets = [], []
        ridkey = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"
        for s in wb.findall("s:sheets/s:sheet", NS):
            target = targets[s.attrib[ridkey]]
            member = target.lstrip("/") if target.startswith("/") else "xl/" + target
            sheet = ET.fromstring(z.read(member))
            dim = sheet.find("s:dimension", NS)
            count = 0
            for c in sheet.findall(".//s:c", NS):
                value = c.find("s:v", NS)
                text = value.text if value is not None else "".join(
                    x.text or "" for x in c.findall(".//s:t", NS))
                if c.get("t") == "s":
                    text = shared[int(text)]
                formula = c.find("s:f", NS)
                if text or formula is not None:
                    cells.append({"sheet": s.attrib["name"], "cell": c.attrib["r"],
                                  "raw_value": text, "formula": formula.text if formula is not None else None,
                                  "cell_type": c.get("t", "n")})
                    count += 1
            sheets.append({"name": s.attrib["name"], "state": s.get("state", "visible"),
                           "member": member, "dimension": dim.get("ref") if dim is not None else None,
                           "nonempty_cells": count})
        members = z.namelist()
        extra = {n: z.read(n).decode("utf-8") for n in members
                 if "comment" in n.lower() or n.startswith("xl/externalLinks/")
                 or n.startswith("docProps/")}
        return cells, sheets, members, extra


def main():
    started = datetime.now(timezone.utc).isoformat()
    for name in FILES:
        if (OUT / name).exists():
            raise FileExistsError(f"Refusing to overwrite {OUT / name}")
    input_hash = sha(WORKBOOK)
    cells, sheets, members, extra = read_all_cells()
    specs = {s[1]: s for s in gt.CONDITION_SPECS}
    points, device = gt.parse_workbook(WORKBOOK)
    current, title_cell = None, None
    samples = []
    for c in cells:
        text = c["raw_value"] or ""
        if "飞行轨迹坐标" in text:
            current = text.replace("飞行轨迹坐标", "").strip()
            title_cell = c["cell"]
            continue
        if text.strip() == "设备坐标":
            current, title_cell = "设备坐标", c["cell"]
            continue
        m = gt.GNSS_ROW_RE.search(text)
        if not m:
            continue
        g = m.groups()
        week, tow, lat, lon, height = int(g[2]), float(g[1]), float(g[6]), float(g[7]), float(g[8])
        p = gt.geodetic_to_enu(lat, lon, height, device["lat_deg"], device["lon_deg"], device["ellh_m"])
        samples.append({"condition_id": specs[current][0] if current in specs else "device_origin",
                        "condition_title": current, "source_path": str(WORKBOOK.relative_to(ROOT)),
                        "source_sheet": c["sheet"], "source_title_cell": title_cell,
                        "source_cell": c["cell"], "raw_text": text, "gnss_index": int(g[0]),
                        "gps_week": week, "gps_tow_s": tow, "gps_utc_unix_s": gt.gps_to_unix(week, tow),
                        "gps_utc": gt.iso_utc(gt.gps_to_unix(week, tow)),
                        "lat_deg": lat, "lon_deg": lon, "ellh_m": height,
                        "enu_e_m": float(p[0]), "enu_n_m": float(p[1]), "enu_u_m": float(p[2]),
                        "opaque_N_raw": g[3], "opaque_E_raw": g[4], "opaque_V_raw": g[5],
                        "opaque_Q_raw": g[12], "opaque_unlabelled_1": g[9],
                        "opaque_unlabelled_2": g[10], "opaque_unlabelled_3": g[11],
                        "independent_presence_certificate": "UNKNOWN",
                        "flight_mode_armed_landed_state": "UNKNOWN",
                        "formal_evaluation_qualified": False})
    assert len(samples) == sum(len(v) for v in points.values()) + 1
    segments, summaries = [], []
    for condition, ps in points.items():
        cid = specs[condition][0]
        rows = [r for r in samples if r["condition_id"] == cid]
        assert len(rows) == len(ps)
        speeds, intervals = [], []
        for a, b in zip(rows, rows[1:]):
            # TOW differences avoid subtracting two large Unix values.
            dt = (b["gps_week"] - a["gps_week"]) * 604800 + b["gps_tow_s"] - a["gps_tow_s"]
            if dt <= 0:
                raise ValueError(f"Non-positive GNSS sample interval in {cid}")
            displacement = np.array([b[k] - a[k] for k in ("enu_e_m", "enu_n_m", "enu_u_m")])
            distance = float(np.linalg.norm(displacement))
            speed = distance / dt
            speeds.append(speed)
            intervals.append(dt)
            segments.append({"condition_id": cid, "source_path": a["source_path"],
                             "source_sheet": a["source_sheet"], "from_cell": a["source_cell"],
                             "to_cell": b["source_cell"], "from_index": a["gnss_index"],
                             "to_index": b["gnss_index"], "gps_week": a["gps_week"],
                             "tow_start_s": a["gps_tow_s"], "tow_end_s": b["gps_tow_s"],
                             "utc_start": a["gps_utc"], "utc_end": b["gps_utc"],
                             "dt_s": dt, "displacement_e_m": float(displacement[0]),
                             "displacement_n_m": float(displacement[1]),
                             "displacement_u_m": float(displacement[2]), "chord_distance_m": distance,
                             "chord_speed_mps": speed,
                             "speed_semantics": "endpoint chord / sample interval; not instantaneous speed or hover certificate",
                             "hover_within_interval": "UNKNOWN", "negative_within_interval": "UNKNOWN",
                             "formal_evaluation_qualified": False})
        ranges = [np.linalg.norm([r[k] for k in ("enu_e_m", "enu_n_m", "enu_u_m")]) for r in rows]
        summaries.append({"condition_id": cid, "condition_title": condition,
                          "source_sheet": rows[0]["source_sheet"], "title_cell": rows[0]["source_title_cell"],
                          "first_cell": rows[0]["source_cell"], "last_cell": rows[-1]["source_cell"],
                          "sample_count": len(rows), "segment_count": len(speeds),
                          "gps_week": rows[0]["gps_week"], "tow_start_s": rows[0]["gps_tow_s"],
                          "tow_end_s": rows[-1]["gps_tow_s"], "utc_start": rows[0]["gps_utc"],
                          "utc_end": rows[-1]["gps_utc"], "dt_min_s": min(intervals),
                          "dt_median_s": float(np.median(intervals)), "dt_max_s": max(intervals),
                          "chord_speed_min_mps": min(speeds), "chord_speed_median_mps": float(np.median(speeds)),
                          "chord_speed_max_mps": max(speeds), "enu_range_min_m": float(min(ranges)),
                          "enu_range_max_m": float(max(ranges)), "enu_u_min_m": min(r["enu_u_m"] for r in rows),
                          "enu_u_max_m": max(r["enu_u_m"] for r in rows),
                          "negative_window_certificate": "DATA_GAP", "hover_window_certificate": "DATA_GAP",
                          "formal_evaluation_qualified": False})
    assert len(segments) == sum(len(v) - 1 for v in points.values())
    first = next(r for r in samples if r["source_cell"] == "A2")
    origin = next(r for r in samples if r["condition_id"] == "device_origin")
    inventory = {"status": "DATA_GAP_NO_INDEPENDENT_NEGATIVE_OR_HOVER_CERTIFICATE",
                 "reader": "Python standard-library ZIP/XML; source opened read-only",
                 "openpyxl_available_in_run_environment": importlib.util.find_spec("openpyxl") is not None,
                 "openpyxl_fallback_reason": "ModuleNotFoundError observed in fmcw/bin/python and system python3; no installation/environment change",
                 "workbook": str(WORKBOOK.relative_to(ROOT)), "workbook_sha256": input_hash,
                 "sheets": sheets, "all_zip_members": members, "extra_metadata_xml": extra,
                 "all_nonempty_cells": cells, "nonempty_cell_count": len(cells),
                 "flight_gnss_samples": sum(len(v) for v in points.values()), "device_samples": 1,
                 "adjacent_segments": len(segments), "conditions": len(points),
                 "comments_members": [n for n in members if "comment" in n.lower()],
                 "external_link_members": [n for n in members if n.startswith("xl/externalLinks/")],
                 "A2_equals_device_A85_raw_text": first["raw_text"] == origin["raw_text"],
                 "interpretation_limits": [
                     "GNSS week/TOW and Lat/Lon/Ellh use existing verified code conventions, including GPS-UTC 18 s and WGS84 ENU device origin.",
                     "N/E/V/Q and three unlabelled fields remain opaque; they do not encode authenticated flight state here.",
                     "The first roundtrip10 sample exactly duplicates the device coordinate; this is not a takeoff/landed/absence event.",
                     "All endpoint chord speeds are descriptive; sparse sampling cannot authenticate hover or moving-to-hover transition.",
                     "Six cross conditions each contain four samples; roundtrip10/13/5 contain 28/12/7, with nonuniform sample intervals.",
                     "No timestamp-linked independent armed/landed/video/motion-state event exists in this workbook.",
                     "Low height, zero ENU range and provisional GNSS projection outside a chosen ROI are not certified LiDAR-FOV negatives.",
                     "Reliable per-bag clock events, surveyed extrinsic/lever arm and sensor FOV remain UNKNOWN."]}
    OUT.mkdir(parents=True, exist_ok=True)
    write_json(FILES[0], inventory)
    write_csv(FILES[1], samples)
    write_csv(FILES[2], segments)
    write_csv(FILES[3], summaries)
    assert sha(WORKBOOK) == input_hash
    execution = {"status": "VERIFIED_DESCRIPTIVE_GNSS_INVENTORY_NOT_FORMAL_GT",
                 "started_utc": started, "ended_utc": datetime.now(timezone.utc).isoformat(),
                 "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_gnss_inventory.py",
                 "python": platform.python_version(), "numpy": np.__version__,
                 "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                 "script_sha256": sha(__file__), "verified_conversion_source": "scripts/build_gt_benchmark_v0.py",
                 "verified_conversion_source_sha256": sha(gt.__file__), "input_sha256": input_hash,
                 "input_unchanged_after_execution": True, "detector_tracker_or_E18_executed": False,
                 "output_sha256": {n: sha(OUT / n) for n in FILES[:-1]},
                 "counts": {"all_nonempty_cells": len(cells), "all_gnss_rows_including_device": len(samples),
                            "flight_gnss_rows": len(samples) - 1, "adjacent_segments": len(segments), "conditions": len(summaries)}}
    write_json(FILES[4], execution)
    print(json.dumps(execution["counts"], ensure_ascii=False))


if __name__ == "__main__":
    main()
