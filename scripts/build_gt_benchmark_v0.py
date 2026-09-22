#!/usr/bin/env python3
"""Build the 2026-02-05 FMCW-LiDAR GNSS alignment and Benchmark v0.

This is an analysis/evaluation tool.  It does not change the research tracker.
Raw bags and archives are opened read-only.  Intermediate caches are written to
``results/gt_benchmark_v0/cache`` so that alignment can be audited without
re-reading roughly 82 GB of uncompressed ROS bag payloads.

The script deliberately preserves epistemic boundaries:

* Excel fields N/E/V/Q are retained as opaque values; only GPS week, TOW,
  Lat, Lon, and Ellh are interpreted.
* ROS bag record time, filename time, PointCloud2 header time, and GPS time are
  separate columns.
* Spatial alignment is estimated from data and carries a validity/confidence
  status.  It is never hand-tuned.
* GNSS interpolation is piecewise linear and never extrapolates.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import re
import struct
import sys
import time
import zipfile
from contextlib import contextmanager, redirect_stdout
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import BinaryIO, Iterator
from xml.etree import ElementTree as ET

import numpy as np
from scipy.optimize import differential_evolution, least_squares
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN


ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = ROOT / "bag" / "20260205"
WORKBOOK = DATA_ROOT / "飞行轨迹坐标.xlsx"
OUTPUT_ROOT = ROOT / "results" / "gt_benchmark_v0"
CACHE_ROOT = OUTPUT_ROOT / "cache"

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
# GPS-UTC was 18 s on 2026-02-05.  This is kept explicit rather than inferred
# from the unlabeled workbook fields.
GPS_UTC_LEAP_SECONDS = 18
CHINA_TZ = timezone(timedelta(hours=8))

INTENSITY_MIN = 10.0
INTENSITY_MAX = 250.0
HEIGHT_MIN = 10.0
VOXEL_SIZE = 1.0
BACKGROUND_INIT_FRAMES = 8
BACKGROUND_PROB_THRESHOLD = 0.05
DBSCAN_EPS = 2.0
DBSCAN_MIN_SAMPLES = 1
GT_NEIGHBORHOOD_RADIUS_M = 5.0
TRACK_MATCH_RADIUS_M = 5.0
CALIBRATION_MATCH_RADIUS_M = 8.0
CALIBRATION_VELOCITY_RESIDUAL_MPS = 5.0


CONDITION_SPECS = [
    ("300m_roundtrip_10", "300m往返10m/s", "300m往返速度10ms.zip", "roundtrip", 300, 10),
    ("300m_roundtrip_13", "300m往返13m/s", "300m往返速度13ms.zip", "roundtrip", 300, 13),
    ("300m_roundtrip_5", "300m往返5m/s", "300m往返速度5ms.zip", "roundtrip", 300, 5),
    ("100m_cross_10", "100m横飞10m/s", "100m横飞速度10ms.zip", "cross", 100, 10),
    ("100m_cross_5", "100m横飞5m/s", "100m横飞速度5ms.zip", "cross", 100, 5),
    ("200m_cross_10", "200m横飞10m/s", "200m横飞速度10ms.zip", "cross", 200, 10),
    ("200m_cross_5", "200m横飞5m/s", "200m横飞速度5ms.zip", "cross", 200, 5),
    ("300m_cross_10", "300m横飞10m/s", "300m横飞速度10ms.zip", "cross", 300, 10),
    ("300m_cross_5", "300m横飞5m/s", "300m横飞速度5ms.zip", "cross", 300, 5),
]


ROS_TYPES = {
    1: (np.int8, 1), 2: (np.uint8, 1), 3: (np.int16, 2), 4: (np.uint16, 2),
    5: (np.int32, 4), 6: (np.uint32, 4), 7: (np.float32, 4), 8: (np.float64, 8),
}


@dataclass
class GnssPoint:
    condition: str
    index: int
    gps_week: int
    tow_s: float
    gps_unix_s: float
    lat_deg: float
    lon_deg: float
    ellh_m: float
    enu_e_m: float
    enu_n_m: float
    enu_u_m: float
    n_raw: int
    e_raw: int
    v_raw: int
    q_raw: int
    unknown_1: float
    unknown_2: float
    unknown_3: float


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        while True:
            block = fh.read(chunk_size)
            if not block:
                break
            h.update(block)
    return h.hexdigest()


def gps_to_unix(week: int, tow_s: float) -> float:
    return (GPS_EPOCH + timedelta(weeks=week, seconds=tow_s - GPS_UTC_LEAP_SECONDS)).timestamp()


def iso_utc(value: float | None) -> str:
    if value is None or not np.isfinite(value):
        return ""
    return datetime.fromtimestamp(float(value), timezone.utc).isoformat(timespec="microseconds")


def parse_filename_time(member_name: str) -> float | None:
    m = re.search(r"(2026-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})", member_name)
    if not m:
        return None
    local = datetime.strptime(m.group(1), "%Y-%m-%d-%H-%M-%S").replace(tzinfo=CHINA_TZ)
    return local.timestamp()


def normalize_zip_member_name(name: str) -> str:
    """Recover legacy GBK filenames that ZIP exposed through CP437."""
    try:
        recovered = name.encode("cp437").decode("gbk")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return name
    return recovered if any("\u4e00" <= ch <= "\u9fff" for ch in recovered) else name


def _xlsx_column_a_strings(path: Path) -> list[str]:
    """Read column A strings using only the standard XLSX ZIP/XML format."""
    ns = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"
    with zipfile.ZipFile(path) as zf:
        shared_root = ET.fromstring(zf.read("xl/sharedStrings.xml"))
        shared = ["".join(t.text or "" for t in si.iter(ns + "t"))
                  for si in shared_root.findall(ns + "si")]
        sheet = ET.fromstring(zf.read("xl/worksheets/sheet1.xml"))
        values: list[str] = []
        for cell in sheet.findall(".//" + ns + "c"):
            if not cell.attrib.get("r", "").startswith("A"):
                continue
            value = cell.find(ns + "v")
            if value is None:
                continue
            if cell.attrib.get("t") == "s":
                values.append(shared[int(value.text)])
            else:
                values.append(value.text or "")
        return values


GNSS_ROW_RE = re.compile(
    r"^\s*(\d+)\s+([0-9.]+)\s+\[(\d+)\].*?"
    r"(-?\d+)\s*,N\s+(-?\d+)\s*,E\s+(-?\d+)\s*,V\s+"
    r"([0-9.]+)\s*,Lat\s+([0-9.]+)\s*,Lon\s+([0-9.]+)\s*,Ellh\s+"
    r"([0-9.]+)\s*,\s*([0-9.]+)\s*,\s*([0-9.]+)\s+(\d+)\s*,Q"
)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, h_m: float) -> np.ndarray:
    # WGS-84 ellipsoid.
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    n = a / math.sqrt(1.0 - e2 * sl * sl)
    return np.array([(n + h_m) * cl * co,
                     (n + h_m) * cl * so,
                     (n * (1.0 - e2) + h_m) * sl], dtype=float)


def geodetic_to_enu(lat_deg: float, lon_deg: float, h_m: float,
                    lat0_deg: float, lon0_deg: float, h0_m: float) -> np.ndarray:
    p = geodetic_to_ecef(lat_deg, lon_deg, h_m)
    p0 = geodetic_to_ecef(lat0_deg, lon0_deg, h0_m)
    d = p - p0
    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    sl, cl = math.sin(lat0), math.cos(lat0)
    so, co = math.sin(lon0), math.cos(lon0)
    rot = np.array([[-so, co, 0.0],
                    [-sl * co, -sl * so, cl],
                    [cl * co, cl * so, sl]])
    return rot @ d


def parse_workbook(path: Path) -> tuple[dict[str, list[GnssPoint]], dict[str, object]]:
    values = _xlsx_column_a_strings(path)
    raw_by_condition: dict[str, list[dict[str, object]]] = {}
    current: str | None = None
    device: dict[str, object] | None = None
    for text in values:
        if "飞行轨迹坐标" in text:
            current = text.replace("飞行轨迹坐标", "").strip()
            raw_by_condition[current] = []
            continue
        if text.strip() == "设备坐标":
            current = "设备坐标"
            continue
        match = GNSS_ROW_RE.search(text)
        if not match:
            continue
        g = match.groups()
        row = {
            "index": int(g[0]), "tow_s": float(g[1]), "gps_week": int(g[2]),
            "n_raw": int(g[3]), "e_raw": int(g[4]), "v_raw": int(g[5]),
            "lat_deg": float(g[6]), "lon_deg": float(g[7]), "ellh_m": float(g[8]),
            "unknown_1": float(g[9]), "unknown_2": float(g[10]),
            "unknown_3": float(g[11]), "q_raw": int(g[12]),
        }
        if current == "设备坐标":
            device = row
        elif current is not None:
            raw_by_condition[current].append(row)
    if device is None:
        raise RuntimeError("Workbook does not contain a parseable 设备坐标 row")

    parsed: dict[str, list[GnssPoint]] = {}
    for condition, rows in raw_by_condition.items():
        points = []
        for row in rows:
            enu = geodetic_to_enu(
                float(row["lat_deg"]), float(row["lon_deg"]), float(row["ellh_m"]),
                float(device["lat_deg"]), float(device["lon_deg"]), float(device["ellh_m"]),
            )
            points.append(GnssPoint(
                condition=condition,
                gps_unix_s=gps_to_unix(int(row["gps_week"]), float(row["tow_s"])),
                enu_e_m=float(enu[0]), enu_n_m=float(enu[1]), enu_u_m=float(enu[2]),
                **row,
            ))
        parsed[condition] = points
    return parsed, device


def _u32(buf: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<I", buf, offset)[0]


def _read_exact(fh: BinaryIO, size: int) -> bytes:
    data = fh.read(size)
    if len(data) != size:
        raise EOFError(f"short read: got {len(data)}, expected {size}")
    return data


def _skip_exact(fh: BinaryIO, size: int) -> None:
    if size <= 0:
        return
    try:
        start = fh.tell()
        fh.seek(size, io.SEEK_CUR)
        if fh.tell() - start == size:
            return
    except (AttributeError, OSError, io.UnsupportedOperation):
        pass
    remaining = size
    while remaining:
        block = fh.read(min(8 * 1024 * 1024, remaining))
        if not block:
            raise EOFError(f"short skip with {remaining} bytes remaining")
        remaining -= len(block)


def _record_fields(buf: bytes) -> dict[str, bytes]:
    out: dict[str, bytes] = {}
    offset = 0
    while offset < len(buf):
        size = _u32(buf, offset)
        offset += 4
        item = buf[offset:offset + size]
        offset += size
        if b"=" in item:
            key, value = item.split(b"=", 1)
            out[key.decode(errors="replace")] = value
    return out


def _ros_time(buf: bytes) -> float:
    sec, nsec = struct.unpack("<II", buf)
    return float(sec) + float(nsec) * 1e-9


def decode_pointcloud2(payload: bytes) -> tuple[np.ndarray, dict[str, object]]:
    seq, sec, nsec = struct.unpack_from("<III", payload, 0)
    offset = 12
    frame_len = _u32(payload, offset)
    offset += 4
    frame_id = payload[offset:offset + frame_len].decode(errors="replace")
    offset += frame_len
    height, width = struct.unpack_from("<II", payload, offset)
    offset += 8
    nfields = _u32(payload, offset)
    offset += 4
    fields = []
    for _ in range(nfields):
        name_len = _u32(payload, offset)
        offset += 4
        name = payload[offset:offset + name_len].decode(errors="replace")
        offset += name_len
        field_offset = _u32(payload, offset)
        datatype = payload[offset + 4]
        count = _u32(payload, offset + 5)
        offset += 9
        fields.append((name, field_offset, datatype, count))
    is_bigendian = bool(payload[offset])
    offset += 1
    point_step, row_step = struct.unpack_from("<II", payload, offset)
    offset += 8
    data_len = _u32(payload, offset)
    offset += 4
    data = memoryview(payload)[offset:offset + data_len]
    endian = ">" if is_bigendian else "<"
    names, formats, offsets = [], [], []
    for name, field_offset, datatype, count in fields:
        base, _ = ROS_TYPES[datatype]
        dtype = np.dtype(base).newbyteorder(endian)
        names.append(name)
        formats.append((dtype, (count,)) if count != 1 else dtype)
        offsets.append(field_offset)
    dtype = np.dtype({"names": names, "formats": formats,
                      "offsets": offsets, "itemsize": point_step})
    arr = np.frombuffer(data, dtype=dtype, count=height * width)
    meta = {
        "seq": int(seq), "header_time": float(sec) + float(nsec) * 1e-9,
        "frame_id": frame_id, "height": int(height), "width": int(width),
        "point_step": int(point_step), "row_step": int(row_step),
        "fields": names,
    }
    return arr, meta


@contextmanager
def open_condition_bag(spec: tuple[str, str, str, str, int, int]):
    condition_id, _, archive_name, _, _, _ = spec
    archive_path = DATA_ROOT / archive_name
    if not archive_path.exists():
        raise FileNotFoundError(archive_path)
    zf = zipfile.ZipFile(archive_path)
    members = [name for name in zf.namelist() if name.endswith(".bag")]
    if len(members) != 1:
        zf.close()
        raise RuntimeError(f"{archive_path}: expected exactly one .bag, found {members}")
    member = members[0]
    display_member = normalize_zip_member_name(member)
    # Read the canonical archive member for every condition.  The compressed
    # stream is substantially faster than the extracted 4.6 GB copy on this
    # filesystem and keeps source handling identical across all nine packages.
    fh = zf.open(member, "r")
    source = f"{archive_path.relative_to(ROOT)}::{display_member}"
    try:
        yield fh, {
            "condition_id": condition_id,
            "archive_path": str(archive_path.relative_to(ROOT)),
            "bag_member": display_member,
            "bag_source_used": source,
            "archive_crc32": f"{zf.getinfo(member).CRC:08x}",
            "bag_uncompressed_bytes": int(zf.getinfo(member).file_size),
            "bag_compressed_bytes": int(zf.getinfo(member).compress_size),
        }
    finally:
        fh.close()
        zf.close()


def iter_condition_pointclouds(spec: tuple[str, str, str, str, int, int]) -> Iterator[tuple[np.ndarray, dict[str, object]]]:
    """Yield decoded PointCloud2 arrays and metadata from one condition."""
    with open_condition_bag(spec) as (fh, _):
        if _read_exact(fh, 13) != b"#ROSBAG V2.0\n":
            raise ValueError("not ROS bag v2")
        while True:
            first = fh.read(4)
            if not first:
                break
            header_len = _u32(first)
            header = _record_fields(_read_exact(fh, header_len))
            data_len = _u32(_read_exact(fh, 4))
            op = header.get("op", b"\0")[0]
            if op != 5:
                _skip_exact(fh, data_len)
                continue
            if header.get("compression", b"").decode(errors="replace") != "none":
                raise NotImplementedError("compressed ROS chunks are unsupported")
            consumed = 0
            while consumed < data_len:
                inner_header_len = _u32(_read_exact(fh, 4))
                inner_header = _record_fields(_read_exact(fh, inner_header_len))
                inner_len = _u32(_read_exact(fh, 4))
                consumed += 4 + inner_header_len + 4 + inner_len
                if inner_header.get("op", b"\0")[0] != 2:
                    _skip_exact(fh, inner_len)
                    continue
                payload = _read_exact(fh, inner_len)
                try:
                    arr, meta = decode_pointcloud2(payload)
                except Exception:
                    continue
                needed = {"x", "y", "z", "intensity", "velocity"}
                if not needed.issubset(arr.dtype.names or ()):
                    continue
                meta["bag_time"] = _ros_time(inner_header["time"])
                meta["conn"] = _u32(inner_header["conn"])
                yield arr, meta


class StageFilter:
    """Current 1 m occupancy/neighborhood preprocessing, with masks exposed."""

    def __init__(self):
        self.bounds = ((0.0, 500.0), (-150.0, 150.0), (-5.0, 50.0))
        self.shape = tuple(int(math.ceil((hi - lo) / VOXEL_SIZE)) for lo, hi in self.bounds)
        self.counts = np.zeros(np.prod(self.shape), dtype=np.int32)
        self.frame_count = 0

    def indices(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        low = np.array([x[0] for x in self.bounds])
        q = np.floor((points - low) / VOXEL_SIZE).astype(np.int64)
        valid = np.all((q >= 0) & (q < np.array(self.shape)), axis=1)
        idx = np.full(len(points), -1, dtype=np.int64)
        idx[valid] = q[valid, 0] * self.shape[1] * self.shape[2] + q[valid, 1] * self.shape[2] + q[valid, 2]
        return idx, valid

    def neighborhood(self, dynamic: np.ndarray, idx: np.ndarray) -> np.ndarray:
        if not np.any(dynamic):
            return dynamic.copy()
        dyn_idx = idx[dynamic]
        rows = np.flatnonzero(dynamic)
        nynz = self.shape[1] * self.shape[2]
        ix = dyn_idx // nynz
        iy = (dyn_idx % nynz) // self.shape[2]
        iz = dyn_idx % self.shape[2]
        deltas = np.array([(dx, dy, dz) for dx in (-1, 0, 1)
                           for dy in (-1, 0, 1) for dz in (-1, 0, 1)], dtype=np.int64)
        offsets = deltas[:, 0] * nynz + deltas[:, 1] * self.shape[2] + deltas[:, 2]
        neighbors = dyn_idx[:, None] + offsets[None, :]
        coords = np.stack([ix, iy, iz], axis=1)[:, None, :] + deltas[None, :, :]
        inbound = np.all((coords >= 0) & (coords < np.array(self.shape)), axis=2)
        neighbors = np.clip(neighbors, 0, len(self.counts) - 1)
        occupied = (self.counts[neighbors] > 0) & inbound
        dynamic_neighbor = np.isin(neighbors.ravel(), dyn_idx).reshape(neighbors.shape) & inbound
        occupied_n = occupied.sum(axis=1)
        static_n = (occupied & ~dynamic_neighbor).sum(axis=1)
        ratio = np.divide(static_n, occupied_n, out=np.zeros_like(static_n, dtype=float), where=occupied_n > 0)
        remove = (occupied_n > 0) & (ratio > 0.6)
        refined = dynamic.copy()
        refined[rows[remove]] = False
        return refined

    def process(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.frame_count += 1
        idx, valid = self.indices(points)
        if self.frame_count <= BACKGROUND_INIT_FRAMES:
            pre = np.zeros(len(points), dtype=bool)
            post = pre.copy()
            static = valid
        else:
            prob = np.zeros(len(points), dtype=float)
            prob[valid] = self.counts[idx[valid]] / self.frame_count
            pre = (prob < BACKGROUND_PROB_THRESHOLD) & valid
            post = self.neighborhood(pre, idx)
            static = (~post) & valid
        occupied = np.unique(idx[static & (idx >= 0)])
        self.counts[occupied] += 1
        return pre, post


def flatten_frames(frames: list[np.ndarray], width: int) -> tuple[np.ndarray, np.ndarray]:
    if not frames or not any(len(x) for x in frames):
        return np.empty((0,), dtype=np.int32), np.empty((0, width), dtype=np.float32)
    ids = np.concatenate([np.full(len(x), i, dtype=np.int32) for i, x in enumerate(frames)])
    values = np.concatenate(frames, axis=0).astype(np.float32, copy=False)
    return ids, values


def extract_condition(spec: tuple[str, str, str, str, int, int], force: bool = False) -> dict[str, object]:
    condition_id, condition_cn, _, motion_class, nominal_range, nominal_speed = spec
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    npz_path = CACHE_ROOT / f"{condition_id}.npz"
    json_path = CACHE_ROOT / f"{condition_id}.json"
    if npz_path.exists() and json_path.exists() and not force:
        return json.loads(json_path.read_text(encoding="utf-8"))

    frame_rows: list[dict[str, object]] = []
    height_frames: list[np.ndarray] = []
    background_frames: list[np.ndarray] = []
    neighborhood_frames: list[np.ndarray] = []
    connection_rows: dict[int, dict[str, str]] = {}
    top_ops: dict[int, int] = {}
    stage_filter = StageFilter()
    started = time.time()

    with open_condition_bag(spec) as (fh, source_meta):
        if _read_exact(fh, 13) != b"#ROSBAG V2.0\n":
            raise ValueError(f"{source_meta['bag_source_used']} is not ROS bag v2")
        while True:
            first = fh.read(4)
            if not first:
                break
            if len(first) != 4:
                raise EOFError("truncated top-level record")
            header_len = _u32(first)
            header = _record_fields(_read_exact(fh, header_len))
            data_len = _u32(_read_exact(fh, 4))
            op = header.get("op", b"\0")[0]
            top_ops[op] = top_ops.get(op, 0) + 1
            if op == 7:  # CONNECTION
                data = _record_fields(_read_exact(fh, data_len))
                conn = _u32(header["conn"])
                connection_rows[conn] = {
                    "topic": header.get("topic", b"").decode(errors="replace"),
                    "type": data.get("type", b"").decode(errors="replace"),
                    "md5sum": data.get("md5sum", b"").decode(errors="replace"),
                }
                continue
            if op != 5:  # not CHUNK
                _skip_exact(fh, data_len)
                continue
            compression = header.get("compression", b"").decode(errors="replace")
            if compression != "none":
                raise NotImplementedError(f"ROS chunk compression={compression!r} is unsupported")
            consumed = 0
            while consumed < data_len:
                inner_header_len = _u32(_read_exact(fh, 4))
                inner_header = _record_fields(_read_exact(fh, inner_header_len))
                inner_len = _u32(_read_exact(fh, 4))
                consumed += 4 + inner_header_len + 4 + inner_len
                inner_op = inner_header.get("op", b"\0")[0]
                if inner_op != 2:
                    _skip_exact(fh, inner_len)
                    continue
                payload = _read_exact(fh, inner_len)
                try:
                    arr, pc_meta = decode_pointcloud2(payload)
                except Exception:
                    # Preserve non-PointCloud2 messages in the topic manifest,
                    # but do not reinterpret their payload.
                    continue
                needed = {"x", "y", "z", "intensity", "velocity"}
                if not needed.issubset(arr.dtype.names or ()):
                    continue
                conn = _u32(inner_header["conn"])
                bag_time = _ros_time(inner_header["time"])
                x = np.asarray(arr["x"], dtype=np.float32).reshape(-1)
                y_sensor = np.asarray(arr["y"], dtype=np.float32).reshape(-1)
                z_sensor = np.asarray(arr["z"], dtype=np.float32).reshape(-1)
                intensity = np.asarray(arr["intensity"], dtype=np.float32).reshape(-1)
                velocity = np.asarray(arr["velocity"], dtype=np.float32).reshape(-1)
                # Rich 80-byte Feb-2026 packets use the repository's documented
                # x-forward/y-down/z-left -> x-forward/y-left/z-up conversion.
                if int(pc_meta["point_step"]) == 80:
                    y = z_sensor
                    z = -y_sensor
                else:
                    y = y_sensor
                    z = z_sensor
                finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity) & np.isfinite(velocity)
                intensity_mask = finite & (intensity >= INTENSITY_MIN) & (intensity <= INTENSITY_MAX)
                height_mask = intensity_mask & (z > HEIGHT_MIN)
                hi = np.flatnonzero(height_mask)
                hp = np.column_stack([x[hi], y[hi], z[hi]]).astype(np.float32, copy=False)
                hv = velocity[hi].astype(np.float32, copy=False)
                hval = intensity[hi].astype(np.float32, copy=False)
                if len(hp):
                    pre, post = stage_filter.process(hp)
                else:
                    # Match the current main baseline: completely empty height
                    # frames do not advance its background frame counter.
                    pre = post = np.zeros(0, dtype=bool)
                height_frames.append(np.column_stack([hp, hval, hv]))
                background_frames.append(np.column_stack([hp[pre], hval[pre], hv[pre]]))
                neighborhood_frames.append(np.column_stack([hp[post], hval[post], hv[post]]))
                clusters_n = 0
                if np.any(post):
                    clusters_n = int(len(set(DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES)
                                             .fit_predict(hp[post]))))
                frame_rows.append({
                    "frame_index": len(frame_rows), "conn": conn,
                    "bag_time": bag_time, **pc_meta,
                    "raw_n": int(finite.sum()), "intensity_n": int(intensity_mask.sum()),
                    "height_n": int(len(hp)), "background_n": int(pre.sum()),
                    "neighborhood_n": int(post.sum()), "clusters_n": clusters_n,
                    "background_frame_count": int(stage_filter.frame_count),
                })
                if len(frame_rows) == 1 or len(frame_rows) % 25 == 0:
                    elapsed = time.time() - started
                    print(f"extract {condition_id}: frame={len(frame_rows)} elapsed={elapsed:.1f}s", flush=True)
                del arr, payload

    h_frame, h_values = flatten_frames(height_frames, 5)
    b_frame, b_values = flatten_frames(background_frames, 5)
    n_frame, n_values = flatten_frames(neighborhood_frames, 5)
    np.savez_compressed(
        npz_path,
        height_frame=h_frame, height_values=h_values,
        background_frame=b_frame, background_values=b_values,
        neighborhood_frame=n_frame, neighborhood_values=n_values,
    )
    filename_time = parse_filename_time(str(source_meta["bag_member"]))
    result = {
        **source_meta,
        "condition_cn": condition_cn, "motion_class": motion_class,
        "nominal_range_m": nominal_range, "nominal_speed_mps": nominal_speed,
        "filename_time_unix": filename_time,
        "filename_time_local": (datetime.fromtimestamp(filename_time, CHINA_TZ).isoformat()
                                if filename_time is not None else ""),
        "connections": {str(k): v for k, v in connection_rows.items()},
        "top_record_counts": {str(k): v for k, v in top_ops.items()},
        "frames": frame_rows,
        "elapsed_s": time.time() - started,
        "cache_npz": str(npz_path.relative_to(ROOT)),
        "cache_json": str(json_path.relative_to(ROOT)),
        "preprocessing": {
            "coordinate_transform_rich80": "[x,z,-y]",
            "intensity_range_inclusive": [INTENSITY_MIN, INTENSITY_MAX],
            "height_gt_m": HEIGHT_MIN,
            "voxel_size_m": VOXEL_SIZE,
            "background_init_frames": BACKGROUND_INIT_FRAMES,
            "background_probability_threshold": BACKGROUND_PROB_THRESHOLD,
            "dbscan_eps_m": DBSCAN_EPS,
            "dbscan_min_samples": DBSCAN_MIN_SAMPLES,
        },
    }
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def extract_all(force: bool = False) -> list[dict[str, object]]:
    gnss, _ = parse_workbook(WORKBOOK)
    expected = {spec[1] for spec in CONDITION_SPECS}
    if set(gnss) != expected:
        raise RuntimeError(f"Workbook/condition mismatch: workbook={set(gnss)}, expected={expected}")
    rows = []
    for spec in CONDITION_SPECS:
        rows.append(extract_condition(spec, force=force))
    return rows


def circular_difference(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def estimate_design_yaw(gnss: dict[str, list[GnssPoint]]) -> dict[str, object]:
    """Estimate ENU->LiDAR yaw from the three named radial round trips.

    This is a trajectory-geometry estimate, not a surveyed sensor extrinsic.
    The sign is fixed by mapping the farthest point away from the device to
    positive LiDAR x.  Cross flights are retained as an independent geometry
    check and are not used to force a better-looking point-cloud match.
    """
    estimates = []
    diagnostics = []
    for spec in CONDITION_SPECS:
        condition_id, condition_cn, _, motion, _, _ = spec
        if motion != "roundtrip":
            continue
        xy = np.array([[p.enu_e_m, p.enu_n_m] for p in gnss[condition_cn]], dtype=float)
        centered = xy - xy.mean(axis=0)
        cov = centered.T @ centered
        eigvals, eigvecs = np.linalg.eigh(cov)
        axis = eigvecs[:, int(np.argmax(eigvals))]
        far = xy[int(np.argmax(np.linalg.norm(xy, axis=1)))]
        if np.dot(axis, far) < 0:
            axis = -axis
        axis_angle = math.atan2(axis[1], axis[0])
        yaw = -axis_angle
        estimates.append(yaw)
        diagnostics.append({
            "condition_id": condition_id,
            "axis_angle_deg": math.degrees(axis_angle),
            "yaw_deg": math.degrees(yaw),
            "pca_linearity": float(np.max(eigvals) / np.sum(eigvals)),
        })
    mean = math.atan2(np.mean(np.sin(estimates)), np.mean(np.cos(estimates)))
    residuals = np.array([circular_difference(x, mean) for x in estimates])
    return {
        "yaw_rad": float(mean), "yaw_deg": float(math.degrees(mean)),
        "between_roundtrip_sd_deg": float(math.degrees(np.std(residuals, ddof=1))),
        "method": "PCA axes of three named roundtrip trajectories mapped to +LiDAR x",
        "diagnostics": diagnostics,
        "translation_m": [0.0, 0.0, 0.0],
        "translation_status": "UNKNOWN lever arm; device ENU origin is provisionally identified with LiDAR origin",
    }


def interpolate_gnss(points: list[GnssPoint], query_unix: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    source_t = np.array([p.gps_unix_s for p in points], dtype=float)
    source_xyz = np.array([[p.enu_e_m, p.enu_n_m, p.enu_u_m] for p in points], dtype=float)
    valid = (query_unix >= source_t[0]) & (query_unix <= source_t[-1])
    out = np.full((len(query_unix), 3), np.nan, dtype=float)
    segment = np.full(len(query_unix), -1, dtype=int)
    if np.any(valid):
        for dim in range(3):
            out[valid, dim] = np.interp(query_unix[valid], source_t, source_xyz[:, dim])
        segment[valid] = np.clip(np.searchsorted(source_t, query_unix[valid], side="right") - 1,
                                 0, len(source_t) - 2)
    return out, valid, segment


def enu_to_lidar(enu: np.ndarray, yaw_rad: float,
                 translation: np.ndarray | None = None) -> np.ndarray:
    translation = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    out = np.empty_like(enu, dtype=float)
    out[:, 0] = c * enu[:, 0] - s * enu[:, 1] + translation[0]
    out[:, 1] = s * enu[:, 0] + c * enu[:, 1] + translation[1]
    out[:, 2] = enu[:, 2] + translation[2]
    return out


def stage_arrays_by_frame(condition_id: str, stage: str, frame_count: int) -> list[np.ndarray]:
    with np.load(CACHE_ROOT / f"{condition_id}.npz") as cache:
        frame_ids = cache[f"{stage}_frame"]
        values = cache[f"{stage}_values"]
        return [values[frame_ids == i].copy() for i in range(frame_count)]


def clusters_by_frame(condition_id: str, frame_count: int, stage: str = "height") -> list[np.ndarray]:
    """Return [x,y,z,count,mean_velocity] DBSCAN clusters per frame."""
    arrays = stage_arrays_by_frame(condition_id, stage, frame_count)
    result: list[np.ndarray] = []
    for values in arrays:
        if not len(values):
            result.append(np.empty((0, 5), dtype=float))
            continue
        labels = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit_predict(values[:, :3])
        rows = []
        for label in np.unique(labels):
            selected = values[labels == label]
            rows.append([*selected[:, :3].mean(axis=0), len(selected), selected[:, 4].mean()])
        result.append(np.asarray(rows, dtype=float))
    return result


def alignment_matches(frames: list[dict[str, object]], clusters: list[np.ndarray],
                      points: list[GnssPoint], yaw_rad: float, offset_s: float) -> dict[str, object]:
    bag_t = np.array([float(row["bag_time"]) for row in frames], dtype=float)
    query = bag_t + offset_s
    enu, valid, _ = interpolate_gnss(points, query)
    expected = enu_to_lidar(enu, yaw_rad)
    # Piecewise-linear truth range rate, evaluated at actual frame times.
    expected_range = np.linalg.norm(expected, axis=1)
    range_rate = np.full(len(frames), np.nan)
    valid_idx = np.flatnonzero(valid)
    if len(valid_idx) >= 2:
        range_rate[valid_idx] = np.gradient(expected_range[valid_idx], query[valid_idx])
    pairs = []
    for i in valid_idx:
        candidates = clusters[i]
        if not len(candidates):
            continue
        distance = np.linalg.norm(candidates[:, :3] - expected[i], axis=1)
        velocity_residual = np.abs(-candidates[:, 4] - range_rate[i])
        combined = distance + 0.5 * np.minimum(velocity_residual, 10.0)
        j = int(np.argmin(combined))
        if distance[j] <= CALIBRATION_MATCH_RADIUS_M and velocity_residual[j] <= CALIBRATION_VELOCITY_RESIDUAL_MPS:
            pairs.append({
                "frame_index": int(i), "cluster_index": j,
                "distance_m": float(distance[j]),
                "velocity_residual_mps": float(velocity_residual[j]),
                "expected": expected[i].tolist(),
                "observed": candidates[j, :3].tolist(),
                "cluster_points": int(candidates[j, 3]),
                "cluster_velocity": float(candidates[j, 4]),
                "expected_range_rate": float(range_rate[i]),
            })
    distances = np.array([x["distance_m"] for x in pairs], dtype=float)
    velocity_residuals = np.array([x["velocity_residual_mps"] for x in pairs], dtype=float)
    return {
        "matches": pairs,
        "match_count": len(pairs),
        "valid_frame_count": int(valid.sum()),
        "distance_rmse_m": float(np.sqrt(np.mean(distances ** 2))) if len(distances) else None,
        "distance_median_m": float(np.median(distances)) if len(distances) else None,
        "velocity_residual_median_mps": float(np.median(velocity_residuals)) if len(velocity_residuals) else None,
    }


def _alignment_rank(result: dict[str, object], midpoint_delta: float, tested_delta: float) -> tuple[float, ...]:
    # Lexicographic: support first, then spatial and Doppler residual, then the
    # least departure from interval-midpoint registration.
    return (
        float(result["match_count"]),
        -float(result["distance_median_m"] if result["distance_median_m"] is not None else 1e6),
        -float(result["velocity_residual_median_mps"] if result["velocity_residual_median_mps"] is not None else 1e6),
        -abs(tested_delta - midpoint_delta),
    )


def estimate_condition_offset(meta: dict[str, object], clusters: list[np.ndarray],
                              points: list[GnssPoint], yaw_rad: float) -> dict[str, object]:
    frames = meta["frames"]
    bag_start = float(frames[0]["bag_time"])
    bag_end = float(frames[-1]["bag_time"])
    gnss_start = points[0].gps_unix_s
    gnss_end = points[-1].gps_unix_s
    midpoint = 0.5 * ((gnss_start + gnss_end) - (bag_start + bag_end))
    grid = np.arange(midpoint - 30.0, midpoint + 30.0001, 0.25)
    tested = []
    best_delta = midpoint
    best = alignment_matches(frames, clusters, points, yaw_rad, midpoint)
    best_rank = _alignment_rank(best, midpoint, midpoint)
    for delta in grid:
        result = alignment_matches(frames, clusters, points, yaw_rad, float(delta))
        tested.append((float(delta), result))
        rank = _alignment_rank(result, midpoint, float(delta))
        if rank > best_rank:
            best_delta, best, best_rank = float(delta), result, rank
    data_supported = int(best["match_count"]) >= 3
    if data_supported:
        selected_delta = best_delta
        method = "height-cluster/GNSS grid fit initialized by interval midpoints"
        confidence = "MEDIUM" if int(best["match_count"]) >= 5 else "LOW"
        near = [d for d, r in tested
                if int(r["match_count"]) >= int(best["match_count"]) - 1
                and (r["distance_median_m"] or 1e6) <= (best["distance_median_m"] or 1e6) + 2.0]
        uncertainty = 0.5 * (max(near) - min(near)) if near else 0.25
        selected = best
    else:
        selected_delta = midpoint
        method = "LiDAR/GNSS interval-midpoint registration; no unique target event found"
        confidence = "LOW"
        median_gnss_dt = float(np.median(np.diff([p.gps_unix_s for p in points])))
        uncertainty = 0.5 * abs((bag_end - bag_start) - (gnss_end - gnss_start)) + 0.5 * median_gnss_dt
        selected = alignment_matches(frames, clusters, points, yaw_rad, selected_delta)
    return {
        "offset_s": float(selected_delta), "offset_method": method,
        "offset_confidence": confidence, "offset_uncertainty_s": float(uncertainty),
        "interval_midpoint_offset_s": float(midpoint),
        "bag_start_to_gnss_start_s": float(gnss_start - bag_start),
        "bag_end_to_gnss_end_s": float(gnss_end - bag_end),
        "fit": selected,
    }


def build_alignment() -> dict[str, object]:
    gnss, device = parse_workbook(WORKBOOK)
    design = estimate_design_yaw(gnss)
    yaw = float(design["yaw_rad"])
    conditions: dict[str, object] = {}
    for spec in CONDITION_SPECS:
        condition_id, condition_cn, _, _, _, _ = spec
        meta = json.loads((CACHE_ROOT / f"{condition_id}.json").read_text(encoding="utf-8"))
        clusters = clusters_by_frame(condition_id, len(meta["frames"]), stage="height")
        offset = estimate_condition_offset(meta, clusters, gnss[condition_cn], yaw)
        conditions[condition_id] = {"offset": offset}

    total_matches = sum(int(x["offset"]["fit"]["match_count"]) for x in conditions.values())
    supported_conditions = sum(int(x["offset"]["fit"]["match_count"] >= 3) for x in conditions.values())
    spatial_status = (
        "trajectory-geometry yaw with real-height-cluster validation"
        if total_matches >= 10 and supported_conditions >= 2
        else "trajectory-geometry yaw only; real point clouds insufficient for a unique rigid fit"
    )
    alignment = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "workbook": str(WORKBOOK.relative_to(ROOT)),
        "workbook_sha256": sha256_file(WORKBOOK),
        "gps_utc_leap_seconds": GPS_UTC_LEAP_SECONDS,
        "device_origin": device,
        "spatial": {**design, "status": spatial_status,
                    "real_cluster_match_count": total_matches,
                    "real_cluster_supported_conditions": supported_conditions},
        "conditions": conditions,
    }
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    (OUTPUT_ROOT / "alignment.json").write_text(json.dumps(alignment, ensure_ascii=False, indent=2), encoding="utf-8")
    return alignment


def write_manifest_and_frame_gt(alignment: dict[str, object]) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    gnss, _ = parse_workbook(WORKBOOK)
    yaw = float(alignment["spatial"]["yaw_rad"])
    manifest: list[dict[str, object]] = []
    frame_gt: list[dict[str, object]] = []
    preserved_stats: dict[tuple[str, int], dict[str, str]] = {}
    existing_gt = ROOT / "frame_ground_truth.csv"
    stat_columns = [
        "raw_target_points", "intensity_target_points", "height_target_points",
        "background_target_points", "neighborhood_target_points", "cluster_target_points",
        "target_point_count_bin", "target_intensity_median", "target_intensity_q25",
        "target_intensity_q75", "target_radial_velocity_median_mps",
        "target_radial_velocity_q25_mps", "target_radial_velocity_q75_mps",
        "target_range_median_m",
    ]
    if existing_gt.exists():
        with existing_gt.open("r", newline="", encoding="utf-8-sig") as fh:
            for old in csv.DictReader(fh):
                preserved_stats[(old["condition_id"], int(old["frame_index"]))] = {
                    name: old.get(name, "") for name in stat_columns
                }
    for spec in CONDITION_SPECS:
        condition_id, condition_cn, _, motion, nominal_range, nominal_speed = spec
        meta = json.loads((CACHE_ROOT / f"{condition_id}.json").read_text(encoding="utf-8"))
        points = gnss[condition_cn]
        frames = meta["frames"]
        offset = alignment["conditions"][condition_id]["offset"]
        delta = float(offset["offset_s"])
        bag_t = np.array([float(x["bag_time"]) for x in frames])
        gps_t = bag_t + delta
        enu, valid, segment = interpolate_gnss(points, gps_t)
        lidar = enu_to_lidar(enu, yaw)
        topic_rows = [x for x in meta["connections"].values() if x.get("type") == "sensor_msgs/PointCloud2"]
        topic = topic_rows[0]["topic"] if topic_rows else "UNKNOWN"
        interpolation_confidence = "LOW" if len(points) == 4 else ("MEDIUM" if len(points) < 10 else "HIGH")
        spatial_confidence = "LOW" if "only" in alignment["spatial"]["status"] else "MEDIUM"
        bag_member = normalize_zip_member_name(str(meta["bag_member"]))
        bag_source_used = f"{meta['archive_path']}::{bag_member}"
        manifest.append({
            "condition_id": condition_id, "condition": condition_cn,
            "motion_class": motion, "nominal_range_m": nominal_range,
            "nominal_speed_mps": nominal_speed,
            "archive_path": meta["archive_path"], "archive_crc32": meta["archive_crc32"],
            "bag_member": bag_member, "bag_source_used": bag_source_used,
            "bag_uncompressed_bytes": meta["bag_uncompressed_bytes"],
            "match_method": "normalized Excel title == normalized archive directory/bag filename",
            "match_confidence": "HIGH", "topic": topic,
            "message_type": "sensor_msgs/PointCloud2", "lidar_frame_count": len(frames),
            "lidar_bag_start_unix": bag_t[0], "lidar_bag_end_unix": bag_t[-1],
            "lidar_bag_start_utc": iso_utc(bag_t[0]), "lidar_bag_end_utc": iso_utc(bag_t[-1]),
            "lidar_header_start_unix": frames[0]["header_time"],
            "lidar_header_end_unix": frames[-1]["header_time"],
            "lidar_header_start_utc": iso_utc(float(frames[0]["header_time"])),
            "lidar_header_end_utc": iso_utc(float(frames[-1]["header_time"])),
            "filename_time_unix": meta["filename_time_unix"],
            "filename_time_local": meta["filename_time_local"],
            "filename_minus_bag_start_s": (float(meta["filename_time_unix"]) - bag_t[0]
                                            if meta["filename_time_unix"] is not None else ""),
            "gnss_week": points[0].gps_week, "gnss_point_count": len(points),
            "gnss_tow_start_s": points[0].tow_s, "gnss_tow_end_s": points[-1].tow_s,
            "gnss_start_unix": points[0].gps_unix_s, "gnss_end_unix": points[-1].gps_unix_s,
            "gnss_start_utc": iso_utc(points[0].gps_unix_s), "gnss_end_utc": iso_utc(points[-1].gps_unix_s),
            "gnss_minus_lidar_offset_s": delta, "offset_method": offset["offset_method"],
            "offset_confidence": offset["offset_confidence"],
            "offset_uncertainty_s": offset["offset_uncertainty_s"],
            "alignment_match_frames": offset["fit"]["match_count"],
            "alignment_rmse_m": offset["fit"]["distance_rmse_m"],
            "valid_gt_frames": int(valid.sum()),
            "interpolation_confidence": interpolation_confidence,
            "enu_to_lidar_yaw_deg": alignment["spatial"]["yaw_deg"],
            "yaw_between_roundtrip_sd_deg": alignment["spatial"]["between_roundtrip_sd_deg"],
            "translation_xyz_m": "0;0;0", "translation_status": alignment["spatial"]["translation_status"],
            "spatial_alignment_status": alignment["spatial"]["status"],
            "spatial_confidence": spatial_confidence,
        })
        source_t = np.array([p.gps_unix_s for p in points])
        for i, row in enumerate(frames):
            if valid[i]:
                j = int(segment[i])
                alpha = ((gps_t[i] - source_t[j]) / (source_t[j + 1] - source_t[j]))
                reason = "within GNSS interval; piecewise-linear interpolation"
            else:
                j, alpha = -1, np.nan
                reason = "outside GNSS interval; no extrapolation"
            output_row = {
                "condition_id": condition_id, "condition": condition_cn,
                "motion_class": motion, "frame_index": i, "topic": topic,
                "bag_time_unix": bag_t[i], "bag_time_utc": iso_utc(bag_t[i]),
                "header_time_unix": row["header_time"], "header_time_utc": iso_utc(float(row["header_time"])),
                "filename_time_unix": meta["filename_time_unix"],
                "estimated_gps_time_unix": gps_t[i], "estimated_gps_time_utc": iso_utc(gps_t[i]),
                "gps_week": points[0].gps_week,
                "gps_tow_s": points[0].tow_s + (gps_t[i] - points[0].gps_unix_s),
                "in_gnss_interval": bool(valid[i]), "gt_valid": bool(valid[i]),
                "validity_reason": reason, "interpolation_segment": j,
                "interpolation_alpha": alpha if valid[i] else "",
                "interpolation_confidence": interpolation_confidence,
                "time_offset_s": delta, "time_offset_confidence": offset["offset_confidence"],
                "time_offset_uncertainty_s": offset["offset_uncertainty_s"],
                "spatial_confidence": spatial_confidence,
                "enu_e_m": enu[i, 0] if valid[i] else "", "enu_n_m": enu[i, 1] if valid[i] else "",
                "enu_u_m": enu[i, 2] if valid[i] else "", "gt_lidar_x_m": lidar[i, 0] if valid[i] else "",
                "gt_lidar_y_m": lidar[i, 1] if valid[i] else "", "gt_lidar_z_m": lidar[i, 2] if valid[i] else "",
                "raw_target_points": "", "intensity_target_points": "", "height_target_points": "",
                "background_target_points": "", "neighborhood_target_points": "", "cluster_target_points": "",
                "target_point_count_bin": "", "target_intensity_median": "", "target_intensity_q25": "",
                "target_intensity_q75": "", "target_radial_velocity_median_mps": "",
                "target_radial_velocity_q25_mps": "", "target_radial_velocity_q75_mps": "",
                "target_range_median_m": "",
            }
            output_row.update(preserved_stats.get((condition_id, i), {}))
            frame_gt.append(output_row)
    for path, rows in [(ROOT / "gt_manifest.csv", manifest), (ROOT / "frame_ground_truth.csv", frame_gt)]:
        with path.open("w", newline="", encoding="utf-8-sig") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
    return manifest, frame_gt


def _quantile_or_blank(values: np.ndarray, q: float) -> float | str:
    return float(np.quantile(values, q)) if len(values) else ""


def _count_bin(value: int) -> str:
    if value == 0:
        return "0"
    if value == 1:
        return "1"
    if value == 2:
        return "2"
    if value <= 5:
        return "3-5"
    if value <= 10:
        return "6-10"
    return ">10"


def augment_frame_gt_with_raw_stats() -> list[dict[str, object]]:
    path = ROOT / "frame_ground_truth.csv"
    with path.open("r", newline="", encoding="utf-8-sig") as fh:
        rows: list[dict[str, object]] = list(csv.DictReader(fh))
    lookup = {(str(row["condition_id"]), int(row["frame_index"])): row for row in rows}
    for spec in CONDITION_SPECS:
        condition_id = spec[0]
        meta = json.loads((CACHE_ROOT / f"{condition_id}.json").read_text(encoding="utf-8"))
        background = stage_arrays_by_frame(condition_id, "background", len(meta["frames"]))
        neighborhood = stage_arrays_by_frame(condition_id, "neighborhood", len(meta["frames"]))
        started = time.time()
        for frame_index, (arr, pc_meta) in enumerate(iter_condition_pointclouds(spec)):
            row = lookup[(condition_id, frame_index)]
            if str(row["gt_valid"]).lower() not in ("true", "1"):
                continue
            gt = np.array([float(row["gt_lidar_x_m"]), float(row["gt_lidar_y_m"]),
                           float(row["gt_lidar_z_m"])], dtype=float)
            x = np.asarray(arr["x"], dtype=np.float32).reshape(-1)
            y_sensor = np.asarray(arr["y"], dtype=np.float32).reshape(-1)
            z_sensor = np.asarray(arr["z"], dtype=np.float32).reshape(-1)
            intensity = np.asarray(arr["intensity"], dtype=np.float32).reshape(-1)
            velocity = np.asarray(arr["velocity"], dtype=np.float32).reshape(-1)
            if int(pc_meta["point_step"]) == 80:
                y, z = z_sensor, -y_sensor
            else:
                y, z = y_sensor, z_sensor
            finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity) & np.isfinite(velocity)
            # Organized clouds use (0,0,0) as no-return placeholders.  They are
            # samples, not physical target returns, and must not inflate counts
            # when the interpolated trajectory passes through the origin.
            nonzero_return = (x * x + y * y + z * z) > 1e-8
            distance2 = (x - gt[0]) ** 2 + (y - gt[1]) ** 2 + (z - gt[2]) ** 2
            target = finite & nonzero_return & (distance2 <= GT_NEIGHBORHOOD_RADIUS_M ** 2)
            target_i = target & (intensity >= INTENSITY_MIN) & (intensity <= INTENSITY_MAX)
            target_h = target_i & (z > HEIGHT_MIN)
            raw_count = int(target.sum())
            target_intensity = intensity[target]
            target_velocity = velocity[target]
            target_range = np.sqrt(x[target] ** 2 + y[target] ** 2 + z[target] ** 2)
            bp = background[frame_index]
            np_ = neighborhood[frame_index]
            background_count = int(np.sum(np.linalg.norm(bp[:, :3] - gt, axis=1) <= GT_NEIGHBORHOOD_RADIUS_M)) if len(bp) else 0
            neighborhood_count = int(np.sum(np.linalg.norm(np_[:, :3] - gt, axis=1) <= GT_NEIGHBORHOOD_RADIUS_M)) if len(np_) else 0
            row.update({
                "raw_target_points": raw_count,
                "intensity_target_points": int(target_i.sum()),
                "height_target_points": int(target_h.sum()),
                "background_target_points": background_count,
                "neighborhood_target_points": neighborhood_count,
                # DBSCAN min_samples=1 labels every neighborhood point.
                "cluster_target_points": neighborhood_count,
                "target_point_count_bin": _count_bin(raw_count),
                "target_intensity_median": _quantile_or_blank(target_intensity, 0.5),
                "target_intensity_q25": _quantile_or_blank(target_intensity, 0.25),
                "target_intensity_q75": _quantile_or_blank(target_intensity, 0.75),
                "target_radial_velocity_median_mps": _quantile_or_blank(target_velocity, 0.5),
                "target_radial_velocity_q25_mps": _quantile_or_blank(target_velocity, 0.25),
                "target_radial_velocity_q75_mps": _quantile_or_blank(target_velocity, 0.75),
                "target_range_median_m": _quantile_or_blank(target_range, 0.5),
            })
            if frame_index == 0 or (frame_index + 1) % 25 == 0:
                print(f"raw-stats {condition_id}: frame={frame_index + 1} elapsed={time.time() - started:.1f}s", flush=True)
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    return rows


def run_current_baseline() -> list[dict[str, object]]:
    """Run the repository's current main MultiTargetTracker without editing it."""
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from target_track_without_initialize_and_grid import MultiTargetTracker

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    all_states: list[dict[str, object]] = []
    for spec in CONDITION_SPECS:
        condition_id = spec[0]
        meta = json.loads((CACHE_ROOT / f"{condition_id}.json").read_text(encoding="utf-8"))
        dynamic = stage_arrays_by_frame(condition_id, "neighborhood", len(meta["frames"]))
        tracker = MultiTargetTracker(
            min_points=1, search_radius=15.0, radial_threshold=5.0,
            max_association_distance=30.0, resolution=0.5,
        )
        tracker_initialized = False
        initialization_frame: int | None = None
        log_path = OUTPUT_ROOT / f"baseline_{condition_id}.log"
        with log_path.open("w", encoding="utf-8") as log, redirect_stdout(log):
            for frame_index, values in enumerate(dynamic):
                points = values[:, :3]
                velocities = values[:, 4]
                before = {int(t.target_id): len(t.track_history) for t in tracker.trackers if t.is_active}
                if not tracker_initialized:
                    if int(meta["frames"][frame_index]["background_frame_count"]) <= BACKGROUND_INIT_FRAMES:
                        continue
                    if len(points) < 5:
                        continue
                    initialized = tracker.initialize_by_global_clustering(points, velocities)
                    if not initialized:
                        continue
                    tracker_initialized = True
                    initialization_frame = frame_index
                tracker.track_frame(points, velocities)
                for target in tracker.trackers:
                    if not target.is_active:
                        continue
                    target_id = int(target.target_id)
                    history_before = before.get(target_id)
                    if history_before is None or len(target.track_history) > history_before:
                        state_kind = "predicted" if bool(target.is_predicted) else "measured"
                    else:
                        # Current tracker returns immediately on an empty point
                        # frame.  This is a carried state, not a measurement.
                        state_kind = "stale"
                    all_states.append({
                        "condition_id": condition_id, "frame_index": frame_index,
                        "track_id": target_id, "state_kind": state_kind,
                        "center_x_m": float(target.center_3d[0]),
                        "center_y_m": float(target.center_3d[1]),
                        "center_z_m": float(target.center_3d[2]),
                        "num_points": int(target.num_points),
                        "radial_velocity_mps": float(target.radial_velocity),
                        "lost_count": int(target.lost_count),
                        "initialization_frame": initialization_frame,
                    })
        print(f"baseline {condition_id}: states={sum(x['condition_id'] == condition_id for x in all_states)} "
              f"tracks_created={tracker.target_id_counter} init_frame={initialization_frame}", flush=True)
    states_path = OUTPUT_ROOT / "baseline_states.csv"
    fields = list(all_states[0]) if all_states else [
        "condition_id", "frame_index", "track_id", "state_kind", "center_x_m",
        "center_y_m", "center_z_m", "num_points", "radial_velocity_mps",
        "lost_count", "initialization_frame",
    ]
    with states_path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader(); writer.writerows(all_states)
    return all_states


def _longest_false_run(found: list[bool]) -> int:
    best = current = 0
    for value in found:
        if value:
            current = 0
        else:
            current += 1
            best = max(best, current)
    return best


def _gap_metrics(found: list[bool]) -> tuple[int, int, int]:
    gaps = recovered = 0
    i = 0
    while i < len(found):
        if found[i]:
            i += 1
            continue
        start = i
        while i < len(found) and not found[i]:
            i += 1
        # A gap is truth-relative only when preceded by a detection.  It is
        # recovered when another detection follows.
        if start > 0 and found[start - 1]:
            gaps += 1
            if i < len(found) and found[i]:
                recovered += 1
    return gaps, recovered, _longest_false_run(found)


def evaluate_baseline(states: list[dict[str, object]]) -> list[dict[str, object]]:
    with (ROOT / "frame_ground_truth.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        gt_rows = list(csv.DictReader(fh))
    gt_by_condition: dict[str, list[dict[str, object]]] = {}
    for row in gt_rows:
        if str(row["gt_valid"]).lower() in ("true", "1"):
            gt_by_condition.setdefault(str(row["condition_id"]), []).append(row)
    state_by_key: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in states:
        state_by_key.setdefault((str(row["condition_id"]), int(row["frame_index"])), []).append(row)

    metrics: list[dict[str, object]] = []
    for spec in CONDITION_SPECS:
        condition_id, condition_cn, _, motion, _, _ = spec
        truth = sorted(gt_by_condition.get(condition_id, []), key=lambda x: int(x["frame_index"]))
        for subset in ("measured", "predicted", "stale", "all"):
            frame_found: list[bool] = []
            errors: list[float] = []
            false_detections = 0
            seen_ids: set[int] = set()
            matched_ids: set[int] = set()
            matched_frames_by_id: dict[int, set[int]] = {}
            duplicate_ids: set[int] = set()
            duplicate_frames = 0
            evaluated_states = 0
            for gt in truth:
                frame_index = int(gt["frame_index"])
                candidates = state_by_key.get((condition_id, frame_index), [])
                if subset != "all":
                    candidates = [x for x in candidates if x["state_kind"] == subset]
                gt_xyz = np.array([float(gt["gt_lidar_x_m"]), float(gt["gt_lidar_y_m"]),
                                   float(gt["gt_lidar_z_m"])])
                distances = []
                for state in candidates:
                    seen_ids.add(int(state["track_id"]))
                    center = np.array([state["center_x_m"], state["center_y_m"], state["center_z_m"]], dtype=float)
                    distances.append(float(np.linalg.norm(center - gt_xyz)))
                evaluated_states += len(candidates)
                matched = [j for j, distance in enumerate(distances) if distance <= TRACK_MATCH_RADIUS_M]
                found = bool(matched)
                frame_found.append(found)
                if found:
                    best = min(matched, key=lambda j: distances[j])
                    errors.append(distances[best])
                    best_id = int(candidates[best]["track_id"])
                    matched_ids.add(best_id)
                    matched_frames_by_id.setdefault(best_id, set()).add(frame_index)
                    if len(matched) > 1:
                        duplicate_frames += 1
                        for j in matched:
                            if j != best:
                                duplicate_ids.add(int(candidates[j]["track_id"]))
                false_detections += len(candidates) - len(matched)
            gaps, recovered, max_miss = _gap_metrics(frame_found)
            best_track_frames = max((len(x) for x in matched_frames_by_id.values()), default=0)
            e = np.asarray(errors, dtype=float)
            false_track_ids = seen_ids - matched_ids
            metrics.append({
                "condition_id": condition_id, "condition": condition_cn,
                "motion_class": motion, "state_subset": subset,
                "evaluation_status": "PROVISIONAL_LOW_SPATIAL_CONFIDENCE",
                "gt_frames": len(truth), "evaluated_track_states": evaluated_states,
                "detected_frames": int(sum(frame_found)),
                "frame_detection_recall": float(np.mean(frame_found)) if truth else "",
                "false_detections": false_detections,
                "false_detections_per_gt_frame": false_detections / len(truth) if truth else "",
                "unique_track_ids": len(seen_ids), "matched_track_ids": len(matched_ids),
                "false_tracks": len(false_track_ids),
                "false_track_ids": ";".join(map(str, sorted(false_track_ids))),
                "track_completeness_best_id": best_track_frames / len(truth) if truth else "",
                "center_error_count": len(e),
                "center_error_mean_m": float(np.mean(e)) if len(e) else "",
                "center_error_median_m": float(np.median(e)) if len(e) else "",
                "center_error_rmse_m": float(np.sqrt(np.mean(e ** 2))) if len(e) else "",
                "center_error_p95_m": float(np.quantile(e, 0.95)) if len(e) else "",
                "consecutive_miss_max_frames": max_miss,
                "detection_gap_count": gaps, "recovered_gap_count": recovered,
                "gap_recovery_rate": recovered / gaps if gaps else "",
                "duplicate_track_ids": len(duplicate_ids),
                "duplicate_track_id_list": ";".join(map(str, sorted(duplicate_ids))),
                "duplicate_id_frames": duplicate_frames,
                "track_match_radius_m": TRACK_MATCH_RADIUS_M,
            })
    path = ROOT / "benchmark_v0.csv"
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(metrics[0]))
        writer.writeheader(); writer.writerows(metrics)
    return metrics


def diagnose_100m_cross_10() -> dict[str, object]:
    condition_id = "100m_cross_10"
    spec = next(x for x in CONDITION_SPECS if x[0] == condition_id)
    with (ROOT / "frame_ground_truth.csv").open("r", newline="", encoding="utf-8-sig") as fh:
        gt_rows = {int(x["frame_index"]): x for x in csv.DictReader(fh)
                   if x["condition_id"] == condition_id}
    gnss, _ = parse_workbook(WORKBOOK)
    points = gnss[spec[1]]
    radii = np.array([math.hypot(x.enu_e_m, x.enu_n_m) for x in points])
    heights = np.array([x.enu_u_m for x in points])
    broad_r = (float(radii.min() - 15.0), float(radii.max() + 15.0))
    broad_z = (float(heights.min() - 3.0), float(heights.max() + 3.0))
    frames = []
    broad_points = []
    for frame_index, (arr, meta) in enumerate(iter_condition_pointclouds(spec)):
        x = np.asarray(arr["x"], dtype=np.float32).reshape(-1)
        y_sensor = np.asarray(arr["y"], dtype=np.float32).reshape(-1)
        z_sensor = np.asarray(arr["z"], dtype=np.float32).reshape(-1)
        intensity = np.asarray(arr["intensity"], dtype=np.float32).reshape(-1)
        velocity = np.asarray(arr["velocity"], dtype=np.float32).reshape(-1)
        y, z = (z_sensor, -y_sensor) if int(meta["point_step"]) == 80 else (y_sensor, z_sensor)
        horizontal = np.hypot(x, y)
        valid_return = (np.isfinite(horizontal) & np.isfinite(z) & np.isfinite(intensity)
                        & np.isfinite(velocity) & ((x * x + y * y + z * z) > 1e-8))
        broad = (valid_return & (horizontal >= broad_r[0]) & (horizontal <= broad_r[1])
                 & (z >= broad_z[0]) & (z <= broad_z[1]))
        broad_i = broad & (intensity >= INTENSITY_MIN) & (intensity <= INTENSITY_MAX)
        row = gt_rows[frame_index]
        if str(row["gt_valid"]).lower() in ("true", "1"):
            gt_r = math.hypot(float(row["gt_lidar_x_m"]), float(row["gt_lidar_y_m"]))
            gt_z = float(row["gt_lidar_z_m"])
            ann5 = valid_return & (np.abs(horizontal - gt_r) <= 5.0) & (np.abs(z - gt_z) <= 5.0)
            ann10 = valid_return & (np.abs(horizontal - gt_r) <= 10.0) & (np.abs(z - gt_z) <= 10.0)
        else:
            ann5 = ann10 = np.zeros(len(x), dtype=bool)
        if np.any(broad):
            for idx in np.flatnonzero(broad):
                broad_points.append({
                    "frame_index": frame_index, "x_m": float(x[idx]), "y_m": float(y[idx]),
                    "z_m": float(z[idx]), "horizontal_range_m": float(horizontal[idx]),
                    "intensity": float(intensity[idx]), "radial_velocity_mps": float(velocity[idx]),
                })
        frames.append({
            "frame_index": frame_index, "bag_time": float(meta["bag_time"]),
            "gt_valid": str(row["gt_valid"]).lower() in ("true", "1"),
            "azimuth_invariant_5m_count": int(ann5.sum()),
            "azimuth_invariant_10m_count": int(ann10.sum()),
            "broad_range_height_count": int(broad.sum()),
            "broad_after_intensity_count": int(broad_i.sum()),
        })
    result = {
        "condition_id": condition_id,
        "purpose": "yaw-invariant raw-return check; annulus/shell counts are not point labels",
        "broad_horizontal_range_m": list(broad_r), "broad_height_m": list(broad_z),
        "gt_shell_half_widths_m": [5.0, 10.0],
        "aggregate": {
            "valid_gt_frames": int(sum(x["gt_valid"] for x in frames)),
            "azimuth_invariant_5m_points": int(sum(x["azimuth_invariant_5m_count"] for x in frames)),
            "azimuth_invariant_10m_points": int(sum(x["azimuth_invariant_10m_count"] for x in frames)),
            "broad_range_height_points": int(sum(x["broad_range_height_count"] for x in frames)),
            "broad_after_intensity_points": int(sum(x["broad_after_intensity_count"] for x in frames)),
        },
        "broad_points": broad_points, "frames": frames,
    }
    path = OUTPUT_ROOT / "diagnostic_100m_cross_10.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("extract", "align", "raw-stats", "benchmark", "diagnose-100m"), default="extract")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    if args.phase == "extract":
        rows = extract_all(force=args.force)
        print(json.dumps({r["condition_id"]: len(r["frames"]) for r in rows}, indent=2))
    elif args.phase == "align":
        alignment = build_alignment()
        manifest, frame_gt = write_manifest_and_frame_gt(alignment)
        print(json.dumps({"manifest_rows": len(manifest), "frame_gt_rows": len(frame_gt),
                          "spatial": alignment["spatial"]}, ensure_ascii=False, indent=2))
    elif args.phase == "raw-stats":
        rows = augment_frame_gt_with_raw_stats()
        print(json.dumps({"frame_gt_rows": len(rows)}, indent=2))
    elif args.phase == "benchmark":
        states = run_current_baseline()
        metrics = evaluate_baseline(states)
        print(json.dumps({"baseline_states": len(states), "benchmark_rows": len(metrics)}, indent=2))
    elif args.phase == "diagnose-100m":
        print(json.dumps(diagnose_100m_cross_10(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
