"""Lossless rich-cloud I/O and field diagnostics for an exploratory study.

This module has no detection, tracking, calibration fitting or score policy.
Field names do not establish physical units, time anchoring or sensor semantics.
An exhausted bag iterator supplies a cryptographic identity of the entire
uncompressed archive member; an interrupted iterator supplies only a partial
hash, explicitly distinguished from a complete raw-file identity.
"""
from __future__ import annotations

from collections import Counter
import hashlib
from pathlib import Path
import struct
from typing import Iterator
import zipfile

import numpy as np


ROS_DTYPES = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2",
    5: "i4", 6: "u4", 7: "f4", 8: "f8",
}
ROS_TYPE_NAMES = {
    1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
    5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64",
}
QUANTILES = (0.05, 0.25, 0.5, 0.75, 0.95)


class CloudDecodeError(ValueError):
    """Malformed PointCloud2 serialization or invalid point/row layout."""


class _Cursor:
    def __init__(self, payload: bytes):
        self.payload = memoryview(payload)
        self.offset = 0

    def take(self, size: int) -> memoryview:
        if size < 0 or self.offset + size > len(self.payload):
            raise CloudDecodeError(
                f"truncated payload at {self.offset}: need {size}, "
                f"have {len(self.payload) - self.offset}"
            )
        value = self.payload[self.offset:self.offset + size]
        self.offset += size
        return value

    def unpack(self, fmt: str):
        return struct.unpack(fmt, self.take(struct.calcsize(fmt)))

    def string(self) -> str:
        (size,) = self.unpack("<I")
        try:
            return bytes(self.take(size)).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise CloudDecodeError("invalid UTF-8 ROS string") from exc


def decode_rich_pointcloud2(payload: bytes) -> tuple[np.ndarray, dict]:
    """Decode ROS1 PointCloud2 without discarding padding or rich-field types.

    ROS message serialization uses little-endian lengths/header integers;
    ``is_bigendian`` controls the point-data byte order independently. The
    returned flat structured array is a view when rows are contiguous and a
    row-correct copy when row padding prevents a flat view. Native field
    integer types and FLOAT64 precision are preserved; no geometric filters
    are applied. Every PointField count, offset and datatype is recorded.
    """
    c = _Cursor(payload)
    seq, sec, nsec = c.unpack("<III")
    if nsec >= 1_000_000_000:
        raise CloudDecodeError(f"invalid header nanoseconds: {nsec}")
    frame_id = c.string()
    height, width, field_count = c.unpack("<III")
    fields = []
    for _ in range(field_count):
        name = c.string()
        offset, datatype, count = c.unpack("<IBI")
        if not name or datatype not in ROS_DTYPES or count < 1:
            raise CloudDecodeError(
                f"invalid PointField name={name!r}, datatype={datatype}, count={count}"
            )
        fields.append({"name": name, "offset": offset,
                       "datatype": datatype, "datatype_name": ROS_TYPE_NAMES[datatype],
                       "count": count})
    (endian_flag,) = c.unpack("<B")
    if endian_flag not in (0, 1):
        raise CloudDecodeError(f"invalid endian flag: {endian_flag}")
    point_step, row_step, data_len = c.unpack("<III")
    data = c.take(data_len)
    (dense_flag,) = c.unpack("<B")
    if dense_flag not in (0, 1):
        raise CloudDecodeError(f"invalid is_dense flag: {dense_flag}")
    if c.offset != len(c.payload):
        raise CloudDecodeError(f"unexpected trailing serialized bytes: {len(c.payload) - c.offset}")
    if height * width and point_step == 0:
        raise CloudDecodeError("nonempty cloud with point_step=0")
    if row_step < width * point_step:
        raise CloudDecodeError("row_step is smaller than width*point_step")
    if data_len != height * row_step:
        raise CloudDecodeError("data length differs from height*row_step")

    names, formats, offsets = [], [], []
    endian = ">" if endian_flag else "<"
    for field in fields:
        name, offset, count = field["name"], field["offset"], field["count"]
        if name in names:
            raise CloudDecodeError(f"duplicate PointField name: {name}")
        dtype = np.dtype(endian + ROS_DTYPES[field["datatype"]])
        if offset + count * dtype.itemsize > point_step:
            raise CloudDecodeError(f"field {name!r} exceeds point_step")
        names.append(name)
        formats.append(dtype if count == 1 else (dtype, (count,)))
        offsets.append(offset)
        field["numpy_dtype"] = dtype.str
        field["semantics_status"] = "UNKNOWN"
        field["unit_status"] = "UNKNOWN"
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                      "itemsize": point_step})
    organized = np.ndarray((height, width), dtype=dtype, buffer=data,
                           strides=(row_step, point_step))
    arr = organized.reshape(-1)
    meta = {
        "seq": int(seq), "header_sec": int(sec), "header_nsec": int(nsec),
        "header_time": float(sec) + float(nsec) * 1e-9,
        "frame_id": frame_id, "height": int(height), "width": int(width),
        "point_step": int(point_step), "row_step": int(row_step),
        "data_length": int(data_len), "is_bigendian": bool(endian_flag),
        "is_dense": bool(dense_flag), "fields": names,
        "field_schema": fields,
        "row_padding_bytes": int(row_step - width * point_step),
        "row_padding_preserved": True,
        "flat_array_is_view": bool(np.shares_memory(arr, organized)),
        "physical_field_semantics": "UNKNOWN_UNLESS_SEPARATELY_DOCUMENTED",
    }
    return arr, meta


class _HashingReader:
    """Sequential reads only: skipped bytes must also enter the member hash."""
    def __init__(self, stream):
        self.stream = stream
        self.hash = hashlib.sha256()
        self.bytes_read = 0

    def read(self, size: int) -> bytes:
        value = self.stream.read(size)
        self.hash.update(value)
        self.bytes_read += len(value)
        return value

    def exact(self, size: int) -> bytes:
        value = self.read(size)
        if len(value) != size:
            raise EOFError(f"short member read: got {len(value)}, expected {size}")
        return value

    def skip(self, size: int) -> None:
        remaining = size
        while remaining:
            block = self.exact(min(remaining, 8 * 1024 * 1024))
            remaining -= len(block)


def _record_fields(raw: bytes) -> dict[str, bytes]:
    fields = {}
    cursor = 0
    while cursor < len(raw):
        if cursor + 4 > len(raw):
            raise ValueError("truncated ROS bag header length")
        size = struct.unpack_from("<I", raw, cursor)[0]
        cursor += 4
        if cursor + size > len(raw):
            raise ValueError("truncated ROS bag header field")
        value = raw[cursor:cursor + size]
        cursor += size
        if b"=" not in value:
            raise ValueError("ROS bag header field without '='")
        key, value = value.split(b"=", 1)
        key = key.decode("utf-8")
        if key in fields:
            raise ValueError(f"duplicate ROS bag header key: {key}")
        fields[key] = value
    return fields


def _uint(raw: bytes) -> int:
    if len(raw) != 4:
        raise ValueError("ROS bag uint32 must contain four bytes")
    return struct.unpack("<I", raw)[0]


def _time(raw: bytes) -> tuple[int, int]:
    if len(raw) != 8:
        raise ValueError("ROS bag time must contain eight bytes")
    sec, nsec = struct.unpack("<II", raw)
    if nsec >= 1_000_000_000:
        raise ValueError("invalid ROS bag record nanoseconds")
    return sec, nsec


def _record_head(reader: _HashingReader, first: bytes | None = None):
    size = _uint(reader.exact(4) if first is None else first)
    if size > 1024 * 1024:
        raise ValueError(f"unreasonable ROS bag record header length: {size}")
    fields = _record_fields(reader.exact(size))
    length = _uint(reader.exact(4))
    if len(fields.get("op", b"")) != 1:
        raise ValueError("missing or invalid ROS bag record op")
    return fields, length


def _connection(header: dict, raw: bytes) -> tuple[int, dict]:
    values = _record_fields(raw)
    conn = _uint(header["conn"])
    return conn, {
        "topic": header["topic"].decode("utf-8"),
        "type": values.get("type", b"").decode("utf-8"),
        "md5sum": values.get("md5sum", b"").decode("utf-8"),
        "message_definition": values.get("message_definition", b"").decode("utf-8"),
    }


def iter_rich_frames(archive_path: str | Path, member_name: str | None = None,
                     audit: dict | None = None) -> Iterator[tuple[np.ndarray, dict]]:
    """Stream cloud frames and hash every uncompressed raw bag-member byte.

    Caller must exhaust this generator to obtain ``bag_member_sha256`` and
    ``complete_member_hash=True``. Non-cloud messages are consumed but never
    decoded as PointCloud2. Malformed cloud messages, absent connection
    metadata and unsupported ROS chunk compression raise and are recorded;
    they are not silently converted into missing/zero-return observations.
    ``archive_sha256`` is not computed here: the member identity is different
    from the enclosing compressed ZIP identity.
    """
    if audit is None:
        audit = {}
    archive_path = Path(archive_path)
    audit.update({
        "archive_path": str(archive_path), "archive_bytes": archive_path.stat().st_size,
        "bag_member_sha256": None, "complete_member_hash": False,
        "zip_crc_verified_at_eof": False, "connections": {}, "errors": [],
        "frame_count": 0, "message_count": 0, "noncloud_message_count": 0,
        "bytes_read": 0, "sha256_partial": None,
        "hash_method": "SHA256_OF_EVERY_UNCOMPRESSED_MEMBER_BYTE_SEQUENTIAL_READ",
    })
    reader = None
    top_counts, inner_counts = Counter(), Counter()
    try:
        with zipfile.ZipFile(archive_path) as zf:
            candidates = [info.filename for info in zf.infolist()
                          if not info.is_dir() and info.filename.endswith(".bag")]
            if member_name is None:
                if len(candidates) != 1:
                    raise ValueError(f"expected one .bag member, found {len(candidates)}")
                member_name = candidates[0]
            if member_name not in candidates:
                raise ValueError(f"specified .bag member not found: {member_name!r}")
            info = zf.getinfo(member_name)
            audit.update({
                "bag_member": member_name, "bag_uncompressed_bytes": info.file_size,
                "bag_compressed_bytes": info.compress_size,
                "zip_member_crc32": f"{info.CRC:08x}",
                "zip_compression": info.compress_type,
            })
            with zf.open(info, "r") as stream:
                reader = _HashingReader(stream)
                if reader.exact(13) != b"#ROSBAG V2.0\n":
                    raise ValueError("not a ROS bag V2.0 member")
                while True:
                    first = reader.read(4)
                    if not first:
                        break
                    header, data_len = _record_head(reader, first)
                    op = header["op"][0]
                    top_counts[str(op)] += 1
                    if op == 7:
                        conn, values = _connection(header, reader.exact(data_len))
                        prior = audit["connections"].get(str(conn))
                        if prior is not None and prior != values:
                            raise ValueError(f"connection {conn} changed metadata")
                        audit["connections"][str(conn)] = values
                        continue
                    if op != 5:
                        reader.skip(data_len)
                        continue
                    compression = header.get("compression", b"").decode("utf-8")
                    if compression != "none":
                        raise NotImplementedError(f"unsupported ROS chunk compression: {compression!r}")
                    if "size" in header and _uint(header["size"]) != data_len:
                        raise ValueError("uncompressed ROS chunk size differs from record data length")
                    chunk_start = reader.bytes_read
                    while reader.bytes_read - chunk_start < data_len:
                        inner, inner_len = _record_head(reader)
                        if reader.bytes_read - chunk_start + inner_len > data_len:
                            raise ValueError("inner record crosses ROS chunk boundary")
                        inner_op = inner["op"][0]
                        inner_counts[str(inner_op)] += 1
                        if inner_op == 7:
                            conn, values = _connection(inner, reader.exact(inner_len))
                            prior = audit["connections"].get(str(conn))
                            if prior is not None and prior != values:
                                raise ValueError(f"connection {conn} changed metadata")
                            audit["connections"][str(conn)] = values
                            continue
                        if inner_op != 2:
                            reader.skip(inner_len)
                            continue
                        audit["message_count"] += 1
                        conn = _uint(inner["conn"])
                        declaration = audit["connections"].get(str(conn))
                        if declaration is None:
                            raise ValueError(f"message connection {conn} has no prior declaration")
                        if declaration["type"] != "sensor_msgs/PointCloud2":
                            audit["noncloud_message_count"] += 1
                            reader.skip(inner_len)
                            continue
                        payload = reader.exact(inner_len)
                        arr, meta = decode_rich_pointcloud2(payload)
                        sec, nsec = _time(inner["time"])
                        meta.update({
                            "bag_time": float(sec) + float(nsec) * 1e-9,
                            "bag_time_sec": sec, "bag_time_nsec": nsec,
                            "frame_index": audit["frame_count"], "conn": conn,
                            "topic": declaration["topic"],
                            "message_type": declaration["type"],
                        })
                        audit["frame_count"] += 1
                        yield arr, meta
                    if reader.bytes_read - chunk_start != data_len:
                        raise ValueError("ROS chunk byte accounting mismatch")
                if reader.bytes_read != info.file_size:
                    raise EOFError("whole-member read length differs from ZIP declared length")
                # ZipExtFile checks its advertised CRC while consuming EOF.
                audit["zip_crc_verified_at_eof"] = True
                audit["complete_member_hash"] = True
                audit["bag_member_sha256"] = reader.hash.hexdigest()
    except Exception as exc:
        audit["errors"].append({
            "error_type": type(exc).__name__, "message": str(exc),
            "bytes_read_before_error": reader.bytes_read if reader else 0,
            "frames_yielded_before_error": audit["frame_count"],
        })
        raise
    finally:
        if reader is not None:
            audit["bytes_read"] = reader.bytes_read
            audit["sha256_partial"] = reader.hash.hexdigest()
        audit["top_record_counts"] = dict(top_counts)
        audit["inner_record_counts"] = dict(inner_counts)


def _native_number(value):
    return value.item() if isinstance(value, np.generic) else value


def _finite_summary(values: np.ndarray, quantiles: bool = True) -> dict:
    values = np.asarray(values).reshape(-1)
    finite = np.isfinite(values)
    use = values[finite]
    out = {
        "value_count": int(values.size), "finite_count": int(finite.sum()),
        "nonfinite_count": int((~finite).sum()),
        "nan_count": int(np.isnan(values).sum()),
        "positive_inf_count": int(np.isposinf(values).sum()),
        "negative_inf_count": int(np.isneginf(values).sum()),
        "zero_count": int((values == 0).sum()),
        "minus_one_count": int((values == -1).sum()),
        "finite_min": _native_number(use.min()) if len(use) else None,
        "finite_max": _native_number(use.max()) if len(use) else None,
        "finite_mean": float(np.mean(use, dtype=np.float64)) if len(use) else None,
    }
    if quantiles:
        qs = np.quantile(use, QUANTILES) if len(use) else [None] * len(QUANTILES)
        out.update({name: None if value is None else float(value)
                    for name, value in zip(("p05", "p25", "p50", "p75", "p95"), qs)})
    return out


def scalar_field_audit(arr: np.ndarray, valid_xyz_mask: np.ndarray) -> list[dict]:
    """Exact full-frame field counts plus exact valid-XYZ quantile summaries.

    No intensity/Doppler filter is applied by this helper. ``valid_xyz_mask``
    must be the caller's finite/nonzero XYZ mask. Multi-count fields pool their
    scalar components for this field audit; they are not interpreted as a
    physical vector. No stride approximation or random sampling is used.
    """
    mask = np.asarray(valid_xyz_mask, dtype=bool)
    if mask.ndim != 1 or len(mask) != len(arr):
        raise ValueError("valid_xyz_mask must have one entry per stored point slot")
    rows = []
    for name in arr.dtype.names or ():
        values = np.asarray(arr[name])
        field_dtype, offset = arr.dtype.fields[name][:2]
        base, shape = field_dtype.subdtype if field_dtype.subdtype else (field_dtype, ())
        count = int(np.prod(shape)) if shape else 1
        row = {
            "field": name, "numpy_dtype": base.str, "offset_bytes": int(offset),
            "component_count": count, "stored_point_slots": len(arr),
            "finite_nonzero_xyz_point_slots": int(mask.sum()),
            "semantics_status": "UNKNOWN", "unit_status": "UNKNOWN",
            "multicomponent_summary": "SCALAR_COMPONENTS_POOLED_NOT_VECTOR_MAGNITUDE",
            "quantile_method": "EXACT_NUMPY_LINEAR_ALL_FINITE_VALID_XYZ_VALUES",
        }
        row.update({f"all_slots_{k}": v for k, v in
                    _finite_summary(values, quantiles=False).items()})
        row.update({f"finite_nonzero_xyz_{k}": v for k, v in
                    _finite_summary(values[mask], quantiles=True).items()})
        row["constant_all_finite_values"] = (
            row["all_slots_finite_min"] == row["all_slots_finite_max"]
            if row["all_slots_finite_count"] else None
        )
        rows.append(row)
    return rows


def field_relationship_audit(arr: np.ndarray, mask: np.ndarray,
                             meta: dict | None = None) -> dict:
    """Empirical relationships only, never claims about sensor physical truth.

    Pass decoder metadata to inspect organized row/ring/time relationships.
    Without it organized layout relationships are UNKNOWN, not inferred from
    the ring name or distinct ring count. All summaries use complete relevant
    arrays, without bounded-stride quantile approximations.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 1 or len(mask) != len(arr):
        raise ValueError("mask must have one entry per stored point slot")
    names = set(arr.dtype.names or ())
    out = {"evidence_type": "EMPIRICAL_SERIALIZED_FIELD_RELATIONSHIP_ONLY",
           "physical_semantics_status": "UNKNOWN", "stored_point_slots": len(arr),
           "valid_xyz_slots": int(mask.sum())}
    scalar = lambda name: name in names and np.asarray(arr[name]).ndim == 1
    height, width = (int(meta["height"]), int(meta["width"])) if meta else (0, 0)
    organized = height > 0 and width > 0 and height * width == len(arr)
    out["organized_layout_status"] = "VERIFIED_METADATA_LAYOUT" if organized else "UNKNOWN"
    if scalar("ring"):
        ring = arr["ring"]
        out["ring_unique_values_all_slots"] = [_native_number(v) for v in np.unique(ring)]
        if organized:
            rows = ring.reshape(height, width)
            out["ring_row_index_equal_count"] = int((rows == np.arange(height)[:, None]).sum())
            out["ring_row_index_equal_fraction"] = float(np.mean(rows == np.arange(height)[:, None]))
            out["ring_unique_values_by_row"] = [
                [_native_number(v) for v in np.unique(row)] for row in rows
            ]
    if scalar("time"):
        out["point_time_summary_all_slots"] = _finite_summary(arr["time"])
        out["point_time_anchor_unit_status"] = "UNKNOWN_NO_ABSOLUTE_OR_DESKEW_CONVERSION"
        if organized:
            rows = arr["time"].reshape(height, width)
            index = np.arange(width, dtype=np.float64)
            out["point_time_by_row"] = []
            for row in rows:
                valid = np.isfinite(row)
                corr = (float(np.corrcoef(index[valid], row[valid])[0, 1])
                        if valid.sum() > 1 and np.ptp(row[valid]) > 0 else None)
                d = np.diff(row.astype(np.float64))
                out["point_time_by_row"].append({
                    "finite_min": float(row[valid].min()) if valid.any() else None,
                    "finite_max": float(row[valid].max()) if valid.any() else None,
                    "backwards_adjacent_steps": int((d < 0).sum()),
                    "correlation_with_column_index": corr,
                    "exactly_equal_to_first_row_fraction": float(np.mean(row == rows[0])),
                })
    if scalar("velocity") and scalar("velocity_direction"):
        v, direction = arr["velocity"][mask], arr["velocity_direction"][mask]
        good = np.isfinite(v) & np.isfinite(direction)
        use = good & (v != 0)
        out["velocity_direction_nonzero_sign_compared_count"] = int(use.sum())
        out["velocity_direction_nonzero_sign_equal_count"] = int((np.sign(v[use]) == direction[use]).sum())
        out["velocity_direction_nonzero_sign_equal_fraction"] = (
            float(np.mean(np.sign(v[use]) == direction[use])) if use.any() else None
        )
        out["velocity_direction_at_zero_velocity_values"] = [
            _native_number(x) for x in np.unique(direction[good & (v == 0)])
        ]
    if scalar("velocity") and scalar("velocity0"):
        v, v0 = arr["velocity"][mask], arr["velocity0"][mask]
        good = np.isfinite(v) & np.isfinite(v0)
        out["velocity0_minus_velocity"] = _finite_summary(
            v0[good].astype(np.float64) - v[good].astype(np.float64)
        )
        out["velocity0_equal_velocity_fraction"] = (
            float(np.mean(v0[good] == v[good])) if good.any() else None
        )
    if all(scalar(name) for name in ("x", "y", "z", "radius")):
        xyz = np.column_stack([arr[name][mask].astype(np.float64) for name in ("x", "y", "z")])
        radius = arr["radius"][mask].astype(np.float64)
        good = np.isfinite(xyz).all(axis=1) & np.isfinite(radius)
        out["radius_minus_xyz_norm"] = _finite_summary(
            radius[good] - np.linalg.norm(xyz[good], axis=1)
        )
    return out
