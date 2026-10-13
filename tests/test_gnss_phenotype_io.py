"""Synthetic parsing/provenance checks, not detector or method experiments."""
import hashlib
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest
import zipfile

import numpy as np


spec = importlib.util.spec_from_file_location(
    "gnss_phenotype_io", Path(__file__).parents[1] / "scripts/gnss_phenotype_io.py"
)
io = importlib.util.module_from_spec(spec)
spec.loader.exec_module(io)


def _string(text):
    raw = text.encode()
    return struct.pack("<I", len(raw)) + raw


def cloud_payload(big=False, padded=False):
    """Two rows with an integer ring, FLOAT64 time and multi-count field."""
    endian = ">" if big else "<"
    fields = [("x", 0, 7, 1), ("ring", 4, 6, 1),
              ("time", 8, 8, 1), ("pair", 16, 3, 2)]
    dtype = np.dtype({"names": [f[0] for f in fields],
                      "formats": [endian + "f4", endian + "u4", endian + "f8", (endian + "i2", (2,))],
                      "offsets": [f[1] for f in fields], "itemsize": 20})
    arr = np.zeros((2, 2), dtype=dtype)
    arr["x"] = [[1, 2], [3, 4]]
    arr["ring"] = [[0, 0], [4_000_000_001, 4_000_000_001]]
    arr["time"] = [[1.0, 1.0 + 1e-10], [2.0, 2.0 + 1e-10]]
    arr["pair"] = [[[1, -2], [3, -4]], [[5, -6], [7, -8]]]
    data = b"".join(row.tobytes() + (b"PAD!" if padded else b"") for row in arr)
    payload = struct.pack("<III", 17, 123, 456) + _string("sensor")
    payload += struct.pack("<III", 2, 2, len(fields))
    for name, offset, datatype, count in fields:
        payload += _string(name) + struct.pack("<IBI", offset, datatype, count)
    payload += struct.pack("<BIII", int(big), 20, 44 if padded else 40, len(data))
    return payload + data + b"\1", arr


def _header(fields):
    out = b""
    for key, value in fields.items():
        item = key.encode() + b"=" + value
        out += struct.pack("<I", len(item)) + item
    return out


def _record(fields, data):
    header = _header(fields)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def bag_bytes(payload=None, nested_connection=False, compression=b"none"):
    if payload is None:
        payload, _ = cloud_payload()
    conn_header = {"op": b"\7", "conn": struct.pack("<I", 0), "topic": b"/rich"}
    conn = _record(conn_header, _header({"type": b"sensor_msgs/PointCloud2", "md5sum": b"example"}))
    other = _record({"op": b"\7", "conn": struct.pack("<I", 1), "topic": b"/other"},
                    _header({"type": b"std_msgs/String", "md5sum": b"other"}))
    chunk = (conn if nested_connection else b"")
    chunk += _record({"op": b"\2", "conn": struct.pack("<I", 0),
                      "time": struct.pack("<II", 456, 789)}, payload)
    chunk += _record({"op": b"\2", "conn": struct.pack("<I", 1),
                      "time": struct.pack("<II", 457, 0)}, b"DO NOT DECODE AS A CLOUD")
    # Include ignored data before/after the cloud to test whole-byte coverage.
    bag = b"#ROSBAG V2.0\n" + _record({"op": b"\3"}, b"FILE HEADER PADDING")
    bag += (b"" if nested_connection else conn) + other
    bag += _record({"op": b"\5", "compression": compression,
                    "size": struct.pack("<I", len(chunk))}, chunk)
    return bag + _record({"op": b"\4"}, b"TRAILING INDEX THAT MUST ALSO BE HASHED")


class RichDecodeTests(unittest.TestCase):
    def test_padding_endian_integer_and_float64_precision(self):
        for big in (False, True):
            for padded in (False, True):
                payload, expected = cloud_payload(big, padded)
                arr, meta = io.decode_rich_pointcloud2(payload)
                np.testing.assert_array_equal(arr, expected.reshape(-1))
                self.assertEqual(arr["ring"][2], 4_000_000_001)
                self.assertGreater(arr["time"][1] - arr["time"][0], 0)
                self.assertLess(arr["time"][1] - arr["time"][0], 1e-9)
                self.assertEqual(meta["row_padding_bytes"], 4 if padded else 0)
                self.assertEqual(meta["is_bigendian"], big)
                self.assertEqual(meta["field_schema"][3]["count"], 2)
                self.assertEqual(arr["pair"].shape, (4, 2))
                self.assertEqual(meta["header_nsec"], 456)

    def test_truncated_payload_and_trailing_bytes_rejected(self):
        payload, _ = cloud_payload()
        for truncated in (payload[:8], payload[:-1], payload[:-5]):
            with self.assertRaises(io.CloudDecodeError):
                io.decode_rich_pointcloud2(truncated)
        with self.assertRaises(io.CloudDecodeError):
            io.decode_rich_pointcloud2(payload + b"extra")

    def test_invalid_row_layout_and_field_bounds_rejected(self):
        payload, _ = cloud_payload()
        # Locate the final PointCloud2 fixed-layout header before point bytes.
        position = len(payload) - 1 - 80 - 13
        bad_row = bytearray(payload)
        struct.pack_into("<I", bad_row, position + 5, 39)
        with self.assertRaises(io.CloudDecodeError):
            io.decode_rich_pointcloud2(bytes(bad_row))
        bad_step = bytearray(payload)
        struct.pack_into("<I", bad_step, position + 1, 16)
        with self.assertRaises(io.CloudDecodeError):
            io.decode_rich_pointcloud2(bytes(bad_step))

    def test_field_audit_preserves_slots_and_pooled_components(self):
        arr, _ = io.decode_rich_pointcloud2(cloud_payload()[0])
        rows = {row["field"]: row for row in io.scalar_field_audit(arr, np.array([True, False, True, False]))}
        self.assertEqual(rows["ring"]["all_slots_finite_max"], 4_000_000_001)
        self.assertEqual(rows["pair"]["component_count"], 2)
        self.assertEqual(rows["pair"]["all_slots_value_count"], 8)
        self.assertEqual(rows["pair"]["finite_nonzero_xyz_value_count"], 4)
        self.assertEqual(rows["time"]["semantics_status"], "UNKNOWN")
        with self.assertRaises(ValueError):
            io.scalar_field_audit(arr, np.ones(2, bool))

    def test_nonfinite_summary_and_relationships_not_physical_truth(self):
        arr = np.array([
            (1., 0., 0., 1., 0, 0., 2., 2.2, 1., 0., -1., 0.),
            (2., 0., 0., 2., 0, .1, 0., 0., -1., 0., -1., 0.),
            (3., 0., 0., 3., 1, 0., -2., -2.2, -1., 0., -1., 0.),
            (4., 0., 0., 4., 1, .1, float("nan"), 0., 1., 0., -1., 0.),
        ], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("radius", "f4"),
                  ("ring", "u4"), ("time", "f8"), ("velocity", "f4"),
                  ("velocity0", "f4"), ("velocity_direction", "f4"),
                  ("acceleration", "f4"), ("acceleration_direction", "f4"),
                  ("surface_evenness", "f4")])
        mask = np.ones(4, bool)
        rows = {r["field"]: r for r in io.scalar_field_audit(arr, mask)}
        self.assertEqual(rows["velocity"]["all_slots_nonfinite_count"], 1)
        self.assertEqual(rows["acceleration_direction"]["all_slots_minus_one_count"], 4)
        relationships = io.field_relationship_audit(arr, mask, {"height": 2, "width": 2})
        self.assertEqual(relationships["physical_semantics_status"], "UNKNOWN")
        self.assertEqual(relationships["ring_row_index_equal_fraction"], 1.)
        self.assertEqual(relationships["velocity_direction_nonzero_sign_equal_fraction"], 1.)
        self.assertEqual(relationships["radius_minus_xyz_norm"]["finite_max"], 0.)
        self.assertEqual(relationships["point_time_by_row"][1]["exactly_equal_to_first_row_fraction"], 1.)
        self.assertEqual(io.field_relationship_audit(arr, mask)["organized_layout_status"], "UNKNOWN")


class BagIdentityTests(unittest.TestCase):
    def _write_zip(self, tmp, bag):
        path = Path(tmp) / "test.zip"
        with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("raw/example.bag", bag)
        return path

    def test_complete_hash_includes_skipped_and_trailing_bytes(self):
        for nested in (False, True):
            bag = bag_bytes(nested_connection=nested)
            with tempfile.TemporaryDirectory() as tmp:
                path = self._write_zip(tmp, bag)
                audit = {}
                frames = list(io.iter_rich_frames(path, audit=audit))
            self.assertEqual(len(frames), 1)
            self.assertEqual(audit["bag_member_sha256"], hashlib.sha256(bag).hexdigest())
            self.assertEqual(audit["bytes_read"], len(bag))
            self.assertTrue(audit["complete_member_hash"])
            self.assertTrue(audit["zip_crc_verified_at_eof"])
            self.assertEqual(audit["noncloud_message_count"], 1)
            self.assertEqual(frames[0][1]["topic"], "/rich")
            self.assertEqual(frames[0][1]["frame_index"], 0)
            self.assertEqual(frames[0][1]["bag_time_nsec"], 789)
            self.assertEqual(audit["errors"], [])

    def test_early_close_does_not_claim_raw_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_zip(tmp, bag_bytes())
            audit = {}
            iterator = io.iter_rich_frames(path, audit=audit)
            next(iterator)
            iterator.close()
            self.assertFalse(audit["complete_member_hash"])
            self.assertFalse(audit["zip_crc_verified_at_eof"])
            self.assertIsNone(audit["bag_member_sha256"])
            self.assertIsNotNone(audit["sha256_partial"])

    def test_bad_cloud_is_recorded_and_not_silently_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_zip(tmp, bag_bytes(payload=b"broken"))
            audit = {}
            with self.assertRaises(io.CloudDecodeError):
                list(io.iter_rich_frames(path, audit=audit))
            self.assertEqual(len(audit["errors"]), 1)
            self.assertEqual(audit["errors"][0]["error_type"], "CloudDecodeError")
            self.assertFalse(audit["complete_member_hash"])

    def test_unsupported_chunk_compression_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_zip(tmp, bag_bytes(compression=b"bz2"))
            audit = {}
            with self.assertRaises(NotImplementedError):
                list(io.iter_rich_frames(path, audit=audit))
            self.assertEqual(audit["errors"][0]["error_type"], "NotImplementedError")
            self.assertFalse(audit["complete_member_hash"])

    def test_zip_crc_failure_is_explicit_and_not_a_complete_hash(self):
        bag = bag_bytes()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad_crc.zip"
            with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED) as zf:
                zf.writestr("example.bag", bag)
            with zipfile.ZipFile(path) as zf:
                info = zf.getinfo("example.bag")
                offset = info.header_offset
            # Corrupt ignored trailing index data without rewriting its CRC.
            with path.open("r+b") as fh:
                fh.seek(offset)
                local = fh.read(30)
                name_size, extra_size = struct.unpack_from("<HH", local, 26)
                fh.seek(offset + 30 + name_size + extra_size + len(bag) - 1)
                fh.write(b"!")
            audit = {}
            with self.assertRaises(zipfile.BadZipFile):
                list(io.iter_rich_frames(path, audit=audit))
            self.assertFalse(audit["zip_crc_verified_at_eof"])
            self.assertFalse(audit["complete_member_hash"])
            self.assertIsNone(audit["bag_member_sha256"])
            self.assertEqual(audit["errors"][0]["error_type"], "BadZipFile")


if __name__ == "__main__":
    unittest.main()
