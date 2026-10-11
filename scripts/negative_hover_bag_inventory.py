#!/usr/bin/env python3
"""Read-only raw-record metadata inventory for E20 negative/hover evidence.

This tool never decodes a point cloud, calls a detector or constructs a graph.
Standalone bags use their final connection/chunk index. Archived bags expose
only a bounded decompressed prefix; no seek to their remote final index occurs.
The outputs describe available records, not UAV presence or absence labels.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import platform
import struct
import subprocess
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

import PIL
from PIL import ExifTags, Image
import zipfile


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
DEFAULT_OUT = ROOT / "results/negative_hover_evidence_audit"
OUTPUTS = (
    "raw_bag_inventory.json", "raw_bag_inventory.csv",
    "archive_member_inventory.csv", "pcd_inventory.csv",
    "data_file_inventory.csv", "bag_inventory_execution.json",
)


def fields(buf: bytes) -> dict[str, bytes]:
    out = {}
    pos = 0
    while pos < len(buf):
        if pos + 4 > len(buf):
            raise ValueError("Truncated ROS bag header field length")
        length = struct.unpack_from("<I", buf, pos)[0]
        pos += 4
        if pos + length > len(buf):
            raise ValueError("Truncated ROS bag header field")
        item = buf[pos:pos + length]
        pos += length
        if b"=" not in item:
            raise ValueError("Malformed ROS bag header field")
        key, value = item.split(b"=", 1)
        out[key.decode()] = value
    return out


class Reader:
    """Account for explicitly requested metadata bytes, never cloud payloads."""

    def __init__(self, stream):
        self.stream = stream
        self.bytes_read = 0

    def read(self, length: int) -> bytes:
        data = self.stream.read(length)
        self.bytes_read += len(data)
        return data

    def record(self):
        length_buf = self.read(4)
        if not length_buf:
            return None
        if len(length_buf) != 4:
            raise ValueError("Truncated record header length")
        header_len = struct.unpack("<I", length_buf)[0]
        if header_len > 16384:
            raise ValueError("Metadata header exceeds 16 KiB safety bound")
        header_buf = self.read(header_len)
        data_len_buf = self.read(4)
        if len(header_buf) != header_len or len(data_len_buf) != 4:
            raise ValueError("Truncated record header")
        return fields(header_buf), struct.unpack("<I", data_len_buf)[0], self.stream.tell()


def u32(buf: bytes) -> int:
    return struct.unpack("<I", buf)[0]


def ros_time(buf: bytes) -> float:
    sec, nsec = struct.unpack("<II", buf)
    return sec + nsec * 1e-9


def utc(timestamp):
    if timestamp is None:
        return "UNKNOWN"
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat(timespec="microseconds")


def decode_name(name: str) -> str:
    try:
        recovered = name.encode("cp437").decode("gbk")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return name
    return recovered if any("\u4e00" <= c <= "\u9fff" for c in recovered) else name


def connection(header, payload):
    data = fields(payload)
    return {
        "connection_id": u32(header["conn"]),
        "topic": header["topic"].decode(),
        "message_type": data.get("type", b"").decode(),
        "md5sum": data.get("md5sum", b"").decode(),
        "callerid": data.get("callerid", b"").decode(),
        "latching": data.get("latching", b"").decode(),
    }


def message_sample(header, prefix):
    result = {
        "connection_id": u32(header["conn"]),
        "first_bag_record_time_unix": ros_time(header["time"]),
    }
    if len(prefix) >= 16:
        seq, sec, nsec, frame_len = struct.unpack_from("<IIII", prefix)
        result.update({
            "sequence": seq, "first_ros_header_time_unix": sec + nsec * 1e-9,
            "frame_id": prefix[16:16 + frame_len].decode(errors="replace"),
        })
    return result


def file_stat(path: Path):
    stat = path.stat()
    return {"size_bytes": stat.st_size, "mtime_ns": stat.st_mtime_ns,
            "mode": stat.st_mode}


def standalone(path: Path):
    result = {"source_kind": "STANDALONE_BAG", "source_path": str(path.relative_to(ROOT)),
              "archive_member": "", "source_size_bytes": path.stat().st_size,
              "probe_byte_limit": "NOT_APPLICABLE_INDEX_METADATA_ONLY"}
    with path.open("rb") as stream:
        reader = Reader(stream)
        if reader.read(13) != b"#ROSBAG V2.0\n":
            raise ValueError(f"Not a ROS1 V2.0 bag: {path}")
        header, _data_len, _data_start = reader.record()
        index_pos = struct.unpack("<Q", header["index_pos"])[0]
        declared_connections = u32(header["conn_count"])
        declared_chunks = u32(header["chunk_count"])
        if not (13 < index_pos < path.stat().st_size):
            raise ValueError(f"Missing/out-of-range final index: {path}")
        stream.seek(index_pos)
        connections, chunks = [], []
        while True:
            record = reader.record()
            if record is None:
                break
            h, length, _pos = record
            if length > 1048576:
                raise ValueError("Final index metadata exceeds 1 MiB safety bound")
            payload = reader.read(length)
            if len(payload) != length:
                raise ValueError("Truncated final index data")
            op = h["op"][0]
            if op == 7:
                connections.append(connection(h, payload))
            elif op == 6:
                count = u32(h["count"])
                if len(payload) != count * 8:
                    raise ValueError("Chunk-info connection-count payload mismatch")
                chunks.append({
                    "position": struct.unpack("<Q", h["chunk_pos"])[0],
                    "start": ros_time(h["start_time"]), "end": ros_time(h["end_time"]),
                    "counts": [struct.unpack_from("<II", payload, 8 * i) for i in range(count)],
                })
            else:
                raise ValueError(f"Unexpected op {op} in finalized metadata index")
        if len(connections) != declared_connections or len(chunks) != declared_chunks:
            raise ValueError("Declared/indexed metadata counts disagree")
        counts = defaultdict(int)
        for chunk in chunks:
            for conn, count in chunk["counts"]:
                counts[conn] += count
        first_sample = {}
        if chunks:
            stream.seek(chunks[0]["position"])
            ch, _length, _pos = reader.record()
            compression = ch["compression"].decode()
            if compression == "none":
                while True:
                    mh, message_len, message_pos = reader.record()
                    if mh["op"][0] == 2:
                        first_sample = message_sample(mh, reader.read(min(128, message_len)))
                        first_sample["serialized_message_size_bytes"] = message_len
                        break
                    stream.seek(message_pos + message_len)
            result["first_chunk_compression"] = compression
        start = min((chunk["start"] for chunk in chunks), default=None)
        end = max((chunk["end"] for chunk in chunks), default=None)
        result.update({
            "declared_connection_count": declared_connections,
            "observed_connection_count": len(connections), "connections": connections,
            "all_declared_connections_observed": len(connections) == declared_connections,
            "declared_chunk_count": declared_chunks, "indexed_chunk_count": len(chunks),
            "indexed_message_count_by_connection": dict(counts),
            "indexed_message_count_total": sum(counts.values()),
            "index_pos": index_pos, "record_start_unix": start, "record_end_unix": end,
            "record_start_utc": utc(start), "record_end_utc": utc(end),
            "record_duration_s": end - start if start is not None and end is not None else "UNKNOWN",
            "first_message_sample": first_sample,
            "bytes_read": reader.bytes_read,
            "topic_enumeration_coverage": "COMPLETE_FINAL_CONNECTION_INDEX",
            "record_range_source": "FINAL_CHUNK_INFO_INDEX",
            "full_cloud_decode": False,
        })
    return result


def bounded_record(buf, pos):
    if pos + 4 > len(buf):
        return None
    length = struct.unpack_from("<I", buf, pos)[0]
    if length > 16384:
        raise ValueError("Prefix metadata header exceeds 16 KiB safety bound")
    pos += 4
    if pos + length + 4 > len(buf):
        return None
    header = fields(buf[pos:pos + length])
    pos += length
    return header, struct.unpack_from("<I", buf, pos)[0], pos + 4


def archived_prefix(archive: Path, member, zf, limit):
    with zf.open(member) as stream:
        buf = stream.read(limit)
    if buf[:13] != b"#ROSBAG V2.0\n":
        raise ValueError(f"Not a ROS1 V2.0 archived bag: {member.filename}")
    h, data_len, data_start = bounded_record(buf, 13)
    declared_connections = u32(h["conn_count"])
    result = {
        "source_kind": "ZIP_BAG_PREFIX", "source_path": str(archive.relative_to(ROOT)),
        "archive_member": decode_name(member.filename), "source_size_bytes": member.file_size,
        "zip_member_crc32": f"{member.CRC:08x}", "zip_member_compression_type": member.compress_type,
        "probe_byte_limit": limit, "bytes_read": len(buf),
        "prefix_sha256": hashlib.sha256(buf).hexdigest(),
        "declared_connection_count": declared_connections,
        "declared_chunk_count": u32(h["chunk_count"]),
        "index_pos": struct.unpack("<Q", h["index_pos"])[0],
        "record_end_unix": "UNKNOWN_PREFIX_ONLY", "record_end_utc": "UNKNOWN_PREFIX_ONLY",
        "record_duration_s": "UNKNOWN_PREFIX_ONLY", "indexed_chunk_count": "NOT_READ",
        "indexed_message_count_total": "UNKNOWN_PREFIX_ONLY", "full_cloud_decode": False,
        "record_range_source": "FIRST_MESSAGE_HEADER_ONLY_NO_ARCHIVE_INDEX_SEEK",
    }
    pos = data_start + data_len
    observed = {}
    samples = []
    compressions = []
    while True:
        record = bounded_record(buf, pos)
        if record is None:
            break
        outer, length, data_start = record
        op = outer["op"][0]
        if op == 5:
            compression = outer["compression"].decode()
            compressions.append(compression)
            if compression != "none":
                break
            inner_pos = data_start
            end = min(data_start + length, len(buf))
            while inner_pos < end:
                inner_record = bounded_record(buf, inner_pos)
                if inner_record is None:
                    break
                inner, inner_length, inner_data_start = inner_record
                inner_op = inner["op"][0]
                if inner_op == 7 and inner_data_start + inner_length <= len(buf):
                    conn = connection(inner, buf[inner_data_start:inner_data_start + inner_length])
                    observed[conn["connection_id"]] = conn
                elif inner_op == 2:
                    sample = message_sample(inner, buf[inner_data_start:min(inner_data_start + 128, len(buf))])
                    sample["serialized_message_size_bytes"] = inner_length
                    samples.append(sample)
                inner_pos = inner_data_start + inner_length
        elif op == 7 and data_start + length <= len(buf):
            conn = connection(outer, buf[data_start:data_start + length])
            observed[conn["connection_id"]] = conn
        pos = data_start + length
    complete = len(observed) == declared_connections and declared_connections > 0
    result.update({
        "observed_connection_count": len(observed), "connections": list(observed.values()),
        "all_declared_connections_observed": complete,
        "topic_enumeration_coverage": "ALL_HEADER_DECLARED_CONNECTIONS_OBSERVED_IN_BOUNDED_PREFIX" if complete else "PARTIAL_PREFIX_ONLY",
        "first_chunk_compression": compressions[0] if compressions else "UNKNOWN",
        "first_message_sample": samples[0] if samples else {},
        "record_start_unix": samples[0]["first_bag_record_time_unix"] if samples else "UNKNOWN_PREFIX_ONLY",
        "record_start_utc": utc(samples[0]["first_bag_record_time_unix"]) if samples else "UNKNOWN_PREFIX_ONLY",
        "footer_integrity_check": "NOT_READ_BOUNDED_PROBE_ONLY",
    })
    return result


def write_csv(path, rows, columns):
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(row[key], ensure_ascii=False) if isinstance(row.get(key), (list, dict)) else row.get(key, "") for key in columns})


def write_json(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--prefix-bytes", type=int, default=1048576)
    args = parser.parse_args()
    if args.prefix_bytes < 16384 or args.prefix_bytes > 1048576:
        raise ValueError("Metadata prefix bound must be 16 KiB to 1 MiB")
    output = args.output_dir.resolve()
    if not output.is_relative_to(ROOT / "results"):
        raise ValueError("Output directory must be under this repository results/")
    for name in OUTPUTS:
        if (output / name).exists():
            raise FileExistsError(f"Refusing to overwrite {output / name}")
    started = datetime.now(timezone.utc)
    file_command = ["rg", "--files", "--hidden", "--no-ignore", "bag", "data", "-g", "!**/.claude/**"]
    listed = subprocess.run(file_command, cwd=ROOT, check=True, text=True, capture_output=True).stdout.splitlines()
    paths = sorted(ROOT / relative for relative in listed if relative)
    before = {str(path.relative_to(ROOT)): file_stat(path) for path in paths}
    data_rows = [{"path": str(path.relative_to(ROOT)), "extension": path.suffix.lower() or "NO_SUFFIX", **file_stat(path), "presence_or_absence": "UNKNOWN_NOT_CERTIFIED_BY_FILE_METADATA"} for path in paths]
    bag_rows = [standalone(path) for path in paths if path.suffix.lower() == ".bag"]
    archive_rows = []
    archives = []
    for archive in [path for path in paths if path.suffix.lower() == ".zip"]:
        with zipfile.ZipFile(archive) as zf:
            archives.append({"path": str(archive.relative_to(ROOT)), "comment": zf.comment.decode(errors="replace"), "members_including_directories": len(zf.infolist())})
            for member in zf.infolist():
                row = {"archive": str(archive.relative_to(ROOT)), "member": decode_name(member.filename), "member_name_zip_decoded": member.filename, "is_directory": member.is_dir(), "size_bytes": member.file_size, "compressed_size_bytes": member.compress_size, "compression_type": member.compress_type, "crc32": f"{member.CRC:08x}", "comment": member.comment.decode(errors="replace"), "nonbag_small_text": ""}
                if not member.is_dir() and not member.filename.lower().endswith(".bag") and member.file_size <= 4096:
                    row["nonbag_small_text"] = zf.read(member).decode(errors="replace")
                archive_rows.append(row)
                if member.filename.lower().endswith(".bag"):
                    bag_rows.append(archived_prefix(archive, member, zf, args.prefix_bytes))
            print(f"metadata probe {archive.relative_to(ROOT)} completed; no point cloud decoded", flush=True)
    pcd_groups = defaultdict(list)
    for path in paths:
        if path.suffix.lower() == ".pcd":
            pcd_groups[str(path.parent.relative_to(ROOT))].append(path.name)
    pcd_rows = [{"directory": name, "pcd_files": len(names), "first_filename_lexical": min(names), "last_filename_lexical": max(names), "source_type_by_name": "FILTERED" if name.endswith("_filtered_data") else "RAW_PCD", "timestamp_frame": "FILENAME_LIDAR_HEADER_CLOCK_NOT_SYNCHRONIZED_WALL_CLOCK", "presence_or_absence": "UNKNOWN_NOT_CERTIFIED_BY_PCD_NAMES"} for name, names in sorted(pcd_groups.items())]
    photo = ROOT / "bag/20260205/场景图.jpg"
    photo_record = {"path": str(photo.relative_to(ROOT)), "sha256": hashlib.sha256(photo.read_bytes()).hexdigest()}
    with Image.open(photo) as image:
        exif = image.getexif()
        photo_record.update({"pixel_dimensions": list(image.size), "exif": {ExifTags.TAGS.get(key, str(key)): str(value) for key, value in exif.items()}, "nested_exif": {ExifTags.TAGS.get(key, str(key)): str(value) for key, value in exif.get_ifd(34665).items()}, "gps_exif": {ExifTags.GPSTAGS.get(key, str(key)): str(value) for key, value in exif.get_ifd(34853).items()}, "independent_presence_absence_window": "UNKNOWN_STILL_IMAGE_PRECEDES_ALL_FEBRUARY_BAG_TIMES_NO_TIME_WINDOW_CERTIFICATION"})
    after = {str(path.relative_to(ROOT)): file_stat(path) for path in paths}
    if before != after:
        raise RuntimeError("Raw data file size/mtime/mode changed during metadata inventory")
    topic_counter = Counter(c["topic"] for b in bag_rows for c in b["connections"])
    archived = [row for row in bag_rows if row["source_kind"] == "ZIP_BAG_PREFIX"]
    summary = {
        "audit": "E20_READ_ONLY_NEGATIVE_HOVER_RAW_RECORD_INVENTORY",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "scope": {"roots": ["bag", "data"], "enumeration": file_command,
                  "file_count": len(paths), "extension_counts": dict(sorted(Counter(row["extension"] for row in data_rows).items())),
                  "archive_count": len(archives), "standalone_bags": len(bag_rows) - len(archived), "archived_bag_members": len(archived),
                  "archive_non_directory_members": sum(not row["is_directory"] for row in archive_rows),
                  "pcd_directories": len(pcd_rows), "prefix_limit_per_archived_bag_bytes": args.prefix_bytes,
                  "archived_uncompressed_prefix_bytes_total": sum(row["bytes_read"] for row in archived),
                  "archived_bags_with_all_header_declared_connections_observed": sum(row["all_declared_connections_observed"] for row in archived),
                  "connection_topic_counts_across_source_files": dict(topic_counter)},
        "raw_file_stat_before_after_equal": True,
        "source_identity_limit": "Raw bag/ZIP complete content SHA-256 not computed; read-only size/mtime/mode, ZIP member CRC32 and bounded prefix SHA-256 recorded. Extracted paths are not additional independent sequences.",
        "reliable_target_absence": "UNKNOWN_DATA_GAP_NO_FLIGHT_STATE_VIDEO_OR_ABSENCE_RECORD_IN_INSPECTED_METADATA",
        "independent_hover_identity": "UNKNOWN_DATA_GAP_NOT_CERTIFIED_BY_LIDAR_TOPIC_OR_FILENAMES",
        "archives": archives, "bags": bag_rows, "scene_photo": photo_record,
        "limitations": ["ZIP first-prefix connection coverage is relative to finalized header declared count; full footer integrity and chunk/message time ranges were not checked.", "LiDAR point clouds are available, but were not decoded or used to infer absence.", "Topic exclusivity cannot certify target absence; no hidden UAV outside FOV, takeoff or landing state is inferred.", "No file-name hover/absence inference, thresholds, algorithms, E18 changes or stress tests occur in this inventory.", "Repository bag/data scope only; external records absent from checkout remain UNKNOWN.", "PCD counts/timestamp filenames are derived-data metadata, not complete UAV-presence annotations."],
    }
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "raw_bag_inventory.json", summary)
    bag_columns = ["source_kind", "source_path", "archive_member", "source_size_bytes", "probe_byte_limit", "bytes_read", "prefix_sha256", "zip_member_crc32", "declared_connection_count", "observed_connection_count", "all_declared_connections_observed", "connections", "declared_chunk_count", "indexed_chunk_count", "indexed_message_count_total", "indexed_message_count_by_connection", "index_pos", "record_start_unix", "record_end_unix", "record_start_utc", "record_end_utc", "record_duration_s", "first_message_sample", "first_chunk_compression", "topic_enumeration_coverage", "record_range_source", "full_cloud_decode", "footer_integrity_check"]
    write_csv(output / "raw_bag_inventory.csv", bag_rows, bag_columns)
    write_csv(output / "archive_member_inventory.csv", archive_rows, ["archive", "member", "member_name_zip_decoded", "is_directory", "size_bytes", "compressed_size_bytes", "compression_type", "crc32", "comment", "nonbag_small_text"])
    write_csv(output / "pcd_inventory.csv", pcd_rows, ["directory", "pcd_files", "first_filename_lexical", "last_filename_lexical", "source_type_by_name", "timestamp_frame", "presence_or_absence"])
    write_csv(output / "data_file_inventory.csv", data_rows, ["path", "extension", "size_bytes", "mtime_ns", "mode", "presence_or_absence"])
    execution = {
        "command": "PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python scripts/negative_hover_bag_inventory.py --output-dir results/negative_hover_evidence_audit --prefix-bytes 1048576",
        "argv": sys.argv, "cwd": str(ROOT), "started_utc": started.isoformat(), "finished_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {"python": sys.version, "executable": sys.executable, "platform": platform.platform(), "Pillow": PIL.__version__},
        "script_path": str(SCRIPT.relative_to(ROOT)), "script_sha256": hashlib.sha256(SCRIPT.read_bytes()).hexdigest(),
        "git_head": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, text=True, capture_output=True).stdout.strip(),
        "output_hashes": {name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in OUTPUTS if name != "bag_inventory_execution.json"},
        "raw_data_stat_identity_before": before, "raw_data_stat_identity_after": after,
        "raw_data_stat_check": "EXACT_EQUAL_SIZE_MTIME_MODE_NO_FULL_CONTENT_HASH",
        "metadata_coverage": summary["scope"], "algorithm_execution": "NONE", "new_graphs": "NONE", "labels_changed": False,
    }
    write_json(output / "bag_inventory_execution.json", execution)
    print(json.dumps(summary["scope"], ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
