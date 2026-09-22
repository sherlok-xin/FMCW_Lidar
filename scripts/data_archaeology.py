#!/usr/bin/env python3
"""Read-only ROS1 bag diagnostics for the FMCW LiDAR data archaeology audit."""

from __future__ import annotations

import argparse
import json
import math
import mmap
import struct
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN


ROS_TYPES = {
    1: (np.int8, 1), 2: (np.uint8, 1), 3: (np.int16, 2), 4: (np.uint16, 2),
    5: (np.int32, 4), 6: (np.uint32, 4), 7: (np.float32, 4), 8: (np.float64, 8),
}


def _fields(buf: bytes) -> dict[str, bytes]:
    out, off = {}, 0
    while off < len(buf):
        n = struct.unpack_from("<I", buf, off)[0]
        off += 4
        item = buf[off:off + n]
        off += n
        if b"=" in item:
            key, value = item.split(b"=", 1)
            out[key.decode()] = value
    return out


def _u32(buf: bytes, off: int) -> tuple[int, int]:
    return struct.unpack_from("<I", buf, off)[0], off + 4


def _string(buf: bytes, off: int) -> tuple[str, int]:
    n, off = _u32(buf, off)
    return bytes(buf[off:off + n]).decode(errors="replace"), off + n


def decode_pointcloud2(msg: bytes, wanted=("x", "y", "z", "intensity", "velocity")):
    seq, sec, nsec = struct.unpack_from("<III", msg, 0)
    off = 12
    frame_id, off = _string(msg, off)
    height, width = struct.unpack_from("<II", msg, off)
    off += 8
    nfields, off = _u32(msg, off)
    fields = []
    for _ in range(nfields):
        name, off = _string(msg, off)
        field_off = struct.unpack_from("<I", msg, off)[0]
        datatype = msg[off + 4]
        count = struct.unpack_from("<I", msg, off + 5)[0]
        off += 9
        fields.append((name, field_off, datatype, count))
    is_bigendian = bool(msg[off])
    off += 1
    point_step, row_step = struct.unpack_from("<II", msg, off)
    off += 8
    data_len, off = _u32(msg, off)
    data = memoryview(msg)[off:off + data_len]

    endian = ">" if is_bigendian else "<"
    dtype_fields = []
    for name, field_off, datatype, count in fields:
        base, _ = ROS_TYPES[datatype]
        dt = np.dtype(base).newbyteorder(endian)
        shape = () if count == 1 else (count,)
        dtype_fields.append((name, dt, shape, field_off))
    dtype = np.dtype({
        "names": [x[0] for x in dtype_fields],
        "formats": [(x[1], x[2]) if x[2] else x[1] for x in dtype_fields],
        "offsets": [x[3] for x in dtype_fields],
        "itemsize": point_step,
    })
    arr = np.frombuffer(data, dtype=dtype, count=height * width)
    values = {name: np.asarray(arr[name], dtype=np.float32).reshape(-1).copy()
              for name in wanted if name in arr.dtype.names}
    meta = {
        "seq": int(seq), "header_sec": int(sec), "header_nsec": int(nsec),
        "header_time": sec + nsec * 1e-9, "frame_id": frame_id,
        "height": int(height), "width": int(width), "point_step": int(point_step),
        "fields": [x[0] for x in fields],
    }
    return values, meta


def iter_bag_messages(path: Path, conn_id: int = 0, limit: int | None = None):
    yielded = 0
    with path.open("rb") as fh:
        mm = mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ)
        if mm[:13] != b"#ROSBAG V2.0\n":
            raise ValueError(f"Not a ROS1 bag: {path}")
        pos = 13
        while pos + 4 <= len(mm):
            hlen = struct.unpack_from("<I", mm, pos)[0]
            pos += 4
            if pos + hlen + 4 > len(mm):
                break
            header = _fields(mm[pos:pos + hlen])
            pos += hlen
            dlen = struct.unpack_from("<I", mm, pos)[0]
            pos += 4
            op = header.get("op", b"\0")[0]
            if op != 5:
                pos += dlen
                continue
            if header["compression"] != b"none":
                raise NotImplementedError("Only uncompressed chunks occur in the selected bags")
            chunk_end = pos + dlen
            while pos < chunk_end:
                ihlen = struct.unpack_from("<I", mm, pos)[0]
                pos += 4
                inner = _fields(mm[pos:pos + ihlen])
                pos += ihlen
                ilen = struct.unpack_from("<I", mm, pos)[0]
                pos += 4
                iop = inner.get("op", b"\0")[0]
                conn = struct.unpack("<I", inner["conn"])[0] if "conn" in inner else -1
                if iop == 2 and conn == conn_id:
                    sec, nsec = struct.unpack("<II", inner["time"])
                    payload = memoryview(mm)[pos:pos + ilen]
                    values, meta = decode_pointcloud2(payload)
                    del payload
                    meta["bag_time"] = sec + nsec * 1e-9
                    yield values, meta
                    yielded += 1
                    if limit is not None and yielded >= limit:
                        mm.close()
                        return
                pos += ilen
            pos = chunk_end
        mm.close()


class StageFilter:
    def __init__(self, voxel=1.0, init_frames=8, threshold=0.05):
        self.voxel = voxel
        self.init_frames = init_frames
        self.threshold = threshold
        self.bounds = ((0., 500.), (-150., 150.), (-5., 50.))
        self.shape = tuple(int(math.ceil((b - a) / voxel)) for a, b in self.bounds)
        self.counts = np.zeros(np.prod(self.shape), dtype=np.int32)
        self.frame_count = 0

    def indices(self, points):
        q = np.floor((points - np.array([b[0] for b in self.bounds])) / self.voxel).astype(np.int64)
        valid = np.all((q >= 0) & (q < np.array(self.shape)), axis=1)
        idx = np.full(len(points), -1, dtype=np.int64)
        idx[valid] = (q[valid, 0] * self.shape[1] * self.shape[2]
                      + q[valid, 1] * self.shape[2] + q[valid, 2])
        return idx, valid

    def neighborhood(self, pre, idx, valid):
        if not np.any(pre):
            return pre.copy(), np.zeros(len(pre), dtype=np.float32)
        dyn_idx = idx[pre]
        candidate_rows = np.flatnonzero(pre)
        nynz = self.shape[1] * self.shape[2]
        ix = dyn_idx // nynz
        iy = (dyn_idx % nynz) // self.shape[2]
        iz = dyn_idx % self.shape[2]
        offsets = np.array([dx*nynz + dy*self.shape[2] + dz
                            for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)])
        ds = np.array([(dx, dy, dz) for dx in (-1, 0, 1)
                       for dy in (-1, 0, 1) for dz in (-1, 0, 1)])
        neigh = dyn_idx[:, None] + offsets
        coords = np.stack([ix, iy, iz], axis=1)[:, None, :] + ds[None, :, :]
        inbound = np.all((coords >= 0) & (coords < np.array(self.shape)), axis=2)
        neigh = np.clip(neigh, 0, len(self.counts) - 1)
        occupied = (self.counts[neigh] > 0) & inbound
        dynamic_neighbor = np.isin(neigh.ravel(), dyn_idx).reshape(neigh.shape) & inbound
        occupied_n = occupied.sum(axis=1)
        static_n = (occupied & ~dynamic_neighbor).sum(axis=1)
        ratio = np.divide(static_n, occupied_n, out=np.zeros_like(static_n, dtype=float), where=occupied_n > 0)
        remove = (occupied_n > 0) & (ratio > 0.6)
        post = pre.copy()
        post[candidate_rows[remove]] = False
        ratio_full = np.zeros(len(pre), dtype=np.float32)
        ratio_full[candidate_rows] = ratio
        return post, ratio_full

    def process(self, points):
        self.frame_count += 1
        idx, valid = self.indices(points)
        if self.frame_count <= self.init_frames:
            pre = np.zeros(len(points), dtype=bool)
            post = pre.copy()
            ratio = np.zeros(len(points), dtype=np.float32)
            static = valid
            probability = np.zeros(len(points), dtype=np.float32)
        else:
            probability = np.zeros(len(points), dtype=np.float32)
            probability[valid] = self.counts[idx[valid]] / self.frame_count
            pre = (probability < self.threshold) & valid
            post, ratio = self.neighborhood(pre, idx, valid)
            static = (~post) & valid
        occupied = np.unique(idx[static & (idx >= 0)])
        self.counts[occupied] += 1
        return pre, post, probability, ratio, valid


def transform_xyz(values):
    raw = np.column_stack([values["x"], values["y"], values["z"]])
    return np.column_stack([raw[:, 0], raw[:, 2], -raw[:, 1]])


def sample_idx(n, maximum, seed):
    if n <= maximum:
        return np.arange(n)
    return np.random.default_rng(seed).choice(n, maximum, replace=False)


def scatter(ax, points, color, title, xaxis="x", yaxis="y", size=0.25, cmap="viridis",
            vmin=None, vmax=None, limit=120000, seed=0):
    if len(points) == 0:
        ax.text(.5, .5, "no points", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title)
        return None
    idx = sample_idx(len(points), limit, seed)
    cols = {"x": 0, "y": 1, "z": 2}
    sc = ax.scatter(points[idx, cols[xaxis]], points[idx, cols[yaxis]], c=np.asarray(color)[idx],
                    s=size, cmap=cmap, vmin=vmin, vmax=vmax, linewidths=0, rasterized=True)
    ax.set_title(f"{title} (n={len(points):,})")
    ax.set_xlabel(f"{xaxis} (m)")
    ax.set_ylabel(f"{yaxis} (m)")
    ax.grid(alpha=.15)
    return sc


def plot_views(path, name, frame, points, intensity, velocity, meta):
    fig, axs = plt.subplots(2, 3, figsize=(18, 10), dpi=180, constrained_layout=True)
    s0 = scatter(axs[0, 0], points, intensity, "BEV colored by intensity", vmin=10, vmax=100, seed=frame)
    s1 = scatter(axs[0, 1], points, velocity, "BEV colored by radial velocity", cmap="coolwarm",
                 vmin=-15, vmax=15, seed=frame)
    s2 = scatter(axs[0, 2], points, intensity, "Side view colored by intensity", yaxis="z",
                 vmin=10, vmax=100, seed=frame)
    s3 = scatter(axs[1, 0], points, velocity, "Side view colored by radial velocity", yaxis="z",
                 cmap="coolwarm", vmin=-15, vmax=15, seed=frame)
    r = np.linalg.norm(points, axis=1)
    ridx = sample_idx(len(points), 160000, frame)
    s4 = axs[1, 1].scatter(r[ridx], velocity[ridx], c=intensity[ridx], s=.25, cmap="viridis",
                           vmin=10, vmax=100, linewidths=0, rasterized=True)
    axs[1, 1].set(title="Range–radial velocity colored by intensity", xlabel="range (m)", ylabel="radial velocity")
    axs[1, 1].grid(alpha=.15)
    h = axs[1, 2].hist2d(r, velocity, bins=(300, 240), range=((0, 520), (-60, 60)), cmap="magma",
                         norm=matplotlib.colors.LogNorm())
    axs[1, 2].set(title="Range–radial velocity density", xlabel="range (m)", ylabel="radial velocity")
    for ax, sc, label in [(axs[0, 0], s0, "intensity"), (axs[0, 1], s1, "radial velocity"),
                          (axs[0, 2], s2, "intensity"), (axs[1, 0], s3, "radial velocity"),
                          (axs[1, 1], s4, "intensity")]:
        if sc is not None:
            fig.colorbar(sc, ax=ax, label=label, fraction=.045)
    fig.colorbar(h[3], ax=axs[1, 2], label="point count", fraction=.045)
    fig.suptitle(f"{name} frame {frame:03d} | bag={meta['bag_time']:.3f} header={meta['header_time']:.3f}", fontsize=14)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def plot_stages(path, name, frame, stages, intensity, velocity):
    fig, axs = plt.subplots(2, 3, figsize=(18, 10), dpi=180, constrained_layout=True)
    titles = ["Raw", "Intensity [10,250]", "Height z>10", "Background pre-neighborhood",
              "Neighborhood-filtered", "Final DBSCAN clusters"]
    arrays = [stages[k] for k in ("raw", "intensity", "height", "background", "neighborhood", "cluster")]
    for j, (ax, pts, title) in enumerate(zip(axs.ravel(), arrays, titles)):
        if j == 5 and len(pts):
            labels = DBSCAN(eps=2.0, min_samples=1).fit_predict(pts)
            color = labels
            cmap, vmin, vmax = "tab20", None, None
        else:
            color = np.linalg.norm(pts, axis=1) if len(pts) else np.array([])
            cmap, vmin, vmax = "viridis", 0, 500
        scatter(ax, pts, color, title, cmap=cmap, vmin=vmin, vmax=vmax, limit=100000, seed=frame+j)
        ax.set_xlim(0, 520)
        ax.set_ylim(-160, 160)
    fig.suptitle(f"{name} frame {frame:03d}: current preprocessing survival", fontsize=14)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def plot_counts(path, name, rows):
    frame = np.array([r["frame"] for r in rows])
    fig, axs = plt.subplots(2, 1, figsize=(14, 8), dpi=180, sharex=True, constrained_layout=True)
    for key, label in [("intensity_n", "intensity"), ("height_n", "height"),
                       ("background_n", "background pre-neighborhood"),
                       ("neighborhood_n", "neighborhood/final")]:
        axs[0].plot(frame, [r[key] for r in rows], label=label, lw=1.4)
    axs[0].axvspan(-.5, 7.5, alpha=.12, color="gray", label="background initialization")
    axs[0].set_yscale("symlog", linthresh=1)
    axs[0].set_ylabel("points per frame")
    axs[0].legend(ncol=3)
    axs[0].grid(alpha=.2)
    pre = np.array([r["background_n"] for r in rows])
    post = np.array([r["neighborhood_n"] for r in rows])
    removed = np.divide(pre-post, pre, out=np.zeros_like(pre, dtype=float), where=pre>0)
    axs[1].plot(frame, removed * 100, color="crimson")
    axs[1].set(xlabel="frame", ylabel="pre-background candidates removed\nby neighborhood (%)", ylim=(-2, 102))
    axs[1].grid(alpha=.2)
    fig.suptitle(name)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def plot_temporal(path, name, cache):
    fig, axs = plt.subplots(1, 3, figsize=(18, 6), dpi=180, constrained_layout=True)
    for ax, key, title in zip(axs, ("height", "background", "neighborhood"),
                              ("Height-filtered", "Background pre-neighborhood", "Final candidates")):
        pts, times = [], []
        for item in cache:
            p = item[key]
            if len(p):
                pts.append(p)
                times.append(np.full(len(p), item["frame"]))
        if pts:
            p = np.vstack(pts); t = np.concatenate(times)
            idx = sample_idx(len(p), 180000, 19)
            sc = ax.scatter(p[idx, 0], p[idx, 1], c=t[idx], s=.5, cmap="turbo", linewidths=0, rasterized=True)
            fig.colorbar(sc, ax=ax, label="frame")
        ax.set(title=title, xlabel="x (m)", ylabel="y (m)", xlim=(0, 520), ylim=(-160, 160))
        ax.grid(alpha=.15)
    fig.suptitle(f"{name}: temporal overlay")
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def run_sequence(name, bag, outdir, conn, start, limit, representative):
    seq_dir = outdir / name
    seq_dir.mkdir(parents=True, exist_ok=True)
    filt = StageFilter()
    rows, cache, meta_rows = [], [], []
    rep = set(representative)
    for source_frame, (values, meta) in enumerate(iter_bag_messages(bag, conn, start + limit)):
        if source_frame < start:
            continue
        frame = source_frame - start
        # Rich 80-byte records use the documented sensor axes and match the PCD only
        # after [x,z,-y]. The selected 20-byte 300 m bag already matches its PCD in
        # processed coordinates, byte-for-byte, so transforming it again is wrong.
        points = transform_xyz(values) if meta["point_step"] == 80 else np.column_stack(
            [values["x"], values["y"], values["z"]])
        intensity = values["intensity"]
        velocity = values["velocity"]
        finite = np.isfinite(points).all(axis=1) & np.isfinite(intensity) & np.isfinite(velocity)
        points, intensity, velocity = points[finite], intensity[finite], velocity[finite]
        imask = (intensity >= 10) & (intensity <= 250)
        ip, ii, iv = points[imask], intensity[imask], velocity[imask]
        hmask = ip[:, 2] > 10
        hp, hi, hv = ip[hmask], ii[hmask], iv[hmask]
        pre, post, prob, ratio, valid = filt.process(hp)
        bp, np_, bv, nv = hp[pre], hp[post], hv[pre], hv[post]
        nclusters = len(set(DBSCAN(eps=2.0, min_samples=1).fit_predict(np_))) if len(np_) else 0
        row = {
            "frame": frame, "source_frame": source_frame, **meta,
            "raw_n": int(len(points)), "intensity_n": int(len(ip)),
            "height_n": int(len(hp)), "background_n": int(len(bp)),
            "neighborhood_n": int(len(np_)), "clusters_n": int(nclusters),
        }
        rows.append(row)
        cache.append({"frame": frame, "height": hp.astype(np.float32),
                      "height_i": hi.astype(np.float32), "height_v": hv.astype(np.float32),
                      "background": bp.astype(np.float32), "background_v": bv.astype(np.float32),
                      "neighborhood": np_.astype(np.float32), "neighborhood_v": nv.astype(np.float32),
                      "pre_probability": prob[pre].astype(np.float32),
                      "pre_static_ratio": ratio[pre].astype(np.float32)})
        meta_rows.append(meta)
        if frame in rep:
            plot_views(seq_dir / f"frame_{frame:03d}_raw_views.png", name, frame, points, intensity, velocity, meta)
            stages = {"raw": points, "intensity": ip, "height": hp, "background": bp,
                      "neighborhood": np_, "cluster": np_}
            plot_stages(seq_dir / f"frame_{frame:03d}_pipeline.png", name, frame, stages, ii, iv)
        print(name, frame, row["intensity_n"], row["height_n"], row["background_n"], row["neighborhood_n"], flush=True)

    (seq_dir / "stage_counts.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    np.savez_compressed(seq_dir / "stage_cache.npz", frames=np.array(cache, dtype=object))
    plot_counts(seq_dir / "stage_counts.png", name, rows)
    plot_temporal(seq_dir / "temporal_overlay.png", name, cache)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="results/data_archaeology")
    parser.add_argument("--sequence", choices=("100m", "300m", "cross", "all"), default="all")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = root / args.output
    out.mkdir(parents=True, exist_ok=True)
    specs = {
        "100m": (root / "bag/20260205/100m横飞速度10ms/100m_v10m_s_2026-02-05-15-08-28.bag", 0, 0, 46, [8, 20, 35, 45]),
        # The processed directory starts at source frame 7/header 1649364319, so
        # use exactly those 50 frames when reproducing the current pipeline.
        "300m": (root / "bag/300m_v10_ms_2026-03-19-14-11-42/300m_v10_ms_2026-03-19-14-11-42.bag", 0, 7, 50, [8, 20, 35, 49]),
        "cross": (root / "bag/81-82-pm-cross-x10_2025-11-19-15-57-13.bag", 0, 0, 60, [8, 20, 40, 59]),
    }
    chosen = specs if args.sequence == "all" else {args.sequence: specs[args.sequence]}
    summary = {}
    for name, (bag, conn, start, limit, rep) in chosen.items():
        summary[name] = run_sequence(name, bag, out, conn, start, limit, rep)
    (out / "run_summary.json").write_text(json.dumps({k: len(v) for k, v in summary.items()}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
