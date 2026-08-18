#!/usr/bin/env python3
"""Normalize Rerun ROS 2 MCAP TF frames for Web Viewer.

Rerun's MCAP importer stores TF child/parent frames with a leading "/",
but stores ROS 2 header frame_id values without it. The result is a
transform path error in the Rerun Web Viewer. This script rewrites only
Transform3D child_frame/parent_frame strings and adds a root
CoordinateFrame("odom"), then writes an .rrd.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pyarrow as pa

import rerun as rr
from rerun.experimental import Chunk, LazyChunkStream, McapReader


def _fix_tf(c: Chunk) -> Chunk:
    batch = c.to_record_batch()
    for name in ("Transform3D:child_frame", "Transform3D:parent_frame"):
        idx = batch.schema.get_field_index(name)
        if idx < 0:
            continue
        field = batch.schema.field(idx)
        arr = batch.column(idx)
        values = [
            [s[1:] if s.startswith("/") else s for s in row]
            for row in arr.to_pylist()
        ]
        batch = batch.set_column(idx, field, pa.array(values, type=field.type))
    return Chunk.from_record_batch(batch)[0]


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {Path(sys.argv[0]).name} INPUT.mcap OUTPUT.rrd")
        return 2

    src = sys.argv[1]
    out = sys.argv[2]
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    stream = McapReader(src).stream().map(_fix_tf)
    root = Chunk.from_columns(
        "/",
        indexes=[],
        columns=rr.CoordinateFrame.columns(frame="odom"),
    )
    merged = LazyChunkStream.merge(stream, LazyChunkStream.from_iter([root]))
    recording_id = Path(out).stem.removesuffix("_normalized")
    merged.write_rrd(out, application_id="mobin-calib", recording_id=recording_id)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
