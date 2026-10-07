#!/usr/bin/env python3
"""Regenerate testdata/frames/mav0: a tiny synthetic EuRoC-layout sequence.

5 frames at 20 Hz of a smooth 160x120 texture shifting 1 px/frame, and 41
static IMU samples at 200 Hz (gravity on +x as at a level EuRoC start, no
rotation). Deterministic; stdlib only (zlib-written PNGs). The sensor.yaml
files are copied from ../mav0 (the real EuRoC calibration).

Usage: python3 gen_frames_fixture.py   (run from this directory)
"""
import math
import pathlib
import shutil
import struct
import zlib

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE / "frames" / "mav0"
W, H = 160, 120
T0 = 1403715273262142976  # ns
DT_CAM, DT_IMU = 50_000_000, 5_000_000


def write_png(path: pathlib.Path, px: list[int]) -> None:
    raw = b"".join(b"\x00" + bytes(px[r * W:(r + 1) * W]) for r in range(H))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 0, 0, 0, 0)) +
                     chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def main() -> None:
    for sensor in ("cam0", "imu0"):
        (ROOT / sensor).mkdir(parents=True, exist_ok=True)
        shutil.copy(HERE / "mav0" / sensor / "sensor.yaml", ROOT / sensor / "sensor.yaml")
    (ROOT / "cam0" / "data").mkdir(exist_ok=True)

    lines = ["#timestamp [ns],filename"]
    for k in range(5):
        ts = T0 + k * DT_CAM
        px = [int(128 + 60 * math.sin((x + k) * 0.35) * math.cos(y * 0.27) + 40 * math.sin((x + k + y) * 0.11))
              for y in range(H) for x in range(W)]
        write_png(ROOT / "cam0" / "data" / f"{ts}.png", [max(0, min(255, v)) for v in px])
        lines.append(f"{ts},{ts}.png")
    (ROOT / "cam0" / "data.csv").write_text("\n".join(lines) + "\n")

    lines = ["#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
             "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"]
    lines += [f"{T0 + i * DT_IMU},0.0,0.0,0.0,9.81,0.0,0.0" for i in range(41)]
    (ROOT / "imu0" / "data.csv").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
