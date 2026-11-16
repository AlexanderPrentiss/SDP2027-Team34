#!/usr/bin/env python3
"""
sg_decode.py - LAPTOP-SIDE tool. Never runs on the ESP32.

Reads a ShotGuard log (SG0001.BIN) copied off the microSD card and turns it
into something humans and the analysis app can use:
  - prints the file header and a list of detected touches/shots
  - optionally writes all raw samples to a CSV (open in Excel / pandas)
  - optionally writes the events to a second CSV

It mirrors main/log_format.h byte-for-byte. If that header changes, this
file must change too. This is the starting point for the laptop app (Block 4).

Usage:  python3 sg_decode.py SG0001.BIN [samples.csv] [events.csv]
"""
import struct
import sys

FILE_HDR = struct.Struct("<IHHBBHfHH16s")   # sg_file_header_t   (36 B)
REC_HDR = struct.Struct("<HBBIHBBQ")        # sg_record_header_t (20 B)
SAMPLE = struct.Struct("<Q4H3h3h")          # sg_sample_t        (28 B)

MAGIC, SYNC = 0x474C4753, 0xA55A
REC_DATA, REC_EVENT = 1, 2
KINDS = {1: "touch", 2: "shot"}
ZONES = ["instep", "laces", "outside", "toe"]


def read_log(path):
    with open(path, "rb") as f:
        data = f.read()

    magic, ver, rate, imu, n_fsr, a_lsb, g_lsb, adc_max, _, fw = FILE_HDR.unpack_from(data, 0)
    if magic != MAGIC:
        raise ValueError("not a ShotGuard log")
    if ver != 2:
        raise ValueError(f"log format v{ver}, this decoder reads v2")
    header = dict(rate_hz=rate, imu=("ICM-20649", "MPU6050")[imu],
                  accel_lsb_per_g=a_lsb, gyro_lsb_per_dps=g_lsb,
                  fw=fw.rstrip(b"\0").decode())

    samples, events = [], []
    off = FILE_HDR.size
    while off + REC_HDR.size <= len(data):
        sync, rtype, kind, seq, n, zone, peak_dg, t_us = REC_HDR.unpack_from(data, off)
        if sync != SYNC:            # corrupted/truncated -> scan for next sync word
            off += 1
            continue
        off += REC_HDR.size
        if rtype == REC_EVENT:
            events.append(dict(id=seq, kind=KINDS.get(kind, "?"), zone=ZONES[zone % 4],
                               peak_g=peak_dg / 10, t_us=t_us))
        for _ in range(n):
            if off + SAMPLE.size > len(data):
                break               # file ended mid-record (battery died)
            t, *v = SAMPLE.unpack_from(data, off)
            samples.append([t, *v[0:4],
                            *(x / a_lsb for x in v[4:7]),
                            *(x / g_lsb for x in v[7:10])])
            off += SAMPLE.size
    return header, samples, events


if __name__ == "__main__":
    hdr, samples, events = read_log(sys.argv[1])
    print(hdr)
    secs = (samples[-1][0] - samples[0][0]) / 1e6 if samples else 0
    print(f"{len(samples)} samples ({secs:.1f} s), {len(events)} events")
    for e in events:
        print(f"  #{e['id']:<4} {e['kind']:<5} zone={e['zone']:<8} peak={e['peak_g']:.1f} g  t={e['t_us']/1e6:.3f} s")

    if len(sys.argv) > 2:
        with open(sys.argv[2], "w") as out:
            out.write("t_us,instep,laces,outside,toe,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n")
            for row in samples:
                out.write(",".join(f"{x:.4g}" if isinstance(x, float) else str(x) for x in row) + "\n")
        print("wrote", sys.argv[2])
    if len(sys.argv) > 3:
        with open(sys.argv[3], "w") as out:
            out.write("id,kind,zone,peak_g,t_us\n")
            for e in events:
                out.write(f"{e['id']},{e['kind']},{e['zone']},{e['peak_g']},{e['t_us']}\n")
        print("wrote", sys.argv[3])
