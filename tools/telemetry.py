#!/usr/bin/env python3
"""Read a SumoSkripsie run log off the ESP32 and plot it.

The firmware writes fixed-size binary records into the raw `telemetry` flash
partition (see src/telemetry.c). This pulls them back with esptool, parses them
with a numpy dtype, and draws the desired-vs-actual plots.

Typical use, once per tuning iteration:

    pio run -t upload          # flash the new gains
    # unplug, run the robot, plug back in
    python tools/telemetry.py --plot

Every plot is titled with the gains that produced it, read from the log header,
so runs cannot be confused with one another later.

Requires: numpy, matplotlib, esptool (esptool ships with PlatformIO).
"""

import argparse
import csv
import os
import struct
import subprocess
import sys
import tempfile

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARTITIONS_CSV = os.path.join(REPO_ROOT, "partitions.csv")

TELEM_MAGIC = 0x4D4C4554
TELEM_FOOTER_MAGIC = 0x444E4554

HEADER_OFFSET = 0
FOOTER_OFFSET = 64
RECORDS_OFFSET = 128
HEADER_SIZE = 64
FOOTER_SIZE = 64

# Must match the scalings in include/telemetry.h.
ANGLE_SCALE = 10.0
RATE_SCALE = 10.0
ACCEL_SCALE = 1000.0
TERM_SCALE = 10.0

FLAG_TARGET_VISIBLE = 0x01
FLAG_CONTACT = 0x02

STATE_NAMES = {0: "IDLE", 1: "WAITING", 2: "SEARCHING", 3: "APPROACHING", 4: "STOPPED"}

# Mirrors telem_record_t. Little-endian, packed, 32 bytes.
RECORD_DTYPE = np.dtype(
    [
        ("t_us", "<u4"),
        ("yaw_ddeg", "<i2"),
        ("setpoint_ddeg", "<i2"),
        ("yaw_rate_ddps", "<i2"),
        ("accel_fwd_mg", "<i2"),
        ("accel_lat_mg", "<i2"),
        ("p_term", "<i2"),
        ("i_term", "<i2"),
        ("d_term", "<i2"),
        ("motor_a", "<i2"),
        ("motor_b", "<i2"),
        ("range_left_mm", "<u2"),
        ("range_right_mm", "<u2"),
        ("state", "u1"),
        ("flags", "u1"),
        ("reserved", "<u2"),
    ]
)

HEADER_STRUCT = struct.Struct("<IHHII5f3hH20s")
FOOTER_STRUCT = struct.Struct("<IIII48s")

assert RECORD_DTYPE.itemsize == 32, RECORD_DTYPE.itemsize
assert HEADER_STRUCT.size == 64, HEADER_STRUCT.size
assert FOOTER_STRUCT.size == 64, FOOTER_STRUCT.size


class TelemetryError(Exception):
    pass


def find_partition(name="telemetry"):
    """Parse partitions.csv so the offset is never duplicated by hand."""
    if not os.path.exists(PARTITIONS_CSV):
        raise TelemetryError("partitions.csv not found at %s" % PARTITIONS_CSV)

    with open(PARTITIONS_CSV, newline="") as handle:
        for row in csv.reader(handle):
            fields = [field.strip() for field in row if field.strip()]
            if len(fields) < 5 or fields[0].startswith("#"):
                continue
            if fields[0] == name:
                return int(fields[3], 0), int(fields[4], 0)
    raise TelemetryError("no '%s' partition in partitions.csv" % name)


def read_flash(port, offset, size, baud):
    """Pull a byte range off the chip. esptool resets into the bootloader to do
    this, which is harmless - flash contents survive."""
    with tempfile.TemporaryDirectory() as workdir:
        target = os.path.join(workdir, "chunk.bin")
        command = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32",
            "--baud", str(baud),
        ]
        if port:
            command += ["--port", port]
        command += ["read_flash", hex(offset), hex(size), target]

        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            raise TelemetryError(
                "esptool failed (is the board connected, and the serial monitor "
                "closed?)\n%s\n%s" % (result.stdout, result.stderr)
            )
        with open(target, "rb") as handle:
            return handle.read()


def parse_header(raw):
    fields = HEADER_STRUCT.unpack(raw[:HEADER_SIZE])
    if fields[0] != TELEM_MAGIC:
        raise TelemetryError(
            "no valid log header (magic 0x%08X). Has a run been completed since "
            "the last flash erase?" % fields[0]
        )
    return {
        "version": fields[1],
        "record_size": fields[2],
        "run_number": fields[3],
        "kp": fields[5],
        "ki": fields[6],
        "kd": fields[7],
        "integral_limit": fields[8],
        "sample_rate_hz": fields[9],
        "drive_speed": fields[10],
        "turn_speed": fields[11],
        "target_distance_mm": fields[12],
        "build": fields[14].split(b"\x00")[0].decode("ascii", "replace"),
    }


def parse_footer(raw):
    fields = FOOTER_STRUCT.unpack(raw[:FOOTER_SIZE])
    if fields[0] != TELEM_FOOTER_MAGIC:
        return None
    return {
        "sample_count": fields[1],
        "dropped_count": fields[2],
        "duration_ms": fields[3],
    }


def load_run(port, baud, max_records):
    offset, part_size = find_partition()

    meta = read_flash(port, offset, RECORDS_OFFSET, baud)
    header = parse_header(meta[HEADER_OFFSET:HEADER_OFFSET + HEADER_SIZE])
    footer = parse_footer(meta[FOOTER_OFFSET:FOOTER_OFFSET + FOOTER_SIZE])

    if header["record_size"] != RECORD_DTYPE.itemsize:
        raise TelemetryError(
            "record size mismatch: firmware says %d bytes, this script expects %d. "
            "telem_record_t and RECORD_DTYPE have drifted apart."
            % (header["record_size"], RECORD_DTYPE.itemsize)
        )

    if footer is not None:
        count = footer["sample_count"]
    else:
        # Run was interrupted before the footer was written. Read a bounded
        # chunk and trim on erased flash instead.
        print("warning: no footer - run was interrupted, trimming on erased flash",
              file=sys.stderr)
        count = max_records

    size = count * RECORD_DTYPE.itemsize
    available = part_size - RECORDS_OFFSET
    size = min(size, available)
    if size <= 0:
        raise TelemetryError("log contains no samples")

    payload = read_flash(port, offset + RECORDS_OFFSET, size, baud)
    data = np.frombuffer(payload, dtype=RECORD_DTYPE, count=size // RECORD_DTYPE.itemsize)

    if footer is None:
        valid = data["t_us"] != 0xFFFFFFFF
        data = data[valid]

    return header, footer, data


def to_engineering_units(data):
    """Scale the fixed-point record into a dict of float arrays."""
    return {
        "t": data["t_us"] / 1e6,
        "yaw": data["yaw_ddeg"] / ANGLE_SCALE,
        "setpoint": data["setpoint_ddeg"] / ANGLE_SCALE,
        "yaw_rate": data["yaw_rate_ddps"] / RATE_SCALE,
        "accel_fwd": data["accel_fwd_mg"] / ACCEL_SCALE,
        "accel_lat": data["accel_lat_mg"] / ACCEL_SCALE,
        "p": data["p_term"] / TERM_SCALE,
        "i": data["i_term"] / TERM_SCALE,
        "d": data["d_term"] / TERM_SCALE,
        "motor_a": data["motor_a"].astype(float),
        "motor_b": data["motor_b"].astype(float),
        "range_left": np.where(data["range_left_mm"] == 0xFFFF, np.nan,
                               data["range_left_mm"]).astype(float),
        "range_right": np.where(data["range_right_mm"] == 0xFFFF, np.nan,
                                data["range_right_mm"]).astype(float),
        "state": data["state"],
        "visible": (data["flags"] & FLAG_TARGET_VISIBLE) != 0,
        "contact": (data["flags"] & FLAG_CONTACT) != 0,
    }


def summarise(header, footer, run):
    error = run["setpoint"] - run["yaw"]
    error = (error + 180.0) % 360.0 - 180.0

    print("run %d   kp=%.3f ki=%.3f kd=%.3f   built %s"
          % (header["run_number"], header["kp"], header["ki"], header["kd"],
             header["build"]))
    print("samples %d   duration %.2f s   mean rate %.1f Hz"
          % (len(run["t"]), run["t"][-1] if len(run["t"]) else 0.0,
             (len(run["t"]) / run["t"][-1]) if len(run["t"]) and run["t"][-1] > 0 else 0.0))
    if footer and footer["dropped_count"]:
        print("WARNING: %d samples dropped - the run outlasted the buffer"
              % footer["dropped_count"])

    print("heading error: rms %.2f deg   max |e| %.2f deg"
          % (float(np.sqrt(np.mean(error ** 2))), float(np.max(np.abs(error)))))
    print("target visible %.0f%% of samples, contact flagged %.0f%%"
          % (100.0 * np.mean(run["visible"]), 100.0 * np.mean(run["contact"])))

    # Sample-interval jitter is the health check on the control loop: if the
    # IMU data-ready clock is being disturbed, the PID derivative is suspect.
    if len(run["t"]) > 2:
        dt = np.diff(run["t"]) * 1000.0
        print("sample interval: mean %.2f ms, sd %.2f ms, max %.2f ms"
              % (dt.mean(), dt.std(), dt.max()))


def plot(header, run, output):
    import matplotlib.pyplot as plt

    t = run["t"]
    error = run["setpoint"] - run["yaw"]
    error = (error + 180.0) % 360.0 - 180.0

    figure, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    figure.suptitle(
        "Run %d   Kp=%.3f  Ki=%.3f  Kd=%.3f   (%d samples)"
        % (header["run_number"], header["kp"], header["ki"], header["kd"], len(t)),
        fontsize=13,
    )

    # Shade the intervals where the opponent was actually in view - it explains
    # every setpoint step below.
    def shade(axis):
        if not len(run["visible"]):
            return
        edges = np.diff(run["visible"].astype(int))
        starts = list(np.where(edges == 1)[0] + 1)
        ends = list(np.where(edges == -1)[0] + 1)
        if run["visible"][0]:
            starts.insert(0, 0)
        if run["visible"][-1]:
            ends.append(len(t) - 1)
        for start, end in zip(starts, ends):
            axis.axvspan(t[start], t[end], color="tab:green", alpha=0.07, lw=0)

    axes[0].plot(t, run["setpoint"], label="desired heading", lw=1.6, color="tab:blue")
    axes[0].plot(t, run["yaw"], label="actual heading", lw=1.2, color="tab:red")
    axes[0].set_ylabel("heading (deg)")
    axes[0].legend(loc="upper right")
    axes[0].set_title("Desired vs actual", loc="left", fontsize=10)
    shade(axes[0])

    axes[1].plot(t, error, lw=1.0, color="tab:purple")
    axes[1].axhline(0, color="grey", lw=0.6)
    axes[1].set_ylabel("error (deg)")
    shade(axes[1])

    # Plotting the terms separately is the point: when the robot oscillates you
    # need to see which one caused it.
    axes[2].plot(t, run["p"], label="P", lw=1.0)
    axes[2].plot(t, run["i"], label="I", lw=1.0)
    axes[2].plot(t, run["d"], label="D", lw=1.0)
    axes[2].plot(t, run["p"] + run["i"] + run["d"], label="sum", lw=0.8,
                 color="black", alpha=0.5)
    axes[2].set_ylabel("PID contribution")
    axes[2].legend(loc="upper right", ncol=4)
    shade(axes[2])

    axes[3].plot(t, run["motor_a"], label="motor A", lw=1.0)
    axes[3].plot(t, run["motor_b"], label="motor B", lw=1.0)
    axes[3].axhline(255, color="grey", ls=":", lw=0.6)
    axes[3].axhline(-255, color="grey", ls=":", lw=0.6)
    axes[3].set_ylabel("output (duty)")
    axes[3].legend(loc="upper right")
    shade(axes[3])

    axes[4].plot(t, run["accel_fwd"], label="forward accel (g)", lw=1.0, color="tab:orange")
    contact = run["contact"]
    if contact.any():
        axes[4].fill_between(t, 0, 1, where=contact, transform=axes[4].get_xaxis_transform(),
                             color="tab:red", alpha=0.15, lw=0, label="contact")
    axes[4].set_ylabel("accel (g)")
    axes[4].set_xlabel("time (s)")
    axes[4].legend(loc="upper right")

    for axis in axes:
        axis.grid(alpha=0.25)

    figure.tight_layout(rect=(0, 0, 1, 0.98))

    if output:
        figure.savefig(output, dpi=130)
        print("wrote %s" % output)
    else:
        plt.show()


def export_csv(run, path):
    columns = [key for key in run if key not in ("state",)]
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s"] + [c for c in columns if c != "t"])
        rows = zip(run["t"], *[run[c] for c in columns if c != "t"])
        writer.writerows(rows)
    print("wrote %s" % path)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=921600, help="esptool baud rate")
    parser.add_argument("--plot", action="store_true", help="draw the run")
    parser.add_argument("--save", metavar="PNG", help="save the plot instead of showing it")
    parser.add_argument("--csv", metavar="FILE", help="also export the samples as CSV")
    parser.add_argument("--raw", metavar="FILE", help="save the raw record bytes")
    parser.add_argument("--max-records", type=int, default=3000,
                        help="records to read when the log has no footer")
    arguments = parser.parse_args()

    try:
        header, footer, data = load_run(arguments.port, arguments.baud,
                                        arguments.max_records)
    except TelemetryError as error:
        print("error: %s" % error, file=sys.stderr)
        return 1

    if len(data) == 0:
        print("error: log is empty", file=sys.stderr)
        return 1

    run = to_engineering_units(data)
    summarise(header, footer, run)

    if arguments.raw:
        with open(arguments.raw, "wb") as handle:
            handle.write(data.tobytes())
        print("wrote %s" % arguments.raw)
    if arguments.csv:
        export_csv(run, arguments.csv)
    if arguments.plot or arguments.save:
        plot(header, run, arguments.save)

    return 0


if __name__ == "__main__":
    sys.exit(main())
