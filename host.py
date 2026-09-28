#!/usr/bin/env python3
# ha1, the pc side. two parts:
#   capture: three recordings in a row (board still, shaken, rocked), each after a 5 s countdown.
#            for each it sends 's' to the pico and saves every line up to DONE in
#            output/<still|shake|rock>_run_log.txt
#   analyse: for each recording, a short summary, the fft redone with numpy, the two files the
#            course fft.py reads and four plots, all prefixed with the recording's name. then
#            overall_data.txt with every number the report uses, from all three recordings
#
#   python3 host.py                         capture + analyse (finds the port itself)
#   python3 host.py --port /dev/ttyACM0     the same with a fixed port
#   python3 host.py --analyse               only analyse the saved logs, no board needed
#
# capture needs pyserial, the analysis needs numpy and matplotlib.

import argparse
import glob
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(HERE, "output")
SECTIONS = ["info", "raw", "stats", "quantization", "quant_samples", "zcr", "orientation",
            "downsampled", "fft_input", "fft", "fft_peaks", "timing", "fft_timing"]
ACCEL = ["ax", "ay", "az"]
RUNS = [("still", "leave the board STILL on the table"),
        ("shake", "pick the board up. at GO, SHAKE it hard and keep going until it says STOP"),
        ("rock", "at GO, ROCK the board gently side to side and keep going until it says STOP")]


def find_port():
    # mac, linux, and the udev name i gave the board in my vm
    for pattern in ["/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/pico"]:
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


def open_port(serial, port_name):
    # usb serial ignores the baud rate, but never use 1200: that reboots the pico into bootsel
    try:
        port = serial.Serial(port_name, 115200, timeout=0.25)
    except serial.SerialException as e:
        sys.exit(f"cannot open {port_name}: {e}. is another program (logs.sh, minicom) using it?")
    # after flashing or plugging in, the pico greets and measures the gyro bias now (about 0.4 s).
    # a board that already ran says nothing, so wait at most 1.5 s for its "send s" line
    deadline = time.time() + 1.5
    while time.time() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if line:
            print("  pico: " + line)
        if line.startswith("error"):
            sys.exit("the pico reported an error, see above")
        if line.startswith("send s"):
            break
    port.timeout = 1
    return port


def countdown():
    for k in [5, 4, 3, 2, 1]:
        print(f"  {k}", end="", flush=True)
        time.sleep(1)
    print("  GO, recording 1 s", flush=True)


def read_run(port):
    # every line the pico prints for one 's', up to its DONE line
    lines = []
    started = False            # a DONE before our "run:" line would be the tail of an older run
    deadline = time.time() + 90
    while time.time() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue           # readline waited 1 s, the pico is still busy
        lines.append(line)
        if line.startswith("error"):
            sys.exit("the pico reported an error: " + line)
        started = started or line.startswith("run:")
        if started and line.startswith("DONE"):
            return lines, True
    return lines, False


def capture(port_name, data_dir):
    # three recordings in a row, each after a 5 s countdown, so there is time to get the board ready.
    # the port stays open for all three, the gyro bias is measured when it is first opened
    try:
        import serial
    except ImportError:
        sys.exit("capture needs pyserial: pip install pyserial (ubuntu: sudo apt install python3-serial)")
    port_name = port_name or find_port()
    if not port_name:
        sys.exit("no pico serial port found. is the pico's own micro-usb plugged in and running this firmware?")
    os.makedirs(data_dir, exist_ok=True)
    # open the port first: a busy port shows up at once, and the gyro bias is measured now,
    # while the board still lies on the table
    print("put the board flat on the table and do not touch it", flush=True)
    port = open_port(serial, port_name)
    for name, what in RUNS:
        print(f"\n{name}: {what}", flush=True)
        countdown()
        port.write(b"s")
        time.sleep(1.2)       # the recording takes 1.024 s
        print("  STOP: 1 s recorded. the pico is processing it, about 5 s", flush=True)
        lines, done = read_run(port)
        # an incomplete log goes to its own file, so it never replaces the last good one
        log_path = os.path.join(data_dir, f"{name}_run_log.txt" if done else f"{name}_run_log.partial.txt")
        with open(log_path, "w") as f:
            f.write("\n".join(lines) + "\n")
        print(f"  saved {len(lines)} lines to {log_path}", flush=True)
        if not done:
            sys.exit("no DONE line within 90 s, the log is incomplete. capture again")
    port.close()


def read_sections(path):
    # {"info": [lines], "raw": [lines], ...}. lines outside a section are messages
    if not os.path.exists(path):
        sys.exit(f"no {path}, record first (python3 host.py or ./run.sh)")
    sections = {}
    name = None
    with open(path, errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#begin "):
                name = line[7:]
                sections[name] = []
            elif line.startswith("#end "):
                name = None
            elif name and line:
                sections[name].append(line)
    missing = [s for s in SECTIONS if s not in sections]
    if missing:
        sys.exit(f"{path} has no {', '.join(missing)} section. was the capture cut short?")
    return sections


def table(lines):
    # csv lines -> {column name: list of text values}
    header = lines[0].split(",")
    rows = [line.split(",") for line in lines[1:]]
    return {name: [row[k] for row in rows] for k, name in enumerate(header)}


def gyro_turning(sections, info):
    # how far the board turned by the gyroscope: its rate minus the start-up bias, added up.
    # returns the largest minus the smallest angle per axis, in degrees
    raw, spans = table(sections["raw"]), {}
    for axis in "xyz":
        angle = low = high = 0.0
        for rate in raw[f"g{axis}_dps"]:
            angle += (float(rate) - float(info[f"gyro_bias_{axis}_dps"])) / float(info["fs_hz"])
            low, high = min(low, angle), max(high, angle)
        spans["g" + axis] = high - low
    return spans


def print_summary(sections, info):
    print(f"sampling: {info['achieved_hz']} hz, late samples {info['late_samples']}, "
          f"imu read {info['read_us_avg']} us average, {info['read_us_max']} us max")
    q = table(sections["quantization"])
    print("quantization, measured vs theory:\n  axis bits   snr_db  snr_th      rmse   rmse_th  clips")
    for k in range(len(q["axis"])):
        print(f"  {q['axis'][k]:>4} {q['bits'][k]:>4} {float(q['snr_db'][k]):8.2f} "
              f"{float(q['theory_snr_db'][k]):7.2f}  {float(q['rmse'][k]):8.2e}  "
              f"{float(q['theory_rmse'][k]):8.2e} {q['clips'][k]:>6}")
    spans = gyro_turning(sections, info)
    print("gyro turning added up, largest minus smallest angle (deg): "
          + ", ".join(f"{axis} {span:.1f}" for axis, span in spans.items()))


def compare_fft(np, sections, fs, n):
    # the fft input the mcu printed, through numpy with the same hamming window and the course
    # scaling (2/n)/0.54, compared with the spectrum the mcu printed.
    # np comes in as an argument: numpy is imported only for the analysis, so capture works without it
    fft_in, fft = table(sections["fft_input"]), table(sections["fft"])
    peaks, down = table(sections["fft_peaks"]), table(sections["downsampled"])
    freq = np.array(fft["freq_hz"], dtype=float)
    print("fft: mcu top 5 bins per accel axis, and the same fft done by numpy")
    spectra, checks = {}, {}
    for axis in ACCEL:
        mcu = np.array(fft[axis], dtype=float)
        ref = np.abs(np.fft.rfft(np.array(fft_in[axis], dtype=float) * np.hamming(n))) * (2.0 / n) / 0.54
        mcu_top = [int(b) for name, b in zip(peaks["axis"], peaks["bin"]) if name == axis]
        # like the course fft.py: sort bins 1.. by amplitude, biggest first, take 5 (+1: bin 0 skipped)
        np_top = [int(b) for b in np.argsort(ref[1:])[::-1][:5] + 1]
        # the 250 hz version: 256 points, so its bins are 250/256 = 1000/1024 hz apart, the same bins
        slow = np.array(down[axis + "_g"], dtype=float)
        slow_amp = np.abs(np.fft.rfft((slow - slow.mean()) * np.hamming(len(slow))))
        slow_top = [int(b) for b in np.argsort(slow_amp[1:])[::-1][:5] + 1]
        diff = np.max(np.abs(mcu - ref))
        # sorted, like fft.py: two near-equal peaks swapping places are still the same five
        checks[axis] = {"diff": diff, "rel": diff / np.max(ref), "same": sorted(mcu_top) == sorted(np_top),
                        "slow_same": sorted(mcu_top) == sorted(slow_top), "peak_hz": freq[mcu_top[0]],
                        "peak_g": mcu[mcu_top[0]]}
        print(f"  {axis}: " + ", ".join(f"{freq[b]:.2f} hz ({mcu[b]:.4f} g)" for b in mcu_top))
        print(f"      max |mcu - numpy| {diff:.2e}, numpy top 5 bins {'the same' if checks[axis]['same'] else 'DIFFERENT'}, "
              f"at 250 hz {'the same' if checks[axis]['slow_same'] else 'DIFFERENT'}")
        spectra[axis] = (mcu, ref)
    return freq, spectra, checks


def write_course_files(sections, info, data_dir, run):
    # the two files the course python_examples/fft.py reads: the fft input of ax on the first
    # line (fft.py only reads that one), and bin,freq,amp of the mcu top 5 of the same axis.
    # fft.py wants them without the prefix: copy them to raw_imu_mcu.txt and out_fft_mcu.txt first
    peaks = table(sections["fft_peaks"])
    with open(os.path.join(data_dir, f"{run}_raw_imu_mcu.txt"), "w") as f:
        f.write(",".join(table(sections["fft_input"])["ax"]) + "\n")
        f.write(f"fs_hz={info['fs_hz']},n={info['n']},axis=ax\n")
    with open(os.path.join(data_dir, f"{run}_out_fft_mcu.txt"), "w") as f:
        f.write("bin,freq,amp\n")
        for name, b, freq, amp in zip(peaks["axis"], peaks["bin"], peaks["freq_hz"], peaks["amp"]):
            if name == "ax":
                f.write(f"{b},{freq},{amp}\n")


def make_plots(np, plt, sections, fs, freq, spectra, data_dir, run):
    raw, down = table(sections["raw"]), table(sections["downsampled"])
    quant, orient = table(sections["quant_samples"]), table(sections["orientation"])
    t = np.arange(len(raw["i"])) / fs

    def col(columns, name):
        return np.array(columns[name], dtype=float)

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(os.path.join(data_dir, f"{run}_{name}"), dpi=130)
        plt.close(fig)

    # 1) raw imu, with the 250 hz accel as dots. a mean of 4 samples sits in the middle of its 4
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    t_down = (np.arange(len(down["i"])) * 4 + 1.5) / fs
    for name in ["ax_g", "ay_g", "az_g"]:
        line = top.plot(t, col(raw, name), lw=0.8, label=name[:2])[0]   # plot returns a list
        top.plot(t_down, col(down, name), ".", ms=2, color=line.get_color())
    for name in ["gx_dps", "gy_dps", "gz_dps"]:
        bottom.plot(t, col(raw, name), lw=0.8, label=name[:2])
    top.set(ylabel="acceleration (g)", title=f"{run}: raw imu at {fs:.0f} hz (dots: downsampled to 250 hz)")
    bottom.set(ylabel="rotation rate (deg/s)", xlabel="time (s)")
    top.legend(loc="upper right")
    bottom.legend(loc="upper right")
    save(fig, "imu_raw.png")

    # 2) mcu and numpy spectrum on the same axes. log y, or only the peaks show and the rest is 0
    fig, panels = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for panel, axis in zip(panels, ACCEL):
        mcu, ref = spectra[axis]
        panel.semilogy(freq, mcu, lw=1.6, label="mcu (pico fft)")
        panel.semilogy(freq, ref, "--", lw=1.0, label="pc (numpy rfft)")
        panel.set(ylabel=f"{axis} amplitude (g)", ylim=(1e-6, None))
        panel.set_title(f"{run}, {axis}: max |mcu - numpy| = {np.max(np.abs(mcu - ref)):.1e}", fontsize=9)
        panel.legend(loc="upper right")
    panels[-1].set(xlabel="frequency (hz)", xlim=(0, fs / 2))
    save(fig, "fft_mcu_vs_numpy.png")

    # 3) quantization of ax: the first 150 samples, so the steps are visible, and the error
    x, original = np.arange(150), col(quant, "ax_in")[:150]
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    top.plot(x, original, lw=2.0, color="black", label="original (float32)")
    for bits in [16, 8, 4]:
        values = col(quant, f"ax_q{bits}")[:150]
        top.step(x, values, where="mid", lw=1.0, label=f"{bits} bit")
        bottom.plot(x, values - original, lw=0.8, label=f"{bits} bit")
    top.set(ylabel="ax, min-max * 0.875", title=f"{run}: quantization of ax, first 150 samples")
    bottom.set(ylabel="dequantized - original", xlabel="sample")
    top.legend(loc="upper right")
    bottom.legend(loc="upper right")
    save(fig, "quantization.png")

    # 4) orientation: accelerometer only vs complementary filter, and the gyro-only yaw
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for name, label, width in [("pitch_acc", "pitch, accel only", 0.6), ("pitch_cf", "pitch, filter", 1.5),
                               ("roll_acc", "roll, accel only", 0.6), ("roll_cf", "roll, filter", 1.5)]:
        top.plot(t, col(orient, name), lw=width, label=label)
    bottom.plot(t, col(orient, "yaw_gyro"), lw=1.2, color="C4", label="yaw, gyro z integrated")
    top.set(ylabel="degrees", title=f"{run}: pitch and roll")
    bottom.set(ylabel="degrees", xlabel="time (s)", title="yaw: no accel reference, leftover gyro bias drifts")
    top.legend(loc="upper right", fontsize=8)
    bottom.legend(loc="upper left")
    save(fig, "orientation.png")


def analyse(np, plt, data_dir, run):
    log_path = os.path.join(data_dir, f"{run}_run_log.txt")
    print(f"\n{log_path}")
    sections = read_sections(log_path)
    info = dict(line.split("=", 1) for line in sections["info"])
    fs, n = float(info["fs_hz"]), int(info["n"])
    # the pico's usb serial drops output when the pc stops reading, so check the long sections
    for name in ["raw", "orientation", "fft_input"]:
        if len(sections[name]) - 1 != n:
            sys.exit(f"{name} has {len(sections[name]) - 1} rows instead of {n}, lines got lost. capture again")
    print_summary(sections, info)
    freq, spectra, checks = compare_fft(np, sections, fs, n)
    write_course_files(sections, info, data_dir, run)
    make_plots(np, plt, sections, fs, freq, spectra, data_dir, run)
    print(f"wrote {run}_raw_imu_mcu.txt, {run}_out_fft_mcu.txt and 4 {run}_*.png plots to {data_dir}")
    return {"sections": sections, "info": info, "checks": checks}


def write_overall(data_dir, results):
    # every number the report uses, from the three recordings, as small tables with one column per
    # recording, so each number in the text can be found again. raw counts come back from the
    # printed g and deg/s values (x16384, x32.8)
    runs = list(results)
    lines = ["HA1 OVERALL NUMBERS",
             "from " + ", ".join(f"{run}_run_log.txt" for run in runs) + ", written by host.py",
             "every number in report/ha1_report.tex comes from this file"]

    def header(title, columns=None):
        columns = columns or runs
        lines.append("")
        lines.append(f"{title:<44}" + "".join(f"{c:>10}" for c in columns))

    def row(label, values, fmt="{}"):
        lines.append(f"  {label:<42}" + "".join(f"{fmt.format(v):>10}" for v in values))

    def note(text):
        lines.append(f"  {text}")

    info = [results[run]["info"] for run in runs]
    header("SAMPLING")
    row("samples per second", [i["achieved_hz"] for i in info])
    row("late samples", [i["late_samples"] for i in info])
    row("sensor read, average (microseconds)", [i["read_us_avg"] for i in info])
    row("sensor read, longest (microseconds)", [i["read_us_max"] for i in info])
    i = info[0]
    note(f"gyroscope offset, measured once at start-up (deg/s): x {float(i['gyro_bias_x_dps']):.2f}, "
         f"y {float(i['gyro_bias_y_dps']):.2f}, z {float(i['gyro_bias_z_dps']):.2f}")

    checks = [results[run]["checks"] for run in runs]
    header("PICO FFT AGAINST NUMPY (ax, ay, az)")
    row("largest difference (millionths of a g)", [max(c[a]["diff"] for a in ACCEL) * 1e6 for c in checks], "{:.2f}")
    row("largest difference (% of strongest value)", [max(c[a]["rel"] for a in ACCEL) * 100 for c in checks], "{:.4f}")
    row("same five strongest frequencies", ["yes" if all(c[a]["same"] for a in ACCEL) else "no" for c in checks])
    row("same five from the 250 per second data", ["yes" if all(c[a]["slow_same"] for a in ACCEL) else "no" for c in checks])
    for axis in ACCEL:
        row(f"strongest frequency {axis} (Hz)", [c[axis]["peak_hz"] for c in checks], "{:.2f}")
    row("strongest value ay (g)", [c["ay"]["peak_g"] for c in checks], "{:.3f}")
    note(f"frequency step: {1000 / 1024:.2f} Hz (1000 samples per second / 1024 samples)")

    header("ZERO CROSSINGS, ROUGH FREQUENCY (Hz)")
    zcr = [dict(zip(table(results[run]["sections"]["zcr"])["axis"], table(results[run]["sections"]["zcr"])["rough_hz"]))
           for run in runs]
    for axis in ["ax", "ay", "az", "gx", "gy", "gz"]:
        row(axis, [z[axis] for z in zcr])

    header("TIME PER STEP (ms)", runs + ["average"])
    steps = {}
    for run in runs:
        tm = table(results[run]["sections"]["timing"])
        for step, us in zip(tm["step"], tm["us"]):
            steps.setdefault(step, []).append(int(us) / 1000)
    for step, values in steps.items():
        row(step, values + [sum(values) / len(values)], "{:.1f}")
    totals = [sum(steps[s][k] for s in steps if s != "record") for k in range(len(runs))]
    row("processing, without recording (s)", [t / 1000 for t in totals] + [sum(totals) / len(totals) / 1000], "{:.2f}")
    row("statistics, share of processing (%)", [steps["stats"][k] / totals[k] * 100 for k in range(len(runs))]
        + [sum(steps["stats"]) / sum(totals) * 100], "{:.0f}")

    header("FFT TIMING, AVERAGE OF THE RECORDINGS", ["time ms", "between", "buffer"])
    note("between: highest samples per second if the FFT must fit between two samples")
    note("buffer: highest samples per second with a second buffer")
    timing = {}
    for run in runs:
        ft = table(results[run]["sections"]["fft_timing"])
        for k, n in enumerate(ft["n"]):
            timing.setdefault(n, []).append((int(ft["fft_us"][k]) / 1000, float(ft["fs_max_blocking_hz"][k]),
                                             float(ft["fs_max_buffered_hz"][k])))
    for n, values in timing.items():
        mean = [sum(v[k] for v in values) / len(values) for k in range(3)]
        lines.append(f"  {'FFT size ' + n:<42}{mean[0]:>10.1f}{mean[1]:>10.1f}{mean[2]:>10.0f}")

    header("QUANTIZATION (signal-to-noise ratio, dB)")
    normal, exact = [], []
    for run in runs:
        q = table(results[run]["sections"]["quantization"])
        rows = list(zip(q["axis"], q["bits"], [float(v) for v in q["snr_db"]], [float(v) for v in q["theory_snr_db"]]))
        # above 120 db the error is only float rounding: those values sat exactly on the stored levels
        normal.append(max(abs(m - e) for a, b, m, e in rows if m < 120))
        exact += [f"{run} {a} {b} bit: {m:.2f} dB (expected {e:.2f})" for a, b, m, e in rows if m >= 120]
    row("largest measured - expected difference", normal, "{:.2f}")
    if "rock" in results:
        q = table(results["rock"]["sections"]["quantization"])
        ay = [k for k, a in enumerate(q["axis"]) if a == "ay"]
        header("  rock ay", ["16 bit", "8 bit", "4 bit"])
        row("measured", [q["snr_db"][k] for k in ay])
        row("expected for this signal", [q["theory_snr_db"][k] for k in ay])
    if exact:
        lines.append("")
        note("almost no error (the values sat exactly on the stored levels):")
        for text in exact:
            note("  " + text)
    lines.append("")
    note("memory per axis: 32-bit 4096 B, 16-bit 2048 B, 8-bit 1024 B, 4-bit 1024 B as stored (512 B two per byte)")

    header("SENSOR STEPS EACH AXIS SPANS (raw counts)")
    for axis, scale in [("ax_g", 16384), ("ay_g", 16384), ("az_g", 16384),
                        ("gx_dps", 32.8), ("gy_dps", 32.8), ("gz_dps", 32.8)]:
        spans = []
        for run in runs:
            counts = [round(float(v) * scale) for v in table(results[run]["sections"]["raw"])[axis]]
            spans.append(max(counts) - min(counts))
        row(axis[:2], spans)

    header("STATISTICS")
    note("size: 7 numbers x 6 axes x 4 bytes = 168 B, against 1024 x 6 x 2 = 12288 B of raw values (73 times smaller)")
    stats = []
    for run in runs:
        st = table(results[run]["sections"]["stats"])
        rows = [k for k, d in enumerate(st["data"]) if d == "raw"]
        stats.append({st["axis"][k]: {c: st[c][k] for c in ["mean", "std", "min", "max", "mode_count"]} for k in rows})
    row("az average (g)", [float(s["az"]["mean"]) for s in stats], "{:.3f}")
    row("az spread (thousandths of a g)", [float(s["az"]["std"]) * 1000 for s in stats], "{:.1f}")
    row("az smallest (g)", [float(s["az"]["min"]) for s in stats], "{:.3f}")
    row("az largest (g)", [float(s["az"]["max"]) for s in stats], "{:.3f}")
    row("az samples at the 2 g limit", [sum(1 for v in table(results[run]["sections"]["raw"])["az_g"]
                                            if abs(float(v)) >= 1.99993) for run in runs])
    for axis in ["ax", "ay", "az", "gx", "gy", "gz"]:
        row(f"most common value of {axis}, times", [s[axis]["mode_count"] for s in stats])

    header("TILT (degrees, largest minus smallest)")
    spans, turning = [], []
    for run in runs:
        o = table(results[run]["sections"]["orientation"])
        spans.append({name: max(float(v) for v in o[name]) - min(float(v) for v in o[name])
                      for name in ["pitch_acc", "roll_acc", "pitch_cf", "roll_cf", "yaw_gyro"]})
        turning.append(gyro_turning(results[run]["sections"], results[run]["info"]))
    for name, label in [("roll_acc", "roll from gravity"), ("roll_cf", "roll, complementary filter"),
                        ("pitch_acc", "pitch from gravity"), ("pitch_cf", "pitch, complementary filter"),
                        ("yaw_gyro", "yaw from the gyroscope")]:
        row(label, [s[name] for s in spans], "{:.1f}")
    for axis in ["gx", "gy", "gz"]:
        row(f"gyroscope turning {axis}, offset removed", [t[axis] for t in turning], "{:.2f}")

    path = os.path.join(data_dir, "overall_data.txt")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\nwrote {path}")


def main():
    parser = argparse.ArgumentParser(description="ha1 imu: capture from the pico and analyse")
    parser.add_argument("--port", help="serial port, default: the first /dev/cu.usbmodem*, /dev/ttyACM*, /dev/pico")
    parser.add_argument("--analyse", action="store_true", help="only analyse the saved logs, no board needed")
    parser.add_argument("--out", default=DATA_DIR, help="output folder, default: output/ here")
    args = parser.parse_args()
    # line by line even into a pipe (./run.sh | tee), or STOP would show up seconds late
    sys.stdout.reconfigure(line_buffering=True)
    if not args.analyse:
        capture(args.port, args.out)
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")      # no window, only png files
        import matplotlib.pyplot as plt
    except ImportError:
        if args.analyse:
            sys.exit("--analyse needs numpy and matplotlib (or run it in the ias0360-2026 docker image)")
        # the vm's python has pyserial but no numpy: run.sh then analyses in the course docker image
        print("\nno numpy/matplotlib in this python, so no analysis here. run.sh does it in the course")
        print(f"docker image, or later in a python with them: python3 {os.path.join(HERE, 'host.py')} --analyse")
        return
    results = {}
    for run, _ in RUNS:
        results[run] = analyse(np, plt, args.out, run)
    write_overall(args.out, results)


if __name__ == "__main__":
    main()
