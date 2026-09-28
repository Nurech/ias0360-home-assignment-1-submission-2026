#!/usr/bin/env bash
# builds ha1_report.pdf next to ha1_report.tex. the text is plain latex with the numbers written in;
# only the figure is drawn fresh, from the shake recording that ../run.sh saved.
#   ./build.sh              figure from ../output/shake_run_log.txt
#   ./build.sh DATA_DIR     figure from DATA_DIR/shake_run_log.txt
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
DATA="${1:-$HERE/../output}"
if [ ! -f "$DATA/shake_run_log.txt" ]; then echo "no $DATA/shake_run_log.txt, record first with ../run.sh" >&2; exit 1; fi
DATA="$(cd "$DATA" && pwd)"
LOG="$DATA/shake_run_log.txt"
BUILD="$HERE/.build"

if ! command -v pdflatex > /dev/null; then
    echo "no pdflatex here, run build.sh where LaTeX is installed (the mac)" >&2; exit 1
fi
rm -rf "$BUILD"; mkdir "$BUILD"
trap 'rm -rf "$BUILD"' EXIT    # the temporary folder goes away even when a step fails

# 1) the figure: raw acceleration while shaking, and the az frequencies from the pico and from numpy
cat > "$BUILD/figure.py" << 'PY'
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

log_path, fig_path = sys.argv[1], sys.argv[2]

# the log is text: "#begin name", a csv header, the rows, "#end name"
sections, done, name = {}, set(), None
for line in open(log_path):
    line = line.strip()
    if line.startswith("#begin "):
        name = line[7:]
        sections[name] = []
    elif line.startswith("#end "):
        done.add(line[5:])
        name = None
    elif name:
        sections[name].append(line.split(","))
missing = [s for s in ("raw", "fft_input", "fft", "fft_peaks") if s not in done]
if missing:
    sys.exit(f"{log_path} is incomplete (no {', '.join(missing)} section), record it again")

def column(section, col):
    header, rows = sections[section][0], sections[section][1:]
    i = header.index(col)
    return np.array([float(row[i]) for row in rows])

plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Times", "STIXGeneral", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 7, "axes.linewidth": 0.5, "lines.linewidth": 0.8,
                     "legend.fontsize": 6.5, "legend.frameon": False, "pdf.fonttype": 42})
fig, (top, bottom) = plt.subplots(2, 1, figsize=(3.45, 1.66))

# (a) raw acceleration, with the sensor's +-2 g limit as dotted lines
t = column("raw", "i") / 1000.0
for axis in ["ax", "ay", "az"]:
    top.plot(t, column("raw", axis + "_g"), label=axis)
for limit in (2.0, -2.0):
    top.axhline(limit, color="0.4", lw=0.5, ls=":")
top.set_xlim(0, t[-1])
top.set_xlabel("time (s)", labelpad=1)
top.set_ylabel("acceleration (g)")
top.set_title("(a) raw acceleration while shaking", loc="left", fontsize=7, pad=2)
top.legend(ncol=3, loc="lower right", bbox_to_anchor=(1.0, 0.98), borderaxespad=0.0)

# (b) the pico's spectrum of az, and numpy's from the same input with the same window and scaling
freq = column("fft", "freq_hz")
pico = column("fft", "az")
x = column("fft_input", "az")
pc = np.abs(np.fft.rfft(x * np.hamming(len(x)))) * (2.0 / len(x)) / 0.54
keep = (freq > 0) & (freq <= 60)     # 0 hz is the removed average, nothing to show
bottom.semilogy(freq[keep], pico[keep], label="Pico")
bottom.semilogy(freq[keep], pc[keep], "--", color="#d62728", label="PC")
peaks = sections["fft_peaks"]
bin_col = peaks[0].index("bin")
top5 = [int(row[bin_col]) for row in peaks[1:] if row[0] == "az"]
bottom.plot(freq[top5], pico[top5], "o", ms=2.2, color="k", label="Pico top 5")
bottom.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))   # 0.01 instead of 10^-2
bottom.set_xlim(0, 60)
bottom.set_xlabel("frequency (Hz)", labelpad=1)
bottom.set_ylabel("amplitude (g)")
bottom.set_title("(b) frequencies in az, log scale", loc="left", fontsize=7, pad=2)
bottom.legend(ncol=3, loc="upper right", handlelength=1.5, columnspacing=1.0)
fig.tight_layout(pad=0.2, h_pad=0.6)
fig.savefig(fig_path)
PY
if python3 -c "import numpy, matplotlib" 2> /dev/null; then
    python3 "$BUILD/figure.py" "$LOG" "$BUILD/fig_fft.pdf"
else
    # no numpy in this python (the mac's python3): the course docker image has it
    if [ "$(uname -s)" = Darwin ]; then export DOCKER_CONTEXT="${DOCKER_CONTEXT:-desktop-linux}"; fi
    docker run --rm --user "$(id -u):$(id -g)" --entrypoint python3 -e MPLCONFIGDIR=/tmp/mpl \
        -v "$HERE:$HERE" -v "$DATA:$DATA" ias0360-2026 "$BUILD/figure.py" "$LOG" "$BUILD/fig_fft.pdf"
fi

# 2) latex twice (the second pass fills in the figure and table numbers), then keep only the pdf
cp "$HERE/ha1_report.tex" "$BUILD/"
cd "$BUILD"
for pass in 1 2; do
    pdflatex -interaction=nonstopmode -halt-on-error ha1_report.tex > latex.log || { tail -30 latex.log; exit 1; }
done
cp ha1_report.pdf "$HERE/"
echo "built $HERE/ha1_report.pdf"
