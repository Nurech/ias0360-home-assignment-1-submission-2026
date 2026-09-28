#!/usr/bin/env bash
# ha1 in one go: build, flash, wait for the pico's serial port, record and analyse.
#   ./run.sh            all steps
#   ./run.sh build      only build build/ha1_imu.uf2
#   ./run.sh flash      only flash it
#   ./run.sh capture    the board already runs this firmware: record and analyse
# capture asks for three recordings in turn (board still, shaken, gently rocked), each after a
# 5 s countdown, and puts the logs, plots and overall_data.txt in output/.
# report/build.sh then turns them into the report.
set -eu    # stop at the first failing command or unset variable
HERE="$(cd "$(dirname "$0")" && pwd)"
UF2="$HERE/build/ha1_imu.uf2"

build() {
    if [ -n "${PICO_SDK_PATH:-}" ]; then
        # the sdk is here (the course container): the plain build. cmake still prints the sdk's
        # status lines, make only warnings and errors
        rm -rf "$HERE/build"; mkdir "$HERE/build"; cd "$HERE/build"
        cmake .. > /dev/null
        make -j4 > /dev/null
        echo "built $UF2"
    else
        # no sdk here: the same build inside the course docker image, folder mounted at the same
        # path. PICO_SDK_PATH is set in the image, so run.sh takes the first branch in there
        if [ "$(uname -s)" = Darwin ]; then export DOCKER_CONTEXT="${DOCKER_CONTEXT:-desktop-linux}"; fi
        docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
            -v "$HERE:$HERE" -w "$HERE" ias0360-2026 -c "./run.sh build"
    fi
}

find_port() {
    for p in /dev/cu.usbmodem* /dev/ttyACM* /dev/pico; do
        if [ -e "$p" ]; then echo "$p"; return 0; fi
    done
    return 1
}

wait_for_port() {
    for i in $(seq 1 40); do
        if PORT="$(find_port)"; then return 0; fi
        sleep 0.5
    done
    echo "no pico serial port after 20 s. is the pico's own micro-usb plugged in?" >&2
    exit 1
}

flash() {
    if command -v picotool > /dev/null; then
        # -f reboots the running board into bootsel by itself, -x starts the new program afterwards
        picotool load "$UF2" -f -x
    else
        if [ "$(uname -s)" = Darwin ]; then DRIVE=/Volumes/RPI-RP2; else DRIVE="/media/$USER/RPI-RP2"; fi
        if [ ! -d "$DRIVE" ]; then
            # no picotool: opening the serial port at 1200 baud makes the pico sdk reboot into
            # bootsel. the port vanishes while pyserial still holds it, so its complaint is ignored
            wait_for_port
            python3 -c "import serial, sys; serial.Serial(sys.argv[1], 1200)" "$PORT" 2> /dev/null || true
        fi
        for i in $(seq 1 40); do
            if [ -d "$DRIVE" ]; then break; fi
            sleep 0.5
        done
        if [ ! -d "$DRIVE" ]; then
            echo "no $DRIVE drive. python3 needs pyserial, or hold BOOTSEL while plugging the board in" >&2
            exit 1
        fi
        cp "$UF2" "$DRIVE/"    # copying the uf2 onto the drive flashes it, then the pico restarts
    fi
    sleep 2    # give the board a moment to restart before looking for its serial port
}

capture() {
    # host.py counts down and says what to do before each recording. the gyro bias is measured
    # when it first opens the port after flashing or plugging in, just before the still countdown
    wait_for_port
    python3 "$HERE/host.py" --port "$PORT"
    # the vm's python has pyserial but no numpy, so host.py only recorded. the course docker image
    # has numpy and matplotlib, so the analysis of the saved logs runs in there (set -e: a log with
    # lost lines stops run.sh instead of passing quietly)
    if ! python3 -c "import numpy, matplotlib" 2> /dev/null && command -v docker > /dev/null; then
        if [ "$(uname -s)" = Darwin ]; then export DOCKER_CONTEXT="${DOCKER_CONTEXT:-desktop-linux}"; fi
        echo "analysing in the ias0360-2026 docker image"
        docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash -e MPLCONFIGDIR=/tmp/mpl \
            -v "$HERE:$HERE" -w "$HERE" ias0360-2026 \
            -c "python3 host.py --analyse"
    fi
}

case "${1:-all}" in
    all)     build; flash; capture ;;
    build)   build ;;
    flash)   flash ;;
    capture) capture ;;
    *)       echo "usage: ./run.sh [all|build|flash|capture]" >&2; exit 1 ;;
esac
