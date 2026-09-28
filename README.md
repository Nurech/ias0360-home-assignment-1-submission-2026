# IAS0360 Home Assignment 1: IMU feature extraction on a Raspberry Pi Pico W
Joosep Parts, 256780IAXD. The Pico W reads the ICM-20948 motion sensor at 1000 Hz, runs the Lab 1_2
algorithms on 1 s of data and prints the results over USB; `host.py` saves them, checks the FFT
against NumPy and plots. `explained.txt` explains the code.

Compile (course container): `mkdir build && cd build && cmake .. && make` gives `build/ha1_imu.uf2`.

Run: `./run.sh` builds, flashes, then asks for three 1 s recordings (board still, shaken, gently
rocked), each after a 5 4 3 2 1 GO countdown. Keep the board still until the shake prompt.
By hand: flash the .uf2 (`picotool load build/ha1_imu.uf2 -f -x`), then `python3 host.py`.

Expected output: per recording the Pico prints text sections (`#begin name` ... `#end name`, then
`DONE`): info, raw, stats, quantization, quant_samples, zcr, orientation, downsampled, fft_input,
fft, fft_peaks, timing, fft_timing. Everything goes to `output/`, prefixed with the recording
(`still_`, `shake_`, `rock_`): `_run_log.txt`, the course `fft.py` inputs (`_raw_imu_mcu.txt`,
`_out_fft_mcu.txt`) and four plots, plus `overall_data.txt` with every number the report uses.
`python3 host.py --analyse` redoes the analysis from the saved logs.

Report: `cd report && ./build.sh` draws the figure from `output/shake_run_log.txt` and builds
`ha1_report.pdf` (needs LaTeX). The numbers in its text come from `output/overall_data.txt`.
