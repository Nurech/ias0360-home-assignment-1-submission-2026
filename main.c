// ias0360 home assignment 1, imu track. joosep parts, 256780IAXD
//
// pico w on the pico-eval-board, icm-20948 imu on i2c1. send 's' over usb serial and the pico
// records 1024 samples at 1000 hz, runs the lab 1_2 algorithms on them and prints each result as
//     #begin name / csv header and rows (or key=value lines) / #end name
// with DONE at the very end. host.py on the pc sends the 's', saves the output and plots it.
// the math follows the course files lab_1_2/statistic.c, quantization.c and fft.c.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define SAMPLE_RATE_HZ    1000     // lab 1_1 asks for at least 500 hz
#define N_SAMPLES         1024     // one window, about 1 s. a power of two for the radix-2 fft
#define N_AXES            6        // ax ay az gx gy gz
#define N_HALF            (N_SAMPLES / 2 + 1)   // one sided spectrum: bins 0..N/2
#define N_PEAKS           5        // the course fft.c reports the top 5
#define DOWNSAMPLE_FACTOR 4        // 1000 hz -> 250 hz
#define N_DOWN            (N_SAMPLES / DOWNSAMPLE_FACTOR)
#define QUANT_RANGE       0.875f   // see run_quantization()
#define N_DEPTHS          3        // quantization at 16, 8 and 4 bit
#define N_FFT_SIZES       4        // fft timing for n = 128, 256, 512, 1024
#define N_STEPS           10       // one per step_done() call in run_once()
#define CF_TAU_S          0.5f     // complementary filter trusts the gyro for about this long
#define PI_F              3.14159265f   // own pi, newlib does not always define M_PI
#define RAD_TO_DEG        (180.0f / PI_F)

// imu wiring, the same as the course driver. address 0x68 because the ad0 pin is low
#define IMU_I2C     i2c1
#define IMU_SDA_PIN 6
#define IMU_SCL_PIN 7
#define IMU_ADDR    0x68

// icm-20948 registers. the chip has 4 banks, REG_BANK_SEL picks which bank the addresses mean
#define REG_BANK_SEL           0x7F   // in every bank
#define REG_WHO_AM_I           0x00   // bank 0, an icm-20948 answers 0xEA
#define REG_PWR_MGMT_1         0x06   // bank 0
#define REG_PWR_MGMT_2         0x07   // bank 0
#define REG_ACCEL_XOUT_H       0x2D   // bank 0, first of 12 data bytes: accel xyz, gyro xyz
#define REG_GYRO_SMPLRT_DIV    0x00   // bank 2
#define REG_GYRO_CONFIG_1      0x01   // bank 2
#define REG_ACCEL_SMPLRT_DIV_1 0x10   // bank 2
#define REG_ACCEL_SMPLRT_DIV_2 0x11   // bank 2
#define REG_ACCEL_CONFIG       0x14   // bank 2

// datasheet: at +-2 g one g is 16384 counts, at +-1000 deg/s one deg/s is 32.8 counts
#define ACCEL_COUNTS_PER_G  16384.0f
#define GYRO_COUNTS_PER_DPS 32.8f

// the big arrays are static globals, never locals: the sdk gives core 0 a 2 kb stack and one
// float[1024] is already 4 kb. a stack overflow gives no error, it just overwrites other memory.
static const char *AXIS_NAME[N_AXES] = { "ax", "ay", "az", "gx", "gy", "gz" };
static int16_t g_counts[N_AXES][N_SAMPLES];     // raw sensor counts
static float   g_data[N_AXES][N_SAMPLES];       // the same in g and deg/s
static float   g_centered[N_AXES][N_SAMPLES];   // window mean removed
static float   g_minmax[N_AXES][N_SAMPLES];     // min-max normalised to [-1, 1]
static float   g_zscore[N_AXES][N_SAMPLES];     // z-score normalised
static float   g_sort_copy[N_SAMPLES];          // the median sorts this copy, the data keeps its order

static float    g_gyro_bias[3];                 // deg/s, measured once at start-up
static unsigned g_late_samples, g_read_us_total, g_read_us_max;
static float    g_achieved_hz;

typedef struct { float min, max, mean, median, mode; int mode_count; float variance, std; } axis_stats_t;
static axis_stats_t g_stats_raw[N_AXES];        // on the data in g and deg/s
static axis_stats_t g_stats_z[N_AXES];          // on the z-score data

static const int QUANT_BITS[N_DEPTHS] = { 16, 8, 4 };
typedef struct { float snr_db, rmse, max_err, signal_rms; int clips; } quant_result_t;
static quant_result_t g_quant[N_AXES][N_DEPTHS];
static float   g_quant_in[N_SAMPLES];           // one axis scaled to +-0.875, float32
static int16_t g_codes_16[N_SAMPLES];           // 16 bit codes
static int8_t  g_codes_8[N_SAMPLES];            // 8 and 4 bit codes, c has no 4 bit type
static float   g_dequant[N_SAMPLES];            // the codes turned back into floats
static float   g_ax_dequant[N_DEPTHS][N_SAMPLES];   // ax after 16, 8 and 4 bit, for quant_samples

static int   g_crossings[N_AXES];
static float g_pitch_acc[N_SAMPLES], g_roll_acc[N_SAMPLES];   // degrees, accelerometer only
static float g_pitch_cf[N_SAMPLES], g_roll_cf[N_SAMPLES];     // degrees, complementary filter
static float g_yaw[N_SAMPLES];                                // degrees, gyro z integrated
static float g_down[N_AXES][N_DOWN];            // the window at 250 hz

typedef struct { float re, im; } complex_t;     // same as c32 in the course fft.c
static complex_t g_fft_buf[N_SAMPLES];
static float g_amp[3][N_HALF];                  // amplitude spectrum of ax, ay, az
static int   g_peak_bin[3][N_PEAKS];
static float g_peak_amp[3][N_PEAKS];
static float g_timing_amp[N_HALF];              // scratch output for the fft timing runs
static const int FFT_TIMING_N[N_FFT_SIZES] = { 128, 256, 512, 1024 };
static unsigned g_fft_us[N_FFT_SIZES];

static const char *g_step_name[N_STEPS];        // how long each processing step took
static unsigned    g_step_us[N_STEPS];
static int         g_step_count;

// ---- imu, register level ----

static void imu_write(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };    // first byte picks the register, second is the value
    i2c_write_blocking(IMU_I2C, IMU_ADDR, buf, 2, false);
}

static bool imu_read(uint8_t reg, uint8_t *out, int count) {
    // true keeps the bus (repeated start), so the read follows the address right away
    if (i2c_write_blocking(IMU_I2C, IMU_ADDR, &reg, 1, true) != 1) return false;
    // the chip steps to the next register by itself, so one read gets them all in a row
    return i2c_read_blocking(IMU_I2C, IMU_ADDR, out, count, false) == count;
}

static bool imu_init(void) {
    i2c_init(IMU_I2C, 400 * 1000);      // 400 khz, the fastest the icm-20948 i2c goes
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    // bank 0 first: after a pico reset in the middle of an init the imu can still be in bank 2
    imu_write(REG_BANK_SEL, 0x00);
    uint8_t who = 0;
    imu_read(REG_WHO_AM_I, &who, 1);
    if (who != 0xEA) {
        printf("error: no icm-20948 on i2c1 gp6/gp7 (who_am_i=0x%02x, expected 0xea)\n", who);
        return false;
    }
    imu_write(REG_PWR_MGMT_1, 0x80);    // reset, every register back to its default
    sleep_ms(50);
    imu_write(REG_PWR_MGMT_1, 0x01);    // wake up (sleep bit off), auto clock. asleep it reads 0
    imu_write(REG_PWR_MGMT_2, 0x00);    // all accel and gyro axes on

    imu_write(REG_BANK_SEL, 0x20);      // bank 2, the sensor settings live there
    // accel rate = 1125 / (1 + divider), gyro rate = 1100 / (1 + divider). 0 gives 1125 and
    // 1100 hz, both a bit faster than i sample, so every sample is a new value (the course
    // divider 7 gives about 140 hz, so at 1000 hz each value would repeat about 7 times)
    imu_write(REG_GYRO_SMPLRT_DIV, 0);
    imu_write(REG_ACCEL_SMPLRT_DIV_1, 0);
    imu_write(REG_ACCEL_SMPLRT_DIV_2, 0);
    // bits [5:3] low-pass setting, [2:1] range, [0] low-pass on. setting 3 is about 50 hz: hand
    // motion is below 20 hz, and it is my anti-alias filter for the 250 hz downsampling
    imu_write(REG_GYRO_CONFIG_1, 0x1D); // (3 << 3) | (2 << 1) | 1: ~51 hz low-pass, +-1000 deg/s
    imu_write(REG_ACCEL_CONFIG, 0x19);  // (3 << 3) | (0 << 1) | 1: ~50 hz low-pass, +-2 g
    imu_write(REG_BANK_SEL, 0x00);      // back to bank 0, the data registers are there
    sleep_ms(100);                      // the gyro needs a moment to start
    return true;
}

// one 12 byte burst: ax ay az gx gy gz, high byte first, all six from the same moment.
// out is only changed when the read worked, so a failed read leaves the previous values in it
static void imu_read_sample(int16_t out[N_AXES]) {
    uint8_t b[12];
    if (!imu_read(REG_ACCEL_XOUT_H, b, 12)) return;
    for (int axis = 0; axis < N_AXES; axis++) {
        out[axis] = (int16_t)((b[2 * axis] << 8) | b[2 * axis + 1]);   // the cast makes 0xFFxx negative
    }
}

// a still gyro should read 0 deg/s, what it reads instead is its bias. average 250 readings
// at start (a 0.4 ms read + 1 ms sleep each, about a third of a second), the board must lie still
static void measure_gyro_bias(void) {
    int16_t s[N_AXES] = { 0 };
    double sum[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < 250; i++) {
        imu_read_sample(s);             // a failed read keeps the previous values
        for (int k = 0; k < 3; k++) sum[k] += (double)s[3 + k] / GYRO_COUNTS_PER_DPS;
        sleep_ms(1);
    }
    for (int k = 0; k < 3; k++) g_gyro_bias[k] = (float)(sum[k] / 250);
}

// ---- sampling ----

// the loop only reads and stores, anything slow in here (printf, float math) would make samples late
static void record_window(void) {
    const uint64_t period_us = 1000000 / SAMPLE_RATE_HZ;
    int16_t sample[N_AXES] = { 0 };
    g_late_samples = 0;
    g_read_us_total = 0;
    g_read_us_max = 0;
    // absolute schedule: sample i is due at start + i * period. sleeping one period after each
    // read would add the read time to every gap and the rate would drift
    uint64_t next_tick = time_us_64() + period_us;    // first sample one period from now
    uint64_t first_us = 0, last_us = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
        while (time_us_64() < next_tick) {
            // busy wait until the tick, the simplest exact wait
        }
        uint64_t now = time_us_64();
        if (i == 0) first_us = now;
        last_us = now;
        imu_read_sample(sample);        // if the read fails, sample keeps the previous row
        unsigned read_us = (unsigned)(time_us_64() - now);
        g_read_us_total += read_us;
        if (read_us > g_read_us_max) g_read_us_max = read_us;
        for (int axis = 0; axis < N_AXES; axis++) g_counts[axis][i] = sample[axis];
        next_tick += period_us;
        if (time_us_64() > next_tick) g_late_samples++;   // this sample took longer than a period
    }
    // n samples have n - 1 gaps between them
    float seconds = (float)(last_us - first_us) / 1e6f;
    g_achieved_hz = (float)(N_SAMPLES - 1) / seconds;
}

static void convert_to_units(void) {
    for (int i = 0; i < N_SAMPLES; i++) {
        for (int axis = 0; axis < 3; axis++) g_data[axis][i] = (float)g_counts[axis][i] / ACCEL_COUNTS_PER_G;
        for (int axis = 3; axis < N_AXES; axis++) g_data[axis][i] = (float)g_counts[axis][i] / GYRO_COUNTS_PER_DPS;
    }
}

// ---- statistics (course statistic.c) ----

// the sum is a double: a float sum of 1024 values starts to lose the small ones
static float mean_f32(const float *x, int n) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += x[i];
    return (float)(sum / n);
}

// sample variance, divided by n - 1 like the course (numpy needs ddof=1 to match)
static float variance_f32(const float *x, int n, float mean) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        double d = (double)x[i] - (double)mean;
        sum += d * d;
    }
    return (float)(sum / (n - 1));
}

static void min_max_f32(const float *x, int n, float *min_out, float *max_out) {
    float lo = x[0], hi = x[0];
    for (int i = 1; i < n; i++) {
        if (x[i] < lo) lo = x[i];
        if (x[i] > hi) hi = x[i];
    }
    *min_out = lo;
    *max_out = hi;
}

// insertion sort of a copy, then the middle value. slow (about n*n/4 moves) but simple
static float median_f32(const float *x, int n) {
    for (int i = 0; i < n; i++) g_sort_copy[i] = x[i];
    for (int i = 1; i < n; i++) {
        float key = g_sort_copy[i];
        int j = i - 1;
        while (j >= 0 && g_sort_copy[j] > key) {   // shift the bigger ones one place right
            g_sort_copy[j + 1] = g_sort_copy[j];
            j--;
        }
        g_sort_copy[j + 1] = key;
    }
    if (n % 2 == 1) return g_sort_copy[n / 2];
    return 0.5f * (g_sort_copy[n / 2 - 1] + g_sort_copy[n / 2]);   // even n: mean of the middle two
}

// the course mode with eps = 0: count how often each value comes again, keep the most common.
// exact == only works because the imu gives whole counts, so equal counts give the same float
static float mode_f32(const float *x, int n, int *count_out) {
    int best_count = 0;
    float best_value = x[0];
    for (int i = 0; i < n; i++) {
        int count = 1;
        for (int j = i + 1; j < n; j++) if (x[j] == x[i]) count++;
        if (count > best_count) {       // strictly bigger, so the first value wins a tie
            best_count = count;
            best_value = x[i];
        }
    }
    *count_out = best_count;
    return best_value;
}

static void compute_axis_stats(const float *x, axis_stats_t *s) {
    min_max_f32(x, N_SAMPLES, &s->min, &s->max);
    s->mean = mean_f32(x, N_SAMPLES);
    s->median = median_f32(x, N_SAMPLES);
    s->mode = mode_f32(x, N_SAMPLES, &s->mode_count);
    s->variance = variance_f32(x, N_SAMPLES, s->mean);
    s->std = sqrtf(s->variance);
}

// ---- dc offset and normalisation ----

// subtract the window mean: removes gravity and bias, the fft and zero crossings need a signal around 0
static void remove_dc_offset(void) {
    for (int axis = 0; axis < N_AXES; axis++) {
        float mean = mean_f32(g_data[axis], N_SAMPLES);
        for (int i = 0; i < N_SAMPLES; i++) g_centered[axis][i] = g_data[axis][i] - mean;
    }
}

// smallest value becomes -1, biggest +1
static void normalise_min_max(const float *x, float *out) {
    float lo, hi;
    min_max_f32(x, N_SAMPLES, &lo, &hi);
    float range = hi - lo;
    for (int i = 0; i < N_SAMPLES; i++) {
        if (range > 0.0f) out[i] = 2.0f * (x[i] - lo) / range - 1.0f;
        else out[i] = 0.0f;             // a flat axis
    }
}

// how many standard deviations each value is from the mean
static void normalise_z_score(const float *x, float *out) {
    float mean = mean_f32(x, N_SAMPLES);
    float std = sqrtf(variance_f32(x, N_SAMPLES, mean));
    for (int i = 0; i < N_SAMPLES; i++) {
        if (std > 0.0f) out[i] = (x[i] - mean) / std;
        else out[i] = 0.0f;
    }
}

// ---- quantization (course quantization.c) ----

// the course quantize_q15 for any bit depth: scale 2^(bits-1), so 16 bit is exactly q15
static int quantize_value(float x, int bits, bool *clipped) {
    const float scale = (float)(1 << (bits - 1));   // 32768, 128 or 8
    const int q_max = (1 << (bits - 1)) - 1;         // 32767, 127 or 7
    const int q_min = -(1 << (bits - 1));            // -32768, -128 or -8
    const float limit = (float)q_max / scale;        // the biggest value that fits
    // first limit the input, that is what counts as a clip, then scale and round
    float s = x;
    if (s > limit) s = limit;           // +1.0 would become 32768, too big for an int16
    if (s < -limit) s = -limit;         // the course clamps both sides the same, so -1.0 is a clip too
    *clipped = (s != x);
    int q = (int)lrintf(s * scale);     // round to the nearest whole number, like the course
    // kept from the course quantize_q15, it never triggers after the clamp above
    if (q > q_max) q = q_max;
    if (q < q_min) q = q_min;
    return q;
}

// course snr_and_error: signal power over error power, plus rms and max of the error
static void snr_and_error(const float *ref, const float *test, int n,
                          float *snr_db, float *max_abs_err, float *rms_err) {
    double signal_power = 0.0, error_power = 0.0;
    float max_err = 0.0f;
    for (int i = 0; i < n; i++) {
        float e = ref[i] - test[i];
        signal_power += (double)ref[i] * (double)ref[i];
        error_power += (double)e * (double)e;
        if (fabsf(e) > max_err) max_err = fabsf(e);
    }
    *rms_err = (float)sqrt(error_power / n);
    *max_abs_err = max_err;
    if (error_power == 0.0) *snr_db = INFINITY;     // lossless
    else *snr_db = 10.0f * log10f((float)(signal_power / error_power));
}

static void run_quantization(void) {
    for (int axis = 0; axis < N_AXES; axis++) {
        // input: the min-max axis times 0.875. a 4 bit signed number only reaches +7/8, so this
        // way no depth clips and the error can be compared with the theory (it assumes no clipping)
        double power = 0.0;
        for (int i = 0; i < N_SAMPLES; i++) {
            g_quant_in[i] = g_minmax[axis][i] * QUANT_RANGE;
            power += (double)g_quant_in[i] * (double)g_quant_in[i];
        }
        float signal_rms = (float)sqrt(power / N_SAMPLES);

        for (int b = 0; b < N_DEPTHS; b++) {
            int bits = QUANT_BITS[b];
            quant_result_t *r = &g_quant[axis][b];
            r->signal_rms = signal_rms;
            r->clips = 0;
            // each code goes into the smallest c type it fits in, that is the memory saving
            for (int i = 0; i < N_SAMPLES; i++) {
                bool clipped;
                int code = quantize_value(g_quant_in[i], bits, &clipped);
                if (clipped) r->clips++;
                if (bits == 16) g_codes_16[i] = (int16_t)code;
                else g_codes_8[i] = (int8_t)code;         // -128..127 and -8..7 both fit
            }
            // back to floats in a separate array, the input is still needed for the error
            for (int i = 0; i < N_SAMPLES; i++) {
                int code = (bits == 16) ? g_codes_16[i] : g_codes_8[i];
                g_dequant[i] = (float)code / (float)(1 << (bits - 1));
            }
            snr_and_error(g_quant_in, g_dequant, N_SAMPLES, &r->snr_db, &r->max_err, &r->rmse);
            if (axis == 0) {            // keep ax so the pc can plot original vs quantized
                for (int i = 0; i < N_SAMPLES; i++) g_ax_dequant[b][i] = g_dequant[i];
            }
        }
    }
}

// ---- zero crossings, orientation, downsampling ----

// sign changes of the dc free signal. a sine crosses zero twice per period
static int zero_crossings(const float *x, int n) {
    int count = 0;
    for (int i = 1; i < n; i++) {
        bool was_negative = x[i - 1] < 0.0f;
        bool is_negative = x[i] < 0.0f;
        if (was_negative != is_negative) count++;
    }
    return count;
}

// accel: when still it only sees gravity, the tilt is the direction of that. gyro: integrate the
// rate, smooth but a bias adds up. the complementary filter follows the gyro short term and pulls
// slowly towards the accel angle. yaw has no accel reference (turning flat does not move gravity).
static void compute_orientation(void) {
    const float dt = 1.0f / SAMPLE_RATE_HZ;
    // alpha = tau / (tau + dt) = 0.998 at 1000 hz (the 0.98 often seen online is for ~100 hz)
    const float alpha = CF_TAU_S / (CF_TAU_S + dt);
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    for (int i = 0; i < N_SAMPLES; i++) {
        float ax = g_data[0][i];
        float ay = g_data[1][i];
        float az = g_data[2][i];
        // start-up bias removed, not the window mean: that would also remove a real turn
        float gx = g_data[3][i] - g_gyro_bias[0];
        float gy = g_data[4][i] - g_gyro_bias[1];
        float gz = g_data[5][i] - g_gyro_bias[2];
        // +roll about x moves gravity from z into y: ay = sin(roll), az = cos(roll).
        // atan2(y, x) keeps the sign and quadrant and never divides by zero
        float roll_acc = atan2f(ay, az) * RAD_TO_DEG;
        // +pitch about y gives ax = -sin(pitch). the bottom is all the gravity in the y-z plane,
        // so pitch stays right when the board is also rolled
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
        if (i == 0) {
            roll = roll_acc;            // start from the accel angle, not from 0
            pitch = pitch_acc;
        } else {
            roll = alpha * (roll + gx * dt) + (1.0f - alpha) * roll_acc;
            pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitch_acc;
            yaw = yaw + gz * dt;
        }
        g_roll_acc[i] = roll_acc;
        g_pitch_acc[i] = pitch_acc;
        g_roll_cf[i] = roll;
        g_pitch_cf[i] = pitch;
        g_yaw[i] = yaw;
    }
}

// average every 4 samples into one: 1000 hz -> 250 hz. averaging is a weak low-pass, the imu's
// own ~50 hz low-pass is what keeps anything above 125 hz from folding back (aliasing)
static void downsample(void) {
    for (int axis = 0; axis < N_AXES; axis++) {
        for (int j = 0; j < N_DOWN; j++) {
            float sum = 0.0f;
            for (int k = 0; k < DOWNSAMPLE_FACTOR; k++) sum += g_data[axis][j * DOWNSAMPLE_FACTOR + k];
            g_down[axis][j] = sum / DOWNSAMPLE_FACTOR;
        }
    }
}

// ---- fft (course fft.c) ----

// hamming window: tapers both ends of the window so its sharp edges do not smear the spectrum
static float hamming(int i, int n) {
    return 0.54f - 0.46f * cosf(2.0f * PI_F * (float)i / (float)(n - 1));
}

// mirror the lowest bit_count bits: 0011 -> 1100 for 4 bits
static unsigned reverse_bits(unsigned value, int bit_count) {
    unsigned result = 0;
    for (int b = 0; b < bit_count; b++) {
        result = (result << 1) | (value & 1u);   // move the lowest bit of value into result
        value >>= 1;
    }
    return result;
}

// in place radix-2 fft, the same steps as the course fft_radix2 (forward only)
static void fft_radix2(complex_t *x, int n) {
    int stages = 0;                     // how often n halves down to 1: 10 for 1024
    while ((1 << stages) < n) stages++;
    // move every value to its bit-reversed index, then the butterflies can work in place.
    // only swap when j > i, or every pair would be swapped back again
    for (int i = 0; i < n; i++) {
        int j = (int)reverse_bits((unsigned)i, stages);
        if (j > i) {
            complex_t tmp = x[i];
            x[i] = x[j];
            x[j] = tmp;
        }
    }
    // stage 1 makes 2 point dfts, stage 2 merges them into 4 point ones, and so on up to n
    for (int s = 1; s <= stages; s++) {
        int half = (1 << s) / 2;        // a block in stage s is 2^s values, two halves of half each
        // the twiddle w starts at 1 and turns by theta = -pi/half (-2*pi / block length) each step.
        // like the course it turns by a recurrence instead of cosf/sinf per butterfly:
        // w = w + w * (wpr + i*wpi), where wpr = cos(theta) - 1 is written as -2 sin^2(theta/2),
        // which stays accurate for small theta
        float theta = -PI_F / (float)half;
        float wpr = -2.0f * sinf(0.5f * theta) * sinf(0.5f * theta);
        float wpi = sinf(theta);
        for (int start = 0; start < n; start += 2 * half) {
            float wr = 1.0f, wi = 0.0f;
            for (int j = 0; j < half; j++) {
                int top = start + j;
                int bottom = start + j + half;
                float tr = wr * x[bottom].re - wi * x[bottom].im;   // t = w * bottom
                float ti = wr * x[bottom].im + wi * x[bottom].re;
                x[bottom].re = x[top].re - tr;                      // bottom = top - t
                x[bottom].im = x[top].im - ti;
                x[top].re = x[top].re + tr;                         // top = top + t
                x[top].im = x[top].im + ti;
                float old_wr = wr;                                  // turn w one step further
                wr = wr + (wr * wpr - wi * wpi);
                wi = wi + (wi * wpr + old_wr * wpi);
            }
        }
    }
}

// window + fft + one sided amplitude of the first n values of x
static void fft_amplitude(const float *x, int n, float *amp) {
    // the window is applied on the way into the complex buffer, x itself stays as it was
    for (int i = 0; i < n; i++) {
        g_fft_buf[i].re = x[i] * hamming(i, n);
        g_fft_buf[i].im = 0.0f;
    }
    fft_radix2(g_fft_buf, n);
    // 2/n: the dft 1/n, times 2 for the negative frequencies folded in. 1/0.54 undoes the average
    // height of the hamming window. then a tone of amplitude a reads about a
    const float scale = (2.0f / (float)n) / 0.54f;
    for (int k = 0; k <= n / 2; k++) {
        amp[k] = sqrtf(g_fft_buf[k].re * g_fft_buf[k].re + g_fft_buf[k].im * g_fft_buf[k].im) * scale;
    }
}

// course top_k_peaks: N_PEAKS passes, each takes the biggest bin not taken yet, bin 0 (dc) skipped.
// these are the biggest bins, not real peaks, so the neighbours of one tone count too
static void top_peaks(const float *amp, int n_bins, int *peak_bin, float *peak_amp) {
    for (int k = 0; k < N_PEAKS; k++) {
        int best_bin = -1;
        float best_amp = -1.0f;
        for (int bin = 1; bin < n_bins; bin++) {
            bool taken = false;
            for (int j = 0; j < k; j++) if (peak_bin[j] == bin) taken = true;
            if (!taken && amp[bin] > best_amp) {
                best_amp = amp[bin];
                best_bin = bin;
            }
        }
        peak_bin[k] = best_bin;
        peak_amp[k] = best_amp;
    }
}

// time window + fft + amplitude + peaks of one axis for n = 128 .. 1024
static void measure_fft_timing(void) {
    int bins[N_PEAKS];
    float amps[N_PEAKS];
    for (int t = 0; t < N_FFT_SIZES; t++) {
        int n = FFT_TIMING_N[t];
        uint64_t start = time_us_64();
        fft_amplitude(g_centered[0], n, g_timing_amp);
        top_peaks(g_timing_amp, n / 2 + 1, bins, amps);
        g_fft_us[t] = (unsigned)(time_us_64() - start);
    }
}

// ---- output ----

static void begin_section(const char *name, const char *csv_header) {
    printf("#begin %s\n%s\n", name, csv_header);
}

static void print_stats_row(int axis, const char *data, const axis_stats_t *s) {
    printf("%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f\n", AXIS_NAME[axis], data,
           s->min, s->max, s->mean, s->median, s->mode, s->mode_count, s->variance, s->std);
}

static void print_quantization(void) {
    begin_section("quantization",
                  "axis,bits,snr_db,rmse,max_abs_err,clips,delta,theory_rmse,theory_max_err,theory_snr_db");
    for (int axis = 0; axis < N_AXES; axis++) {
        for (int b = 0; b < N_DEPTHS; b++) {
            const quant_result_t *r = &g_quant[axis][b];
            // theory (lab 1_2, section 1.3) for rounding without clipping: step delta = 2 / 2^bits,
            // the error is spread evenly over +-delta/2, so its rms is delta / sqrt(12), the largest
            // of n errors is about n/(n+1) * delta/2, and snr = 20 log10(rms signal / rms error)
            float delta = 2.0f / (float)(1 << QUANT_BITS[b]);
            float theory_rmse = delta / sqrtf(12.0f);
            float theory_max = ((float)N_SAMPLES / (N_SAMPLES + 1)) * delta / 2.0f;
            float theory_snr = 20.0f * log10f(r->signal_rms / theory_rmse);
            printf("%s,%d,%.2f,%.3e,%.3e,%d,%.3e,%.3e,%.3e,%.2f\n", AXIS_NAME[axis], QUANT_BITS[b],
                   r->snr_db, r->rmse, r->max_err, r->clips, delta, theory_rmse, theory_max, theory_snr);
        }
    }
    printf("#end quantization\n");
    // ax_in is the quantizer input, the same value as in run_quantization() (min-max ax * 0.875)
    begin_section("quant_samples", "i,ax_in,ax_q16,ax_q8,ax_q4");
    for (int i = 0; i < N_SAMPLES; i++) {
        printf("%d,%.7f,%.7f,%.7f,%.7f\n", i, g_minmax[0][i] * QUANT_RANGE,
               g_ax_dequant[0][i], g_ax_dequant[1][i], g_ax_dequant[2][i]);
    }
    printf("#end quant_samples\n");
}

static void print_fft(void) {
    // exactly what went into the fft: the accel with its mean removed, before the window
    begin_section("fft_input", "i,ax,ay,az");
    for (int i = 0; i < N_SAMPLES; i++) {
        printf("%d,%.6f,%.6f,%.6f\n", i, g_centered[0][i], g_centered[1][i], g_centered[2][i]);
    }
    printf("#end fft_input\n");
    // 7 decimals: 1000/1024 = 0.9765625 needs all 7, and the course fft.py compares with ==
    begin_section("fft", "bin,freq_hz,ax,ay,az");
    for (int k = 0; k < N_HALF; k++) {
        float freq = (float)k * SAMPLE_RATE_HZ / N_SAMPLES;
        printf("%d,%.7f,%.7f,%.7f,%.7f\n", k, freq, g_amp[0][k], g_amp[1][k], g_amp[2][k]);
    }
    printf("#end fft\n");
    begin_section("fft_peaks", "axis,rank,bin,freq_hz,amp");
    for (int axis = 0; axis < 3; axis++) {
        for (int p = 0; p < N_PEAKS; p++) {
            float freq = (float)g_peak_bin[axis][p] * SAMPLE_RATE_HZ / N_SAMPLES;
            printf("%s,%d,%d,%.7f,%.7f\n", AXIS_NAME[axis], p + 1, g_peak_bin[axis][p], freq,
                   g_peak_amp[axis][p]);
        }
    }
    printf("#end fft_peaks\n");
}

static void print_timing(void) {
    begin_section("timing", "step,us");
    for (int s = 0; s < g_step_count; s++) printf("%s,%u\n", g_step_name[s], g_step_us[s]);
    printf("#end timing\n");
    // can the fft run on the same core as the sampling without losing samples?
    // blocking: no second buffer, the fft has to fit between two samples: period >= read + fft.
    // buffered: a timer interrupt fills a second buffer while the fft works on the last one,
    // so n reads and one fft must fit in n periods: period >= read + fft / n.
    float read_us = (float)g_read_us_total / N_SAMPLES;
    begin_section("fft_timing", "n,fft_us,read_us,fs_max_blocking_hz,fs_max_buffered_hz");
    for (int t = 0; t < N_FFT_SIZES; t++) {
        int n = FFT_TIMING_N[t];
        float fft_us = (float)g_fft_us[t];
        float blocking = 1e6f / (read_us + fft_us);
        float buffered = 1e6f / (read_us + fft_us / n);
        printf("%d,%u,%.1f,%.1f,%.1f\n", n, g_fft_us[t], read_us, blocking, buffered);
    }
    printf("#end fft_timing\n");
}

static void print_results(void) {
    printf("#begin info\n");
    printf("fs_hz=%d\nn=%d\n", SAMPLE_RATE_HZ, N_SAMPLES);
    printf("achieved_hz=%.2f\nlate_samples=%u\n", g_achieved_hz, g_late_samples);
    printf("read_us_avg=%.1f\nread_us_max=%u\n", (float)g_read_us_total / N_SAMPLES, g_read_us_max);
    printf("gyro_bias_x_dps=%.4f\n", g_gyro_bias[0]);
    printf("gyro_bias_y_dps=%.4f\n", g_gyro_bias[1]);
    printf("gyro_bias_z_dps=%.4f\n", g_gyro_bias[2]);
    printf("#end info\n");

    begin_section("raw", "i,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
    for (int i = 0; i < N_SAMPLES; i++) {
        printf("%d,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n", i,
               g_data[0][i], g_data[1][i], g_data[2][i], g_data[3][i], g_data[4][i], g_data[5][i]);
    }
    printf("#end raw\n");

    begin_section("stats", "axis,data,min,max,mean,median,mode,mode_count,variance,std");
    for (int axis = 0; axis < N_AXES; axis++) {
        print_stats_row(axis, "raw", &g_stats_raw[axis]);
        print_stats_row(axis, "zscore", &g_stats_z[axis]);
    }
    printf("#end stats\n");

    print_quantization();

    // two crossings per period, so half the crossings per second is a rough frequency
    float seconds = (float)N_SAMPLES / SAMPLE_RATE_HZ;
    begin_section("zcr", "axis,crossings,per_second,rough_hz");
    for (int axis = 0; axis < N_AXES; axis++) {
        int c = g_crossings[axis];
        printf("%s,%d,%.1f,%.1f\n", AXIS_NAME[axis], c, c / seconds, c / seconds / 2.0f);
    }
    printf("#end zcr\n");

    begin_section("orientation", "i,pitch_acc,roll_acc,pitch_cf,roll_cf,yaw_gyro");
    for (int i = 0; i < N_SAMPLES; i++) {
        printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f\n", i,
               g_pitch_acc[i], g_roll_acc[i], g_pitch_cf[i], g_roll_cf[i], g_yaw[i]);
    }
    printf("#end orientation\n");

    begin_section("downsampled", "i,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
    for (int j = 0; j < N_DOWN; j++) {
        printf("%d,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n", j,
               g_down[0][j], g_down[1][j], g_down[2][j], g_down[3][j], g_down[4][j], g_down[5][j]);
    }
    printf("#end downsampled\n");

    print_fft();
    print_timing();
    printf("DONE\n");
}

// ---- one run: record, process, print ----

// store how long the step since *t0 took, then restart the clock for the next step
static void step_done(const char *name, uint64_t *t0) {
    g_step_name[g_step_count] = name;
    g_step_us[g_step_count] = (unsigned)(time_us_64() - *t0);
    g_step_count++;
    *t0 = time_us_64();
}

static void run_once(void) {
    printf("run: recording %d samples at %d hz, then about 5 s of processing\n", N_SAMPLES, SAMPLE_RATE_HZ);
    g_step_count = 0;
    uint64_t t0 = time_us_64();
    record_window();
    step_done("record", &t0);
    convert_to_units();
    step_done("convert", &t0);
    remove_dc_offset();
    step_done("dc_offset", &t0);
    for (int axis = 0; axis < N_AXES; axis++) {
        normalise_min_max(g_data[axis], g_minmax[axis]);
        normalise_z_score(g_data[axis], g_zscore[axis]);
    }
    step_done("normalise", &t0);
    for (int axis = 0; axis < N_AXES; axis++) {
        compute_axis_stats(g_data[axis], &g_stats_raw[axis]);
        compute_axis_stats(g_zscore[axis], &g_stats_z[axis]);
    }
    step_done("stats", &t0);
    run_quantization();
    step_done("quantization", &t0);
    for (int axis = 0; axis < N_AXES; axis++) g_crossings[axis] = zero_crossings(g_centered[axis], N_SAMPLES);
    step_done("zcr", &t0);
    compute_orientation();
    step_done("orientation", &t0);
    downsample();
    step_done("downsample", &t0);
    // the fft of the dc free accel axes. with gravity still in az, the window would spread it into
    // bin 1 (0.88 g at rest), so bin 1 would always win and hide real slow motion in that bin
    for (int axis = 0; axis < 3; axis++) {
        fft_amplitude(g_centered[axis], N_SAMPLES, g_amp[axis]);
        top_peaks(g_amp[axis], N_HALF, g_peak_bin[axis], g_peak_amp[axis]);
    }
    step_done("fft_3_axes", &t0);
    measure_fft_timing();
    print_results();
}

int main(void) {
    stdio_init_all();
    // anything printed before the pc opens the port is lost, so wait for it
    while (!stdio_usb_connected()) sleep_ms(10);
    printf("\nias0360 ha1: imu feature extraction (joosep parts, 256780IAXD)\n");
    if (!imu_init()) {
        while (true) sleep_ms(1000);    // nothing to measure without the imu, imu_init() said why
    }
    printf("imu: found, measuring the gyro bias, keep the board still\n");
    measure_gyro_bias();
    printf("imu: gyro bias %.3f %.3f %.3f dps\n", g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2]);
    printf("send s to record and process one window\n");
    while (true) {
        int c = getchar();              // waits for the next byte from the pc
        if (c == 's') run_once();       // every other byte is ignored
    }
}
