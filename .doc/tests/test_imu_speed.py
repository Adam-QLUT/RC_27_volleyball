"""Host numerical tests for actual IMU preprocessing and speed functions.

Run: python3 .doc/tests/test_imu_speed.py
Optional: HOST_CFLAGS='-isysroot /path/to/MacOSX.sdk'
The production functions and IMU structure are extracted unchanged; only their
hardware-independent inputs are supplied here. IMU_Updata is also extracted to
check wiring with read/AHRS/temperature stubs; this does not validate real AHRS
dynamics, hardware timing or sensor noise. Build the firmware separately.
"""

import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


root = Path(__file__).resolve().parents[2]
source = (root / "User/Hardware/IMU/IMU_updata.c").read_text()
header = (root / "User/Hardware/IMU/IMU_updata.h").read_text()
structure = re.search(r"struct IMU_t\s*\{.*?\n\};", header, re.S).group()
constants = "\n".join(line for line in header.splitlines() if line.startswith("#define IMU_"))
functions = source[source.index("uint8_t IMU_AccelUpdata(fp32 timing_time)\n{"):
                   source.index("float rad2degree(float a)")]
update = source[source.index("void IMU_Updata() // 1000HZ"):
                source.index("/**\n * @brief          零偏及安装位置补偿")]

fixture = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef float fp32;
typedef struct { float unused; } Attitude_3D_t;
""" + constants + "\n" + structure + r"""
#ifdef TEST_DEADZONE
#undef IMU_ACCEL_DEADZONE
#define IMU_ACCEL_DEADZONE 0.05f
#endif
#ifdef TEST_ZERO_POSITION
#undef IMU_POSITION_X
#define IMU_POSITION_X 0.0f
#endif
struct IMU_t IMU_data;
static fp32 accel_fliter_1[3], accel_fliter_2[3];
static fp32 accel_fliter_3[3];
""" + re.search(r"static const fp32 fliter_num\[3\] = .*?;", source).group() + r"""
static fp32 accel_earth_last[3];
static uint8_t speed_started;
static fp32 gravity_accel_mean[3];
static fp32 gravity_gyro_mean[3], gyro_last[3];
static uint8_t rotation_started, accel_filter_started;
void IMU_SpeedReset(void);
void IMU_GravityReset(void);
void IMU_GravityUpdata(void);
""" + functions + r"""
static void near(float actual, float expected)
{
    if (!isfinite(actual) || fabsf(actual - expected) > 2e-4f) {
        fprintf(stderr, "actual=%g expected=%g\n", actual, expected);
        assert(0);
    }
}

static void init(void)
{
    memset(&IMU_data, 0, sizeof(IMU_data));
    IMU_GravityReset();
    IMU_data.gravity = IMU_GRAVITY;
    IMU_data.gravity_ready = 1;
    IMU_data.AHRS.q[0] = 1.0f;
    IMU_data.AHRS.err_code = 1;
    accel_fliter_3[0] = accel_fliter_3[1] = 0;
    accel_fliter_3[2] = IMU_GRAVITY;
}

#define PI 3.14159265358979323846f
struct IMU_t IMU_data_history[10];
static float sample_gyro[3], sample_accel[3], ahrs_gyro[3], ahrs_accel[3];
static int ahrs_calls, ahrs_result = 1;
static void BMI088_read(float *gyro, float *accel, float *temp)
{
    memcpy(gyro, sample_gyro, sizeof(sample_gyro));
    memcpy(accel, sample_accel, sizeof(sample_accel));
    *temp = 25;
}
static void IMU_TempControl(float temp) { (void)temp; }
static int AHRS_update(float *q, float dt, float *gyro, float *accel, float *mag)
{
    (void)q; (void)dt; (void)mag;
    memcpy(ahrs_gyro, gyro, sizeof(ahrs_gyro));
    memcpy(ahrs_accel, accel, sizeof(ahrs_accel));
    ++ahrs_calls;
    return ahrs_result;
}
static void get_angle(float *q, float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]),
                 q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    *pitch = *roll = 0;
}
""" + update + r"""

int main(void)
{
    // Static gravity removal at combined pitch and roll, including non-unit q.
    for (int pose = 0; pose < 3; ++pose) {
        init();
        float roll = pose * 0.4f, pitch = pose * -0.3f;
        float cr = cosf(roll / 2), sr = sinf(roll / 2);
        float cp = cosf(pitch / 2), sp = sinf(pitch / 2);
        IMU_data.AHRS.q[0] = 2 * cr * cp;
        IMU_data.AHRS.q[1] = 2 * sr * cp;
        IMU_data.AHRS.q[2] = 2 * cr * sp;
        IMU_data.AHRS.q[3] = -2 * sr * sp;
        accel_fliter_3[0] = -IMU_GRAVITY * sinf(pitch);
        accel_fliter_3[1] = IMU_GRAVITY * cosf(pitch) * sinf(roll);
        accel_fliter_3[2] = IMU_GRAVITY * cosf(pitch) * cosf(roll);
        for (int i = 0; i < 1000; ++i) IMU_SpeedUpdata(0.001f);
        assert(IMU_data.speed_valid);
        for (int axis = 0; axis < 3; ++axis) near(IMU_data.speed[axis], 0);
    }

    // IMU's own right-handed axes: no extra X/Y sign flip.
    init();
    accel_fliter_3[0] = 1;
    accel_fliter_3[1] = 2;
    accel_fliter_3[2] += 3;
    for (int i = 0; i < 1000; ++i) IMU_SpeedUpdata(0.001f);
    for (int axis = 0; axis < 3; ++axis) {
        near(IMU_data.accel_linear[axis], axis + 1);
        near(IMU_data.speed[axis], axis + 1);
    }

    // Coasting velocity stays constant in the fixed frame during a 90 deg turn.
    init();
    IMU_data.speed_earth[0] = 1;
    for (int i = 0; i <= 1000; ++i) {
        float half_yaw = 0.7853981634f * i / 1000;
        IMU_data.AHRS.q[0] = cosf(half_yaw);
        IMU_data.AHRS.q[3] = sinf(half_yaw);
        IMU_SpeedUpdata(0.001f);
    }
    near(IMU_data.speed[0], 0);
    near(IMU_data.speed[1], -1);
    near(IMU_data.speed_earth[0], 1);

    // Variable valid dt and trapezoidal integration of a linear acceleration ramp.
    init();
    IMU_SpeedUpdata(0.001f);
    float time = 0;
    for (int i = 0; i < 100; ++i) {
        float dt = (i % 2) ? 0.006f : 0.014f;
        time += dt;
        accel_fliter_3[0] = 10 * time;
        IMU_SpeedUpdata(dt);
    }
    near(IMU_data.speed[0], 5);

    // Reject invalid dt, quaternion, acceleration and failed AHRS updates.
    const float invalid_dt[] = {0, -0.001f, NAN, INFINITY, 0.1f};
    for (unsigned i = 0; i < sizeof(invalid_dt) / sizeof(*invalid_dt); ++i) {
        IMU_SpeedUpdata(invalid_dt[i]);
        assert(!IMU_data.speed_valid);
        near(IMU_data.speed[0], 5);
    }
    IMU_data.AHRS.q[0] = 0;
    IMU_SpeedUpdata(0.001f);
    assert(!IMU_data.speed_valid);
    IMU_data.AHRS.q[0] = NAN;
    IMU_SpeedUpdata(0.001f);
    assert(!IMU_data.speed_valid);
    IMU_data.AHRS.q[0] = 1;
    accel_fliter_3[0] = NAN;
    IMU_SpeedUpdata(0.001f);
    assert(!IMU_data.speed_valid);
    accel_fliter_3[0] = 0;
    IMU_data.AHRS.err_code = 0;
    IMU_SpeedUpdata(0.001f);
    assert(!IMU_data.speed_valid);
    near(IMU_data.speed[0], 5);
    IMU_data.AHRS.err_code = 1;
    IMU_SpeedUpdata(0.001f);
    near(IMU_data.speed[0], 5); // no stale pre-gap acceleration contribution

    IMU_SpeedReset();
    assert(!IMU_data.speed_valid);
    for (int axis = 0; axis < 3; ++axis) {
        near(IMU_data.speed[axis], 0);
        near(IMU_data.speed_earth[axis], 0);
    }
    IMU_SpeedUpdata(0.001f);
    near(IMU_data.speed[0], 0);

#ifdef TEST_DEADZONE
    init();
    accel_fliter_3[0] = -0.02f;
    for (int i = 0; i < 1000; ++i) IMU_SpeedUpdata(0.001f);
    near(IMU_data.speed[0], 0);
#endif
    // Startup: no velocity integration until 3000 consecutive valid samples.
    init();
    IMU_GravityReset();
    IMU_data.accel_raw[0] = 0.3f;
    IMU_data.accel_raw[1] = -0.2f;
    IMU_data.accel_raw[2] = 10.0f;
    IMU_data.gyro_raw[0] = 0.01f;
    IMU_data.gyro_raw[1] = -0.02f;
    IMU_data.gyro_raw[2] = 0.03f;
    float measured_gravity = sqrtf(100.13f);
    accel_fliter_3[0] = -1;
    for (unsigned i = 0; i < IMU_GRAVITY_SAMPLE_COUNT - 1; ++i) {
        IMU_GravityUpdata();
        IMU_SpeedUpdata(0.001f);
        assert(!IMU_data.gravity_ready && !IMU_data.speed_valid);
        near(IMU_data.speed[0], 0);
    }
    assert(IMU_data.gravity_sample_count == 2999);
    IMU_GravityUpdata();
    assert(IMU_data.gravity_ready && !IMU_data.speed_valid);
    near(IMU_data.gravity, measured_gravity);
    for (int axis = 0; axis < 3; ++axis) {
        near(IMU_data.gyro_offset[axis], IMU_data.gyro_raw[axis]);
        near(IMU_data.accel_offset[axis], IMU_data.accel_raw[axis] -
             ((axis == 2) ? measured_gravity : 0));
    }

    // Result is used for gravity subtraction, and persists after speed reset.
    accel_fliter_3[0] = 0;
    accel_fliter_3[2] = measured_gravity;
    for (int i = 0; i < 1000; ++i) IMU_SpeedUpdata(0.001f);
    assert(IMU_data.speed_valid);
    near(IMU_data.speed[2], 0);
    IMU_SpeedReset();
    assert(IMU_data.gravity_ready);
    near(IMU_data.gravity, measured_gravity);
    IMU_data.accel_raw[2] = 20;
    IMU_GravityUpdata();
    near(IMU_data.gravity, measured_gravity); // no recalibration while driving

    // Nonconstant samples are averaged; motion and invalid samples restart window.
    IMU_GravityReset();
    memset(IMU_data.accel_raw, 0, sizeof(IMU_data.accel_raw));
    memset(IMU_data.gyro_raw, 0, sizeof(IMU_data.gyro_raw));
    for (int i = 0; i < 100; ++i) {
        IMU_data.accel_raw[2] = (i % 2) ? 9.8f : 9.9f;
        IMU_GravityUpdata();
    }
    near(IMU_data.gravity, 9.85f);
    IMU_data.gyro_raw[1] = 1;
    IMU_GravityUpdata();
    assert(!IMU_data.gravity_ready && IMU_data.gravity_sample_count == 0);
    IMU_data.gyro_raw[1] = 0;
    IMU_GravityUpdata();
    IMU_data.accel_raw[0] = 1; // translation/shaking above vector deviation threshold
    IMU_GravityUpdata();
    assert(IMU_data.gravity_sample_count == 0);
    IMU_data.accel_raw[0] = NAN;
    IMU_GravityUpdata();
    assert(IMU_data.gravity_sample_count == 0);
    IMU_data.accel_raw[0] = 0;
    IMU_data.accel_raw[2] = 0; // missing accelerometer data
    IMU_GravityUpdata();
    assert(IMU_data.gravity_sample_count == 0);
    IMU_data.accel_raw[2] = 9.7f;
    for (unsigned i = 0; i < IMU_GRAVITY_SAMPLE_COUNT; ++i) IMU_GravityUpdata();
    assert(IMU_data.gravity_ready);
    near(IMU_data.gravity, 9.7f);

    // Gross tilt, upside-down mounting and changing slow gyro reject calibration.
    IMU_GravityReset();
    IMU_data.accel_raw[0] = 6;
    IMU_data.accel_raw[2] = 8;
    IMU_GravityUpdata();
    assert(IMU_data.gravity_sample_count == 0);
    IMU_data.accel_raw[0] = 0;
    IMU_data.accel_raw[2] = -9.8f;
    IMU_GravityUpdata();
    assert(IMU_data.gravity_sample_count == 0);
    IMU_data.accel_raw[2] = 9.8f;
    IMU_GravityUpdata();
    IMU_data.gyro_raw[2] = 0.05f;
    IMU_GravityUpdata();
    assert(IMU_data.gravity_sample_count == 0);

    // Exercise actual preprocessing: calibration transition reseeds the filter,
    // removes gyro bias without a derivative spike, levels tilt, preserves yaw.
    init();
    IMU_GravityReset();
    IMU_data.AHRS.q[0] = cosf(0.4f);
    IMU_data.AHRS.q[3] = sinf(0.4f);
    IMU_data.accel_raw[0] = 0.3f;
    IMU_data.accel_raw[1] = -0.2f;
    IMU_data.accel_raw[2] = 10.0f;
    IMU_data.gyro_raw[0] = 0.01f;
    IMU_data.gyro_raw[1] = -0.02f;
    IMU_data.gyro_raw[2] = 0.03f;
    for (unsigned i = 0; i < IMU_GRAVITY_SAMPLE_COUNT; ++i) {
        assert(IMU_AccelUpdata(0.001f));
        if (i + 1 < IMU_GRAVITY_SAMPLE_COUNT) assert(!IMU_data.gravity_ready);
    }
    assert(IMU_data.gravity_ready);
    near(IMU_data.AHRS.q[0], cosf(0.4f));
    near(IMU_data.AHRS.q[3], sinf(0.4f));
    for (int i = 0; i < 3000; ++i) {
        assert(IMU_AccelUpdata(0.001f));
        IMU_SpeedUpdata(0.001f);
    }
    for (int axis = 0; axis < 3; ++axis) {
        near(IMU_data.gyro[axis], 0);
        near(IMU_data.accel_rotation[axis], 0);
        near(IMU_data.speed[axis], 0);
    }

    // Constant spin about Z: a_IMU=(-r*w^2,0,g), center remains stationary.
    init();
    IMU_data.gyro_raw[2] = 2;
    IMU_data.accel_raw[0] = -IMU_POSITION_X * 4;
    IMU_data.accel_raw[2] = IMU_GRAVITY;
    for (int i = 0; i < 1000; ++i) {
        assert(IMU_AccelUpdata(0.001f));
        float half_yaw = i * 0.001f;
        IMU_data.AHRS.q[0] = cosf(half_yaw);
        IMU_data.AHRS.q[3] = sinf(half_yaw);
        IMU_SpeedUpdata(0.001f);
    }
    near(IMU_data.accel_rotation[0], -IMU_POSITION_X * 4);
    for (int axis = 0; axis < 3; ++axis) near(IMU_data.speed[axis], 0);

    // Accelerating/braking rotation on all three axes, with simultaneous center
    // translation. Independent cross products generate synthetic sensor input.
    init();
    const float r[3] = {IMU_POSITION_X, IMU_POSITION_Y, IMU_POSITION_Z};
    const float a_center[3] = {0.3f, -0.4f, IMU_GRAVITY + 0.5f};
    float previous[3] = {0};
    for (int n = 0; n < 2000; ++n) {
        float w[3] = {0.1f * sinf(n * 0.005f), 0.2f * cosf(n * 0.003f),
                      2.0f * sinf(n * 0.002f)};
        float alpha[3], wxr[3], expected[3];
        for (int i = 0; i < 3; ++i) {
            alpha[i] = n ? (w[i] - previous[i]) / 0.001f : 0;
            int j = (i + 1) % 3, k = (i + 2) % 3;
            wxr[i] = w[j] * r[k] - w[k] * r[j];
        }
        for (int i = 0; i < 3; ++i) {
            int j = (i + 1) % 3, k = (i + 2) % 3;
            expected[i] = alpha[j] * r[k] - alpha[k] * r[j] + w[j] * wxr[k] - w[k] * wxr[j];
            IMU_data.gyro_raw[i] = w[i];
            IMU_data.accel_raw[i] = a_center[i] + expected[i];
            previous[i] = w[i];
        }
        assert(IMU_AccelUpdata(0.001f));
        for (int i = 0; i < 3; ++i) {
            near(IMU_data.accel_rotation[i], expected[i]);
            near(accel_fliter_3[i], a_center[i]);
        }
    }
    IMU_data.accel_raw[0] = NAN;
    assert(!IMU_AccelUpdata(0.001f));
    assert(!IMU_data.speed_valid && !rotation_started);
    IMU_data.accel_raw[0] = 0;
    assert(!IMU_AccelUpdata(0));
    assert(!IMU_AccelUpdata(NAN));
    IMU_GravityReset();
    for (int i = 0; i < 3; ++i) {
        near(IMU_data.gyro_offset[i], 0);
        near(IMU_data.accel_offset[i], 0);
        near(IMU_data.accel_rotation[i], 0);
    }

    // Real update-function wiring: raw -> calibration -> corrected AHRS inputs
    // -> speed; speed starts one frame after calibration, never before.
    init();
    IMU_GravityReset();
    sample_accel[0] = 0.3f;
    sample_accel[1] = -0.2f;
    sample_accel[2] = 10;
    sample_gyro[0] = 0.01f;
    sample_gyro[1] = -0.02f;
    sample_gyro[2] = 0.03f;
    for (unsigned n = 0; n < IMU_GRAVITY_SAMPLE_COUNT; ++n) {
        IMU_Updata();
        assert(!IMU_data.speed_valid);
        assert(IMU_data.gravity_sample_count == n + 1);
    }
    assert(IMU_data.gravity_ready);
    IMU_Updata();
    assert(IMU_data.speed_valid);
    for (int i = 0; i < 3; ++i) {
        near(IMU_data.accel_raw[i], sample_accel[i]);
        near(IMU_data.gyro_raw[i], sample_gyro[i]);
        near(ahrs_gyro[i], 0);
        near(ahrs_accel[i], (i == 2) ? IMU_data.gravity : 0);
        near(IMU_data.speed[i], 0);
    }
    ahrs_result = 0;
    IMU_Updata();
    assert(!IMU_data.speed_valid);
    ahrs_result = 1;
    int previous_calls = ahrs_calls;
    sample_gyro[0] = NAN;
    IMU_Updata();
    assert(!IMU_data.speed_valid && ahrs_calls == previous_calls);
    puts("IMU speed numerical tests passed");
    return 0;
}
"""

with tempfile.TemporaryDirectory(prefix="imu-speed-test-") as directory:
    path = Path(directory)
    (path / "test.c").write_text(fixture)
    for definitions in ([], ["-DTEST_DEADZONE"], ["-DTEST_ZERO_POSITION"]):
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                        *shlex.split(os.environ.get("HOST_CFLAGS", "")), *definitions,
                        str(path / "test.c"), "-lm", "-o", str(path / "test")], check=True)
        subprocess.run([str(path / "test")], check=True)
