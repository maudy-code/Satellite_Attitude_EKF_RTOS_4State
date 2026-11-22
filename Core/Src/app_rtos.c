#include "app_rtos.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "icm42688.h"
#include "icm42688_calib.h"
#include "bno055.h"
#include "mag_calib.h"
#include "imu_norm.h"
#include "quaternion.h"
#include "imu2quaternion.h"
#include "attitude_kalman_extended.h"
#include "quat2euler.h"
#include "webserial_print.h"

#define CPU_MHZ      120U
#define BNO_MAG_DIV  ((SENSOR_RATE_HZ / 20) > 0 ? (SENSOR_RATE_HZ / 20) : 1)
#define BNO_EUL_DIV  ((SENSOR_RATE_HZ / 10) > 0 ? (SENSOR_RATE_HZ / 10) : 1)

typedef struct {
    uint32_t      t_ms;
    uint32_t      t_cyc;
    icm42688_data raw;
    int16_t       mx_raw, my_raw, mz_raw;
    uint8_t       mag_new;
    float         bno_h, bno_r, bno_p;
    uint8_t       c_sys, c_gyr, c_acc, c_mag;
} sample_t;

typedef struct {
    uint32_t            t_ms;
    icm42688_data       raw;
    icm42688_calib_data cal;
    int16_t             mx_raw, my_raw, mz_raw;
    float               mx, my, mz;
    quaternion          q;
    euler_angle         ea;
    float               bno_h, bno_r, bno_p;
    uint8_t             c_sys, c_gyr, c_acc, c_mag;
} log_frame_t;

static osMessageQueueId_t qSample;
static osMessageQueueId_t qLog;
static osMutexId_t        mtxAtt;
static attitude_state_t   g_att;
static uint8_t            g_att_valid = 0;
static mag_calib_t        mag_calib;

/* metrik untuk Live Expressions */
volatile uint32_t dbg_sens_us_max = 0;
volatile uint32_t dbg_ekf_us_max  = 0;
volatile uint32_t dbg_dt_us_min   = 0xFFFFFFFFU;
volatile uint32_t dbg_dt_us_max   = 0;
volatile uint32_t dbg_q_drop      = 0;
volatile uint32_t dbg_log_drop    = 0;
volatile uint32_t dbg_log_sent    = 0;
volatile uint32_t dbg_icm_err     = 0;
volatile uint32_t dbg_ekf_us_last = 0;

/* ======================= SensorTask ======================= */
static void SensorTask(void *arg)
{
    (void)arg;
    const uint32_t period = osKernelGetTickFreq() / SENSOR_RATE_HZ;
    uint32_t next = osKernelGetTickCount();
    uint32_t n = 0;

    int16_t mx = 0, my = 0, mz = 0;
    float   bh = 0, br = 0, bp = 0;
    uint8_t cs = 0, cg = 0, ca = 0, cm = 0;

    for (;;)
    {
        next += period;

        sample_t s;
        memset(&s, 0, sizeof(s));
        uint32_t t0 = DWT->CYCCNT;
        s.t_cyc = t0;
        s.t_ms  = osKernelGetTickCount();

        if (icm42688_readRawSensor(&s.raw) == HAL_OK)
        {
            if ((n % BNO_MAG_DIV) == 0)
            {
                bno055_getRawMagnetometer(&mx, &my, &mz);
                s.mag_new = 1;
            }
            if ((n % BNO_EUL_DIV) == 3)
            {
                bno055_calibration_state_t st = bno055_getCalibrationState();
                cs = st.sys; cg = st.gyro; ca = st.accel; cm = st.mag;
                bno055_vector_t e = bno055_getVectorEuler();
                bh = e.x; br = e.y; bp = e.z;
            }
            s.mx_raw = mx; s.my_raw = my; s.mz_raw = mz;
            s.bno_h = bh;  s.bno_r = br;  s.bno_p = bp;
            s.c_sys = cs;  s.c_gyr = cg;  s.c_acc = ca;  s.c_mag = cm;

            if (osMessageQueuePut(qSample, &s, 0, 0) != osOK) { dbg_q_drop++; }
        }
        else
        {
            dbg_icm_err++;
        }

        uint32_t us = (DWT->CYCCNT - t0) / CPU_MHZ;
        if (us > dbg_sens_us_max) { dbg_sens_us_max = us; }

        n++;
        osDelayUntil(next);
    }
}

/* ======================== EKFTask ========================= */
static void EKFTask(void *arg)
{
    (void)arg;
    sample_t            s;
    icm42688_calib_data cal;
    imu_norm            imu;
    quaternion          qekf;
    euler_angle         ea;
    float               mx = 0, my = 0, mz = 0;
    uint32_t            prev_cyc = 0, cnt = 0;
    uint8_t             have_prev = 0;

    for (;;)
    {
        osMessageQueueGet(qSample, &s, NULL, osWaitForever);
        uint32_t t0 = DWT->CYCCNT;

        /* dt nyata dari timestamp, dalam detik */
        float dt_s = 1.0f / SENSOR_RATE_HZ;
        if (have_prev)
        {
            uint32_t dt_cyc = s.t_cyc - prev_cyc;
            uint32_t dt_us  = dt_cyc / CPU_MHZ;
            if (dt_us < dbg_dt_us_min) { dbg_dt_us_min = dt_us; }
            if (dt_us > dbg_dt_us_max) { dbg_dt_us_max = dt_us; }

            float d = (float)dt_cyc / (CPU_MHZ * 1.0e6f);
            if (d > 0.0f && d < 0.1f) { dt_s = d; }
        }
        prev_cyc  = s.t_cyc;
        have_prev = 1;

        icm42688_calib_apply_accel_and_gyro(&s.raw, &cal);
        mag_correct_ellipsoid(s.mx_raw, s.my_raw, s.mz_raw,
                              &mag_calib, &mx, &my, &mz);

        sensor2imu(cal, mx, my, mz, &imu);   /* gyro dalam rad/s */
        imu.gx *= dt_s;                      /* jadi rad per sampel */
        imu.gy *= dt_s;
        imu.gz *= dt_s;

        attitude_set_mag_valid(s.mag_new);
        run_kalman_extended(imu, &qekf);
        quat2euler(qekf, 1, &ea);            /* 1 = derajat */

        osMutexAcquire(mtxAtt, osWaitForever);
        g_att.t_ms = s.t_ms;
        g_att.q    = qekf;
        g_att.ea   = ea;
        g_att.cal  = cal;
        g_att_valid = 1;
        osMutexRelease(mtxAtt);

        if ((++cnt % LOG_DECIM) == 0)
        {
            log_frame_t f;
            f.t_ms = s.t_ms;   f.raw = s.raw;   f.cal = cal;
            f.mx_raw = s.mx_raw; f.my_raw = s.my_raw; f.mz_raw = s.mz_raw;
            f.mx = mx; f.my = my; f.mz = mz;
            f.q = qekf; f.ea = ea;
            f.bno_h = s.bno_h; f.bno_r = s.bno_r; f.bno_p = s.bno_p;
            f.c_sys = s.c_sys; f.c_gyr = s.c_gyr; f.c_acc = s.c_acc; f.c_mag = s.c_mag;
            if (osMessageQueuePut(qLog, &f, 0, 0) != osOK) { dbg_log_drop++; }
        }


        uint32_t us = (DWT->CYCCNT - t0) / CPU_MHZ;
        dbg_ekf_us_last = us;                          /* tambahkan baris ini */
        if (us > dbg_ekf_us_max) { dbg_ekf_us_max = us; }
    }
}

/* ======================== LogTask ========================= */
static void LogTask(void *arg)
{
    (void)arg;
    log_frame_t f;
    for (;;)
    {
        osMessageQueueGet(qLog, &f, NULL, osWaitForever);
        csv_print_frame(f.t_ms, &f.raw, &f.cal,
                        f.mx_raw, f.my_raw, f.mz_raw, f.mx, f.my, f.mz,
                        f.q, f.ea, f.bno_h, f.bno_r, f.bno_p,
                        f.c_sys, f.c_gyr, f.c_acc, f.c_mag);
        dbg_log_sent++;
    }
}

/* ====================== ControlTask ======================= */
static void ControlTask(void *arg)
{
    (void)arg;
    attitude_state_t a;
    const uint32_t period = osKernelGetTickFreq() / CONTROL_RATE_HZ;
    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        next += period;
        if (app_attitude_get(&a))
        {
            /* TODO: hitung torsi dari a.q dan kecepatan sudut,
             *       lalu kirim PWM ke driver reaction wheel. */
        }
        osDelayUntil(next);
    }
}

/* ====================== API publik ======================== */
int app_attitude_get(attitude_state_t *out)
{
    int ok;
    osMutexAcquire(mtxAtt, osWaitForever);
    ok = g_att_valid;
    if (ok) { *out = g_att; }
    osMutexRelease(mtxAtt);
    return ok;
}

void app_rtos_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    mag_calib_init(&mag_calib);
    attitude_init_extended();
    csv_print_header();

    qSample = osMessageQueueNew(16, sizeof(sample_t),    NULL);
    qLog    = osMessageQueueNew(8,  sizeof(log_frame_t), NULL);
    mtxAtt  = osMutexNew(NULL);
    if (qSample == NULL || qLog == NULL || mtxAtt == NULL) { Error_Handler(); }

    /* stack_size dalam BYTE di CMSIS-RTOS v2 */
    const osThreadAttr_t aSensor  = { .name = "Sensor",  .stack_size = 2048, .priority = (osPriority_t)osPriorityHigh };
    const osThreadAttr_t aEkf     = { .name = "EKF",     .stack_size = 4096, .priority = (osPriority_t)osPriorityAboveNormal };
    const osThreadAttr_t aControl = { .name = "Control", .stack_size = 1024, .priority = (osPriority_t)osPriorityNormal };
    const osThreadAttr_t aLog     = { .name = "Log",     .stack_size = 4096, .priority = (osPriority_t)osPriorityBelowNormal };

    if (osThreadNew(SensorTask,  NULL, &aSensor)  == NULL) { Error_Handler(); }
    if (osThreadNew(EKFTask,     NULL, &aEkf)     == NULL) { Error_Handler(); }
    if (osThreadNew(ControlTask, NULL, &aControl) == NULL) { Error_Handler(); }
    if (osThreadNew(LogTask,     NULL, &aLog)     == NULL) { Error_Handler(); }
}
