/*
 * webserial_print.c
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 */


/*
 *  Sends orientation, quaternion, and (static) calibration data through
 *  the serial port in the exact text format expected by the Adafruit
 *  3D Model Viewer (Web Serial) tool:
 *
 *      Orientation: <heading>, <roll>, <pitch>\r\n
 *      Quaternion: <w>, <x>, <y>, <z>\r\n
 *      Calibration: <sys>, <gyro>, <accel>, <mag>\r\n
 *
 *  Data source:
 *   - euler_angle (phi=roll, theta=pitch, psi=yaw)  from quat2euler()
 *   - quaternion  (s=q1/w, x=q2, y=q3, z=q4)        from run_kalman_extended()
 *
 *  Mapping note:
 *   Adafruit's original BNO055 example sends euler.x as heading,
 *   euler.y as roll, euler.z as pitch (BNO055's own euler convention),
 *   and uses (360 - heading) so the 3D model viewer rotates in the
 *   expected on-screen direction. To stay compatible with that viewer,
 *   our yaw (psi) is mapped to "heading", our roll (phi) to "roll",
 *   and our pitch (theta) to "pitch".
 */

#include "webserial_print.h"
#include "stdio.h"

volatile uint8_t g_mag_ready = 0;

void webserial_print_frame(euler_angle euler, quaternion quat)
{
    float32_t heading = euler.psi;   /* yaw   -> heading */
    float32_t roll    = euler.phi;   /* roll  -> roll    */
    float32_t pitch   = euler.theta; /* pitch -> pitch   */

    /* normalize heading to 0..360 range before flipping it,
     * same convention as the Adafruit example (360 - x) */
    if (heading < 0.0f)
    {
        heading += 360.0f;
    }

    printf(
        "Orientation: %.2f, %.2f, %.2f\r\n",
        360.0f - heading,
        roll,
        pitch);

    printf(
        "Quaternion: %.4f, %.4f, %.4f, %.4f\r\n",
        quat.s,
        quat.x,
        quat.y,
        quat.z);

    /* Calibration data: not available from this EKF pipeline,
     * so it is reported as fully calibrated (3,3,3,3), the
     * maximum value expected by the Adafruit Web Serial viewer. */
    printf(
        "Calibration: %d, %d, %d, %d\r\n",
        3,
        3,
        3,
        3);
}

/* ---------- Format CSV untuk Python logger ---------- */
void csv_print_header(void)
{
    printf("CSV_HEADER,ms,"
           "ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,"
           "ax,ay,az,gx,gy,gz,"
           "mx_raw,my_raw,mz_raw,mx,my,mz,"
           "q0,q1,q2,q3,roll,pitch,yaw,"
           "bno_h,bno_r,bno_p,"
           "bno_sys,bno_gyr,bno_acc,bno_mag\r\n");
}

void csv_print_frame(uint32_t ms, const icm42688_data *raw,
                     const icm42688_calib_data *cal,
                     int16_t mx_raw, int16_t my_raw, int16_t mz_raw,
                     float mx, float my, float mz,
                     quaternion q, euler_angle ea,
                     float bno_h, float bno_r, float bno_p,
                     uint8_t c_sys, uint8_t c_gyr, uint8_t c_acc, uint8_t c_mag)
{

	if (c_mag == 3) { g_mag_ready = 1; }

	printf("CSV,%lu,"
           "%d,%d,%d,%d,%d,%d,"
           "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
           "%d,%d,%d,%.3f,%.3f,%.3f,"
           "%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,"
           "%.2f,%.2f,%.2f,"
           "%u,%u,%u,%u\r\n",
           (unsigned long)HAL_GetTick(),
           raw->x_accel, raw->y_accel, raw->z_accel,
           raw->x_gyro,  raw->y_gyro,  raw->z_gyro,
           cal->x_accel, cal->y_accel, cal->z_accel,
           cal->x_gyro,  cal->y_gyro,  cal->z_gyro,
           mx_raw, my_raw, mz_raw, mx, my, mz,
           q.s, q.x, q.y, q.z,
           ea.phi, ea.theta, ea.psi,
           bno_h, bno_r, bno_p,
           (unsigned)c_sys, (unsigned)c_gyr, (unsigned)c_acc, (unsigned)c_mag);
}
