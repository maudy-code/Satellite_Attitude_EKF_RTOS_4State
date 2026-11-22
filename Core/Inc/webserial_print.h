/*
 * webserial_print.h
 */

#ifndef INC_WEBSERIAL_PRINT_H_
#define INC_WEBSERIAL_PRINT_H_

#include <stdint.h>
#include "arm_math.h"
#include "quaternion2imu.h"  /* quaternion struct: s, x, y, z */
#include "quat2euler.h"      /* euler_angle struct: phi, theta, psi */
#include "icm42688.h"
#include "icm42688_calib.h"

/* Format untuk Adafruit Web Serial 3D viewer (rabbit) */
void webserial_print_frame(euler_angle euler, quaternion quat);

/* Format CSV untuk logger Python */
void csv_print_header(void);

void csv_print_frame( uint32_t ms, const icm42688_data *raw,
                     const icm42688_calib_data *cal,
                     int16_t mx_raw, int16_t my_raw, int16_t mz_raw,
                     float mx, float my, float mz,
                     quaternion q, euler_angle ea,
                     float bno_h, float bno_r, float bno_p,
                     uint8_t c_sys, uint8_t c_gyr, uint8_t c_acc, uint8_t c_mag);

#endif /* INC_WEBSERIAL_PRINT_H_ */
