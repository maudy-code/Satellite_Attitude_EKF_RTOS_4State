#ifndef INC_APP_RTOS_H_
#define INC_APP_RTOS_H_

#include <stdint.h>
#include "icm42688.h"
#include "icm42688_calib.h"
#include "quaternion2imu.h"
#include "quat2euler.h"

#define SENSOR_RATE_HZ   100   /* laju baca sensor dan EKF */
#define LOG_DECIM        3     /* kirim 1 dari tiap 3 hasil EKF ke UART */
#define CONTROL_RATE_HZ  50

typedef struct {
    uint32_t            t_ms;
    quaternion          q;
    euler_angle         ea;
    icm42688_calib_data cal;
} attitude_state_t;

void app_rtos_init(void);
int  app_attitude_get(attitude_state_t *out);

#endif /* INC_APP_RTOS_H_ */
