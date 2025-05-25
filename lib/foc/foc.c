/*
 * Field Oriented Control (FOC) implementation
 *
 * Implements FOC control algorithms for BLDC/PMSM motors
 * Features:
 * - Current loop regulation
 * - Position/speed control
 * - Open loop control
 */

 #include "algorithmlib/pid.h"
#include "zephyr/device.h"
 
 #include <zephyr/logging/log.h>
 #include <lib/foc/foc.h>

 LOG_MODULE_REGISTER(foc, LOG_LEVEL_DBG);
 
 #define DT_DRV_COMPAT foc_ctrl_algo 
 
 /* FOC configuration structure */


static void _write(const struct device* dev,int16_t flag,float *input)
{
    struct foc_data *data = dev->data;
    switch (flag) {
        case FOC_PARAM_D_PID:
            {
                float kp,ki,kc,max,min;
                kp = input[0];ki = input[1];kc = input[2];max = input[3];min = input[4];
                pid_init(&data->id_pid,kp,ki,kc,max,min);
            }
        break;
        case FOC_PARAM_Q_PID:
            {
                float kp,ki,kc,max,min;
                kp = input[0];ki = input[1];kc = input[2];max = input[3];min = input[4];
                pid_init(&data->iq_pid,kp,ki,kc,max,min);
            }
        break;
        case FOC_PARAM_DQ_REAL:
            {
                data->i_d = input[0];
                data->i_q = input[1];
            }
        break;
        case FOC_PARAM_ME_ANGLE_REAL:
            {
                data->angle = input[0];
                data->eangle = input[1];
            }
        break;
    } 
}
 /*
  * Position control loop (stub)
  * Returns: 0 on success
  */
 static int foc_posloop(const struct device* dev)
 {
     return 0;
 }
 
 /*
  * Current control loop (stub)
  * Returns: 0 on success
  */
 static int foc_currentloop(const struct device* dev)
 {
     return 0;
 }
 
 /*
  * Open loop control (stub)
  * Returns: 0 on success
  */
 static int foc_openloop(const struct device* dev)
 {
     return 0;
 }
 
 

 /*
  * Initialize FOC device
  * Returns: 0 on success
  */
 static int foc_init(const struct device* dev)
 {
    const struct foc_data *data = dev->data;
    pid_init((pid_cb_t*)&(data->id_pid), 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    pid_init((pid_cb_t*)&(data->iq_pid), 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    return 0;
 }




 /* Device instance macro */
 #define FOC_INIT(n) \
     static const struct foc_api foc_api_##n = { \
         .posloop = foc_posloop, \
         .currloop = foc_currentloop, \
         .opencloop = foc_openloop, \
         .write_data = _write,\
     }; \
     static svm_t svm_##n; \
     static struct foc_data foc_data_##n = { \
         .svm_handle = &svm_##n, \
     }; \
     static const struct foc_config foc_cfg_##n = { \
         .modulate = svm_set, \
     }; \
     DEVICE_DT_INST_DEFINE(n, \
         &foc_init, \
         NULL, \
         &foc_data_##n, \
         &foc_cfg_##n, \
         POST_KERNEL, \
         CONFIG_FOC_INIT_PRIORITY, \
         &foc_api_##n \
     );
 
 /* Initialize all FOC instances */
 DT_INST_FOREACH_STATUS_OKAY(FOC_INIT)