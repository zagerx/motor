/*
 * Field Oriented Control (FOC) implementation
 *
 * Implements FOC control algorithms for BLDC/PMSM motors
 * Features:
 * - Current loop regulation
 * - Position/speed control
 * - Open loop control
 */

 #include "dsp/controller_functions.h"
 #include "zephyr/device.h"
 
 #include <zephyr/logging/log.h>
 #include <lib/foc/foc.h>
 #include <drivers/currsmp.h>
 #include <drivers/pwm.h>
 #include <lib/focutils/svm/svm.h>
 #include <lib/focutils/utils/utils.h>
 #include <drivers/feedback.h>
 #include <zephyr/drivers/gpio.h>
 #include <stm32h7xx_ll_gpio.h> /* TODO: STM32 specific */
 
 LOG_MODULE_REGISTER(foc, LOG_LEVEL_DBG);
 
 #define DT_DRV_COMPAT foc_ctrl_algo 
 
 /* FOC configuration structure */
 struct foc_config {
     const struct device *pwm;       /* PWM device */
     const struct device *currsmp;   /* Current sampling device */
     const struct device *feedback;  /* Feedback device (encoder/hall) */
     void (*modulate)(svm_t*,float,float); /* Modulation function */
 };
 
 /* FOC runtime data */
 struct foc_data {
     svm_t *svm_handle;              /* Space Vector Modulation handle */
 
     float self_theta;               /* Internal theta for open loop */
     float test_a;                   /* Test variable */
     
     /* Read only variables */
     float i_d, i_q;                 /* D/Q axis currents */
     float rads;                     /* Rotor speed (rad/s) */
     float angle;                    /* Mechanical angle */
     float eangle;                   /* Electrical angle */
     float sin_eangle, cos_eangle;   /* sin/cos of electrical angle */
     float v_alpha, v_beta;          /* Alpha/beta voltages */
     float v_q, v_d;                 /* Q/D axis voltages */
 };
 
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
 
 extern const struct gpio_dt_spec led;
 
 /*
  * Current regulator callback
  * Implements FOC current control loop
  */
 static void foc_curr_regulator(void *ctx)
 {    
     struct device *dev = (struct device*)ctx;
     struct foc_config *cfg = (struct foc_config *)dev->config;
     struct device *currsmp = (struct device *)cfg->currsmp;
     struct foc_data *data = dev->data;
 
     struct currsmp_curr current_now;
     LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_1);

     /* Get current measurements */
     currsmp_get_currents(currsmp, &current_now);
     data->eangle = feedback_get_eangle(cfg->feedback);
 
     /* Open loop test generation */
     float alph, beta, sin_the, cos_the;
     sin_cos_f32((data->self_theta * 57.2957795131f), &sin_the, &cos_the);
     
     /* Update theta */
     data->self_theta += 0.0008f;
     if (data->self_theta > 6.28f) {
         data->self_theta = 0.0f;
     }
 
     /* Inverse Park transform */
     inv_park_f32(0.01f, 0.0f, &alph, &beta, sin_the, cos_the);
     
     /* Space Vector Modulation */
     cfg->modulate(data->svm_handle, alph, beta);
 
     /* Set PWM duty cycles */
     svm_t *svm = data->svm_handle;
     pwm_set_phase_voltages(cfg->pwm, svm->duties.a, svm->duties.b, svm->duties.c);
     data->test_a = svm->duties.a * 10000.0f;
     LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_1);
 }
 
 /*
  * Initialize FOC device
  * Returns: 0 on success
  */
 static int foc_init(const struct device* dev)
 {
     const struct foc_config *cfg = dev->config;
     const struct device *currsmp = cfg->currsmp;
     currsmp_configure(currsmp, foc_curr_regulator, (void *)dev);
     LOG_INF("foc_init name: %s", dev->name);
     return 0;
 }
 
 /*
  * Start FOC control
  * Enables PWM outputs
  */
 void foc_start(const struct device* dev)
 {
     const struct foc_config *cfg = dev->config;
     const struct device *devc = cfg->pwm;
     pwm_start(devc);
 }
 
 /* Device instance macro */
 #define FOC_INIT(n) \
     static const struct foc_api foc_api_##n = { \
         .posloop = foc_posloop, \
         .currloop = foc_currentloop, \
         .opencloop = foc_openloop, \
     }; \
     static svm_t svm_##n; \
     static struct foc_data foc_data_##n = { \
         .svm_handle = &svm_##n, \
     }; \
     static const struct foc_config foc_cfg_##n = { \
         .pwm = DEVICE_DT_GET(DT_INST(n, st_stm32_pwm_custom)), \
         .currsmp = DEVICE_DT_GET(DT_INST(n, st_stm32_currsmp_shunt)), \
         .feedback = DEVICE_DT_GET(DT_INST(n, st_stm32_abz_hall)), \
         .modulate = svm_set, \
     }; \
     DEVICE_DT_INST_DEFINE(n, \
         &foc_init, \
         NULL, \
         &foc_data_##n, \
         &foc_cfg_##n, \
         POST_KERNEL, \
         99, \
         &foc_api_##n \
     );
 
 /* Initialize all FOC instances */
 DT_INST_FOREACH_STATUS_OKAY(FOC_INIT)