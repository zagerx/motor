/**
 * @file
 *
 * SV-PWM API.
 *
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef _DRIVERS_PWM_H_
 #define _DRIVERS_PWM_H_
 
 #include <stdint.h>
#include <zephyr/device.h>
 #include <zephyr/types.h>
 
 /**
  * @defgroup spinner_drivers_svpwm SV-PWM API
  * @ingroup spinner_drivers
  * @{
  */
 
 /** @cond INTERNAL_HIDDEN */
 
 struct pwm_driver_api {
     void (*start)(const struct device *dev);
     void (*stop)(const struct device *dev);
     void (*set_phase_voltages)(const struct device *dev,
                    float ua, float ub, float uc);
     void (*set_phase_state)(const struct device *dev,int8_t flag);
};
 
 /** @endcond */
 
 /**
  * @brief Start the SV-PWM controller.
  *
  * @note Current sampling device must be started prior to SV-PWM, since SV-PWM
  * is the responsible to trigger current sampling measurements.
  *
  * @param[in] dev SV-PWM device.
  */
 static inline void pwm_start(const struct device *dev)
 {
     const struct pwm_driver_api *api = dev->api;
 
     api->start(dev);
 }
 
 /**
  * @brief Stop the SV-PWM controller.
  *
  * @param[in] dev SV-PWM device.
  */
 static inline void pwm_stop(const struct device *dev)
 {
     const struct pwm_driver_api *api = dev->api;
 
     api->stop(dev);
 }
 
 /**
  * @brief Set phase voltages.
  *
  * @param[in] dev SV-PWM device.
  * @param[in] v_alpha Alpha voltage.
  * @param[in] v_beta Beta voltage.
  */
 static inline void pwm_set_phase_voltages(const struct device *dev,
                         float ua, float ub, float uc)
 {
    //  float a, b, c;
     const struct pwm_driver_api *api = dev->api;
 
     /* normalization */
    //  a = (ua + 1.0f) * 0.5f;
    //  b = (ub + 1.0f) * 0.5f;
    //  c = (uc + 1.0f) * 0.5f;
     api->set_phase_voltages(dev, ua, ub, uc);
 }
 
 static inline void svpwm_set_phase_state(const struct device *dev,int8_t flag)
 {
    
     const struct pwm_driver_api *api = dev->api;
 
     if (api->set_phase_state) {
        if(flag)
        {
            api->set_phase_state(dev,1);
        }else{
            api->set_phase_state(dev,0);
        }
     }
 }
 
 /** @} */
 
 #endif /* _SPINNER_DRIVERS_SVPWM_H_ */
 