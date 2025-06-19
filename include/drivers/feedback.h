/**
 * @file
 *
 * Feedback API.
 *
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef _DRIVERS_FEEDBACK_H_
 #define _DRIVERS_FEEDBACK_H_
 
 #include <zephyr/device.h>
 #include <zephyr/types.h>
 
 #ifndef M_PI
 #define M_PI		3.14159265358979323846f
 #endif
 
 #ifndef M_TWOPI
 #define M_TWOPI         (M_PI * 2.0f)
 #endif
 
 /**
  * @defgroup spinner_drivers_feedback Feedback API
  * @ingroup spinner_drivers
  * @{
  */
 
 /** @cond INTERNAL_HIDDEN */
 
 struct feedback_driver_api {
     /* Get velocity. unit: rad/s */
     float (*get_rads)(const struct device *dev);
 
     /* Get mechanical angle. unit: rad. */
     float (*get_eangle)(const struct device *dev);
 
     /* Get  electrical angle. unit: rad. */
     float (*get_mangle)(const struct device *dev);
 
     /* Get  position. unit: rad. */
     float (*get_rel_odom)(const struct device *dev);

     void (*set_rel_odom)(const struct device *dev);
     /* feedback calibration */
     int (*calibration)(const struct device *dev);

     void (*feedback_enable)(const struct device *dev);
 };
 
 /** @endcond */

 
 static inline void feedback_start(const struct device *dev)
 {
    const struct feedback_driver_api *api = dev->api;
    api->feedback_enable(dev);
    return;
 }


 /**
  * @brief Get rads.
  *
  * @param dev Feedback instance.
  * @return rad/s.
  */
 static inline float feedback_get_rads(const struct device *dev)
 {
     const struct feedback_driver_api *api = dev->api;
 
     return api->get_rads(dev);
 }
 
 /**
  * @brief Get electrical angle.
  *
  * @param dev Feedback instance.
  * @return Electrical angle.
  */
 static inline float feedback_get_eangle(const struct device *dev)
 {
     const struct feedback_driver_api *api = dev->api;
 
     return api->get_eangle(dev);
 }
 
 /**
  * @brief Get Machine angle.
  *
  * @param dev Feedback instance.
  * @return Machine angle.
  */
 static inline float feedback_get_mangle(const struct device *dev)
 {
     const struct feedback_driver_api *api = dev->api;
 
     if (api->get_eangle) {
         return api->get_eangle(dev);
     }
 
     return 0.0f;
 }
 
 /**
  * @brief Get electrical angle.
  *
  * @param dev Feedback instance.
  * @return Electrical angle.
  */
 static inline float feedback_get_position(const struct device *dev)
 {
     const struct feedback_driver_api *api = dev->api;
 
     if (api->get_rel_odom) {
         return api->get_rel_odom(dev);
     }
 
     return 0.0f;
 }
 static inline void feedback_set_pos(const struct device *dev)
 {
     const struct feedback_driver_api *api = dev->api;
 
     if (api->set_rel_odom) {
        api->set_rel_odom(dev);
     }
 }
  
 /**
  * @brief Feedback calibration
  *
  * @param dev Feedback instance.
  * @return 0 sucess.
  */
 static inline int feedback_calibration(const struct device *dev)
 {
     const struct feedback_driver_api *api = dev->api;
 
     if (api->calibration) {
         return api->calibration(dev);
     }
 
     return 0;
 }
 
 /** @} */
 
 #endif /* _SPINNER_DRIVERS_FEEDBACK_H_ */
 