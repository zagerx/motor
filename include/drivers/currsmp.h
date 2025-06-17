/**
 * @file
 *
 * Current Sampling API.
 *
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef _SPINNER_DRIVERS_CURRSMP_H_
 #define _SPINNER_DRIVERS_CURRSMP_H_
 
 #include <zephyr/device.h>
 #include <zephyr/types.h>
 
 /**
  * @defgroup spinner_drivers_currsmp Current Sampling API
  * @ingroup spinner_drivers
  * @{
  */
 
 /** @brief Current sampling regulation callback. */
 typedef void (*currsmp_regulation_cb_t)(void *ctx);
 
 /** @brief Current sampling currents. */
 struct currsmp_curr {
     /** Phase a current. */
     float i_a;
     /** Phase b current. */
     float i_b;
     /** Phase c current. */
     float i_c;
 };
 
 /** @cond INTERNAL_HIDDEN */
 
 struct currsmp_driver_api {
     void (*configure)(const struct device *dev,
               currsmp_regulation_cb_t regulation_cb, void *ctx);
     void (*get_currents)(const struct device *dev,
                  struct currsmp_curr *curr);
     void (*get_bus_volcurr)(const struct device *dev,float* vol,float* curr);
 };
 
 /** @endcond */
 
 /**
  * @brief Configure current sampling device.
  *
  * @note This function needs to be called before calling currsmp_start().
  *
  * @param[in] dev Current sampling device.
  * @param[in] regulation_cb Callback called on each regulation cycle.
  * @param[in] ctx Callback context.
  */
 static inline void currsmp_configure(const struct device *dev,
                      currsmp_regulation_cb_t regulation_cb,
                      void *ctx)
 {
     const struct currsmp_driver_api *api = dev->api;
 
     api->configure(dev, regulation_cb, ctx);
 }
 
 /**
  * @brief Get phase currents.
  *
  * @param[in] dev Current sampling device.
  * @param[out] curr Pointer where phase currents will be stored.
  */
 static inline void currsmp_get_currents(const struct device *dev,
                     struct currsmp_curr *curr)
 {
     const struct currsmp_driver_api *api = dev->api;
 
     api->get_currents(dev, curr);
 }
 
 static inline void currsmp_get_bus_vol_curr(const struct device *dev,
    float *bus_vol,float *bus_curr)
{
const struct currsmp_driver_api *api = dev->api;
api->get_bus_volcurr(dev, bus_vol,bus_curr);
}


 float currsmp_get_busvol(void);
 
 /** @} */
 
 #endif /* _SPINNER_DRIVERS_CURRSMP_H_ */
 