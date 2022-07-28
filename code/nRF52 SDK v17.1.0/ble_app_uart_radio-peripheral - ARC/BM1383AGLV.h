#ifndef _BM1383_AGLV_H
#define _BM1383_AGLV_H

#include <stdint.h>
#include "nrf_drv_twi.h"

#define BM1383AGLV_DEVICE_ADDRESS           (0x5D)    // 7bit Addrss
#define BM1383AGLV_ID_VAL                   (0x32)
#define BM1383AGLV_ID                       (0x10)
#define BM1383AGLV_POWER_DOWN               (0x12)
#define BM1383AGLV_RESET                    (0x13)
#define BM1383AGLV_MODE_CONTROL             (0x14)
#define BM1383AGLV_PRESSURE_MSB             (0x1A)

#define BM1383AGLV_POWER_DOWN_PWR_DOWN          (1 << 0)
#define BM1383AGLV_RESET_RSTB                   (1 << 0)
#define BM1383AGLV_MODE_CONTROL_AVE_NUM16       (4 << 5)
#define BM1383AGLV_MODE_CONTROL_AVE_NUM64       (6 << 5)
#define BM1383AGLV_MODE_CONTROL_RESERVED_3BIT   (1 << 3)
#define BM1383AGLV_MODE_CONTROL_MODE_CONTINUOUS (4 << 0)

#define BM1383AGLV_POWER_DOWN_VAL      (BM1383AGLV_POWER_DOWN_PWR_DOWN)
#define BM1383AGLV_RESET_VAL           (BM1383AGLV_RESET_RSTB)
#define BM1383AGLV_MODE_CONTROL_VAL    (BM1383AGLV_MODE_CONTROL_AVE_NUM16 | BM1383AGLV_MODE_CONTROL_RESERVED_3BIT | BM1383AGLV_MODE_CONTROL_MODE_CONTINUOUS)

#define HPA_PER_COUNT                           (2048)
#define DEGREES_CELSIUS_PER_COUNT               (32)
#define GET_BYTE_PRESS_TEMP                     (5)
#define WAIT_TMT_MAX                            (240)
#define WAIT_BETWEEN_POWER_DOWN_AND_RESET       (2)

uint8_t nrf52_twi_read(nrf_drv_twi_t const *p , uint8_t addr, uint8_t reg, uint8_t *data, uint8_t size);
uint8_t nrf52_twi_write(nrf_drv_twi_t const *p , uint8_t addr, uint8_t reg, uint8_t *data, uint8_t size);
uint8_t BM1383AGLV_init(nrf_drv_twi_t const *p,uint8_t addr);
uint8_t BM1383AGLV_getraw(nrf_drv_twi_t const *p,uint8_t addr , uint8_t *data);
uint8_t BM1383AGLV_getval(nrf_drv_twi_t const *p,float *press , float *temp);
uint8_t BM1383AGLV_power_down(nrf_drv_twi_t const *p);

#endif