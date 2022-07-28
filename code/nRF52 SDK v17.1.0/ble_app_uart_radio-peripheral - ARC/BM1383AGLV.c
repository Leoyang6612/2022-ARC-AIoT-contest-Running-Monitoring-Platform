#include "nrf_drv_twi.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "string.h"

#include "BM1383AGLV.h"

#define UART_EN 1

uint8_t nrf52_twi_read(nrf_drv_twi_t const *p , uint8_t addr, uint8_t reg, uint8_t *data, uint8_t size)
{
    ret_code_t ret;

    ret = nrf_drv_twi_tx(p, addr, &reg, 1, true);
    if(ret == NRF_SUCCESS) {
        ret = nrf_drv_twi_rx(p, addr, data, size);
    }
    if(ret != 0) {
        return ret;
    }

    return NRF_SUCCESS;
}

uint8_t nrf52_twi_write(nrf_drv_twi_t const *p , uint8_t addr, uint8_t reg, uint8_t *data, uint8_t size)
{
    ret_code_t ret;
    uint8_t buf[size+1];
    
    buf[0] = reg;    
    memcpy(&buf[1], data, size);

    ret = nrf_drv_twi_tx(p, addr, buf, size+1, false);
    if(ret != 0) {
        return ret;
    }

    return NRF_SUCCESS;
}


uint8_t BM1383AGLV_init(nrf_drv_twi_t const *p , uint8_t addr )
{
    uint8_t rc;
    uint8_t reg;


    rc = nrf52_twi_read(p,addr,BM1383AGLV_ID, &reg, sizeof(reg));
    if (rc != NRF_SUCCESS) {
        #if UART_EN
        printf("read BM1383AGLV_ID fail\r\n");
        #endif
        return rc;
    }
    

    if(reg != BM1383AGLV_ID_VAL) {
        #if UART_EN
        printf("BM1383AGLV ID fail\r\n");
        #endif
        return rc;
    }
    
    reg = BM1383AGLV_POWER_DOWN_VAL;
    rc = nrf52_twi_write(p,addr,BM1383AGLV_POWER_DOWN, &reg, sizeof(reg));
    if (rc != NRF_SUCCESS) {
        #if UART_EN
        printf("Can't write BM1383AGLV POWER_DOWN register\r\n");
        #endif
        return rc;
    }

    nrf_delay_ms(WAIT_BETWEEN_POWER_DOWN_AND_RESET);

    reg = BM1383AGLV_RESET_VAL;
    rc = nrf52_twi_write(p,addr,BM1383AGLV_RESET, &reg, sizeof(reg));
    if (rc != NRF_SUCCESS) {
        #if UART_EN
        printf("Can't write BM1383AGLV RESET register\r\n");
        #endif
        return rc;
    }
    reg = BM1383AGLV_MODE_CONTROL_VAL;
    rc = nrf52_twi_write(p,addr,BM1383AGLV_MODE_CONTROL, &reg, sizeof(reg));
    if (rc != NRF_SUCCESS) {
        #if UART_EN
        printf("Can't write BM1383AGLV MODE_CONTROL register\r\n");
        #endif
        return rc;
    }

   nrf_delay_ms(WAIT_TMT_MAX);


    return NRF_SUCCESS;
}

uint8_t BM1383AGLV_getraw(nrf_drv_twi_t const *p , uint8_t addr , uint8_t *data)
{
    uint8_t rc;
    rc = nrf52_twi_read(p,addr,BM1383AGLV_PRESSURE_MSB , data , GET_BYTE_PRESS_TEMP);
    if (rc != NRF_SUCCESS) {
        #if UART_EN
        printf("Can't get Press value\r\n");
        #endif
        return rc;
    }
    return NRF_SUCCESS;
}

uint8_t BM1383AGLV_getval(nrf_drv_twi_t const *p ,float *press , float *temp)
{
    uint8_t rc , val[GET_BYTE_PRESS_TEMP];
    uint32_t rawpress;
    uint16_t rawtemp;

    rc = BM1383AGLV_getraw(p ,BM1383AGLV_DEVICE_ADDRESS,val);
    if (rc != NRF_SUCCESS)  
      return rc;

    rawpress = (((uint32_t)val[0] << 16) | ((uint32_t) val[1] << 8) | val[2] & 0xFC) >>2;

    if(rawpress ==0)
      return -1;

    *press = (float)rawpress / HPA_PER_COUNT;
    rawtemp = ((uint16_t)val[3]<<8) | val[4];

    *temp = (float)rawtemp / DEGREES_CELSIUS_PER_COUNT;

    return rc;
}

uint8_t BM1383AGLV_power_down(nrf_drv_twi_t const *p)
{
    uint8_t rc,reg;

    reg = 0;
    nrf52_twi_write(p,BM1383AGLV_DEVICE_ADDRESS ,BM1383AGLV_POWER_DOWN, &reg, sizeof(reg));

    nrf_delay_ms(WAIT_BETWEEN_POWER_DOWN_AND_RESET);

    reg = 0;
    nrf52_twi_write(p,BM1383AGLV_DEVICE_ADDRESS ,BM1383AGLV_RESET, &reg, sizeof(reg));
    return rc;
}