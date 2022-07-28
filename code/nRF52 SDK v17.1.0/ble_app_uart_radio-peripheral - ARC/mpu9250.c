#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "string.h"
#include <math.h>
#include <stdio.h>

#include "mpu9250.h"


static uint8_t accel_get_three_axis_tx_buf[6] = {

    (MPU_9250_REG_ACCEL_XOUT_H),
    (MPU_9250_REG_ACCEL_XOUT_L),
    (MPU_9250_REG_ACCEL_YOUT_H),
    (MPU_9250_REG_ACCEL_YOUT_L),
    (MPU_9250_REG_ACCEL_ZOUT_H),
    (MPU_9250_REG_ACCEL_ZOUT_L),
};

static uint8_t gyro_get_three_axis_tx_buf[6] = {

    (MPU_9250_REG_GYRO_XOUT_H),
    (MPU_9250_REG_GYRO_XOUT_L),
    (MPU_9250_REG_GYRO_YOUT_H),
    (MPU_9250_REG_GYRO_YOUT_L),
    (MPU_9250_REG_GYRO_ZOUT_H),
    (MPU_9250_REG_GYRO_ZOUT_L),
};

void mpu9250_accel_three_axis_raw(nrf_drv_twi_t const *p, float * ax, float * ay , float * az) //scaler_len must less than 85
{
    static uint8_t rx_buf[6];
    int16_t x,y,z;
    
    for(int i=0;i<6;i++){
      nrf52_twi_read(p, MPU_9250_SLAVE_ADDRESS, accel_get_three_axis_tx_buf[i], rx_buf+i , sizeof(rx_buf[i]));
      nrf_delay_us(40);
    }
    
    x = ((int16_t)(rx_buf[0])<<8) | rx_buf[1];
    y = ((int16_t)(rx_buf[2])<<8) | rx_buf[3];
    z = ((int16_t)(rx_buf[4])<<8) | rx_buf[5];

    *ax = (float)((x) * 8 / pow(2.0,15.0));
    *ay = (float)((y) * 8 / pow(2.0,15.0));
    *az = (float)((z) * 8 / pow(2.0,15.0));
}
void mpu9250_gyro_three_axis_raw(nrf_drv_twi_t const *p, float * gx, float * gy , float * gz) //scaler_len must less than 85
{
    static uint8_t rx_buf[6];
    int16_t x,y,z;
    
    for(int i=0;i<6;i++){
      nrf52_twi_read(p, MPU_9250_SLAVE_ADDRESS, gyro_get_three_axis_tx_buf[i], rx_buf+i , sizeof(rx_buf[i]));
      nrf_delay_us(40);
    }
    
    
    x = ((int16_t)(rx_buf[0])<<8) | rx_buf[1];
    y = ((int16_t)(rx_buf[2])<<8) | rx_buf[3];
    z = ((int16_t)(rx_buf[4])<<8) | rx_buf[5];

    *gx = (float)(x) * 250.0 / pow(2.0,15.0);
    *gy = (float)(y) * 250.0 / pow(2.0,15.0);
    *gz = (float)(z) * 250.0 / pow(2.0,15.0);
}

void mpu9250_accel_three_axis_raw_i(nrf_drv_twi_t const *p, uint8_t *buf) //scaler_len must less than 85
{
    //static uint8_t rx_buf[6];
    
    for(int i=0;i<6;i++){
      nrf52_twi_read(p, MPU_9250_SLAVE_ADDRESS, accel_get_three_axis_tx_buf[i], buf+i , sizeof(buf[i]));
      nrf_delay_us(40);
    }
    /*
    buf[0] = rx_buf[0];
    buf[1] = rx_buf[1];
    buf[2] = rx_buf[2];
    buf[3] = rx_buf[3];
    buf[4] = rx_buf[4];
    buf[5] = rx_buf[5];
    */

}
void mpu9250_gyro_three_axis_raw_i(nrf_drv_twi_t const *p, uint8_t *buf) //scaler_len must less than 85
{
    //static uint8_t rx_buf[6];
    
    for(int i=0;i<6;i++){
      nrf52_twi_read(p, MPU_9250_SLAVE_ADDRESS, gyro_get_three_axis_tx_buf[i], buf+i , sizeof(buf[i]));
      nrf_delay_us(40);
    }
    /*
    buf[0] = rx_buf[0];
    buf[1] = rx_buf[1];
    buf[2] = rx_buf[2];
    buf[3] = rx_buf[3];
    buf[4] = rx_buf[4];
    buf[5] = rx_buf[5];
    */
}


void mpu9250_accel_init_wom_mode(nrf_drv_twi_t const *p)
{
    mpu_reg_config_t        reg_config          = {.ubyte = 0};
    mpu_reg_accel_config_t  reg_accel_config    = {.ubyte = 0};
    mpu_reg_accel_config2_t reg_accel_config2   = {.ubyte = 0};
    mpu_reg_fifo_enable_t   reg_fifo_en         = {.ubyte = 0};
    mpu_reg_pwr_mgmt1_t     reg_pwr_mgmt1       = {.ubyte = 0};
    mpu_reg_pwr_mgmt2_t     reg_pwr_mgmt2       = {.ubyte = 0};
    uint8_t threshold , int_ubyte , MOT_ubyte , ODR_ubyte , MAG_CNTL_ubyte = 0x00;

    reg_pwr_mgmt2.bitfields.disabled_xg = 1;
    reg_pwr_mgmt2.bitfields.disabled_yg = 1;
    reg_pwr_mgmt2.bitfields.disabled_zg = 1;

    reg_accel_config.bitfields.accel_fs_sel = ACCEL_FULL_SCALE_8G;

    reg_accel_config2.bitfields.accel_fchoice_b = 0;
    reg_accel_config2.bitfields.accel_lpf_config = 0b001;
    threshold = map(400U , 0U , 1020U , 0U , 255U);
    int_ubyte = 0x40;
    MOT_ubyte = 0xC0;
    ODR_ubyte = 0x02;
    uint8_t c;
    nrf52_twi_read(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_ACCEL_CONFIG2 ,&c , sizeof(c));
    c = c & ~0x0F;
    c = c | 0x01;
    mpu9250_reset(p);
    
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_ACCEL_CONFIG2 , &c,sizeof(uint8_t));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_1, &reg_pwr_mgmt1.ubyte,sizeof(reg_pwr_mgmt1.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_2, &reg_pwr_mgmt2.ubyte,sizeof(reg_pwr_mgmt2.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_ACCEL_CONFIG2, &reg_accel_config2.ubyte,sizeof(reg_accel_config2.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_INT_ENABLE, &int_ubyte,sizeof(uint8_t));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_MOT_DETECT_CTRL, &MOT_ubyte,sizeof(uint8_t));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_WOM_THR, &threshold,sizeof(uint8_t));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_LP_ACCEL_ODR, &ODR_ubyte,sizeof(uint8_t));
    //reg_pwr_mgmt1.bitfields.cycle = 1;
    nrf52_twi_read(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_1 ,&c , sizeof(c));
    c = c | 0x20;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_1, &c,sizeof(c));
    nrf_delay_ms(10);
}

void mpu9250_accel_init(nrf_drv_twi_t const *p)
{
    mpu_reg_config_t        reg_config          = {.ubyte = 0};
    mpu_reg_accel_config_t  reg_accel_config    = {.ubyte = 0};
    mpu_reg_accel_config2_t reg_accel_config2   = {.ubyte = 0};
    mpu_reg_fifo_enable_t   reg_fifo_en         = {.ubyte = 0};
    mpu_reg_user_ctrl_t     reg_user_ctrl       = {.ubyte = 0};
    mpu_reg_pwr_mgmt1_t     reg_pwr_mgmt1       = {.ubyte = 0};
    mpu_reg_pwr_mgmt2_t     reg_pwr_mgmt2       = {.ubyte = 0};
    uint8_t reg=0;
    reg_config.bitfields.fifo_mode = FIFO_MODE_REPLACE_OLDEST_DATA_WHEN_FIFO_FULL;
    
    reg_accel_config.bitfields.accel_fs_sel = ACCEL_FULL_SCALE_8G;
    reg_accel_config2.bitfields.accel_fchoice_b = 0;
    reg_accel_config2.bitfields.accel_lpf_config = 1;

    
    reg_fifo_en.bitfields.accel = 1;
    reg_user_ctrl.bitfields.fifo_en = 1;
    reg_user_ctrl.bitfields.fifo_rst = 1;

    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_USER_CTRL, &reg_user_ctrl.ubyte , sizeof(reg_user_ctrl.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_1, &reg_pwr_mgmt1.ubyte,sizeof(reg_pwr_mgmt1.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_2, &reg_pwr_mgmt2.ubyte,sizeof(reg_pwr_mgmt2.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_CONFIG, &reg_config.ubyte , sizeof(reg_config.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_ACCEL_CONFIG, &reg_accel_config.ubyte , sizeof(reg_accel_config.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_ACCEL_CONFIG2, &reg_accel_config2.ubyte , sizeof(reg_accel_config2.ubyte));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_FIFO_EN, &reg_fifo_en.ubyte , sizeof(reg_fifo_en.ubyte));
    reg = 0;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_SMPLRT_DIV, &reg,sizeof(uint8_t));

    
}

void mpu9250_total_init(nrf_drv_twi_t const *p)
{
    uint8_t reg=0;

    reg = 0x01;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_1, &reg ,sizeof(uint8_t));
    reg = 0x00;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_2, &reg , sizeof(uint8_t));
    reg = 0x03;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_CONFIG, &reg , sizeof(uint8_t));
    reg = 0x00;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_GYRO_CONFIG , &reg , sizeof(uint8_t));
    reg = 0x08;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_ACCEL_CONFIG, &reg , sizeof(uint8_t));
    reg = 0x00;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_ACCEL_CONFIG2, &reg , sizeof(uint8_t));
    reg = 0x00;
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_FIFO_EN, &reg , sizeof(uint8_t));
    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_SMPLRT_DIV, &reg,sizeof(uint8_t));
}


uint8_t map(uint16_t in , uint16_t in_min , uint16_t in_max , uint16_t out_min , uint16_t out_max)
{
  return ((((uint16_t)in-in_min) * (out_max-out_min)) / (in_max-in_min)) + out_min;
}

void mpu9250_reset(nrf_drv_twi_t const *p)
{
    uint8_t temp = 0x80;

    nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_1, &temp , sizeof(uint8_t));
    nrf_delay_ms(100);
}

void mpu9250_shutdown(nrf_drv_twi_t const *p)
{
  uint8_t temp;
  temp = 0x3F;
  nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_2, &temp , sizeof(uint8_t));
  temp = 0x40;
  nrf52_twi_write(p, MPU_9250_SLAVE_ADDRESS, MPU_9250_REG_PWR_MGMT_1, &temp , sizeof(uint8_t));
  nrf_delay_ms(100);
}

void acc_cal(float *ax, float *ay, float *az, uint8_t *rx_buf)
{
    int16_t x, y, z;

    x = ((int16_t)(rx_buf[0]) << 8) | rx_buf[1];
    y = ((int16_t)(rx_buf[2]) << 8) | rx_buf[3];
    z = ((int16_t)(rx_buf[4]) << 8) | rx_buf[5];

    *ax = (float)((x)*8 / pow(2.0, 15.0));
    *ay = (float)((y)*8 / pow(2.0, 15.0));
    *az = (float)((z)*8 / pow(2.0, 15.0));
    // printf("%.2f\r\n", (float)((x) * 8 / pow(2.0,15.0)));
    // printf("%d %d %d, %.2f %.2f %.2f\r\n", x, y, z, *ax, *ay, *az);
}