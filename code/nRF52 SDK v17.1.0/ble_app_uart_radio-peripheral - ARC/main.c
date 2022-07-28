/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "radio_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_drv_clock.h"

#include "nrf_drv_rtc.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "mpu9250.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// added by boy, fft author: monkeyzx
#include "zx_fft.h"

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME "Nordic_UART_Boy"                    /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL 64 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION 18000 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)    /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY 0                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0
/* I2C pin for MPU9250 */
#define SDA_PIN NRF_GPIO_PIN_MAP(0, 13)
#define SCL_PIN NRF_GPIO_PIN_MAP(0, 24)

#define DEF_BUTTON_1 NRF_GPIO_PIN_MAP(0, 2)
#define DEF_BUTTON_2 NRF_GPIO_PIN_MAP(0, 30)
#define DEF_BUTTON_RESET NRF_GPIO_PIN_MAP(0, 18)
#define BLE_LED_1 NRF_GPIO_PIN_MAP(1, 11)
#define BLE_LED_2 NRF_GPIO_PIN_MAP(1, 10)
#define BLE_LED_3 NRF_GPIO_PIN_MAP(0, 03)
#define BLE_LED_4 NRF_GPIO_PIN_MAP(0, 28)
#define BLE_LED_5 NRF_GPIO_PIN_MAP(1, 13)
#define GPIO_NOTIFY_PIN NRF_GPIO_PIN_MAP(1, 02)

// Defined by Lin
#define ACC_BUF_SIZE 1024
#define TICK_MS 16.384
#define LIN_RX_PIN_NUMBER NRF_GPIO_PIN_MAP(1, 06)
#define LIN_TX_PIN_NUMBER NRF_GPIO_PIN_MAP(1, 04)

// variables for FFT
complex fft_x[SAMPLE_NODES];
complex fft_y[SAMPLE_NODES];
complex fft_z[SAMPLE_NODES];
int INPUT[SAMPLE_NODES];
int OUTPUT[SAMPLE_NODES];

typedef struct acc_raw
{
    uint8_t time[3];
    uint8_t ac[6];
} Acc_Raw;

typedef struct acc_processed
{
    float x;
    float y;
    float z;
} Acc_Proc;

Acc_Raw gait_raw[ACC_BUF_SIZE];
// Proc = processed ( = transformed uint8_t to float)
Acc_Proc gait_proc[ACC_BUF_SIZE];

static uint8_t mpu9250_register[6] = {
    (MPU_9250_REG_ACCEL_XOUT_H),
    (MPU_9250_REG_ACCEL_XOUT_L),
    (MPU_9250_REG_ACCEL_YOUT_H),
    (MPU_9250_REG_ACCEL_YOUT_L),
    (MPU_9250_REG_ACCEL_ZOUT_H),
    (MPU_9250_REG_ACCEL_ZOUT_L),
};

volatile int tick_ms = 0;

static volatile bool sample_start = false;
static volatile bool uart_send = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

const nrf_drv_rtc_t m_rtc = NRF_DRV_RTC_INSTANCE(2);
APP_TIMER_DEF(m_rtc_timer);
APP_TIMER_DEF(m_hint_timer);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t *p_event)
{
    //.....
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no = LIN_RX_PIN_NUMBER,
        .tx_pin_no = LIN_TX_PIN_NUMBER,
        .rts_pin_no = RTS_PIN_NUMBER,
        .cts_pin_no = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity = false,
#if defined(UART_PRESENT)
        .baud_rate = NRF_UART_BAUDRATE_115200
#else
        .baud_rate = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief I2C initialization.
 */
void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
        .scl = SCL_PIN,
        .sda = SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW,
        .clear_bus_init = false};

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void rtc_handler(void *p_context)
{
    // uint32_t tick_ms = app_timer_cnt_get()/TICK_MS;
    // printf("tick:%d ,%d\r\n",tick_ms, app_timer_cnt_get());
}

static void startRecordHint(void *p_context)
{
    sample_start = true;

    nrf_gpio_pin_clear(BLE_LED_1);
    nrf_gpio_pin_clear(BLE_LED_2);
    nrf_gpio_pin_set(BLE_LED_3);
    nrf_gpio_pin_clear(BLE_LED_4);
    nrf_gpio_pin_clear(BLE_LED_5);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   pin   GPIO PIN NUMBER
 * @param[in]   action   Check GPIOTE_POLARITY.
 */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == NRF_GPIOTE_POLARITY_HITOLO)
    {
        switch (pin)
        {
        case DEF_BUTTON_1:
            // printf("Button 1 pressed!\r\n");
            app_timer_start(m_hint_timer, APP_TIMER_TICKS(15000), NULL);

            nrf_gpio_pin_clear(BLE_LED_1);
            nrf_gpio_pin_set(BLE_LED_2);
            nrf_gpio_pin_clear(BLE_LED_3);
            nrf_gpio_pin_clear(BLE_LED_4);
            nrf_gpio_pin_clear(BLE_LED_5);

            break;

        case DEF_BUTTON_2:
            // printf("Button 2 pressed!\r\n");
            nrf_gpio_pin_clear(BLE_LED_1);
            nrf_gpio_pin_clear(BLE_LED_2);
            nrf_gpio_pin_clear(BLE_LED_3);
            nrf_gpio_pin_clear(BLE_LED_4);
            nrf_gpio_pin_set(BLE_LED_5);
            uart_send = true;
            break;

        default:
            // printf("Something wrong...\n");
            break;
        }
    }
}

/**@brief Function for initializing GPIO.
 */
void GPIO_init(void)
{
    nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false); // Set high-to-low event
    in_config.pull = NRF_GPIO_PIN_PULLUP;                                             // gpio initial pull-up

    nrf_drv_gpiote_in_init(DEF_BUTTON_1, &in_config, in_pin_handler); // initial gpio pin
    nrf_drv_gpiote_in_event_enable(DEF_BUTTON_1, true);               // enable gpio pin interrupt
    nrf_drv_gpiote_in_init(DEF_BUTTON_2, &in_config, in_pin_handler); // initial gpio pin
    nrf_drv_gpiote_in_event_enable(DEF_BUTTON_2, true);               // enable gpio pin interrupt

    nrf_gpio_cfg_output(BLE_LED_1);
    nrf_gpio_cfg_output(BLE_LED_2);
    nrf_gpio_cfg_output(BLE_LED_3);
    nrf_gpio_cfg_output(BLE_LED_4);
    nrf_gpio_cfg_output(BLE_LED_5);
    nrf_gpio_cfg_output(GPIO_NOTIFY_PIN);

    nrf_gpio_pin_clear(BLE_LED_1);
    nrf_gpio_pin_clear(BLE_LED_2);
    nrf_gpio_pin_clear(BLE_LED_3);
    nrf_gpio_pin_clear(BLE_LED_4);
    nrf_gpio_pin_clear(BLE_LED_5);
    nrf_gpio_pin_set(GPIO_NOTIFY_PIN);
}

void time_type_trans(uint32_t time_32, uint8_t *time)
{
    uint32_t t;
    t = time_32;
    time[0] = t & 255;
    t = t >> 8;
    time[1] = t & 255;
    t = t >> 8;
    time[2] = t & 255;
    // t = ((uint32_t)(time[2])<<16) | ((uint32_t)(time[1])<<8) | time[0];
    // printf("t32: %d, t: %d\r\n",time_32, t);
}

/**@brief Function for request low frequency clock.
 */
static void lfclk_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    if (err_code == NRF_SUCCESS)
        nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Function for initialization oscillators.
 */
void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    // printf("Wait for the external oscillator to start up\r\n");
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}

void timers_create()
{
    // timer to count ticks, not used now
    ret_code_t err_code = app_timer_create(&m_rtc_timer, APP_TIMER_MODE_REPEATED, rtc_handler);
    APP_ERROR_CHECK(err_code);

    // timer to stop collect from MPU920
    err_code = app_timer_create(&m_hint_timer, APP_TIMER_MODE_SINGLE_SHOT, startRecordHint);
    APP_ERROR_CHECK(err_code);
}

static void setup(void)
{
    twi_init();
    uart_init();
    GPIO_init();
    timers_init();
    timers_create();
    clock_initialization();

    mpu9250_reset(&m_twi);
    mpu9250_total_init(&m_twi);
    mpu9250_accel_init(&m_twi);

    power_management_init();
}

static void makeInput(Acc_Proc *acc)
{
    for (int i = 0; i < ACC_BUF_SIZE; i++)
    {
        fft_x[i].real = acc[i].x;
        fft_x[i].img = 0.0f;

        fft_y[i].real = acc[i].y;
        fft_y[i].img = 0.0f;

        fft_z[i].real = acc[i].z;
        fft_z[i].img = 0.0f;
    }
}

static void makeOutput(complex *fft_acc)
{
    for (int i = 0; i < ACC_BUF_SIZE; i++)
    {
        OUTPUT[i] = sqrt(fft_x[i].real * fft_x[i].real + fft_x[i].img * fft_x[i].img);
        // OUTPUT[i] = sqrt(fft_y[i].real * fft_y[i].real + fft_y[i].img * fft_y[i].img);
        // OUTPUT[i] = sqrt(fft_z[i].real * fft_z[i].real + fft_z[i].img * fft_z[i].img);
    }
}

int main(void)
{
    uint32_t count = 0;

    setup();
    nrf_gpio_pin_set(BLE_LED_1);

    //printf("<--Peripheral-->: Basic config finished. Gait pod project start!\r\n");
    //app_timer_start(m_rtc_timer, APP_TIMER_TICKS(100000), NULL);

    while (true)
    {
		//uint32_t time_32 = app_timer_cnt_get() / TICK_MS;
		//printf("t %d\r\n", time_32);
        nrf_delay_ms(1);
        if (!sample_start)
            continue;

        for (int i = 0; i < 6; i++)
        {
            nrf52_twi_read(&m_twi, MPU_9250_SLAVE_ADDRESS, mpu9250_register[i], gait_raw[count].ac + i, sizeof(gait_raw[count].ac[i]));
            nrf_delay_us(40);
        }
        acc_cal(&gait_proc[count].x, &gait_proc[count].y, &gait_proc[count].z, gait_raw[count].ac);

        count += 1;
        if (count == ACC_BUF_SIZE)
        {
            sample_start = false;
            count = 0;

            nrf_gpio_pin_clear(BLE_LED_1);
            nrf_gpio_pin_clear(BLE_LED_2);
            nrf_gpio_pin_clear(BLE_LED_3);
            nrf_gpio_pin_set(BLE_LED_4);
            nrf_gpio_pin_clear(BLE_LED_5);

			resend:
            while (uart_send == false)
                ;
			app_uart_put(0xAA);
            for (uint16_t j = 0; j < ACC_BUF_SIZE; j++)
            {
				uint8_t idx_arr[2];
				memcpy(idx_arr, &j, 2);
				app_uart_put(idx_arr[0]);
				app_uart_put(idx_arr[1]);

				app_uart_put(gait_raw[j].ac[0]);
		        app_uart_put(gait_raw[j].ac[1]);
		        app_uart_put(gait_raw[j].ac[2]);
		        app_uart_put(gait_raw[j].ac[3]);
		        app_uart_put(gait_raw[j].ac[4]);
		        app_uart_put(gait_raw[j].ac[5]);
                //printf("%d %.2f %.2f %.2fx", j, gait_proc[j].x, gait_proc[j].y, gait_proc[j].z);
                nrf_delay_ms(10);
            }

            nrf_gpio_pin_set(BLE_LED_1);
            nrf_gpio_pin_set(BLE_LED_2);
            nrf_gpio_pin_set(BLE_LED_3);
            nrf_gpio_pin_set(BLE_LED_4);
            nrf_gpio_pin_set(BLE_LED_5);
            
			nrf_delay_ms(100);
            nrf_gpio_pin_clear(GPIO_NOTIFY_PIN);
			
			nrf_delay_ms(2000);
			nrf_gpio_pin_set(GPIO_NOTIFY_PIN);
			uart_send = false;
			goto resend;

        }
    }
}

/**
 * @}
 */
