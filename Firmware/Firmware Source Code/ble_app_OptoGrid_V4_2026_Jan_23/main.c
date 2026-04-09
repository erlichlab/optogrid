/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
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

/** @brief GATT Service server example application main file with DFU support.

    @details This file contains the main source code for OptoGrid application that includes:
             - GATT Server Peripheral functionality for nRF52832
             - PWM control for LED amplitude modulation
             - Ramp up/down functionality for smooth amplitude transitions
             - Secure DFU Buttonless Service for firmware updates
             - I2C device support (IMU + MAG sensors)
*/
// Include headers for used modules
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_util.h"
#include "app_error.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_pwm.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "fds.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "nrfx_wdt.h"

// Direct Firmware Update (DFU) includes
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"

#define APP_BLE_CONN_CFG_TAG      1
#define APP_BLE_OBSERVER_PRIO     3
#define APP_SOC_OBSERVER_PRIO     1
#define DEVICE_NAME                     "JON-O-0001"                                /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                0                                           /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// LED Control Pin Definitions
#define STATUS_LED_PIN      10
#define SHAM_LED_PIN        9
#define STIM_LED_ENABLE_PIN 12

// Charlie-plexing pins for 64 LED array
#define AX0_PIN             7  // Column select bit 0
#define AX1_PIN             8   // Column select bit 1  
#define AX2_PIN             6   // Column select bit 2
#define AY0_PIN             3   // Row select bit 0
#define AY1_PIN             2   // Row select bit 1
#define AY2_PIN             30  // Row select bit 2


#define LOG_BUFFER_SIZE 72
#define MAX_LOG_LENGTH 128

// Track global time
static uint32_t global_time = 0; // Tracks time since power-on in milliseconds
APP_TIMER_DEF(global_time_timer); // Timer instance
static void global_time_timer_handler(void *p_context);

// Battery ADC pin
#define VBAT_ADC             NRF_SAADC_INPUT_AIN3 // ADC pin for battery voltage measurement, P05/AIN3
static uint16_t read_battery_voltage_mv(void);
static uint64_t read_uLED_check(void);

#define LED_CURRENT_ADC      NRF_SAADC_INPUT_AIN2 // ADC pin for LED current measurement, P04/AIN2

//Mapping to remap AX values to uLED array index
static uint8_t AX_map[8] = {3,7,6,5,2,4,1,0};
static uint8_t AY_map[8] = {5,6,7,4,3,1,0,2};

NRF_BLE_GATT_DEF(m_gatt);
BLE_ADVERTISING_DEF(m_advertising);

// Single timer instance for all timing operations
const nrf_drv_timer_t TIMER_OPTO = NRF_DRV_TIMER_INSTANCE(1);

// PWM instance for LED amplitude control
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

/* I2C IMU + MAG definition */
// TWI/I2C Configuration
#define TWI_SCL_PIN                 28   // Define SCL pin
#define TWI_SDA_PIN                 16   // Define SDA pin
#define TWI_INSTANCE_ID             0


// IIM-42652 IMU Configuration
#define IIM42652_I2C_ADDR          0x68   // 7-bit I2C address (AD0=0)
#define IIM42652_WHO_AM_I_REG      0x75   // WHO_AM_I register

// Add these new definitions in the IMU configuration section
#define IMU_DATA_TIMER_INTERVAL    APP_TIMER_TICKS(1000/imu_sample_rate)  // 100Hz = ~100ms interval
#define IIM42652_GYRO_DATA_REG    0x25    // Gyro data registers
#define IIM42652_ACCEL_DATA_REG   0x1F    // Accel data registers
#define IIM42652_PWR_MGMT0_REG    0x4E    // Power management register
#define IIM42652_GYRO_CONFIG0_REG 0x4F    // Gyro configuration register
#define IIM42652_ACCEL_CONFIG0_REG 0x50   // Accel configuration register

// LIS2MDLTR Magnetometer Configuration
#define LIS2MDLTR_I2C_ADDR      0x1E   // 7-bit I2C address for LIS2MDLTR
#define LIS2MDLTR_OUTX_L_REG    0x68   // Output X low byte register
#define LIS2MDLTR_OUTY_L_REG    0x6A   // Output Y low byte register
#define LIS2MDLTR_OUTZ_L_REG    0x6C   // Output Z low byte register
#define LIS2MDLTR_WHO_AM_I_REG  0x4F   // WHO_AM_I register
#define LIS2MDLTR_WHO_AM_I_VAL  0x40   // Expected value for LIS2MDLTR


// IMU buffer
#define IMU_DATA_BUFFER_SIZE 32
static int16_t imu_data_buffer[IMU_DATA_BUFFER_SIZE][11];
static volatile uint16_t imu_data_buf_head = 0;
static volatile uint16_t imu_data_buf_tail = 0;


// TWI instance
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Global variables
static bool m_twi_initialized = false;

// Add new timer definition after other APP_TIMER_DEFs
APP_TIMER_DEF(m_imu_timer_id);

// Add new characteristic handles in the handle section
static ble_gatts_char_handles_t m_imu_data_char_handles;
static ble_gatts_char_handles_t m_imu_enable_char_handles;
static ble_gatts_char_handles_t m_imu_sample_rate_char_handles;

// Add these new function prototypes in the forward declarations section
static void imu_timer_handler(void * p_context);
static ret_code_t imu_read_data(void);
static ret_code_t imu_configure(void);
static ret_code_t mag_configure(void);
static void send_next_imu_data(void);
// Temporarily unused, for future use
static ret_code_t i2c_devices_init(void);
static ret_code_t imu_init(void);

// Add new control variables
static bool imu_streaming = false;
static uint32_t imu_samplestamp = 0;

#define I2C_TRANSFER_TIMEOUT_MS 500

// Watchdog configuration
#define WDT_TIMEOUT_MS 5000  // 5 second timeout
static nrfx_wdt_channel_id m_channel_id;

/* End of I2C IMU+MAG definition */


static void advertising_start(bool erase_bonds);

// Stimulation phases
typedef enum {
    STIM_PHASE_INIT,
    STIM_PHASE_RAMP_UP,
    STIM_PHASE_MAIN,
    STIM_PHASE_RAMP_DOWN,
    STIM_PHASE_COMPLETE
} stimulation_phase_t;

// Forward declarations - declare all functions used before they're defined
static void start_next_led(void);
static void start_next_period(void);
static void start_next_sequence(void);
static void start_sequence(uint8_t sequence_index);
static void start_next_phase(void);
static void start_phase(stimulation_phase_t phase);
static void stimulation_complete(void);
static void extract_selected_leds(uint64_t led_selection, volatile uint8_t *led_array, volatile uint8_t *count);
static void timer_event_handler(nrf_timer_event_t event_type, void* p_context);
static uint8_t calculate_ramp_amplitude(uint8_t sequence_index, uint32_t phase_elapsed_us);
static void update_ramp_amplitude(void);
static void trigger_stimulation(void);

// PWM control functions
static void pwm_init(void);
static void pwm_configure_for_sequence(uint8_t sequence_index);
static void pwm_enable(void);
static void pwm_disable(void);

// DFU forward declarations
static void disconnect(uint16_t conn_handle, void * p_context);

// Watchdog forward declarations
static void wdt_event_handler(void);
static void wdt_init(void);


// ===== Custom GATT Services Implementation (embedded directly) =====

#define UUID_DEVICE_INFO_SERVICE         0x1400
#define UUID_OPTO_CONTROL_SERVICE        0x1401
#define UUID_DATA_STREAMING_SERVICE      0x1402

#define UUID_DEVICE_ID_CHAR              0x1500
#define UUID_FIRMWARE_VERSION_CHAR       0x1501
#define UUID_ULED_COLOR_CHAR             0x1503
#define UUID_ULED_CHECK_CHAR             0x1504
#define UUID_BATTERY_VOLTAGE_CHAR        0x1506
#define UUID_STATUS_LED_CHAR             0x1507
#define UUID_SHAM_LED_CHAR               0x1508
#define UUID_DEVICE_LOG_CHAR             0x1509
#define UUID_LAST_STIM_TIME              0x150A

#define UUID_SEQUENCE_LENGTH_CHAR        0x1600
#define UUID_LED_SELECTION_CHAR          0x1601
#define UUID_DURATION_CHAR               0x1602
#define UUID_PERIOD_CHAR                 0x1603
#define UUID_PULSE_WIDTH_CHAR            0x1604
#define UUID_AMPLITUDE_CHAR              0x1605
#define UUID_PWM_FREQ_CHAR               0x1606
#define UUID_RAMP_UP_TIME_CHAR           0x1607
#define UUID_RAMP_DOWN_TIME_CHAR         0x1608
#define UUID_TRIGGER_CHAR                0x1609

#define UUID_IMU_ENABLE_CHAR             0x1700
#define UUID_IMU_SAMPLE_RATE_CHAR        0x1701
#define UUID_IMU_RESOLUTION_CHAR         0x1702
#define UUID_IMU_DATA_CHAR               0x1703
#define MAX_SEQUENCE_LENGTH 10           // Maximum number of sequences supported

// Custom base UUID for OptoGrid services: 12345678-1234-5678-1234-56789abcdef0
static uint8_t m_base_uuid[] = { 0xf0, 0xef, 0xcd, 0xab, 0x78, 0x56, 0x34, 0x12,
                                 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56 };
static uint8_t m_uuid_type = BLE_UUID_TYPE_VENDOR_BEGIN;

static uint8_t device_id[] = DEVICE_NAME;
static uint8_t firmware_version[] = "2026-Jan-23";
static uint8_t uled_color[] = "blue";
static uint64_t uled_check = 0;
static uint16_t battery_voltage = 0; // Battery voltage in millivolts
static uint8_t status_led_state = 1; // 0 = off, 1 = on
static uint8_t sham_led_state = 0;   // 0 = off, 1 = on
static uint32_t last_stim_time_ms = 0;
static uint8_t sequence_length = 1; //units -- determines array size
static uint64_t led_selection[MAX_SEQUENCE_LENGTH] = {0xFFFFFFFFFFFFFFFF}; // Array of LED selections
static uint16_t duration[MAX_SEQUENCE_LENGTH] = {550}; // Array of durations
static uint16_t period[MAX_SEQUENCE_LENGTH] = {100}; // Array of periods
static uint16_t pulse_width[MAX_SEQUENCE_LENGTH] = {1}; // Array of pulse widths
static uint8_t amplitude[MAX_SEQUENCE_LENGTH] = {100}; // Array of amplitudes
static uint32_t pwm_freq[MAX_SEQUENCE_LENGTH] = {50000}; // Array of PWM frequencies
static uint16_t ramp_up_time[MAX_SEQUENCE_LENGTH] = {0}; // Array of ramp up times
static uint16_t ramp_down_time[MAX_SEQUENCE_LENGTH] = {0}; // Array of ramp down times
static uint8_t trigger = 0; //bool, write 1 is trigger
static uint8_t imu_enable = 0; //bool, 1 is enable
static uint8_t imu_sample_rate = 100; //Hz 
static uint8_t imu_resolution = 0; //
static int16_t imu_data[11] = {0};

static struct {
    char messages[LOG_BUFFER_SIZE][MAX_LOG_LENGTH];
    uint8_t count;
} log_buffer = {0};

//A struct to be passed into trigger stimulation function 
typedef struct {
    uint8_t sequence_length;
    uint64_t led_selection[MAX_SEQUENCE_LENGTH];
    uint16_t duration[MAX_SEQUENCE_LENGTH];
    uint16_t period[MAX_SEQUENCE_LENGTH];
    uint16_t pulse_width[MAX_SEQUENCE_LENGTH];
    uint8_t amplitude[MAX_SEQUENCE_LENGTH];
    uint32_t pwm_freq[MAX_SEQUENCE_LENGTH];
    uint16_t ramp_up_time[MAX_SEQUENCE_LENGTH];
    uint16_t ramp_down_time[MAX_SEQUENCE_LENGTH];
} opto_params_t;

// Enhanced stimulation state tracking with ramp phases
typedef struct {
    bool active;
    uint8_t current_sequence;           // Which sequence we're executing (0 to sequence_length-1)
    stimulation_phase_t current_phase;  // Current phase of stimulation
    uint32_t sequence_start_time_ms;    // When current sequence started
    uint32_t phase_start_time_ms;       // When current phase started
    uint32_t period_start_time_ms;      // When current period started
    uint8_t selected_leds[64];          // Array of LED indices that are selected for current sequence
    uint8_t num_selected_leds;          // Number of LEDs selected for current sequence
    uint8_t current_led_index;          // Which LED in the selected array we're currently activating
    bool led_currently_on;              // Is an LED currently turned on
    uint32_t current_period_count;      // Track which period we're in within current sequence
    
    // Ramp control
    uint8_t target_amplitude;           // Target amplitude for current sequence
    uint8_t current_amplitude;          // Current ramped amplitude
} stimulation_state_t;

static volatile stimulation_state_t stim_state = {0};
static opto_params_t current_stim_params;

// PWM state tracking
typedef struct {
    bool initialized;
    bool enabled;
    uint16_t current_duty_cycle;        // Current duty cycle value (0-32767)
    uint32_t current_frequency_hz;      // Current PWM frequency in Hz
} pwm_state_t;

static volatile pwm_state_t pwm_state = {0};

// PWM sequence data - must be in RAM for DMA
static nrf_pwm_values_individual_t m_pwm_values;
static nrf_pwm_sequence_t const m_pwm_sequence = {
    .values.p_individual = &m_pwm_values,
    .length = NRF_PWM_VALUES_LENGTH(m_pwm_values),
    .repeats = 0,
    .end_delay = 0
};


//Handle to store the trigger characteristic and other parameter characteristics
static ble_gatts_char_handles_t m_trigger_char_handles;
static ble_gatts_char_handles_t m_sequence_length_char_handles;
static ble_gatts_char_handles_t m_led_selection_char_handles;
static ble_gatts_char_handles_t m_duration_char_handles;
static ble_gatts_char_handles_t m_period_char_handles;
static ble_gatts_char_handles_t m_pulse_width_char_handles;
static ble_gatts_char_handles_t m_amplitude_char_handles;
static ble_gatts_char_handles_t m_pwm_freq_char_handles;
static ble_gatts_char_handles_t m_ramp_up_time_char_handles;
static ble_gatts_char_handles_t m_ramp_down_time_char_handles;
static ble_gatts_char_handles_t m_battery_voltage_char_handles;
static ble_gatts_char_handles_t m_uled_check_char_handles;
static ble_gatts_char_handles_t m_last_stim_time_char_handles; // Handle for the characteristic

// Create a lookup table for handle-to-parameter mapping
typedef struct {
    uint16_t *handle_ptr;
    void *data_ptr;
    size_t data_size;
    const char *name;
} gatt_param_map_t;

// Initialize the lookup table
static gatt_param_map_t gatt_param_map[] = {
    {&m_sequence_length_char_handles.value_handle, &sequence_length, sizeof(sequence_length), "sequence_length"},
    {&m_led_selection_char_handles.value_handle, led_selection, sizeof(led_selection), "led_selection"},
    {&m_duration_char_handles.value_handle, duration, sizeof(duration), "duration"},
    {&m_period_char_handles.value_handle, period, sizeof(period), "period"},
    {&m_pulse_width_char_handles.value_handle, pulse_width, sizeof(pulse_width), "pulse_width"},
    {&m_amplitude_char_handles.value_handle, amplitude, sizeof(amplitude), "amplitude"},
    {&m_pwm_freq_char_handles.value_handle, pwm_freq, sizeof(pwm_freq), "pwm_freq"},
    {&m_ramp_up_time_char_handles.value_handle, ramp_up_time, sizeof(ramp_up_time), "ramp_up_time"},
    {&m_ramp_down_time_char_handles.value_handle, ramp_down_time, sizeof(ramp_down_time), "ramp_down_time"},
};


static ble_gatts_char_handles_t m_status_led_char_handles;
static ble_gatts_char_handles_t m_sham_led_char_handles;
static ble_gatts_char_handles_t m_device_log_char_handles;

static void ble_log(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    // If buffer is full, drop the oldest message (shift all up)
    if (log_buffer.count >= LOG_BUFFER_SIZE) {
        for (uint8_t i = 1; i < LOG_BUFFER_SIZE; i++) {
            memcpy(log_buffer.messages[i - 1], log_buffer.messages[i], MAX_LOG_LENGTH);
        }
        log_buffer.count = LOG_BUFFER_SIZE - 1;
    }

    // Write new message at the end
    memset(log_buffer.messages[log_buffer.count], 0, MAX_LOG_LENGTH);
    int written = vsnprintf(
        log_buffer.messages[log_buffer.count],
        MAX_LOG_LENGTH,
        format,
        args);

    // Ensure null-termination
    if (written < 0 || written >= MAX_LOG_LENGTH) {
        log_buffer.messages[log_buffer.count][MAX_LOG_LENGTH - 1] = '\0';
    } else {
        log_buffer.messages[log_buffer.count][written] = '\0';
    }

    log_buffer.count++;

    va_end(args);
}

#define GATT_PARAM_MAP_SIZE (sizeof(gatt_param_map) / sizeof(gatt_param_map[0]))

// ===== DFU IMPLEMENTATION START =====

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // Stop any ongoing stimulation before DFU
            if (stim_state.active) {
                stimulation_complete();
            }
            break;

        default:
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Stop any ongoing stimulation
            if (stim_state.active) {
                stimulation_complete();
            }

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            memset(&config, 0, sizeof(ble_adv_modes_config_t));
            config.ble_adv_fast_enabled  = true;
            config.ble_adv_fast_interval = APP_ADV_INTERVAL;
            config.ble_adv_fast_timeout  = APP_ADV_DURATION;
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

// ===== DFU IMPLEMENTATION END =====

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


static void trigger_stimulation(void);

/**@brief Calculate current amplitude based on ramp phase and timing
 */
static uint8_t calculate_ramp_amplitude(uint8_t sequence_index, uint32_t phase_elapsed_us)
{
    uint32_t ramp_duration_us;
    uint8_t target_amp = current_stim_params.amplitude[sequence_index];
    
    switch (stim_state.current_phase) {
        case STIM_PHASE_RAMP_UP:
            ramp_duration_us = current_stim_params.ramp_up_time[sequence_index] * 1000;
            if (ramp_duration_us == 0) return target_amp; // No ramp up
            
            // Linear ramp from 0 to target_amplitude
            if (phase_elapsed_us >= ramp_duration_us) {
                return target_amp;
            }
            return (uint8_t)((target_amp * phase_elapsed_us) / ramp_duration_us);
            
        case STIM_PHASE_MAIN:
            return target_amp;
            
        case STIM_PHASE_RAMP_DOWN:
            ramp_duration_us = current_stim_params.ramp_down_time[sequence_index] * 1000;
            if (ramp_duration_us == 0) return 0; // No ramp down, just turn off
            
            // Linear ramp from target_amplitude to 0
            if (phase_elapsed_us >= ramp_duration_us) {
                return 0;
            }
            return target_amp - (uint8_t)((target_amp * phase_elapsed_us) / ramp_duration_us);
            
        default:
            return 0;
    }
}

/**@brief Update PWM amplitude during ramp phases
 */
static void update_ramp_amplitude(void)
{
    if (!stim_state.active) return;
    
    // Calculate elapsed time in current phase
    uint32_t current_timer_count = nrf_drv_timer_capture(&TIMER_OPTO, NRF_TIMER_CC_CHANNEL0);
    uint32_t phase_start_ticks = nrf_drv_timer_us_to_ticks(&TIMER_OPTO, stim_state.phase_start_time_ms * 1000);
    uint32_t phase_elapsed_ticks = current_timer_count - phase_start_ticks;
    // Convert ticks to microseconds manually (timer runs at 1MHz, so 1 tick = 1 microsecond)
    uint32_t phase_elapsed_us = phase_elapsed_ticks;
    
    // Calculate new amplitude
    uint8_t new_amplitude = calculate_ramp_amplitude(stim_state.current_sequence, phase_elapsed_us);
    
    if (new_amplitude != stim_state.current_amplitude) {
        stim_state.current_amplitude = new_amplitude;
        
        // Update PWM duty cycle if amplitude changed
        if (pwm_state.initialized) {
            // Get current TOP value from PWM configuration
            uint32_t top_value = 16000000UL / pwm_state.current_frequency_hz;
            if (top_value < 3) top_value = 3;
            if (top_value > 32767) top_value = 32767;
            
            // Calculate new duty cycle
            uint16_t duty_cycle_value = (new_amplitude * top_value) / 100;
            
            // Update PWM values
            pwm_state.current_duty_cycle = duty_cycle_value;
            // Apply the PWM duty cycle 
            m_pwm_values.channel_0 = pwm_state.current_duty_cycle | 0x8000;
            
            // If PWM is currently enabled, the new values will take effect on next period
            // NRF_LOG_DEBUG("Ramp amplitude updated: %d%% (duty=%d)", new_amplitude, duty_cycle_value);
        }
    }
    
}

/**@brief Initialize PWM for LED amplitude control
 */
static void pwm_init(void)
{
    ret_code_t err_code;
    
    nrf_drv_pwm_config_t const config = {
        .output_pins = {
            STIM_LED_ENABLE_PIN,        // Channel 0 - STIM_LED_ENABLE pin
            NRF_DRV_PWM_PIN_NOT_USED,   // Channel 1 - not used
            NRF_DRV_PWM_PIN_NOT_USED,   // Channel 2 - not used  
            NRF_DRV_PWM_PIN_NOT_USED,   // Channel 3 - not used
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock = NRF_PWM_CLK_16MHz,    // 16 MHz base clock
        .count_mode = NRF_PWM_MODE_UP,
        .top_value = 320,                   // Default for 50kHz (16MHz/320 = 50kHz)
        .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode = NRF_PWM_STEP_AUTO
    };
    
    err_code = nrf_drv_pwm_init(&m_pwm0, &config, NULL);
    APP_ERROR_CHECK(err_code);
    
    // Explicitly stop PWM after initialization
    nrf_drv_pwm_stop(&m_pwm0, true);  // 

    // Initialize PWM values to 0 (off)
    m_pwm_values.channel_0 = 0 | 0x8000;
    m_pwm_values.channel_1 = 0;
    m_pwm_values.channel_2 = 0;
    m_pwm_values.channel_3 = 0;
    
    pwm_state.initialized = true;
    pwm_state.enabled = false;
    pwm_state.current_duty_cycle = 0;
    pwm_state.current_frequency_hz = 50000; // Default 50kHz
    
    // NRF_LOG_INFO("PWM initialized for LED amplitude control");
}

/**@brief Configure PWM frequency and duty cycle for current sequence
 */
static void pwm_configure_for_sequence(uint8_t sequence_index)
{
    if (!pwm_state.initialized || sequence_index >= MAX_SEQUENCE_LENGTH) {
        return;
    }
    
    uint32_t target_freq_hz = current_stim_params.pwm_freq[sequence_index];
    
    // Ensure frequency is within reasonable bounds (1kHz to 1MHz)
    if (target_freq_hz < 1000) target_freq_hz = 1000;
    if (target_freq_hz > 1000000) target_freq_hz = 1000000;
    
    // Calculate TOP value for desired frequency
    // TOP = base_clock / frequency
    uint32_t top_value = 16000000UL / target_freq_hz;
    
    // Ensure TOP value is within PWM module limits (3 to 32767)
    if (top_value < 3) top_value = 3;
    if (top_value > 32767) top_value = 32767;
    
    // Reconfigure PWM if frequency changed
    if (target_freq_hz != pwm_state.current_frequency_hz) {
        // Stop PWM
        nrf_drv_pwm_stop(&m_pwm0, false);
        pwm_state.enabled = false;
        
        // Uninitialize and reinitialize with new TOP value
        nrf_drv_pwm_uninit(&m_pwm0);
        
        nrf_drv_pwm_config_t config = {
            .output_pins = {
                STIM_LED_ENABLE_PIN,
                NRF_DRV_PWM_PIN_NOT_USED,
                NRF_DRV_PWM_PIN_NOT_USED,
                NRF_DRV_PWM_PIN_NOT_USED,
            },
            .irq_priority = APP_IRQ_PRIORITY_LOWEST,
            .base_clock = NRF_PWM_CLK_16MHz,
            .count_mode = NRF_PWM_MODE_UP,
            .top_value = top_value,
            .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
            .step_mode = NRF_PWM_STEP_AUTO
        };
        
        ret_code_t err_code = nrf_drv_pwm_init(&m_pwm0, &config, NULL);
        APP_ERROR_CHECK(err_code);
        
        pwm_state.current_frequency_hz = target_freq_hz;
        
        // NRF_LOG_DEBUG("PWM reconfigured: %d Hz, TOP=%d", target_freq_hz, top_value);
    }
    
    // Initialize with 0% duty cycle (will be updated by ramp)
    pwm_state.current_duty_cycle = 0;
    // Apply the PWM duty cycle 
    m_pwm_values.channel_0 = pwm_state.current_duty_cycle | 0x8000;
    pwm_state.enabled = false;
    
    // NRF_LOG_DEBUG("PWM configured for sequence %d: %d Hz, starting at 0%% duty cycle", 
    //               sequence_index + 1, target_freq_hz);
}

/**@brief Enable PWM output
 */
static void pwm_enable(void)
{
    if (!pwm_state.initialized) {
        return;
    }
    
    // if (!pwm_state.enabled) {

    // Start PWM playback
    ret_code_t err_code = nrf_drv_pwm_simple_playback(&m_pwm0, &m_pwm_sequence, 1, NRF_DRV_PWM_FLAG_LOOP);
    APP_ERROR_CHECK(err_code);
    
    pwm_state.enabled = true;
    NRF_LOG_DEBUG("PWM state: %d", pwm_state.enabled);
    // NRF_LOG_DEBUG("PWM enabled");
    // }
}

/**@brief Disable PWM output (set output low)
 */
static void pwm_disable(void)
{
    if (!pwm_state.initialized) {
        return;
    }
    
    // Only stop PWM if this is the last LED in the current period
    if (stim_state.current_led_index + 1 < stim_state.num_selected_leds) {
        return; // Don't stop PWM, more LEDs coming in this period
    }
    // if (pwm_state.enabled) {
    // Stop PWM - this will set output low
    nrf_drv_pwm_stop(&m_pwm0, false);
    pwm_state.enabled = false;
    NRF_LOG_DEBUG("PWM state: %d", pwm_state.enabled);
    // NRF_LOG_DEBUG("PWM disabled");
    // }
}

/**@brief Initialize all LED control pins
 */
static void led_control_init(void)
{
    // Configure TOP LED pins as outputs
    nrf_gpio_cfg_output(STATUS_LED_PIN);
    nrf_gpio_cfg_output(SHAM_LED_PIN);
    // Note: STIM_LED_ENABLE_PIN is now configured by PWM module
    
    // Configure Charlie-plexing pins as outputs
    nrf_gpio_cfg_output(AX0_PIN);
    nrf_gpio_cfg_output(AX1_PIN);
    nrf_gpio_cfg_output(AX2_PIN);
    nrf_gpio_cfg_output(AY0_PIN);
    nrf_gpio_cfg_output(AY1_PIN);
    nrf_gpio_cfg_output(AY2_PIN);
    
    // Initialize all LEDs to OFF state
    if(status_led_state){
        nrf_gpio_pin_set(STATUS_LED_PIN);
    } else {
        nrf_gpio_pin_clear(STATUS_LED_PIN);
    }
    if(sham_led_state){
        nrf_gpio_pin_set(SHAM_LED_PIN);
    } else {
        nrf_gpio_pin_clear(SHAM_LED_PIN);
    }
    // STIM_LED_ENABLE_PIN controlled by PWM - will be low when PWM is disabled
    nrf_gpio_pin_clear(AX0_PIN);
    nrf_gpio_pin_clear(AX1_PIN);
    nrf_gpio_pin_set(AX2_PIN);
    nrf_gpio_pin_clear(AY0_PIN);
    nrf_gpio_pin_clear(AY1_PIN);
    nrf_gpio_pin_set(AY2_PIN);
    
    // NRF_LOG_INFO("LED control pins initialized");
}

/**@brief Convert LED bit position to X,Y coordinates and then to AX,AY pin values
 * @param led_bit_position: 0-63, represents bit position in 64-bit led_selection
 * @param ax_pins: output array [AX0, AX1, AX2] pin values
 * @param ay_pins: output array [AY0, AY1, AY2] pin values
 */
static void led_bit_to_pins(uint8_t led_bit_position, uint8_t *ax_pins, uint8_t *ay_pins)
{
    // Convert bit position to X,Y coordinates (1-8 range)
    uint8_t x = (led_bit_position % 8) + 1;  // X: 1-8
    uint8_t y = (led_bit_position / 8) + 1;  // Y: 1-8
    
    // Convert X,Y to 3-bit values (0 represents 1, 1 represents 2, etc.)
    uint8_t X_index = x - 1;  // 0-7
    uint8_t Y_index = y - 1;  // 0-7

    //Remap the user-known value to uLED-matrix-known value
    uint8_t ax_value = AX_map[X_index];
    uint8_t ay_value = AY_map[Y_index];
    
    // Extract individual bits
    ax_pins[0] = (ax_value & 0x01) ? 1 : 0;  // AX0
    ax_pins[1] = (ax_value & 0x02) ? 1 : 0;  // AX1  
    ax_pins[2] = (ax_value & 0x04) ? 1 : 0;  // AX2
    
    ay_pins[0] = (ay_value & 0x01) ? 1 : 0;  // AY0
    ay_pins[1] = (ay_value & 0x02) ? 1 : 0;  // AY1
    ay_pins[2] = (ay_value & 0x04) ? 1 : 0;  // AY2
}

/**@brief Turn on specific LED by setting Charlie-plexing pins and enabling PWM
 */
static void turn_on_led(uint8_t led_bit_position)
{
    uint8_t ax_pins[3], ay_pins[3];
    led_bit_to_pins(led_bit_position, ax_pins, ay_pins);
    
    // Set the Charlie-plexing pins
    nrf_gpio_pin_write(AX0_PIN, ax_pins[0]);
    nrf_gpio_pin_write(AX1_PIN, ax_pins[1]);
    nrf_gpio_pin_write(AX2_PIN, ax_pins[2]);
    nrf_gpio_pin_write(AY0_PIN, ay_pins[0]);
    nrf_gpio_pin_write(AY1_PIN, ay_pins[1]);
    nrf_gpio_pin_write(AY2_PIN, ay_pins[2]);
    
    // Enable PWM output (STIM_LED_ENABLE_PIN)
    pwm_enable();
}

/**@brief Turn off all LEDs by disabling PWM
 */
static void turn_off_all_leds(void)
{
    // Disable PWM output (STIM_LED_ENABLE_PIN goes low)
    pwm_disable();
}

/**@brief Extract selected LED indices from 64-bit mask
 */
static void extract_selected_leds(uint64_t led_selection, volatile uint8_t *led_array, volatile uint8_t *count)
{
    *count = 0;
    for (uint8_t i = 0; i < 64; i++) {
        if (led_selection & (1ULL << i)) {
            led_array[*count] = i;
            (*count)++;
        }
    }
}

/**@brief Enhanced timer event handler with ramp support
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    if (!stim_state.active) return;
    
    switch (event_type) {
        case NRF_TIMER_EVENT_COMPARE0:  // Pulse width timeout
        {
            // Capture current counter value    
            uint32_t current_timer_count = nrf_drv_timer_capture(&TIMER_OPTO, NRF_TIMER_CC_CHANNEL0);   
            NRF_LOG_DEBUG("LED off: %d us", current_timer_count);
            // Turn off current LED (disables PWM)
            turn_off_all_leds();
            stim_state.led_currently_on = false;
            
            // Move to next LED in the sequence  
            stim_state.current_led_index++;
            
            // Check if we've activated all selected LEDs in this period
            if (stim_state.current_led_index >= stim_state.num_selected_leds) {
                // All LEDs in this period are done
                stim_state.current_led_index = 0; // Reset for next period
            } else {
                // Start next LED immediately
                start_next_led();
            }
            break;
        }
        
        case NRF_TIMER_EVENT_COMPARE1: // Period timeout
        {
            // Capture current counter value    
            // uint32_t current_timer_count = nrf_drv_timer_capture(&TIMER_OPTO, NRF_TIMER_CC_CHANNEL0);
            // NRF_LOG_DEBUG("Period timeout: %d us", current_timer_count);
            stim_state.current_period_count++;
            
            start_next_period();
            break;
        }
        
        case NRF_TIMER_EVENT_COMPARE2:  // Phase duration timeout
        {
            // Capture current counter value    
            // uint32_t current_timer_count = nrf_drv_timer_capture(&TIMER_OPTO, NRF_TIMER_CC_CHANNEL0);
            // NRF_LOG_DEBUG("Phase timeout: %d us", current_timer_count);
            start_next_phase();
            break;
        }
        
        default:
            break;
    }
}

/**@brief Start the next LED in the current period
 */
static void start_next_led(void)
{
    if (!stim_state.active || stim_state.current_led_index >= stim_state.num_selected_leds) {
        return;
    }
    
    // Update ramp amplitude before turning on LED (only if in ramp phase)
    if (stim_state.current_phase == STIM_PHASE_RAMP_UP || stim_state.current_phase == STIM_PHASE_RAMP_DOWN) {
        update_ramp_amplitude();
    }

    // Turn on the current LED (enables PWM with pre-configured amplitude)
    uint8_t led_to_activate = stim_state.selected_leds[stim_state.current_led_index];
    turn_on_led(led_to_activate);
    stim_state.led_currently_on = true;
    
    // Set timer for pulse width - use current timer count + pulse width
    uint32_t pulse_width_us = current_stim_params.pulse_width[stim_state.current_sequence] * 1000;
    // Capture current counter value    
    uint32_t current_timer_count = nrf_drv_timer_capture(&TIMER_OPTO, NRF_TIMER_CC_CHANNEL0);
    uint32_t target_time = current_timer_count + nrf_drv_timer_us_to_ticks(&TIMER_OPTO, pulse_width_us);
    
    // NRF_LOG_DEBUG("LED off time Loaded: %d us", target_time);

    nrf_drv_timer_extended_compare(&TIMER_OPTO,
                             NRF_TIMER_CC_CHANNEL0,
                             target_time,
                             false, //Do not clear counter, if event occurs
                             true);
    
    NRF_LOG_DEBUG("LED %d on: %d us", led_to_activate + 1, current_timer_count);
}

/**@brief Start next period - only called once at sequence start
 */
static void start_next_period(void)
{
    if (!stim_state.active) return;
    
    stim_state.current_led_index = 0;
    
    // Set first period timer - use current timer count + period duration
    uint32_t period_us = current_stim_params.period[stim_state.current_sequence] * 1000;
    // Capture current counter value    
    uint32_t current_timer_count = nrf_drv_timer_capture(&TIMER_OPTO, NRF_TIMER_CC_CHANNEL0);
    uint32_t target_time = current_timer_count + nrf_drv_timer_us_to_ticks(&TIMER_OPTO, period_us);
    
    nrf_drv_timer_extended_compare(&TIMER_OPTO, 
                                 NRF_TIMER_CC_CHANNEL1, 
                                 target_time,
                                 false, //Do not clear counter 
                                 true);

    // Start first LED immediately
    start_next_led();
}

/**@brief Start next phase (ramp up -> main -> ramp down -> complete)
 */
static void start_next_phase(void)
{
    if (!stim_state.active) return;


    //Increment the phase flag
    switch (stim_state.current_phase) {

        case STIM_PHASE_INIT:

            // Transition to main phase
            stim_state.current_phase = STIM_PHASE_RAMP_UP;
            break;

        case STIM_PHASE_RAMP_UP:

            // Transition to main phase
            stim_state.current_phase = STIM_PHASE_MAIN;

            // Set current amplitude to static amplitude
            stim_state.current_amplitude = stim_state.target_amplitude;
            
            // Get current TOP value from PWM configuration
            uint32_t top_value = 16000000UL / pwm_state.current_frequency_hz;
            if (top_value < 3) top_value = 3;
            if (top_value > 32767) top_value = 32767;
            
            // Calculate duty cycle
            uint16_t duty_cycle_value = (stim_state.current_amplitude * top_value) / 100;
            // Update PWM values
            pwm_state.current_duty_cycle = duty_cycle_value;
            // Apply the PWM duty cycle, |0x8000 flips the PWM polarity to active-high
            m_pwm_values.channel_0 = pwm_state.current_duty_cycle | 0x8000; 

            break;

        case STIM_PHASE_MAIN:
            
            
            // Transition to ramp down phase
            stim_state.current_phase = STIM_PHASE_RAMP_DOWN;
            break;
            
        case STIM_PHASE_RAMP_DOWN:

            // Phase complete, move to next sequence
            start_next_sequence();
            break;
            
        default:
            break;
    }

    //Start the current phase
    start_phase(stim_state.current_phase);

}

/**@brief Start a specific phase of the current sequence
 */
static void start_phase(stimulation_phase_t phase)
{
    uint32_t phase_duration_us = 0;
    uint8_t seq_idx = stim_state.current_sequence;
    
    // Calculate phase duration
    switch (phase) {
        case STIM_PHASE_RAMP_UP:
            phase_duration_us = current_stim_params.ramp_up_time[seq_idx] * 1000;
            // NRF_LOG_INFO("Starting ramp up phase: %d ms", current_stim_params.ramp_up_time[seq_idx]);
            break;
            
        case STIM_PHASE_MAIN:
            phase_duration_us = current_stim_params.duration[seq_idx] * 1000;
            // NRF_LOG_INFO("Starting main stimulation phase: %d ms", current_stim_params.duration[seq_idx]);
            break;
            
        case STIM_PHASE_RAMP_DOWN:
            phase_duration_us = current_stim_params.ramp_down_time[seq_idx] * 1000;
            // NRF_LOG_INFO("Starting ramp down phase: %d ms", current_stim_params.ramp_down_time[seq_idx]);
            break;
            
        default:
            return;
    }
    
    // Skip phase if duration is 0
    if (phase_duration_us == 0) {
        start_next_phase();
        return;
    }

    // Record phase start time
    uint32_t current_timer_count = nrf_drv_timer_capture(&TIMER_OPTO, NRF_TIMER_CC_CHANNEL0);
    stim_state.phase_start_time_ms = current_timer_count / 1000; //1 count = 1us
    
    // Set phase duration timer (Channel 2)
    nrf_drv_timer_extended_compare(&TIMER_OPTO, 
                                 NRF_TIMER_CC_CHANNEL2, 
                                 current_timer_count + nrf_drv_timer_us_to_ticks(&TIMER_OPTO, phase_duration_us),
                                 false, 
                                 true);
    
    // Reset period management for this phase
    stim_state.current_period_count = 0;
    stim_state.current_led_index = 0;
    
    // Start period management
    start_next_period();
}

/**@brief Start next sequence or complete stimulation
 */
static void start_next_sequence(void)
{
    if (!stim_state.active) return;
    
    stim_state.current_period_count = 0;

    // Stop timer and turn off all LEDs
    nrf_drv_timer_disable(&TIMER_OPTO);
    turn_off_all_leds();
    
    // Move to next sequence
    stim_state.current_sequence++;
    
    if (stim_state.current_sequence >= current_stim_params.sequence_length) {
        // All sequences complete
        stimulation_complete();
        return;
    }
    
    // Start next sequence
    start_sequence(stim_state.current_sequence);
}

/**@brief Enhanced start_sequence function with ramp support
 */
static void start_sequence(uint8_t sequence_index)
{
    if (sequence_index >= current_stim_params.sequence_length) {
        stimulation_complete();
        return;
    }
    
    // NRF_LOG_INFO("Starting sequence %d with ramp support", sequence_index + 1);
    
    // Configure PWM for this sequence (frequency and initial amplitude)
    pwm_configure_for_sequence(sequence_index);
    
    // Extract selected LEDs for this sequence
    extract_selected_leds(current_stim_params.led_selection[sequence_index], 
                         stim_state.selected_leds, 
                         &stim_state.num_selected_leds);
    
    // if (stim_state.num_selected_leds == 0) {
    //     // NRF_LOG_WARNING("No LEDs selected for sequence %d, skipping", sequence_index + 1);
    //     start_next_sequence();
    //     return;
    // }
    
    // Initialize sequence state
    stim_state.current_sequence = sequence_index;
    stim_state.target_amplitude = current_stim_params.amplitude[sequence_index];
    stim_state.current_amplitude = 0; // Will be set by ramp
    stim_state.current_led_index = 0;
    stim_state.led_currently_on = false;
    stim_state.current_period_count = 0;
    
    // Clear and restart timer
    nrf_drv_timer_disable(&TIMER_OPTO);
    nrf_drv_timer_clear(&TIMER_OPTO);
    nrf_drv_timer_enable(&TIMER_OPTO);
    
    // Start with init phase (a place-holder phase that does nothing)
    stim_state.current_phase = STIM_PHASE_INIT;
    start_next_phase();
}

/**@brief Complete stimulation sequence
 */
static void stimulation_complete(void)
{
    NRF_LOG_INFO("=== STIMULATION COMPLETED ===");
    
    // Stop timer
    nrf_drv_timer_disable(&TIMER_OPTO);
    nrf_drv_timer_clear(&TIMER_OPTO);
    
    // Turn off all LEDs and disable PWM
    nrf_drv_pwm_stop(&m_pwm0, false);
    pwm_state.enabled = false;

    // Turn off SHAM
    nrf_gpio_pin_clear(SHAM_LED_PIN);
    
    // Reset state
    memset((void*)&stim_state, 0, sizeof(stim_state));
}

/**@brief Initialize precise timing system
 */
static void precise_timing_init(void)
{
    ret_code_t err_code;
    
    // Initialize single timer for all opto timing operations
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz; // 1μs resolution
    timer_cfg.mode = NRF_TIMER_MODE_TIMER;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    
    err_code = nrf_drv_timer_init(&TIMER_OPTO, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);
    
    // NRF_LOG_INFO("Single timer opto timing system initialized");
}

static void service_add(uint16_t uuid, uint16_t *handle)
{
    ble_uuid_t ble_uuid = { .uuid = uuid, .type = m_uuid_type };
    APP_ERROR_CHECK(sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, handle));
}

static void opto_characteristic_add(uint16_t uuid, ble_gatts_char_handles_t *handles,
                               uint8_t *initial_value, uint16_t len,
                               uint8_t properties, uint16_t service_handle,
                               bool read_authorize)
{
    ble_gatts_char_md_t char_md = {0};
    ble_gatts_attr_t attr_char_value = {0};
    ble_uuid_t char_uuid = { .uuid = uuid, .type = m_uuid_type };
    ble_gatts_attr_md_t attr_md = {0};
    ble_gatts_attr_md_t cccd_md = {0};

    // Configure CCCD if notify is enabled
    if (properties & 0x10) {  // 0x10 is notify bit
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
        char_md.p_cccd_md = &cccd_md;
    }

    char_md.char_props.read  = properties & 0x01;
    char_md.char_props.write = (properties >> 1) & 0x01;
    char_md.char_props.notify = (properties >> 4) & 0x01;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // Enable read authorization if requested
    if (read_authorize) {
        attr_md.rd_auth = 1;
    }

    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = len;
    attr_char_value.max_len = len;
    attr_char_value.p_value = initial_value;

    APP_ERROR_CHECK(sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, handles));
}

static void uuid_base_register(void)
{
    ble_uuid128_t base_uuid = {{0}};
    memcpy(base_uuid.uuid128, m_base_uuid, sizeof(m_base_uuid));
    APP_ERROR_CHECK(sd_ble_uuid_vs_add(&base_uuid, &m_uuid_type));

    // NRF_LOG_INFO("UUID base initialized");
}

static void custom_services_init(void)
{
    uint16_t h1, h2, h3;
    uint32_t err_code;
    ble_dfu_buttonless_init_t dfus_init = {0};
    
    // Initialize DFU buttonless service
    dfus_init.evt_handler = ble_dfu_evt_handler;
    err_code = ble_dfu_buttonless_init(&dfus_init);
    // NRF_LOG_INFO("DFU init result: 0x%08X", err_code);
    APP_ERROR_CHECK(err_code);
    // NRF_LOG_INFO("DFU service initialized");
    // Wait a bit for any flash operations to complete
    nrf_delay_ms(100);

    // NRF_LOG_INFO("Continuing with service initialization...");

    // Device Info Service 
    service_add(UUID_DEVICE_INFO_SERVICE, &h1);
    opto_characteristic_add(UUID_DEVICE_ID_CHAR, NULL, device_id, sizeof(device_id), 0x01, h1, false);
    opto_characteristic_add(UUID_FIRMWARE_VERSION_CHAR, NULL, firmware_version, sizeof(firmware_version), 0x01, h1, false);
    opto_characteristic_add(UUID_ULED_COLOR_CHAR, NULL, uled_color, sizeof(uled_color), 0x03, h1, false);
    opto_characteristic_add(UUID_ULED_CHECK_CHAR, &m_uled_check_char_handles, (uint8_t*)&uled_check, sizeof(uled_check), 0x01, h1, true);
    opto_characteristic_add(UUID_BATTERY_VOLTAGE_CHAR, &m_battery_voltage_char_handles, (uint8_t*)&battery_voltage, sizeof(battery_voltage), 0x1, h1, true);
    opto_characteristic_add(UUID_STATUS_LED_CHAR, &m_status_led_char_handles, &status_led_state, sizeof(status_led_state), 0x03, h1, false);
    opto_characteristic_add(UUID_SHAM_LED_CHAR, &m_sham_led_char_handles, &sham_led_state, sizeof(sham_led_state), 0x03, h1, false);
    opto_characteristic_add(UUID_DEVICE_LOG_CHAR, &m_device_log_char_handles, NULL, 128, 0x11, h1, false);
    opto_characteristic_add(UUID_LAST_STIM_TIME, &m_last_stim_time_char_handles, (uint8_t*)&last_stim_time_ms, sizeof(last_stim_time_ms), 0x01, h1, true);

    // NRF_LOG_INFO("Custom Service 1 init result: 0x%08X", err_code);

    // OptoControl Service - Store handles for parameters we need to update
    service_add(UUID_OPTO_CONTROL_SERVICE, &h2);
    
    opto_characteristic_add(UUID_SEQUENCE_LENGTH_CHAR, &m_sequence_length_char_handles, &sequence_length, sizeof(sequence_length), 0x03, h2, false);
    opto_characteristic_add(UUID_LED_SELECTION_CHAR, &m_led_selection_char_handles, (uint8_t*)led_selection, sizeof(led_selection), 0x03, h2, false);
    opto_characteristic_add(UUID_DURATION_CHAR, &m_duration_char_handles, (uint8_t*)duration, sizeof(duration), 0x03, h2, false);
    opto_characteristic_add(UUID_PERIOD_CHAR, &m_period_char_handles, (uint8_t*)period, sizeof(period), 0x03, h2, false);
    opto_characteristic_add(UUID_PULSE_WIDTH_CHAR, &m_pulse_width_char_handles, (uint8_t*)pulse_width, sizeof(pulse_width), 0x03, h2, false);
    opto_characteristic_add(UUID_AMPLITUDE_CHAR, &m_amplitude_char_handles, (uint8_t*)amplitude, sizeof(amplitude), 0x03, h2, false);
    opto_characteristic_add(UUID_PWM_FREQ_CHAR, &m_pwm_freq_char_handles, (uint8_t*)pwm_freq, sizeof(pwm_freq), 0x03, h2, false);
    opto_characteristic_add(UUID_RAMP_UP_TIME_CHAR, &m_ramp_up_time_char_handles, (uint8_t*)ramp_up_time, sizeof(ramp_up_time), 0x03, h2, false);
    opto_characteristic_add(UUID_RAMP_DOWN_TIME_CHAR, &m_ramp_down_time_char_handles, (uint8_t*)ramp_down_time, sizeof(ramp_down_time), 0x03, h2, false);
    opto_characteristic_add(UUID_TRIGGER_CHAR, &m_trigger_char_handles, &trigger, sizeof(trigger), 0x03, h2, false);

    // NRF_LOG_INFO("Custom Service 2 init result: 0x%08X", err_code);

    // Data Streaming Service
    service_add(UUID_DATA_STREAMING_SERVICE, &h3);
    opto_characteristic_add(UUID_IMU_ENABLE_CHAR, &m_imu_enable_char_handles, &imu_enable, sizeof(imu_enable), 0x03, h3, false);
    opto_characteristic_add(UUID_IMU_SAMPLE_RATE_CHAR, &m_imu_sample_rate_char_handles, &imu_sample_rate, sizeof(imu_sample_rate), 0x03, h3, false);
    opto_characteristic_add(UUID_IMU_RESOLUTION_CHAR, NULL, &imu_resolution, sizeof(imu_resolution), 0x03, h3, false);
    opto_characteristic_add(UUID_IMU_DATA_CHAR, &m_imu_data_char_handles, (uint8_t*)imu_data,sizeof(imu_data), 0x11, h3, false);

    // NRF_LOG_INFO("Custom Service 3 init result: 0x%08X", err_code);
    // NRF_LOG_INFO("Opto services initialized");
}

static void on_write(ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    

    // Handle STATUS LED and SHAM LED characteristic
    if (p_evt_write->handle == m_status_led_char_handles.value_handle)
    {
        if (p_evt_write->len == 1)
        {
            status_led_state = p_evt_write->data[0];
            if (status_led_state == 1)
            {
                nrf_gpio_pin_set(STATUS_LED_PIN);
                // NRF_LOG_INFO("Status LED turned ON");
            }
            else
            {
                nrf_gpio_pin_clear(STATUS_LED_PIN);
                // NRF_LOG_INFO("Status LED turned OFF");
            }
        }
        return;
    }

    // Handle SHAM LED characteristic
    if (p_evt_write->handle == m_sham_led_char_handles.value_handle)
    {
        if (p_evt_write->len == 1)
        {
            sham_led_state = p_evt_write->data[0];
            if (sham_led_state == 1)
            {
                nrf_gpio_pin_set(SHAM_LED_PIN);
                // NRF_LOG_INFO("Sham LED turned ON");
            }
            else
            {
                nrf_gpio_pin_clear(SHAM_LED_PIN);
                // NRF_LOG_INFO("Sham LED turned OFF");

            }
        }
        return;
    }

    // Handle trigger characteristic specially
    if (p_evt_write->handle == m_trigger_char_handles.value_handle)
    {
        if (p_evt_write->len == 1 && p_evt_write->data[0] == 1)
        {
            // Trigger the stimulation
            trigger_stimulation();

            // Reset trigger value back to 0
            trigger = 0;
            ble_gatts_value_t gatts_value = {
                .len = sizeof(trigger),
                .offset = 0,
                .p_value = &trigger
            };
            sd_ble_gatts_value_set(p_ble_evt->evt.gatts_evt.conn_handle,
                                   m_trigger_char_handles.value_handle,
                                   &gatts_value);
            
        }
        return;
    }
    
    // Add IMU enable handling
    if (p_evt_write->handle == m_imu_enable_char_handles.value_handle)
    {
        if (p_evt_write->len == 1)
        {
            imu_enable = p_evt_write->data[0]; 
            if (imu_enable && !imu_streaming) {
                i2c_devices_init();
                imu_streaming = true;
                app_timer_start(m_imu_timer_id, IMU_DATA_TIMER_INTERVAL, NULL);
            } else {
                imu_streaming = false;
                app_timer_stop(m_imu_timer_id);
            }
        }
        return;
    }

    // Add IMU sample rate handling
    if (p_evt_write->handle == m_imu_sample_rate_char_handles.value_handle)
    {
        if (p_evt_write->len == 1)
        {
            imu_sample_rate = p_evt_write->data[0];
            if (imu_streaming) {
                app_timer_stop(m_imu_timer_id);
                app_timer_start(m_imu_timer_id, 
                              APP_TIMER_TICKS(1000/imu_sample_rate), 
                              NULL);
            }
        }
        return;
    }

    // Handle all parameter characteristics using lookup table
    for (int i = 0; i < GATT_PARAM_MAP_SIZE; i++)
    {
        if (p_evt_write->handle == *(gatt_param_map[i].handle_ptr))
        {
            if (p_evt_write->len <= gatt_param_map[i].data_size) {
                memcpy(gatt_param_map[i].data_ptr, p_evt_write->data, p_evt_write->len);
                // NRF_LOG_DEBUG("Updated parameter: %s (%d bytes)", gatt_param_map[i].name, p_evt_write->len);
            } else {
                // NRF_LOG_WARNING("Parameter write too large: %s expects max %d bytes, got %d", 
                //                gatt_param_map[i].name, gatt_param_map[i].data_size, p_evt_write->len);
            }
            return; // Found and handled, exit loop
        }
    }
}

static uint8_t log_send_index = 0;

static void send_log_buffer(void)
{
    while (log_send_index < log_buffer.count) {
        uint16_t length = strlen(log_buffer.messages[log_send_index]) + 1;
        ble_gatts_hvx_params_t hvx_params = {
            .handle = m_device_log_char_handles.value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &length,
            .p_data = (uint8_t*)log_buffer.messages[log_send_index]
        };
        ret_code_t err = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
        if (err == NRF_ERROR_RESOURCES) {
            // Wait for BLE_GATTS_EVT_HVN_TX_COMPLETE before sending more
            return;
        }
        log_send_index++;
    }
    log_buffer.count = 0;
    log_send_index = 0;
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    
    // Feed watchdog during BLE event processing
    nrfx_wdt_channel_feed(m_channel_id);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt); //Write to GATT enters this event

            break;
            
        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            // Handle read requests for battery voltage characteristic
            ble_gatts_evt_rw_authorize_request_t const * p_auth_req =
                &p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_READ &&
                p_auth_req->request.read.handle == m_battery_voltage_char_handles.value_handle)
            {
                battery_voltage = read_battery_voltage_mv();

                ble_gatts_rw_authorize_reply_params_t reply = {0};
                reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
                reply.params.read.update = 1;
                reply.params.read.offset = 0;
                reply.params.read.len = sizeof(battery_voltage);
                reply.params.read.p_data = (uint8_t*)&battery_voltage;

                sd_ble_gatts_rw_authorize_reply(m_conn_handle, &reply);
                // return;
            }
            
            // Handle read requests for uLED_check characteristic
            if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_READ &&
                p_auth_req->request.read.handle == m_uled_check_char_handles.value_handle)
            {
                uled_check = read_uLED_check();

                ble_gatts_rw_authorize_reply_params_t reply = {0};
                reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
                reply.params.read.update = 1;
                reply.params.read.offset = 0;
                reply.params.read.len = sizeof(uled_check);
                reply.params.read.p_data = (uint8_t*)&uled_check;

                sd_ble_gatts_rw_authorize_reply(m_conn_handle, &reply);
                // return;
            }

            // Handle read requests for LAST_STIM_TIME_MS characteristic
            if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_READ &&
                p_auth_req->request.read.handle == m_last_stim_time_char_handles.value_handle)
            {
                ble_gatts_rw_authorize_reply_params_t reply = {0};
                reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
                reply.params.read.update = 1;
                reply.params.read.offset = 0;
                reply.params.read.len = sizeof(last_stim_time_ms);
                reply.params.read.p_data = (uint8_t*)&last_stim_time_ms;

                sd_ble_gatts_rw_authorize_reply(m_conn_handle, &reply);
                return;
            }

            // Send buffered logs
            send_log_buffer();

            break;
        }

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            uint16_t min_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
            uint16_t max_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
        
            ble_log("Connection interval: Min = %d ms, Max = %d ms",
                         min_conn_interval * 1.25, max_conn_interval * 1.25); // Convert units to milliseconds
            break;
        }

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            send_log_buffer();
            send_next_imu_data();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            //Debug
            nrf_gpio_pin_set(SHAM_LED_PIN);
            //
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            //Debug
            nrf_gpio_pin_set(SHAM_LED_PIN);
            //

            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    // Placeholder for SoC events if needed
}

static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);

    // NRF_LOG_INFO("BLE stack initialized");
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create a timer to increment global_time every millisecond
    err_code = app_timer_create(&global_time_timer, APP_TIMER_MODE_REPEATED, global_time_timer_handler);
    APP_ERROR_CHECK(err_code);

    // Start the timer
    err_code = app_timer_start(global_time_timer, APP_TIMER_TICKS(1), NULL); // 1 ms interval
    APP_ERROR_CHECK(err_code);
}

static void global_time_timer_handler(void *p_context)
{
    global_time++;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    // Set the maximum MTU size
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    // NRF_LOG_INFO("GAP initialized");
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    // NRF_LOG_INFO("GATT initialized");
}

static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    // NRF_LOG_INFO("Power Management initialized");
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    // NRF_LOG_INFO("Peer manager initialized");
}

/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            // Enter sleep mode
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    ble_uuid_t adv_uuids[] = {
        {UUID_DEVICE_INFO_SERVICE, BLE_UUID_TYPE_BLE},
        {UUID_OPTO_CONTROL_SERVICE, BLE_UUID_TYPE_BLE},
        {UUID_DATA_STREAMING_SERVICE, BLE_UUID_TYPE_BLE}
    };

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance     = false;
    init.advdata.flags                  = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    err_code = ble_advertising_init(&m_advertising, &init);
    // NRF_LOG_INFO("BLE advertising init result: 0x%08X", err_code);
    APP_ERROR_CHECK(err_code);
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    // NRF_LOG_INFO("Advertising initialized");
}

static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("ble_advertising_start() failed. Error code: 0x%08X", err_code);
        }
        APP_ERROR_CHECK(err_code);
    }
}

static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);

    // NRF_LOG_INFO("Buttons LEDs initialized");
}

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Main trigger function - starts interrupt-driven stimulation with PWM control and ramp support
 */
static void trigger_stimulation(void)
{
    if (stim_state.active) {
        NRF_LOG_WARNING("Stimulation already active, ignoring trigger");
        return;
    }

    // Copy current static variables (these are now updated in on_write())
    current_stim_params.sequence_length = sequence_length;
    
    uint8_t num_seq = MIN(current_stim_params.sequence_length, MAX_SEQUENCE_LENGTH);
    for (int i = 0; i < num_seq; i++) {
        current_stim_params.led_selection[i] = led_selection[i];
        current_stim_params.duration[i] = duration[i];
        current_stim_params.period[i] = period[i];
        current_stim_params.pulse_width[i] = pulse_width[i];
        current_stim_params.amplitude[i] = amplitude[i];
        current_stim_params.pwm_freq[i] = pwm_freq[i];
        current_stim_params.ramp_up_time[i] = ramp_up_time[i];
        current_stim_params.ramp_down_time[i] = ramp_down_time[i];
    }
    
    // Initialize state
    memset((void*)&stim_state, 0, sizeof(stim_state));
    stim_state.active = true;

    // Update last_stim_time_ms with the current global_time
    last_stim_time_ms = global_time;

    //Turn on sham when start stim
    nrf_gpio_pin_set(SHAM_LED_PIN);
    
    // Start first sequence with ramp up phase
    start_sequence(0);
}

static uint16_t read_battery_voltage_mv(void)
{
    nrf_saadc_value_t adc_value = 0;
    ret_code_t err_code = nrfx_saadc_sample_convert(0, &adc_value);
    APP_ERROR_CHECK(err_code);

    // Convert ADC value to millivolts (reference 0.6V, 12-bit ADC, 1/6 gain, 1/2 voltage divider)
    uint32_t result_mv = 2*((uint32_t)adc_value * 3600) / 4095;
    // ble_log("Battery ADC value: %d", adc_value);


    return (uint16_t)result_mv;
}

static uint64_t read_uLED_check(void)
{

    // Initialize uLED check result to be all 0
    uint64_t uled_check_result = 0;

    // Turn on uLED before scanning
    nrf_gpio_cfg_output(STIM_LED_ENABLE_PIN);

    // Measure 10 times when on
    for (int i = 0; i < 64; i++) {

        // Set LED i+1 to be measured
        uint8_t ax_pins[3], ay_pins[3];
        led_bit_to_pins(i, ax_pins, ay_pins);
        
        // Set the Charlie-plexing pins
        nrf_gpio_pin_write(AX0_PIN, ax_pins[0]);
        nrf_gpio_pin_write(AX1_PIN, ax_pins[1]);
        nrf_gpio_pin_write(AX2_PIN, ax_pins[2]);
        nrf_gpio_pin_write(AY0_PIN, ay_pins[0]);
        nrf_gpio_pin_write(AY1_PIN, ay_pins[1]);
        nrf_gpio_pin_write(AY2_PIN, ay_pins[2]);

        // Turn on LED
        nrf_gpio_pin_clear(STIM_LED_ENABLE_PIN); // Turn off LED before measurement
        nrf_gpio_pin_set(STIM_LED_ENABLE_PIN); // Let LED draw current for measurement

        // Perform ADC measurement
        nrf_saadc_value_t adc_value = 0;
        ret_code_t err_code = nrfx_saadc_sample_convert(1, &adc_value);
        APP_ERROR_CHECK(err_code);
        // Convert ADC value to millivolts (reference 0.6V, 12-bit ADC, 1/6 gain, 1/2 voltage divider)
        uint32_t CURRENT_SENSING_mV = ((uint32_t)adc_value * 3600) / 4095;
        uint32_t CURRENT_mA = CURRENT_SENSING_mV/71.5; //I = V/R, R = 71.5 ohm
        // ble_log("LED %d Current: %d mA", i+1, CURRENT_mA);

        // Turn off LED
        nrf_gpio_pin_clear(STIM_LED_ENABLE_PIN); // Turn off LED after measurement

        // Mark bit i in uled_check: 1 if in range [5,7] mA, 0 otherwise
        if (CURRENT_mA >= 5.0f && CURRENT_mA <= 7.0f) {
            uled_check_result |= (1ULL << i);   // Set bit i
        } else {
            uled_check_result &= ~(1ULL << i);  // Clear bit i
        }

    }

    return uled_check_result;

}

static void saadc_init(void)
{
    // Release P0.04 (AIN2), P0.05 (AIN3) from GPIO to analog input
    nrf_gpio_cfg_default(4);
    nrf_gpio_cfg_default(5);

    ret_code_t err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);

    // Configure channel 0 for battery voltage measurement (AIN3 = P0.05)
    nrf_saadc_channel_config_t channel_config_0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VBAT_ADC);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config_0);
    APP_ERROR_CHECK(err_code);

    // Configure second channel for LED current measurement (AIN2 = P0.04)
    nrf_saadc_channel_config_t channel_config_1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(LED_CURRENT_ADC);

    err_code = nrf_drv_saadc_channel_init(1, &channel_config_1);
    APP_ERROR_CHECK(err_code);
}

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/**@brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            // Transfer completed event
            m_xfer_done = true; //Flag that transfer is done
            break;
        
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            NRF_LOG_ERROR("TWI: Address NACK received");
            break;
        
        case NRF_DRV_TWI_EVT_DATA_NACK:
            NRF_LOG_ERROR("TWI: Data NACK received");
            break;
        
        default:
            break;
    }
}


/**@brief Initialize TWI/I2C interface.
 */
static ret_code_t twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,  // 100kHz for compatibility
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    //Temp
    if (m_twi_initialized)
    {
        nrf_drv_twi_uninit(&m_twi); // Uninitialize if already initialized
    }
    //End temp

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    if (err_code == NRF_SUCCESS)
    {
        nrf_drv_twi_enable(&m_twi);
        m_twi_initialized = true;
        NRF_LOG_INFO("TWI initialized successfully");
        ble_log("TWI initialized successfully");
    }
    else
    {
        NRF_LOG_ERROR("TWI initialization failed with error: 0x%08X", err_code);
        ble_log("TWI initialization failed with error: 0x%08X", err_code);
    }

    return err_code;
}

// Add this new IMU initialization function
static ret_code_t imu_init(void)
{
    ret_code_t err_code;
    uint8_t who_am_i;
    uint8_t who_am_i_reg = IIM42652_WHO_AM_I_REG;
    
    NRF_LOG_INFO("Starting IIM42652 initialization...");
    ble_log("Starting IIM42652 initialization...");
    nrf_delay_ms(5);

    // Read WHO_AM_I register
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_tx(&m_twi, IIM42652_I2C_ADDR, &who_am_i_reg, 1, true); //True for repeated start, false for issue stop condition 
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to write WHO_AM_I register address. Error code: 0x%08X", err_code);
        ble_log("Failed to write WHO_AM_I register address. Error code: 0x%08X", err_code);
        return err_code;
    }
    // Wait for transfer to complete with timeout
    uint32_t timeout = I2C_TRANSFER_TIMEOUT_MS;
    while (m_xfer_done == false && timeout > 0)
    {
        nrf_delay_ms(1);
        timeout--;
    }

    if (timeout == 0)
    {
        NRF_LOG_ERROR("Writing to read WH_AM_I register timed out");
        ble_log("Writing to read WHO_AM_I register timed out");
        return NRF_ERROR_TIMEOUT;
    }

    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_rx(&m_twi, IIM42652_I2C_ADDR, &who_am_i, 1);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to read WHO_AM_I register. Error code: 0x%08X", err_code);
        ble_log("Failed to read WHO_AM_I register. Error code: 0x%08X", err_code);
        return err_code;
    }
    // Wait for transfer to complete with timeout
    while (m_xfer_done == false && timeout > 0)
    {
        nrf_delay_ms(1);
        timeout--;
    }

    if (timeout == 0)
    {
        NRF_LOG_ERROR("Reading WHO_AM_I register timed out");
        ble_log("Reading WHO_AM_I register timed out");
        return NRF_ERROR_TIMEOUT;
    }

    // NRF_LOG_ERROR("Finished writing WHO_AM_I register address, now reading value...");
    // ble_log("Finished writing WHO_AM_I register address, now reading value...");


    NRF_LOG_INFO("WHO_AM_I register value: 0x%02X (expected: 0x6F)", who_am_i);
    ble_log("WHO_AM_I register value: 0x%02X (expected: 0x6F)", who_am_i);
    
    if (who_am_i != 0x6F) {
        NRF_LOG_ERROR("WHO_AM_I check failed. Got: 0x%02X, Expected: 0x6F", who_am_i);
        ble_log("WHO_AM_I check failed. Got: 0x%02X, Expected: 0x6F", who_am_i);
        return NRF_ERROR_NOT_FOUND;
    }
    
    NRF_LOG_INFO("WHO_AM_I check passed, configuring IMU...");
    ble_log("WHO_AM_I check passed, configuring IMU...");
    
    // NRF_LOG_INFO("IIM42652 initialization complete!");
    return NRF_SUCCESS;
}

// Add IMU configuration function
static ret_code_t imu_configure(void)
{
    ret_code_t err_code;
    uint8_t config_data[2];
    
    // NRF_LOG_INFO("Configuring IMU...");
    
    // Enable accel and gyro
    config_data[0] = IIM42652_PWR_MGMT0_REG;
    config_data[1] = 0x0F; // Enable both accel and gyro
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_tx(&m_twi, IIM42652_I2C_ADDR, config_data, 2, false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to configure power management. Error code: 0x%08X", err_code);
        ble_log("Failed to configure power management. Error code: 0x%08X", err_code);
        return err_code;
    }
    // Wait for transfer to complete with timeout
    uint32_t timeout = I2C_TRANSFER_TIMEOUT_MS;
    while (m_xfer_done == false && timeout > 0)
    {
        nrf_delay_ms(1);
        timeout--;
    }

    if (timeout == 0)
    {
        NRF_LOG_ERROR("IMU power configuration timed out");
        ble_log("IMU power configuration timed out");
        return NRF_ERROR_TIMEOUT;
    }

    NRF_LOG_DEBUG("Power management configured successfully");
    ble_log("Power management configured successfully");
    
    // Configure gyro: ±2000 dps, 100Hz ODR
    config_data[0] = IIM42652_GYRO_CONFIG0_REG;
    config_data[1] = 0x06; // ODR = 100Hz, range = ±2000dps
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_tx(&m_twi, IIM42652_I2C_ADDR, config_data, 2, false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to configure gyroscope. Error code: 0x%08X", err_code);
        ble_log("Failed to configure gyroscope. Error code: 0x%08X", err_code);
        return err_code;
    }
    // Wait for transfer to complete with timeout
    while (m_xfer_done == false && timeout > 0)
    {
        nrf_delay_ms(1);
        timeout--;
    }

    if (timeout == 0)
    {
        NRF_LOG_ERROR("Gyroscope configuration timed out");
        ble_log("Gyroscope configuration timed out");
        return NRF_ERROR_TIMEOUT;
    }

    NRF_LOG_DEBUG("Gyroscope configured: 100Hz, ±2000dps");
    ble_log("Gyroscope configured: 100Hz, ±2000dps");
    
    // Configure accel: ±16g, 100Hz ODR
    config_data[0] = IIM42652_ACCEL_CONFIG0_REG;
    config_data[1] = 0x06; // ODR = 100Hz, range = ±16g
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_tx(&m_twi, IIM42652_I2C_ADDR, config_data, 2, false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to configure accelerometer. Error code: 0x%08X", err_code);
        ble_log("Failed to configure accelerometer. Error code: 0x%08X", err_code);
        return err_code;
    }
    // Wait for transfer to complete with timeout
    while (m_xfer_done == false && timeout > 0)
    {
        nrf_delay_ms(1);
        timeout--;
    }

    if (timeout == 0)
    {
        NRF_LOG_ERROR("Accelerometer configuration timed out");
        ble_log("Accelerometer configuration timed out");
        return NRF_ERROR_TIMEOUT;
    }

    NRF_LOG_DEBUG("Accelerometer configured: 100Hz, ±16g");
    ble_log("Accelerometer configured: 100Hz, ±16g");
    
    NRF_LOG_INFO("IMU configuration complete!");
    ble_log("IMU configuration complete!");

    return NRF_SUCCESS;
}

static ret_code_t mag_configure(void) {

    // --- Configure LIS2MDLTR Magnetometer ---
    ret_code_t mag_err_code;
    uint8_t mag_config[2];

    // CTRL_REG_A: Enable X, Y, Z, set ODR = 100Hz (default)
    mag_config[0] = 0x60; // CTRL_REG1
    mag_config[1] = 0x8C; // Meaning below:
    // bit 7 = 1: Temperature compensation enabled
    // bit 6 = 0: Reboot memory content disabled
    // bit 5 = 0: Soft reset not triggered
    // bit 4 = 0: High-resolution mode enabled, low power mode disabled
    // bit 3-2 = 11: ODR = 100Hz, 00 = 10Hz, 01 = 20Hz, 10 = 50Hz
    // bit 1-0 = 00: Continuous mode

    m_xfer_done = false;
    mag_err_code = nrf_drv_twi_tx(&m_twi, LIS2MDLTR_I2C_ADDR, mag_config, 2, false);
    if (mag_err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to write CTRL_REG1. Error code: 0x%08X", mag_err_code);
        ble_log("Failed to write CTRL_REG1. Error code: 0x%08X", mag_err_code);
        return mag_err_code;
    }
    while (!m_xfer_done);

    // CTRL_REG_B: Ensure soft reset & reboot disabled
    mag_config[0] = 0x61; // CTRL_REG2
    mag_config[1] = 0x00; // Default, low-pass filter disabled, offset cancellation disabled
    m_xfer_done = false;
    mag_err_code = nrf_drv_twi_tx(&m_twi, LIS2MDLTR_I2C_ADDR, mag_config, 2, false);
    if (mag_err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to write CTRL_REG2. Error code: 0x%08X", mag_err_code);
        ble_log("Failed to write CTRL_REG2. Error code: 0x%08X", mag_err_code);
        return mag_err_code;
    }
    while (!m_xfer_done);

    // CTRL_REG_C: Continuous-conversion mode
    mag_config[0] = 0x62; // CTRL_REG3
    mag_config[1] = 0x10; // Enable BDU, locks data register when read is rquested
    m_xfer_done = false;
    mag_err_code = nrf_drv_twi_tx(&m_twi, LIS2MDLTR_I2C_ADDR, mag_config, 2, false);
    if (mag_err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to write CTRL_REG3. Error code: 0x%08X", mag_err_code);
        ble_log("Failed to write CTRL_REG3. Error code: 0x%08X", mag_err_code);
        return mag_err_code;
    }
    while (!m_xfer_done);

    NRF_LOG_INFO("LIS2MDLTR configured for continuous mode @ 100Hz.");
    ble_log("LIS2MDLTR configured for continuous mode @ 100Hz.");
    
    return NRF_SUCCESS;

}

// Add IMU data reading function
static ret_code_t imu_read_data(void)
{
    ret_code_t err_code;
    uint8_t raw_data[12];
    
    // Read accelerometer data
    uint8_t accel_reg = IIM42652_ACCEL_DATA_REG;
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_tx(&m_twi, IIM42652_I2C_ADDR, &accel_reg, 1, true);
    if (err_code != NRF_SUCCESS) return err_code;
    while (m_xfer_done == false); // Wait for transfer to complete
    
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_rx(&m_twi, IIM42652_I2C_ADDR, raw_data, 6);
    if (err_code != NRF_SUCCESS) return err_code;
    while (m_xfer_done == false); // Wait for transfer to complete
    
    // Read gyroscope data
    uint8_t gyro_reg = IIM42652_GYRO_DATA_REG;
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_tx(&m_twi, IIM42652_I2C_ADDR, &gyro_reg, 1, true);
    if (err_code != NRF_SUCCESS) return err_code;
    while (m_xfer_done == false); // Wait for transfer to complete
    
    m_xfer_done = false; // Reset transfer done flag
    err_code = nrf_drv_twi_rx(&m_twi, IIM42652_I2C_ADDR, &raw_data[6], 6);
    if (err_code != NRF_SUCCESS) return err_code;
    while (m_xfer_done == false); // Wait for transfer to complete

    // Read magnetometer data (X, Y, Z, 2 bytes each)
    uint8_t mag_reg = LIS2MDLTR_OUTX_L_REG;
    uint8_t mag_raw[6] = {0};
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LIS2MDLTR_I2C_ADDR, &mag_reg, 1, true);
    if (err_code != NRF_SUCCESS) return err_code;
    while (!m_xfer_done);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, LIS2MDLTR_I2C_ADDR, mag_raw, 6);
    if (err_code != NRF_SUCCESS) return err_code;
    while (!m_xfer_done);

    // Convert acc, gyro raw data to int16_t values
    for (int i = 0; i < 6; i++) {
        imu_data[i+2] = (int16_t)((raw_data[i*2] << 8) | raw_data[i*2 + 1]);
    }

    // Store magnetometer data in imu_data[7..9]
    imu_data[8] = (int16_t)((mag_raw[1] << 8) | mag_raw[0]); // X
    imu_data[9] = (int16_t)((mag_raw[3] << 8) | mag_raw[2]); // Y
    imu_data[10] = (int16_t)((mag_raw[5] << 8) | mag_raw[4]); // Z
    
    // Update and store timestamp
    imu_data[0] = (int16_t)(imu_samplestamp & 0xFFFF);         // Low 16 bits
    imu_data[1] = (int16_t)((imu_samplestamp >> 16) & 0xFFFF); // High 16 bits
    imu_samplestamp++;
    
    //Send data via BLE if connected
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        uint16_t len = sizeof(imu_data);
        ble_gatts_hvx_params_t hvx_params = {
            .handle = m_imu_data_char_handles.value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &len,
            .p_data = (uint8_t*)imu_data
        };
        ret_code_t err = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
        if (err != NRF_SUCCESS) {
            // Buffer unsent data
            uint16_t next_head = (imu_data_buf_head + 1) % IMU_DATA_BUFFER_SIZE;
            if (next_head != (imu_data_buf_tail % IMU_DATA_BUFFER_SIZE)) {
                memcpy(imu_data_buffer[imu_data_buf_head % IMU_DATA_BUFFER_SIZE], imu_data, sizeof(imu_data));
                imu_data_buf_head++;
            } else {
                // Buffer overflow, drop sample or handle as needed
                // ble_log("IMU buffer overflow, sample dropped");
            }
        }
    }
    
    return NRF_SUCCESS;
}

static void send_next_imu_data(void)
{
    while (imu_data_buf_tail != imu_data_buf_head) {
        if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            break;

        uint16_t idx = imu_data_buf_tail % IMU_DATA_BUFFER_SIZE;
        uint16_t len = sizeof(imu_data_buffer[0]);
        ble_gatts_hvx_params_t hvx_params = {
            .handle = m_imu_data_char_handles.value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &len,
            .p_data = (uint8_t*)imu_data_buffer[idx]
        };
        ret_code_t err = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
        if (err == NRF_SUCCESS) {
            imu_data_buf_tail++;
        } else if (err == NRF_ERROR_RESOURCES) {
            // BLE stack is full, wait for next TX_COMPLETE
            break;
        } else {
            // Unexpected error, drop this sample
            imu_data_buf_tail++;
        }
    }
}

// Add IMU timer handler
static void imu_timer_handler(void * p_context)
{
    if (imu_streaming) {
        imu_read_data();
    }
}

/**@brief Watchdog event handler - called when watchdog times out.
 */
static void wdt_event_handler(void)
{
    // This function is called when the watchdog times out
    // The system will reset automatically after this handler returns
    NRF_LOG_ERROR("Watchdog timeout! System will reset.");
    NRF_LOG_FINAL_FLUSH();
}

/**@brief Initialize the watchdog timer.
 */
static void wdt_init(void)
{
    ret_code_t err_code;
    
    // Configure WDT
    nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;
    config.reload_value = WDT_TIMEOUT_MS;
    
    err_code = nrfx_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    
    nrfx_wdt_enable();
    
    NRF_LOG_INFO("Watchdog timer initialized with %d ms timeout", WDT_TIMEOUT_MS);
}


/**@brief Initialize all I2C devices.
 */
static ret_code_t i2c_devices_init(void)
{
    ret_code_t err_code;
    
    // Initialize TWI first
    err_code = twi_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("TWI initialization failed");
        ble_log("TWI initialization failed");
        return err_code;
    }
    
    // Scan for I2C devices
    // twi_scan();

    // Initialize IMU
    err_code = imu_init();
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("IIM-42652 initialization failed - IMU readings unavailable");
        ble_log("IIM-42652 initialization failed - IMU readings unavailable");
    }

    // Configure IMU
    err_code = imu_configure();
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("IMU configuration failed. Error code: 0x%08X", err_code);
        return err_code;
    }

    // Configure Magnetometer
    err_code = mag_configure();
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Magnetometer configuration failed. Error code: 0x%08X", err_code);
        return err_code;
    }

    // Create IMU timer
    err_code = app_timer_create(&m_imu_timer_id,
                               APP_TIMER_MODE_REPEATED,
                               imu_timer_handler);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to create IMU timer");
        ble_log("Failed to create IMU timer");
    }

    return NRF_SUCCESS;

}

int main(void)
{
    bool erase_bonds;
    ret_code_t err_code;
    
    log_init();
    
    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
    
    led_control_init();
    timer_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    peer_manager_init();
    uuid_base_register();
    custom_services_init();
    advertising_init();
    
    conn_params_init();
    precise_timing_init();
    pwm_init();  // Initialize PWM system
    saadc_init(); // Initialize SAADC for battery voltage measurement
    // i2c_devices_init();

    // Initialize watchdog timer (after all other initialization)
    wdt_init();

    // NRF_LOG_INFO("GATT Server Peripheral with PWM control, ramp support, and DFU started.");
    advertising_start(erase_bonds);
    NRF_LOG_INFO("Advertising started.");

    for (;;)
    {
        // Feed the watchdog to prevent timeout
        nrfx_wdt_channel_feed(m_channel_id);
        
        idle_state_handle();
    }
}