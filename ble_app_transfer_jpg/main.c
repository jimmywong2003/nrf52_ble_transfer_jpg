/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "app_uart.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"

#include "ble_image_transfer_service.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_drv_uart.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */

#define LEDBUTTON_BUTTON_1              BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON_2              BSP_BUTTON_1                            /**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON_3              BSP_BUTTON_2                            /**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON_4              BSP_BUTTON_3                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "S112JPG"                         /**< Name of device. Will be included in the advertising data. */

#define ITS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(20000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCHED_MAX_EVENT_DATA_SIZE           APP_TIMER_SCHED_EVENT_DATA_SIZE            /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                    40                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                    20                                         /**< Maximum number of events in the scheduler queue. */
#endif

#define TX_POWER_LEVEL                  (4)                                    /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */


#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define IMAGE_BUFFER_SIZE     0x1000

BLE_ITS_DEF(m_its, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE JPG Transfer service instance. */

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/


#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(1000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

static nrf_drv_uart_t m_uart =  NRF_DRV_UART_INSTANCE(0);
static uint8_t m_rx_byte[UART_RX_BUF_SIZE];

static uint8_t m_rx_image_buffer[IMAGE_BUFFER_SIZE];
static uint16_t m_rx_image_buffer_len = 0;

static uint8_t m_new_command_received = 0;
static uint8_t m_new_resolution, m_new_phy;
static bool m_stream_mode_active = false;
static ble_its_ble_params_info_t m_ble_params_info = {20, 50, 1, 1};

static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

//use of the data transfer
static uint8_t img_data_buffer[BLE_ITS_MAX_DATA_LEN];
static uint32_t img_data_length = BLE_ITS_MAX_DATA_LEN;
static uint8_t m_buffer_transfer_index = 0;
static bool test_flag = false;
static bool start_thoughput_test = false;
static uint32_t frameCount = 0;
static uint32_t frameCount_previous = 0;

static bool m_active = false;
static uint16_t m_ble_mtu_length = 244;//BLE_GATT_ATT_MTU_DEFAULT - 3;
static uint16_t m_ble_dle_length = 251;//BLE_GATT_ATT_MTU_DEFAULT - 3;
static bool m_file_is_sending = false;

typedef enum {
        PAYLOAD_FILE_OPCODE_PING         = 0xA0,
        PAYLOAD_FILE_OPCODE_FILEINFO_RSP = 0xA1,
        PAYLOAD_FILE_OPCODE_MTU_REQ      = 0xA2,
        PAYLOAD_FILE_OPCODE_MTU_RSP      = 0xA3,
        PAYLOAD_FILE_OPCODE_DATA_REQ     = 0xA4,
        PAYLOAD_FILE_OPCODE_DATA_RSP     = 0xA5,
        PAYLOAD_FILE_OPCODE_COMPLETE     = 0xA6,
        PAYLOAD_FILE_OPCODE_COMPLETE_ACK = 0xA7,
        PAYLOAD_FILE_OPCODE_DATA_RSP_LAST = 0xA8,
} serial_payload_file_cmd_t;

typedef enum {
        eSERIAL_FILE_OBJECT_STATE_IDLE   = 0x00,
        eSERIAL_FILE_OBJECT_STATE_PING,
        eSERIAL_FILE_OBJECT_STATE_GET_FILE_INFO,
        eSERIAL_FILE_OBJECT_STATE_MTU_REQ,
        eSERIAL_FILE_OBJECT_STATE_MTU_RSP,
        eSERIAL_FILE_OBJECT_STATE_DATA_REQ,
        eSERIAL_FILE_OBJECT_STATE_DATA_RSP,
        eSERIAL_FILE_OBJECT_STATE_BLE_TX,
        eSERIAL_FILE_OBJECT_STATE_BLE_COMPLETE,
        eSERIAL_FILE_OBJECT_STATE_DONE
} serial_file_object_state_t;


typedef struct serial_file_object_s
{
        uint32_t filesize;
        uint32_t crc32;
        uint32_t offset_req;
        uint32_t offset_rsp;
        uint32_t length_req;
        uint8_t mtusize;
        // serial_payload_file_cmd_t state;
} serial_file_object_t;

typedef struct serial_object_data_request_s
{
        uint8_t payload_len;
        serial_file_object_state_t opcode;
        uint16_t request_len;
        uint32_t offset_addr;
} serial_object_data_request_t;

static serial_file_object_t m_file_object = {0};

static serial_object_data_request_t m_data_request_obj;
static void serial_uart_response(serial_payload_file_cmd_t cmd);
static void transfer_image_buffer(void * p_event_data, uint16_t size);

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
        .adv_data =
        {
                .p_data = m_enc_advdata,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
                .p_data = m_enc_scan_response_data,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

        }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
        bsp_board_init(BSP_INIT_LEDS);
}



static uint32_t time_count = 0;
/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
        UNUSED_PARAMETER(p_context);
        //serial_uart_response(PAYLOAD_FILE_OPCODE_PING);
        NRF_LOG_DEBUG("Time count = %d", time_count);
        time_count++;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
        // Initialize timer module, making it use the scheduler
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        // Create timers.
        err_code = app_timer_create(&m_battery_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    battery_level_meas_timeout_handler);
        APP_ERROR_CHECK(err_code);

}

/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
        ret_code_t err_code;
        ble_gap_conn_params_t gap_conn_params;
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

//        ble_gap_addr_t ble_address = {.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
//                                      .addr_id_peer = 0,
//                                      .addr = {0xC3,0x11,0x99,0x33,0x44,0xFF}};
//        err_code = sd_ble_gap_addr_set(&ble_address);
//        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                //m_ble_params_info.mtu = m_ble_its_max_data_len;
                m_ble_mtu_length = data_length;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("gatt_event: ATT MTU is set to 0x%X (%d)", data_length, data_length);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                m_ble_mtu_length = data_length;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
        }
        NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

#if !defined(S112)
        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
        ret_code_t err_code;
        ble_advdata_t advdata;
        ble_advdata_t srdata;

        ble_uuid_t adv_uuids[] = {{BLE_UUID_ITS_SERVICE, ITS_SERVICE_UUID_TYPE}};

        // Build and set advertising data.
        memset(&advdata, 0, sizeof(advdata));

        advdata.name_type          = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance = true;
        advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

        memset(&srdata, 0, sizeof(srdata));
        srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
        srdata.uuids_complete.p_uuids  = adv_uuids;

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
        APP_ERROR_CHECK(err_code);

        ble_gap_adv_params_t adv_params;

        // Set advertising parameters.
        memset(&adv_params, 0, sizeof(adv_params));

        adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
        adv_params.duration        = APP_ADV_DURATION;
        adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        adv_params.p_peer_addr     = NULL;
        adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
        adv_params.interval        = APP_ADV_INTERVAL;

        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
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



// /**@brief   Function for handling app_uart events.
//  *
//  * @details This function will receive a single character from the app_uart module and append it to
//  *          a string. The string will be be sent over BLE when the last character received was a
//  *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
//  */
// /**@snippet [Handling the data received over UART] */
// void uart_event_handle(app_uart_evt_t * p_event)
// {
//         static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//         static uint8_t index = 0;
//         uint32_t err_code;
//
//         switch (p_event->evt_type)
//         {
//         case APP_UART_DATA_READY:
//                 UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//                 index++;
//
//                 if ((data_array[index - 1] == '\n') ||
//                     (data_array[index - 1] == '\r') ||
//                     (index >= m_ble_nus_max_data_len))
//                 {
//                         if (index > 1)
//                         {
//                                 NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//                                 NRF_LOG_HEXDUMP_DEBUG(data_array, index);
//                                 do
//                                 {
//                                         uint16_t length = (uint16_t)index;
//                                         err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
//                                         if ((err_code != NRF_ERROR_INVALID_STATE) &&
//                                             (err_code != NRF_ERROR_RESOURCES) &&
//                                             (err_code != NRF_ERROR_NOT_FOUND))
//                                         {
//                                                 APP_ERROR_CHECK(err_code);
//                                         }
//                                 } while (err_code == NRF_ERROR_RESOURCES);
//                         }
//                 }
//                 break;
//
//         case APP_UART_COMMUNICATION_ERROR:
//                 APP_ERROR_HANDLER(p_event->data.error_communication);
//                 break;
//
//         case APP_UART_FIFO_ERROR:
//                 APP_ERROR_HANDLER(p_event->data.error_code);
//                 break;
//
//         default:
//                 break;
//         }
// }
//
// /**@brief  Function for initializing the UART module.
//  */
// /**@snippet [UART Initialization] */
// static void uart_init(void)
// {
//         uint32_t err_code;
//         app_uart_comm_params_t const comm_params =
//         {
//                 .rx_pin_no    = RX_PIN_NUMBER,
//                 .tx_pin_no    = TX_PIN_NUMBER,
//                 .rts_pin_no   = RTS_PIN_NUMBER,
//                 .cts_pin_no   = CTS_PIN_NUMBER,
//                 .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
//                 .use_parity   = false,
// #if defined (UART_PRESENT)
//                 .baud_rate    = NRF_UART_BAUDRATE_115200
// #else
//                 .baud_rate    = NRF_UARTE_BAUDRATE_115200
// #endif
//         };
//
//         APP_UART_FIFO_INIT(&comm_params,
//                            UART_RX_BUF_SIZE,
//                            UART_TX_BUF_SIZE,
//                            uart_event_handle,
//                            APP_IRQ_PRIORITY_LOWEST,
//                            err_code);
//         APP_ERROR_CHECK(err_code);
// }
// /**@snippet [UART Initialization] */


static void its_data_handler(ble_its_t * p_its, uint8_t const * p_data, uint16_t length)
{

        uint32_t err_code = NRF_SUCCESS;
        //NRF_LOG_HEXDUMP_INFO(p_data, length);
        switch(p_data[0])
        {
        // Take picture
        case APP_CMD_SINGLE_CAPTURE:
        case APP_CMD_SEND_BLE_PARAMS:
                m_new_command_received = p_data[0];

                NRF_LOG_INFO("APP_CMD_SINGLE_CAPTURE");
                serial_uart_response(PAYLOAD_FILE_OPCODE_PING);

                break;

        case APP_CMD_START_STREAM:
                m_stream_mode_active = true;
                m_new_command_received = p_data[0];
                break;

        case APP_CMD_STOP_STREAM:
                m_stream_mode_active = false;
                m_new_command_received = p_data[0];
                break;

        case APP_CMD_CHANGE_RESOLUTION:
                m_new_command_received = APP_CMD_CHANGE_RESOLUTION;
                m_new_resolution = p_data[1];
                break;

        case APP_CMD_CHANGE_PHY:
                m_new_command_received = APP_CMD_CHANGE_PHY;
                m_new_phy = p_data[1];
                break;
        case APP_CMD_SEND_BUFFER_REQ:
                NRF_LOG_INFO("APP_CMD_SEND_BUFFER_REQ");
                m_file_is_sending = true;
                err_code = app_sched_event_put(NULL, 0, transfer_image_buffer);
                APP_ERROR_CHECK(err_code);

                break;

        case APP_CMD_SEND_BUFFER_COMPLETE:
                NRF_LOG_INFO("APP_CMD_SEND_BUFFER_COMPLETE");

                if (m_file_is_sending == true)
                        if (m_file_object.offset_req != m_file_object.filesize)
                                serial_uart_response(PAYLOAD_FILE_OPCODE_DATA_REQ);
                        else
                                m_file_is_sending = false;
                break;

        default:
                NRF_LOG_ERROR("Unknown command!!");
                break;
        }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        ret_code_t err_code;
        nrf_ble_qwr_init_t qwr_init = {0};

        ble_its_init_t its_init;
        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(err_code);


        // Initialize ITS.
        memset(&its_init, 0, sizeof(its_init));
        its_init.data_handler = its_data_handler;
        err_code = ble_its_init(&m_its, &its_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
        ret_code_t err_code;

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
        ret_code_t err_code;
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


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
        ret_code_t err_code;

        err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Advertising Start");

        bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        ret_code_t err_code;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                NRF_LOG_INFO("Connected");
                bsp_board_led_on(CONNECTED_LED);
                bsp_board_led_off(ADVERTISING_LED);
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
                APP_ERROR_CHECK(err_code);

                // Start application timers.
                err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);

                tx_power_set();

                break;
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
                ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
                NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
                             p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
                             p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
        } break;

        case BLE_GAP_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected");
                bsp_board_led_off(CONNECTED_LED);
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);
                advertising_start();
                break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported
                err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                       BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                       NULL,
                                                       NULL);
                APP_ERROR_CHECK(err_code);
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
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                // No system attributes have been stored.
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        default:
                // No implementation needed.
                break;
        }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
        ret_code_t err_code;

        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
        APP_ERROR_CHECK(err_code);

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {

        case LEDBUTTON_BUTTON_1:
                break;


        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                {LEDBUTTON_BUTTON_1, false, BUTTON_PULL, button_event_handler},
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);
}


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
        app_sched_execute();
        while(NRF_LOG_PROCESS());
        nrf_pwr_mgmt_run();
}

static void transfer_image_buffer(void * p_event_data, uint16_t size)
{
        uint32_t err_code;

        NRF_LOG_DEBUG("transfer_image_buffer offset %04x m_rx_image_buffer_len = %04x m_ble_its_max_data_len = %x", m_file_object.offset_req, m_rx_image_buffer_len, m_ble_its_max_data_len);
        err_code = ble_its_send_buffer(&m_its, m_rx_image_buffer, m_rx_image_buffer_len, m_ble_its_max_data_len);
        APP_ERROR_CHECK(err_code);
        nrf_delay_ms(10);
        m_rx_image_buffer_len = 0;
}

static uint32_t decode_received_data_handle(serial_payload_file_cmd_t cmd, uint8_t *data, uint16_t len)
{
        uint32_t err_code = NRF_SUCCESS;
        switch (cmd)
        {
        case PAYLOAD_FILE_OPCODE_FILEINFO_RSP:
                m_file_object.filesize = *data;
                m_file_object.filesize += *(data+1) << 8;
                m_file_object.filesize += *(data+2) << 16;
                m_file_object.filesize += *(data+3) << 24;
                m_file_object.offset_req = 0;

                // m_file_object.filesize  = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
                NRF_LOG_INFO("File Size = %04x", m_file_object.filesize);

                ble_its_img_info_t image_info;
                image_info.file_size_bytes = m_file_object.filesize;
                ble_its_img_info_send(&m_its, &image_info);

                data += 4;      // shift the crc32
                m_file_object.crc32 = *data;
                m_file_object.crc32 += *(data+1) << 8;
                m_file_object.crc32 += *(data+2) << 16;
                m_file_object.crc32 += *(data+3) << 24;

                NRF_LOG_INFO("CRC32  = %04x", m_file_object.crc32);

                m_file_object.offset_req = 0;

                serial_uart_response(PAYLOAD_FILE_OPCODE_DATA_REQ);
                break;

        case PAYLOAD_FILE_OPCODE_DATA_RSP:
        {
                NRF_LOG_DEBUG("PAYLOAD_FILE_OPCODE_DATA_RSP %x, %x, %x", m_file_object.offset_req, m_rx_image_buffer_len, len);
                if (m_rx_image_buffer_len > IMAGE_BUFFER_SIZE)
                {
                        err_code = NRF_ERROR_DATA_SIZE;
                        APP_ERROR_CHECK(err_code);
                }
                memcpy(m_rx_image_buffer+m_rx_image_buffer_len, data, len);
                m_rx_image_buffer_len = m_rx_image_buffer_len + len;

        }
        break;

        case PAYLOAD_FILE_OPCODE_DATA_RSP_LAST:

                NRF_LOG_DEBUG("PAYLOAD_FILE_OPCODE_DATA_RSP_LAST %x, %x, %x", m_file_object.offset_req, m_rx_image_buffer_len, len);
                if (m_rx_image_buffer_len > IMAGE_BUFFER_SIZE)
                {
                        err_code = NRF_ERROR_DATA_SIZE;
                        APP_ERROR_CHECK(err_code);
                }

                memcpy(m_rx_image_buffer+m_rx_image_buffer_len, data, len);
                m_rx_image_buffer_len = m_rx_image_buffer_len + len;

                // send the notification
                uint8_t resultData[1];
                resultData[0] = APP_CMD_SEND_BUFFER_REQ;
                m_its.data_handler(&m_its, resultData, 1);

                break;

        default:
                break;
        }

        return err_code;
}



static void serial_uart_response(serial_payload_file_cmd_t cmd)
{
        uint8_t tx_buffer[16];
        uint16_t tx_buffer_len = 0;
        ret_code_t err_code;
        NRF_LOG_DEBUG("serial_uart_response %x", cmd);

        switch (cmd)
        {
        case PAYLOAD_FILE_OPCODE_PING:
                tx_buffer_len = 3;
                tx_buffer[0] =  tx_buffer_len;
                tx_buffer[1] =  PAYLOAD_FILE_OPCODE_PING;
                tx_buffer[2] =  m_ble_mtu_length;
                err_code = nrf_drv_uart_tx(&m_uart, tx_buffer, tx_buffer_len);
                APP_ERROR_CHECK(err_code);
                break;

        case PAYLOAD_FILE_OPCODE_MTU_REQ:
                tx_buffer_len = 3;
                tx_buffer[0] =  tx_buffer_len;
                tx_buffer[1] =  PAYLOAD_FILE_OPCODE_MTU_REQ;
                tx_buffer[2] =  m_ble_mtu_length;
                err_code = nrf_drv_uart_tx(&m_uart, tx_buffer, tx_buffer_len);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_HEXDUMP_INFO(tx_buffer, tx_buffer_len);
                break;

        case PAYLOAD_FILE_OPCODE_DATA_REQ:
                m_data_request_obj.payload_len = 10;
                m_data_request_obj.opcode = PAYLOAD_FILE_OPCODE_DATA_REQ;
                m_data_request_obj.offset_addr = m_file_object.offset_req;
                if (m_file_object.filesize > m_data_request_obj.offset_addr + IMAGE_BUFFER_SIZE)
                {
                        m_data_request_obj.request_len = IMAGE_BUFFER_SIZE;
                }
                else
                {
                        m_data_request_obj.request_len = m_file_object.filesize - m_file_object.offset_req;
                }
                NRF_LOG_DEBUG("PAYLOAD_FILE_OPCODE_DATA_REQ offset_addr= %x, %x", m_data_request_obj.offset_addr, m_data_request_obj.request_len);
                err_code = nrf_drv_uart_tx(&m_uart, (uint8_t *)&m_data_request_obj, sizeof(m_data_request_obj));
                APP_ERROR_CHECK(err_code);
                NRF_LOG_HEXDUMP_DEBUG(&m_data_request_obj, sizeof(m_data_request_obj));
                m_file_object.offset_req += m_data_request_obj.request_len;
                break;
        default:
                break;

        }

}

static void serial_uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
{
        static uint8_t data_array[BLE_ITS_MAX_DATA_LEN];
        static uint8_t index = 0;
        static uint8_t total_len;
        static uint8_t is_first_byte = 1;
        static uint8_t is_end_byte = 0;
        serial_payload_file_cmd_t op_code;
        uint32_t err_code = NRF_SUCCESS;

        switch (p_event->type)
        {
        case NRF_DRV_UART_EVT_RX_DONE:


                NRF_LOG_DEBUG("data %02x, len %02d", p_event->data.rxtx.p_data[0], p_event->data.rxtx.bytes);
                if (is_first_byte == 1)
                {
                        total_len = p_event->data.rxtx.p_data[0];
                        is_first_byte = 0;
                        index++;
                        err_code = nrf_drv_uart_rx(&m_uart, m_rx_byte, total_len);
                        APP_ERROR_CHECK(err_code);
                }
                else
                {
                        op_code = p_event->data.rxtx.p_data[0];
                        memcpy(data_array,  p_event->data.rxtx.p_data+1, p_event->data.rxtx.bytes-1);

                        err_code = nrf_drv_uart_rx(&m_uart, m_rx_byte, 1);

                        index = 0;
                        is_end_byte = 1;
                }
                if (is_end_byte)
                {
                        decode_received_data_handle(op_code, data_array, total_len-1);
                        is_end_byte = 0;
                        is_first_byte = 1;
                }
                break;

        case NRF_DRV_UART_EVT_ERROR:
                APP_ERROR_HANDLER(p_event->data.error.error_mask);
                break;

        default:
                // No action.
                break;
        }
}


static uint32_t serial_file_transport_init(void)
{

        uint32_t err_code = NRF_SUCCESS;
        nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
        NRF_LOG_DEBUG("serial_file_transport_init()");
        uart_config.pseltxd   = TX_PIN_NUMBER;
        uart_config.pselrxd   = RX_PIN_NUMBER;
        uart_config.pselcts   = CTS_PIN_NUMBER;
        uart_config.pselrts   = RTS_PIN_NUMBER;
        uart_config.hwfc      = false ?
                                NRF_UART_HWFC_ENABLED : NRF_UART_HWFC_DISABLED;
        uart_config.p_context = NULL;

        err_code =  nrf_drv_uart_init(&m_uart, &uart_config, serial_uart_event_handler);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("Failed initializing uart");
                return err_code;
        }
        err_code = nrf_drv_uart_rx(&m_uart, m_rx_byte, 1);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("Failed initializing rx");
        }
        m_active = true;

        return err_code;
}


static uint32_t serial_file_transport_close(void)
{
        if (m_active == true)
        {
                nrf_drv_uart_uninit(&m_uart);
                m_active = false;
        }
        return NRF_SUCCESS;
}

/**@brief Function for application main entry.
 */
int main(void)
{

        bool erase_bonds;
        ret_code_t err_code;

#if defined(BOARD_PCA10056)
        /* enable instruction cache */
        NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos) +
                              (NVMC_ICACHECNF_CACHEPROFEN_Disabled << NVMC_ICACHECNF_CACHEPROFEN_Pos);
#endif

        // Initialize.
        log_init();
        leds_init();
        timers_init();
        buttons_init();
        power_management_init();
        ble_stack_init();
        scheduler_init();
        gap_params_init();
        gatt_init();
        services_init();
        advertising_init();
        conn_params_init();

        // Start execution.
        NRF_LOG_INFO("Peripheral JPG Transfer + NUS.");
        advertising_start();

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);

        serial_file_transport_init();

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}


/**
 * @}
 */
