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

#include "ble_nus.h"
#include "ble_image_transfer_service.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// #include "test_image.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_balloc.h"
#include "nrf_drv_uart.h"
#include "app_fifo.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */

#define LEDBUTTON_BUTTON_1              BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON_2              BSP_BUTTON_1                            /**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON_3              BSP_BUTTON_2                            /**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON_4              BSP_BUTTON_3                            /**< Button that will trigger the notification event with the LED Button Service */


#define DEVICE_NAME                     "S112JPG"                         /**< Name of device. Will be included in the advertising data. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

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
#define SCHED_QUEUE_SIZE                    10                                         /**< Maximum number of events in the scheduler queue. */
#endif

#define TX_POWER_LEVEL                  (4)                                    /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */


#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
BLE_ITS_DEF(m_its, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE JPG Transfer service instance. */

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */


static uint8_t m_new_command_received = 0;
static uint8_t m_new_resolution, m_new_phy;
static bool m_stream_mode_active = false;
static ble_its_ble_params_info_t m_ble_params_info = {20, 50, 1, 1};


static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static uint16_t m_ble_mtu_length = BLE_GATT_ATT_MTU_DEFAULT - 3;
//use of the data transfer
static uint8_t img_data_buffer[BLE_ITS_MAX_DATA_LEN];
static uint32_t img_data_length = BLE_ITS_MAX_DATA_LEN;
static uint8_t m_buffer_transfer_index = 0;
// static bool test_flag = false;
// static bool start_thoughput_test = false;
// static uint32_t frameCount = 0;
// static uint32_t frameCount_previous = 0;

static nrf_drv_uart_t m_uart =  NRF_DRV_UART_INSTANCE(0);
static uint8_t m_rx_byte;
static bool m_active;


static uint8_t m_tx_buf[BLE_GATT_ATT_MTU_DEFAULT];
static uint8_t m_rx_buf[BLE_GATT_ATT_MTU_DEFAULT];

enum {
        APP_CMD_NOCOMMAND = 0,
        APP_CMD_SINGLE_CAPTURE,
        APP_CMD_START_STREAM,
        APP_CMD_STOP_STREAM,
        APP_CMD_CHANGE_RESOLUTION,
        APP_CMD_CHANGE_PHY,
        APP_CMD_SEND_BLE_PARAMS
};



// typedef struct nrf_jpg_file_info_s
// {
//         uint32_t file_size_bytes;
//         bool isGetFileSize;
//         uint32_t offset;
// } nrf_jpg_file_info_t;
//
// nrf_jpg_file_info_t m_jpg_file =      \
// {                                     \
//         .file_size_bytes = 0,         \
//         .isGetFileSize = false,       \
//         .offset = 0                   \
// };


typedef enum {
        // PAYLOAD_JPG_HEADER_FILESIZE = 0xAA,
        // PAYLOAD_JPG_HEADER_CRC32    = 0xAB,
        // PAYLOAD_JPG_HEADER_DATA_FIRST_RPN = 0xAC,
        // PAYLOAD_JPG_HEADER_DATA     = 0xAD,
        // PAYLOAD_JPG_HEADER_MTU_RSP  = 0xAE,
        // PAYLOAD_JPG_HEADER_DONE_RSP = 0xAF,

        PAYLOAD_JPG_OPCODE_PING         = 0xA0,
        PAYLOAD_JPG_OPCODE_FILEINFO_RSP = 0xA1,
        PAYLOAD_JPG_OPCODE_MTU_REQ      = 0xA2,
        PAYLOAD_JPG_OPCODE_MTU_RSP      = 0xA3,
        PAYLOAD_JPG_OPCODE_DATA_REQ     = 0xA4,
        PAYLOAD_JPG_OPCODE_DATA_RSP     = 0xA5,
        PAYLOAD_JPG_OPCODE_COMPLETE     = 0xA6,
        PAYLOAD_JPG_OPCODE_COMPLETE_ACK = 0xA7,
} payload_jpg_cmd_type;

#define PAYLOAD_JPG_OPCODE 0


// static uint32_t fileSize = 0;
static uint32_t crc32    = 0;
static uint32_t jpg_offset = 0;
static ble_its_img_info_t m_image_info;
//static uint8_t test_image[100000];
static uint32_t test_image_length = 0;

// static uint8_t jpg_image_buffer[2048];
// static uint32_t jpg_image_buffer_length = 0;

static app_fifo_t m_rx_fifo;                                                /**< RX FIFO buffer for storing data received on the UART until the application fetches them using app_uart_get(). */

// Create a buffer for the FIFO
//static uint16_t image_fifo_buffer_size = 2048;
#define FIFO_BUFFER_SIZE    2048
uint8_t image_fifo_buffer[FIFO_BUFFER_SIZE];
uint8_t image_read_buffer[FIFO_BUFFER_SIZE];

static uint32_t create_circular_buffer(void)
{
        ret_code_t err_code;
        // Configure buffer RX buffer.
        err_code = app_fifo_init(&m_rx_fifo, image_fifo_buffer, (uint16_t)sizeof(image_fifo_buffer));
        VERIFY_SUCCESS(err_code);
        return err_code;
}

static uint32_t write_circular_buffer(uint8_t const * p_byte_array, uint32_t * p_size)
{
        uint32_t err_code = app_fifo_write(&m_rx_fifo, p_byte_array, p_size);
        return err_code;
}

static uint32_t read_circular_buffer(uint8_t * p_byte_array, uint32_t * p_size)
{
        uint32_t err_code = app_fifo_read(&m_rx_fifo, p_byte_array, p_size);
        return err_code;
}

static void jpg_send_image_info(void * p_event_data, uint16_t size)
{
        ret_code_t err_code;
        if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        {
                err_code = ble_its_img_info_send(&m_its, &m_image_info);
                APP_ERROR_CHECK(err_code);
        }
}

static void jpg_send_file_fragment(void * p_event_data, uint16_t size)
{
        ret_code_t err_code;
        uint32_t img_data_length = 244;
        uint8_t img_data_buffer[255];

        //err_code = read_circular_buffer(img_data_buffer, &img_data_length);
        err_code = app_fifo_read(&m_rx_fifo, &img_data_buffer, &img_data_length);
        APP_ERROR_CHECK(err_code);

        do
        {
                err_code = ble_its_send_file_fragment(&m_its, img_data_buffer, img_data_length);
                if(err_code == NRF_SUCCESS)
                {
                        img_data_length = 0;
                }
        } while(err_code == NRF_SUCCESS);
        NRF_LOG_INFO("Fifo Read %04x", img_data_length);
}

void transport_serial_tx_command_handler(payload_jpg_cmd_type cmd_type)
{
        uint8_t tx_buffer[16];
        uint32_t length = 0;
        ret_code_t err_code;
        switch (cmd_type)
        {
        case PAYLOAD_JPG_OPCODE_PING:
                tx_buffer[0] = PAYLOAD_JPG_OPCODE_PING;
                app_uart_put(tx_buffer[0]);
                break;
        case PAYLOAD_JPG_OPCODE_FILEINFO_RSP:
                tx_buffer[0] = PAYLOAD_JPG_OPCODE_FILEINFO_RSP;
                app_uart_put(tx_buffer[0]);
                break;
        case PAYLOAD_JPG_OPCODE_MTU_RSP:
                tx_buffer[0] = PAYLOAD_JPG_OPCODE_MTU_RSP;
                tx_buffer[1] = (uint8_t) m_ble_mtu_length;
                for (i=0; i<2; i++)
                {
                        do {
                                /* code */
                                err_code  = app_uart_put(tx_buffer[i]);
                        } while(err_code == NRF_SUCCESS);
                }
                break;
        case PAYLOAD_JPG_OPCODE_DATA_REQ:
                tx_buffer[0] = (uint8_t) offset & 0xFF;
                tx_buffer[1] = (uint8_t) (offset >> 8) & 0xFF;
                tx_buffer[2] = (uint8_t) (offset >> 16) & 0xFF;
                tx_buffer[3] = (uint8_t) (offset >> 24) & 0xFF;

                for (i=0; i<2; i++)
                {
                        do {
                                /* code */
                                err_code  = app_uart_put(tx_buffer[i]);
                        } while(err_code == NRF_SUCCESS);
                }
                break;
        case PAYLOAD_JPG_OPCODE_COMPLETE_ACK:
                break;
        case PAYLOAD_JPG_HEADER_MTU_RSP:
                tx_buffer[0] = PAYLOAD_JPG_HEADER_MTU_RSP;
                tx_buffer[1] = m_ble_its_max_data_len;
                for (uint32_t i = 0; i < 2; i++)
                {
                        do
                        {
                                err_code = app_uart_put(tx_buffer[i]);
                                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                                {
                                        NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                                        APP_ERROR_CHECK(err_code);
                                }
                        } while (err_code == NRF_ERROR_BUSY);
                }
                break;
        }
}

void rpc_command_handler(uint8_t* cmd_data, uint8_t cmd_length)
{
        ret_code_t err_code;
        uint32_t fileSize = 0;
        uint8_t number_prn;
        uint8_t x;
        switch(cmd_data[PAYLOAD_JPG_OPCODE])
        {
        case PAYLOAD_JPG_HEADER_FILESIZE:
                fileSize = cmd_data[1] + (cmd_data[2] << 8) + (cmd_data[3] << 16) + (cmd_data[4] << 24);
                NRF_LOG_DEBUG("File Size = %08x", fileSize);
                NRF_LOG_HEXDUMP_INFO(cmd_data, cmd_length);
                test_image_length = fileSize;
                m_image_info.file_size_bytes = fileSize;
                err_code = app_sched_event_put(NULL, 0, jpg_send_image_info);
                APP_ERROR_CHECK(err_code);
                number_prn = cmd_data[5];
                break;
        case PAYLOAD_JPG_HEADER_CRC32:
                crc32 =cmd_data[1] + (cmd_data[2] << 8) + (cmd_data[3] << 16) + (cmd_data[4] << 24);
                break;
        case PAYLOAD_JPG_HEADER_DATA:
        {
                uint32_t length = cmd_length-1;
                uint8_t *p_data = cmd_data + 1;
                // err_code = write_circular_buffer(&cmd_data[1],&length);

                err_code = app_fifo_write(&m_rx_fifo, p_data, &length);
                if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
                {
                        err_code = app_sched_event_put(NULL, 0, jpg_send_file_fragment);
                        // err_code = ble_its_send_file_fragment(&m_its, &cmd_data[1], cmd_length-1);
                        APP_ERROR_CHECK(err_code);
                }
                jpg_offset += cmd_length-1;
                NRF_LOG_INFO("offset = %04x, %04x", jpg_offset, length);
        }       break;
        case PAYLOAD_JPG_HEADER_MTU_RSP:
                break;
        case PAYLOAD_JPG_HEADER_DONE_RSP:
                break;
        default:
                break;
        }
}

/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint8_t index = 0;
        static uint8_t total_len;
        static uint8_t is_first_byte = 1;
        uint32_t err_code;

        switch (p_event->evt_type)
        {
        case APP_UART_DATA_READY:
                if(is_first_byte == 1) {
                        UNUSED_VARIABLE(app_uart_get(&total_len));
                        is_first_byte = 0;
                        NRF_LOG_DEBUG("wait cmd length = %02x\n", total_len);
                }else{
                        UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                        NRF_LOG_DEBUG("got %d, data = %02x\n", index, data_array[index]);
                        index++;
                }
                if(index == total_len) {
                        //NRF_LOG_HEXDUMP_INFO(data_array, index);
                        NRF_LOG_INFO("Payload %02d, %04d", index, jpg_offset);
                        rpc_command_handler(data_array, index);
                        index = 0;
                        is_first_byte = 1;
                }
                break;

        case APP_UART_COMMUNICATION_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}

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


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
        // Initialize timer module, making it use the scheduler
        ret_code_t err_code = app_timer_init();
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


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
        if (p_evt->type == BLE_NUS_EVT_RX_DATA)
        {
                uint32_t err_code;

                for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
                {
                        do
                        {
                                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                                {
                                        NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                                        APP_ERROR_CHECK(err_code);
                                }
                        } while (err_code == NRF_ERROR_BUSY);
                }
                if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
                {
                        while (app_uart_put('\n') == NRF_ERROR_BUSY);
                }
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
        {
                NRF_LOG_INFO("NUS : BLE_NUS_EVT_COMM_STARTED");
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
        {
                NRF_LOG_INFO("NUS : BLE_NUS_EVT_COMM_STOPPED");
        }
}



/**@snippet [Handling the data received over BLE] */

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_mtu_length = data_length;//m_ble_params_info.mtu = m_ble_its_max_data_len;

                NRF_LOG_INFO("gatt_event: ATT MTU is set to 0x%X (%d)", data_length, data_length);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                NRF_LOG_INFO("gatt_evt_handler: Data len is set to 0x%X (%d)", data_length, data_length);
        }
        //NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
        //            p_gatt->att_mtu_desired_central,
        //        p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

#if !defined(S112)
        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
#endif

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

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

        ble_uuid_t adv_uuids[] = {{BLE_UUID_ITS_SERVICE, NUS_SERVICE_UUID_TYPE}};

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





/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
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
/**@snippet [UART Initialization] */




void its_data_file_test(void * p_event_data, uint16_t size)
{
        uint32_t err_code;
        // NRF_LOG_INFO("_actest_image %04d", sizeof(_actest_image));
        NRF_LOG_INFO("its_data_file_test %08d", test_image_length);

//        err_code = ble_its_send_file(&m_its, _actest_image, sizeof(_actest_image), m_ble_its_max_data_len);
//        err_code = ble_its_send_file(&m_its, test_image, test_image_length, m_ble_its_max_data_len);
//        APP_ERROR_CHECK(err_code);

        // test_image_length=0;
}




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

                jpg_send_command_handler(PAYLOAD_JPG_HEADER_MTU_RSP);
                // err_code = app_sched_event_put(NULL, 0, its_data_file_test);
                // APP_ERROR_CHECK(err_code);


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
        ble_nus_init_t nus_init;
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


                tx_power_set();

                break;
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
//            m_conn_interval_configured = true;
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

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:

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
                if (button_action == APP_BUTTON_PUSH)
                {

                        err_code = app_sched_event_put(NULL, 0, its_data_file_test);
                        APP_ERROR_CHECK(err_code);

                        // jpg_file_command_handler();
//                        NRF_LOG_INFO("UART init");
//                        transfer_serial_uart_init();
                }
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
                {LEDBUTTON_BUTTON_2, false, BUTTON_PULL, button_event_handler},
                {LEDBUTTON_BUTTON_3, false, BUTTON_PULL, button_event_handler},
                {LEDBUTTON_BUTTON_4, false, BUTTON_PULL, button_event_handler},

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

/**@snippet [Handling the data received over UART] */

// uint32_t transfer_serial_uart_init(void)
// {
//         uint32_t err_code = NRF_SUCCESS;
//
//         if (m_active)
//         {
//                 return err_code;
//         }
//         m_active = true;
//
//         NRF_LOG_DEBUG("serial_transport_init()");
//
// //        err_code = nrf_balloc_init(&m_payload_pool);
// //        APP_ERROR_CHECK(err_code);
//
//         // m_serial.rsp_func           = rsp_send;
//         // m_serial.payload_free_func  = payload_free;
//         // m_serial.mtu                = UART_SLIP_MTU;
//         // m_serial.p_rsp_buf          = &m_rsp_buf[NRF_MAX_RESPONSE_SIZE_SLIP -
//         //                                          NRF_SERIAL_MAX_RESPONSE_SIZE];
//
//         nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
//
//         uart_config.pseltxd   = TX_PIN_NUMBER;
//         uart_config.pselrxd   = RX_PIN_NUMBER;
//         uart_config.pselcts   = CTS_PIN_NUMBER;
//         uart_config.pselrts   = RTS_PIN_NUMBER;
//         uart_config.hwfc      = NRF_SERIAL_UART_USES_HWFC ?
//                                 NRF_UART_HWFC_ENABLED : NRF_UART_HWFC_DISABLED;
//         // uart_config.p_context = &m_serial;
//
//         err_code =  nrf_drv_uart_init(&m_uart, &uart_config, uart_event_handler);
//         if (err_code != NRF_SUCCESS)
//         {
//                 NRF_LOG_ERROR("Failed initializing uart");
//                 return err_code;
//         }
//
//         err_code = nrf_drv_uart_rx(&m_uart, &m_rx_byte, 1);
//         if (err_code != NRF_SUCCESS)
//         {
//                 NRF_LOG_ERROR("Failed initializing rx");
//         }
//         return err_code;
//
// }
//
// uint32_t transfer_serial_uart_close(void)
// {
//         if ((m_active == true))
//         {
//                 nrf_drv_uart_uninit(&m_uart);
//                 m_active = false;
//         }
//
//         return NRF_SUCCESS;
// }

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
        uart_init();
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
        create_circular_buffer();
        // Start execution.
        NRF_LOG_INFO("Channel Map Update Example : Peripheral JPG Transfer + NUS.");
        advertising_start();

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);


        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}


/**
 * @}
 */
