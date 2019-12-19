/*
 * Indoor Air Quality via MQTT with BME680 (I2C) and ESP32
 * I made this code to get some sensor values to my home mqtt server, so I can check how bad the air currently is.
 * This code is best described as "messy proof of concept", so please do not expect fancy things like error checking :-)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp32/ulp.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#define CONFIG_BROKER_URL "mqtt://192.168.178.115:1883"
esp_mqtt_client_handle_t client;
bool mqtt_connected = false;

#include "nvs_flash.h"
#include "nvs.h"
#define STORAGE_NAMESPACE "storage"

#include "driver/i2c.h"
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

#include "firmware_connection.h"

#include "bsec_integration.h"
#include "bsec_serialized_configurations_iaq.h"


// forward declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void bsec_start();

/// main function
void app_main() {
    printf("Starting!\n");

    // i2c init code
    i2c_config_t i2config;
    i2config.mode = I2C_MODE_MASTER;
    i2config.sda_io_num = 4;
    i2config.scl_io_num = 15;
    i2config.sda_pullup_en = 1;
    i2config.scl_pullup_en = 1;
    i2config.master.clk_speed = 100000L;
    i2c_param_config(I2C_NUM_0, &i2config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    // quick i2c scan - this should find a device at 0x77
    uint8_t tmp_data = 0;
    printf("Scanning...");
    for(uint8_t i = 1; i < 127; i++) {
        printf(".");
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, i << 1 | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, i << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &tmp_data, NACK_VAL);

        i2c_master_stop(cmd);

        if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000) == ESP_OK) {
            printf("%04x", i);
        }
        i2c_cmd_link_delete(cmd);
    }
    printf("done\n");


    // setting up the nvs (flash storage)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        printf("Flash erased!\n");
    }
    ESP_ERROR_CHECK( err );


    // setting up wifi and tcp/ip
    esp_event_loop_create_default();
    tcpip_adapter_init();
    printf("Wifi status: %s", esp_err_to_name(wifi_connect()));
    esp_mqtt_client_config_t mqtt_cfg = {
            .uri = CONFIG_BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);


    bsec_start();
    for(;;);
}

/// because reasons
int64_t myGetMicros() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000000LL + tv.tv_usec);
}

// MQTT and wifi code

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    //int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT connected.\n");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            printf("MQTT trying to connect...\n");
            break;
        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT disconnected.\n");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            break;
        case MQTT_EVENT_PUBLISHED:
            printf("MQTT published.\n");
            break;
        case MQTT_EVENT_DATA:
            printf("MQTT data event.\n");
            break;
        case MQTT_EVENT_ERROR:
            printf("Unspecified MQTT_error.\n");
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    printf("Event dispatched from event loop base=%s, event_id=%d\n", base, event_id);
    mqtt_event_handler_cb(event_data);
}




// Modified BSEC example code

/*!
 * @brief           Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    for (int i = 0; i < data_len; i++) {
        i2c_master_write_byte(cmd, reg_data_ptr[i], ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    return 0;
}

/*!
 * @brief           Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if(data_len > 1) i2c_master_read(cmd, reg_data_ptr, data_len-1, ACK_VAL);

    i2c_master_read_byte(cmd, reg_data_ptr + data_len - 1, NACK_VAL);

    i2c_master_stop(cmd);


    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    return 0;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void mysleep(uint32_t t_ms)
{
    vTaskDelay(t_ms * (configTICK_RATE_HZ / 1000));

    // doesn't work for some reason - maybe try light sleep to keep ram?
    //esp_sleep_enable_timer_wakeup(t_ms * 1000);
    //esp_deep_sleep_start();
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    return myGetMicros();
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                  float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
                  float static_iaq, float co2_equivalent, float breath_voc_equivalent, float gas_percentage)
{
    printf("Output ready\n");
// print all
//    printf("Timestamp [ms]: %lu, raw temperature [°C]: %f, pressure [Pa]: %f, raw relative humidity [%%]: %f, "
//           "gas [kOhm]: %f, IAQ: %f, IAQ accuracy: %i, temperature [°C]: %f, relative humidity [%%]: %f, Static IAQ: %f, "
//           "CO2 equivalent: %f, breath VOC equivalent: %f \n",
//           (long)(timestamp/1000), raw_temperature, pressure, raw_humidity, (gas/1000), iaq, iaq_accuracy, temperature,
//           humidity, static_iaq, co2_equivalent, breath_voc_equivalent);

// print most important
    printf("temperature [C]: %f, relative humidity [%%]: %f, pressure [Pa]: %f, IAQ: %f, IAQ accuracy: %i, "
           "static IAQ: %f, CO2 equivalent: %f, breath VOC equivalent: %f, gas [%%]: %f\n",
           temperature, humidity, pressure, iaq, iaq_accuracy, static_iaq, co2_equivalent, breath_voc_equivalent,
           gas_percentage);

// mqtt
// values are rounded to sensible length
    if(mqtt_connected) {
        printf("Sending mqtt message...\n");
        char payload_string[64];
        sprintf(payload_string, "%.1f", temperature);
        esp_mqtt_client_publish(client, "/envsens1/temperature", payload_string, 0, 1, 0);
        sprintf(payload_string, "%.0f", humidity);
        esp_mqtt_client_publish(client, "/envsens1/humidity", payload_string, 0, 1, 0);
        sprintf(payload_string, "%.0f", pressure/100);
        esp_mqtt_client_publish(client, "/envsens1/pressure", payload_string, 0, 1, 0);
        sprintf(payload_string, "%.0f", static_iaq);
        esp_mqtt_client_publish(client, "/envsens1/iaq", payload_string, 0, 1, 0);
        sprintf(payload_string, "%i", iaq_accuracy);
        esp_mqtt_client_publish(client, "/envsens1/iaq_accuracy", payload_string, 0, 1, 0);
        printf("Done.\n");
    }
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    printf("Reading state buffer...\n");
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) goto error;

    // Read previously saved blob if available
    err = nvs_get_blob(my_handle, "bme680_state", state_buffer, &n_buffer);
    if (err != ESP_OK) goto error;

    // Close
    nvs_close(my_handle);
    printf("Done.\n");
    return n_buffer;

    error:
    printf("Error while reading from flash: %s!\n", esp_err_to_name(err));
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    printf("Writing state buffer...\n");
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) goto error;

    // Write
    err = nvs_set_blob(my_handle, "bme680_state", state_buffer, length);
    if (err != ESP_OK) goto error;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) goto error;

    // Close
    nvs_close(my_handle);
    printf("Done.\n");
    return;

    error:
    printf("Error while writing to flash: %s!\n", esp_err_to_name(err));
    return;
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    if(n_buffer < sizeof(bsec_config_iaq)) goto error;
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));
    return sizeof(bsec_config_iaq);

    error:
    printf("Error loading config, size mismatch!");
    return 0;
}

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
void bsec_start() {
    return_values_init ret;

    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, mysleep, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        printf("BME680 Error: %i", (int)ret.bme680_status);

    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        printf("BSEC Error: %i", (int)ret.bme680_status);
    }
    printf("BSEC init done...\n");


    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(mysleep, get_timestamp_us, output_ready, state_save, 10000);

}