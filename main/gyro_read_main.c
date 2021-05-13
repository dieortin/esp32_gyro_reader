/**
 * @file gyro_read_main.c
 * @author Diego Ortín Fernández
 * @date 12-5-2021
 * @brief This file contains the main code for the IMU reader module. It reads accelerometer data from an MPU6050 and relays
 * it over UDP to the other module. It also varies the pitch of a buzzer depending on the current inclination.
 */
#include <stdio.h>
#include <sys/param.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include <freertos/event_groups.h>
#include "freertos/task.h"
#include <hal/ledc_types.h>
#include <driver/ledc.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#include "mpu6050/mpu6050.h"
#include <buzzer/buzzer.h>


// WIFI ----------------------------------------------------------------------------------------------
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID ///< SSID of the Wi-Fi network
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD ///< Password of the Wi-Fi network
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY ///< Maximum number of retries when connecting to Wi-Fi

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *WIFI_TAG = "wifi station";

static int s_retry_num = 0; ///< Number of retries



// I2C -------------------------------------------------------------------------------------------------

#define I2C_FREQ CONFIG_MPU_I2C_CLOCK_FREQ
#define I2C_SDA_PIN CONFIG_MPU_I2C_SDA_GPIO
#define I2C_SCL_PIN CONFIG_MPU_I2C_SCL_GPIO

static const char *MAIN_TAG = "Main";

static const char *GYRO_TAG = "GyroReading";
TaskHandle_t GyroTaskHandle = NULL;

static const char *UDP_TAG = "UdpComms";
TaskHandle_t UdpTaskHandle = NULL;

static const char *BUZZER_TAG = "Buzzer";
TaskHandle_t BuzzerTaskHandle = NULL;

/// Values for self-test and calibration of the MPU
float self_test[6] = {0};
float accel_calibration[3] = {0};
float gyro_calibration[3] = {0};

#define GYRO_SAMPLING_RATE 20

buzzer_t *buzzer = NULL; ///< Global variable to store the buzzer object


// UDP --------------------------------------------------------------------------------------------------

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR ///< IP where the data must be sent
#endif

#define PORT CONFIG_EXAMPLE_PORT ///< Port where the data must be sent


// GPIO -------------------------------------------------------------------------------------------------
#define BUZZER_PIN CONFIG_BUZZER_GPIO ///< GPIO pin where the buzzer is connected


/**
 * Calculates the pitch of an accelerometer given its values for the three cordinates of space.
 *
 * Algorithm for pitch calculation taken from:
 * https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
 * @param accel_x Accelerometer value for the X axis
 * @param accel_y Accelerometer value for the Y axis
 * @param accel_z Accelerometer value for the Z axis
 * @return double representing the pitch of the accelerometer in degrees
 */
double calculate_pitch(int accel_x, int accel_y, int accel_z) {
    double x_buffer = (double) accel_x;
    double y_buffer = (double) accel_y;
    double z_buffer = (double) accel_z;

    double pitch = atan2((-x_buffer), sqrt(y_buffer * y_buffer + z_buffer * z_buffer)) * 57.3;
    return pitch;
}


/**
 * This function handles interrupts coming from the INT pin in the MPU6050.
 * When the interrupt occurs, it sends a notification to the gyroscope task so it can read the new value from the MPU.
 */
static void IRAM_ATTR mpu_int_handler() {
    //ESP_EARLY_LOGI("INT", "Interrupt!\n");
    xTaskGenericNotifyFromISR(GyroTaskHandle, 1, eNoAction, NULL, NULL);
    //xTaskNotifyFromISR(GyroTaskHandle, 1, eSetValueWithoutOverwrite, NULL);
}

/**
 * This function is a FreeRTOS task, which handles playing a different sound out of the buzzer depending on the current
 * pitch angle of the MPU.
 */
void buzzerTask() {
    //int32_t lastValue = 0;
    uint8_t last_final_note = 0;
    while (1) {
        uint32_t newvalue;

        /// Wait until receiving a notification with the new pitch value
        if (xTaskNotifyWait(0, 0, &newvalue, 0xFFFF) == pdPASS) {
            int32_t val = (int32_t) newvalue;
            //if (newvalue == lastValue) continue;
            //lastValue = newvalue;
            val += 90;
            uint8_t deg_per_note = 180 / BUZZER_NOTE_MAX;
            uint8_t final_note = val / deg_per_note;
            if (final_note == last_final_note) continue;
            last_final_note = final_note;
            //buzzer_pause(buzzer);
            buzzer_set_note(buzzer, final_note, 4); /// Play the note
            buzzer_play(buzzer);
        } else {
            ESP_LOGW(BUZZER_TAG, "Time run out while waiting for notification, waiting again...");
        }
    }
}

/**
 * This function is a FreeRTOS task, which handles sending the calculated pitch data to the server via UDP.
 */
void udpTask() {
    char host_ip[] = HOST_IP_ADDR; ///< IP where we want to send the data
    int addr_family = 0; ///< Address family to use
    int ip_protocol = 0;

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#endif
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol); /// Create an UDP socket
        if (sock < 0) {
            ESP_LOGE(UDP_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_TAG, "Socket created, sending to %s:%d", host_ip, PORT);

        while (1) {
            uint32_t newvalue;
            if (xTaskNotifyWait(0, 0, &newvalue, 0xFFFF) == pdPASS) {
                char payload[100];
                snprintf(payload, 100, "%i\n", (int32_t) newvalue);

                /// Send via UDP to the server
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(UDP_TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            } else {
                ESP_LOGW(UDP_TAG, "Time run out while waiting for notification, waiting again...");
            }
        }

        if (sock != -1) {
            ESP_LOGE(UDP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

/**
 * This function is a FreeRTOS task. It handles reading new accelerometer data from the MPU when it gets a message via
 * interrupt, and notifying the gyroscope and buzzer tasks.
 */
void gyroTask() {
    for (;;) {
        /// Wait until there's new data to be read in the MPU's FIFO queue (until we receive the interrupt)
        if (xTaskNotifyWait(0, 0, NULL, 0xFFFF) == pdPASS) {
            mpu6050_acceleration_t acceleration;

            mpu6050_get_acceleration(&acceleration); /// Obtain the accelerometer data

            //printf("Accelerometer: [x(%i), y(%i), z(%i)], ", acceleration.accel_x, acceleration.accel_y,
            //       acceleration.accel_z);
            //printf("Gyroscope: [x(%i), y(%i), z(%i)] ", rotation.gyro_x, rotation.gyro_y, rotation.gyro_z);

            /// Calculate the pitch using the accelerometer data
            double pitch = calculate_pitch(acceleration.accel_x, acceleration.accel_y, acceleration.accel_z);
            //ESP_LOGI(GYRO_TAG, "MPU data ready: acceleration=(%i, %i, %i), pitch= %lf", acceleration.accel_x, acceleration.accel_y, acceleration.accel_z, pitch);
            //printf("Pitch: [%lf]\n", pitch);

            /// Notify the other tasks, including the calculated pitch in the notification
            xTaskGenericNotify(UdpTaskHandle, (int32_t) pitch, eSetValueWithOverwrite, NULL);
            xTaskGenericNotify(BuzzerTaskHandle, (int32_t) pitch, eSetValueWithOverwrite, NULL);
        } else {
            ESP_LOGW(GYRO_TAG, "Time run out while waiting for notification, waiting again...");
        }
    }
}

/**
 * This function handles events related to Wi-Fi connectivity.
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(WIFI_TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:"
                IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * This function initializes Wi-Fi in station mode, connecting to the specified Access Point.
 */
void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = EXAMPLE_ESP_WIFI_SSID,
                    .password = EXAMPLE_ESP_WIFI_PASS,
                    /* Setting a password implies station will connect to all security modes including WEP/WPA.
                     * However these modes are deprecated and not advisable to be used. Incase your Access point
                     * doesn't support WPA2, these mode can be enabled by commenting below line */
                    .threshold.authmode = WIFI_AUTH_WPA2_PSK,

                    .pmf_cfg = {
                            .capable = true,
                            .required = false
                    },
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           (uint) WIFI_CONNECTED_BIT | (uint)WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & (uint) WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & (uint) WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

/**
 * This function initializes i2c for communication with the MPU.
 * @return ESP_FAIL if an error happened, ESP_OK otherwise
 */
static esp_err_t i2c_initialize() {
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_SDA_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_SCL_PIN,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_FREQ
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0); // TODO: flags??

    return ESP_OK;
}

/**
 * This function initializes the MPU, tests it and calibrates it depending on the options set.
 * It also initializes the GPIO pin for receiving interrupts from the MPU, and registers the interrup handler.
 * @return ESP_FAIL if an error ocurred, ESP_OK otherwise
 */
static esp_err_t gyro_initialize() {
    ESP_LOGI(mpu6050_get_tag(), "Initializing MPU6050...");

#ifdef CONFIG_MPU_SELFTEST
    ESP_LOGI(mpu6050_get_tag(), "Performing self-test...");
    mpu6050_self_test(self_test);

    // Check if self test passed for every axis
    if (self_test[0] >= 1.0f || self_test[1] >= 1.0f || self_test[2] >= 1.0f || self_test[3] >= 1.0f ||
        self_test[4] >= 1.0f || self_test[5] >= 1.0f) {
        ESP_LOGE(mpu6050_get_tag(), "Self-test failed!");
        return ESP_FAIL;
    }

    ESP_LOGI(mpu6050_get_tag(), "Passed self-test");
#endif

    mpu6050_reset();
#ifdef CONFIG_MPU_CALIBRATE
    ESP_LOGI(mpu6050_get_tag(), "Performing calibration...");
    mpu6050_calibrate(accel_calibration, gyro_calibration);
    ESP_LOGI(mpu6050_get_tag(), "Calibration successful");
#endif
    ESP_LOGI(mpu6050_get_tag(), "Initializing...");
    mpu6050_init();

    /// Set up the GPIO for the interrupt
    gpio_config_t gpio_conf = {
            .intr_type = GPIO_INTR_POSEDGE, /// Rising edge
            .pin_bit_mask =  (1ULL << (uint) CONFIG_MPU_INT_GPIO), /// Select the INT pin
            .pull_up_en = GPIO_PULLUP_ENABLE, /// Enable pull-up
            .pull_down_en = GPIO_PULLDOWN_DISABLE, /// Disable pull-down
            .mode = GPIO_MODE_INPUT /// Input mode
    };
    gpio_config(&gpio_conf);

    // /// Create an event queue for the interrupt events
    //gpio_event_queue = xQueueCreate(10, sizeof(uint32_t));

    /// Install GPIO interrupt service
    gpio_install_isr_service(0);

    /// Add the ISR handler for the INT pin
    gpio_isr_handler_add(CONFIG_MPU_INT_GPIO, mpu_int_handler, NULL);

    mpu6050_set_interrupt_mode(1); /// Set the interrupt mode to active low
    /// Enable the data ready interrupt to know when we have new readings
    mpu6050_set_int_data_ready_enabled(true);

    mpu6050_set_dlpf_mode(MPU6050_DLPF_BW_20);
    return ESP_OK;

}

/**
 * This function initializes the buzzer and stores it in the global variable. If configured to do so, it also plays
 * the test melody to test it.
 */
static void buzzer_initialize() {
    ESP_LOGI(buzzer_get_tag(), "Initializing buzzer in pin %i", BUZZER_PIN);
    buzzer = buzzer_init(LEDC_CHANNEL_0, LEDC_TIMER_0, BUZZER_PIN);


#ifdef CONFIG_BUZZER_SOUND_TEST
    ESP_LOGI(buzzer_get_tag(), "Playing test melody");
    buzzer_play_test(buzzer, 100);
#endif
}

/**
 * Main function of the module, which executes on boot and initializes all the components and tasks.
 */
void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();


    ESP_LOGI(MAIN_TAG, "Initializing buzzer...");
    buzzer_initialize();


    ESP_LOGI(MAIN_TAG, "Installing the I2C driver...");
    i2c_initialize();

    ESP_LOGI(MAIN_TAG, "MPU is connected!");

    ESP_LOGI(MAIN_TAG, "Starting the UDP communication task...");
    xTaskCreatePinnedToCore(udpTask, "UdpCommTask", 4096, NULL, 3, &UdpTaskHandle, 1);

    ESP_LOGI(MAIN_TAG, "Starting the buzzer task...");
    xTaskCreate(buzzerTask, "BuzzerTask", 1024, NULL, 2, &BuzzerTaskHandle);

    ESP_LOGI(MAIN_TAG, "Starting the gyroscope reading task...");
    xTaskCreatePinnedToCore(gyroTask, "GyroReadTask", 4096, NULL, 4, &GyroTaskHandle, 0);
    buzzer_play_note_ms(buzzer, BUZZER_NOTE_A, 4, 500);

    ESP_LOGI(MAIN_TAG, "Initializing MPU6050...");
    gyro_initialize();
}
