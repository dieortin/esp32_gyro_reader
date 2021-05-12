/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
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
/* The examples use WiFi configuration that you can set via project configuration menu
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *WIFI_TAG = "wifi station";

static int s_retry_num = 0;



// I2C -------------------------------------------------------------------------------------------------

#define I2C_FREQ CONFIG_EXAMPLE_I2C_CLOCK_FREQ
#define I2C_SDA_PIN CONFIG_EXAMPLE_I2C_SDA_GPIO
#define I2C_SCL_PIN CONFIG_EXAMPLE_I2C_SCL_GPIO

static const char *MAIN_TAG = "Main";

static const char *GYRO_TAG = "GyroReading";
TaskHandle_t GyroTaskHandle = NULL;

static const char *TCP_TAG = "TcpComms";
TaskHandle_t TcpTaskHandle = NULL;

static const char *BUZZER_TAG = "Buzzer";
TaskHandle_t BuzzerTaskHandle = NULL;

float self_test[6] = {0};
float accel_calibration[3] = {0};
float gyro_calibration[3] = {0};

buzzer_t *buzzer = NULL;


// TCP --------------------------------------------------------------------------------------------------

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT


// GPIO -------------------------------------------------------------------------------------------------
#define BUZZER_PIN CONFIG_BUZZER_GPIO

#define GYRO_SAMPLING_RATE 4


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

void buzzerTask(void *pvParameters) {
    int32_t lastValue = 0;
    while (1) {
        uint32_t newvalue;
        if (xTaskNotifyWait(0, 0, &newvalue, 0xFFFF) == pdPASS) {
            int32_t val = (int32_t) newvalue;
            if (newvalue == lastValue) continue;
            lastValue = newvalue;
            val += 90;
            uint8_t deg_per_note = 180 / BUZZER_NOTE_MAX;
            uint8_t final_note = val / deg_per_note;
            //buzzer_pause(buzzer);
            buzzer_set_note(buzzer, final_note, 4);
            buzzer_play(buzzer);
        } else {
            ESP_LOGW(BUZZER_TAG, "Time run out while waiting for notification, waiting again...");
        }
    }
}


void tcpTask(void *pvParameters) {
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *) &dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_TAG, "Successfully connected");

        while (1) {
            uint32_t newvalue;
            if (xTaskNotifyWait(0, 0, &newvalue, 0xFFFF) == pdPASS) {
                char payload[100];
                snprintf(payload, 100, "%i\n", (int32_t) newvalue);

                err = send(sock, payload, strlen(payload), 0);
                if (err < 0) {
                    ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            } else {
                ESP_LOGW(TCP_TAG, "Time run out while waiting for notification, waiting again...");
            }
        }

        if (sock != -1) {
            ESP_LOGE(TCP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


void gyroTask(void *pvParameters) {


    for (;;) {
        mpu6050_acceleration_t acceleration;
        mpu6050_rotation_t rotation;

        mpu6050_get_motion(&acceleration, &rotation);

        printf("Accelerometer: [x(%i), y(%i), z(%i)], ", acceleration.accel_x, acceleration.accel_y,
               acceleration.accel_z);
        printf("Gyroscope: [x(%i), y(%i), z(%i)] ", rotation.gyro_x, rotation.gyro_y, rotation.gyro_z);

        double pitch = calculate_pitch(acceleration.accel_x, acceleration.accel_y, acceleration.accel_z);


        xTaskGenericNotify(TcpTaskHandle, (int32_t) pitch, eSetValueWithOverwrite, NULL);
        xTaskGenericNotify(BuzzerTaskHandle, (int32_t)pitch, eSetValueWithOverwrite, NULL);
        printf("Pitch: [%lf]\n", pitch);
        vTaskDelay(pdMS_TO_TICKS(1000 / GYRO_SAMPLING_RATE));
    }
}

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
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
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

static esp_err_t gyro_initialize() {
    ESP_LOGI(mpu6050_get_tag(), "Initializing MPU6050...");

    ESP_LOGI(mpu6050_get_tag(), "Performing self-test...");
    mpu6050_self_test(self_test);


    // Check if self test passed for every axis
    if (self_test[0] >= 1.0f || self_test[1] >= 1.0f || self_test[2] >= 1.0f || self_test[3] >= 1.0f ||
        self_test[4] >= 1.0f || self_test[5] >= 1.0f) {
        ESP_LOGE(mpu6050_get_tag(), "Self-test failed!");
        return ESP_FAIL;
    }

    ESP_LOGI(mpu6050_get_tag(), "Passed self-test");

    mpu6050_reset();
#ifdef CONFIG_EXAMPLE_GYRO_CALIBRATE
    ESP_LOGI(mpu6050_get_tag(), "Performing calibration...");
    mpu6050_calibrate(accel_calibration, gyro_calibration);
    ESP_LOGI(mpu6050_get_tag(), "Calibration successful");
#endif
    ESP_LOGI(mpu6050_get_tag(), "Initializing...");
    mpu6050_init();

    mpu6050_set_dlpf_mode(MPU6050_DLPF_BW_20);
    return ESP_OK;

}

static void buzzer_initialize() {
    ESP_LOGI(buzzer_get_tag(), "Initializing buzzer in pin %i", BUZZER_PIN);
    buzzer = buzzer_init(LEDC_CHANNEL_0, LEDC_TIMER_0, BUZZER_PIN);


#ifdef CONFIG_BUZZER_SOUND_TEST
    buzzer_melody_t m;
    buzzer_musical_note_t notes[] = {
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_F, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_F, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_MINIM},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_B, 3, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_F, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_B, 3, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_F, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_F, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_B, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_F, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_B, 3, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_A, 3, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_QUAVER_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET_DOTTED},
            {BUZZER_NOTE_D, 4, BUZZER_NTYPE_QUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_D, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_F, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_E, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_A, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_B, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_G, 3, BUZZER_NTYPE_SEMIQUAVER},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_E, 4, BUZZER_NTYPE_CROTCHET},
            {BUZZER_NOTE_C, 4, BUZZER_NTYPE_MINIM},
    };

    m.melody = notes;
    m.length = sizeof(notes) / sizeof(buzzer_musical_note_t);

    //buzzer_play_melody(buzzer, &m, 110);

    buzzer_play_test(buzzer, 100);
#endif
}

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

    ESP_LOGI(MAIN_TAG, "Initializing MPU6050...");
    gyro_initialize();


    ESP_LOGI(MAIN_TAG, "MPU is connected!");

    ESP_LOGI(MAIN_TAG, "Starting the TCP communication task...");
    xTaskCreatePinnedToCore(tcpTask, "TcpCommTask", 4096, NULL, 3, &TcpTaskHandle, 1);

    ESP_LOGI(MAIN_TAG, "Starting the buzzer task...");
    xTaskCreate(buzzerTask, "BuzzerTask", 1024, NULL, 2, &BuzzerTaskHandle);

    ESP_LOGI(MAIN_TAG, "Starting the gyroscope reading task...");
    xTaskCreatePinnedToCore(gyroTask, "GyroReadTask", 4096, NULL, 4, &GyroTaskHandle, 0);
    buzzer_play_note_ms(buzzer, BUZZER_NOTE_A, 4, 500);

}
