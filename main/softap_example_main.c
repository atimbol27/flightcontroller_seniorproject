/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include <esp_http_server.h>

#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#define MOTOR1_GPIO     4           //blue
#define MOTOR2_GPIO    27           //green
#define MOTOR3_GPIO    14           //yellow
#define MOTOR4_GPIO    12           //red

static const char *TAG = "wifi softAP";

static httpd_handle_t server = NULL;

static rmt_channel_handle_t esc_chan = NULL;
static rmt_channel_handle_t esc_chan1 = NULL;
static rmt_channel_handle_t esc_chan2 = NULL;
static rmt_channel_handle_t esc_chan3 = NULL;

static rmt_encoder_handle_t dshot_encoder = NULL;

int pressed = 0;

    rmt_transmit_config_t tx_config = {
        .loop_count = -1, // infinite loop
    };

    dshot_esc_throttle_t motor1 = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };
    dshot_esc_throttle_t motor2 = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };
    dshot_esc_throttle_t motor3 = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };
    dshot_esc_throttle_t motor4 = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };
    
    
void reverse_motor_direction();
//Webserver Code

//MOTOR OFF HANDLER
static esp_err_t motoroff_handler(httpd_req_t *req)
{
    esp_err_t error;
    ESP_LOGI(TAG, "Motors OFF");

    if(pressed == 0){
        ESP_LOGI(TAG, "Start ESC by sending zero throttle for a while...");
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor1, sizeof(motor1), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan1, dshot_encoder, &motor2, sizeof(motor2), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan2, dshot_encoder, &motor3, sizeof(motor3), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan3, dshot_encoder, &motor4, sizeof(motor4), &tx_config));     
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    else
    {
        motor1.throttle = 0;
        motor2.throttle = 0;
        motor3.throttle = 0;
        motor4.throttle = 0;

        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor1, sizeof(motor1), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan1, dshot_encoder, &motor2, sizeof(motor2), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan2, dshot_encoder, &motor3, sizeof(motor3), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan3, dshot_encoder, &motor4, sizeof(motor4), &tx_config));

        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_disable(esc_chan1));
        ESP_ERROR_CHECK(rmt_disable(esc_chan2));
        ESP_ERROR_CHECK(rmt_disable(esc_chan3));

        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan1));
        ESP_ERROR_CHECK(rmt_enable(esc_chan2));
        ESP_ERROR_CHECK(rmt_enable(esc_chan3));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
        

    const char *response = (const char *) req->user_ctx;
    error = httpd_resp_send(req, response, strlen(response));
    if(error != ESP_OK)
    {
        ESP_LOGI(TAG, "Error %d while sending Response", error);
    }
    else ESP_LOGI(TAG, "Response sent Successfully");
    return error;
}

static const httpd_uri_t motoroff = {
    .uri       = "/motoroff",
    .method    = HTTP_GET,
    .handler   = motoroff_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "<!DOCTYPE html>\
<html>\
<head>\
<style>\
.button {\
  border: none;\
  color: white;\
  padding: 15px 32px;\
  text-align: center;\
  text-decoration: none;\
  display: inline-block;\
  font-size: 16px;\
  margin: 4px 2px;\
  cursor: pointer;\
}\
\
.button1 {background-color: #4CAF50;} /* Green */\
</style>\
</head>\
<body>\
\
<h1>Drone Webserver</h1>\
<p>Turn ON and OFF Motors</p>\
<h3>MOTOR STATE: OFF</h3>\
\
<button class=\"button button1\" onclick=\"window.location.href='/motoron'\">MOTORS ON</button>\
\
</body>\
</html>"
};

//MOTOR ON HANDLER
static esp_err_t motoron_handler(httpd_req_t *req)
{
    esp_err_t error;
    ESP_LOGI(TAG, "Motors ON");
    ESP_LOGI(TAG, "Increase throttle, no telemetry");

        pressed = 1;
        
        motor2.throttle = 21; //21 is special throttle value to reverse direction  
        motor3.throttle = 21; 

        ESP_ERROR_CHECK(rmt_transmit(esc_chan1, dshot_encoder, &motor2, sizeof(motor2), &tx_config));
        ESP_ERROR_CHECK(rmt_disable(esc_chan1));
        ESP_ERROR_CHECK(rmt_enable(esc_chan1));

        ESP_ERROR_CHECK(rmt_transmit(esc_chan2, dshot_encoder, &motor3, sizeof(motor3), &tx_config));
        ESP_ERROR_CHECK(rmt_disable(esc_chan2));
        ESP_ERROR_CHECK(rmt_enable(esc_chan2));

        vTaskDelay(pdMS_TO_TICKS(5000));
        
        motor1.throttle = 50;
        motor2.throttle = 50;
        motor3.throttle = 50;
        motor4.throttle = 50;

        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor1, sizeof(motor1), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan1, dshot_encoder, &motor2, sizeof(motor2), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan2, dshot_encoder, &motor3, sizeof(motor3), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan3, dshot_encoder, &motor4, sizeof(motor4), &tx_config));
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_disable(esc_chan1));
        ESP_ERROR_CHECK(rmt_disable(esc_chan2));
        ESP_ERROR_CHECK(rmt_disable(esc_chan3));

        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan1));
        ESP_ERROR_CHECK(rmt_enable(esc_chan2));
        ESP_ERROR_CHECK(rmt_enable(esc_chan3));
        
        vTaskDelay(pdMS_TO_TICKS(1000));

        const char *response = (const char *) req->user_ctx;
        error = httpd_resp_send(req, response, strlen(response));

        if(error != ESP_OK)
        {
            ESP_LOGI(TAG, "Error %d while sending Response", error);
        }
        else ESP_LOGI(TAG, "Response sent Successfully");
        return error;

    /*
    const char *response = (const char *) req->user_ctx;
    error = httpd_resp_send(req, response, strlen(response));
    if(error != ESP_OK)
    {
        ESP_LOGI(TAG, "Error %d while sending Response", error);
    }
    else ESP_LOGI(TAG, "Response sent Successfully");
    return error;
    */
}

static const httpd_uri_t motoron = {
    .uri       = "/motoron",
    .method    = HTTP_GET,
    .handler   = motoron_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "<!DOCTYPE html>\
<html>\
<head>\
<style>\
.button {\
  border: none;\
  color: white;\
  padding: 15px 32px;\
  text-align: center;\
  text-decoration: none;\
  display: inline-block;\
  font-size: 16px;\
  margin: 4px 2px;\
  cursor: pointer;\
}\
\
.button1 {background-color: #000000;} /* Green */\
</style>\
</head>\
<body>\
\
<h1>Drone Webserver</h1>\
<p>Turn ON and OFF Motors</p>\
<h3>MOTOR STATE: ON</h3>\
\
<button class=\"button button1\" onclick=\"window.location.href='/motoroff'\">MOTORS OFF</button>\
\
</body>\
</html>"
};

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &motoroff);
        httpd_register_uri_handler(server, &motoron);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

void config_rmt_motors(void){
    //RMT CONFIGURATIONS for MOTORS
    //ESP_LOGI(TAG, "Create RMT TX channel");
    //MOTOR 1 RMT Channel Configuration
    //rmt_channel_handle_t esc_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = MOTOR1_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };

    //MOTOR 2 RMT Channel Configuration
    //rmt_channel_handle_t esc_chan1 = NULL;
    rmt_tx_channel_config_t tx_chan_config1 = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = MOTOR2_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };

    //MOTOR 3 RMT Channel Configuration
    //rmt_channel_handle_t esc_chan2 = NULL;
    rmt_tx_channel_config_t tx_chan_config2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = MOTOR3_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };

    //MOTOR 4 RMT Channel Configuration
    //rmt_channel_handle_t esc_chan3 = NULL;
    rmt_tx_channel_config_t tx_chan_config3 = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = MOTOR4_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    }; 

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chan));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config1, &esc_chan1));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config2, &esc_chan2));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config3, &esc_chan3));
}

void config_dshot_encoder(void){
    //rmt_encoder_handle_t dshot_encoder = NULL;
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };

    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));
}

void enable_rmt_motors(void){
    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(esc_chan)); //motor 1
    ESP_ERROR_CHECK(rmt_enable(esc_chan1)); //motor 2
    ESP_ERROR_CHECK(rmt_enable(esc_chan2)); //motor 1
    ESP_ERROR_CHECK(rmt_enable(esc_chan3)); //motor 3
}

void reverse_motor_direction(void){
        //Reverse Direction for Motor 2
        //Reverse Direction for Motor 3

        motor2.throttle = 21; //21 is special throttle value to reverse direction  
        motor3.throttle = 21; 

        ESP_ERROR_CHECK(rmt_transmit(esc_chan1, dshot_encoder, &motor2, sizeof(motor2), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan2, dshot_encoder, &motor3, sizeof(motor3), &tx_config));

        ESP_ERROR_CHECK(rmt_disable(esc_chan1));
        ESP_ERROR_CHECK(rmt_disable(esc_chan2));

        ESP_ERROR_CHECK(rmt_enable(esc_chan1));
        ESP_ERROR_CHECK(rmt_enable(esc_chan2));

        vTaskDelay(pdMS_TO_TICKS(2000));
}
/*
void PID_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
    float Pterm = P*Error;
    float Iterm = PrevIterm+I*(Error+PrevError)*0.004/2;
    if(Iterm > 400){
        Iterm = 400;
    }
    else if(Iterm < -400){
        Iterm = -400;
    }
    float Dterm = D*(Error-PrevError)/0.004;
    float PIDOutput = Pterm + Iterm + Dterm;
    if(PIDOutput > 400){
        PIDOutput = 400;
    }
    else if(PIDOutput < -400){
        PIDOutput = -400;
    }
}

void gyro_signals(void){
        float acc_x =  get_mpu6050_axis(0)-0.02;
        float acc_y =  get_mpu6050_axis(1)-0.01;
        float acc_z =  get_mpu6050_axis(2);
        
        float AngleRoll = atan(acc_y/(sqrt((acc_x*acc_x)+(acc_z*acc_z))))*1/(3.142/180.0) + 0.47;
        float AnglePitch = -atan(acc_x/(sqrt((acc_y*acc_y)+(acc_z*acc_z))))*1/(3.142/180.0) - 2.2;
        
        //printf("Accelerometer:\tX: %f", acc_x);
        //printf("\tY: %f", acc_y); 
        //printf("\tZ: %f\n", acc_z);
        
        //printf("Roll Angle: %f\t", AngleRoll);
        //printf("Pitch Angle: %f\n", AnglePitch);
    
        float RateRoll = (get_mpu6050_axis(3));
        float RatePitch = (get_mpu6050_axis(4));
        float RateYaw = (get_mpu6050_axis(5));

        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;

        //printf("Gyroscope:\tX: %f", gyro_x);
        //printf("\tY: %f", gyro_y); 
        //printf("\tZ: %f\n\n\n", gyro_z);
    
        vTaskDelay(250/portTICK_PERIOD_MS);
}

void vTask_PID(void *pvParameters){
    TickType_t xLastWakeTime;

    const TickType_t xPeriod = 4;

    xLastWakeTime = xTaskGetTickCount();

    pcPIDgain = * (int *)pvParameters;

    for(;;){
            vTaskDelayUntil(&xLastWakeTime, xPeriod);

            gyro_signals();
            float DesiredRateRoll;
            float DesiredRatePitch;


    }

}
*/ 
void app_main(void)
{
    config_rmt_motors();

    config_dshot_encoder();
    /*
    init_i2c(); 

    init_mpu6050();

     if (check_mpu6050()){
        printf("\nInitialized!!!\n");
    }
    else{
        printf("\nNO Initialized\n");
    }
    */
    enable_rmt_motors();

    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_handler, &server));
}
