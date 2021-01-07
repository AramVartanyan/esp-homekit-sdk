/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/* ESP8266 4MB module for Watering + Temperature and Humidity sensors on Battery */

#include <stdio.h>
#include <string.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_event.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_system.h>
//#include <esp_timer.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
#include <hap_fw_upgrade.h>
/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};
#endif

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include <dht.h>

/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};
static const char *TAG = "HAP Accessory";

#define ACC_TASK_PRIORITY  1
#define ACC_TASK_STACKSIZE 4 * 1024
#define ACC_TASK_NAME      "hap_accessory"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

// Valve type is 0, 1, 2 or 3 [generic, irrigation, shower head, water faucet]
#define V_TYPE 1

#define SETUP_CODE      "101-11-180"
#define D_NAME          "WaterB-80%02X%02X"
#define D_MANIFACTURER  CONFIG_D_MANIFACTURER
#define D_MODEL         CONFIG_D_MODEL
#define D_FW            CONFIG_D_FW
#define D_HW            CONFIG_D_HW

#define VOLTAGE_CAL 220 //Use this constant for correct calculation of battery level.
//220 -> 9V -> 0% (255 - Battery voltage level in mV - actual 10,5V. 100% - 305mV/12,5V)

//ESP8266 GPIO
#define TEMP_SENSOR   CONFIG_TEMP_GPIO    //4 (D2)
#define LED_GPIO      CONFIG_LED_GPIO     //2
#define RESET_GPIO    CONFIG_BUTTON_GPIO  //0 - button
#define RELAY_GPIO    CONFIG_RELAY_GPIO   //5
#define CHARGE_GPIO   CONFIG_CHARGE_GPIO  //14

int hap_keystore_init();
int hap_keystore_set(const char *name_space, const char *key, const uint8_t *val, const size_t val_len);
int hap_keystore_get(const char *name_space, const char *key, uint8_t *val, size_t *val_size);
int hap_keystore_delete(const char *name_space, const char *key);

//Battery Level is from 0 to 100 percentage
//Charging State is 0 - Not charging, 1 - Charging
//Battery Status is 0 - Battery level is normal, 1 - Battery level is low
//Time duration is from 0 to 3600 in 1 s steps

static struct {
    bool ActiveM;
    bool InUseStateM;
    uint16_t SetDurationM;
    uint16_t RemDurationM;
    int16_t TemperatureM;
    int16_t HumidityM;
} container;
//hap_val_t val = {.i = fw_upgrade_status}; //example

TimerHandle_t v_timer;

hap_char_t *ValveActive;
hap_char_t *ValveInUse;
hap_char_t *RemDuration;
hap_char_t *TemperatureChar;
hap_char_t *HumidityChar;

void relay_write(bool on, int gpio);
void led_write(bool on, int gpio);

void app_wifi_init(void);
static void InitializePlatform(void);

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    led_write(true, LED_GPIO);
    hap_keystore_delete("water", "sduration");
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

void relay_write(bool on, int gpio) {
    gpio_set_level(gpio, on ? 1 : 0);
}

void led_write(bool on, int gpio) {
    gpio_set_level(gpio, on ? 0 : 1);
}

static int read_battery() {
    int BatteryLevel = 0;
    uint16_t voltage = 0;
    if (ESP_OK == adc_read(&voltage)) {
        ESP_LOGI(TAG, "Voltage read: %d\r\n", voltage);
    }
    //Measure the voltage at TOUT pin (A0) - 1V max
    //ADC voltage is %.3f", 1.0 / 1024 * sdk_system_adc_read()
    voltage = (1000.0 / 1024) * voltage; //mV
    BatteryLevel = 1.1 * (voltage - VOLTAGE_CAL);
    
    if (BatteryLevel > 99) {
        BatteryLevel = 100;
    } else if (BatteryLevel < 1) {
        BatteryLevel = 0;
    }
    return BatteryLevel;
}

void v_off(TimerHandle_t xTimer) {
    container.RemDurationM--;
    if (container.RemDurationM == 0) {
        hap_val_t ActiveValue;
        hap_val_t InUseState;
        hap_val_t RemDurationValue;
        
        container.ActiveM = 0;
        container.InUseStateM = 0;
        relay_write(container.ActiveM, RELAY_GPIO);
        led_write(container.ActiveM, LED_GPIO);
        
        ActiveValue.i = container.ActiveM;
        InUseState.i = container.InUseStateM;
        RemDurationValue.i = container.RemDurationM;
        hap_char_update_val(ValveActive, &ActiveValue);
        hap_char_update_val(ValveInUse, &InUseState);
        hap_char_update_val(RemDuration, &RemDurationValue);
    } else {
        xTimerStart(v_timer, 1);
    }
    if (container.ActiveM == 0) {
        if (v_timer && xTimerIsTimerActive(v_timer)) {
            xTimerStop(v_timer, 1);
        }
    }
}

void temperature_sensor_task(void *_args) {

 while (1) {
     dht_read_data(DHT_TYPE_AM2301, TEMP_SENSOR, &container.HumidityM, &container.TemperatureM);

     vTaskDelay(30000 / portTICK_PERIOD_MS);
 }
}

static void device_identify_task(void *_args) {
    for (int i=0; i<3; i++) {
      for (int j=0; j<2; j++) {
          led_write(true, LED_GPIO);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          led_write(false, LED_GPIO);
          vTaskDelay(100 / portTICK_PERIOD_MS);
      }
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(false, LED_GPIO);
    vTaskDelete(NULL);
}

static int accessory_identify(hap_acc_t *ha)
{
    xTaskCreate(device_identify_task, "Device identify", 512, NULL, 2, NULL);
    ESP_LOGI(TAG, "Accessory identification");
    return HAP_SUCCESS;
}

static void InitializePlatform() {
    
    container.ActiveM = 0;
    container.InUseStateM = 0;
    container.SetDurationM = 3600;
    container.RemDurationM = 0;
    container.TemperatureM = 200;
    container.HumidityM = 500;
    
    gpio_config_t io_conf = {0};
    
    io_conf.pin_bit_mask = (1ULL<<CHARGE_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL<<TEMP_SENSOR);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL<<RELAY_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL<<LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    
    adc_config_t adc_config;
    // Depend on menuconfig->Component config->PHY->vdd33_const value
    // When measuring system voltage(ADC_READ_VDD_MODE), vdd33_const must be set to 255.
    adc_config.mode = ADC_READ_TOUT_MODE;
    adc_config.clk_div = 16; // ADC sample collection clock = 80MHz/clk_div = 10MHz
    ESP_ERROR_CHECK(adc_init(&adc_config));
    
    hap_keystore_init();
    uint8_t duration;
    size_t size = sizeof(duration);
    if (hap_keystore_get("water", "sduration", (uint8_t *)&duration, &size) == HAP_SUCCESS) {
        container.SetDurationM = duration * 100;
    } else {
        container.SetDurationM = 3600;
    }
    
    relay_write(container.ActiveM, RELAY_GPIO);
    led_write(container.ActiveM, LED_GPIO);
}
    
static int valve_read(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_ACTIVE)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t ActiveValue = {.i = container.ActiveM};
        ESP_LOGI(TAG, "Current active state is %s", cur_val->i ? "On" : "Off");
        hap_char_update_val(hc, &ActiveValue);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_IN_USE)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t InUseState = {.i = container.InUseStateM};
        ESP_LOGI(TAG, "Currently the valve is %s", cur_val->i ? "In use" : "Not in use");
        hap_char_update_val(hc, &InUseState);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_SET_DURATION)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        ESP_LOGI(TAG, "The set duration is %d s", cur_val->u);
        hap_val_t SetDurationValue = {.u = container.SetDurationM};
        hap_char_update_val(hc, &SetDurationValue);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_REMAINING_DURATION)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t RemDurationValue = {.u = container.RemDurationM};
        ESP_LOGI(TAG, "The remaining duration is %d s", cur_val->u);
        hap_char_update_val(hc, &RemDurationValue);
        *status = HAP_STATUS_SUCCESS;
    } else {
        *status = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }
    return ret;
}

static int valve_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ACTIVE)) {
            ESP_LOGI(TAG, "Write active state %s", write->val.i ? "On" : "Off");
            hap_val_t InUseState;
            hap_val_t RemDurationValue;

            container.ActiveM = write->val.i;
            if (container.ActiveM) {
               container.InUseStateM = 1;
               if (container.SetDurationM > 0) {
                   if (v_timer && xTimerIsTimerActive(v_timer)) {
                       ESP_LOGE(TAG, "Valve timer is already started.\n");
                   } else {
                       container.RemDurationM = container.SetDurationM;
                       v_timer = xTimerCreate("Valve timer", pdMS_TO_TICKS(1000), pdFALSE, ( void * ) 0, v_off);
                       xTimerStart(v_timer, 1);
                   }
               }
            } else {
                container.InUseStateM = 0;
                if (v_timer && xTimerIsTimerActive(v_timer)) {
                    xTimerStop(v_timer, 1);
                }
                container.RemDurationM = 0;
            }
         relay_write(container.ActiveM, RELAY_GPIO);
         led_write(container.ActiveM, LED_GPIO);
            
         InUseState.i = container.InUseStateM;
         RemDurationValue.u = container.RemDurationM;
         hap_char_update_val(write->hc, &(write->val));
         hap_char_update_val(ValveInUse, &InUseState);
         hap_char_update_val(RemDuration, &RemDurationValue);
         *(write->status) = HAP_STATUS_SUCCESS;
         
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SET_DURATION)) {
            ESP_LOGI(TAG, "Write set duration is %d s", write->val.u);
            container.SetDurationM = write->val.u;
            uint8_t duration = container.SetDurationM / 100;
            ret = hap_keystore_set("water", "sduration", (uint8_t *)&duration, sizeof(uint8_t));
            if (ret != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write set duration.\n");
            }
            hap_char_update_val(write->hc, &(write->val));
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static int battery_read(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_BATTERY_LEVEL)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t BatteryLevelValue;
        BatteryLevelValue.i = read_battery();
        ESP_LOGI(TAG, "The battery level was %d", cur_val->i);
        ESP_LOGI(TAG, "The read battery level is %d", BatteryLevelValue.i);
        hap_char_update_val(hc, &BatteryLevelValue);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CHARGING_STATE)) {
        hap_val_t ChargingState;
        //ChargingState.i = !gpio_get_level(CHARGE_GPIO);
        if (gpio_get_level(CHARGE_GPIO)) {
           ChargingState.i = 0;
        } else {
           ChargingState.i = 1;
        }
        
        ESP_LOGI(TAG, "Currently the battery is %s", ChargingState.i ? "Charging" : "Not charging");
        hap_char_update_val(hc, &ChargingState);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_STATUS_LOW_BATTERY)) {
        hap_val_t LowBatteryState;
        hap_val_t BatteryLevelValue;
        BatteryLevelValue.i = read_battery();
        if (BatteryLevelValue.i < 20) {
           LowBatteryState.i = 1;
        } else {
           LowBatteryState.i = 0;
        }
        ESP_LOGI(TAG, "Low battery %s", LowBatteryState.i ? "Yes" : "No");
        hap_char_update_val(hc, &LowBatteryState);
        *status = HAP_STATUS_SUCCESS;
    } else {
        *status = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }
    return ret;
}

static int temperature_read(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_TEMPERATURE)) {
        hap_val_t TemperatureValue;
        TemperatureValue.f = (float)container.TemperatureM / 10.0;
        ESP_LOGI(TAG, "The current temperature is %f", TemperatureValue.f);
        hap_char_update_val(hc, &TemperatureValue);
        *status = HAP_STATUS_SUCCESS;
    } else {
        *status = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }
    return ret;
}

static int humidity_read(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) {
        hap_val_t HumidityValue;
        HumidityValue.f = container.HumidityM / 10;
        ESP_LOGI(TAG, "The current humidity is %f", HumidityValue.f);
        hap_char_update_val(hc, &HumidityValue);
        *status = HAP_STATUS_SUCCESS;
    } else {
        *status = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }
    return ret;
}

hap_char_t *hap_char_current_relative_humidity_custom (float curr_rel_humidity)
{
    hap_char_t *hc = hap_char_float_create(HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY,
                                           HAP_CHAR_PERM_PR | HAP_CHAR_PERM_EV, curr_rel_humidity);

    hap_char_float_set_constraints(hc, 0.0, 100.0, 1.0);
    hap_char_add_unit(hc, HAP_CHAR_UNIT_PERCENTAGE);

    return hc;
}

hap_char_t *hap_char_current_temperature_custom (float curr_temp)
{
    hap_char_t *hc = hap_char_float_create(HAP_CHAR_UUID_CURRENT_TEMPERATURE,
                                           HAP_CHAR_PERM_PR | HAP_CHAR_PERM_EV, curr_temp);

    hap_char_float_set_constraints(hc, -50.0, 100.0, 0.1);
    hap_char_add_unit(hc, HAP_CHAR_UNIT_CELSIUS);

    return hc;
}

/*The main thread for handling the Smart Outlet Accessory */
static void acc_thread_entry(void *p)
{
    InitializePlatform();
    char accessory_name[16];

    //get WIFI MAC address
    char wifi_mac_address[16];
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(wifi_mac_address, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("Device WIFI mac_address: %s\n", wifi_mac_address);

    sprintf(accessory_name, D_NAME, mac[4], mac[5]);
    printf("Device Name: %s\n", accessory_name);
    
    hap_acc_t *accessory;
    hap_serv_t *svalve;
    hap_serv_t *sbattery;
    hap_serv_t *stemperature;
    hap_serv_t *shumidity;

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = accessory_name,
        .manufacturer = D_MANIFACTURER,
        .model = D_MODEL,
        .serial_num = wifi_mac_address,
        .fw_rev = D_FW,
        .hw_rev = D_HW,
        .pv = "1.1.0",
        .identify_routine = accessory_identify,
        .cid = 28,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','8','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));
    
    //hap_serv_t *hap_serv_valve_create(uint8_t active, uint8_t in_use, uint8_t valve_type);
    svalve = hap_serv_create(HAP_SERV_UUID_VALVE);
    //svalve = hap_serv_valve_create(ActiveValue.i, InUseState.i, V_TYPE);
    hap_serv_add_char(svalve, hap_char_name_create(accessory_name));
    hap_serv_add_char(svalve, hap_char_active_create(0)); //ActiveValue.i
    hap_serv_add_char(svalve, hap_char_in_use_create(0)); //InUseState.i
    hap_serv_add_char(svalve, hap_char_valve_type_create(V_TYPE));
    
    hap_serv_add_char(svalve, hap_char_set_duration_create(3600)); //SetDurationValue.u
    hap_serv_add_char(svalve, hap_char_remaining_duration_create(0)); //RemDurationValue.u
    
    ValveActive = hap_serv_get_char_by_uuid(svalve, HAP_CHAR_UUID_ACTIVE);
    ValveInUse = hap_serv_get_char_by_uuid(svalve, HAP_CHAR_UUID_IN_USE);
    RemDuration = hap_serv_get_char_by_uuid(svalve, HAP_CHAR_UUID_REMAINING_DURATION);
     
    //hap_char_t *ValveActive;
    //hap_char_t *ValveInUse;
    //hap_char_t *RemDuration;
    //HAP_CHAR_UUID_ACTIVE
    //HAP_CHAR_UUID_SET_DURATION
    //HAP_CHAR_UUID_REMAINING_DURATION
    //HAP_CHAR_UUID_IN_USE
    //HAP_CHAR_UUID_VALVE_TYPE
    
    //Set the read callback for the service (optional)
    hap_serv_set_read_cb(svalve, valve_read);

    //Set the write callback for the service
    hap_serv_set_write_cb(svalve, valve_write);
    
    //Add the Valve Service to the Accessory Object
    hap_acc_add_serv(accessory, svalve);
    
    //hap_serv_t *hap_serv_battery_service_create(uint8_t battery_level, uint8_t charging_state, uint8_t status_low_battery);
    sbattery = hap_serv_battery_service_create(50, 0, 0); //BatteryLevelValue.i, ChargingState.i, LowBatteryState.i
    
    //HAP_CHAR_UUID_BATTERY_LEVEL
    //HAP_CHAR_UUID_CHARGING_STATE
    //HAP_CHAR_UUID_STATUS_LOW_BATTERY
    hap_serv_set_read_cb(sbattery, battery_read);
    hap_acc_add_serv(accessory, sbattery);
    
    //Char: Current Relative Humidity
    //shumidity = hap_serv_humidity_sensor_create(50); //HumidityValue.f
    shumidity = hap_serv_create(HAP_SERV_UUID_HUMIDITY_SENSOR);
    hap_serv_add_char(shumidity, hap_char_current_relative_humidity_custom(50)); //Трябва да се трансформира в собствена функция и да включва следващите редове.
    
    //shumidity = hap_char_float_create(HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY, HAP_CHAR_PERM_PR | HAP_CHAR_PERM_EV, 50);
    //hap_char_float_set_constraints(shumidity, 0.0, 100.0, 1.0);
    //hap_char_add_unit(shumidity, HAP_CHAR_UNIT_PERCENTAGE);
        
    hap_serv_set_read_cb(shumidity, humidity_read);
    hap_acc_add_serv(accessory, shumidity);

    //Char: Current Temperature
    //stemperature = hap_char_string_create(HAP_CHAR_UUID_CURRENT_TEMPERATURE, HAP_CHAR_PERM_PR, "Temperature Sensor");
    stemperature = hap_serv_create(HAP_SERV_UUID_TEMPERATURE_SENSOR);
    hap_serv_add_char(stemperature, hap_char_current_temperature_custom(10));
    
    //stemperature = hap_char_float_create(HAP_CHAR_UUID_CURRENT_TEMPERATURE, HAP_CHAR_PERM_PR | HAP_CHAR_PERM_EV, 20);
    //hap_char_float_set_constraints(stemperature, -50.0, 100.0, 0.1);
    //hap_char_add_unit(stemperature, HAP_CHAR_UNIT_CELSIUS);
        
    hap_serv_set_read_cb(stemperature, temperature_read);
    hap_acc_add_serv(accessory, stemperature);
     
    /*
    
    //hap_serv_t *hap_serv_temperature_sensor_create(float curr_temp);
    stemperature = hap_serv_temperature_sensor_create(20); //TemperatureValue.f
    TemperatureChar = hap_serv_get_char_by_uuid(stemperature, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
    
    //DHT22  -40 ... +80
    
    //TemperatureChar
    //HAP_CHAR_UUID_CURRENT_TEMPERATURE
    
    hap_serv_set_read_cb(stemperature, temperature_read);
    hap_acc_add_serv(accessory, stemperature);
    
    //hap_serv_t *hap_serv_humidity_sensor_create(float curr_relative_humidity);
    shumidity = hap_serv_humidity_sensor_create(50); //HumidityValue.f
    HumidityChar = hap_serv_get_char_by_uuid(shumidity, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);
    
    //HumidityChar
    //HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY
    
    hap_serv_set_read_cb(shumidity, humidity_read);
    hap_acc_add_serv(accessory, shumidity);
    */
    
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    /* Create the Firmware Upgrade HomeKit Custom Service.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    svalve = hap_serv_fw_upgrade_create(&ota_config);
    /* Add the service to the Accessory Object */
    hap_acc_add_serv(accessory, service);
#endif
    
    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);
    
    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);
    

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();
        
    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    
    xTaskCreate(temperature_sensor_task, "Temperature Sensor", 1024, NULL, 4, NULL);

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

void app_main()
{
    /* Create the application thread */
    xTaskCreate(acc_thread_entry, ACC_TASK_NAME, ACC_TASK_STACKSIZE,
                NULL, ACC_TASK_PRIORITY, NULL);
}
