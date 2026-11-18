/* HomeKit ESP32 Dehumidifier with Relay */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <math.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <app_hap_setup_payload.h>

#include "iot_button.h"
#include "esp_aht.h"

#include "app_wifi.h"
#include "outputwrite.h"
#include "fupdateota.h"

static const char *TAG = "HAP Accessory";

#define ACC_TASK_PRIORITY  1
#define ACC_TASK_STACKSIZE 4 * 1024
#define ACC_TASK_NAME      "hap_accessory"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

//Current Humidifier Dehumidifier State
#define INACTIVE      0 //използва се, когато получим сигнал за пълен контейнер
#define IDLE          1 //използва се, когато е достигнато заданието
#define HUMIDIFYING   2
#define DEHUMIDIFYING 3

#define ACTIVE_STATE  1 //определя дали е включено или изключено

//Target Humidifier Dehumidifier State
#define HUMIDIFIER_OR_DEHUMIDIFIER  0
#define HUMIDIFIER    1
#define DEHUMIDIFIER  2

#define HUM_HYSTERESIS 2

#define CAL_HUMIDITY 0 //Корекция на грешка от измерване с X%

/*
 State is a uint8 parameter
 
 WaterLevel - от 0 до 100, през 1 (float in Home)
 HumidityThreshold - от 0 до 100, през 1
 CurrentHumidity - от 0 до 100, през 1
 TargetHumidity - от 0 до 100, през 1
 */

static hap_char_t *ActiveC;
static hap_char_t *CurrentStateC;
static hap_char_t *CurrentHumidityC;

/* къстъмизирана характеристика */
static hap_char_t *TriggerUpdateChar;
static hap_char_t *UpdateStatusChar;

#define HAP_CHAR_CUSTOM_UUID_FW_UPG_STATUS  "d5703b5e-3736-11e8-b467-0ed5f89f718b"
#define HAP_CHAR_CUSTOM_UUID_TROTA          "F0090013-079E-11e8-8F27-9C2605A29F52"
/* къстъмизирана характеристика */

static struct {
    bool LocalOn;
    bool wasWaterTankFull;
    bool isWaterTankFull; //да се замени с четене от вход
    bool FirmwareUpdate;
    bool StatusConnected;
    uint8_t ActiveM;
    uint8_t CurrentStateM;
    uint8_t TargetStateM;
    uint8_t HumidityThresholdM;
    uint8_t DehumidityThresholdM;
    uint8_t CurrentHumidityM;
    uint16_t WaterLevelM;
} container;

//WaterLevelM е 100 пъти по-голямо - 100% се представя като 1000
//Трябва да се създаде метод за ресет на WaterLevelM

int hap_keystore_init();
int hap_keystore_set(const char *name_space, const char *key, const uint8_t *val, const size_t val_len);
int hap_keystore_get(const char *name_space, const char *key, uint8_t *val, size_t *val_size);
int hap_keystore_delete(const char *name_space, const char *key);

void UpdateState(bool UpdateTemp);

static int WriteSettings(uint8_t ValueNum);

void TakeStatusConnected(bool status) {
    container.StatusConnected = status;
    OutputWrite(container.StatusConnected, CONFIG_LED_GPIO, false);
}

void ledFlash(uint8_t FlashingTimes, TickType_t DelayMS){
    
    DelayMS = DelayMS / portTICK_PERIOD_MS;
    
    for (int j=0; j<FlashingTimes; j++) {
        OutputWrite(!container.StatusConnected, CONFIG_LED_GPIO, false);
        vTaskDelay(DelayMS);
        OutputWrite(container.StatusConnected, CONFIG_LED_GPIO, false);
        vTaskDelay(DelayMS);
    }
}

void otaStatus(void *p) {
    
    hap_val_t otaState;
    
    otaState.i = otaUpdateStatus;
    hap_char_update_val(UpdateStatusChar, &otaState);
    
    if (otaUpdateStatus == -1) {
        //update Failed
    } else {
        while (otaUpdateStatus) {
            ledFlash(1, 500);
            
            if (otaUpdateStatus == 2) {
                otaState.i = otaUpdateStatus;
                hap_char_update_val(UpdateStatusChar, &otaState);
                
                vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);
                hap_reboot_accessory();
            }
        }
    }
    
    hap_val_t Trigger;
    container.FirmwareUpdate = false;
    Trigger.b = container.FirmwareUpdate;
    hap_char_update_val(TriggerUpdateChar, &Trigger);
    
    otaState.i = otaUpdateStatus;
    hap_char_update_val(UpdateStatusChar, &otaState);
    
    vTaskDelay(3 * 1000 / portTICK_PERIOD_MS);
    
    vTaskDelete(NULL);
}

void ReadHumidityTask(void *pvParameter) {
    
    /*
     ValueNum = 3;
     bool wasWaterTankFull;
     uint16_t WaterLevelM;
     WriteSettings(3);
     */
    uint8_t UpdateDelay = 0;
    uint8_t NoSamples = 5;
    uint8_t Value;
    
    esp_err_t err = ahtInit(AHT21);
    ESP_LOGI(TAG, "AHT Sensor is %s", err ? "not initialized" : "initialized");
    if (ahtRead(NULL, &Value) == ESP_OK) {
        container.CurrentHumidityM = Value + CAL_HUMIDITY;
    }
    
    while (true) {
        UpdateDelay++;
        
        //the average of the samples read without an error (the sum starts from 0 every time)
        uint16_t Humidity = 0;
        uint8_t Samples = 0;
        for (int i = 0; i < NoSamples; i++) {
            if (ahtRead(NULL, &Value) == ESP_OK) {
                Humidity += Value;
                Samples++;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        
        if (Samples) {
            //ESP_LOGI(TAG, "Current Humidity %u", (Humidity + Samples / 2) / Samples);
            container.CurrentHumidityM = (Humidity + Samples / 2) / Samples + CAL_HUMIDITY;
        } else {
            ESP_LOGE(TAG, "AHT read error: the last humidity value stays");
        }
        
        if (UpdateDelay > 119) {
            UpdateDelay = 0;
            if (container.CurrentStateM == DEHUMIDIFYING) {
                container.WaterLevelM++;
                UpdateState(true); //Update the Current Humidity value to HAP once per 10 minutes
            }
        } else {
            container.LocalOn = !ReadInput(CONFIG_LOCAL_ON_GPIO);
            //ESP_LOGI(TAG, "Read Local On is %s", container.LocalOn ? "On" : "Off");
            container.isWaterTankFull = ReadInput(CONFIG_LVL_SWITCH_GPIO);
            //ESP_LOGI(TAG, "Read Water Tank is %s", container.isWaterTankFull ? "Full" : "Not Full");
            
            if (container.isWaterTankFull) {
                container.WaterLevelM = 999;
                container.wasWaterTankFull = container.isWaterTankFull;
                WriteSettings(3);
            } else {
                if (container.wasWaterTankFull) {
                    container.WaterLevelM = 0;
                    container.wasWaterTankFull = container.isWaterTankFull;
                    WriteSettings(3);
                } else if (container.WaterLevelM > 990) {
                    container.WaterLevelM = 990;
                }
            }
            
            UpdateState(false);
        }
        
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    
}

void UpdateState(bool UpdateHum) {
    
    bool active_state = container.ActiveM;
    uint8_t state = container.CurrentStateM;
    uint8_t hysteresis = HUM_HYSTERESIS;
    
    if (container.LocalOn == true && active_state == false) {
        container.ActiveM = ACTIVE_STATE;
        container.TargetStateM = DEHUMIDIFIER;
    }
    
    if (container.ActiveM) {
        switch (container.TargetStateM) {
            case DEHUMIDIFIER:
                if (container.DehumidityThresholdM < HUM_HYSTERESIS) {
                    hysteresis = 0;
                }
                if (container.CurrentHumidityM > container.DehumidityThresholdM) {
                    if (state != DEHUMIDIFYING) {
                        if (!container.wasWaterTankFull) {
                            container.CurrentStateM = DEHUMIDIFYING;
                            ESP_LOGI(TAG, "DEHUMIDIFIER Switch on Relay");
                            OutputWrite(true, CONFIG_RELAY_GPIO, false);
                        }
                    } else if (container.wasWaterTankFull) {
                        container.CurrentStateM = INACTIVE;
                        ESP_LOGI(TAG, "The tank is full. INACTIVE Switch off the Relay");
                        OutputWrite(false, CONFIG_RELAY_GPIO, false);
                    }
                } else if (container.CurrentHumidityM <= (container.DehumidityThresholdM - hysteresis)) {
                    if (state != IDLE) {
                        container.CurrentStateM = IDLE;
                        ESP_LOGI(TAG, "IDLE Switch off Relay");
                        OutputWrite(false, CONFIG_RELAY_GPIO, false);
                    }
                }
            break;
                
            case HUMIDIFIER:
                if (container.CurrentHumidityM < container.HumidityThresholdM) {
                    if (state != HUMIDIFYING) {
                        container.CurrentStateM = HUMIDIFYING;
                        ESP_LOGI(TAG, "HUMIDIFYING Switch off Relay");
                        OutputWrite(false, CONFIG_RELAY_GPIO, false);
                    }
                } else if (container.CurrentHumidityM >= (container.HumidityThresholdM + hysteresis)) {
                    if (state != IDLE) {
                        container.CurrentStateM = IDLE;
                        ESP_LOGI(TAG, "HUMIDIFYING Switch off Relay");
                        OutputWrite(false, CONFIG_RELAY_GPIO, false);
                    }
                }
            break;
            
            case HUMIDIFIER_OR_DEHUMIDIFIER:
                if (container.HumidityThresholdM < HUM_HYSTERESIS) {
                    hysteresis = 0;
                }
                if (container.CurrentHumidityM <= container.DehumidityThresholdM) {
                    if (state != HUMIDIFYING) {
                        container.CurrentStateM = HUMIDIFYING;
                        ESP_LOGI(TAG, "AUTO (HUMIDIFYING) Switch off the Relay");
                        OutputWrite(false, CONFIG_RELAY_GPIO, false);
                    }
                } else if (container.CurrentHumidityM > (container.DehumidityThresholdM + hysteresis) && container.CurrentHumidityM < (container.HumidityThresholdM - hysteresis)) {
                    if (state != IDLE) {
                        container.CurrentStateM = IDLE;
                        ESP_LOGI(TAG, "AUTO IDLE Switch off All Relays");
                        OutputWrite(false, CONFIG_RELAY_GPIO, false);
                    }
                } else if (container.CurrentHumidityM >= container.HumidityThresholdM) {
                    if (state != DEHUMIDIFYING) {
                        if (!container.wasWaterTankFull) {
                            container.CurrentStateM = DEHUMIDIFYING;
                            ESP_LOGI(TAG, "AUTO DEHUMIDIFYING Switch on Relay");
                            OutputWrite(true, CONFIG_RELAY_GPIO, false);
                        }
                    } else if (container.wasWaterTankFull) {
                        container.CurrentStateM = INACTIVE;
                        ESP_LOGI(TAG, "The tank is full. INACTIVE Switch off the Relay");
                        OutputWrite(false, CONFIG_RELAY_GPIO, false);
                    }
                }
            break;
                
            default:
                ESP_LOGI(TAG, "Unsupported case");
          }
    } else {
        //Ако е Inactive
        container.CurrentStateM = INACTIVE;
        if (state != container.CurrentStateM) {
           ESP_LOGI(TAG, "INACTIVE by user Switch off the Relay");
           OutputWrite(false, CONFIG_RELAY_GPIO, false);
        }
    }
    
    
    if (active_state != container.ActiveM) {
        hap_val_t ActiveState = {.u = container.ActiveM};
        hap_char_update_val(ActiveC, &ActiveState);
    }
    
    if (state != container.CurrentStateM) {
        hap_val_t CurrentState = {.u = container.CurrentStateM};
        hap_char_update_val(CurrentStateC, &CurrentState);
    }
    
    if (UpdateHum) {
        hap_val_t CurrentHumidity = {.f = ((float)container.CurrentHumidityM)};
        hap_char_update_val(CurrentHumidityC, &CurrentHumidity);
    }
}

static void reset_network_handler(void* arg)
{
    ESP_LOGI(TAG, "Reset WiFi settings");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ledFlash(2, 100);
    hap_reset_network();
}

static void reset_to_factory_handler(void* arg)
{
    OutputWrite(true, CONFIG_LED_GPIO, false);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ledFlash(4, 100);
    hap_reset_to_factory();
}

static void fw_update_handler(void* arg)
{
    container.FirmwareUpdate = true;
    
    if (otaUpdateStatus != FW_UPG_STATUS_UPGRADING) {
        otaUpdate();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        xTaskCreate(otaStatus, "otaState", 2*1024, NULL, 3, NULL);
    }
}

static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
    iot_button_set_evt_cb(handle, BUTTON_CB_TAP, fw_update_handler, NULL);
    //iot_button_set_evt_cb(handle, BUTTON_CB_PUSH, fw_update_handler, NULL);
}

static int AccessoryIdentify(hap_acc_t *ha)
{
    //ESP_LOGI(TAG, "Accessory identified");

    for (int i=0; i<3; i++) {
        ledFlash(3, 100);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    OutputWrite(container.StatusConnected, CONFIG_LED_GPIO, false);
    return HAP_SUCCESS;
}

static void InitializePlatform(void) {
    
    ioInitPdown(CONFIG_RELAY_GPIO, true, true);
    ioInit(CONFIG_LED_GPIO, true, false);
    ioInit(CONFIG_LOCAL_ON_GPIO, false, true);
    ioInit(CONFIG_LVL_SWITCH_GPIO, false, true);
    
    container.FirmwareUpdate = false;
    container.StatusConnected = false;
    container.CurrentStateM = INACTIVE;
    container.CurrentHumidityM = 40;
    
    OutputWrite(container.StatusConnected, CONFIG_LED_GPIO, false);
    OutputWrite(false, CONFIG_RELAY_GPIO, false);
    
    hap_keystore_init();
    uint8_t state = 3;
    size_t size = sizeof(uint8_t);
    
    if (hap_keystore_get("dehum", "active", (uint8_t *)&state, &size) == HAP_SUCCESS && state == 1) {
        container.ActiveM = ACTIVE_STATE;
    } else {
        //ESP_LOGE(TAG, "Could not read Active state");
        container.ActiveM = INACTIVE;
    }
     
    if (hap_keystore_get("dehum", "tstate", (uint8_t *)&state, &size) == HAP_SUCCESS && state < 3) {
        container.TargetStateM = state;
    } else {
        ESP_LOGE(TAG, "Could not read TargetState");
        container.TargetStateM = DEHUMIDIFIER;
    }
       
    if (hap_keystore_get("dehum", "humidity", (uint8_t *)&state, &size) == HAP_SUCCESS && state < 101) {
        container.HumidityThresholdM = state;
    } else {
        ESP_LOGE(TAG, "Could not read Humidity Threshold");
        container.HumidityThresholdM = 60;
    }
    
    if (hap_keystore_get("dehum", "dehumidity", (uint8_t *)&state, &size) == HAP_SUCCESS && state < 101) {
        container.DehumidityThresholdM = state;
    } else {
        ESP_LOGE(TAG, "Could not read Dehumidity Threshold");
        container.DehumidityThresholdM = 40;
    }
    
    if (hap_keystore_get("dehum", "watertank", (uint8_t *)&state, &size) == HAP_SUCCESS && state == 1) {
        container.wasWaterTankFull = true;
    } else {
        //ESP_LOGE(TAG, "Could not read Water Level Switch state");
        container.wasWaterTankFull = false;
    }

    if (hap_keystore_get("dehum", "lvlwater", (uint8_t *)&state, &size) == HAP_SUCCESS && state < 101) {
        container.WaterLevelM = state * 10;
    } else {
        //ESP_LOGE(TAG, "Could not read Water Level");
        container.WaterLevelM = 0;
    }
    
    container.LocalOn = !ReadInput(CONFIG_LOCAL_ON_GPIO);
    //container.isWaterTankFull = ReadInput(CONFIG_LVL_SWITCH_GPIO);
    
    xTaskCreate(ReadHumidityTask, "Read Humidity Task", 2048, NULL, 5, NULL);
}

static int WriteSettings(uint8_t ValueNum) {
    int ret = HAP_FAIL;
    uint8_t Value;
        
    /*
     ValueNum = 0;
     uint_8 ActiveM;
     
     ValueNum = 1;
     uint8_t TargetStateM;
     
     ValueNum = 2;
     uint8_t HumidityThresholdM;
     
     ValueNum = 3;
     bool wasWaterTankFull;
     uint16_t WaterLevelM;
     */
        
    switch (ValueNum) {
        case 0:
            Value = container.ActiveM;
            
            ret = hap_keystore_set("dehum", "active", (uint8_t *)&Value, sizeof(uint8_t));
            if (ret != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write Active state");
            }
            break;
            
        case 1:
            Value = container.TargetStateM;
            
            ret = hap_keystore_set("dehum", "tstate", (uint8_t *)&Value, sizeof(uint8_t));
            if (ret != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write Target state");
            }
            break;
            
        case 2:
            Value = container.HumidityThresholdM;
            
            ret = hap_keystore_set("dehum", "humidity", (uint8_t *)&Value, sizeof(uint8_t));
            if (ret != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write Humidity Threshold");
            }
            
            Value = container.DehumidityThresholdM;
            
            ret = hap_keystore_set("dehum", "dehumidity", (uint8_t *)&Value, sizeof(uint8_t));
            if (ret != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write Dehumidity Threshold");
            }
            break;
            
        case 3:
            Value = container.wasWaterTankFull;
            
            ret = hap_keystore_set("dehum", "watertank", (uint8_t *)&Value, sizeof(uint8_t));
            if (ret != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write Water Level Switch state");
            }

            Value = (uint8_t)(container.WaterLevelM / 10);
            
            ret = hap_keystore_set("dehum", "lvlwater", (uint8_t *)&Value, sizeof(uint8_t));
            if (ret != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write Water Level");
            }
            
            break;
            
        default:
            ESP_LOGE(TAG, "Possible values exceeded");
            break;
    }

    return ret;
}

//HAP_CHAR_UUID_STATUS_ACTIVE
//HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY
//HAP_CHAR_UUID_CURRENT_HUMIDIFIER_DEHUMIDIFIER_STATE
//HAP_CHAR_UUID_TARGET_HUMIDIFIER_DEHUMIDIFIER_STATE
//HAP_CHAR_UUID_RELATIVE_HUMIDITY_DEHUMIDIFIER_THRESHOLD
//HAP_CHAR_UUID_WATER_LEVEL
//HAP_CHAR_CUSTOM_UUID_TROTA


static int AccessoryRead(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    } //returns the HAP controller ID

    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_ACTIVE)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        ESP_LOGI(TAG, "Read ActiveState is %s", cur_val->u ? "Active" : "Inactive");
        hap_val_t ActiveState = {.u = container.ActiveM};
        hap_char_update_val(hc, &ActiveState);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t CurrentHumidity = {.f = (float)container.CurrentHumidityM};
        ESP_LOGI(TAG, "Last value CurrentHumidity %f", cur_val->f);
        ESP_LOGI(TAG, "Read the CurrentHumidity %f", CurrentHumidity.f);
        hap_char_update_val(hc, &CurrentHumidity);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_HUMIDIFIER_DEHUMIDIFIER_STATE)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t CurrentState = {.u = container.CurrentStateM};
        ESP_LOGI(TAG, "Last value CurrentState %li", cur_val->u);
        ESP_LOGI(TAG, "Read the CurrentState %li", CurrentState.u);
        hap_char_update_val(hc, &CurrentState);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_TARGET_HUMIDIFIER_DEHUMIDIFIER_STATE)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t TargetState = {.u = container.TargetStateM};
        ESP_LOGI(TAG, "Last value TargetState %li", cur_val->u);
        ESP_LOGI(TAG, "Read the TargetState %li", TargetState.u);
        hap_char_update_val(hc, &TargetState);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_RELATIVE_HUMIDITY_HUMIDIFIER_THRESHOLD)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t HumidityThreshold = {.f = (float)container.HumidityThresholdM};
        ESP_LOGI(TAG, "Last value HumidityThreshold %f", cur_val->f);
        ESP_LOGI(TAG, "Read the HumidityThreshold %f", HumidityThreshold.f);
        hap_char_update_val(hc, &HumidityThreshold);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_RELATIVE_HUMIDITY_DEHUMIDIFIER_THRESHOLD)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t DehumidityThreshold = {.f = (float)container.DehumidityThresholdM};
        ESP_LOGI(TAG, "Last value HumidityThreshold %f", cur_val->f);
        ESP_LOGI(TAG, "Read the HumidityThreshold %f", DehumidityThreshold.f);
        hap_char_update_val(hc, &DehumidityThreshold);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_WATER_LEVEL)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t WaterLevel = {.f = ((float)container.WaterLevelM / 10)};
        ESP_LOGI(TAG, "Last value WaterLevel %f", cur_val->f);
        ESP_LOGI(TAG, "Read the WaterLevel %f", WaterLevel.f);
        hap_char_update_val(hc, &WaterLevel);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_CUSTOM_UUID_FW_UPG_STATUS)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        ESP_LOGI(TAG, "Previous value was %d", cur_val->i);
        hap_val_t UpdateStatus = {.i = otaUpdateStatus};
        ESP_LOGI(TAG, "Controller reads the Update status %d", UpdateStatus.i);
        hap_char_update_val(hc, &UpdateStatus);
        *status_code = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_CUSTOM_UUID_TROTA)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        ESP_LOGI(TAG, "Currently the OTA is %s", cur_val->b ? "Triggered" : "Inactive");
        hap_val_t FirmwareUpdateTrigger = {.b = container.FirmwareUpdate};
        hap_char_update_val(hc, &FirmwareUpdateTrigger);
        *status_code = HAP_STATUS_SUCCESS;
    } else {
        *status_code = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }
    return ret;
}

//HAP_CHAR_UUID_STATUS_ACTIVE
//HAP_CHAR_UUID_TARGET_HUMIDIFIER_DEHUMIDIFIER_STATE
//HAP_CHAR_UUID_RELATIVE_HUMIDITY_DEHUMIDIFIER_THRESHOLD
//HAP_CHAR_CUSTOM_UUID_TROTA

/* A callback for handling a write characteristic */
static int AccessoryWrite(hap_write_data_t write_data[], int count, void *serv_priv, void *write_priv)
{
    if (hap_req_get_ctrl_id(write_priv)) {
        ESP_LOGI(TAG, "Received write from %s", hap_req_get_ctrl_id(write_priv));
    }
    ESP_LOGI(TAG, "Write called with %d chars", count);

    int i, ret, ValueNum = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ACTIVE)) {
            ESP_LOGI(TAG, "Write ActiveState is %s", write->val.u ? "Active" : "Inactive");
            
            ValueNum = 0;
            container.ActiveM = write->val.u;
            
            UpdateState(true);
            
            hap_char_update_val(write->hc, &(write->val));
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_HUMIDIFIER_DEHUMIDIFIER_STATE)) {
            
            ValueNum = 1;
            ESP_LOGI(TAG, "Write set TargetState %li", write->val.u);
            container.TargetStateM = write->val.u;
            
            UpdateState(true);
            
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
                
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_RELATIVE_HUMIDITY_HUMIDIFIER_THRESHOLD)) {
            
            ValueNum = 2;
            ESP_LOGI(TAG, "Write set HumidityThreshold is %f", write->val.f);
            container.HumidityThresholdM = write->val.f;
            
            UpdateState(true);
            
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
            
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_RELATIVE_HUMIDITY_DEHUMIDIFIER_THRESHOLD)) {
            
            ValueNum = 2;
            ESP_LOGI(TAG, "Write set DehumidityThreshold is %f", write->val.f);
            container.DehumidityThresholdM = write->val.f;
            
            UpdateState(true);
            
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
            
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_CUSTOM_UUID_TROTA)) {
            ESP_LOGI(TAG, "The OTA update is %s", write->val.b ? "Triggered" : "Inactive");
            
            container.FirmwareUpdate = write->val.b;
            
            if (container.FirmwareUpdate) {

                if (otaUpdateStatus != FW_UPG_STATUS_UPGRADING) {
                    otaUpdate();
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    xTaskCreate(otaStatus, "otaState", 2*1024, NULL, 3, NULL);
                }
            }
            
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
            ret = HAP_FAIL;
        }
    }
    WriteSettings(ValueNum);
    return ret;
}

/*The main thread for handling the Accessory */
static void AccessoryThreadEntry(void *p)
{
    char AccessoryName[16];
    char wifi_mac_address[16];
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(wifi_mac_address, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device WIFI mac_address: %s", wifi_mac_address);

    sprintf(AccessoryName, CONFIG_NAME, mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG,"Device Name: %s", AccessoryName);

    hap_acc_t *accessory;
    hap_serv_t *service;
    
    hap_init(HAP_TRANSPORT_WIFI);

    hap_acc_cfg_t cfg = {
        .name = AccessoryName,
        .manufacturer = CONFIG_MANIFACTURER,
        .model = CONFIG_MODEL,
        .serial_num = wifi_mac_address,
        .fw_rev = CONFIG_APP_PROJECT_VER,
        .hw_rev = CONFIG_HW,
        .pv = "1.1.0",
        .identify_routine = AccessoryIdentify,
        .cid = HAP_CID_DEHUMIDIFIER,
    };

    accessory = hap_acc_create(&cfg);

    uint8_t product_data[] = {'D','E','H','U','M','D','F','R'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));
    
    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    service = hap_serv_humidifier_dehumidifier_create(0, 10, 0, 0);
    hap_serv_add_char(service, hap_char_name_create("Dehumidifier"));
    hap_serv_add_char(service, hap_char_relative_humidity_dehumidifier_threshold_create(40));
    hap_serv_add_char(service, hap_char_relative_humidity_humidifier_threshold_create(60));
    hap_serv_add_char(service, hap_char_water_level_create(0));
    
    ActiveC = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_ACTIVE);
    CurrentStateC = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_HUMIDIFIER_DEHUMIDIFIER_STATE);
    CurrentHumidityC = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);
    
    TriggerUpdateChar = hap_char_bool_create(HAP_CHAR_CUSTOM_UUID_TROTA, HAP_CHAR_PERM_PR | HAP_CHAR_PERM_PW | HAP_CHAR_PERM_EV, 0);
    hap_serv_add_char(service, TriggerUpdateChar);
    hap_char_add_description(TriggerUpdateChar, "Firmware Update");
    
    UpdateStatusChar = hap_char_int_create(HAP_CHAR_CUSTOM_UUID_FW_UPG_STATUS, HAP_CHAR_PERM_PR | HAP_CHAR_PERM_EV, 0);
    hap_serv_add_char(service, UpdateStatusChar);
    hap_char_add_description(UpdateStatusChar, "FW Update Status");
    
    /*
    HAP_CHAR_UUID_STATUS_ACTIVE
    HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY
    HAP_CHAR_UUID_CURRENT_HUMIDIFIER_DEHUMIDIFIER_STATE
    HAP_CHAR_UUID_TARGET_HUMIDIFIER_DEHUMIDIFIER_STATE
    HAP_CHAR_UUID_RELATIVE_HUMIDITY_DEHUMIDIFIER_THRESHOLD
    HAP_CHAR_UUID_WATER_LEVEL
    HAP_CHAR_CUSTOM_UUID_TROTA
     */
    
    hap_serv_set_write_cb(service, AccessoryWrite);
    hap_serv_set_read_cb(service, AccessoryRead);
    hap_acc_add_serv(accessory, service);
    
    hap_add_accessory(accessory);

    reset_key_init(CONFIG_RESET_GPIO);
    
    InitializePlatform();

    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
                hap_get_paired_controller_count());

#ifdef CONFIG_USE_HARDCODED_SETUP_CODE
    hap_set_setup_code(CONFIG_SETUP_CODE);
    hap_set_setup_id(CONFIG_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_SETUP_CODE, CONFIG_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_SETUP_CODE, CONFIG_SETUP_ID, false, cfg.cid);
#endif
#endif

    //hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    app_wifi_init(AccessoryName);

    hap_start();

    app_wifi_start(portMAX_DELAY);

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(AccessoryThreadEntry, ACC_TASK_NAME, ACC_TASK_STACKSIZE, NULL, ACC_TASK_PRIORITY, NULL);
}
