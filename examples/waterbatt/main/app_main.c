/* esp32s2 4MB module for Watering + Temperature and Humidity sensors on Battery */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_mac.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <app_hap_setup_payload.h>

#include "iot_button.h"

#include "app_wifi.h"
#include "outputwrite.h"
#include "fupdateota.h"

#include "esp_aht.h"

static const char *TAG = "HAP Accessory";

#define ACC_TASK_PRIORITY  1
#define ACC_TASK_STACKSIZE 4 * 1024
#define ACC_TASK_NAME      "hap_accessory"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     7

// Valve type is 0, 1, 2 or 3 [generic, irrigation, shower head, water faucet]
#define V_TYPE 1

#define BATT_NORMAL     0
#define BATT_LOW        1

#define NOT_CHARGING    0
#define CHARGING        1
#define NOT_CHARGEABLE  2

#define BATTERY_MIN     593
#define DELTA_MIN_MAX   195 //788 - 593 = 238

//Use this constant for correct calculation of battery level.
//220 -> 9V -> 0% (Battery voltage level in mV -> 0% - 593mV/9,40V; 100% - 788mV/12,5V)
//Battery Level is from 0 to 100 percentage
//Charging State is 0 - Not charging, 1 - Charging
//Battery Status is 0 - Battery level is normal, 1 - Battery level is low
//Time duration is from 0 to 3600 in 1 s steps

static struct {
    bool ActiveM;
    bool InUseStateM;
    bool FirmwareUpdate;
    bool StatusConnected;
    bool isCalibrated;
    bool ValveTimerCreated;
    uint16_t SetDurationM;
    uint16_t RemDurationM;
    uint16_t HumidityM;
    int16_t TemperatureM;
    adc_cali_handle_t CalHandle;
    adc_oneshot_unit_handle_t adcParmHandle;
} container;

TimerHandle_t vTimer;

static hap_char_t *ValveActive;
static hap_char_t *ValveInUse;
static hap_char_t *RemDuration;

static hap_char_t *TriggerUpdateChar;
static hap_char_t *UpdateStatusChar;

#define HAP_CHAR_CUSTOM_UUID_FW_UPG_STATUS  "d5703b5e-3736-11e8-b467-0ed5f89f718b"
#define HAP_CHAR_CUSTOM_UUID_TROTA          "F0090013-079E-11e8-8F27-9C2605A29F52"

int hap_keystore_init();
int hap_keystore_set(const char *name_space, const char *key, const uint8_t *val, const size_t val_len);
int hap_keystore_get(const char *name_space, const char *key, uint8_t *val, size_t *val_size);
int hap_keystore_delete(const char *name_space, const char *key);
void TemperatureSensorTask(void *_args);

static TaskHandle_t SensorTaskHandle;   //the sensor task runs at most once
static bool AhtReady;                   //ahtInit() done (the I2C bus is started only once)

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
    //uint32_t memSize = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    //ESP_LOGI(TAG, "Free memmory size is %lu", memSize);
    
    otaState.i = otaUpdateStatus;
    hap_char_update_val(UpdateStatusChar, &otaState);
    
    if (otaUpdateStatus == -1) {
        //update Failed
        if (!SensorTaskHandle) {
            xTaskCreate(TemperatureSensorTask, "Temperature Sensor", 2*1024, NULL, 5, &SensorTaskHandle);
        }
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

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    ESP_LOGI(TAG, "Reset WiFi settings");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ledFlash(2, 100);
    hap_reset_network();
}

/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ledFlash(4, 100);
    hap_reset_to_factory();
}

static void fw_update_handler(void* arg)
{
    container.FirmwareUpdate = true;

    if (container.FirmwareUpdate) {

        if (otaUpdateStatus != FW_UPG_STATUS_UPGRADING) {
            otaUpdate();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            xTaskCreate(otaStatus, "otaState", 2*1024, NULL, 3, NULL);
        }
    }
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
    iot_button_set_evt_cb(handle, BUTTON_CB_TAP, fw_update_handler, NULL);
}

static uint8_t ReadBattery() {
    uint8_t BatteryLevel = 50;
    
    uint16_t Voltage = adcRead(container.adcParmHandle, CONFIG_ADC_CHANNEL, container.isCalibrated, container.CalHandle);
    ESP_LOGI(TAG, "Voltage read: %d", Voltage);
    
    /*
    Measure the voltage at TOUT pin (A0) - 1V max
    ADC voltage is %.3f", 1.0 / 1024 * sdk_system_adc_read()
    voltage = (1000.0 / 1024) * voltage; //mV
    BatteryLevel = 1.1 * (voltage - VOLTAGE_CAL);
     */
    
    if (Voltage < BATTERY_MIN) {
        BatteryLevel = 0;
    } else {
        BatteryLevel = ((Voltage - BATTERY_MIN) * 100) / DELTA_MIN_MAX;
        if (BatteryLevel > 99) {
            BatteryLevel = 100;
        } else if (BatteryLevel < 2) {
            BatteryLevel = 1;
        }
    }
    
    return BatteryLevel;
}

static void NotifyActiveState(void) {
    hap_val_t ActiveValue;
    hap_val_t InUseState;
    hap_val_t RemDurationValue;

    ActiveValue.i = container.ActiveM;
    InUseState.i = container.InUseStateM;
    RemDurationValue.i = container.RemDurationM;
    
    OutputWrite(container.ActiveM, CONFIG_RELAY_GPIO, false);
    
    hap_char_update_val(ValveActive, &ActiveValue);
    hap_char_update_val(ValveInUse, &InUseState);
    hap_char_update_val(RemDuration, &RemDurationValue);
}

static void DisableValveTimer(void){
    if (xTimerIsTimerActive(vTimer)) {
        xTimerStop(vTimer, 0);
    }
    xTimerDelete(vTimer, 0);
    container.ValveTimerCreated = false;
    ESP_LOGI(TAG, "Valve Timer deleted");
    NotifyActiveState();
}

static void TimeUpCallback(TimerHandle_t xTimer) {
    container.RemDurationM--;
    if (container.RemDurationM == 0) {
        container.ActiveM = false;
        container.InUseStateM = false;
        DisableValveTimer();
    } else if (!container.ActiveM) {
        DisableValveTimer();
    } else {
        xTimerStart(vTimer, 0);
    }
}

static void CreateValveTimer(void) {
    if (!container.ValveTimerCreated) {
        vTimer = xTimerCreate("Valve timer", pdMS_TO_TICKS(1000), pdTRUE, ( void * ) 0, TimeUpCallback);
        container.ValveTimerCreated = true;
        ESP_LOGI(TAG, "Valve Timer created");
    } else if (xTimerIsTimerActive(vTimer)) {
        xTimerStop(vTimer, 0);
    }
    xTimerStart(vTimer, 1);
    ESP_LOGI(TAG, "Valve Timer started");
}

void TemperatureSensorTask(void *_args) {

    if (!AhtReady) {
        AhtReady = ahtInit(AHT21) == ESP_OK;
        ESP_LOGI(TAG, "AHT Sensor is %s", AhtReady ? "initialized" : "not initialized");
    }
    
    while (!container.FirmwareUpdate) {
        float Temperature;
        uint8_t Humidity;
        esp_err_t err = ahtRead(&Temperature, &Humidity);
        if (err == ESP_OK) {
            container.HumidityM = 10 * Humidity;
            container.TemperatureM = (int16_t)lroundf(10.0f * Temperature);
            ESP_LOGI(TAG, "Current Humidity %i, Temperature %i", container.HumidityM, container.TemperatureM);
        } else {
            ESP_LOGE(TAG, "AHT read error: %s", esp_err_to_name(err));   //the last values stay
        }
        
        vTaskDelay(180000 / portTICK_PERIOD_MS);
    }
    
    SensorTaskHandle = NULL;
    vTaskDelete(NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int AccessoryIdentify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");

    for (int i=0; i<3; i++) {
        ledFlash(3, 100);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    return HAP_SUCCESS;
}

static void InitializePlatform() {
    
    container.ActiveM = false;
    container.InUseStateM = false;
    container.SetDurationM = 3600;
    container.RemDurationM = 0;
    container.TemperatureM = 200;
    container.HumidityM = 500;
    container.FirmwareUpdate = false;
    
    ioInit(CONFIG_RELAY_GPIO, true, false);
    ioInit(CONFIG_LED_GPIO, true, false);
    
    ioInit(CONFIG_CHARGE_GPIO, false, true);
    
    container.isCalibrated = adcCalibrationInit(CONFIG_ADC_UNIT, CONFIG_ADC_CHANNEL, CONFIG_ADC_ATTENUATION, &container.adcParmHandle, &container.CalHandle);
    
    hap_keystore_init();
    uint8_t state = 0;
    size_t size = sizeof(state);
    
    if (hap_keystore_get("water", "sduration", (uint8_t *)&state, &size) == HAP_SUCCESS) {
        container.SetDurationM = state * 100;
    } else {
        container.SetDurationM = 3600;
    }
    
    OutputWrite(container.ActiveM, CONFIG_RELAY_GPIO, false);
}
    
static int ValveRead(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_ACTIVE)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t ActiveValue = {.i = container.ActiveM};
        //ActiveValue.i = container.ActiveM;
        ESP_LOGI(TAG, "Current active state is %s", cur_val->i ? "On" : "Off");
        hap_char_update_val(hc, &ActiveValue);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_IN_USE)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t InUseState = {.i = container.InUseStateM};
        //InUseState.i = container.InUseStateM;
        ESP_LOGI(TAG, "Currently the valve is %s", cur_val->i ? "In use" : "Not in use");
        hap_char_update_val(hc, &InUseState);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_SET_DURATION)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        ESP_LOGI(TAG, "The set duration is %lu s", cur_val->u);
        hap_val_t SetDurationValue = {.u = container.SetDurationM};
        //SetDurationValue.u = container.SetDurationM;
        hap_char_update_val(hc, &SetDurationValue);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_REMAINING_DURATION)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t RemDurationValue = {.u = container.RemDurationM};
        //RemDurationValue.u = container.RemDurationM;
        ESP_LOGI(TAG, "The remaining duration is %lu s", cur_val->u);
        hap_char_update_val(hc, &RemDurationValue);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_CUSTOM_UUID_FW_UPG_STATUS)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        ESP_LOGI(TAG, "Previous value was %d", cur_val->i);
        hap_val_t UpdateStatus = {.i = otaUpdateStatus};
        ESP_LOGI(TAG, "Controller reads the Update status %d", UpdateStatus.i);
        hap_char_update_val(hc, &UpdateStatus);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_CUSTOM_UUID_TROTA)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        ESP_LOGI(TAG, "Currently the OTA is %s", cur_val->b ? "Triggered" : "Inactive");
        hap_val_t FirmwareUpdateTrigger = {.b = container.FirmwareUpdate};
        hap_char_update_val(hc, &FirmwareUpdateTrigger);
        *status = HAP_STATUS_SUCCESS;
    } else {
        *status = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }
    return ret;
}

static int ValveWrite(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ACTIVE)) {
            ESP_LOGI(TAG, "Write active state %s", write->val.i ? "On" : "Off");
            
            container.ActiveM = write->val.i;
            
            if (container.ActiveM) {
                container.InUseStateM = container.ActiveM;
                if (container.SetDurationM > 0 && container.ValveTimerCreated == true) {
                    if (xTimerIsTimerActive(vTimer)) {
                        ESP_LOGE(TAG, "Valve timer is already started.\n");
                    }
                } else {
                    container.RemDurationM = container.SetDurationM;
                    CreateValveTimer();
                }
            } else {
                container.InUseStateM = container.ActiveM;
                container.RemDurationM = 0;
            }
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
            NotifyActiveState();
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SET_DURATION)) {
            ESP_LOGI(TAG, "Write set duration is %lu s", write->val.u);
            container.SetDurationM = write->val.u;
            container.RemDurationM = container.SetDurationM;
            uint8_t duration = container.SetDurationM / 100;
            if (hap_keystore_set("water", "sduration", (uint8_t *)&duration, sizeof(uint8_t)) != HAP_SUCCESS) {
                ESP_LOGE(TAG, "Could not write set duration.\n");
            }
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
    return ret;
}

static int BatteryRead(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_BATTERY_LEVEL)) {
        const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t BatteryLevelValue;
        BatteryLevelValue.i = ReadBattery();
        ESP_LOGI(TAG, "The battery level was %d", cur_val->i);
        ESP_LOGI(TAG, "The read battery level is %d", BatteryLevelValue.i);
        hap_char_update_val(hc, &BatteryLevelValue);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CHARGING_STATE)) {
        hap_val_t ChargingState;
        //ChargingState.i = !gpio_get_level(CONFIG_CHARGE_GPIO);
        if (ReadInput(CONFIG_CHARGE_GPIO)) {
           ChargingState.i = NOT_CHARGING;
        } else {
           ChargingState.i = CHARGING;
        }
        
        ESP_LOGI(TAG, "Currently the battery is %s", ChargingState.i ? "Charging" : "Not charging");
        hap_char_update_val(hc, &ChargingState);
        *status = HAP_STATUS_SUCCESS;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_STATUS_LOW_BATTERY)) {
        hap_val_t LowBatteryState;
        hap_val_t BatteryLevelValue;
        BatteryLevelValue.i = ReadBattery();
        if (BatteryLevelValue.i < 20) {
           LowBatteryState.i = BATT_LOW;
        } else {
           LowBatteryState.i = BATT_NORMAL;
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

static int TemperatureRead(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_TEMPERATURE)) {
        //const hap_val_t *cur_val = hap_char_get_val(hc);
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

static int HumidityRead(hap_char_t *hc, hap_status_t *status, void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) {
        //const hap_val_t *cur_val = hap_char_get_val(hc);
        hap_val_t HumidityValue;
        HumidityValue.f = container.HumidityM / 10.0;
        ESP_LOGI(TAG, "The current humidity is %f", HumidityValue.f);
        hap_char_update_val(hc, &HumidityValue);
        *status = HAP_STATUS_SUCCESS;
    } else {
        *status = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }
    return ret;
}

/*The main thread for handling the Smart Outlet Accessory */
static void acc_thread_entry(void *p)
{
    char AccessoryName[16];
    char WiFiMacAddress[16];
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(WiFiMacAddress, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device WIFI mac_address: %s", WiFiMacAddress);

    sprintf(AccessoryName, CONFIG_NAME, mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG,"Device Name: %s", AccessoryName);
    
    hap_acc_t *accessory;
    hap_serv_t *service;
    
    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = AccessoryName,
        .manufacturer = CONFIG_MANIFACTURER,
        .model = CONFIG_MODEL,
        .serial_num = WiFiMacAddress,
        .fw_rev = CONFIG_APP_PROJECT_VER,
        .hw_rev = CONFIG_HW,
        .pv = "1.1.0",
        .identify_routine = AccessoryIdentify,
        .cid = HAP_CID_SPRINKLER,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'W','A','T','E','R','B','A','T'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);
    
    //hap_serv_t *hap_serv_valve_create(uint8_t active, uint8_t in_use, uint8_t valve_type);
    service = hap_serv_create(HAP_SERV_UUID_VALVE);
    //svalve = hap_serv_valve_create(ActiveValue.i, InUseState.i, V_TYPE);
    hap_serv_add_char(service, hap_char_name_create(AccessoryName));
    hap_serv_add_char(service, hap_char_active_create(0)); //ActiveValue.i
    hap_serv_add_char(service, hap_char_in_use_create(0)); //InUseState.i
    hap_serv_add_char(service, hap_char_valve_type_create(V_TYPE));
    
    hap_serv_add_char(service, hap_char_set_duration_create(3600)); //SetDurationValue.u
    hap_serv_add_char(service, hap_char_remaining_duration_create(0)); //RemDurationValue.u
    
    ValveActive = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_ACTIVE);
    ValveInUse = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_IN_USE);
    RemDuration = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_REMAINING_DURATION);
    
    TriggerUpdateChar = hap_char_bool_create(HAP_CHAR_CUSTOM_UUID_TROTA, HAP_CHAR_PERM_PR | HAP_CHAR_PERM_PW | HAP_CHAR_PERM_EV, 0);
    hap_serv_add_char(service, TriggerUpdateChar);
    hap_char_add_description(TriggerUpdateChar, "Firmware Update");
    
    UpdateStatusChar = hap_char_int_create(HAP_CHAR_CUSTOM_UUID_FW_UPG_STATUS, HAP_CHAR_PERM_PR | HAP_CHAR_PERM_EV, 0);
    hap_serv_add_char(service, UpdateStatusChar);
    hap_char_add_description(UpdateStatusChar, "FW Update Status");
    
    //hap_char_t *ValveActive;
    //hap_char_t *ValveInUse;
    //hap_char_t *RemDuration;
    //HAP_CHAR_UUID_ACTIVE
    //HAP_CHAR_UUID_SET_DURATION
    //HAP_CHAR_UUID_REMAINING_DURATION
    //HAP_CHAR_UUID_IN_USE
    //HAP_CHAR_UUID_VALVE_TYPE
    
    //Set the read callback for the service (optional)
    hap_serv_set_read_cb(service, ValveRead);

    //Set the write callback for the service
    hap_serv_set_write_cb(service, ValveWrite);
    
    //Add the Valve Service to the Accessory Object
    hap_acc_add_serv(accessory, service);
    
    //Create the Battery service
    
    //HAP_SERV_UUID_BATTERY_SERVICE
    //hap_serv_t *hap_serv_battery_service_create(uint8_t battery_level, uint8_t charging_state, uint8_t status_low_battery);
    service = hap_serv_battery_service_create(50, NOT_CHARGING, BATT_NORMAL); //BatteryLevelValue.i, ChargingState.i, LowBatteryState.i
        
    //HAP_CHAR_UUID_BATTERY_LEVEL
    //HAP_CHAR_UUID_CHARGING_STATE
    //HAP_CHAR_UUID_STATUS_LOW_BATTERY
    hap_serv_set_read_cb(service, BatteryRead);
    hap_acc_add_serv(accessory, service);
    
    
    //Create the Current Relative Humidity service
    
    //HAP_SERV_UUID_HUMIDITY_SENSOR
    service = hap_serv_humidity_sensor_create(50.0); //HumidityValue.f
        
    hap_serv_set_read_cb(service, HumidityRead);
    hap_acc_add_serv(accessory, service);

    //Create the Current Temperature service
    
    service = hap_serv_create(HAP_SERV_UUID_TEMPERATURE_SENSOR);
    hap_char_t *Temperature;
    Temperature = hap_char_float_create(HAP_CHAR_UUID_CURRENT_TEMPERATURE,
                                        HAP_CHAR_PERM_PR | HAP_CHAR_PERM_EV, 20);
    hap_char_float_set_constraints(Temperature, -50.0, 100.0, 0.1);
    hap_char_add_unit(Temperature, HAP_CHAR_UNIT_CELSIUS);
    
    hap_serv_add_char(service, Temperature);
        
    hap_serv_set_read_cb(service, TemperatureRead);
    hap_acc_add_serv(accessory, service);
    
    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);
    
    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(CONFIG_BUTTON_GPIO);
    
    InitializePlatform();
    
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
#ifdef CONFIG_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_SETUP_CODE, CONFIG_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_SETUP_CODE, CONFIG_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    //hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init(AccessoryName);
    
    //hap_http_debug_enable();
    
    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    
    xTaskCreate(TemperatureSensorTask, "Temperature Sensor", 2*1024, NULL, 5, &SensorTaskHandle);

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

void app_main()
{
    /* Create the application thread */
    xTaskCreate(acc_thread_entry, ACC_TASK_NAME, ACC_TASK_STACKSIZE,
                NULL, ACC_TASK_PRIORITY, NULL);
}
