Irrigation valve control based on ESP8266

The device is working on rechargeable Li-ion batteries (3x 4.2V / 2800 mAh) combined with 10W Solar module and two voltage regulators. One for charging the batteries and another one for feeding the ESP8266 module.
The system also includes a DHT22 module for measuring the outside Temperature and Humidity.
All of that is working with HomeKit and have the count down irrigation timer option. The time can be selected in the native Apple Home app between 5 and 60 minutes. Every value in this range is possible, if Siri is used.

The project is based on the most recent ESP8266_RTOS_SDK master <https://github.com/espressif/ESP8266_RTOS_SDK>. With Espressif software optimizations, the module reduces the power consumption of the chip (by nearly 70% comparing to the esp-open-rtos) and allows it to run on battery power supply.
The current setup gives the possibility for at least 5 days on single charge, without sunlight. Normally on sunny days the battery pack is charging for 4 hours.

The HomeKit support rely on Espressif's esp-homekit-sdk <https://github.com/espressif/esp-homekit-sdk>

For DHT22 the Ruslan V. Uss library is used <https://github.com/UncleRus>
However there is some kind of conflict and the LOGI and LOGE should be disabled during the I2C communication with the sensor.
