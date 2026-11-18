/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <esp_err.h>

//external function for transfer the signal for status connected
extern void TakeStatusConnected(bool status);

void app_wifi_init(const char *hostname);
esp_err_t app_wifi_start(TickType_t ticks_to_wait);
