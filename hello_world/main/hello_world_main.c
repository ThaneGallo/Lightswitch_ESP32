/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"


#include "driver/gpio.h"

#define GPIO_BTN 25
#define ESP_INTR_FLAG_DEFAULT 0
#define TAG "LIGHT-SWITCH"

uint8_t toggle_switch = 0;


void IRAM_ATTR gpio_btn_handler(void *arg)
{
    
    ESP_DRAM_LOGI(TAG, "In interrupt handler");
    toggle_switch = 1;

}


void gpio_setup()
{

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};

    // interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // bit mask of the pins, use GPIO 25
    io_conf.pin_bit_mask = (1ULL << GPIO_BTN);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable high on default
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // takes start time of button1 press
    gpio_isr_handler_add(GPIO_BTN, gpio_btn_handler, (void *)GPIO_BTN);
}



void app_main(void)
{

   gpio_setup();
   

}
