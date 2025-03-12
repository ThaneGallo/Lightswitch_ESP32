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

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_event.h"
#include "esp_nimble_hci.h"
#include "sdkconfig.h"

#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

const ble_addr_t server_addr = {
    .type = BLE_ADDR_RANDOM,
    .val = {0xDE, 0xCA, 0xFB, 0xEE, 0xFE, 0xD2}};

const ble_addr_t client_addr = {
    .type = BLE_ADDR_RANDOM,
    .val = {0xCA, 0xFF, 0xED, 0xBE, 0xEE, 0xEF}};

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

void poll_event_task(void *param)
{
    // just ticks tbh
    int cnt = 0;
    while (1)
    {
        // printf("cnt: %d\n", cnt++);
        ESP_LOGI(TAG, "cnt: %d", cnt++);

        if (toggle_switch)
        {
            ESP_LOGI(TAG, "Toggle switch");
            toggle_switch = 0;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static struct ble_gap_disc_params disc_params = {
    .filter_duplicates = 1,
    .passive = 0,
    .itvl = 0,
    .window = 0,
    .filter_policy = BLE_HCI_SCAN_FILT_USE_WL,
    //.filter_policy = BLE_HCI_SCAN_FILT_NO_WL, // find all things no whitelist used.
    .limited = 0};

/**
 * Callback function for gap events.
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{

    uint8_t err;
    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        // Handle device discovery
        ESP_LOGI(TAG, "Device found: %x%x%x%x%x%x", event->disc.addr.val[0], event->disc.addr.val[1],
                 event->disc.addr.val[2], event->disc.addr.val[3], event->disc.addr.val[4], event->disc.addr.val[5]);
        // Connect to the device if it matches your criteria
        // Replace `event->disc.addr` with the address of the device you want to connect to
        ble_gap_disc_cancel(); // cancel discovery to allow for connection

        // err = ble_gap_connect(BLE_OWN_ADDR_RANDOM, server_ptr, 10000, NULL, NULL, NULL);
        err = ble_gap_connect(BLE_OWN_ADDR_RANDOM, &event->disc.addr, 10000, NULL, ble_gap_event, NULL); // works just fine.
        // err = ble_gap_connect(BLE_OWN_ADDR_RANDOM, NULL, 10000, NULL, NULL, NULL);

        // ble_gap_disc_event_helper(err);
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Discover event complete");
        break;
    case BLE_GAP_EVENT_CONNECT:
        // err = ble_gap_connect_event_helper(event, arg, profile_ptr);
        // if(err != 0) {
        //     return err;
        // }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "ble_gap_event_disconnect successful");
        break;
    default:
        ESP_LOGI(TAG, "Called Event without handler: %u", event->type);
        break;
    }

    return 0;
}

// ble_task which runs infinitely and checks for ble_events.
void ble_task(void *param)
{
    nimble_port_run();
    return;
}

void ble_app_on_sync(void)
{
    // *********create the profile structure, allocate memory, and pass it the characteristic data*********

    uint8_t err;
    err = ble_hs_id_set_rnd(client_addr.val);
    if (err != 0)
    {
        ESP_LOGI(TAG, "BLE gap set random address failed %d", err);
    }

    err = ble_gap_wl_set(&server_addr, 1); // sets white list for connection to 1 other device (server)
    if (err != 0)
    {
        ESP_LOGI(TAG, "BLE gap set whitelist failed");
    }

    // begin gap discovery
    err = ble_gap_disc(BLE_OWN_ADDR_RANDOM, 10 * 1000, &disc_params, ble_gap_event, NULL);
    if (err != 0)
    {
        ESP_LOGI(TAG, "BLE GAP Discovery Failed: %u", err);
    }
}

void app_main(void)
{

    gpio_setup();

    uint8_t err;

    // init
    nvs_flash_init(); // sets up flash memory
    nimble_port_init();

    err = ble_svc_gap_device_name_set("BLE-Scan-Client"); // 4 - Set device name characteristic
    if (err != 0)
    {
        ESP_LOGI(TAG, "GAP device name set");
    }

    ble_svc_gap_init(); // initialize gap
    ble_gattc_init();   // initialize gatt

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    xTaskCreate(poll_event_task, "Poll Event Task", 2048, NULL, 5, NULL);

    // starts first task
    nimble_port_freertos_init(ble_task);
}
