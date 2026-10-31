/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/ext_power.h>
#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/rgb_underglow.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define BATTERY_LAYER 1
#define BATTERY_LED_STEP_PERCENT 5
#define BATTERY_DISPLAY_DELAY K_MSEC(100)
#define BATTERY_RESTORE_DELAY K_SECONDS(2)

#define LED_STRIP_NODE DT_CHOSEN(zmk_underglow)
#define LED_STRIP_COUNT DT_PROP(LED_STRIP_NODE, chain_length)

BUILD_ASSERT(LED_STRIP_COUNT > 0, "RGB strip must contain at least one LED");

static const struct device *const led_strip = DEVICE_DT_GET(LED_STRIP_NODE);
static struct led_rgb pixels[LED_STRIP_COUNT];
static bool indicator_showing;
static bool restore_underglow;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
static const struct device *const ext_power = DEVICE_DT_GET(DT_INST(0, zmk_ext_power_generic));
#endif

static void battery_rgb_render(struct k_work *work);
static void battery_rgb_restore(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(battery_rgb_render_work, battery_rgb_render);
K_WORK_DELAYABLE_DEFINE(battery_rgb_restore_work, battery_rgb_restore);

static void battery_rgb_render(struct k_work *work) {
    if (!indicator_showing || !device_is_ready(led_strip)) {
        return;
    }

    uint8_t percentage = MIN(zmk_battery_state_of_charge(), 100);
    size_t lit_count = (percentage + BATTERY_LED_STEP_PERCENT - 1) / BATTERY_LED_STEP_PERCENT;
    lit_count = MIN(lit_count, ARRAY_SIZE(pixels));

    for (size_t i = 0; i < ARRAY_SIZE(pixels); i++) {
        pixels[i] = i < lit_count ? (struct led_rgb){.r = 255, .g = 0, .b = 0}
                                  : (struct led_rgb){.r = 0, .g = 0, .b = 0};
    }

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (device_is_ready(ext_power)) {
        int err = ext_power_enable(ext_power);
        if (err != 0) {
            LOG_ERR("Failed to enable RGB power for battery display (%d)", err);
            return;
        }
    }
#endif

    int err = led_strip_update_rgb(led_strip, pixels, ARRAY_SIZE(pixels));
    if (err != 0) {
        LOG_ERR("Failed to update RGB battery display (%d)", err);
    }
}

static void battery_rgb_restore(struct k_work *work) {
    if (zmk_keymap_layer_active(BATTERY_LAYER)) {
        return;
    }

    indicator_showing = false;
    k_work_cancel_delayable(&battery_rgb_render_work);

    for (size_t i = 0; i < ARRAY_SIZE(pixels); i++) {
        pixels[i] = (struct led_rgb){.r = 0, .g = 0, .b = 0};
    }
    led_strip_update_rgb(led_strip, pixels, ARRAY_SIZE(pixels));

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (device_is_ready(ext_power)) {
        ext_power_disable(ext_power);
    }
#endif

    if (restore_underglow) {
        zmk_rgb_underglow_on();
    }
}

static int battery_rgb_listener(const zmk_event_t *event) {
    if (as_zmk_layer_state_changed(event)) {
        const struct zmk_layer_state_changed *layer_event = as_zmk_layer_state_changed(event);

        if (layer_event->layer != BATTERY_LAYER) {
            return ZMK_EV_EVENT_BUBBLE;
        }

        if (layer_event->state) {
            if (!indicator_showing) {
                zmk_rgb_underglow_get_state(&restore_underglow);
                indicator_showing = true;
                zmk_rgb_underglow_off();
            }

            k_work_cancel_delayable(&battery_rgb_restore_work);
            k_work_reschedule(&battery_rgb_render_work, BATTERY_DISPLAY_DELAY);
        } else if (indicator_showing) {
            k_work_reschedule(&battery_rgb_restore_work, BATTERY_RESTORE_DELAY);
        }

        return ZMK_EV_EVENT_BUBBLE;
    }

    if (as_zmk_battery_state_changed(event) && indicator_showing) {
        k_work_reschedule(&battery_rgb_render_work, BATTERY_DISPLAY_DELAY);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_rgb_indicator, battery_rgb_listener);
ZMK_SUBSCRIPTION(battery_rgb_indicator, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(battery_rgb_indicator, zmk_battery_state_changed);
