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
#include <dt-bindings/zmk/hid_usage.h>
#include <zmk/battery.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/hid_indicators.h>
#include <zmk/keymap.h>
#include <zmk/rgb_underglow.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define FN_LAYER 1
#define BATTERY_LAYER 2
#define BATTERY_LED_STEP_PERCENT 5
#define RGB_REFRESH_DELAY K_MSEC(50)
#define BATTERY_DISPLAY_HOLD K_SECONDS(2)
#define CAPS_LOCK_INDICATOR BIT(HID_USAGE_LED_CAPS_LOCK - HID_USAGE_LED_NUM_LOCK)

#define LED_STRIP_NODE DT_CHOSEN(zmk_underglow)
#define LED_STRIP_COUNT DT_PROP(LED_STRIP_NODE, chain_length)

BUILD_ASSERT(LED_STRIP_COUNT > 0, "RGB strip must contain at least one LED");

static const struct device *const led_strip = DEVICE_DT_GET(LED_STRIP_NODE);
static struct led_rgb pixels[LED_STRIP_COUNT];
static bool fn_layer_active;
static bool fn_display_latched;
static bool battery_layer_active;
static bool battery_display_latched;
static bool rgb_override_active;
static bool restore_underglow;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
static const struct device *const ext_power = DEVICE_DT_GET(DT_INST(0, zmk_ext_power_generic));
#endif

static void rgb_status_refresh(struct k_work *work);
static void fn_display_clear(struct k_work *work);
static void battery_display_clear(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(rgb_status_refresh_work, rgb_status_refresh);
K_WORK_DELAYABLE_DEFINE(fn_display_clear_work, fn_display_clear);
K_WORK_DELAYABLE_DEFINE(battery_display_clear_work, battery_display_clear);

static void schedule_rgb_refresh(void) {
    k_work_reschedule(&rgb_status_refresh_work, RGB_REFRESH_DELAY);
}

static void fill_strip(struct led_rgb color) {
    for (size_t i = 0; i < ARRAY_SIZE(pixels); i++) {
        pixels[i] = color;
    }
}

static void render_battery(void) {
    uint8_t percentage = MIN(zmk_battery_state_of_charge(), 100);
    size_t lit_count = (percentage + BATTERY_LED_STEP_PERCENT - 1) / BATTERY_LED_STEP_PERCENT;
    lit_count = MIN(lit_count, ARRAY_SIZE(pixels));

    for (size_t i = 0; i < ARRAY_SIZE(pixels); i++) {
        pixels[i] = i < lit_count ? (struct led_rgb){.r = 255, .g = 0, .b = 0}
                                  : (struct led_rgb){.r = 0, .g = 0, .b = 0};
    }
}

static void render_connection(void) {
    const struct zmk_endpoint_instance endpoint = zmk_endpoints_selected();
    struct led_rgb color = {.r = 0, .g = 255, .b = 0};

    if (endpoint.transport == ZMK_TRANSPORT_BLE) {
        switch (endpoint.ble.profile_index) {
        case 0:
            color = (struct led_rgb){.r = 0, .g = 160, .b = 255};
            break;
        case 1:
            color = (struct led_rgb){.r = 0, .g = 0, .b = 255};
            break;
        default:
            color = (struct led_rgb){.r = 0, .g = 0, .b = 96};
            break;
        }
    }

    fill_strip(color);
}

static bool caps_lock_active(void) {
    return (zmk_hid_indicators_get_current_profile() & CAPS_LOCK_INDICATOR) != 0;
}

static bool indicator_active(void) {
    return caps_lock_active() || battery_layer_active || battery_display_latched || fn_layer_active ||
           fn_display_latched;
}

static void rgb_status_refresh(struct k_work *work) {
    if (!device_is_ready(led_strip)) {
        return;
    }

    if (!indicator_active()) {
        if (!rgb_override_active) {
            return;
        }

        fill_strip((struct led_rgb){.r = 0, .g = 0, .b = 0});
        led_strip_update_rgb(led_strip, pixels, ARRAY_SIZE(pixels));

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
        if (device_is_ready(ext_power)) {
            ext_power_disable(ext_power);
        }
#endif

        rgb_override_active = false;
        if (restore_underglow) {
            zmk_rgb_underglow_on();
        }
        return;
    }

    if (!rgb_override_active) {
        zmk_rgb_underglow_get_state(&restore_underglow);
        zmk_rgb_underglow_off();
        rgb_override_active = true;
    }

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (device_is_ready(ext_power)) {
        int err = ext_power_enable(ext_power);
        if (err != 0) {
            LOG_ERR("Failed to enable RGB status indicator power (%d)", err);
            return;
        }
    }
#endif

    if (caps_lock_active()) {
        fill_strip((struct led_rgb){.r = 255, .g = 255, .b = 255});
    } else if (battery_layer_active || battery_display_latched) {
        render_battery();
    } else {
        render_connection();
    }

    int err = led_strip_update_rgb(led_strip, pixels, ARRAY_SIZE(pixels));
    if (err != 0) {
        LOG_ERR("Failed to update RGB status indicator (%d)", err);
    }
}

static void battery_display_clear(struct k_work *work) {
    battery_display_latched = false;
    schedule_rgb_refresh();
}

static void fn_display_clear(struct k_work *work) {
    fn_display_latched = false;
    schedule_rgb_refresh();
}

static int rgb_status_listener(const zmk_event_t *event) {
    if (as_zmk_layer_state_changed(event)) {
        const struct zmk_layer_state_changed *layer_event = as_zmk_layer_state_changed(event);

        if (layer_event->layer == FN_LAYER) {
            fn_layer_active = layer_event->state;
            if (fn_layer_active) {
                fn_display_latched = false;
                k_work_cancel_delayable(&fn_display_clear_work);
            } else {
                fn_display_latched = true;
                k_work_reschedule(&fn_display_clear_work, BATTERY_DISPLAY_HOLD);
            }
            schedule_rgb_refresh();
        } else if (layer_event->layer == BATTERY_LAYER) {
            battery_layer_active = layer_event->state;
            if (battery_layer_active) {
                battery_display_latched = true;
                k_work_cancel_delayable(&battery_display_clear_work);
            } else {
                k_work_reschedule(&battery_display_clear_work, BATTERY_DISPLAY_HOLD);
            }
            schedule_rgb_refresh();
        }

        return ZMK_EV_EVENT_BUBBLE;
    }

    if (as_zmk_hid_indicators_changed(event) || as_zmk_endpoint_changed(event)) {
        schedule_rgb_refresh();
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (as_zmk_battery_state_changed(event) &&
        (battery_layer_active || battery_display_latched)) {
        schedule_rgb_refresh();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(rgb_status_indicator, rgb_status_listener);
ZMK_SUBSCRIPTION(rgb_status_indicator, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(rgb_status_indicator, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(rgb_status_indicator, zmk_hid_indicators_changed);
ZMK_SUBSCRIPTION(rgb_status_indicator, zmk_endpoint_changed);
