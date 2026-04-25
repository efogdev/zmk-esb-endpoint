/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_esb_shell_req

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <drivers/behavior.h>
#include "../shell/shell_relay.h"

LOG_MODULE_DECLARE(zmk_esb_shell, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    esb_shell_relay_request();
    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_esb_shell_req_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static const struct behavior_driver_api behavior_esb_shell_req_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
};

#define ESB_SHELL_REQ_INST(n)                                                                      \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_esb_shell_req_init, NULL, NULL, NULL,                      \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                      \
                            &behavior_esb_shell_req_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ESB_SHELL_REQ_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
