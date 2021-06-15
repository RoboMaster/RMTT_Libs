/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 * 2021-06-15     robomaster   support TT swarm combo
 */

#pragma once

#include <Arduino.h>
#include "stdint.h"

typedef enum {
    NO_CLICK_EVENT,
    SINGAL_CLICK_EVENT,
    DOUBLE_CLICK_EVENT,
    MULTI_CLICK_EVENT
} sm_event;

typedef enum {
    CHOOSE_FILE_STATE,
    ENTER_MODE_STATE,
    SEARCH_DEONE_STATE,
    COUNTDOWN_STATE,
    EXECUTE_STATE,
    MAX_STATE_NUM
} sm_state;

typedef struct {
    sm_state (*do_action)();
} sm_action;

void clean_event();
sm_event get_event();
void set_event(sm_event event);


