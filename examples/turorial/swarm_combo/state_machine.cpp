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

#include "state_machine.h"

sm_event g_event = NO_CLICK_EVENT;

void clean_event()
{
    g_event = NO_CLICK_EVENT;
}

void set_event(sm_event event)
{
    g_event = event;
}

sm_event get_event()
{
    sm_event e = g_event;
    g_event = NO_CLICK_EVENT;
    return e;
}


