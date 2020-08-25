/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 */

#pragma once

#define MOVE_MIN_TIME   50
#define BREATH_MIN_TIME 10
#define BLINK_MIN_TIME  50

#define MATRIX_EFFECT_STATIC_MODE         1
#define MATRIX_EFFECT_STRING_MOVE_MODE    2
#define MATRIX_EFFECT_GRAPH_MOVE_MODE     3
#define MATRIX_EFFECT_FACTORY_MODE        4

#define LED_EFFECT_BREATH_MODE            1
#define LED_EFFECT_BLINK_MODE             2
#define LED_EFFECT_FACTORY_MODE           3
#define LED_EFFECT_CONST_MODE             4

#define NONE_CACHE_INDEX   0
#define FIRST_CACHE_INDEX  1
#define SECOND_CACHE_INDEX 2

void matrix_effect_init(uint8_t bright);
void matrix_effect_deinit(void);
void matrix_effect_set_graph(uint8_t *graph);
uint8_t get_matrix_effect_mode(void);
void matrix_effect_set_bright(uint8_t bright);
void matrix_effect_task(void *param);
void matrix_effect_move_str(char *str, int len, char color, char move_mode, float freq);
void matrix_effect_move_graph(uint8_t *buff, char move_mode, float freq);

void led_effect_init(void);
void led_effect_deinit(void);
uint8_t get_led_effect_mode(void);
void led_effect_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void led_effect_task(void *param);
void led_effect_blink(uint8_t r1, uint8_t g1, uint8_t b1,
                      uint8_t r2, uint8_t g2, uint8_t b2, float freq);
void led_effect_breath(uint8_t r, uint8_t g, uint8_t b, float freq);


