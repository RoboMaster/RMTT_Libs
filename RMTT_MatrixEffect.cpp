/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 */

#include "RMTT_Libs.h"

uint8_t matrix_effect_mode = MATRIX_EFFECT_FACTORY_MODE;
uint8_t matrix_effect_next_mode = MATRIX_EFFECT_FACTORY_MODE;

/* string move */
static char string_move_mode = 'l';
static char matrix_color_mode = 'r';
static int  move_time_interval = MOVE_MIN_TIME;
static int  matrix_str_len = 0;
static uint8_t matrix_str_buff[256] = {0};

static uint8_t next_cache = NONE_CACHE_INDEX;
static uint8_t first_cache[128] = {0};
static uint8_t second_cache[128] = {0};
static uint8_t matrix_buff[128] = {0};

/* letter point */
int next_str_index = 1;
/* font buff cs/sw point */
int str_c_index = 0;

uint8_t matrix_bright = 0;
uint8_t matrix_next_bright = 0;

/* graph move */
static uint8_t graph_index = 0;
static uint8_t graph_move_buff[128] = {0};

TaskHandle_t matrixEffectTaskHandle = NULL;

void matrix_effect_init(uint8_t bright)
{
    xTaskCreateUniversal(matrix_effect_task, "matrix_effect_task", 8192, &bright, 4, &matrixEffectTaskHandle, 1);
}

void matrix_effect_deinit(void)
{
    matrix_effect_mode = MATRIX_EFFECT_FACTORY_MODE;
    matrix_effect_next_mode = MATRIX_EFFECT_FACTORY_MODE;
}

void matrix_effect_move_str(char *str, int len, char color, char move_mode, float freq)
{
    string_move_mode = move_mode;
    matrix_color_mode = color;

    if (freq != 0)
    {
        move_time_interval = max(1/freq * 1000 / 8, (float)MOVE_MIN_TIME);
    }

    matrix_str_len = min(len, (int)sizeof(matrix_str_buff));

    memcpy(matrix_str_buff, str, matrix_str_len);

    matrix_effect_next_mode = MATRIX_EFFECT_STRING_MOVE_MODE;
    next_cache = NONE_CACHE_INDEX;
}

void matrix_effect_move_graph(uint8_t *buff, char move_mode, float freq)
{
    memcpy(graph_move_buff, buff, 128);
    string_move_mode = move_mode;

    if (freq != 0)
    {
        move_time_interval = max(1/freq * 1000 / 8, (float)MOVE_MIN_TIME);
    }

    matrix_effect_next_mode = MATRIX_EFFECT_GRAPH_MOVE_MODE;
    graph_index = 0;
}

void matrix_effect_set_graph(uint8_t *graph)
{
    matrix_effect_next_mode = MATRIX_EFFECT_STATIC_MODE;
    memcpy(matrix_buff, graph, 128);
}

uint8_t get_matrix_effect_mode(void)
{
    return matrix_effect_mode;
}

void matrix_effect_set_bright(uint8_t bright)
{
    matrix_next_bright = bright;
}

void matrix_effect_task(void *param)
{
    matrix_next_bright = *(uint8_t *)param;
    while (1)
    {
        matrix_bright = matrix_next_bright;
        matrix_effect_mode = matrix_effect_next_mode;

        switch (matrix_effect_mode)
        {
        case MATRIX_EFFECT_FACTORY_MODE:
            /* do nothing */
            delay(50);
            break;
        case MATRIX_EFFECT_STATIC_MODE:
            RMTT_Matrix::SetAllPWM(matrix_buff);
            delay(50);
            break;
        case MATRIX_EFFECT_GRAPH_MOVE_MODE:
        {
            if (graph_index == 8)
            {
                graph_index = 0;
            }

            graph_move_effect2buff(graph_move_buff, matrix_buff, string_move_mode, graph_index);

            if (matrix_effect_mode == MATRIX_EFFECT_GRAPH_MOVE_MODE)
            {
                RMTT_Matrix::SetAllPWM(matrix_buff);
            }
            graph_index++;
            delay(move_time_interval);
            break;
        }
        case MATRIX_EFFECT_STRING_MOVE_MODE:
        {
            if (next_cache == NONE_CACHE_INDEX)
            {
                /* first init */
                next_cache = FIRST_CACHE_INDEX;

                if ((string_move_mode == 'r') || (string_move_mode == 'd'))
                {
                    for (int i  = 0; i < matrix_str_len / 2; i++)
                    {
                        char chr = matrix_str_buff[i];
                        matrix_str_buff[i] = matrix_str_buff[matrix_str_len - i - 1];
                        matrix_str_buff[matrix_str_len - i - 1] = chr;
                    }
                }

                if (matrix_str_len == 1)
                {
                    mled_font2buff(first_cache, matrix_str_buff[0], matrix_color_mode, matrix_bright);
                    mled_font2buff(second_cache, matrix_str_buff[0], matrix_color_mode, matrix_bright);
                }
                else
                {
                    next_str_index = 1;
                    str_c_index = 0;
                    mled_font2buff(first_cache, matrix_str_buff[0], matrix_color_mode, matrix_bright);
                    mled_font2buff(second_cache, matrix_str_buff[1], matrix_color_mode, matrix_bright);
                }
            }

            if (str_c_index == 8)
            {
                str_c_index = 0;

                if (matrix_str_len != 1)
                {
                    next_str_index++;
                    if (next_str_index == matrix_str_len)
                    {
                        next_str_index = 0;
                    }
                    if (next_cache == FIRST_CACHE_INDEX)
                    {
                        mled_font2buff(first_cache, matrix_str_buff[next_str_index], matrix_color_mode, matrix_bright);
                        next_cache = SECOND_CACHE_INDEX;
                    }
                    else if (next_cache == SECOND_CACHE_INDEX)
                    {
                        mled_font2buff(second_cache, matrix_str_buff[next_str_index], matrix_color_mode, matrix_bright);
                        next_cache = FIRST_CACHE_INDEX;
                    }
                }
            }

            if (next_cache == FIRST_CACHE_INDEX)
            {
                string_move_effect2buff(first_cache, second_cache, matrix_buff, string_move_mode, str_c_index);
            }
            else if (next_cache == SECOND_CACHE_INDEX)
            {
                string_move_effect2buff(second_cache, first_cache, matrix_buff, string_move_mode, str_c_index);
            }

            if (matrix_effect_mode == MATRIX_EFFECT_STRING_MOVE_MODE)
            {
                RMTT_Matrix::SetAllPWM(matrix_buff);
            }
            str_c_index++;
            delay(move_time_interval);
        }
        break;
        default:
        {
            /* error mode */
            delay(100);
        }
        break;
        }
    }
}



