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

#include <Arduino.h>
#include "protocol.h"

static recv_callback_t msg_recv_callback = NULL;

void callback_register(recv_callback_t cb)
{
    msg_recv_callback = cb;
}

uint16_t ctrl_package_pack(uint8_t *buff, uint8_t num, uint8_t cmd, uint8_t *pdata, uint16_t len, uint8_t mode)
{
	server_head_t head = { 0 };
	server_tail_t tail = { 0 };
	head.sof = 0xAA;
	head.cmd = cmd;
	head.id = num;
    head.udp_mode = mode;
	head.length = len + SOF_FRAME_LENGTH + sizeof(server_tail_t);

	memcpy(buff, &head, SOF_FRAME_LENGTH);
	memcpy(buff + SOF_FRAME_LENGTH, pdata, len);

	for (int i = 0; i < (int)(len + SOF_FRAME_LENGTH); i++)
	{
		tail.sum += buff[i];
	}

	memcpy(buff + SOF_FRAME_LENGTH + len, &tail, sizeof(server_tail_t));

	return head.length;
}

static uint8_t msg_buff[1024] = {0};
uint16_t ctrl_msg_send(uint8_t cmd, uint8_t id, uint8_t *pdata, uint16_t len)
{
    uint16_t send_len;
	send_len = ctrl_package_pack(msg_buff, id, cmd, pdata, len, 1);

    Serial1.write(msg_buff, send_len);

	return send_len;
}

static uint8_t uart_rx_buff[256];
static uint8_t uart_index = 0;
static uint8_t uart_unpack_step = 0;
uint16_t ctrl_package_unpack(uint8_t *pdata, uint16_t len)
{
	uint16_t search_index = 0;
	uint16_t unpack_len = len;
	server_head_t *phead = (server_head_t *)uart_rx_buff;

	while (unpack_len--)
	{
		switch (uart_unpack_step)
		{
			/** find sof */
		case 0:
		{
			if (pdata[search_index] == 0xAA)
			{
				uart_index = 0;
				uart_rx_buff[uart_index++] = pdata[search_index];
				uart_unpack_step = 1;
			}
			break;
		}
		case 1:
		{
			uart_rx_buff[uart_index++] = pdata[search_index];

			if (uart_index == SOF_FRAME_LENGTH)
			{
				uart_unpack_step = 2;
			}
			break;
		}
		/** analyze protocol */
		case 2:
		{
			if (phead->length > sizeof(uart_rx_buff))
			{
				uart_unpack_step = 0;
			}

			if (uart_index < phead->length)
			{
				uart_rx_buff[uart_index++] = pdata[search_index];
			}
			/** FINISH */
			if (uart_index == phead->length)
			{
				uint16_t sum = 0;
				for (int i = 0; i < (int)(uart_index - sizeof(server_tail_t)); i++)
				{
					sum += uart_rx_buff[i];
				}
				server_tail_t *ptail = (server_tail_t *)&uart_rx_buff[uart_index - sizeof(server_tail_t)];
				if (sum == ptail->sum)
				{
                    if(msg_recv_callback)
                    {
                        msg_recv_callback(phead);
                    }
				}

				uart_unpack_step = 0;
			}
			break;
		}

		default:
			break;
		}
		search_index++;
	}
	return 0;
}


