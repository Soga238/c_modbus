/****************************************************************************
 * Copyright (c) [2019] [core.zhang@outlook.com]                            *
 * [Soft Timer] is licensed under Mulan PSL v2.                             *
 * You can use this software according to the terms and conditions of       *
 * the Mulan PSL v2.                                                        *
 * You may obtain a copy of Mulan PSL v2 at:                                *
 *          http://license.coscl.org.cn/MulanPSL2                           *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF     *
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO        *
 * NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.       *
 * See the Mulan PSL v2 for more details.                                   *
 *                                                                          *
 ***************************************************************************/
#ifndef C_MODBUS_UART_UART_H_
#define C_MODBUS_UART_UART_H_
#ifdef __cplusplus
extern "C" {
#endif

/* Includes --------------------------------------------------------*/
/* Global variables ------------------------------------------------*/
/* Global typedef --------------------------------------------------*/
typedef struct uart_option_t uart_option_t;
struct uart_option_t {
    int speed;
    int flow_control;
    int data_bits;
    int stop_bits;
    int parity;
};

/* Global define ---------------------------------------------------*/
/* Global macro ----------------------------------------------------*/
/* Global variables ------------------------------------------------*/
/* Global function prototypes --------------------------------------*/
extern int uart_open(char *port);

extern void uart_close(int fd);

extern int uart_set(int fd, uart_option_t *u);

extern int uart_recv(int fd, char *dst, int length);

extern int uart_send(int fd, const char *src, int length);

#ifdef __cplusplus
}
#endif
#endif

/*************************** End of file ****************************/
