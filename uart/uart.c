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
#include <stdio.h>
#include "uart.h"
#include "unistd.h"
#include "fcntl.h"
#include "sys/types.h"
#include <termios.h>

#include "./uart.h"

#ifndef countof
#define countof(ARR)    (sizeof(ARR) / sizeof((ARR)[0]))
#endif

int speeds[] = {
    B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300
};

int speed_names[] = {
    115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300
};

int uart_open(char *port)
{
    int fd;

    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        return -1;
    }

    if (fcntl(fd, F_SETFL, 0) < 0) {
        printf("fcntl failed");
        return -1;
    }

    /*! 判断是否为终端设备 */
    if (0 == isatty(STDIN_FILENO)) {
        printf("std input is not a terminal dev");
        return -1;
    }

    return fd;
}

void uart_close(int fd)
{
    close(fd);
}

int uart_set(int fd, uart_option_t *u)
{
    struct termios opts;
    int i;

    /*! tcgetattr得到与fd指向对象的相关参数，并将它们保存于options, 该函数还可以
     * 测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，
     * 若调用失败，函数返回值为1.
     */
    if (tcgetattr(fd, &opts) != 0) {
        printf("tcgetattr failed\n");
        return -1;
    }

    /*! 设置输入输出波特率 */
    for (i = 0; i < countof(speed_names); ++i) {
        if (u->speed == speed_names[i]) {
            cfsetispeed(&opts, speeds[i]);
            cfsetospeed(&opts, speeds[i]);
            break;
        }
    }

    if (i == countof(speed_names)) {
        return -1;
    }

    opts.c_cflag |= CLOCAL; // 修改控制模式，保证程序不会占用串口
    opts.c_cflag |= CREAD; // 修改控制模式，使得能够从串口中读取输入数据

    switch (u->flow_control) {
        default:
        case 0:
            opts.c_cflag &= ~CRTSCTS;   // 不使用流控
            break;
        case 1:
            opts.c_cflag |= CRTSCTS;    // 硬件流控
            break;
        case 2:
            opts.c_cflag |= IXON | IXOFF | IXANY; // 软件流控
            break;
    }

    //设置数据位
    opts.c_cflag &= ~CSIZE; //屏蔽其他标志位
    switch (u->data_bits) {
        case 5 :
            opts.c_cflag |= CS5;
            break;
        case 6    :
            opts.c_cflag |= CS6;
            break;
        case 7    :
            opts.c_cflag |= CS7;
            break;
        case 8:
            opts.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr, "Unsupported data size\n");
            return -1;
    }

    //设置校验位
    switch (u->parity) {
        case 'n':
        case 'N': //无奇偶校验位。
            opts.c_cflag &= ~PARENB;
            opts.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O': //设置为奇校验
            opts.c_cflag |= (PARODD | PARENB);
            opts.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E': //设置为偶校验
            opts.c_cflag |= PARENB;
            opts.c_cflag &= ~PARODD;
            opts.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //设置为空格
            opts.c_cflag &= ~PARENB;
            opts.c_cflag &= ~CSTOPB;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return -1;
    }

    // 设置停止位
    switch (u->stop_bits) {
        case 1:
            opts.c_cflag &= ~CSTOPB;
            break;
        case 2:
            opts.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported stop bits\n");
            return -1;
    }

    //修改输出模式，原始数据输出
    opts.c_oflag &= ~OPOST;
    //设置等待时间和最小接收字符
    opts.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    opts.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取
    tcflush(fd, TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd, TCSANOW, &opts) != 0) {
        perror("com set error!/n");
        return -1;
    }

    return 0;
}

int uart_recv(int fd, char *dst, int length)
{
    int len, fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = 10;
    time.tv_usec = 0;

    //使用select实现串口的多路通信
    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
    if (fs_sel) {
        len = read(fd, dst, length);
        return len;
    }

    return -1;
}

int uart_send(int fd, const char *src, int length)
{
    int rc = -1;

    if (length) {
        rc = write(fd, src, length);
        if (rc != length) {
            tcflush(fd, TCOFLUSH);
            rc = length;
        }
    }

    return rc;
}

/*************************** End of file ****************************/
