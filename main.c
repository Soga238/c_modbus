#include <stdio.h>
#include "modbus/modbus.h"
#include "pthread.h"
#include "unistd.h"
#include <sys/time.h>
#include <asm/param.h>
// #include ""
#include "./uart/uart.h"

#define DEV_NAME    "/dev/ttyUSB0"
mb_master_t master;

int32_t modbus_send(uint8_t *src, uint16_t hwLength, uint32_t wTimeout);
int32_t modbus_recv(uint8_t *dst, uint16_t hwLength, uint32_t wTimeout);

int g_fd;

uint8_t s_chRecvBuf[256];
uint8_t s_chSendBuf[256];

// pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond;
pthread_mutex_t mutex;

static mb_request_t s_tRequestList[1] = {
    {.chSlave = 1, .chCode = 0x01, .hwDataAddr = 0, .hwDataNum= 16, .wTimeout = 100},
};
uint32_t get_tick_1ms(void);

void *modbus_thread(void *argument)
{
    int32_t nLoop = 1000;
    int32_t nCircle = 0;

    printf("modbus thread run\n");
    struct timeval now;
    struct timespec out;

    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);

    int nTimeout = 100;

    while (nLoop--) {
        mb_master_poll(&master);
        if (++nCircle > 10) {
            nCircle = 0;
            mb_do_request(&master, &s_tRequestList[0]);
        }

        usleep(10000);

        // uart_send(g_fd, "hello world", 11);
   }

    return 0;
}

int main(void)
{
    serial_ctl_t tCtl = {
        .fnRecv = modbus_recv,
        .fnSend = modbus_send,
        .hwRcvBufSize = 256,
        .pRcvBuf = s_chRecvBuf,
        .hwSndBufSize = 256,
        .pSndBuf = s_chSendBuf,
    };

    struct uart_option_t option = {
        .speed = 115200,
        .parity = 'n',
        .stop_bits = 1,
        .flow_control = 0,
        .data_bits = 8
    };

    mb_master_init(&master, &tCtl);

    g_fd = uart_open(DEV_NAME);
    if (g_fd < 0) {
        printf("open %s failed\n", DEV_NAME);
        return 0;
    }

    uart_set(g_fd, &option);
    printf("open %s success\n", DEV_NAME);

    // pthread_t tId1;
    // pthread_create(&tId1, NULL, modbus_thread, NULL);
    // pthread_join(tId1, 0);

    modbus_thread(NULL);

    uart_close(g_fd);

    return 0;
}

int32_t modbus_send(uint8_t *src, uint16_t hwLength, uint32_t wTimeout)
{
    int rc;
    rc = uart_send(g_fd, (char *) src, hwLength);
    return rc > 0 ? rc : 0;
}

int32_t modbus_recv(uint8_t *dst, uint16_t hwLength, uint32_t wTimeout)
{
    int rc;
    rc = uart_recv(g_fd, (char *) dst, hwLength);
    return rc > 0 ? rc : 0;
}

uint32_t get_tick_1ms(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return tv.tv_usec / 1000;
}

void port_hold_register_cb(const mb_master_t *ptMaster,
                           const mb_request_t *ptRequest,
                           const mb_response_t *ptResponse)
{

}

void port_coil_cb(const mb_master_t *ptMaster,
                  const mb_request_t *ptRequest,
                  const mb_response_t *ptResponse)
{

}

void port_error_cb(const mb_master_t *ptMaster,
                   const mb_request_t *ptRequest,
                   const mb_response_t *ptResponse)
{

}
