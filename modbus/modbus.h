/****************************************************************************
 * Copyright (c) [2019] [core.zhang@outlook.com]                            *
 * [C modbus] is licensed under Mulan PSL v2.                               *
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
#ifndef __MODBUS_H__
#define __MODBUS_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes --------------------------------------------------------*/
#include "./modbus_config.h"
#include "../../service/soft_timer/soft_timer.h"

/* Global variables ------------------------------------------------*/
extern uint32_t get_tick_1ms(void);

/* Global typedef --------------------------------------------------*/
/*! 串口函数指针 */
typedef int32_t (*pSerialFun)(uint8_t *buf, uint16_t size, uint32_t timeout);

typedef struct serial_ctl serial_ctl_t;
struct serial_ctl {
    uint8_t *pSndBuf;            /*! 数据接收缓冲区      */
    uint8_t *pRcvBuf;            /*! 数据发送缓冲区      */

    uint16_t hwSndBufSize;       /*! 数据接收缓区长度    */
    uint16_t hwRcvBufSize;       /*! 数据发送缓区长度    */

    uint16_t hwSndLen;           /*! 发送数据长度        */
    uint16_t hwRcvLen;           /*! 已接收数据长度      */

    pSerialFun fnSend;           /*! 串口发送函数指针    */
    pSerialFun fnRecv;           /*! 串口接收函数指针    */
};

/**
 *  \brief modbus请求结构体，属于主动读取的一方
 */
typedef struct mb_request {
    uint32_t wTimeout;           /*! 串口接收超时时间    */
    timer_table_t *ptTimer;      /*! 软定时器结构体指针  */

    uint16_t *phwHR;             /*! 数据值接收缓区      */
    const void *pWR;             /*! 数据值发送缓区      */

    /*!! 保存写入单个数据时存放的数据 */
    union {
        uint16_t hwValue;
        uint32_t wValue;
    };

    uint8_t  chSlave;            /*! 从机站台号          */
    uint8_t chCode;              /*! 功能码              */
    uint16_t hwDataAddr;         /*! 数据地址            */
    uint16_t hwDataNum;          /*! 数据读取或写入个数   */
} mb_request_t;

/**
 *  \brief modbus应答结构体，属于被动读取的一方
 */
typedef struct mb_response {
    uint8_t chSlave;             /*! 从机站台号          */
    uint8_t chCode;              /*! 功能码              */

    uint16_t hwByteCount;        /*! 字节个数            */
    uint16_t hwDataAddr;         /*! 数据地址            */
    uint16_t hwDataNum;          /*! 数据读取或写入个数   */

    uint8_t *pchRegStart;        /*! 应答数据缓冲区头指针 */
} mb_response_t;

typedef enum {
    MB_STATUE_IDLE = 0,
    MB_STATUS_BUSY = 1
} mb_eu_state_t;

typedef struct {
    mb_request_t tRequest;       /*! modbus请求结构体    */
    mb_response_t tResponse;     /*! modbus应答结构体    */
    serial_ctl_t tSIO;           /*! 串口操作结构体      */

    uint8_t chState;             /*! 内部状态机变量      */
    mb_eu_state_t tStatus;

    uint8_t bInitOk : 1;
    uint32_t wEvent;             /*! 内部事件变量        */
    uint8_t chContinueCount;
    uint8_t chExceptionCode;
    uint8_t chErr;
    /*! mb_err_code__t tErrorNum; */
} mb_control_t, mb_master_t, mb_slave_t;

/* Global define ---------------------------------------------------*/
#define MB_SLAVE_ADDRESS_MIN       0x01        /*! 从机地址最小值 */
#define MB_SLAVE_ADDRESS_MAX       0xF7        /*! 从机地址最大值 */

#define MB_READ_REG_CNT_MIN        0x0001      /*! 读取保持寄存器的最小长度 */
#define MB_READ_REG_CNT_MAX        0x007D      /*! 读取保持寄存器的最大长度 */

#define MB_READ_COIL_CNT_MIN       0x0001      /*! 读取线圈的最小长度 */
#define MB_READ_COIL_CNT_MAX       0x07D0      /*! 读取线圈的最大长度 */

/*
*********************************************************************************
*                               MODBUS 功能码
*********************************************************************************
*/
#define MB_CODE_NONE                            0     /*! 未定义功能码 */
#define MB_CODE_READ_COILS                      1     /*! 读取多个线圈 */
#define MB_CODE_READ_DISCRETE_INPUTS            2     /*! 读取多个输入线圈 */
#define MB_CODE_READ_HOLDING_REGISTERS          3     /*! 读单/多个保持寄存器 */
#define MB_CODE_READ_INPUT_REGISTERS            4     /*! 读单/多个输入寄存器 */
#define MB_CODE_WRITE_COIL                      5     /*! 强制单个线圈 */
#define MB_CODE_WRITE_REGISTER                  6     /*! 写单个保持寄存器 */
#define MB_CODE_WRITE_MULTIPLE_COILS            15    /*! 写多个线圈寄存器 */
#define MB_CODE_WRITE_MULTIPLE_REGISTERS        16    /*! 写多个保持寄存器 */
#define MB_CODE_WRITE_MULTIPLE_REGISTERS_RDHR   23    /*! 读/写多个保持寄存器 */
/*
*****************************************************************************
*                               MODBUS 标准异常
*****************************************************************************
*/
#define MB_EX_NONE                              0x00  /*! 无异常 */
#define MB_EX_ILLEGAL_FUNCTION                  0x01  /*! 非法功能 */
#define MB_EX_ILLEGAL_DATA_ADDRESS              0x02  /*! 非法数据地址 */
#define MB_EX_ILLEGAL_DATA_VALUE                0x03  /*! 非法数据值 */
#define MB_EX_SLAVE_DEVICE_FAILURE              0x04  /*! 从机设备故障 */
#define MB_EX_ACKNOWLEDGE                       0x05  /*! 确认 */
#define MB_EX_SLAVE_BUSY                        0x06  /*! 从机繁忙 */
#define MB_EX_MEMORY_PARITY_ERROR               0x08  /*! 内存奇偶校验错无 */
#define MB_EX_GATEWAY_PATH_FAILED               0x0A  /*! 不可用网关路径 */
#define MB_EX_GATEWAY_TGT_FAILED                0x0B  /*! 网关目标设备响应失败 */

#define MB_ERR_SEND_FAILED                      1
#define MB_ERR_RECV_TIMEOUT                     2
#define MB_ERR_CREATE_TIMER                     3
#define MB_ERR_REQUEST                          4
#define MB_ERR_RESPONSE                         5
#define MB_ERR_NONSUPPORT                       6

/* Global macro ----------------------------------------------------*/
#define MB_GET_ERROR(MASTER_SLAVE_PTR)          ((MASTER_SLAVE_PTR)->chErr)

/* Global variables ------------------------------------------------*/
/* Global function prototypes --------------------------------------*/
#if defined(C_MODBUS_MASTER_ENABLE) | defined(C_MODBUS_SLAVE_ENABLE)
extern bool mb_control_init(mb_control_t *ptCtl,
                            serial_ctl_t *ptSerialConfig);
extern bool mb_control_is_idle(mb_control_t *ptCtl);
#endif

#if defined(C_MODBUS_MASTER_ENABLE)
#define mb_master_init      mb_control_init
#define mb_master_is_idle   mb_control_is_idle

extern void mb_master_poll(mb_master_t *ptMaster);

extern int32_t mb_do_request(mb_master_t *ptMaster,
                             const mb_request_t *ptRequest);

extern int32_t mb_read_hold_register(mb_master_t *ptMaster,
                                     uint8_t chSlaveNumber,
                                     uint16_t hwDataAddr,
                                     uint16_t hwDataNumber,
                                     uint32_t wTimeout);

extern int32_t mb_write_hold_register(mb_master_t *ptMaster,
                                      uint8_t chSlaveNumber,
                                      uint16_t hwDataAddr,
                                      uint16_t hwValue,
                                      uint32_t wTimeout);

extern int32_t mb_write_hold_multi_register(mb_master_t *ptMaster,
                                            uint8_t chSlaveNumber,
                                            uint16_t hwDataAddr,
                                            const uint16_t *phwBuf,
                                            uint16_t hwDataNumber,
                                            uint32_t wTimeout);

extern int32_t mb_write_single_coil(mb_master_t *ptMaster,
                                    uint8_t chSlaveNumber,
                                    uint16_t hwCoilAddr,
                                    uint8_t chSwitch,
                                    uint32_t wTimeout);

extern int32_t mb_read_input_register(mb_master_t *ptMaster,
                                      uint8_t chSlaveNumber,
                                      uint16_t hwDataAddr,
                                      uint16_t hwDataNumber,
                                      uint32_t wTimeout);

extern int32_t mb_read_discrete_inputs(mb_master_t *ptMaster,
                                       uint8_t chSlaveNumber,
                                       uint16_t hwDataAddr,
                                       uint16_t hwDataNumber,
                                       uint32_t wTimeout);

extern int32_t mb_read_coils(mb_master_t *ptMaster,
                             uint8_t chSlaveNumber,
                             uint16_t hwDataAddr,
                             uint16_t hwDataNumber,
                             uint32_t wTimeout);

#endif

#if defined(C_MODBUS_SLAVE_ENABLE)

#define mb_slave_init       mb_control_init
#define mb_slave_is_idle    mb_control_is_idle

extern void mb_slave_push(mb_slave_t *ptSlave);

extern int32_t mb_do_request(mb_master_t *ptMaster,
                             const mb_request_t *ptRequest);

#endif

#ifdef __cplusplus
}
#endif
#endif

/*************************** End of file ****************************/
