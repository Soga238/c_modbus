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

/* Includes --------------------------------------------------------*/
#include "./modbus.h"

/* Global variables ------------------------------------------------*/
/* Private typedef -------------------------------------------------*/
/* Private define --------------------------------------------------*/
#define MB_SER_ADU_SIZE_MIN             (5)        /*! RTU串行帧的最小长度 */
#define MB_SER_ADU_SIZE_MAX             (256)      /*! RTU串行帧的最大长度 */

// (PDU_SIZE + 3) = ADU_SIZE
// #define MB_SER_PDU_SIZE_MIN             (3)        /*! RTU串行帧的最小长度 */
// #define MB_SER_PDU_SIZE_MAX             (253)      /*! RTU串行帧的最大长度 */

#define EV_MASTER_NONE                  (0)
#define EV_MASTER_RECV_TIMEOUT          (1 << 0)
#define EV_MASTER_ERROR                 (1 << 2)
#define EV_MASTER_START_SEND            (1 << 3)

#define MAXIMUM_READ_CONTINUE_COUNT     2
/* Private macro ---------------------------------------------------*/
#define CALC_MODBUS_CRC16(PTR, SIZE)     MODBUS_CRC16(PTR, SIZE)
#define VALID_MODBUS_FMT(PTR, SIZE)      valid_modbus_crc(PTR, SIZE)

#define CALC_READ_COILS_REQUIRED_BYTES(__COIL_NUMBER)           \
    ((((__COIL_NUMBER) & 0x0007) ? 1 : 0) + ((__COIL_NUMBER) >> 3))

#ifndef FALL_THROUGH
#define FALL_THROUGH()

#if defined(__GNUC__)
    #if ((__GNUC__ > 7) || ((__GNUC__ == 7) && (__GNUC_MINOR__ >= 1)))
        #undef  FALL_THROUGH
        #define FALL_THROUGH() __attribute__ ((fallthrough))
    #endif
#endif
#endif

#define CHECK_PARAM_NULL_PTR_RETURN(__PTR) \
            do {if (!(__PTR)) return -1;} while(0)

#define CHECK_MASTER_BUSY_RETURN(__PTR) \
            do { if (MB_STATUE_IDLE != (__PTR)->tStatus) return -1;} while(0)

/* Private variables -----------------------------------------------*/
/* Private function prototypes -------------------------------------*/
/* Private functions -----------------------------------------------*/

bool mb_control_init(mb_control_t *ptCtl, serial_ctl_t *ptSerialConfig)
{
    enum { START = 0 };

    if ((NULL == ptCtl) || (NULL == ptSerialConfig)) {
        return false;
    }

    if ((0 == ptSerialConfig->hwRcvBufSize) ||
        (0 == ptSerialConfig->hwSndBufSize)) {
        return false;
    }

    if ((NULL == ptSerialConfig->fnRecv) ||
        (NULL == ptSerialConfig->fnSend)) {
        return false;
    }

    if ((NULL == ptSerialConfig->pRcvBuf) ||
        (NULL == ptSerialConfig->pSndBuf)) {
        return false;
    }

    ptCtl->chState = START;
    ptCtl->wEvent = EV_MASTER_NONE;
    ptCtl->tSIO = *ptSerialConfig;
    ptCtl->tSIO.hwRcvLen = 0;
    ptCtl->tSIO.hwSndLen = 0;
    ptCtl->bInitOk = 1;

#ifdef C_MODBUS_NOBLOCK
    soft_timer_init(get_tick_1ms, MAX_VALUE_32_BIT);
#endif

    return true;
}

bool mb_control_is_idle(mb_control_t *ptCtl)
{
    if (NULL != ptCtl) {
        return ptCtl->tStatus == MB_STATUE_IDLE;
    }
    return false;
}

#if defined(C_MODBUS_MASTER_ENABLE)

extern void port_hold_register_cb(const mb_master_t *ptMaster,
                                  const mb_request_t *ptRequest,
                                  const mb_response_t *ptResponse);

extern void port_coil_cb(const mb_master_t *ptMaster,
                         const mb_request_t *ptRequest,
                         const mb_response_t *ptResponse);

extern void port_error_cb(const mb_master_t *ptMaster,
                          const mb_request_t *ptRequest,
                          const mb_response_t *ptResponse);

static void master_req_resp_cb(const mb_master_t *ptMaster,
                               const mb_request_t *ptRequest,
                               const mb_response_t *ptResponse)
{
    switch (ptRequest->chCode) {
        case MB_CODE_WRITE_COIL:
                FALL_THROUGH();
        case MB_CODE_READ_DISCRETE_INPUTS:
                FALL_THROUGH();
        case MB_CODE_READ_COILS:
            port_coil_cb(ptMaster, ptRequest, ptResponse);
            break;

        case MB_CODE_WRITE_REGISTER:
                FALL_THROUGH();
        case MB_CODE_WRITE_MULTIPLE_REGISTERS:
                FALL_THROUGH();
        case MB_CODE_READ_INPUT_REGISTERS:
                FALL_THROUGH();
        case MB_CODE_READ_HOLDING_REGISTERS:
            port_hold_register_cb(ptMaster, ptRequest, ptResponse);
            break;

        default:
            break;
    }
}

static bool mb_post_event(mb_control_t *ptCtl, uint8_t tEvent)
{
    ptCtl->wEvent |= tEvent;
    return true;
}

static bool mb_wait_event(mb_control_t *ptCtl, uint8_t tEvent)
{
    if (ptCtl->wEvent & tEvent) {
        ptCtl->wEvent &= ~tEvent;
        return true;
    }
    return false;
}

static uint16_t master_send_request(uint8_t *pchBuf,
                                    const mb_request_t *ptRequest)
{
    uint16_t hwLength;
    uint16_t hwCRC;

    pchBuf[0] = ptRequest->chSlave;
    pchBuf[1] = ptRequest->chCode;
    pchBuf[2] = ptRequest->hwDataAddr >> 8;
    pchBuf[3] = ptRequest->hwDataAddr;
    pchBuf[4] = ptRequest->hwDataNum >> 8;
    pchBuf[5] = ptRequest->hwDataNum;

    switch (ptRequest->chCode) {
        case MB_CODE_READ_COILS:
                FALL_THROUGH();
        case MB_CODE_READ_DISCRETE_INPUTS:
                FALL_THROUGH();
        case MB_CODE_READ_HOLDING_REGISTERS:
                FALL_THROUGH();
        case MB_CODE_READ_INPUT_REGISTERS:
            /*! Fixed length */
            hwLength = 6;
            break;

        case MB_CODE_WRITE_REGISTER:
            short_copy_xch(&pchBuf[4],
                           ptRequest->pWR,
                           ptRequest->hwDataNum,
                           true);
            hwLength = 4 + (ptRequest->hwDataNum << 1);
            break;

        case MB_CODE_WRITE_MULTIPLE_REGISTERS:
            pchBuf[6] = ptRequest->hwDataNum << 1;
            short_copy_xch(&pchBuf[7],
                           ptRequest->pWR,
                           ptRequest->hwDataNum,
                           true);
            hwLength = 4 + 3 + pchBuf[6];
            break;

        case MB_CODE_WRITE_COIL:
            short_copy_xch(&pchBuf[4],
                           ptRequest->pWR,
                           ptRequest->hwDataNum,
                           true);
            hwLength = 4 + (ptRequest->hwDataNum << 1);
            break;

        default:
            return 0;
    }

    hwCRC = CALC_MODBUS_CRC16(pchBuf, hwLength);
    pchBuf[hwLength++] = hwCRC & 0x00FF;
    pchBuf[hwLength++] = (hwCRC & 0xFF00) >> 8;

    return hwLength;
}

static bool master_recv_response(uint8_t *pchBuf,
                                 uint16_t hwLength,
                                 mb_response_t *ptResponse)
{
    if (hwLength < MB_SER_ADU_SIZE_MIN ||
        hwLength > MB_SER_ADU_SIZE_MAX) {
        return false;
    }

    if (!VALID_MODBUS_FMT(pchBuf, hwLength)) {
        return false;
    }

    ptResponse->chSlave = pchBuf[0];
    ptResponse->chCode = pchBuf[1];

    switch (ptResponse->chCode) {
        case MB_CODE_READ_DISCRETE_INPUTS:
                FALL_THROUGH();
        case MB_CODE_READ_INPUT_REGISTERS:
                FALL_THROUGH();
        case MB_CODE_READ_COILS:
                FALL_THROUGH();
        case MB_CODE_READ_HOLDING_REGISTERS:
            ptResponse->hwByteCount = pchBuf[2];
            ptResponse->pchRegStart = &pchBuf[3];
            break;

        case MB_CODE_WRITE_COIL:
                FALL_THROUGH();
        case MB_CODE_WRITE_REGISTER:
            ptResponse->hwDataAddr = CHAR_HL_SHORT(pchBuf[2], pchBuf[3]);
            // ptResponse->hwDataNum = 1;
            break;

        case MB_CODE_WRITE_MULTIPLE_REGISTERS:
            ptResponse->hwDataAddr = CHAR_HL_SHORT(pchBuf[2], pchBuf[3]);
            ptResponse->hwDataNum = CHAR_HL_SHORT(pchBuf[4], pchBuf[5]);
            break;

        case (0x80 + MB_CODE_READ_INPUT_REGISTERS):
            FALL_THROUGH();
        case (0x80 + MB_CODE_READ_HOLDING_REGISTERS):
            FALL_THROUGH();
        case (0x80 + MB_CODE_READ_DISCRETE_INPUTS):
            FALL_THROUGH();
        case (0x80 + MB_CODE_READ_COILS):
            FALL_THROUGH();
        case (0x80 + MB_CODE_WRITE_COIL):
            FALL_THROUGH();
        case (0x80 + MB_CODE_WRITE_REGISTER):
             return false;
        default:
            return false;
    }

    return true;
}

static bool master_do_action(mb_master_t *ptMaster, uint32_t wTimeout)
{
    uint16_t hwLength;
    mb_request_t *ptRequest = (mb_request_t *) &ptMaster->tRequest;

    if (MB_SLAVE_ADDRESS_MIN > ptRequest->chSlave ||
        MB_SLAVE_ADDRESS_MAX < ptRequest->chSlave) {
        return false;
    }

    if ((ptMaster->tSIO.hwRcvBufSize < MB_SER_ADU_SIZE_MAX) ||
        (ptMaster->tSIO.hwSndBufSize < MB_SER_ADU_SIZE_MAX)) {
        return false;
    }

    switch (ptRequest->chCode) {
        case MB_CODE_WRITE_REGISTER:
                FALL_THROUGH();
        case MB_CODE_WRITE_MULTIPLE_REGISTERS:
                FALL_THROUGH();
        case MB_CODE_READ_INPUT_REGISTERS:
                FALL_THROUGH();
        case MB_CODE_READ_HOLDING_REGISTERS:
            if (MB_READ_REG_CNT_MIN > ptRequest->hwDataNum ||
                MB_READ_REG_CNT_MAX < ptRequest->hwDataNum) {
                return false;
            }
            break;
        case MB_CODE_WRITE_COIL:
                FALL_THROUGH();
        case MB_CODE_READ_COILS:
                FALL_THROUGH();
        case MB_CODE_READ_DISCRETE_INPUTS:
            if (MB_READ_COIL_CNT_MIN > ptRequest->hwDataNum ||
                MB_READ_COIL_CNT_MAX < ptRequest->hwDataNum) {
                return false;
            }
            break;
        default:
            return false;
    }

    hwLength = master_send_request(ptMaster->tSIO.pSndBuf, &ptMaster->tRequest);
    if (hwLength && hwLength <= ptMaster->tSIO.hwSndBufSize) {
        ptMaster->tSIO.hwSndLen = hwLength;
        ptMaster->tRequest.wTimeout = wTimeout;
        return true;
    }

    return false;
}

#ifdef C_MODBUS_NOBLOCK
static int32_t recv_timeout_cb(void *arg)
{
    mb_master_t *ptMaster = (mb_master_t *) arg;
    soft_timer_delete(ptMaster->tRequest.ptTimer);
    ptMaster->tRequest.ptTimer = NULL;
    mb_post_event(ptMaster, EV_MASTER_RECV_TIMEOUT);
    return 0;
}
#endif

static bool is_req_resp_match(const mb_request_t *ptRequest,
                              const mb_response_t *ptResponse)
{
    if (ptRequest->chCode != ptResponse->chCode) {
        return false;
    }

    switch (ptRequest->chCode) {
        case MB_CODE_READ_DISCRETE_INPUTS:
                FALL_THROUGH();
        case MB_CODE_READ_COILS:
            if (CALC_READ_COILS_REQUIRED_BYTES(ptRequest->hwDataNum) !=
                ptResponse->hwByteCount) {
                return false;
            }
            break;

        case MB_CODE_READ_INPUT_REGISTERS:
                FALL_THROUGH();
        case MB_CODE_READ_HOLDING_REGISTERS:
            if (ptRequest->hwDataNum != (ptResponse->hwByteCount >> 1)) {
                return false;
            }
            break;

        case MB_CODE_WRITE_COIL:
                FALL_THROUGH();
        case MB_CODE_WRITE_REGISTER:
            if (ptRequest->hwDataAddr != ptResponse->hwDataAddr) {
                return false;
            }
            break;
        case MB_CODE_WRITE_MULTIPLE_REGISTERS:
            if (ptRequest->hwDataAddr != ptResponse->hwDataAddr ||
                ptRequest->hwDataNum != ptResponse->hwDataNum) {
                return false;
            }
            break;

        default:
            break;
    }

    return true;
}

static void master_poll(mb_master_t *ptMaster)
{
    int32_t n;
    enum work_state {
        START = 0,
        WAIT_EVENT,
        SEND_REQUEST,
        WAIT_RESPONSE,
        CONTINUE_READ,
        HANDLE_RESPONSE,
        HANDLE_ERROR,
    };

    switch (ptMaster->chState) {
        case START:
            ptMaster->tStatus = MB_STATUE_IDLE;
            ptMaster->chErr = 0;

                FALL_THROUGH();
        case WAIT_EVENT:
            ptMaster->chState = WAIT_EVENT;
            if (!mb_wait_event(ptMaster, EV_MASTER_START_SEND)) {
                break;
            }

                FALL_THROUGH();
        case SEND_REQUEST:
            ptMaster->chState = SEND_REQUEST;
#ifdef C_MODBUS_CLEAN_RECEIVER_BUFFER
            /*! clean receive buffer */
            do {
                n = ptMaster->tSIO.fnRecv(ptMaster->tSIO.pRcvBuf,
                                          ptMaster->tSIO.hwRcvBufSize, 0);
            } while (n > 0);
#endif
#ifdef C_MODBUS_NOBLOCK
            ptMaster->tRequest.ptTimer = \
                soft_timer_create(ptMaster->tRequest.wTimeout,
                                  false,
                                  recv_timeout_cb,
                                  ptMaster);
            if (NULL == ptMaster->tRequest.ptTimer) {
                ptMaster->chState = HANDLE_ERROR;
                ptMaster->chErr = MB_ERR_CREATE_TIMER;
                break;
            }
#endif
            n = ptMaster->tSIO.fnSend(ptMaster->tSIO.pSndBuf,
                                      ptMaster->tSIO.hwSndLen,
                                      ptMaster->tRequest.wTimeout);
            if (n <= 0) {
                soft_timer_delete(ptMaster->tRequest.ptTimer);
                ptMaster->chState = HANDLE_ERROR;
                ptMaster->chErr = MB_ERR_SEND_FAILED;
                break;
            }

            soft_timer_reset(ptMaster->tRequest.ptTimer);

                FALL_THROUGH();
        case WAIT_RESPONSE:
            ptMaster->chState = WAIT_RESPONSE;
            n = ptMaster->tSIO.fnRecv(ptMaster->tSIO.pRcvBuf,
                                      ptMaster->tSIO.hwRcvBufSize,
                                      ptMaster->tRequest.wTimeout);

#ifdef C_MODBUS_NOBLOCK
            /*! read in poll mode*/
            if (0 < n) {
                ptMaster->chContinueCount = 0;
                ptMaster->tSIO.hwRcvLen = (uint16_t)n;
            } else if (mb_wait_event(ptMaster, EV_MASTER_RECV_TIMEOUT)) {
                ptMaster->chErr = MB_ERR_RECV_TIMEOUT;
                ptMaster->chState = HANDLE_ERROR;
                break;
            } else {
                break;
            }

                FALL_THROUGH();
        case CONTINUE_READ:
            ptMaster->chState = CONTINUE_READ;

            /*! continue read data */
            n = ptMaster->tSIO.fnRecv(ptMaster->tSIO.pRcvBuf + ptMaster->tSIO.hwRcvLen,
                                      ptMaster->tSIO.hwRcvBufSize - ptMaster->tSIO.hwRcvLen, 0);
            if (mb_wait_event(ptMaster, EV_MASTER_RECV_TIMEOUT)) {
                ptMaster->tSIO.hwRcvLen += (0 < n ? n : 0);
            }
#ifdef C_MODBUS_CONTINUE_READ
            else if (ptMaster->chContinueCount > MAXIMUM_READ_CONTINUE_COUNT) {
                soft_timer_delete(ptMaster->tRequest.ptTimer);
            } else {
                ptMaster->tSIO.hwRcvLen += (0 < n ? n : 0);
                ptMaster->chContinueCount = (0 < n ? 0 : ptMaster->chContinueCount + 1);
                break;
            }
#else
            else {
                ptMaster->tSIO.hwRcvLen += (0 < n ? n : 0);
                break;
            }
#endif
#else
                if (0 == ptMaster->tSIO.hwRcvLen) {
                    ptMaster->chState = HANDLE_ERROR;
                    break;
                }
#endif
                FALL_THROUGH();
        case HANDLE_RESPONSE:
            ptMaster->chState = HANDLE_RESPONSE;
            if (!master_recv_response(ptMaster->tSIO.pRcvBuf,
                                      ptMaster->tSIO.hwRcvLen,
                                      &ptMaster->tResponse)) {
                ptMaster->chErr = MB_ERR_REQUEST;
                ptMaster->chState = HANDLE_ERROR;
                break;
            }

            if (is_req_resp_match(&ptMaster->tRequest, &ptMaster->tResponse)) {
                master_req_resp_cb(ptMaster, &ptMaster->tRequest,
                                   &ptMaster->tResponse);
                ptMaster->chState = START;
                break;
            } else {
                ptMaster->chErr = MB_ERR_RESPONSE;
            }

                FALL_THROUGH();
        default:
        case HANDLE_ERROR:
            ptMaster->chState = HANDLE_ERROR;
            port_error_cb(ptMaster, &ptMaster->tRequest, &ptMaster->tResponse);
            ptMaster->chState = START;
            break;
    }
}

void mb_master_poll(mb_master_t *ptMaster)
{
    if (NULL != ptMaster && ptMaster->bInitOk) {
#ifdef C_MODBUS_NOBLOCK
        soft_timer_process();
#endif
        master_poll(ptMaster);
    }
}

static int32_t do_request(mb_master_t *ptMaster, const mb_request_t *ptRequest)
{
    if (&ptMaster->tRequest != ptRequest) {
        ptMaster->tRequest = *ptRequest;
    }

    if (master_do_action(ptMaster, ptRequest->wTimeout)) {
        ptMaster->tStatus = MB_STATUS_BUSY;
        mb_post_event(ptMaster, EV_MASTER_START_SEND);
        return 0;
    } else {
        mb_post_event(ptMaster, EV_MASTER_ERROR);
    }

    return -1;
}

int32_t mb_do_request(mb_master_t *ptMaster, const mb_request_t *ptRequest)
{
    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    CHECK_PARAM_NULL_PTR_RETURN(ptRequest);

    return do_request(ptMaster, ptRequest);
}

int32_t mb_read_hold_register(mb_master_t *ptMaster,
                              uint8_t chSlave,
                              uint16_t hwDataAddr,
                              uint16_t hwDataNumber,
                              uint32_t wTimeout)
{
    mb_request_t *ptRequest;

    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    ptRequest = (mb_request_t *) &ptMaster->tRequest;
    CHECK_MASTER_BUSY_RETURN(ptMaster);

    ptRequest->chSlave = chSlave;
    ptRequest->hwDataAddr = hwDataAddr;
    ptRequest->hwDataNum = hwDataNumber;
    ptRequest->chCode = MB_CODE_READ_HOLDING_REGISTERS;
    ptRequest->wTimeout = wTimeout;

    return do_request(ptMaster, ptRequest);
}

int32_t mb_read_input_register(mb_master_t *ptMaster,
                               uint8_t chSlave,
                               uint16_t hwDataAddr,
                               uint16_t hwDataNumber,
                               uint32_t wTimeout)
{
    mb_request_t *ptRequest;

    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    ptRequest = (mb_request_t *) &ptMaster->tRequest;
    CHECK_MASTER_BUSY_RETURN(ptMaster);

    ptRequest->chSlave = chSlave;
    ptRequest->hwDataAddr = hwDataAddr;
    ptRequest->hwDataNum = hwDataNumber;
    ptRequest->chCode = MB_CODE_READ_INPUT_REGISTERS;
    ptRequest->wTimeout = wTimeout;

    return do_request(ptMaster, ptRequest);
}

int32_t mb_read_discrete_inputs(mb_master_t *ptMaster,
                                uint8_t chSlave,
                                uint16_t hwDataAddr,
                                uint16_t hwDataNumber,
                                uint32_t wTimeout)
{
    mb_request_t *ptRequest;

    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    ptRequest = (mb_request_t *) &ptMaster->tRequest;
    CHECK_MASTER_BUSY_RETURN(ptMaster);

    ptRequest->chSlave = chSlave;
    ptRequest->hwDataAddr = hwDataAddr;
    ptRequest->hwDataNum = hwDataNumber;
    ptRequest->chCode = MB_CODE_READ_DISCRETE_INPUTS;
    ptRequest->wTimeout = wTimeout;

    return do_request(ptMaster, ptRequest);
}

int32_t mb_read_coils(mb_master_t *ptMaster,
                      uint8_t chSlave,
                      uint16_t hwDataAddr,
                      uint16_t hwDataNumber,
                      uint32_t wTimeout)
{
    mb_request_t *ptRequest;

    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    ptRequest = (mb_request_t *) &ptMaster->tRequest;
    CHECK_MASTER_BUSY_RETURN(ptMaster);

    ptRequest->chSlave = chSlave;
    ptRequest->hwDataAddr = hwDataAddr;
    ptRequest->hwDataNum = hwDataNumber;
    ptRequest->chCode = MB_CODE_READ_COILS;
    ptRequest->wTimeout = wTimeout;

    return do_request(ptMaster, ptRequest);
}

int32_t mb_write_hold_register(mb_master_t *ptMaster,
                               uint8_t chSlave,
                               uint16_t hwDataAddr,
                               uint16_t hwValue,
                               uint32_t wTimeout)
{
    mb_request_t *ptRequest;

    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    ptRequest = (mb_request_t *) &ptMaster->tRequest;
    CHECK_MASTER_BUSY_RETURN(ptMaster);

    ptRequest->chSlave = chSlave;
    ptRequest->hwDataAddr = hwDataAddr;
    ptRequest->hwDataNum = 1;
    ptRequest->chCode = MB_CODE_WRITE_REGISTER;
    ptRequest->hwValue = hwValue;
    ptRequest->pWR = &ptRequest->hwValue;
    ptRequest->wTimeout = wTimeout;

    return do_request(ptMaster, ptRequest);
}

int32_t mb_write_hold_multi_register(mb_master_t *ptMaster,
                                     uint8_t chSlave,
                                     uint16_t hwDataAddr,
                                     const uint16_t *phwBuf,
                                     uint16_t hwDataNumber,
                                     uint32_t wTimeout)
{
    mb_request_t *ptRequest;

    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    ptRequest = (mb_request_t *) &ptMaster->tRequest;
    CHECK_MASTER_BUSY_RETURN(ptMaster);

    ptRequest->chSlave = chSlave;
    ptRequest->hwDataAddr = hwDataAddr;
    ptRequest->hwDataNum = hwDataNumber;
    ptRequest->chCode = MB_CODE_WRITE_MULTIPLE_REGISTERS;
    ptRequest->pWR = phwBuf;
    ptRequest->wTimeout = wTimeout;

    return do_request(ptMaster, ptRequest);
}

int32_t mb_write_single_coil(mb_master_t *ptMaster,
                             uint8_t chSlave,
                             uint16_t hwCoilAddr,
                             uint8_t chSwitch,
                             uint32_t wTimeout)
{
    mb_request_t *ptRequest;

    CHECK_PARAM_NULL_PTR_RETURN(ptMaster);
    ptRequest = (mb_request_t *) &ptMaster->tRequest;
    CHECK_MASTER_BUSY_RETURN(ptMaster);

    ptRequest->chSlave = chSlave;
    ptRequest->hwDataAddr = hwCoilAddr;
    ptRequest->hwDataNum = 1;
    ptRequest->chCode = MB_CODE_WRITE_COIL;
    ptRequest->hwValue = (chSwitch == 0 ? 0x0000 : 0xFF00);
    ptRequest->pWR = &ptRequest->hwValue;
    ptRequest->wTimeout = wTimeout;

    return do_request(ptMaster, ptRequest);
}

#endif

// -------------------------------------------------------------------------
//                              从机
// -------------------------------------------------------------------------
#if defined(C_MODBUS_SLAVE_ENABLE)

extern void port_slave_cb(const mb_slave_t *ptSlave, mb_request_t *ptRequest);

extern bool port_valid_slave_cb(const mb_slave_t *ptSlave, uint8_t chSlave);

extern void port_slave_error_cb(const mb_slave_t *ptSlave, mb_request_t *ptRequest);

static bool slave_rcv_handle(mb_slave_t *ptSlave, mb_request_t *ptRequest)
{
    if (ptSlave->tSIO.hwRcvLen < MB_SER_ADU_SIZE_MIN ||
        ptSlave->tSIO.hwRcvLen > MB_SER_ADU_SIZE_MAX) {
        return false;
    }

    if (!VALID_MODBUS_FMT(ptSlave->tSIO.pRcvBuf,
                          ptSlave->tSIO.hwRcvLen)) {
        return false;
    }

    ptRequest->chSlave = ptSlave->tSIO.pRcvBuf[0];
    ptRequest->chCode = ptSlave->tSIO.pRcvBuf[1];

    return true;
}

static bool slave_handle_request(mb_slave_t *ptSlave,
                                 mb_request_t *ptRequest)
{
    uint16_t hwValue;

    switch (ptRequest->chCode) {
        case MB_CODE_WRITE_COIL:
            ptRequest->hwDataAddr = CHAR_HL_SHORT(ptSlave->tSIO.pRcvBuf[2], ptSlave->tSIO.pRcvBuf[3]);
            ptRequest->hwDataNum = 1;
            hwValue = CHAR_HL_SHORT(ptSlave->tSIO.pRcvBuf[4], ptSlave->tSIO.pRcvBuf[5]);
            if ((hwValue == 0) || (hwValue == 0xFF00)) {
                ptRequest->pWR = &ptSlave->tSIO.pRcvBuf[4];
            } else {
                ptSlave->chExceptionCode = MB_EX_ILLEGAL_DATA_VALUE;
                return false;
            }
            break;

        case MB_CODE_WRITE_REGISTER:
            ptRequest->hwDataAddr = CHAR_HL_SHORT(ptSlave->tSIO.pRcvBuf[2], ptSlave->tSIO.pRcvBuf[3]);
            ptRequest->hwDataNum = 1;
            ptRequest->pWR = &ptSlave->tSIO.pRcvBuf[4];
            break;

        case MB_CODE_WRITE_MULTIPLE_REGISTERS:
            ptRequest->pWR = &ptSlave->tSIO.pRcvBuf[7];
                    FALL_THROUGH();
        case MB_CODE_READ_HOLDING_REGISTERS:
            ptRequest->hwDataAddr = CHAR_HL_SHORT(ptSlave->tSIO.pRcvBuf[2], ptSlave->tSIO.pRcvBuf[3]);
            ptRequest->hwDataNum = CHAR_HL_SHORT(ptSlave->tSIO.pRcvBuf[4], ptSlave->tSIO.pRcvBuf[5]);
            if ((MB_READ_REG_CNT_MIN > ptRequest->hwDataNum) ||
                (MB_READ_REG_CNT_MAX < ptRequest->hwDataNum)) {
                ptSlave->chExceptionCode = MB_EX_ILLEGAL_DATA_VALUE;
                return false;
            }
            break;

//        case MB_CODE_WRITE_MULTIPLE_COILS:
//            ptRequest->pWR = &ptSlave->tSIO.pRcvBuf[7];
//                    FALL_THROUGH();
        case MB_CODE_READ_COILS:
            ptRequest->hwDataAddr = CHAR_HL_SHORT(ptSlave->tSIO.pRcvBuf[2], ptSlave->tSIO.pRcvBuf[3]);
            ptRequest->hwDataNum = CHAR_HL_SHORT(ptSlave->tSIO.pRcvBuf[4], ptSlave->tSIO.pRcvBuf[5]);
            if ((MB_READ_COIL_CNT_MIN > ptRequest->hwDataNum) ||
                (MB_READ_COIL_CNT_MAX < ptRequest->hwDataNum)) {
                ptSlave->chExceptionCode = MB_EX_ILLEGAL_DATA_VALUE;
                return false;
            }
            break;

        default:
            ptSlave->chExceptionCode = MB_EX_ILLEGAL_FUNCTION;
            return false;
    }

    return true;
}

bool slave_error_default_cb(const mb_slave_t *ptSlave, mb_request_t *ptRequest) {

    uint16_t hwCrc;

    switch (ptSlave->chExceptionCode) {
        default:
            return false;

        case MB_EX_ILLEGAL_FUNCTION:
            ptSlave->tSIO.pSndBuf[2] = 0x01;
            break;

        case MB_EX_ILLEGAL_DATA_VALUE:
            ptSlave->tSIO.pSndBuf[2] = 0x03;
            break;

    }
    ptSlave->tSIO.pSndBuf[0] = ptRequest->chSlave;
    ptSlave->tSIO.pSndBuf[1] = ptRequest->chCode + 0x80;

    hwCrc = MODBUS_CRC16(ptSlave->tSIO.pSndBuf, 3);
    ptSlave->tSIO.pSndBuf[3] = hwCrc;
    ptSlave->tSIO.pSndBuf[4] = hwCrc >> 8;
    ptSlave->tSIO.fnSend(ptSlave->tSIO.pSndBuf, 5, 25);

    return true;
}

void mb_slave_push(mb_slave_t *ptSlave)
{
    enum { START = 0, READ_DATA, CHECK_REQUEST, HANDLE_REQUEST };

    if (NULL == ptSlave) {
        return;
    }

    switch (ptSlave->chState) {
        case START:
            ptSlave->tStatus = MB_STATUE_IDLE;
                FALL_THROUGH();

        case READ_DATA:
            ptSlave->chState = READ_DATA;
            ptSlave->tSIO.hwRcvLen = \
                    ptSlave->tSIO.fnRecv(ptSlave->tSIO.pRcvBuf, ptSlave->tSIO.hwRcvBufSize, 0);
            if (0 == ptSlave->tSIO.hwRcvLen) {
                break;
            }

                FALL_THROUGH();
        case CHECK_REQUEST:
            ptSlave->chState = CHECK_REQUEST;
            ptSlave->tStatus = MB_STATUS_BUSY;
            if (!port_valid_slave_cb(ptSlave,
                                     ptSlave->tSIO.pRcvBuf[0])) {
                ptSlave->chState = START;
                break;
            }

            if (!slave_rcv_handle(ptSlave, &ptSlave->tRequest)) {
                ptSlave->chState = START;
                break;
            }

                FALL_THROUGH();
        case HANDLE_REQUEST:
            ptSlave->chState = HANDLE_REQUEST;
            if (slave_handle_request(ptSlave, &ptSlave->tRequest)) {
                port_slave_cb(ptSlave, &ptSlave->tRequest);
            } else {
                if (!slave_error_default_cb(ptSlave, &ptSlave->tRequest)) {
                    port_slave_error_cb(ptSlave, &ptSlave->tRequest);
                }
            }

                FALL_THROUGH();
        default:
            ptSlave->chState = START;
            break;
    }
}

#endif

/*************************** End of file ****************************/
