/****************************************************************************
 * Copyright (c) [2019] [core.zhang@outlook.com]                            *
 * [Software Name] is licensed under Mulan PSL v2.                          *
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
#ifndef _LZ_UTILS_H_
#define _LZ_UTILS_H_

#include <stdint.h>
#include <stdbool.h>

#if defined(HALF_WORD_BYTE_ALGIN_REQUIRED)
#define BYTE_ALGINED_REQUIRED
#endif

#define CALC_CRC_WITH_TABLE
#ifdef CALC_CRC_WITH_TABLE
#define MODBUS_CRC16(BUFFER, LENGTH)   crc16_with_table(0xFFFF, BUFFER, LENGTH)
extern uint16_t crc16_with_table(uint16_t hwInitValue, const uint8_t * pchBuf, uint16_t hwLength);
#else
#define MODBUS_CRC16(BUFFER, LENGTH)   crc16(0xFFFF, BUFFER, LENGTH)
extern uint16_t crc16(uint16_t hwInitValue, const uint8_t *pchBuf, uint16_t hwLength);
#endif

#ifndef CHAR_HL_SHORT
#define CHAR_HL_SHORT(__H, __L) ((uint16_t)(((0x00FF & (__H)) << 8) | (__L)))
#endif

#ifndef SHORT_XCH_HL
#define SHORT_XCH_HL(__HW) (((uint16_t)((__HW) & 0x00FF) << 8) | (uint8_t)(((__HW) & 0xFF00) >> 8));
#endif

extern bool valid_modbus_crc(const uint8_t *pchBuf, uint16_t hwLength);
extern uint16_t char_hl_short(uint8_t chHigh, uint8_t chLow);
extern uint16_t short_xch_hl(uint16_t hwValue);
extern void short_copy_xch(void *dst, const void *src, int32_t nNumofShort, bool bSwitch);

#endif

/*************************** End of file ****************************/
