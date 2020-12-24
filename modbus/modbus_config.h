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
#ifndef MODBUS_CONFIG_H_
#define MODBUS_CONFIG_H_
#ifdef __cplusplus
extern "C" {
#endif

/* Includes --------------------------------------------------------*/
#include "../../utility/lz_types.h"
#include "../../utility/lz_utils.h"

/* Global variables ------------------------------------------------*/
/* Global typedef --------------------------------------------------*/
/* Global define ---------------------------------------------------*/
//#define C_MODBUS_MASTER_ENABLE
#define C_MODBUS_SLAVE_ENABLE
#define C_MODBUS_NOBLOCK
#define C_MODBUS_CLEAN_RECEIVER_BUFFER
//#define C_MODBUS_CONTINUE_READ

/* Global macro ----------------------------------------------------*/
/* Global variables ------------------------------------------------*/
/* Global function prototypes --------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif

/*************************** End of file ****************************/
