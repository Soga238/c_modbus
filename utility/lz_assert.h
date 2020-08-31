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
#ifndef __LZ_ASSERT_H__
#define __LZ_ASSERT_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include ".\lz_types.h"

#if defined(USE_ASSERT)

#define ASSERT(expr)    ((expr) ? (void)0 : assert_handle((uint8_t *)#expr, (uint8_t *)__FILE__, __LINE__))

#define TRACE(expr, ...)   \
        ((expr) ? (void)0 : assert_handle((uint8_t *)#expr, #__VA_ARGS__, (uint8_t *)__FILE__, __LINE__))

#else

#define ASSERT(expr)    ((void)0)

#define TRACE(expr, ...)    ((void)0)
    
#endif
    

extern void assert_handle(uint8_t *expr, uint8_t* file, uint32_t line);

#ifdef __cplusplus
}
#endif

#endif
/*************************** End of file ****************************/
