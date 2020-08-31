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
#ifndef __LZ_COMMON_H__
#define __LZ_COMMON_H__
#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX
#define MAX(x, y)               (((x) < (y)) ? (y) : (x))
#endif
    
#ifndef MIN
#define MIN(x, y)               (((x) < (y)) ? (x) : (y))
#endif
    
/** 
 *  \brief calc structural member offest
 */
#define LZ_OFFSET(structure, member)    ((size_t)(&(((structure *)0)->member)))

#ifndef offsetof
#define offsetof(type, member)           LZ_OFFSET(type, member)
#endif
    
#define LZ_CONTAINER_OF(ptr, type, member) \
            ((type *)((char *)(ptr) - offsetof(type,member)))
      
#ifndef container_of
#define container_of(ptr, type, member)  LZ_CONTAINER_OF(ptr, type, member)
#endif
                
#define LZ_ELEMENTS(array)              (sizeof (array) / sizeof ((array) [0]))
          
#define countof(array)                   LZ_ELEMENTS(array)
                
#define cup8(addr)          (*(volatile uint8_t *)(addr))
#define cup16(addr)         (*(volatile uint16_t *)(addr))
#define cup32(addr)         (*(volatile uint32_t *)(addr))
    
#ifdef __cplusplus
    }
#endif
#endif
/*************************** End of file ****************************/
