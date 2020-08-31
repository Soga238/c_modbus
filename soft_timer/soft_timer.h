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
#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes --------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Global variables ------------------------------------------------*/
/* Global typedef --------------------------------------------------*/
typedef int32_t  (*timer_callback_fn)(void *pArg);
typedef uint32_t (*timer_tick_fn)(void);

typedef struct _timer
{
    bool                bPeriodic;           /*! 单次触发/周期触发     */
    uint32_t            wStart;              /*! 计时器起始时间        */
    uint32_t            wTimeout;            /*! 计时器计时时间        */
    timer_callback_fn   pfTimerCallback;     /*! 计时回掉函数          */
    void               *pArg;                /*! 回调函数的参数        */
} soft_timer_t;

typedef struct _timer_table
{
    soft_timer_t          data;              /*! 本计时器结点数据      */
    struct _timer_table  *next;              /*! 下一个定时器结点地址  */
} timer_table_t;

#ifndef size_t
    #define CUSTOM_SIZE_T
    #define size_t unsigned long
#endif

typedef struct _malloc_hook
{
      void *(*malloc_fn)(size_t sz);        /*! pointer of malloc    */
      void (*free_fn)(void *ptr);           /*! pointer of free      */
} timer_hook_t;

#ifdef CUSTOM_SIZE_T
    #undef CUSTOM_SIZE_T
    #undef size_t
#endif

/* Global define ---------------------------------------------------*/
#define MAX_VALUE_32_BIT        (0xFFFFFFFF) /*! 32bit最大ms数 */
#define MAX_VALUE_16_BIT        (0xFFFF)

/* Global macro ----------------------------------------------------*/
/* Global variables ------------------------------------------------*/
/* Global function prototypes --------------------------------------*/

extern bool soft_timer_init(timer_tick_fn pfGetSysTick, uint32_t wMaxTime);

extern bool soft_timer_init_hook(timer_hook_t *pHooks);

extern timer_table_t* soft_timer_create(uint32_t wTimeout, bool bPeriodic,
                                        timer_callback_fn pFunc,
                                        void *pArg);

extern bool soft_timer_delete(timer_table_t* ptNode);

extern bool soft_timer_reset(timer_table_t* ptNode);

extern bool soft_timer_process(void);

#ifdef __cplusplus
}
#endif
#endif

/*************************** End of file ****************************/
