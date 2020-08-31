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
/* Includes --------------------------------------------------------*/
#include "./soft_timer.h"

/* Global variables ------------------------------------------------*/
/* Private typedef -------------------------------------------------*/
/* Private define --------------------------------------------------*/
#ifndef CUSTOM_TIMER_MALLOC
#include <stdlib.h>
    #define TIMER_MALLOC    malloc
    #define TIMER_FREE      free
#else
    #ifndef TIMER_MALLOC
        #include "../heap/heap.h"
        #define TIMER_MALLOC    port_malloc_4
        #define TIMER_FREE      port_free_4
    #endif
#endif

/* Private macro ---------------------------------------------------*/
/* Private variables -----------------------------------------------*/
static uint32_t s_wTimeMaxValue = MAX_VALUE_32_BIT;
static timer_table_t *s_ptTimerTableHead = NULL;
static timer_tick_fn s_pfGetSysTick = NULL;
static timer_hook_t s_tHooks = {TIMER_MALLOC, TIMER_FREE};

static bool s_bTimerInitOk = false;

/* Private function prototypes -------------------------------------*/
/* Private functions -----------------------------------------------*/

/**
 * \brief Initialize the timer control block.
 * \param pfGetSysTick Pointer to the function.
 * \param wMaxTime  Size of character array.
 * \return  true:   Success
 *          false:  Fail
 */
bool soft_timer_init(timer_tick_fn pfGetSysTick, uint32_t wMaxTime)
{
    if (s_bTimerInitOk) {
        return true;
    } else if (NULL == pfGetSysTick) {
        return false;
    }

    s_ptTimerTableHead = \
        (timer_table_t *) s_tHooks.malloc_fn(sizeof(timer_table_t));
    if (NULL == s_ptTimerTableHead) {
        return false;
    }

    s_ptTimerTableHead->next = NULL;
    s_pfGetSysTick = pfGetSysTick;
    s_wTimeMaxValue = wMaxTime;
    s_bTimerInitOk = true;

    return true;
}

bool soft_timer_init_hook(timer_hook_t *pHooks)
{
    if (NULL == pHooks) {
        return false;
    }

    if (NULL == pHooks->malloc_fn || NULL == pHooks->free_fn) {
        return false;
    }

    s_tHooks.malloc_fn = pHooks->malloc_fn;
    s_tHooks.free_fn = pHooks->free_fn;

    return true;
}

timer_table_t *soft_timer_create(uint32_t wTimeout,
                                 bool bPeriodic,
                                 timer_callback_fn pfTimerCallback,
                                 void *pArg)
{
    timer_table_t *ptTimerNode;
    timer_table_t *ptFind;

    if (NULL == s_ptTimerTableHead) {
        return NULL;
    }

    ptTimerNode = (timer_table_t *) s_tHooks.malloc_fn(sizeof(timer_table_t));
    if (NULL == ptTimerNode) {
        return NULL;
    }

    ptTimerNode->next = NULL;
    ptTimerNode->data.bPeriodic = bPeriodic;
    ptTimerNode->data.wStart = s_pfGetSysTick();
    ptTimerNode->data.wTimeout = wTimeout;
    ptTimerNode->data.pfTimerCallback = pfTimerCallback;
    ptTimerNode->data.pArg = pArg;

    for (ptFind = s_ptTimerTableHead; NULL != ptFind->next;
         ptFind = ptFind->next) {
        /*! do nothing */
    }

    ptFind->next = ptTimerNode;

    return ptTimerNode;
}

bool soft_timer_delete(timer_table_t *ptNode)
{
    timer_table_t *ptFind;

    if (NULL == ptNode) {
        return false;
    }

    for (ptFind = s_ptTimerTableHead; NULL != ptFind; ptFind = ptFind->next) {
        if (ptNode == ptFind->next) {
            ptFind->next = ptNode->next;
            s_tHooks.free_fn(ptNode);
            return true;
        }
    }

    return false;
}

bool soft_timer_reset(timer_table_t *ptNode)
{
    if (NULL != ptNode) {
        ptNode->data.wStart = s_pfGetSysTick();
        return true;
    }
    return true;
}

bool soft_timer_process(void)
{
    uint32_t wDelta, wNow;
    timer_table_t *ptFind;
    timer_table_t *ptPre;

    if (!s_bTimerInitOk) {
        return false;
    }

    ptPre = s_ptTimerTableHead;
    for (ptFind = ptPre->next; ptFind != NULL; ptFind = ptPre->next) {

        wNow = s_pfGetSysTick();
        if (wNow >= ptFind->data.wStart) {
            wDelta = wNow - ptFind->data.wStart;
        } else {
            wDelta = (s_wTimeMaxValue - wNow) + (ptFind->data.wStart - 0);
        }

        if (wDelta < ptFind->data.wTimeout) {
            ptPre = ptFind;
            continue;
        }

        if (ptFind->data.pfTimerCallback) {
            ptFind->data.pfTimerCallback(ptFind->data.pArg);
        }

        if (ptFind->data.bPeriodic) {
            soft_timer_reset(ptFind);
            ptPre = ptFind;
        } else {
            ptPre->next = ptFind->next; /*! del timeout node */
            s_tHooks.free_fn(ptFind);
        }
    }

    return true;
}

/*************************** End of file ****************************/
