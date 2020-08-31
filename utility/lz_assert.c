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

#include ".\lz_assert.h"
#include "..\usr_app\usr_app_cfg.h"

#include "cmsis_os2.h"
/*********************************************************************
*
*       Function prototypes
*
**********************************************************************
*/

void assert_handle(uint8_t *expr, uint8_t *file, uint32_t line)
{
    SYSLOG_F("assert(%s): %s line: %d", expr, file, line);
    
    osDelay(200);   /*! delay for display more*/   
    
    while(1) {
        ;
    }
}

/*************************** End of file ****************************/
