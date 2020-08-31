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
#ifndef CORE_USER_UTILITY_COMPILER_LZ_DEF_H_
#define CORE_USER_UTILITY_COMPILER_LZ_DEF_H_
#ifdef __cplusplus
extern "C" {
#endif

#define LZ_OFFSET_OF(TYPE, MEMBER) \
            ((char *)&((TYPE *)0)->MEMBER)

#define LZ_CONTAINER_OF(PTR, TYPE, MEMBER) \
            ((TYPE *)((char *)(PTR) - LZ_OFFSET_OF(TYPE, MEMBER)))

#ifdef __cplusplus
}
#endif
#endif

/*************************** End of file ****************************/
