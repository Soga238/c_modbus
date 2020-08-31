/*********************************************************************
*                      Hangzhou Lingzhi Lzlinks                      *
*                        Internet of Things                          *
**********************************************************************
*                                                                    *
*            (c) 2018 - 8102 Hangzhou Lingzhi Lzlinks                *
*                                                                    *
*       www.lzlinks.com     Support: embedzjh@gmail.com              *
*                                                                    *
**********************************************************************
*                                                                    *
*       arm_compiler.h *                                             *
*                                                                    *
**********************************************************************
*/
#ifndef __ARM_COMPILER_H__
#define __ARM_COMPILER_H__

#include "cmsis_compiler.h"

//! note for IAR
#if defined(__IAR_SYSTEMS_ICC__)
#   define __IS_COMPILER_IAR__                 1
#endif

//! \note for arm compiler 5
#ifdef __IS_COMPILER_ARM_COMPILER_5__
#   undef __IS_COMPILER_ARM_COMPILER_5__
#endif
#if ((__ARMCC_VERSION >= 5000000) && (__ARMCC_VERSION < 6000000))
#   define __IS_COMPILER_ARM_COMPILER_5__      1
#endif

#if ((__ARMCC_VERSION >= 6000000) && (__ARMCC_VERSION < 7000000))
#   define __IS_COMPILER_ARM_COMPILER_6__      1
#endif

/* -----------------  Start of section using anonymous unions  -------------- */
#if __IS_COMPILER_ARM_COMPILER_5__
  //#pragma push
  #pragma anon_unions
#elif __IS_COMPILER_ARM_COMPILER_6__
#elif __IS_COMPILER_IAR__
  #pragma language=extended
#elif __IS_COMPILER_GCC__
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


# if __IS_COMPILER_IAR__
#   define WEAK                     __weak
#   define __AT_ADDR(__ADDR)        @ __ADDR
#elif __IS_COMPILER_ARM_COMPILER_5__
#   define WEAK                     __attribute__((weak))
#   define __AT_ADDR(__ADDR)        __attribute((at(__ADDR)))
#elif __IS_COMPILER_ARM_COMPILER_6__
#   define WEAK                     __attribute__((weak))
#   define __AT_ADDR(__ADDR)        __attribute__((section(".ARM__at_"#__ADDR)))
#elif __IS_COMPILER_LLVM__
#   define WEAK                     __weak_symbol
#   define __AT_ADDR(__ADDR)        __section(".ARM.__at_"#__ADDR)
#else // GCC compiler
#   define WEAK                     __attribute__((weak))
#   define __AT_ADDR(__ADDR)        __section(".ARM.__at_"#__ADDR)
#endif

#define AT_ADDR(__ADDR)     __AT_ADDR(__ADDR)

#ifndef STATIC
#define STATIC  static
#endif

#ifndef INLINE
#define INLINE  __INLINE
#endif

#endif

/*************************** End of file ****************************/
