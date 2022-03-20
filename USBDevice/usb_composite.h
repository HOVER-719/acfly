#ifndef _USB_COMPOSITE_H_
#define _USB_COMPOSITE_H_

#ifdef __cplusplus
 extern "C" {
#endif
	 // FreeRTOS Headers
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
	 
#define tusb_option_h  1
#define tusb_config_h  1	 
#define tusb_h  1
#define osal_h  1	 
#define tusb_common_h  1		 
#define usbd_h  1	
#define msc_h  1	
#define usbd_pvt_h  1
#define dcd_h  1
#define msc_device_h  1
#define cdc_h  1
#define cdc_device_h  1
#define tusb_compiler_h  1
#define tusb_types_h  1
#define tusb_verify_h 1
#define tusb_error_h 1
#define tusb_timeout_h 1
#define osal_freertos_h 1


#define OPT_MCU_NONE                0
// STM32
#define OPT_MCU_STM32F0           300 ///< ST STM32F0
#define OPT_MCU_STM32F1           301 ///< ST STM32F1
#define OPT_MCU_STM32F2           302 ///< ST STM32F2
#define OPT_MCU_STM32F3           303 ///< ST STM32F3
#define OPT_MCU_STM32F4           304 ///< ST STM32F4
#define OPT_MCU_STM32F7           305 ///< ST STM32F7
#define OPT_MCU_STM32H7           306 ///< ST STM32H7
#define OPT_MCU_STM32L0           307 ///< ST STM32L0
#define OPT_MCU_STM32L1           308 ///< ST STM32L1
#define OPT_MCU_STM32L4           309 ///< ST STM32L4
// TI MSP430
#define OPT_MCU_MSP430x5xx        500 ///< TI MSP430x5xx
// Espressif
#define OPT_MCU_ESP32S2           900 ///< Espressif ESP32-S2
// defined by board.mk
#define CFG_TUSB_MCU OPT_MCU_STM32H7
#ifndef CFG_TUSB_MCU
//  #error CFG_TUSB_MCU must be defined
	 #define CFG_TUSB_MCU OPT_MCU_STM32H7
#endif



#if osal_freertos_h 
// Return immediately
#define OSAL_TIMEOUT_NOTIMEOUT (0)
// Default timeout
#define OSAL_TIMEOUT_NORMAL       (10)
// Wait forever
#define OSAL_TIMEOUT_WAIT_FOREVER  (UINT32_MAX)

#define OSAL_TIMEOUT_CONTROL_XFER  OSAL_TIMEOUT_WAIT_FOREVER
//--------------------------------------------------------------------+
// TASK API
//--------------------------------------------------------------------+
static inline void osal_task_delay(uint32_t msec)
{
  vTaskDelay( pdMS_TO_TICKS(msec) );
}

//--------------------------------------------------------------------+
// Semaphore API
//--------------------------------------------------------------------+
typedef StaticSemaphore_t osal_semaphore_def_t;
typedef SemaphoreHandle_t osal_semaphore_t;

static inline osal_semaphore_t osal_semaphore_create(osal_semaphore_def_t* semdef)
{
  return xSemaphoreCreateBinaryStatic(semdef);
}

static inline bool osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr)
{
  if ( !in_isr )
  {
    return xSemaphoreGive(sem_hdl) != 0;
  }
  else
  {
    BaseType_t xHigherPriorityTaskWoken;
    BaseType_t res = xSemaphoreGiveFromISR(sem_hdl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return res != 0;
  }
}

static inline bool osal_semaphore_wait (osal_semaphore_t sem_hdl, uint32_t msec)
{
  uint32_t const ticks = (msec == OSAL_TIMEOUT_WAIT_FOREVER) ? portMAX_DELAY : pdMS_TO_TICKS(msec);
  return xSemaphoreTake(sem_hdl, ticks);
}

static inline void osal_semaphore_reset(osal_semaphore_t const sem_hdl)
{
  xQueueReset(sem_hdl);
}

//--------------------------------------------------------------------+
// MUTEX API (priority inheritance)
//--------------------------------------------------------------------+
typedef StaticSemaphore_t osal_mutex_def_t;
typedef SemaphoreHandle_t osal_mutex_t;

static inline osal_mutex_t osal_mutex_create(osal_mutex_def_t* mdef)
{
  return xSemaphoreCreateMutexStatic(mdef);
}

static inline bool osal_mutex_lock (osal_mutex_t mutex_hdl, uint32_t msec)
{
  return osal_semaphore_wait(mutex_hdl, msec);
}

static inline bool osal_mutex_unlock(osal_mutex_t mutex_hdl)
{
  return xSemaphoreGive(mutex_hdl);
}

//--------------------------------------------------------------------+
// QUEUE API
//--------------------------------------------------------------------+

// role device/host is used by OS NONE for mutex (disable usb isr) only
#define OSAL_QUEUE_DEF(_role, _name, _depth, _type) \
  static __attribute__((section(".DTCM"))) _type _name##_##buf[_depth];\
  osal_queue_def_t _name = { .depth = _depth, .item_sz = sizeof(_type), .buf = _name##_##buf };

typedef struct
{
  uint16_t depth;
  uint16_t item_sz;
  void*    buf;

  StaticQueue_t sq;
}osal_queue_def_t;

typedef QueueHandle_t osal_queue_t;

static inline osal_queue_t osal_queue_create(osal_queue_def_t* qdef)
{
  return xQueueCreateStatic(qdef->depth, qdef->item_sz, (uint8_t*) qdef->buf, &qdef->sq);
}

static inline bool osal_queue_receive(osal_queue_t qhdl, void* data)
{
  return xQueueReceive(qhdl, data, portMAX_DELAY);
}

static inline bool osal_queue_send(osal_queue_t qhdl, void const * data, bool in_isr)
{
  if ( !in_isr )
  {
    return xQueueSendToBack(qhdl, data, OSAL_TIMEOUT_WAIT_FOREVER) != 0;
  }
  else
  {
    BaseType_t xHigherPriorityTaskWoken;
    BaseType_t res = xQueueSendToBackFromISR(qhdl, data, &xHigherPriorityTaskWoken);
		
#if (CFG_TUSB_MCU==OPT_MCU_ESP32S2)
    if ( xHigherPriorityTaskWoken ) 
#else
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif 
    return res != 0;
  }
}

static inline bool osal_queue_empty(osal_queue_t qhdl)
{
  return uxQueueMessagesWaiting(qhdl) == 0;
}
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if osal_h
typedef void (*osal_task_func_t)( void * );
//--------------------------------------------------------------------+
// OSAL Porting API
//--------------------------------------------------------------------+
static inline void osal_task_delay(uint32_t msec);

//------------- Semaphore -------------//
static inline osal_semaphore_t osal_semaphore_create(osal_semaphore_def_t* semdef);
static inline bool osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr);
static inline bool osal_semaphore_wait(osal_semaphore_t sem_hdl, uint32_t msec);

static inline void osal_semaphore_reset(osal_semaphore_t sem_hdl); // TODO removed

//------------- Mutex -------------//
static inline osal_mutex_t osal_mutex_create(osal_mutex_def_t* mdef);
static inline bool osal_mutex_lock (osal_mutex_t sem_hdl, uint32_t msec);
static inline bool osal_mutex_unlock(osal_mutex_t mutex_hdl);

//------------- Queue -------------//
static inline osal_queue_t osal_queue_create(osal_queue_def_t* qdef);
static inline bool osal_queue_receive(osal_queue_t qhdl, void* data);
static inline bool osal_queue_send(osal_queue_t qhdl, void const * data, bool in_isr);
static inline bool osal_queue_empty(osal_queue_t qhdl);

#if 0  // TODO remove subtask related macros later
// Sub Task
#define OSAL_SUBTASK_BEGIN
#define OSAL_SUBTASK_END                    return TUSB_ERROR_NONE;

#define STASK_RETURN(_error)                return _error;
#define STASK_INVOKE(_subtask, _status)     (_status) = _subtask
#define STASK_ASSERT(_cond)                 TU_VERIFY(_cond, TUSB_ERROR_OSAL_TASK_FAILED)
#endif

#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_timeout_h
typedef struct {
  uint32_t start;
  uint32_t interval;
}tu_timeout_t;
#if 0
extern uint32_t tusb_hal_millis(void);
static inline void tu_timeout_set(tu_timeout_t* tt, uint32_t msec)
{
  tt->interval = msec;
  tt->start    = tusb_hal_millis();
}
static inline bool tu_timeout_expired(tu_timeout_t* tt)
{
  return ( tusb_hal_millis() - tt->start ) >= tt->interval;
}
// For used with periodic event to prevent drift
static inline void tu_timeout_reset(tu_timeout_t* tt)
{
  tt->start += tt->interval;
}
static inline void tu_timeout_restart(tu_timeout_t* tt)
{
  tt->start = tusb_hal_millis();
}
#endif
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_compiler_h
#define TU_STRING(x)      #x              ///< stringify without expand
#define TU_XSTRING(x)     TU_STRING(x)    ///< expand then stringify
#define TU_STRCAT(a, b)   a##b            ///< concat without expand
#define TU_XSTRCAT(a, b)  TU_STRCAT(a, b) ///< expand then concat

#if defined __COUNTER__ && __COUNTER__ != __COUNTER__
  #define _TU_COUNTER_ __COUNTER__
#else
  #define _TU_COUNTER_ __LINE__
#endif

// Compile-time Assert
#if __STDC_VERSION__ >= 201112L
  #define TU_VERIFY_STATIC   _Static_assert
#else
  #define TU_VERIFY_STATIC(const_expr, _mess) enum { TU_XSTRCAT(_verify_static_, _TU_COUNTER_) = 1/(!!(const_expr)) }
#endif

// for declaration of reserved field, make use of _TU_COUNTER_
#define TU_RESERVED           TU_XSTRCAT(reserved, _TU_COUNTER_)

#define TU_LITTLE_ENDIAN (0x12u)
#define TU_BIG_ENDIAN (0x21u)

//--------------------------------------------------------------------+
// Compiler porting with Attribute and Endian
//--------------------------------------------------------------------+
#if defined(__GNUC__)
  #define TU_ATTR_ALIGNED(Bytes)        __attribute__ ((aligned(Bytes)))
  #define TU_ATTR_SECTION(sec_name)     __attribute__ ((section(#sec_name)))
  #define TU_ATTR_PACKED                __attribute__ ((packed))
  #define TU_ATTR_PREPACKED
  #define TU_ATTR_WEAK                  __attribute__ ((weak))
  #define TU_ATTR_DEPRECATED(mess)      __attribute__ ((deprecated(mess))) // warn if function with this attribute is used
  #define TU_ATTR_UNUSED                __attribute__ ((unused))           // Function/Variable is meant to be possibly unused
  #define TU_ATTR_USED                  __attribute__ ((used))             // Function/Variable is meant to be used

  // Endian conversion use well-known host to network (big endian) naming
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    #define TU_BYTE_ORDER TU_LITTLE_ENDIAN
  #else
    #define TU_BYTE_ORDER TU_BIG_ENDIAN
  #endif

  #define TU_BSWAP16(u16) (__builtin_bswap16(u16))
  #define TU_BSWAP32(u32) (__builtin_bswap32(u32))

#elif defined(__TI_COMPILER_VERSION__)
  #define TU_ATTR_ALIGNED(Bytes)        __attribute__ ((aligned(Bytes)))
  #define TU_ATTR_SECTION(sec_name)     __attribute__ ((section(#sec_name)))
  #define TU_ATTR_PACKED                __attribute__ ((packed))
  #define TU_ATTR_PREPACKED
  #define TU_ATTR_WEAK                  __attribute__ ((weak))
  #define TU_ATTR_DEPRECATED(mess)      __attribute__ ((deprecated(mess))) // warn if function with this attribute is used
  #define TU_ATTR_UNUSED                __attribute__ ((unused))           // Function/Variable is meant to be possibly unused
  #define TU_ATTR_USED                  __attribute__ ((used))

  // __BYTE_ORDER is defined in the TI ARM compiler, but not MSP430 (which is little endian)
  #if ((__BYTE_ORDER__) == (__ORDER_LITTLE_ENDIAN__)) || defined(__MSP430__)
    #define TU_BYTE_ORDER TU_LITTLE_ENDIAN
  #else
    #define TU_BYTE_ORDER TU_BIG_ENDIAN
  #endif

  #define TU_BSWAP16(u16) (__builtin_bswap16(u16))
  #define TU_BSWAP32(u32) (__builtin_bswap32(u32))

#elif defined(__ICCARM__)
  #define TU_ATTR_ALIGNED(Bytes)        __attribute__ ((aligned(Bytes)))
  #define TU_ATTR_SECTION(sec_name)     __attribute__ ((section(#sec_name)))
  #define TU_ATTR_PACKED                __attribute__ ((packed))
  #define TU_ATTR_PREPACKED
  #define TU_ATTR_WEAK                  __attribute__ ((weak))
  #define TU_ATTR_DEPRECATED(mess)      __attribute__ ((deprecated(mess))) // warn if function with this attribute is used
  #define TU_ATTR_UNUSED                __attribute__ ((unused))           // Function/Variable is meant to be possibly unused
  #define TU_ATTR_USED                  __attribute__ ((used))             // Function/Variable is meant to be used

  // Endian conversion use well-known host to network (big endian) naming
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    #define TU_BYTE_ORDER TU_LITTLE_ENDIAN
  #else
    #define TU_BYTE_ORDER TU_BIG_ENDIAN
  #endif

  #define TU_BSWAP16(u16) (__iar_builtin_REV16(u16))
  #define TU_BSWAP32(u32) (__iar_builtin_REV(u32))
#else 
  #error "Compiler attribute porting is required"
#endif

#if (TU_BYTE_ORDER == TU_LITTLE_ENDIAN)

  #define tu_htons(u16)  (TU_BSWAP16(u16))
  #define tu_ntohs(u16)  (TU_BSWAP16(u16))

  #define tu_htonl(u32)  (TU_BSWAP32(u32))
  #define tu_ntohl(u32)  (TU_BSWAP32(u32))

  #define tu_htole16(u16) (u16)
  #define tu_le16toh(u16) (u16)

  #define tu_htole32(u32) (u32)
  #define tu_le32toh(u32) (u32)

#elif (TU_BYTE_ORDER == TU_BIG_ENDIAN)

  #define tu_htons(u16)  (u16)
  #define tu_ntohs(u16)  (u16)

  #define tu_htonl(u32)  (u32)
  #define tu_ntohl(u32)  (u32)

  #define tu_htole16(u16) (tu_bswap16(u16))
  #define tu_le16toh(u16) (tu_bswap16(u16))

  #define tu_htole32(u32) (tu_bswap32(u32))
  #define tu_le32toh(u32) (tu_bswap32(u32))

#else
  #error Byte order is undefined
#endif
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_h	
// Initialize device/host stack
// Note: when using with RTOS, this should be called after scheduler/kernel is started.
// Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
bool tusb_init(void);
// Check if stack is initialized
bool tusb_inited(void);	  
#endif	 
/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_common_h

//--------------------------------------------------------------------+
// Macros Helper
//--------------------------------------------------------------------+
#define TU_ARRAY_SIZE(_arr)   ( sizeof(_arr) / sizeof(_arr[0]) )
#define TU_MIN(_x, _y)        ( ( (_x) < (_y) ) ? (_x) : (_y) )
#define TU_MAX(_x, _y)        ( ( (_x) > (_y) ) ? (_x) : (_y) )

#define TU_U16_HIGH(u16)      ((uint8_t) (((u16) >> 8) & 0x00ff))
#define TU_U16_LOW(u16)       ((uint8_t) ((u16)       & 0x00ff))
#define U16_TO_U8S_BE(u16)    TU_U16_HIGH(u16), TU_U16_LOW(u16)
#define U16_TO_U8S_LE(u16)    TU_U16_LOW(u16), TU_U16_HIGH(u16)

#define U32_B1_U8(u32)        ((uint8_t) (((u32) >> 24) & 0x000000ff)) // MSB
#define U32_B2_U8(u32)        ((uint8_t) (((u32) >> 16) & 0x000000ff))
#define U32_B3_U8(u32)        ((uint8_t) (((u32) >>  8) & 0x000000ff))
#define U32_B4_U8(u32)        ((uint8_t) ((u32)         & 0x000000ff)) // LSB

#define U32_TO_U8S_BE(u32)    U32_B1_U8(u32), U32_B2_U8(u32), U32_B3_U8(u32), U32_B4_U8(u32)
#define U32_TO_U8S_LE(u32)    U32_B4_U8(u32), U32_B3_U8(u32), U32_B2_U8(u32), U32_B1_U8(u32)

#define TU_BIT(n)             (1U << (n))

//--------------------------------------------------------------------+
// Includes
//--------------------------------------------------------------------+

// Standard Headers
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

//--------------------------------------------------------------------+
// Inline Functions
//--------------------------------------------------------------------+
#define tu_memclr(buffer, size)  memset((buffer), 0, (size))
#define tu_varclr(_var)          tu_memclr(_var, sizeof(*(_var)))

static inline uint32_t tu_u32(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
  return ( ((uint32_t) b1) << 24) + ( ((uint32_t) b2) << 16) + ( ((uint32_t) b3) << 8) + b4;
}

static inline uint16_t tu_u16(uint8_t high, uint8_t low)
{
  return (uint16_t)((((uint16_t) high) << 8) + low);
}

static inline uint8_t tu_u16_high(uint16_t u16) { return (uint8_t) (((uint16_t) (u16 >> 8)) & 0x00ff); }
static inline uint8_t tu_u16_low (uint16_t u16) { return (uint8_t) (u16 & 0x00ff); }

// Min
static inline uint8_t  tu_min8  (uint8_t  x, uint8_t y ) { return (x < y) ? x : y; }
static inline uint16_t tu_min16 (uint16_t x, uint16_t y) { return (x < y) ? x : y; }
static inline uint32_t tu_min32 (uint32_t x, uint32_t y) { return (x < y) ? x : y; }

// Max
static inline uint8_t  tu_max8  (uint8_t  x, uint8_t y ) { return (x > y) ? x : y; }
static inline uint16_t tu_max16 (uint16_t x, uint16_t y) { return (x > y) ? x : y; }
static inline uint32_t tu_max32 (uint32_t x, uint32_t y) { return (x > y) ? x : y; }

// Align
static inline uint32_t tu_align(uint32_t value, uint32_t alignment)
{
  return value & ((uint32_t) ~(alignment-1));
}

static inline uint32_t tu_align32 (uint32_t value) { return (value & 0xFFFFFFE0UL); }
static inline uint32_t tu_align16 (uint32_t value) { return (value & 0xFFFFFFF0UL); }
static inline uint32_t tu_align4k (uint32_t value) { return (value & 0xFFFFF000UL); }
static inline uint32_t tu_offset4k(uint32_t value) { return (value & 0xFFFUL); }

//------------- Mathematics -------------//
static inline uint32_t tu_abs(int32_t value) { return (uint32_t)((value < 0) ? (-value) : value); }

/// inclusive range checking
static inline bool tu_within(uint32_t lower, uint32_t value, uint32_t upper)
{
  return (lower <= value) && (value <= upper);
}

// log2 of a value is its MSB's position
// TODO use clz TODO remove
static inline uint8_t tu_log2(uint32_t value)
{
  uint8_t result = 0;

  while (value >>= 1)
  {
    result++;
  }
  return result;
}

// Bit
static inline uint32_t tu_bit_set  (uint32_t value, uint8_t pos) { return value | TU_BIT(pos);                  }
static inline uint32_t tu_bit_clear(uint32_t value, uint8_t pos) { return value & (~TU_BIT(pos));               }
static inline bool     tu_bit_test (uint32_t value, uint8_t pos) { return (value & TU_BIT(pos)) ? true : false; }

/*------------------------------------------------------------------*/
/* Count number of arguments of __VA_ARGS__
 * - reference https://groups.google.com/forum/#!topic/comp.std.c/d-6Mj5Lko_s
 * - _GET_NTH_ARG() takes args >= N (64) but only expand to Nth one (64th)
 * - _RSEQ_N() is reverse sequential to N to add padding to have
 * Nth position is the same as the number of arguments
 * - ##__VA_ARGS__ is used to deal with 0 paramerter (swallows comma)
 *------------------------------------------------------------------*/
#ifndef TU_ARGS_NUM

#define TU_ARGS_NUM(...) 	 NARG_(_0, ##__VA_ARGS__,_RSEQ_N())

#define NARG_(...)        _GET_NTH_ARG(__VA_ARGS__)
#define _GET_NTH_ARG( \
          _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
         _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
         _21,_22,_23,_24,_25,_26,_27,_28,_29,_30, \
         _31,_32,_33,_34,_35,_36,_37,_38,_39,_40, \
         _41,_42,_43,_44,_45,_46,_47,_48,_49,_50, \
         _51,_52,_53,_54,_55,_56,_57,_58,_59,_60, \
         _61,_62,_63,N,...) N
#define _RSEQ_N() \
         62,61,60,                      \
         59,58,57,56,55,54,53,52,51,50, \
         49,48,47,46,45,44,43,42,41,40, \
         39,38,37,36,35,34,33,32,31,30, \
         29,28,27,26,25,24,23,22,21,20, \
         19,18,17,16,15,14,13,12,11,10, \
         9,8,7,6,5,4,3,2,1,0
#endif

// To be removed
//------------- Binary constant -------------//
#if defined(__GNUC__) && !defined(__CC_ARM)

#define TU_BIN8(x)               ((uint8_t)  (0b##x))
#define TU_BIN16(b1, b2)         ((uint16_t) (0b##b1##b2))
#define TU_BIN32(b1, b2, b3, b4) ((uint32_t) (0b##b1##b2##b3##b4))

#else

//  internal macro of B8, B16, B32
#define _B8__(x) (((x&0x0000000FUL)?1:0) \
                +((x&0x000000F0UL)?2:0) \
                +((x&0x00000F00UL)?4:0) \
                +((x&0x0000F000UL)?8:0) \
                +((x&0x000F0000UL)?16:0) \
                +((x&0x00F00000UL)?32:0) \
                +((x&0x0F000000UL)?64:0) \
                +((x&0xF0000000UL)?128:0))

#define TU_BIN8(d) ((uint8_t) _B8__(0x##d##UL))
#define TU_BIN16(dmsb,dlsb) (((uint16_t)TU_BIN8(dmsb)<<8) + TU_BIN8(dlsb))
#define TU_BIN32(dmsb,db2,db3,dlsb) \
            (((uint32_t)TU_BIN8(dmsb)<<24) \
            + ((uint32_t)TU_BIN8(db2)<<16) \
            + ((uint32_t)TU_BIN8(db3)<<8) \
            + TU_BIN8(dlsb))
#endif

//--------------------------------------------------------------------+
// Debug Function
//--------------------------------------------------------------------+

// CFG_TUSB_DEBUG for debugging
// 0 : no debug
// 1 : print when there is error
// 2 : print out log
#if CFG_TUSB_DEBUG

void tu_print_mem(void const *buf, uint16_t count, uint8_t indent);

#ifdef CFG_TUSB_DEBUG_PRINTF
  extern int CFG_TUSB_DEBUG_PRINTF(const char *format, ...);
  #define tu_printf    CFG_TUSB_DEBUG_PRINTF
#else
  #define tu_printf    printf
#endif

static inline
void tu_print_var(uint8_t const* buf, uint32_t bufsize)
{
  for(uint32_t i=0; i<bufsize; i++) tu_printf("%02X ", buf[i]);
}

// Log with debug level 1
#define TU_LOG1               tu_printf
#define TU_LOG1_MEM           tu_print_mem
#define TU_LOG1_VAR(_x)       tu_print_var((uint8_t const*)(_x), sizeof(*(_x)))
#define TU_LOG1_INT(_x)       tu_printf(#_x " = %ld\n", (uint32_t) (_x) )
#define TU_LOG1_HEX(_x)       tu_printf(#_x " = %lX\n", (uint32_t) (_x) )
#define TU_LOG1_LOCATION()    tu_printf("%s: %d:\r\n", __PRETTY_FUNCTION__, __LINE__)
#define TU_LOG1_FAILED()      tu_printf("%s: %d: Failed\r\n", __PRETTY_FUNCTION__, __LINE__)

// Log with debug level 2
#if CFG_TUSB_DEBUG > 1
  #define TU_LOG2             TU_LOG1
  #define TU_LOG2_MEM         TU_LOG1_MEM
  #define TU_LOG2_VAR         TU_LOG1_VAR
  #define TU_LOG2_INT         TU_LOG1_INT
  #define TU_LOG2_HEX         TU_LOG1_HEX
  #define TU_LOG2_LOCATION()  TU_LOG1_LOCATION()
#endif


typedef struct
{
  uint32_t key;
  const char* data;
} tu_lookup_entry_t;

typedef struct
{
  uint16_t count;
  tu_lookup_entry_t const* items;
} tu_lookup_table_t;

static inline const char* tu_lookup_find(tu_lookup_table_t const* p_table, uint32_t key)
{
  for(uint16_t i=0; i<p_table->count; i++)
  {
    if (p_table->items[i].key == key) return p_table->items[i].data;
  }

  return NULL;
}

#endif // CFG_TUSB_DEBUG

#ifndef TU_LOG1
  #define TU_LOG1(...)
  #define TU_LOG1_MEM(...)
  #define TU_LOG1_VAR(...)
  #define TU_LOG1_INT(...)
  #define TU_LOG1_HEX(...)
  #define TU_LOG1_LOCATION()
  #define TU_LOG1_FAILED()
#endif

#ifndef TU_LOG2
  #define TU_LOG2(...)
  #define TU_LOG2_MEM(...)
  #define TU_LOG2_VAR(...)
  #define TU_LOG2_INT(...)
  #define TU_LOG2_HEX(...)
  #define TU_LOG2_LOCATION()
#endif
#endif


/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_types_h

/*------------------------------------------------------------------*/
/* CONSTANTS
 *------------------------------------------------------------------*/

/// defined base on EHCI specs value for Endpoint Speed
typedef enum
{
  TUSB_SPEED_FULL = 0,
  TUSB_SPEED_LOW     ,
  TUSB_SPEED_HIGH
}tusb_speed_t;

/// defined base on USB Specs Endpoint's bmAttributes
typedef enum
{
  TUSB_XFER_CONTROL = 0 ,
  TUSB_XFER_ISOCHRONOUS ,
  TUSB_XFER_BULK        ,
  TUSB_XFER_INTERRUPT
}tusb_xfer_type_t;

typedef enum
{
  TUSB_DIR_OUT = 0,
  TUSB_DIR_IN  = 1,

  TUSB_DIR_IN_MASK = 0x80
}tusb_dir_t;

/// USB Descriptor Types
typedef enum
{
  TUSB_DESC_DEVICE                = 0x01,
  TUSB_DESC_CONFIGURATION         = 0x02,
  TUSB_DESC_STRING                = 0x03,
  TUSB_DESC_INTERFACE             = 0x04,
  TUSB_DESC_ENDPOINT              = 0x05,
  TUSB_DESC_DEVICE_QUALIFIER      = 0x06,
  TUSB_DESC_OTHER_SPEED_CONFIG    = 0x07,
  TUSB_DESC_INTERFACE_POWER       = 0x08,
  TUSB_DESC_OTG                   = 0x09,
  TUSB_DESC_DEBUG                 = 0x0A,
  TUSB_DESC_INTERFACE_ASSOCIATION = 0x0B,

  TUSB_DESC_BOS                   = 0x0F,
  TUSB_DESC_DEVICE_CAPABILITY     = 0x10,

  TUSB_DESC_FUNCTIONAL            = 0x21,

  // Class Specific Descriptor
  TUSB_DESC_CS_DEVICE             = 0x21,
  TUSB_DESC_CS_CONFIGURATION      = 0x22,
  TUSB_DESC_CS_STRING             = 0x23,
  TUSB_DESC_CS_INTERFACE          = 0x24,
  TUSB_DESC_CS_ENDPOINT           = 0x25,

  TUSB_DESC_SUPERSPEED_ENDPOINT_COMPANION     = 0x30,
  TUSB_DESC_SUPERSPEED_ISO_ENDPOINT_COMPANION = 0x31
}tusb_desc_type_t;

typedef enum
{
  TUSB_REQ_GET_STATUS        = 0  ,
  TUSB_REQ_CLEAR_FEATURE     = 1  ,
  TUSB_REQ_RESERVED          = 2  ,
  TUSB_REQ_SET_FEATURE       = 3  ,
  TUSB_REQ_RESERVED2         = 4  ,
  TUSB_REQ_SET_ADDRESS       = 5  ,
  TUSB_REQ_GET_DESCRIPTOR    = 6  ,
  TUSB_REQ_SET_DESCRIPTOR    = 7  ,
  TUSB_REQ_GET_CONFIGURATION = 8  ,
  TUSB_REQ_SET_CONFIGURATION = 9  ,
  TUSB_REQ_GET_INTERFACE     = 10 ,
  TUSB_REQ_SET_INTERFACE     = 11 ,
  TUSB_REQ_SYNCH_FRAME       = 12
}tusb_request_code_t;

typedef enum
{
  TUSB_REQ_FEATURE_EDPT_HALT     = 0,
  TUSB_REQ_FEATURE_REMOTE_WAKEUP = 1,
  TUSB_REQ_FEATURE_TEST_MODE     = 2
}tusb_request_feature_selector_t;

typedef enum
{
  TUSB_REQ_TYPE_STANDARD = 0,
  TUSB_REQ_TYPE_CLASS,
  TUSB_REQ_TYPE_VENDOR,
  TUSB_REQ_TYPE_INVALID
} tusb_request_type_t;

typedef enum
{
  TUSB_REQ_RCPT_DEVICE =0,
  TUSB_REQ_RCPT_INTERFACE,
  TUSB_REQ_RCPT_ENDPOINT,
  TUSB_REQ_RCPT_OTHER
} tusb_request_recipient_t;

// https://www.usb.org/defined-class-codes
typedef enum
{
  TUSB_CLASS_UNSPECIFIED          = 0    ,
  TUSB_CLASS_AUDIO                = 1    ,
  TUSB_CLASS_CDC                  = 2    ,
  TUSB_CLASS_HID                  = 3    ,
  TUSB_CLASS_RESERVED_4           = 4    ,
  TUSB_CLASS_PHYSICAL             = 5    ,
  TUSB_CLASS_IMAGE                = 6    ,
  TUSB_CLASS_PRINTER              = 7    ,
  TUSB_CLASS_MSC                  = 8    ,
  TUSB_CLASS_HUB                  = 9    ,
  TUSB_CLASS_CDC_DATA             = 10   ,
  TUSB_CLASS_SMART_CARD           = 11   ,
  TUSB_CLASS_RESERVED_12          = 12   ,
  TUSB_CLASS_CONTENT_SECURITY     = 13   ,
  TUSB_CLASS_VIDEO                = 14   ,
  TUSB_CLASS_PERSONAL_HEALTHCARE  = 15   ,
  TUSB_CLASS_AUDIO_VIDEO          = 16   ,

  TUSB_CLASS_DIAGNOSTIC           = 0xDC ,
  TUSB_CLASS_WIRELESS_CONTROLLER  = 0xE0 ,
  TUSB_CLASS_MISC                 = 0xEF ,
  TUSB_CLASS_APPLICATION_SPECIFIC = 0xFE ,
  TUSB_CLASS_VENDOR_SPECIFIC      = 0xFF
}tusb_class_code_t;

typedef enum
{
  MISC_SUBCLASS_COMMON = 2
}misc_subclass_type_t;

typedef enum
{
  MISC_PROTOCOL_IAD = 1
}misc_protocol_type_t;

typedef enum
{
  APP_SUBCLASS_USBTMC = 0x03,
  APP_SUBCLASS_DFU_RUNTIME = 0x01
} app_subclass_type_t;

typedef enum
{
  DEVICE_CAPABILITY_WIRELESS_USB               = 0x01,
  DEVICE_CAPABILITY_USB20_EXTENSION            = 0x02,
  DEVICE_CAPABILITY_SUPERSPEED_USB             = 0x03,
  DEVICE_CAPABILITY_CONTAINER_id               = 0x04,
  DEVICE_CAPABILITY_PLATFORM                   = 0x05,
  DEVICE_CAPABILITY_POWER_DELIVERY             = 0x06,
  DEVICE_CAPABILITY_BATTERY_INFO               = 0x07,
  DEVICE_CAPABILITY_PD_CONSUMER_PORT           = 0x08,
  DEVICE_CAPABILITY_PD_PROVIDER_PORT           = 0x09,
  DEVICE_CAPABILITY_SUPERSPEED_PLUS            = 0x0A,
  DEVICE_CAPABILITY_PRECESION_TIME_MEASUREMENT = 0x0B,
  DEVICE_CAPABILITY_WIRELESS_USB_EXT           = 0x0C,
  DEVICE_CAPABILITY_BILLBOARD                  = 0x0D,
  DEVICE_CAPABILITY_AUTHENTICATION             = 0x0E,
  DEVICE_CAPABILITY_BILLBOARD_EX               = 0x0F,
  DEVICE_CAPABILITY_CONFIGURATION_SUMMARY      = 0x10
}device_capability_type_t;

enum {
  TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP = TU_BIT(5),
  TUSB_DESC_CONFIG_ATT_SELF_POWERED  = TU_BIT(6),
};

#define TUSB_DESC_CONFIG_POWER_MA(x)  ((x)/2)

/// Device State TODO remove
typedef enum
{
  TUSB_DEVICE_STATE_UNPLUG = 0  ,
  TUSB_DEVICE_STATE_CONFIGURED  ,
  TUSB_DEVICE_STATE_SUSPENDED   ,
}tusb_device_state_t;

typedef enum
{
  XFER_RESULT_SUCCESS,
  XFER_RESULT_FAILED,
  XFER_RESULT_STALLED,
}xfer_result_t;

enum // TODO remove
{
  DESC_OFFSET_LEN  = 0,
  DESC_OFFSET_TYPE = 1
};

enum
{
  INTERFACE_INVALID_NUMBER = 0xff
};


typedef enum
{
  MS_OS_20_SET_HEADER_DESCRIPTOR       = 0x00,
  MS_OS_20_SUBSET_HEADER_CONFIGURATION = 0x01,
  MS_OS_20_SUBSET_HEADER_FUNCTION      = 0x02,
  MS_OS_20_FEATURE_COMPATBLE_ID        = 0x03,
  MS_OS_20_FEATURE_REG_PROPERTY        = 0x04,
  MS_OS_20_FEATURE_MIN_RESUME_TIME     = 0x05,
  MS_OS_20_FEATURE_MODEL_ID            = 0x06,
  MS_OS_20_FEATURE_CCGP_DEVICE         = 0x07,
  MS_OS_20_FEATURE_VENDOR_REVISION     = 0x08
} microsoft_os_20_type_t;

//--------------------------------------------------------------------+
// USB Descriptors
//--------------------------------------------------------------------+

/// USB Device Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength            ; ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType    ; ///< DEVICE Descriptor Type.
  uint16_t bcdUSB             ; ///< BUSB Specification Release Number in Binary-Coded Decimal (i.e., 2.10 is 210H). This field identifies the release of the USB Specification with which the device and its descriptors are compliant.

  uint8_t  bDeviceClass       ; ///< Class code (assigned by the USB-IF). \li If this field is reset to zero, each interface within a configuration specifies its own class information and the various interfaces operate independently. \li If this field is set to a value between 1 and FEH, the device supports different class specifications on different interfaces and the interfaces may not operate independently. This value identifies the class definition used for the aggregate interfaces. \li If this field is set to FFH, the device class is vendor-specific.
  uint8_t  bDeviceSubClass    ; ///< Subclass code (assigned by the USB-IF). These codes are qualified by the value of the bDeviceClass field. \li If the bDeviceClass field is reset to zero, this field must also be reset to zero. \li If the bDeviceClass field is not set to FFH, all values are reserved for assignment by the USB-IF.
  uint8_t  bDeviceProtocol    ; ///< Protocol code (assigned by the USB-IF). These codes are qualified by the value of the bDeviceClass and the bDeviceSubClass fields. If a device supports class-specific protocols on a device basis as opposed to an interface basis, this code identifies the protocols that the device uses as defined by the specification of the device class. \li If this field is reset to zero, the device does not use class-specific protocols on a device basis. However, it may use classspecific protocols on an interface basis. \li If this field is set to FFH, the device uses a vendor-specific protocol on a device basis.
  uint8_t  bMaxPacketSize0    ; ///< Maximum packet size for endpoint zero (only 8, 16, 32, or 64 are valid). For HS devices is fixed to 64.

  uint16_t idVendor           ; ///< Vendor ID (assigned by the USB-IF).
  uint16_t idProduct          ; ///< Product ID (assigned by the manufacturer).
  uint16_t bcdDevice          ; ///< Device release number in binary-coded decimal.
  uint8_t  iManufacturer      ; ///< Index of string descriptor describing manufacturer.
  uint8_t  iProduct           ; ///< Index of string descriptor describing product.
  uint8_t  iSerialNumber      ; ///< Index of string descriptor describing the device's serial number.

  uint8_t  bNumConfigurations ; ///< Number of possible configurations.
} tusb_desc_device_t;

// USB Binary Device Object Store (BOS) Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength         ; ///< Size of this descriptor in bytes
  uint8_t  bDescriptorType ; ///< CONFIGURATION Descriptor Type
  uint16_t wTotalLength    ; ///< Total length of data returned for this descriptor
  uint8_t  bNumDeviceCaps  ; ///< Number of device capability descriptors in the BOS
} tusb_desc_bos_t;

/// USB Configuration Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength             ; ///< Size of this descriptor in bytes
  uint8_t  bDescriptorType     ; ///< CONFIGURATION Descriptor Type
  uint16_t wTotalLength        ; ///< Total length of data returned for this configuration. Includes the combined length of all descriptors (configuration, interface, endpoint, and class- or vendor-specific) returned for this configuration.

  uint8_t  bNumInterfaces      ; ///< Number of interfaces supported by this configuration
  uint8_t  bConfigurationValue ; ///< Value to use as an argument to the SetConfiguration() request to select this configuration.
  uint8_t  iConfiguration      ; ///< Index of string descriptor describing this configuration
  uint8_t  bmAttributes        ; ///< Configuration characteristics \n D7: Reserved (set to one)\n D6: Self-powered \n D5: Remote Wakeup \n D4...0: Reserved (reset to zero) \n D7 is reserved and must be set to one for historical reasons. \n A device configuration that uses power from the bus and a local source reports a non-zero value in bMaxPower to indicate the amount of bus power required and sets D6. The actual power source at runtime may be determined using the GetStatus(DEVICE) request (see USB 2.0 spec Section 9.4.5). \n If a device configuration supports remote wakeup, D5 is set to one.
  uint8_t  bMaxPower           ; ///< Maximum power consumption of the USB device from the bus in this specific configuration when the device is fully operational. Expressed in 2 mA units (i.e., 50 = 100 mA).
} tusb_desc_configuration_t;

/// USB Interface Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength            ; ///< Size of this descriptor in bytes
  uint8_t  bDescriptorType    ; ///< INTERFACE Descriptor Type

  uint8_t  bInterfaceNumber   ; ///< Number of this interface. Zero-based value identifying the index in the array of concurrent interfaces supported by this configuration.
  uint8_t  bAlternateSetting  ; ///< Value used to select this alternate setting for the interface identified in the prior field
  uint8_t  bNumEndpoints      ; ///< Number of endpoints used by this interface (excluding endpoint zero). If this value is zero, this interface only uses the Default Control Pipe.
  uint8_t  bInterfaceClass    ; ///< Class code (assigned by the USB-IF). \li A value of zero is reserved for future standardization. \li If this field is set to FFH, the interface class is vendor-specific. \li All other values are reserved for assignment by the USB-IF.
  uint8_t  bInterfaceSubClass ; ///< Subclass code (assigned by the USB-IF). \n These codes are qualified by the value of the bInterfaceClass field. \li If the bInterfaceClass field is reset to zero, this field must also be reset to zero. \li If the bInterfaceClass field is not set to FFH, all values are reserved for assignment by the USB-IF.
  uint8_t  bInterfaceProtocol ; ///< Protocol code (assigned by the USB). \n These codes are qualified by the value of the bInterfaceClass and the bInterfaceSubClass fields. If an interface supports class-specific requests, this code identifies the protocols that the device uses as defined by the specification of the device class. \li If this field is reset to zero, the device does not use a class-specific protocol on this interface. \li If this field is set to FFH, the device uses a vendor-specific protocol for this interface.
  uint8_t  iInterface         ; ///< Index of string descriptor describing this interface
} tusb_desc_interface_t;

/// USB Endpoint Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength          ; ///< Size of this descriptor in bytes
  uint8_t  bDescriptorType  ; ///< ENDPOINT Descriptor Type

  uint8_t  bEndpointAddress ; ///< The address of the endpoint on the USB device described by this descriptor. The address is encoded as follows: \n Bit 3...0: The endpoint number \n Bit 6...4: Reserved, reset to zero \n Bit 7: Direction, ignored for control endpoints 0 = OUT endpoint 1 = IN endpoint.

  struct TU_ATTR_PACKED {
    uint8_t xfer  : 2;
    uint8_t sync  : 2;
    uint8_t usage : 2;
    uint8_t       : 2;
  } bmAttributes     ; ///< This field describes the endpoint's attributes when it is configured using the bConfigurationValue. \n Bits 1..0: Transfer Type \n- 00 = Control \n- 01 = Isochronous \n- 10 = Bulk \n- 11 = Interrupt \n If not an isochronous endpoint, bits 5..2 are reserved and must be set to zero. If isochronous, they are defined as follows: \n Bits 3..2: Synchronization Type \n- 00 = No Synchronization \n- 01 = Asynchronous \n- 10 = Adaptive \n- 11 = Synchronous \n Bits 5..4: Usage Type \n- 00 = Data endpoint \n- 01 = Feedback endpoint \n- 10 = Implicit feedback Data endpoint \n- 11 = Reserved \n Refer to Chapter 5 of USB 2.0 specification for more information. \n All other bits are reserved and must be reset to zero. Reserved bits must be ignored by the host.

  struct TU_ATTR_PACKED {
    uint16_t size           : 11; ///< Maximum packet size this endpoint is capable of sending or receiving when this configuration is selected. \n For isochronous endpoints, this value is used to reserve the bus time in the schedule, required for the per-(micro)frame data payloads. The pipe may, on an ongoing basis, actually use less bandwidth than that reserved. The device reports, if necessary, the actual bandwidth used via its normal, non-USB defined mechanisms. \n For all endpoints, bits 10..0 specify the maximum packet size (in bytes). \n For high-speed isochronous and interrupt endpoints: \n Bits 12..11 specify the number of additional transaction opportunities per microframe: \n- 00 = None (1 transaction per microframe) \n- 01 = 1 additional (2 per microframe) \n- 10 = 2 additional (3 per microframe) \n- 11 = Reserved \n Bits 15..13 are reserved and must be set to zero.
    uint16_t hs_period_mult : 2;
    uint16_t : 0;
  }wMaxPacketSize;

  uint8_t  bInterval        ; ///< Interval for polling endpoint for data transfers. Expressed in frames or microframes depending on the device operating speed (i.e., either 1 millisecond or 125 us units). \n- For full-/high-speed isochronous endpoints, this value must be in the range from 1 to 16. The bInterval value is used as the exponent for a \f$ 2^(bInterval-1) \f$ value; e.g., a bInterval of 4 means a period of 8 (\f$ 2^(4-1) \f$). \n- For full-/low-speed interrupt endpoints, the value of this field may be from 1 to 255. \n- For high-speed interrupt endpoints, the bInterval value is used as the exponent for a \f$ 2^(bInterval-1) \f$ value; e.g., a bInterval of 4 means a period of 8 (\f$ 2^(4-1) \f$) . This value must be from 1 to 16. \n- For high-speed bulk/control OUT endpoints, the bInterval must specify the maximum NAK rate of the endpoint. A value of 0 indicates the endpoint never NAKs. Other values indicate at most 1 NAK each bInterval number of microframes. This value must be in the range from 0 to 255. \n Refer to Chapter 5 of USB 2.0 specification for more information.
} tusb_desc_endpoint_t;

/// USB Other Speed Configuration Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength             ; ///< Size of descriptor
  uint8_t  bDescriptorType     ; ///< Other_speed_Configuration Type
  uint16_t wTotalLength        ; ///< Total length of data returned

  uint8_t  bNumInterfaces      ; ///< Number of interfaces supported by this speed configuration
  uint8_t  bConfigurationValue ; ///< Value to use to select configuration
  uint8_t  IConfiguration      ; ///< Index of string descriptor
  uint8_t  bmAttributes        ; ///< Same as Configuration descriptor
  uint8_t  bMaxPower           ; ///< Same as Configuration descriptor
} tusb_desc_other_speed_t;

/// USB Device Qualifier Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength            ; ///< Size of descriptor
  uint8_t  bDescriptorType    ; ///< Device Qualifier Type
  uint16_t bcdUSB             ; ///< USB specification version number (e.g., 0200H for V2.00)

  uint8_t  bDeviceClass       ; ///< Class Code
  uint8_t  bDeviceSubClass    ; ///< SubClass Code
  uint8_t  bDeviceProtocol    ; ///< Protocol Code
  uint8_t  bMaxPacketSize0    ; ///< Maximum packet size for other speed
  uint8_t  bNumConfigurations ; ///< Number of Other-speed Configurations
  uint8_t  bReserved          ; ///< Reserved for future use, must be zero
} tusb_desc_device_qualifier_t;

/// USB Interface Association Descriptor (IAD ECN)
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength           ; ///< Size of descriptor
  uint8_t bDescriptorType   ; ///< Other_speed_Configuration Type

  uint8_t bFirstInterface   ; ///< Index of the first associated interface.
  uint8_t bInterfaceCount   ; ///< Total number of associated interfaces.

  uint8_t bFunctionClass    ; ///< Interface class ID.
  uint8_t bFunctionSubClass ; ///< Interface subclass ID.
  uint8_t bFunctionProtocol ; ///< Interface protocol ID.

  uint8_t iFunction         ; ///< Index of the string descriptor describing the interface association.
} tusb_desc_interface_assoc_t;

// USB String Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength         ; ///< Size of this descriptor in bytes
  uint8_t  bDescriptorType ; ///< Descriptor Type
  uint16_t unicode_string[];
} tusb_desc_string_t;

// USB Binary Device Object Store (BOS)
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength;
  uint8_t bDescriptorType ;
  uint8_t bDevCapabilityType;
  uint8_t bReserved;
  uint8_t PlatformCapabilityUUID[16];
  uint8_t CapabilityData[];
} tusb_desc_bos_platform_t;

// USB WebuSB URL Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bScheme;
  char    url[];
} tusb_desc_webusb_url_t;

/*------------------------------------------------------------------*/
/* Types
 *------------------------------------------------------------------*/
typedef struct TU_ATTR_PACKED{
  union {
    struct TU_ATTR_PACKED {
      uint8_t recipient :  5; ///< Recipient type tusb_request_recipient_t.
      uint8_t type      :  2; ///< Request type tusb_request_type_t.
      uint8_t direction :  1; ///< Direction type. tusb_dir_t
    } bmRequestType_bit;

    uint8_t bmRequestType;
  };

  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} tusb_control_request_t;

TU_VERIFY_STATIC( sizeof(tusb_control_request_t) == 8, "mostly compiler option issue");

// TODO move to somewhere suitable
static inline uint8_t bm_request_type(uint8_t direction, uint8_t type, uint8_t recipient)
{
  return ((uint8_t) (direction << 7)) | ((uint8_t) (type << 5)) | (recipient);
}

//--------------------------------------------------------------------+
// Endpoint helper
//--------------------------------------------------------------------+

// Get direction from Endpoint address
static inline tusb_dir_t tu_edpt_dir(uint8_t addr)
{
  return (addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
}

// Get Endpoint number from address
static inline uint8_t tu_edpt_number(uint8_t addr)
{
  return (uint8_t)(addr & (~TUSB_DIR_IN_MASK));
}

static inline uint8_t tu_edpt_addr(uint8_t num, uint8_t dir)
{
  return (uint8_t)(num | (dir ? TUSB_DIR_IN_MASK : 0));
}

//--------------------------------------------------------------------+
// Descriptor helper
//--------------------------------------------------------------------+
static inline uint8_t const * tu_desc_next(void const* desc)
{
  uint8_t const* desc8 = (uint8_t const*) desc;
  return desc8 + desc8[DESC_OFFSET_LEN];
}

static inline uint8_t tu_desc_type(void const* desc)
{
  return ((uint8_t const*) desc)[DESC_OFFSET_TYPE];
}

static inline uint8_t tu_desc_len(void const* desc)
{
  return ((uint8_t const*) desc)[DESC_OFFSET_LEN];
}
#endif

/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_config_h	

// RHPort number used for device can be defined by board.mk, default to port 0
#ifndef BOARD_DEVICE_RHPORT_NUM
  #define BOARD_DEVICE_RHPORT_NUM     0
#endif
	 
#define BOARD_DEVICE_RHPORT_SPEED   OPT_MODE_FULL_SPEED

// Device mode with rhport and speed defined by board.mk
#if   BOARD_DEVICE_RHPORT_NUM == 0
  #define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif BOARD_DEVICE_RHPORT_NUM == 1
  #define CFG_TUSB_RHPORT1_MODE     (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
  #error "Incorrect RHPort configuration"
#endif

// This examples use FreeRTOS
#define CFG_TUSB_OS               OPT_OS_FREERTOS

// can be defined by compiler in DEBUG build
#ifndef CFG_TUSB_DEBUG
  #define CFG_TUSB_DEBUG           0
#endif

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION __attribute__ (( section(".AXI_DMA_RAM") ))
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC              1
#define CFG_TUD_MSC              2
#define CFG_TUD_HID              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0

// MSC Buffer size of Device Mass storage
#define CFG_TUD_MSC_EP_BUFSIZE   8192	 
#endif 	 
	 
	
/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_option_h	 

/* group_supported_os Supported RTOS
 * CFG_TUSB_OS must be defined to one of these
 * */
#define OPT_OS_NONE       1  ///< No RTOS
#define OPT_OS_FREERTOS   2  ///< FreeRTOS
#define OPT_OS_MYNEWT     3  ///< Mynewt OS
#define OPT_OS_CUSTOM     4  ///< Custom OS is implemented by application

// Lower 4-bit is operational mode
#define OPT_MODE_NONE         0x00 ///< Disabled
#define OPT_MODE_DEVICE       0x01 ///< Device Mode
#define OPT_MODE_HOST         0x02 ///< Host Mode

//--------------------------------------------------------------------
// RootHub Mode Configuration(USB根集线器)
// CFG_TUSB_RHPORTx_MODE contains operation mode and speed for that port
//--------------------------------------------------------------------
// Higher 4-bit is max operational speed (corresponding to tusb_speed_t)
#define OPT_MODE_FULL_SPEED   0x00 ///< Max Full Speed
#define OPT_MODE_LOW_SPEED    0x10 ///< Max Low Speed
#define OPT_MODE_HIGH_SPEED   0x20 ///< Max High Speed

#ifndef CFG_TUSB_RHPORT0_MODE
  #define CFG_TUSB_RHPORT0_MODE OPT_MODE_NONE
#endif
#ifndef CFG_TUSB_RHPORT1_MODE
  #define CFG_TUSB_RHPORT1_MODE OPT_MODE_NONE
#endif

#if ((CFG_TUSB_RHPORT0_MODE & OPT_MODE_HOST  ) && (CFG_TUSB_RHPORT1_MODE & OPT_MODE_HOST  )) || ((CFG_TUSB_RHPORT0_MODE & OPT_MODE_DEVICE) && (CFG_TUSB_RHPORT1_MODE & OPT_MODE_DEVICE))
  #error "TinyUSB currently does not support same modes on more than 1 roothub port"
#endif

// Which roothub port is configured as host
#define TUH_OPT_RHPORT          ( (CFG_TUSB_RHPORT0_MODE & OPT_MODE_HOST) ? 0 : ((CFG_TUSB_RHPORT1_MODE & OPT_MODE_HOST) ? 1 : -1) )
#define TUSB_OPT_HOST_ENABLED   ( TUH_OPT_RHPORT >= 0 )

// Which roothub port is configured as device
#define TUD_OPT_RHPORT          ( (CFG_TUSB_RHPORT0_MODE & OPT_MODE_DEVICE) ? 0 : ((CFG_TUSB_RHPORT1_MODE & OPT_MODE_DEVICE) ? 1 : -1) )

#if TUD_OPT_RHPORT == 0
#define TUD_OPT_HIGH_SPEED      ( CFG_TUSB_RHPORT0_MODE & OPT_MODE_HIGH_SPEED )
#else
#define TUD_OPT_HIGH_SPEED      ( CFG_TUSB_RHPORT1_MODE & OPT_MODE_HIGH_SPEED )
#endif

#define TUSB_OPT_DEVICE_ENABLED ( TUD_OPT_RHPORT >= 0 )

// Debug enable to print out error message
#ifndef CFG_TUSB_DEBUG
  #define CFG_TUSB_DEBUG 0
#endif

// place data in accessible RAM for usb controller
#ifndef CFG_TUSB_MEM_SECTION
  #define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
  #define CFG_TUSB_MEM_ALIGN      TU_ATTR_ALIGNED(4)
#endif

#ifndef CFG_TUSB_OS
  #define CFG_TUSB_OS             OPT_OS_NONE
#endif

//--------------------------------------------------------------------
// DEVICE OPTIONS
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
  #define CFG_TUD_ENDPOINT0_SIZE  64
#endif

#ifndef CFG_TUD_CDC
  #define CFG_TUD_CDC             0
#endif

#ifndef CFG_TUD_MSC
  #define CFG_TUD_MSC             0
#endif

#ifndef CFG_TUD_HID
  #define CFG_TUD_HID             0
#endif

#ifndef CFG_TUD_MIDI
  #define CFG_TUD_MIDI            0
#endif

#ifndef CFG_TUD_VENDOR
  #define CFG_TUD_VENDOR          0
#endif

#ifndef CFG_TUD_USBTMC
  #define CFG_TUD_USBTMC          0
#endif

#ifndef CFG_TUD_DFU_RT
  #define CFG_TUD_DFU_RT          0
#endif

#ifndef CFG_TUD_NET
  #define CFG_TUD_NET             0
#endif

#ifndef CFG_TUD_BTH
  #define CFG_TUD_BTH             0
#endif

//------------------------------------------------------------------
// Configuration Validation
//------------------------------------------------------------------
#if CFG_TUD_ENDPOINT0_SIZE > 64
  #error Control Endpoint Max Packet Size cannot be larger than 64
#endif
#endif	 
	 
/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_verify_h	 

//--------------------------------------------------------------------+
// TU_VERIFY Helper
//--------------------------------------------------------------------+

#if CFG_TUSB_DEBUG
  #include <stdio.h>
  #define _MESS_ERR(_err)   printf("%s %d: failed, error = %s\r\n", __func__, __LINE__, tusb_strerr[_err])
  #define _MESS_FAILED()    printf("%s %d: assert failed\r\n", __func__, __LINE__)
#else
  #define _MESS_ERR(_err) do {} while (0)
  #define _MESS_FAILED() do {} while (0)
#endif

// Halt CPU (breakpoint) when hitting error, only apply for Cortex M3, M4, M7
#if defined(__ARM_ARCH_7M__) || defined (__ARM_ARCH_7EM__)
  #define TU_BREAKPOINT() do                                                                                \
  {                                                                                                         \
    volatile uint32_t* ARM_CM_DHCSR =  ((volatile uint32_t*) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */ \
    if ( (*ARM_CM_DHCSR) & 1UL ) __asm("BKPT #0\n"); /* Only halt mcu if debugger is attached */            \
  } while(0)

#elif defined(__riscv)
  #define TU_BREAKPOINT() do { __asm("ebreak\n"); } while(0)

#else
  #define TU_BREAKPOINT() do {} while (0)
#endif

/*------------------------------------------------------------------*/
/* Macro Generator
 *------------------------------------------------------------------*/

// Helper to implement optional parameter for TU_VERIFY Macro family
#define GET_3RD_ARG(arg1, arg2, arg3, ...)        arg3
#define GET_4TH_ARG(arg1, arg2, arg3, arg4, ...)  arg4

/*------------- Generator for TU_VERIFY and TU_VERIFY_HDLR -------------*/
#define TU_VERIFY_DEFINE(_cond, _handler, _ret)  do            \
{                                                              \
  if ( !(_cond) ) { _handler; return _ret;  }                  \
} while(0)

/*------------------------------------------------------------------*/
/* TU_VERIFY
 * - TU_VERIFY_1ARGS : return false if failed
 * - TU_VERIFY_2ARGS : return provided value if failed
 *------------------------------------------------------------------*/
#define TU_VERIFY_1ARGS(_cond)                         TU_VERIFY_DEFINE(_cond, , false)
#define TU_VERIFY_2ARGS(_cond, _ret)                   TU_VERIFY_DEFINE(_cond, , _ret)

#define TU_VERIFY(...)                   GET_3RD_ARG(__VA_ARGS__, TU_VERIFY_2ARGS, TU_VERIFY_1ARGS, UNUSED)(__VA_ARGS__)


/*------------------------------------------------------------------*/
/* TU_VERIFY WITH HANDLER
 * - TU_VERIFY_HDLR_2ARGS : execute handler, return false if failed
 * - TU_VERIFY_HDLR_3ARGS : execute handler, return provided error if failed
 *------------------------------------------------------------------*/
#define TU_VERIFY_HDLR_2ARGS(_cond, _handler)           TU_VERIFY_DEFINE(_cond, _handler, false)
#define TU_VERIFY_HDLR_3ARGS(_cond, _handler, _ret)     TU_VERIFY_DEFINE(_cond, _handler, _ret)

#define TU_VERIFY_HDLR(...)              GET_4TH_ARG(__VA_ARGS__, TU_VERIFY_HDLR_3ARGS, TU_VERIFY_HDLR_2ARGS,UNUSED)(__VA_ARGS__)

/*------------------------------------------------------------------*/
/* ASSERT
 * basically TU_VERIFY with TU_BREAKPOINT() as handler
 * - 1 arg : return false if failed
 * - 2 arg : return error if failed
 *------------------------------------------------------------------*/
#define ASSERT_1ARGS(_cond)            TU_VERIFY_DEFINE(_cond, _MESS_FAILED(); TU_BREAKPOINT(), false)
#define ASSERT_2ARGS(_cond, _ret)      TU_VERIFY_DEFINE(_cond, _MESS_FAILED(); TU_BREAKPOINT(), _ret)

#define TU_ASSERT(...)             GET_3RD_ARG(__VA_ARGS__, ASSERT_2ARGS, ASSERT_1ARGS,UNUSED)(__VA_ARGS__)

// TODO remove TU_ASSERT_ERR() later

/*------------- Generator for TU_VERIFY_ERR and TU_VERIFY_ERR_HDLR -------------*/
#define TU_VERIFY_ERR_DEF2(_error, _handler)  do               \
{                                                              \
  uint32_t _err = (uint32_t)(_error);                          \
  if ( 0 != _err ) { _MESS_ERR(_err); _handler; return _err; } \
} while(0)

#define TU_VERIFY_ERR_DEF3(_error, _handler, _ret) do          \
{                                                              \
  uint32_t _err = (uint32_t)(_error);                          \
  if ( 0 != _err ) { _MESS_ERR(_err); _handler; return _ret; } \
} while(0)

/*------------------------------------------------------------------*/
/* ASSERT Error
 * basically TU_VERIFY Error with TU_BREAKPOINT() as handler
 *------------------------------------------------------------------*/
#define ASERT_ERR_1ARGS(_error)         TU_VERIFY_ERR_DEF2(_error, TU_BREAKPOINT())
#define ASERT_ERR_2ARGS(_error, _ret)   TU_VERIFY_ERR_DEF3(_error, TU_BREAKPOINT(), _ret)

#define TU_ASSERT_ERR(...)         GET_3RD_ARG(__VA_ARGS__, ASERT_ERR_2ARGS, ASERT_ERR_1ARGS,UNUSED)(__VA_ARGS__)

/*------------------------------------------------------------------*/
/* ASSERT HDLR
 *------------------------------------------------------------------*/
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_error_h 

#define ERROR_ENUM(x) x,
#define ERROR_STRING(x) #x,

#define ERROR_TABLE(ENTRY) \
    ENTRY(TUSB_ERROR_NONE                            )\
    ENTRY(TUSB_ERROR_INVALID_PARA                    )\
    ENTRY(TUSB_ERROR_DEVICE_NOT_READY                )\
    ENTRY(TUSB_ERROR_INTERFACE_IS_BUSY               )\
    ENTRY(TUSB_ERROR_HCD_OPEN_PIPE_FAILED            )\
    ENTRY(TUSB_ERROR_OSAL_TIMEOUT                    )\
    ENTRY(TUSB_ERROR_CDCH_DEVICE_NOT_MOUNTED         )\
    ENTRY(TUSB_ERROR_MSCH_DEVICE_NOT_MOUNTED         )\
    ENTRY(TUSB_ERROR_NOT_SUPPORTED                   )\
    ENTRY(TUSB_ERROR_NOT_ENOUGH_MEMORY               )\
    ENTRY(TUSB_ERROR_FAILED                          )\

/// \brief Error Code returned
typedef enum
{
  ERROR_TABLE(ERROR_ENUM)
  TUSB_ERROR_COUNT
}tusb_error_t;

#if CFG_TUSB_DEBUG
/// Enum to String for debugging purposes. Only available if \ref CFG_TUSB_DEBUG > 0
extern char const* const tusb_strerr[TUSB_ERROR_COUNT];
#endif
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if usbd_h

typedef struct
{
  struct TU_ATTR_PACKED
  {
    volatile uint8_t connected    ;
    volatile uint8_t addressed    : 1;
    volatile uint8_t configured   : 1;
    volatile uint8_t suspended    : 1;

    uint8_t remote_wakeup_en      : 1; // enable/disable by host
    uint8_t remote_wakeup_support : 1; // configuration descriptor's attribute
    uint8_t self_powered          : 1; // configuration descriptor's attribute
  };

  uint8_t speed;

  uint8_t itf2drv[16];     // map interface number to driver (0xff is invalid)
  uint8_t ep2drv[8][2];    // map endpoint to driver ( 0xff is invalid )

  struct TU_ATTR_PACKED
  {
    volatile bool busy    ;
    volatile bool stalled ;
		volatile bool claimed ;
    // TODO merge ep2drv here, 4-bit should be sufficient
  }ep_status[8][2];
}usbd_device_t;	 

extern usbd_device_t _usbd_dev;
//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Init device stack
// Note: when using with RTOS, this should be called after scheduler/kernel is started.
// Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
bool tud_init (void);

// Task function should be called in main/rtos loop
void tud_task (void);

// Check if there is pending events need proccessing by tud_task()
bool tud_task_event_ready(void);

// Interrupt handler, name alias to DCD
#define tud_int_handler   usb_handler

// Get current bus speed
tusb_speed_t tud_speed_get(void);

// Check if device is connected and configured
bool tud_mounted(void);

// Check if device is suspended
bool tud_suspended(void);

// Check if device is ready to transfer
static inline bool tud_ready(void)
{
  return tud_mounted() && !tud_suspended();
}

// Remote wake up host, only if suspended and enabled by host
bool tud_remote_wakeup(void);

// Enable pull-up resistor on D+ D-
// Return false on unsupported MCUs
bool tud_disconnect(void);

// Disable pull-up resistor on D+ D-
// Return false on unsupported MCUs
bool tud_connect(void);

// Carry out Data and Status stage of control transfer
// - If len = 0, it is equivalent to sending status only
// - If len > wLength : it will be truncated
bool tud_control_xfer(uint8_t rhport, tusb_control_request_t const * request, void* buffer, uint16_t len);

// Send STATUS (zero length) packet
bool tud_control_status(uint8_t rhport, tusb_control_request_t const * request);

//--------------------------------------------------------------------+
// Application Callbacks (WEAK is optional)
//--------------------------------------------------------------------+

// Invoked when received GET DEVICE DESCRIPTOR request
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void);

// Invoked when received GET BOS DESCRIPTOR request
// Application return pointer to descriptor
TU_ATTR_WEAK uint8_t const * tud_descriptor_bos_cb(void);

// Invoked when received GET CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index);

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
TU_ATTR_WEAK uint8_t const* tud_descriptor_device_qualifier_cb(void);

//// Invoked when device is mounted (configured)
TU_ATTR_WEAK void tud_mount_cb(void);

//// Invoked when device is unmounted
TU_ATTR_WEAK void tud_umount_cb(uint8_t rhport);

// Invoked when usb bus is suspended
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(uint8_t rhport);

// Invoked when usb bus is resumed
TU_ATTR_WEAK void tud_resume_cb(void);

// Invoked when received control request with VENDOR TYPE
TU_ATTR_WEAK bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request);

// Invoked when vendor control request is complete
TU_ATTR_WEAK bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request);


//--------------------------------------------------------------------+
// Binary Device Object Store (BOS) Descriptor Templates
//--------------------------------------------------------------------+

#define TUD_BOS_DESC_LEN      5

// total length, number of device caps
#define TUD_BOS_DESCRIPTOR(_total_len, _caps_num) \
  5, TUSB_DESC_BOS, U16_TO_U8S_LE(_total_len), _caps_num

// Device Capability Platform 128-bit UUID + Data
#define TUD_BOS_PLATFORM_DESCRIPTOR(...) \
  4+TU_ARGS_NUM(__VA_ARGS__), TUSB_DESC_DEVICE_CAPABILITY, DEVICE_CAPABILITY_PLATFORM, 0x00, __VA_ARGS__

//------------- WebUSB BOS Platform -------------//

// Descriptor Length
#define TUD_BOS_WEBUSB_DESC_LEN         24

// Vendor Code, iLandingPage
#define TUD_BOS_WEBUSB_DESCRIPTOR(_vendor_code, _ipage) \
  TUD_BOS_PLATFORM_DESCRIPTOR(TUD_BOS_WEBUSB_UUID, U16_TO_U8S_LE(0x0100), _vendor_code, _ipage)

#define TUD_BOS_WEBUSB_UUID   \
  0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47, \
  0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65

//------------- Microsoft OS 2.0 Platform -------------//
#define TUD_BOS_MICROSOFT_OS_DESC_LEN   28

// Total Length of descriptor set, vendor code
#define TUD_BOS_MS_OS_20_DESCRIPTOR(_desc_set_len, _vendor_code) \
  TUD_BOS_PLATFORM_DESCRIPTOR(TUD_BOS_MS_OS_20_UUID, U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(_desc_set_len), _vendor_code, 0)

#define TUD_BOS_MS_OS_20_UUID \
  0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C, \
  0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F

//--------------------------------------------------------------------+
// Configuration & Interface Descriptor Templates
//--------------------------------------------------------------------+

//------------- Configuration -------------//
#define TUD_CONFIG_DESC_LEN   (9)

// Config number, interface count, string index, total length, attribute, power in mA
#define TUD_CONFIG_DESCRIPTOR(config_num, _itfcount, _stridx, _total_len, _attribute, _power_ma) \
  9, TUSB_DESC_CONFIGURATION, U16_TO_U8S_LE(_total_len), _itfcount, config_num, _stridx, TU_BIT(7) | _attribute, (_power_ma)/2

//------------- CDC -------------//

// Length of template descriptor: 66 bytes
#define TUD_CDC_DESC_LEN  (8+9+5+5+4+5+7+9+7+7)

// CDC Descriptor Template
// Interface number, string index, EP notification address and size, EP data address (out, in) and size.
#define TUD_CDC_DESCRIPTOR(_itfnum, _stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize) \
  /* Interface Associate */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, _stridx,\
  /* CDC Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC Call */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0, (uint8_t)((_itfnum) + 1),\
  /* CDC ACM: support line request */\
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 2,\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 16,\
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//------------- MSC -------------//

// Length of template descriptor: 23 bytes
#define TUD_MSC_DESC_LEN    (9 + 7 + 7)

// Interface number, string index, EP Out & EP In address, EP size
#define TUD_MSC_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_MSC, MSC_SUBCLASS_SCSI, MSC_PROTOCOL_BOT, _stridx,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//------------- HID -------------//

// Length of template descriptor: 25 bytes
#define TUD_HID_DESC_LEN    (9 + 9 + 7)

// HID Input only descriptor
// Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
#define TUD_HID_DESCRIPTOR(_itfnum, _stridx, _boot_protocol, _report_desc_len, _epin, _epsize, _ep_interval) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_HID, (uint8_t)((_boot_protocol) ? HID_SUBCLASS_BOOT : 0), _boot_protocol, _stridx,\
  /* HID descriptor */\
  9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0111), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(_report_desc_len),\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval

// Length of template descriptor: 32 bytes
#define TUD_HID_INOUT_DESC_LEN    (9 + 9 + 7 + 7)

// HID Input & Output descriptor
// Interface number, string index, protocol, report descriptor len, EP OUT & IN address, size & polling interval
#define TUD_HID_INOUT_DESCRIPTOR(_itfnum, _stridx, _boot_protocol, _report_desc_len, _epout, _epin, _epsize, _ep_interval) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_HID, (uint8_t)((_boot_protocol) ? HID_SUBCLASS_BOOT : 0), _boot_protocol, _stridx,\
  /* HID descriptor */\
  9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0111), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(_report_desc_len),\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval, \
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval

//------------- MIDI -------------//

#define TUD_MIDI_DESC_HEAD_LEN (9 + 9 + 9 + 7)
#define TUD_MIDI_DESC_HEAD(_itfnum,  _stridx, _numcables) \
  /* Audio Control (AC) Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_PROTOCOL_V1, _stridx,\
  /* AC Header */\
  9, TUSB_DESC_CS_INTERFACE, AUDIO_CS_INTERFACE_HEADER, U16_TO_U8S_LE(0x0100), U16_TO_U8S_LE(0x0009), 1, (uint8_t)((_itfnum) + 1),\
  /* MIDI Streaming (MS) Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum) + 1), 0, 2, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING, AUDIO_PROTOCOL_V1, 0,\
  /* MS Header */\
  7, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_HEADER, U16_TO_U8S_LE(0x0100), U16_TO_U8S_LE(7 + (_numcables) * TUD_MIDI_DESC_JACK_LEN)

#define TUD_MIDI_JACKID_IN_EMB(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 1)

#define TUD_MIDI_JACKID_IN_EXT(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 2)

#define TUD_MIDI_JACKID_OUT_EMB(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 3)

#define TUD_MIDI_JACKID_OUT_EXT(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 4)

#define TUD_MIDI_DESC_JACK_LEN (6 + 6 + 9 + 9)
#define TUD_MIDI_DESC_JACK(_cablenum) \
  /* MS In Jack (Embedded) */\
  6, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_IN_JACK, MIDI_JACK_EMBEDDED, TUD_MIDI_JACKID_IN_EMB(_cablenum), 0,\
  /* MS In Jack (External) */\
  6, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_IN_JACK, MIDI_JACK_EXTERNAL, TUD_MIDI_JACKID_IN_EXT(_cablenum), 0,\
  /* MS Out Jack (Embedded), connected to In Jack External */\
  9, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_OUT_JACK, MIDI_JACK_EMBEDDED, TUD_MIDI_JACKID_OUT_EMB(_cablenum), 1, TUD_MIDI_JACKID_IN_EXT(_cablenum), 1, 0,\
  /* MS Out Jack (External), connected to In Jack Embedded */\
  9, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_OUT_JACK, MIDI_JACK_EXTERNAL, TUD_MIDI_JACKID_OUT_EXT(_cablenum), 1, TUD_MIDI_JACKID_IN_EMB(_cablenum), 1, 0

#define TUD_MIDI_DESC_EP_LEN(_numcables) (7 + 4 + (_numcables))
#define TUD_MIDI_DESC_EP(_epout, _epsize, _numcables) \
  /* Endpoint */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* MS Endpoint (connected to embedded jack) */\
  (uint8_t)(4 + (_numcables)), TUSB_DESC_CS_ENDPOINT, MIDI_CS_ENDPOINT_GENERAL, _numcables

// Length of template descriptor (88 bytes)
#define TUD_MIDI_DESC_LEN (TUD_MIDI_DESC_HEAD_LEN + TUD_MIDI_DESC_JACK_LEN + TUD_MIDI_DESC_EP_LEN(1) * 2)

// MIDI simple descriptor
// - 1 Embedded Jack In connected to 1 External Jack Out
// - 1 Embedded Jack out connected to 1 External Jack In
#define TUD_MIDI_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  TUD_MIDI_DESC_HEAD(_itfnum, _stridx, 1),\
  TUD_MIDI_DESC_JACK(1),\
  TUD_MIDI_DESC_EP(_epout, _epsize, 1),\
  TUD_MIDI_JACKID_IN_EMB(1),\
  TUD_MIDI_DESC_EP(_epin, _epsize, 1),\
  TUD_MIDI_JACKID_OUT_EMB(1)


//------------- TUD_USBTMC/USB488 -------------//
#define TUD_USBTMC_APP_CLASS    (TUSB_CLASS_APPLICATION_SPECIFIC)
#define TUD_USBTMC_APP_SUBCLASS 0x03u

#define TUD_USBTMC_PROTOCOL_STD    0x00u
#define TUD_USBTMC_PROTOCOL_USB488 0x01u

//   Interface number, number of endpoints, EP string index, USB_TMC_PROTOCOL*, bulk-out endpoint ID,
//   bulk-in endpoint ID
#define TUD_USBTMC_IF_DESCRIPTOR(_itfnum, _bNumEndpoints, _stridx, _itfProtocol) \
  /* Interface */ \
  0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, _bNumEndpoints, TUD_USBTMC_APP_CLASS, TUD_USBTMC_APP_SUBCLASS, _itfProtocol, _stridx

#define TUD_USBTMC_IF_DESCRIPTOR_LEN 9u

#define TUD_USBTMC_BULK_DESCRIPTORS(_epout, _epin, _bulk_epsize) \
  /* Endpoint Out */ \
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_bulk_epsize), 0u, \
  /* Endpoint In */ \
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_bulk_epsize), 0u

#define TUD_USBTMC_BULK_DESCRIPTORS_LEN (7u+7u)

/* optional interrupt endpoint */ \
// _int_pollingInterval : for LS/FS, expressed in frames (1ms each). 16 may be a good number?
#define TUD_USBTMC_INT_DESCRIPTOR(_ep_interrupt, _ep_interrupt_size, _int_pollingInterval ) \
  7, TUSB_DESC_ENDPOINT, _ep_interrupt, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_interrupt_size), 0x16

#define TUD_USBTMC_INT_DESCRIPTOR_LEN (7u)


//------------- Vendor -------------//
#define TUD_VENDOR_DESC_LEN  (9+7+7)

// Interface number, string index, EP Out & IN address, EP size
#define TUD_VENDOR_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//------------- DFU Runtime -------------//
#define TUD_DFU_APP_CLASS    (TUSB_CLASS_APPLICATION_SPECIFIC)
#define TUD_DFU_APP_SUBCLASS 0x01u

// Length of template descriptr: 18 bytes
#define TUD_DFU_RT_DESC_LEN (9 + 9)

// DFU runtime descriptor
// Interface number, string index, attributes, detach timeout, transfer size
#define TUD_DFU_RT_DESCRIPTOR(_itfnum, _stridx, _attr, _timeout, _xfer_size) \
  /* Interface */ \
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUD_DFU_APP_CLASS, TUD_DFU_APP_SUBCLASS, DFU_PROTOCOL_RT, _stridx, \
  /* Function */ \
  9, DFU_DESC_FUNCTIONAL, _attr, U16_TO_U8S_LE(_timeout), U16_TO_U8S_LE(_xfer_size), U16_TO_U8S_LE(0x0101)


//------------- CDC-ECM -------------//

// Length of template descriptor: 71 bytes
#define TUD_CDC_ECM_DESC_LEN  (8+9+5+5+13+7+9+9+7+7)

// CDC-ECM Descriptor Template
// Interface number, description string index, MAC address string index, EP notification address and size, EP data address (out, in), and size, max segment size.
#define TUD_CDC_ECM_DESCRIPTOR(_itfnum, _desc_stridx, _mac_stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize, _maxsegmentsize) \
  /* Interface Association */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ETHERNET_NETWORKING_CONTROL_MODEL, 0, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ETHERNET_NETWORKING_CONTROL_MODEL, 0, _desc_stridx,\
  /* CDC-ECM Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC-ECM Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* CDC-ECM Functional Descriptor */\
  13, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ETHERNET_NETWORKING, _mac_stridx, 0, 0, 0, 0, U16_TO_U8S_LE(_maxsegmentsize), U16_TO_U8S_LE(0), 0,\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 1,\
  /* CDC Data Interface (default inactive) */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 0, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* CDC Data Interface (alternative active) */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 1, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0


//------------- RNDIS -------------//

#if 0
  /* Windows XP */
  #define TUD_RNDIS_ITF_CLASS    TUSB_CLASS_CDC
  #define TUD_RNDIS_ITF_SUBCLASS CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL
  #define TUD_RNDIS_ITF_PROTOCOL 0xFF /* CDC_COMM_PROTOCOL_MICROSOFT_RNDIS */
#else
  /* Windows 7+ */
  #define TUD_RNDIS_ITF_CLASS    TUSB_CLASS_WIRELESS_CONTROLLER
  #define TUD_RNDIS_ITF_SUBCLASS 0x01
  #define TUD_RNDIS_ITF_PROTOCOL 0x03
#endif

// Length of template descriptor: 66 bytes
#define TUD_RNDIS_DESC_LEN  (8+9+5+5+4+5+7+9+7+7)

// RNDIS Descriptor Template
// Interface number, string index, EP notification address and size, EP data address (out, in) and size.
#define TUD_RNDIS_DESCRIPTOR(_itfnum, _stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize) \
  /* Interface Association */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUD_RNDIS_ITF_CLASS, TUD_RNDIS_ITF_SUBCLASS, TUD_RNDIS_ITF_PROTOCOL, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUD_RNDIS_ITF_CLASS, TUD_RNDIS_ITF_SUBCLASS, TUD_RNDIS_ITF_PROTOCOL, _stridx,\
  /* CDC-ACM Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0110),\
  /* CDC Call Management */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0, (uint8_t)((_itfnum) + 1),\
  /* ACM */\
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 0,\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 1,\
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//------------- BT Radio -------------//
#define TUD_BT_APP_CLASS                    (TUSB_CLASS_WIRELESS_CONTROLLER)
#define TUD_BT_APP_SUBCLASS                 0x01
#define TUD_BT_PROTOCOL_PRIMARY_CONTROLLER  0x01
#define TUD_BT_PROTOCOL_AMP_CONTROLLER      0x02

#ifndef CFG_TUD_BTH_ISO_ALT_COUNT
#define CFG_TUD_BTH_ISO_ALT_COUNT 0
#endif

// Length of template descriptor: 30 bytes + number of ISO alternatives * 23
#define TUD_BTH_DESC_LEN (9 + 7 + 7 + 7 + (CFG_TUD_BTH_ISO_ALT_COUNT) * (9 + 7 + 7))

/* Primary Interface */
#define TUD_BTH_PRI_ITF(_itfnum, _stridx, _ep_evt, _ep_evt_size, _ep_evt_interval, _ep_in, _ep_out, _ep_size) \
  9, TUSB_DESC_INTERFACE, _itfnum, _stridx, 3, TUD_BT_APP_CLASS, TUD_BT_APP_SUBCLASS, TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, \
  /* Endpoint In for events */ \
  7, TUSB_DESC_ENDPOINT, _ep_evt, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_evt_size), _ep_evt_interval, \
  /* Endpoint In for ACL data */ \
  7, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size), 1, \
  /* Endpoint Out for ACL data */ \
  7, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size), 1

#define TUD_BTH_ISO_ITF(_itfnum, _alt, _ep_in, _ep_out, _n) ,\
  /* Interface with 2 endpoints */ \
  9, TUSB_DESC_INTERFACE, _itfnum, _alt, 2, TUD_BT_APP_CLASS, TUD_BT_APP_SUBCLASS, TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, \
  /* Isochronous endpoints */ \
  7, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_n), 1, \
  7, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_n), 1

#define _FIRST(a, ...) a
#define _REST(a, ...) __VA_ARGS__

#define TUD_BTH_ISO_ITF_0(_itfnum, ...)
#define TUD_BTH_ISO_ITF_1(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 1, _ep_in, _ep_out, _FIRST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_2(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 2, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
	TUD_BTH_ISO_ITF_1(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_3(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 3, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
	TUD_BTH_ISO_ITF_2(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_4(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 4, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
	TUD_BTH_ISO_ITF_3(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_5(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 5, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
	TUD_BTH_ISO_ITF_4(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_6(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 6, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
	TUD_BTH_ISO_ITF_5(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))

#define TUD_BTH_ISO_ITFS(_itfnum, _ep_in, _ep_out, ...) \
	TU_XSTRCAT(TUD_BTH_ISO_ITF_, CFG_TUD_BTH_ISO_ALT_COUNT)(_itfnum, _ep_in, _ep_out, __VA_ARGS__)

// BT Primary controller descriptor
// Interface number, string index, attributes, event endpoint, event endpoint size, interval, data in, data out, data endpoint size, iso endpoint sizes
#define TUD_BTH_DESCRIPTOR(_itfnum, _stridx, _ep_evt, _ep_evt_size, _ep_evt_interval, _ep_in, _ep_out, _ep_size,...) \
  TUD_BTH_PRI_ITF(_itfnum, _stridx, _ep_evt, _ep_evt_size, _ep_evt_interval, _ep_in, _ep_out, _ep_size) \
  TUD_BTH_ISO_ITFS(_itfnum + 1, _ep_in + 1, _ep_out + 1, __VA_ARGS__)

#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if msc_h

//--------------------------------------------------------------------+
// Mass Storage Class Constant
//--------------------------------------------------------------------+
/// MassStorage Subclass
typedef enum
{
  MSC_SUBCLASS_RBC = 1 , ///< Reduced Block Commands (RBC) T10 Project 1240-D
  MSC_SUBCLASS_SFF_MMC , ///< SFF-8020i, MMC-2 (ATAPI). Typically used by a CD/DVD device
  MSC_SUBCLASS_QIC     , ///< QIC-157. Typically used by a tape device
  MSC_SUBCLASS_UFI     , ///< UFI. Typically used by Floppy Disk Drive (FDD) device
  MSC_SUBCLASS_SFF     , ///< SFF-8070i. Can be used by Floppy Disk Drive (FDD) device
  MSC_SUBCLASS_SCSI      ///< SCSI transparent command set
}msc_subclass_type_t;

enum {
  MSC_CBW_SIGNATURE = 0x43425355, ///< Constant value of 43425355h (little endian)
  MSC_CSW_SIGNATURE = 0x53425355  ///< Constant value of 53425355h (little endian)
};

/// \brief MassStorage Protocol.
/// \details CBI only approved to use with full-speed floopy disk & should not used with highspeed or device other than floopy
typedef enum
{
  MSC_PROTOCOL_CBI              = 0 ,  ///< Control/Bulk/Interrupt protocol (with command completion interrupt)
  MSC_PROTOCOL_CBI_NO_INTERRUPT = 1 ,  ///< Control/Bulk/Interrupt protocol (without command completion interrupt)
  MSC_PROTOCOL_BOT              = 0x50 ///< Bulk-Only Transport
}msc_protocol_type_t;

/// MassStorage Class-Specific Control Request
typedef enum
{
  MSC_REQ_GET_MAX_LUN = 254, ///< The Get Max LUN device request is used to determine the number of logical units supported by the device. Logical Unit Numbers on the device shall be numbered contiguously starting from LUN 0 to a maximum LUN of 15
  MSC_REQ_RESET       = 255  ///< This request is used to reset the mass storage device and its associated interface. This class-specific request shall ready the device for the next CBW from the host.
}msc_request_type_t;

/// \brief Command Block Status Values
/// \details Indicates the success or failure of the command. The device shall set this byte to zero if the command completed
/// successfully. A non-zero value shall indicate a failure during command execution according to the following
typedef enum
{
  MSC_CSW_STATUS_PASSED = 0 , ///< MSC_CSW_STATUS_PASSED
  MSC_CSW_STATUS_FAILED     , ///< MSC_CSW_STATUS_FAILED
  MSC_CSW_STATUS_PHASE_ERROR  ///< MSC_CSW_STATUS_PHASE_ERROR
}msc_csw_status_t;

/// Command Block Wrapper
typedef struct TU_ATTR_PACKED
{
  uint32_t signature;   ///< Signature that helps identify this data packet as a CBW. The signature field shall contain the value 43425355h (little endian), indicating a CBW.
  uint32_t tag;         ///< Tag sent by the host. The device shall echo the contents of this field back to the host in the dCSWTagfield of the associated CSW. The dCSWTagpositively associates a CSW with the corresponding CBW.
  uint32_t total_bytes; ///< The number of bytes of data that the host expects to transfer on the Bulk-In or Bulk-Out endpoint (as indicated by the Direction bit) during the execution of this command. If this field is zero, the device and the host shall transfer no data between the CBW and the associated CSW, and the device shall ignore the value of the Direction bit in bmCBWFlags.
  uint8_t dir;          ///< Bit 7 of this field define transfer direction \n - 0 : Data-Out from host to the device. \n - 1 : Data-In from the device to the host.
  uint8_t lun;          ///< The device Logical Unit Number (LUN) to which the command block is being sent. For devices that support multiple LUNs, the host shall place into this field the LUN to which this command block is addressed. Otherwise, the host shall set this field to zero.
  uint8_t cmd_len;      ///< The valid length of the CBWCBin bytes. This defines the valid length of the command block. The only legal values are 1 through 16
  uint8_t command[16];  ///< The command block to be executed by the device. The device shall interpret the first cmd_len bytes in this field as a command block
}msc_cbw_t;

TU_VERIFY_STATIC(sizeof(msc_cbw_t) == 31, "size is not correct");

/// Command Status Wrapper
typedef struct TU_ATTR_PACKED
{
  uint32_t signature    ; ///< Signature that helps identify this data packet as a CSW. The signature field shall contain the value 53425355h (little endian), indicating CSW.
  uint32_t tag          ; ///< The device shall set this field to the value received in the dCBWTag of the associated CBW.
  uint32_t data_residue ; ///< For Data-Out the device shall report in the dCSWDataResiduethe difference between the amount of data expected as stated in the dCBWDataTransferLength, and the actual amount of data processed by the device. For Data-In the device shall report in the dCSWDataResiduethe difference between the amount of data expected as stated in the dCBWDataTransferLengthand the actual amount of relevant data sent by the device
  uint8_t  status       ; ///< indicates the success or failure of the command. Values from \ref msc_csw_status_t
}msc_csw_t;

TU_VERIFY_STATIC(sizeof(msc_csw_t) == 13, "size is not correct");

//--------------------------------------------------------------------+
// SCSI Constant
//--------------------------------------------------------------------+

/// SCSI Command Operation Code
typedef enum
{
  SCSI_CMD_TEST_UNIT_READY              = 0x00, ///< The SCSI Test Unit Ready command is used to determine if a device is ready to transfer data (read/write), i.e. if a disk has spun up, if a tape is loaded and ready etc. The device does not perform a self-test operation.
  SCSI_CMD_INQUIRY                      = 0x12, ///< The SCSI Inquiry command is used to obtain basic information from a target device.
  SCSI_CMD_MODE_SELECT_6                = 0x15, ///<  provides a means for the application client to specify medium, logical unit, or peripheral device parameters to the device server. Device servers that implement the MODE SELECT(6) command shall also implement the MODE SENSE(6) command. Application clients should issue MODE SENSE(6) prior to each MODE SELECT(6) to determine supported mode pages, page lengths, and other parameters.
  SCSI_CMD_MODE_SENSE_6                 = 0x1A, ///< provides a means for a device server to report parameters to an application client. It is a complementary command to the MODE SELECT(6) command. Device servers that implement the MODE SENSE(6) command shall also implement the MODE SELECT(6) command.
  SCSI_CMD_START_STOP_UNIT              = 0x1B,
  SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL = 0x1E,
  SCSI_CMD_READ_CAPACITY_10             = 0x25, ///< The SCSI Read Capacity command is used to obtain data capacity information from a target device.
  SCSI_CMD_REQUEST_SENSE                = 0x03, ///< The SCSI Request Sense command is part of the SCSI computer protocol standard. This command is used to obtain sense data -- status/error information -- from a target device.
  SCSI_CMD_READ_FORMAT_CAPACITY         = 0x23, ///< The command allows the Host to request a list of the possible format capacities for an installed writable media. This command also has the capability to report the writable capacity for a media when it is installed
  SCSI_CMD_READ_10                      = 0x28, ///< The READ (10) command requests that the device server read the specified logical block(s) and transfer them to the data-in buffer.
  SCSI_CMD_WRITE_10                     = 0x2A, ///< The WRITE (10) command requests thatthe device server transfer the specified logical block(s) from the data-out buffer and write them.
}scsi_cmd_type_t;

/// SCSI Sense Key
typedef enum
{
  SCSI_SENSE_NONE            = 0x00, ///< no specific Sense Key. This would be the case for a successful command
  SCSI_SENSE_RECOVERED_ERROR = 0x01, ///< ndicates the last command completed successfully with some recovery action performed by the disc drive.
  SCSI_SENSE_NOT_READY       = 0x02, ///< Indicates the logical unit addressed cannot be accessed.
  SCSI_SENSE_MEDIUM_ERROR    = 0x03, ///< Indicates the command terminated with a non-recovered error condition.
  SCSI_SENSE_HARDWARE_ERROR  = 0x04, ///< Indicates the disc drive detected a nonrecoverable hardware failure while performing the command or during a self test.
  SCSI_SENSE_ILLEGAL_REQUEST = 0x05, ///< Indicates an illegal parameter in the command descriptor block or in the additional parameters
  SCSI_SENSE_UNIT_ATTENTION  = 0x06, ///< Indicates the disc drive may have been reset.
  SCSI_SENSE_DATA_PROTECT    = 0x07, ///< Indicates that a command that reads or writes the medium was attempted on a block that is protected from this operation. The read or write operation is not performed.
  SCSI_SENSE_FIRMWARE_ERROR  = 0x08, ///< Vendor specific sense key.
  SCSI_SENSE_ABORTED_COMMAND = 0x0b, ///< Indicates the disc drive aborted the command.
  SCSI_SENSE_EQUAL           = 0x0c, ///< Indicates a SEARCH DATA command has satisfied an equal comparison.
  SCSI_SENSE_VOLUME_OVERFLOW = 0x0d, ///< Indicates a buffered peripheral device has reached the end of medium partition and data remains in the buffer that has not been written to the medium.
  SCSI_SENSE_MISCOMPARE      = 0x0e  ///< ndicates that the source data did not match the data read from the medium.
}scsi_sense_key_type_t;

//--------------------------------------------------------------------+
// SCSI Primary Command (SPC-4)
//--------------------------------------------------------------------+

/// SCSI Test Unit Ready Command
typedef struct TU_ATTR_PACKED
{
  uint8_t cmd_code    ; ///< SCSI OpCode for \ref SCSI_CMD_TEST_UNIT_READY
  uint8_t lun         ; ///< Logical Unit
  uint8_t reserved[3] ;
  uint8_t control     ;
} scsi_test_unit_ready_t;

TU_VERIFY_STATIC(sizeof(scsi_test_unit_ready_t) == 6, "size is not correct");

/// SCSI Inquiry Command
typedef struct TU_ATTR_PACKED
{
  uint8_t cmd_code     ; ///< SCSI OpCode for \ref SCSI_CMD_INQUIRY
  uint8_t reserved1    ;
  uint8_t page_code    ;
  uint8_t reserved2    ;
  uint8_t alloc_length ; ///< specifies the maximum number of bytes that USB host has allocated in the Data-In Buffer. An allocation length of zero specifies that no data shall be transferred.
  uint8_t control      ;
} scsi_inquiry_t, scsi_request_sense_t;

TU_VERIFY_STATIC(sizeof(scsi_inquiry_t) == 6, "size is not correct");

/// SCSI Inquiry Response Data
typedef struct TU_ATTR_PACKED
{
  uint8_t peripheral_device_type     : 5;
  uint8_t peripheral_qualifier       : 3;

  uint8_t                            : 7;
  uint8_t is_removable               : 1;

  uint8_t version;

  uint8_t response_data_format       : 4;
  uint8_t hierarchical_support       : 1;
  uint8_t normal_aca                 : 1;
  uint8_t                            : 2;

  uint8_t additional_length;

  uint8_t protect                    : 1;
  uint8_t                            : 2;
  uint8_t third_party_copy           : 1;
  uint8_t target_port_group_support  : 2;
  uint8_t access_control_coordinator : 1;
  uint8_t scc_support                : 1;

  uint8_t addr16                     : 1;
  uint8_t                            : 3;
  uint8_t multi_port                 : 1;
  uint8_t                            : 1; // vendor specific
  uint8_t enclosure_service          : 1;
  uint8_t                            : 1;

  uint8_t                            : 1; // vendor specific
  uint8_t cmd_que                    : 1;
  uint8_t                            : 2;
  uint8_t sync                       : 1;
  uint8_t wbus16                     : 1;
  uint8_t                            : 2;

  uint8_t vendor_id[8]  ; ///< 8 bytes of ASCII data identifying the vendor of the product.
  uint8_t product_id[16]; ///< 16 bytes of ASCII data defined by the vendor.
  uint8_t product_rev[4]; ///< 4 bytes of ASCII data defined by the vendor.
} scsi_inquiry_resp_t;

TU_VERIFY_STATIC(sizeof(scsi_inquiry_resp_t) == 36, "size is not correct");


typedef struct TU_ATTR_PACKED
{
  uint8_t response_code : 7; ///< 70h - current errors, Fixed Format 71h - deferred errors, Fixed Format
  uint8_t valid         : 1;

  uint8_t reserved;

  uint8_t sense_key     : 4;
  uint8_t               : 1;
  uint8_t ili           : 1; ///< Incorrect length indicator
  uint8_t end_of_medium : 1;
  uint8_t filemark      : 1;

  uint32_t information;
  uint8_t  add_sense_len;
  uint32_t command_specific_info;
  uint8_t  add_sense_code;
  uint8_t  add_sense_qualifier;
  uint8_t  field_replaceable_unit_code;

  uint8_t  sense_key_specific[3]; ///< sense key specific valid bit is bit 7 of key[0], aka MSB in Big Endian layout

} scsi_sense_fixed_resp_t;

TU_VERIFY_STATIC(sizeof(scsi_sense_fixed_resp_t) == 18, "size is not correct");

typedef struct TU_ATTR_PACKED
{
  uint8_t cmd_code     ; ///< SCSI OpCode for \ref SCSI_CMD_MODE_SENSE_6

  uint8_t : 3;
  uint8_t disable_block_descriptor : 1;
  uint8_t : 0;

  uint8_t page_code : 6;
  uint8_t page_control : 2;

  uint8_t subpage_code;
  uint8_t alloc_length;
  uint8_t control;
} scsi_mode_sense6_t;

TU_VERIFY_STATIC( sizeof(scsi_mode_sense6_t) == 6, "size is not correct");

// This is only a Mode parameter header(6).
typedef struct TU_ATTR_PACKED
{
  uint8_t data_len;
  uint8_t medium_type;

  uint8_t reserved : 7;
  bool write_protected : 1;

  uint8_t block_descriptor_len;
} scsi_mode_sense6_resp_t;

TU_VERIFY_STATIC( sizeof(scsi_mode_sense6_resp_t) == 4, "size is not correct");

typedef struct TU_ATTR_PACKED
{
  uint8_t cmd_code; ///< SCSI OpCode for \ref SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL
  uint8_t reserved[3];
  uint8_t prohibit_removal;
  uint8_t control;
} scsi_prevent_allow_medium_removal_t;

TU_VERIFY_STATIC( sizeof(scsi_prevent_allow_medium_removal_t) == 6, "size is not correct");

typedef struct TU_ATTR_PACKED
{
  uint8_t cmd_code;

  uint8_t immded : 1;
  uint8_t        : 7;

  uint8_t TU_RESERVED;

  uint8_t power_condition_mod : 4;
  uint8_t                     : 4;

  uint8_t start           : 1;
  uint8_t load_eject      : 1;
  uint8_t no_flush        : 1;
  uint8_t                 : 1;
  uint8_t power_condition : 4;

  uint8_t control;
} scsi_start_stop_unit_t;

TU_VERIFY_STATIC( sizeof(scsi_start_stop_unit_t) == 6, "size is not correct");

//--------------------------------------------------------------------+
// SCSI MMC
//--------------------------------------------------------------------+
/// SCSI Read Format Capacity: Write Capacity
typedef struct TU_ATTR_PACKED
{
  uint8_t cmd_code;
  uint8_t reserved[6];
  uint16_t alloc_length;
  uint8_t control;
} scsi_read_format_capacity_t;

TU_VERIFY_STATIC( sizeof(scsi_read_format_capacity_t) == 10, "size is not correct");

typedef struct TU_ATTR_PACKED{
  uint8_t reserved[3];
  uint8_t list_length; /// must be 8*n, length in bytes of formattable capacity descriptor followed it.

  uint32_t block_num; /// Number of Logical Blocks
  uint8_t  descriptor_type; // 00: reserved, 01 unformatted media , 10 Formatted media, 11 No media present

  uint8_t  reserved2;
  uint16_t block_size_u16;

} scsi_read_format_capacity_data_t;

TU_VERIFY_STATIC( sizeof(scsi_read_format_capacity_data_t) == 12, "size is not correct");

//--------------------------------------------------------------------+
// SCSI Block Command (SBC-3)
// NOTE: All data in SCSI command are in Big Endian
//--------------------------------------------------------------------+

/// SCSI Read Capacity 10 Command: Read Capacity
typedef struct TU_ATTR_PACKED
{
  uint8_t  cmd_code                 ; ///< SCSI OpCode for \ref SCSI_CMD_READ_CAPACITY_10
  uint8_t  reserved1                ;
  uint32_t lba                      ; ///< The first Logical Block Address (LBA) accessed by this command
  uint16_t reserved2                ;
  uint8_t  partial_medium_indicator ;
  uint8_t  control                  ;
} scsi_read_capacity10_t;

TU_VERIFY_STATIC(sizeof(scsi_read_capacity10_t) == 10, "size is not correct");

/// SCSI Read Capacity 10 Response Data
typedef struct {
  uint32_t last_lba   ; ///< The last Logical Block Address of the device
  uint32_t block_size ; ///< Block size in bytes
} scsi_read_capacity10_resp_t;

TU_VERIFY_STATIC(sizeof(scsi_read_capacity10_resp_t) == 8, "size is not correct");

/// SCSI Read 10 Command
typedef struct TU_ATTR_PACKED
{
  uint8_t  cmd_code    ; ///< SCSI OpCode
  uint8_t  reserved    ; // has LUN according to wiki
  uint32_t lba         ; ///< The first Logical Block Address (LBA) accessed by this command
  uint8_t  reserved2   ;
  uint16_t block_count ; ///< Number of Blocks used by this command
  uint8_t  control     ;
} scsi_read10_t, scsi_write10_t;

TU_VERIFY_STATIC(sizeof(scsi_read10_t) == 10, "size is not correct");
TU_VERIFY_STATIC(sizeof(scsi_write10_t) == 10, "size is not correct");
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if usbd_pvt_h

//--------------------------------------------------------------------+
// Class Drivers
//--------------------------------------------------------------------+

typedef struct
{
  #if CFG_TUSB_DEBUG >= 2
  char const* name;
  #endif

  void     (* init             ) (void);
  void     (* reset            ) (uint8_t rhport);
  uint16_t (* open             ) (uint8_t rhport, tusb_desc_interface_t const * desc_intf, uint16_t max_len);
  bool     (* control_request  ) (uint8_t rhport, tusb_control_request_t * request);
  bool     (* control_complete ) (uint8_t rhport, tusb_control_request_t const * request);
  bool     (* xfer_cb          ) (uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);
  void     (* sof              ) (uint8_t rhport); /* optional */
} usbd_class_driver_t;

// Invoked when initializing device stack to get additional class drivers.
// Can optionally implemented by application to extend/overwrite class driver support.
// Note: The drivers array must be accessible at all time when stack is active
usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) TU_ATTR_WEAK;

//--------------------------------------------------------------------+
// USBD Endpoint API
//--------------------------------------------------------------------+

// Open an endpoint
bool usbd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep);

// Close an endpoint
void usbd_edpt_close(uint8_t rhport, uint8_t ep_addr);

// Claim an endpoint before submitting a transfer
bool usbd_edpt_claim(uint8_t cdc_msc, uint8_t ep_addr);

// Release an endpoint without submitting a transfer
bool usbd_edpt_release(uint8_t cdc_msc, uint8_t ep_addr);

// Submit a usb transfer
bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes);

// Check if endpoint transferring is complete
bool usbd_edpt_busy(uint8_t rhport, uint8_t ep_addr);

// Stall endpoint
void usbd_edpt_stall(uint8_t rhport, uint8_t ep_addr);

// Clear stalled endpoint
void usbd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr);

// Check if endpoint is stalled
bool usbd_edpt_stalled(uint8_t rhport, uint8_t ep_addr);

static inline
bool usbd_edpt_ready(uint8_t rhport, uint8_t ep_addr)
{
  return !usbd_edpt_busy(rhport, ep_addr) && !usbd_edpt_stalled(rhport, ep_addr);
}

/*------------------------------------------------------------------*/
/* Helper
 *------------------------------------------------------------------*/

bool usbd_open_edpt_pair(uint8_t rhport, uint8_t const* p_desc, uint8_t ep_count, uint8_t xfer_type, uint8_t* ep_out, uint8_t* ep_in);
void usbd_defer_func( osal_task_func_t func, void* param, bool in_isr );

#endif





/*************************************************************************
**************************************************************************
**************************************************************************/
#if dcd_h

typedef enum
{
  DCD_EVENT_INVALID = 0,
  DCD_EVENT_BUS_RESET,//复位
  DCD_EVENT_UNPLUGGED,
  DCD_EVENT_SOF,
  DCD_EVENT_SUSPEND, //挂起TODO LPM Sleep L1 support
  DCD_EVENT_RESUME,  //检测到恢复

  DCD_EVENT_SETUP_RECEIVED,
  DCD_EVENT_XFER_COMPLETE,

  // Not an DCD event, just a convenient way to defer ISR function
  USBD_EVENT_FUNC_CALL,

  DCD_EVENT_COUNT
} dcd_eventid_t;

typedef struct TU_ATTR_ALIGNED(4)
{
  uint8_t rhport;
  uint8_t event_id;

  union
  {
    // BUS RESET
    struct {
      tusb_speed_t speed;
    } bus_reset;

    // SETUP_RECEIVED
    tusb_control_request_t setup_received;

    // XFER_COMPLETE
    struct {
      uint8_t  ep_addr;
      uint8_t  result;
      uint32_t len;
    }xfer_complete;

    // USBD_EVENT_FUNC_CALL
    struct {
      void (*func) (void*);
      void* param;
    }func_call;
  };
} dcd_event_t;

//TU_VERIFY_STATIC(sizeof(dcd_event_t) <= 12, "size is not correct");

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
void dcd_init       (uint8_t rhport);

// Interrupt Handler
void usb_handler(uint8_t rhport);

// Enable device interrupt
void dcd_int_enable (uint8_t rhport);

// Disable device interrupt
void dcd_int_disable(uint8_t rhport);

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr);

// Wake up host
void dcd_remote_wakeup(uint8_t rhport);

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport) TU_ATTR_WEAK;

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport) TU_ATTR_WEAK;

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request) TU_ATTR_WEAK;

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open        (uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc);

// Close an endpoint.
// Since it is weak, caller must TU_ASSERT this function's existence before calling it.
void dcd_edpt_close        (uint8_t rhport, uint8_t ep_addr) TU_ATTR_WEAK;

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer        (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes);

// Stall endpoint
void dcd_edpt_stall       (uint8_t rhport, uint8_t ep_addr);

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr);

//--------------------------------------------------------------------+
// Event API (Implemented by device stack)
//--------------------------------------------------------------------+

// Called by DCD to notify device stack
extern void dcd_event_handler(dcd_event_t const * event, bool in_isr);

// helper to send bus signal event
extern void dcd_event_bus_signal (uint8_t rhport, dcd_eventid_t eid, bool in_isr);

// helper to send bus reset event
extern void dcd_event_bus_reset (uint8_t rhport, tusb_speed_t speed, bool in_isr);

// helper to send setup received
extern void dcd_event_setup_received(uint8_t rhport, uint8_t const * setup, bool in_isr);

// helper to send transfer complete event
extern void dcd_event_xfer_complete (uint8_t rhport, uint8_t ep_addr, uint32_t xferred_bytes, uint8_t result, bool in_isr);
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if msc_device_h
//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

#if !defined(CFG_TUD_MSC_EP_BUFSIZE) & defined(CFG_TUD_MSC_BUFSIZE)
  // TODO warn user to use new name later on
  // #warning CFG_TUD_MSC_BUFSIZE is renamed to CFG_TUD_MSC_EP_BUFSIZE, please update to use the new name
  #define CFG_TUD_MSC_EP_BUFSIZE  CFG_TUD_MSC_BUFSIZE
#endif

#ifndef CFG_TUD_MSC_EP_BUFSIZE
  #error CFG_TUD_MSC_EP_BUFSIZE must be defined, value of a block size should work well, the more the better
#endif

TU_VERIFY_STATIC(CFG_TUD_MSC_EP_BUFSIZE < UINT16_MAX, "Size is not correct");

/** \addtogroup ClassDriver_MSC
 *  @{
 * \defgroup MSC_Device Device
 *  @{ */

bool tud_msc_set_sense(uint8_t lun, uint8_t sense_key, uint8_t add_sense_code, uint8_t add_sense_qualifier);

//--------------------------------------------------------------------+
// Application Callbacks (WEAK is optional)
//--------------------------------------------------------------------+

/**
 * Invoked when received \ref SCSI_CMD_READ_10 command
 * \param[in]   lun         Logical unit number
 * \param[in]   lba         Logical Block Address to be read
 * \param[in]   offset      Byte offset from LBA
 * \param[out]  buffer      Buffer which application need to update with the response data.
 * \param[in]   bufsize     Requested bytes
 *
 * \return      Number of byte read, if it is less than requested bytes by \a \b bufsize. Tinyusb will transfer
 *              this amount first and invoked this again for remaining data.
 *
 * \retval      zero        Indicate application is not ready yet to response e.g disk I/O is not complete.
 *                          tinyusb will invoke this callback with the same parameters again some time later.
 *
 * \retval      negative    Indicate error e.g reading disk I/O. tinyusb will \b STALL the corresponding
 *                          endpoint and return failed status in command status wrapper phase.
 */
int32_t tud_msc_read10_cb (uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize);

/**
 * Invoked when received \ref SCSI_CMD_WRITE_10 command
 * \param[in]   lun         Logical unit number
 * \param[in]   lba         Logical Block Address to be write
 * \param[in]   offset      Byte offset from LBA
 * \param[out]  buffer      Buffer which holds written data.
 * \param[in]   bufsize     Requested bytes
 *
 * \return      Number of byte written, if it is less than requested bytes by \a \b bufsize. Tinyusb will proceed with
 *              other work and invoked this again with adjusted parameters.
 *
 * \retval      zero        Indicate application is not ready yet e.g disk I/O is not complete.
 *                          Tinyusb will invoke this callback with the same parameters again some time later.
 *
 * \retval      negative    Indicate error writing disk I/O. Tinyusb will \b STALL the corresponding
 *                          endpoint and return failed status in command status wrapper phase.
 */
int32_t tud_msc_write10_cb (uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize);

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]);

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun);

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size);

/**
 * Invoked when received an SCSI command not in built-in list below.
 * - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, TEST_UNIT_READY, START_STOP_UNIT, MODE_SENSE6, REQUEST_SENSE
 * - READ10 and WRITE10 has their own callbacks
 *
 * \param[in]   lun         Logical unit number
 * \param[in]   scsi_cmd    SCSI command contents which application must examine to response accordingly
 * \param[out]  buffer      Buffer for SCSI Data Stage.
 *                            - For INPUT: application must fill this with response.
 *                            - For OUTPUT it holds the Data from host
 * \param[in]   bufsize     Buffer's length.
 *
 * \return      Actual bytes processed, can be zero for no-data command.
 * \retval      negative    Indicate error e.g unsupported command, tinyusb will \b STALL the corresponding
 *                          endpoint and return failed status in command status wrapper phase.
 */
int32_t tud_msc_scsi_cb (uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize);

/*------------- Optional callbacks -------------*/

// Invoked when received GET_MAX_LUN request, required for multiple LUNs implementation
TU_ATTR_WEAK uint8_t tud_msc_get_maxlun_cb(void);

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
TU_ATTR_WEAK bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject);

// Invoked when Read10 command is complete
TU_ATTR_WEAK void tud_msc_read10_complete_cb(uint8_t lun);

// Invoke when Write10 command is complete, can be used to flush flash caching
TU_ATTR_WEAK void tud_msc_write10_complete_cb(uint8_t lun);

// Invoked when command in tud_msc_scsi_cb is complete
TU_ATTR_WEAK void tud_msc_scsi_complete_cb(uint8_t lun, uint8_t const scsi_cmd[16]);

// Hook to make a mass storage device read-only. TODO remove
bool tud_msc_is_writable_cb(uint8_t lun);

/** @} */
/** @} */

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void     mscd_init             (void);
void     mscd_reset            (uint8_t rhport);
uint16_t mscd_open             (uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool     mscd_control_request  (uint8_t rhport, tusb_control_request_t* p_request);
bool     mscd_control_complete (uint8_t rhport, tusb_control_request_t const * p_request);
bool     mscd_xfer_cb          (uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);

#endif



/*************************************************************************
**************************************************************************
**************************************************************************/
#if cdc_h

/** \defgroup ClassDriver_CDC_Common Common Definitions
 *  @{ */

// TODO remove
/// CDC Pipe ID, used to indicate which pipe the API is addressing to (Notification, Out, In)
typedef enum
{
  CDC_PIPE_NOTIFICATION , ///< Notification pipe
  CDC_PIPE_DATA_IN      , ///< Data in pipe
  CDC_PIPE_DATA_OUT     , ///< Data out pipe
  CDC_PIPE_ERROR        , ///< Invalid Pipe ID
}cdc_pipeid_t;

//--------------------------------------------------------------------+
// CDC Communication Interface Class
//--------------------------------------------------------------------+

/// Communication Interface Subclass Codes
typedef enum
{
  CDC_COMM_SUBCLASS_DIRECT_LINE_CONTROL_MODEL = 0x01  , ///< Direct Line Control Model  [USBPSTN1.2]
  CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL            , ///< Abstract Control Model  [USBPSTN1.2]
  CDC_COMM_SUBCLASS_TELEPHONE_CONTROL_MODEL           , ///< Telephone Control Model  [USBPSTN1.2]
  CDC_COMM_SUBCLASS_MULTICHANNEL_CONTROL_MODEL        , ///< Multi-Channel Control Model  [USBISDN1.2]
  CDC_COMM_SUBCLASS_CAPI_CONTROL_MODEL                , ///< CAPI Control Model  [USBISDN1.2]
  CDC_COMM_SUBCLASS_ETHERNET_NETWORKING_CONTROL_MODEL , ///< Ethernet Networking Control Model  [USBECM1.2]
  CDC_COMM_SUBCLASS_ATM_NETWORKING_CONTROL_MODEL      , ///< ATM Networking Control Model  [USBATM1.2]
  CDC_COMM_SUBCLASS_WIRELESS_HANDSET_CONTROL_MODEL    , ///< Wireless Handset Control Model  [USBWMC1.1]
  CDC_COMM_SUBCLASS_DEVICE_MANAGEMENT                 , ///< Device Management  [USBWMC1.1]
  CDC_COMM_SUBCLASS_MOBILE_DIRECT_LINE_MODEL          , ///< Mobile Direct Line Model  [USBWMC1.1]
  CDC_COMM_SUBCLASS_OBEX                              , ///< OBEX  [USBWMC1.1]
  CDC_COMM_SUBCLASS_ETHERNET_EMULATION_MODEL            ///< Ethernet Emulation Model  [USBEEM1.0]
} cdc_comm_sublcass_type_t;

/// Communication Interface Protocol Codes
typedef enum
{
  CDC_COMM_PROTOCOL_NONE                   = 0x00 , ///< No specific protocol
  CDC_COMM_PROTOCOL_ATCOMMAND                     , ///< AT Commands: V.250 etc
  CDC_COMM_PROTOCOL_ATCOMMAND_PCCA_101            , ///< AT Commands defined by PCCA-101
  CDC_COMM_PROTOCOL_ATCOMMAND_PCCA_101_AND_ANNEXO , ///< AT Commands defined by PCCA-101 & Annex O
  CDC_COMM_PROTOCOL_ATCOMMAND_GSM_707             , ///< AT Commands defined by GSM 07.07
  CDC_COMM_PROTOCOL_ATCOMMAND_3GPP_27007          , ///< AT Commands defined by 3GPP 27.007
  CDC_COMM_PROTOCOL_ATCOMMAND_CDMA                , ///< AT Commands defined by TIA for CDMA
  CDC_COMM_PROTOCOL_ETHERNET_EMULATION_MODEL        ///< Ethernet Emulation Model
} cdc_comm_protocol_type_t;

//------------- SubType Descriptor in COMM Functional Descriptor -------------//
/// Communication Interface SubType Descriptor
typedef enum
{
  CDC_FUNC_DESC_HEADER                                           = 0x00 , ///< Header Functional Descriptor, which marks the beginning of the concatenated set of functional descriptors for the interface.
  CDC_FUNC_DESC_CALL_MANAGEMENT                                  = 0x01 , ///< Call Management Functional Descriptor.
  CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT                      = 0x02 , ///< Abstract Control Management Functional Descriptor.
  CDC_FUNC_DESC_DIRECT_LINE_MANAGEMENT                           = 0x03 , ///< Direct Line Management Functional Descriptor.
  CDC_FUNC_DESC_TELEPHONE_RINGER                                 = 0x04 , ///< Telephone Ringer Functional Descriptor.
  CDC_FUNC_DESC_TELEPHONE_CALL_AND_LINE_STATE_REPORTING_CAPACITY = 0x05 , ///< Telephone Call and Line State Reporting Capabilities Functional Descriptor.
  CDC_FUNC_DESC_UNION                                            = 0x06 , ///< Union Functional Descriptor
  CDC_FUNC_DESC_COUNTRY_SELECTION                                = 0x07 , ///< Country Selection Functional Descriptor
  CDC_FUNC_DESC_TELEPHONE_OPERATIONAL_MODES                      = 0x08 , ///< Telephone Operational ModesFunctional Descriptor
  CDC_FUNC_DESC_USB_TERMINAL                                     = 0x09 , ///< USB Terminal Functional Descriptor
  CDC_FUNC_DESC_NETWORK_CHANNEL_TERMINAL                         = 0x0A , ///< Network Channel Terminal Descriptor
  CDC_FUNC_DESC_PROTOCOL_UNIT                                    = 0x0B , ///< Protocol Unit Functional Descriptor
  CDC_FUNC_DESC_EXTENSION_UNIT                                   = 0x0C , ///< Extension Unit Functional Descriptor
  CDC_FUNC_DESC_MULTICHANEL_MANAGEMENT                           = 0x0D , ///< Multi-Channel Management Functional Descriptor
  CDC_FUNC_DESC_CAPI_CONTROL_MANAGEMENT                          = 0x0E , ///< CAPI Control Management Functional Descriptor
  CDC_FUNC_DESC_ETHERNET_NETWORKING                              = 0x0F , ///< Ethernet Networking Functional Descriptor
  CDC_FUNC_DESC_ATM_NETWORKING                                   = 0x10 , ///< ATM Networking Functional Descriptor
  CDC_FUNC_DESC_WIRELESS_HANDSET_CONTROL_MODEL                   = 0x11 , ///< Wireless Handset Control Model Functional Descriptor
  CDC_FUNC_DESC_MOBILE_DIRECT_LINE_MODEL                         = 0x12 , ///< Mobile Direct Line Model Functional Descriptor
  CDC_FUNC_DESC_MOBILE_DIRECT_LINE_MODEL_DETAIL                  = 0x13 , ///< MDLM Detail Functional Descriptor
  CDC_FUNC_DESC_DEVICE_MANAGEMENT_MODEL                          = 0x14 , ///< Device Management Model Functional Descriptor
  CDC_FUNC_DESC_OBEX                                             = 0x15 , ///< OBEX Functional Descriptor
  CDC_FUNC_DESC_COMMAND_SET                                      = 0x16 , ///< Command Set Functional Descriptor
  CDC_FUNC_DESC_COMMAND_SET_DETAIL                               = 0x17 , ///< Command Set Detail Functional Descriptor
  CDC_FUNC_DESC_TELEPHONE_CONTROL_MODEL                          = 0x18 , ///< Telephone Control Model Functional Descriptor
  CDC_FUNC_DESC_OBEX_SERVICE_IDENTIFIER                          = 0x19   ///< OBEX Service Identifier Functional Descriptor
}cdc_func_desc_type_t;

//--------------------------------------------------------------------+
// CDC Data Interface Class
//--------------------------------------------------------------------+

// SUBCLASS code of Data Interface is not used and should/must be zero
/// Data Interface Protocol Codes
typedef enum{
  CDC_DATA_PROTOCOL_ISDN_BRI                               = 0x30, ///< Physical interface protocol for ISDN BRI
  CDC_DATA_PROTOCOL_HDLC                                   = 0x31, ///< HDLC
  CDC_DATA_PROTOCOL_TRANSPARENT                            = 0x32, ///< Transparent
  CDC_DATA_PROTOCOL_Q921_MANAGEMENT                        = 0x50, ///< Management protocol for Q.921 data link protocol
  CDC_DATA_PROTOCOL_Q921_DATA_LINK                         = 0x51, ///< Data link protocol for Q.931
  CDC_DATA_PROTOCOL_Q921_TEI_MULTIPLEXOR                   = 0x52, ///< TEI-multiplexor for Q.921 data link protocol
  CDC_DATA_PROTOCOL_V42BIS_DATA_COMPRESSION                = 0x90, ///< Data compression procedures
  CDC_DATA_PROTOCOL_EURO_ISDN                              = 0x91, ///< Euro-ISDN protocol control
  CDC_DATA_PROTOCOL_V24_RATE_ADAPTION_TO_ISDN              = 0x92, ///< V.24 rate adaptation to ISDN
  CDC_DATA_PROTOCOL_CAPI_COMMAND                           = 0x93, ///< CAPI Commands
  CDC_DATA_PROTOCOL_HOST_BASED_DRIVER                      = 0xFD, ///< Host based driver. Note: This protocol code should only be used in messages between host and device to identify the host driver portion of a protocol stack.
  CDC_DATA_PROTOCOL_IN_PROTOCOL_UNIT_FUNCTIONAL_DESCRIPTOR = 0xFE  ///< The protocol(s) are described using a ProtocolUnit Functional Descriptors on Communications Class Interface
}cdc_data_protocol_type_t;

//--------------------------------------------------------------------+
// Management Element Request (Control Endpoint)
//--------------------------------------------------------------------+

/// Communication Interface Management Element Request Codes
typedef enum
{
  CDC_REQUEST_SEND_ENCAPSULATED_COMMAND                    = 0x00, ///< is used to issue a command in the format of the supported control protocol of the Communications Class interface
  CDC_REQUEST_GET_ENCAPSULATED_RESPONSE                    = 0x01, ///< is used to request a response in the format of the supported control protocol of the Communications Class interface.

  CDC_REQUEST_SET_COMM_FEATURE                             = 0x02,
  CDC_REQUEST_GET_COMM_FEATURE                             = 0x03,
  CDC_REQUEST_CLEAR_COMM_FEATURE                           = 0x04,

  CDC_REQUEST_SET_AUX_LINE_STATE                           = 0x10,
  CDC_REQUEST_SET_HOOK_STATE                               = 0x11,
  CDC_REQUEST_PULSE_SETUP                                  = 0x12,
  CDC_REQUEST_SEND_PULSE                                   = 0x13,
  CDC_REQUEST_SET_PULSE_TIME                               = 0x14,
  CDC_REQUEST_RING_AUX_JACK                                = 0x15,

  CDC_REQUEST_SET_LINE_CODING                              = 0x20,
  CDC_REQUEST_GET_LINE_CODING                              = 0x21,
  CDC_REQUEST_SET_CONTROL_LINE_STATE                       = 0x22,
  CDC_REQUEST_SEND_BREAK                                   = 0x23,

  CDC_REQUEST_SET_RINGER_PARMS                             = 0x30,
  CDC_REQUEST_GET_RINGER_PARMS                             = 0x31,
  CDC_REQUEST_SET_OPERATION_PARMS                          = 0x32,
  CDC_REQUEST_GET_OPERATION_PARMS                          = 0x33,
  CDC_REQUEST_SET_LINE_PARMS                               = 0x34,
  CDC_REQUEST_GET_LINE_PARMS                               = 0x35,
  CDC_REQUEST_DIAL_DIGITS                                  = 0x36,
  CDC_REQUEST_SET_UNIT_PARAMETER                           = 0x37,
  CDC_REQUEST_GET_UNIT_PARAMETER                           = 0x38,
  CDC_REQUEST_CLEAR_UNIT_PARAMETER                         = 0x39,
  CDC_REQUEST_GET_PROFILE                                  = 0x3A,

  CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS               = 0x40,
  CDC_REQUEST_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x41,
  CDC_REQUEST_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x42,
  CDC_REQUEST_SET_ETHERNET_PACKET_FILTER                   = 0x43,
  CDC_REQUEST_GET_ETHERNET_STATISTIC                       = 0x44,

  CDC_REQUEST_SET_ATM_DATA_FORMAT                          = 0x50,
  CDC_REQUEST_GET_ATM_DEVICE_STATISTICS                    = 0x51,
  CDC_REQUEST_SET_ATM_DEFAULT_VC                           = 0x52,
  CDC_REQUEST_GET_ATM_VC_STATISTICS                        = 0x53,

  CDC_REQUEST_MDLM_SEMANTIC_MODEL                          = 0x60,
}cdc_management_request_t;

//--------------------------------------------------------------------+
// Management Elemenent Notification (Notification Endpoint)
//--------------------------------------------------------------------+

/// Communication Interface Management Element Notification Codes
typedef enum
{
  NETWORK_CONNECTION               = 0x00, ///< This notification allows the device to notify the host about network connection status.
  RESPONSE_AVAILABLE               = 0x01, ///< This notification allows the device to notify the hostthat a response is available. This response can be retrieved with a subsequent \ref CDC_REQUEST_GET_ENCAPSULATED_RESPONSE request.

  AUX_JACK_HOOK_STATE              = 0x08,
  RING_DETECT                      = 0x09,

  SERIAL_STATE                     = 0x20,

  CALL_STATE_CHANGE                = 0x28,
  LINE_STATE_CHANGE                = 0x29,
  CONNECTION_SPEED_CHANGE          = 0x2A, ///< This notification allows the device to inform the host-networking driver that a change in either the upstream or the downstream bit rate of the connection has occurred
  MDLM_SEMANTIC_MODEL_NOTIFICATION = 0x40,
}cdc_notification_request_t;

//--------------------------------------------------------------------+
// Class Specific Functional Descriptor (Communication Interface)
//--------------------------------------------------------------------+

/// Header Functional Descriptor (Communication Interface)
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType one of above CDC_FUNC_DESC_
  uint16_t bcdCDC            ; ///< CDC release number in Binary-Coded Decimal
}cdc_desc_func_header_t;

/// Union Functional Descriptor (Communication Interface)
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength                  ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType          ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType       ; ///< Descriptor SubType one of above CDC_FUCN_DESC_
  uint8_t bControlInterface        ; ///< Interface number of Communication Interface
  uint8_t bSubordinateInterface    ; ///< Array of Interface number of Data Interface
}cdc_desc_func_union_t;

#define cdc_desc_func_union_n_t(no_slave)\
 struct TU_ATTR_PACKED { \
  uint8_t bLength                         ;\
  uint8_t bDescriptorType                 ;\
  uint8_t bDescriptorSubType              ;\
  uint8_t bControlInterface               ;\
  uint8_t bSubordinateInterface[no_slave] ;\
}

/// Country Selection Functional Descriptor (Communication Interface)
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength             ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType     ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType  ; ///< Descriptor SubType one of above CDC_FUCN_DESC_
  uint8_t iCountryCodeRelDate ; ///< Index of a string giving the release date for the implemented ISO 3166 Country Codes.
  uint16_t wCountryCode       ; ///< Country code in the format as defined in [ISO3166], release date as specified inoffset 3 for the first supported country.
}cdc_desc_func_country_selection_t;

#define cdc_desc_func_country_selection_n_t(no_country) \
  struct TU_ATTR_PACKED {\
  uint8_t bLength                   ;\
  uint8_t bDescriptorType           ;\
  uint8_t bDescriptorSubType        ;\
  uint8_t iCountryCodeRelDate       ;\
  uint16_t wCountryCode[no_country] ;\
}

//--------------------------------------------------------------------+
// PUBLIC SWITCHED TELEPHONE NETWORK (PSTN) SUBCLASS
//--------------------------------------------------------------------+

/// \brief Call Management Functional Descriptor
/// \details This functional descriptor describes the processing of calls for the Communications Class interface.
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType one of above CDC_FUCN_DESC_

  struct {
    uint8_t handle_call    : 1; ///< 0 - Device sends/receives call management information only over the Communications Class interface. 1 - Device can send/receive call management information over a Data Class interface.
    uint8_t send_recv_call : 1; ///< 0 - Device does not handle call management itself. 1 - Device handles call management itself.
    uint8_t : 0;
  } bmCapabilities;

  uint8_t bDataInterface;
}cdc_desc_func_call_management_t;


typedef struct TU_ATTR_PACKED
{
  uint8_t support_comm_request                    : 1; ///< Device supports the request combination of Set_Comm_Feature, Clear_Comm_Feature, and Get_Comm_Feature.
  uint8_t support_line_request                    : 1; ///< Device supports the request combination of Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State.
  uint8_t support_send_break                      : 1; ///< Device supports the request Send_Break
  uint8_t support_notification_network_connection : 1; ///< Device supports the notification Network_Connection.
  uint8_t : 0;
}cdc_acm_capability_t;

TU_VERIFY_STATIC(sizeof(cdc_acm_capability_t) == 1, "mostly problem with compiler");

/// \brief Abstract Control Management Functional Descriptor
/// \details This functional descriptor describes the commands supported by by the Communications Class interface with SubClass code of \ref CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength                  ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType          ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType       ; ///< Descriptor SubType one of above CDC_FUCN_DESC_
  cdc_acm_capability_t bmCapabilities ;
}cdc_desc_func_acm_t;

/// \brief Direct Line Management Functional Descriptor
/// \details This functional descriptor describes the commands supported by the Communications Class interface with SubClass code of \ref CDC_FUNC_DESC_DIRECT_LINE_MANAGEMENT
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType one of above CDC_FUCN_DESC_
  struct {
    uint8_t require_pulse_setup   : 1; ///< Device requires extra Pulse_Setup request during pulse dialing sequence to disengage holding circuit.
    uint8_t support_aux_request   : 1; ///< Device supports the request combination of Set_Aux_Line_State, Ring_Aux_Jack, and notification Aux_Jack_Hook_State.
    uint8_t support_pulse_request : 1; ///< Device supports the request combination of Pulse_Setup, Send_Pulse, and Set_Pulse_Time.
    uint8_t : 0;
  } bmCapabilities;
}cdc_desc_func_direct_line_management_t;

/// \brief Telephone Ringer Functional Descriptor
/// \details The Telephone Ringer functional descriptor describes the ringer capabilities supported by the Communications Class interface,
/// with the SubClass code of \ref CDC_COMM_SUBCLASS_TELEPHONE_CONTROL_MODEL
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType one of above CDC_FUCN_DESC_
  uint8_t bRingerVolSteps    ;
  uint8_t bNumRingerPatterns ;
}cdc_desc_func_telephone_ringer_t;

/// \brief Telephone Operational Modes Functional Descriptor
/// \details The Telephone Operational Modes functional descriptor describes the operational modes supported by
/// the Communications Class interface, with the SubClass code of \ref CDC_COMM_SUBCLASS_TELEPHONE_CONTROL_MODEL
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType one of above CDC_FUCN_DESC_
  struct {
    uint8_t simple_mode           : 1;
    uint8_t standalone_mode       : 1;
    uint8_t computer_centric_mode : 1;
    uint8_t : 0;
  } bmCapabilities;
}cdc_desc_func_telephone_operational_modes_t;

/// \brief Telephone Call and Line State Reporting Capabilities Descriptor
/// \details The Telephone Call and Line State Reporting Capabilities functional descriptor describes the abilities of a
/// telephone device to report optional call and line states.
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType one of above CDC_FUCN_DESC_
  struct {
    uint32_t interrupted_dialtone   : 1; ///< 0 : Reports only dialtone (does not differentiate between normal and interrupted dialtone). 1 : Reports interrupted dialtone in addition to normal dialtone
    uint32_t ringback_busy_fastbusy : 1; ///< 0 : Reports only dialing state. 1 : Reports ringback, busy, and fast busy states.
    uint32_t caller_id              : 1; ///< 0 : Does not report caller ID. 1 : Reports caller ID information.
    uint32_t incoming_distinctive   : 1; ///< 0 : Reports only incoming ringing. 1 : Reports incoming distinctive ringing patterns.
    uint32_t dual_tone_multi_freq   : 1; ///< 0 : Cannot report dual tone multi-frequency (DTMF) digits input remotely over the telephone line. 1 : Can report DTMF digits input remotely over the telephone line.
    uint32_t line_state_change      : 1; ///< 0 : Does not support line state change notification. 1 : Does support line state change notification
    uint32_t : 0;
  } bmCapabilities;
}cdc_desc_func_telephone_call_state_reporting_capabilities_t;

static inline uint8_t cdc_functional_desc_typeof(uint8_t const * p_desc)
{
  return p_desc[2];
}

//--------------------------------------------------------------------+
// Requests
//--------------------------------------------------------------------+
typedef struct TU_ATTR_PACKED
{
  uint32_t bit_rate;
  uint8_t  stop_bits; ///< 0: 1 stop bit - 1: 1.5 stop bits - 2: 2 stop bits
  uint8_t  parity;    ///< 0: None - 1: Odd - 2: Even - 3: Mark - 4: Space
  uint8_t  data_bits; ///< can be 5, 6, 7, 8 or 16
} cdc_line_coding_t;

TU_VERIFY_STATIC(sizeof(cdc_line_coding_t) == 7, "size is not correct");

typedef struct TU_ATTR_PACKED
{
  uint16_t dte_is_present : 1; ///< Indicates to DCE if DTE is presentor not. This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR.
  uint16_t half_duplex_carrier_control : 1;
  uint16_t : 14;
} cdc_line_control_state_t;

TU_VERIFY_STATIC(sizeof(cdc_line_control_state_t) == 2, "size is not correct");
#endif

/*************************************************************************
**************************************************************************
**************************************************************************/
#if cdc_device_h
//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+
#if !defined(CFG_TUD_CDC_EP_BUFSIZE) && defined(CFG_TUD_CDC_EPSIZE)
  #warning CFG_TUD_CDC_EPSIZE is renamed to CFG_TUD_CDC_EP_BUFSIZE, please update to use the new name
  #define CFG_TUD_CDC_EP_BUFSIZE    CFG_TUD_CDC_EPSIZE
#endif

#ifndef CFG_TUD_CDC_EP_BUFSIZE
  #define CFG_TUD_CDC_EP_BUFSIZE    (TUD_OPT_HIGH_SPEED ? 512 : 64)
#endif

/** \addtogroup CDC_Serial Serial
 *  @{
 *  \defgroup   CDC_Serial_Device Device
 *  @{ */

//--------------------------------------------------------------------+
// Application API (Multiple Ports)
// CFG_TUD_CDC > 1
//--------------------------------------------------------------------+

// Check if terminal is connected to this port
bool     tud_cdc_n_connected       (uint8_t itf);

// Get current line state. Bit 0:  DTR (Data Terminal Ready), Bit 1: RTS (Request to Send)
uint8_t  tud_cdc_n_get_line_state  (uint8_t itf);

// Get current line encoding: bit rate, stop bits parity etc ..
void     tud_cdc_n_get_line_coding (uint8_t itf, cdc_line_coding_t* coding);

// Set special character that will trigger tud_cdc_rx_wanted_cb() callback on receiving
void     tud_cdc_n_set_wanted_char (uint8_t itf, char wanted);

// Force sending data if possible, return number of forced bytes
uint32_t cdc_write (uint8_t itf);


//--------------------------------------------------------------------+
// Application API (Single Port)
//--------------------------------------------------------------------+
static inline bool     tud_cdc_connected       (void);
static inline uint8_t  tud_cdc_get_line_state  (void);
static inline void     tud_cdc_get_line_coding (cdc_line_coding_t* coding);
static inline void     tud_cdc_set_wanted_char (char wanted);


//--------------------------------------------------------------------+
// Application Callback API (weak is optional)
//--------------------------------------------------------------------+

// Invoked when received new data
void tud_cdc_rx_cb(uint8_t itf,const void* p_data,uint16_t count);

// Invoked when received `wanted_char`
TU_ATTR_WEAK void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char);

// Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
TU_ATTR_WEAK void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts);

// Invoked when line coding is change via SET_LINE_CODING
TU_ATTR_WEAK void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding);


//--------------------------------------------------------------------+
// INTERNAL USBD-CLASS DRIVER API
//--------------------------------------------------------------------+
void     cdcd_init             (void);
void     cdcd_reset            (uint8_t rhport);
uint16_t cdcd_open             (uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool     cdcd_control_request  (uint8_t rhport, tusb_control_request_t * request);
bool     cdcd_control_complete (uint8_t rhport, tusb_control_request_t const * request);
bool     cdcd_xfer_cb          (uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

#endif






/*************************************************************************
**************************************************************************
**************************************************************************/



#endif

#ifdef __cplusplus
 }
#endif	 