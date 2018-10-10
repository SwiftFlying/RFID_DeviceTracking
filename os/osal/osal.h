/**************************************************************************************************
* Filename:             osal.h
* Revised:        
* Revision:       
* Description:    
**************************************************************************************************/
#ifndef OSAL_H
#define OSAL_H

#ifdef __cplusplus

extern "C"
{
  
#endif
/*********************************************************************
* INCLUDES
*/
  
#include "stm32l0xx.h"

/*********************************************************************
* MACROS
*/
#define TASK_NO_TASK      0xFF
  
/*** Generic Status Return Values ***/   
#define SUCCESS                   0x00
#define FAILURE                   0x01
#define INVALIDPARAMETER          0x02
#define INVALID_TASK              0x03
#define MSG_BUFFER_NOT_AVAIL      0x04
#define INVALID_MSG_POINTER       0x05
#define INVALID_EVENT_ID          0x06
#define INVALID_INTERRUPT_ID      0x07
#define NO_TIMER_AVAIL            0x08
#define NV_ITEM_UNINIT            0x09
#define NV_OPER_FAILED            0x0A
#define INVALID_MEM_SIZE          0x0B
#define NV_BAD_ITEM_LEN           0x0C
  
#define OSAL_MSG_ID(msg_ptr)      ((osal_msg_hdr_t *) (msg_ptr) - 1)->dest_id
#define st(x)      do { x } while (__LINE__ == -1)
  
  
/*********************************************************************
* CONSTANTS
*/
#ifndef false
  #define false 0
#endif

#ifndef true
  #define true 1
#endif

#ifndef CONST
  #define CONST const
#endif

#ifndef GENERIC
  #define GENERIC
#endif
//////////////////////  
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif
  
#define BUILD_uint32_t(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define BUILD_uint16_t(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define OSAL_MSG_NEXT(msg_ptr)      ((osal_msg_hdr_t *) (msg_ptr) - 1)->next

#define HAL_ENABLE_INTERRUPTS()         st( __enable_irq(); )
#define HAL_DISABLE_INTERRUPTS()        st( __disable_irq(); )
//#define HAL_INTERRUPTS_ARE_ENABLED()    ()
#define HAL_ENTER_CRITICAL_SECTION(x)   st( x = __get_PRIMASK();  HAL_DISABLE_INTERRUPTS(); )
#define HAL_EXIT_CRITICAL_SECTION(x)    st( __set_PRIMASK(x);HAL_ENABLE_INTERRUPTS(); )
#define HAL_CRITICAL_STATEMENT(x)       st( halIntState_t _s; HAL_ENTER_CRITICAL_SECTION(_s); x; HAL_EXIT_CRITICAL_SECTION(_s); )
/*** Task Synchronization  ***/

  /*
   * Set a Task Event
   */  
#define SYS_EVENT_MSG               0x8000  // A message is waiting event
#define INTS_ALL    								0xFF
/*********************************************************************
* TYPEDEFS
*/

/////////////////////////
typedef uint32_t           halDataAlign_t;
///////////////////////////
typedef struct
{
  void   *next;
  uint16_t len;
  uint8_t  dest_id;
} osal_msg_hdr_t;

typedef struct
{
  uint8_t  event;
  uint8_t  status;
} osal_event_hdr_t;

typedef void * osal_msg_q_t;
extern osal_msg_q_t osal_qHead;


typedef uint32_t halIntState_t;
//////////
typedef uint16_t (*pTaskEventHandlerFn)( uint8_t task_id, uint16_t event );
/*********************************************************************
* GLOBAL VARIABLES
*/
extern const pTaskEventHandlerFn tasksArr[];
extern const uint8_t tasksCnt;
extern uint16_t *tasksEvents;
/*********************************************************************
* FUNCTIONS
*/
extern void osal_msg_enqueue( osal_msg_q_t *q_ptr, void *msg_ptr );
extern uint8_t osal_set_event( uint8_t task_id, uint16_t event_flag );
extern int osal_strlen( char *pString );
extern void *osal_memcpy( void *dst, const void GENERIC *src, unsigned int len );
extern void *osal_revmemcpy( void *dst, const void GENERIC *src, unsigned int len );
extern void *osal_memdup( const void GENERIC *src, unsigned int len );
extern uint8_t osal_memcmp( const void GENERIC *src1, const void GENERIC *src2, unsigned int len );
extern void *osal_memset( void *dest, uint8_t value, int len );
extern uint16_t osal_build_uint16_t( uint8_t *swapped );
extern uint32_t osal_build_uint32_t( uint8_t *swapped, uint8_t len );
extern uint16_t osal_rand( void );
extern uint8_t * osal_msg_allocate( uint16_t len );
extern uint8_t osal_msg_deallocate( uint8_t *msg_ptr );
extern uint8_t osal_msg_send( uint8_t destination_task, uint8_t *msg_ptr );
extern uint8_t *osal_msg_receive( uint8_t task_id );
extern osal_event_hdr_t *osal_msg_find(uint8_t task_id, uint8_t event);
extern void osal_msg_enqueue( osal_msg_q_t *q_ptr, void *msg_ptr );
extern void *osal_msg_dequeue( osal_msg_q_t *q_ptr );
extern void osal_msg_push( osal_msg_q_t *q_ptr, void *msg_ptr );
extern void osal_msg_extract( osal_msg_q_t *q_ptr, void *msg_ptr, void *prev_ptr );
extern uint8_t osal_msg_enqueue_max( osal_msg_q_t *q_ptr, void *msg_ptr, uint8_t max );
extern uint8_t osal_clear_event( uint8_t task_id, uint16_t event_flag );
extern uint8_t osal_int_enable( uint8_t interrupt_id );
extern uint8_t osal_int_disable( uint8_t interrupt_id );
extern uint8_t osal_init_system( void );
extern void osal_start_system( void );
extern void osalTimeUpdate( void );

extern void osalInitTasks( void );


#define OSALMEM_METRICS 1

/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* FUNCTIONS
*/
 /*
  * Allocate a block of memory.
  */
void osal_mem_init( void );
void osal_mem_kick( void );
extern void *osal_mem_alloc( uint16_t size );
void osal_mem_free( void *ptr );
uint16_t osal_heap_block_max( void );
uint16_t osal_heap_block_cnt( void );
uint16_t osal_heap_block_free( void );
uint16_t osal_heap_mem_used( void );
uint16_t osal_heap_mem_max( void );


typedef struct mutex_struct
{
 uint32_t mutex_value;
 struct mutex_struct *next_mutex;
}osal_mutex_t;

osal_mutex_t* osal_mutex_create(void);
void osal_mutex_delete(osal_mutex_t** mutex);
void osal_mutex_take(osal_mutex_t** mutex,uint32_t mutex_overtime);
uint32_t osal_mutex_check(osal_mutex_t* mutex);
void osal_mutex_release(osal_mutex_t** mutex);
void osal_mutex_updata(void);


extern void osalTimerUpdate( uint16_t updateTime );  
extern void osalTimerInit( void );
extern uint8_t osal_start_timerEx( uint8_t taskID, uint16_t event_id, uint16_t timeout_value );
extern uint8_t osal_stop_timerEx( uint8_t task_id, uint16_t event_id );
extern uint16_t osal_get_timeoutEx( uint8_t task_id, uint16_t event_id );
extern uint8_t osal_timer_num_active( void );
extern uint32_t osal_GetSystemClock( void );


/*********************************************************************
*********************************************************************/
#ifdef __cplusplus
}
#endif

#endif 
