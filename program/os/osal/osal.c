#include "osal.h"
#include "bsp_Config.h"

osal_msg_q_t osal_qHead;
extern u16 Onboard_rand(void);

/*********************************************************************
 * @fn      osal_strlen
 *
 * @brief
 *
 *   Calculates the length of a string.  The string must be null
 *   terminated.
 *
 * @param   char *pString - pointer to text string
 *
 * @return  int - number of characters
 */
int osal_strlen( char *pString )
{
  return (int)( strlen( pString ) );
}

/*********************************************************************
 * @fn      osal_memcpy
 *
 * @brief
 *
 *   Generic memory copy.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination u8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - destination address
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to end of destination buffer
 */
void *osal_memcpy( void *dst, const void GENERIC *src, unsigned int len )
{
  u8 *pDst;
  const u8 GENERIC *pSrc;

  pSrc = src;
  pDst = dst;

  while ( len-- )
    *pDst++ = *pSrc++;

  return ( pDst );
}

/*********************************************************************
 * @fn      osal_revmemcpy
 *
 * @brief   Generic reverse memory copy.  Starts at the end of the
 *   source buffer, by taking the source address pointer and moving
 *   pointer ahead "len" bytes, then decrementing the pointer.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination u8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - destination address
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to end of destination buffer
 */
void *osal_revmemcpy( void *dst, const void GENERIC *src, unsigned int len )
{
  u8 *pDst;
  const u8 GENERIC *pSrc;

  pSrc = src;
  pSrc += (len-1);
  pDst = dst;

  while ( len-- )
  {
  	*pDst++ = *pSrc--;
  }
  return ( pDst );
}

/*********************************************************************
 * @fn      osal_memdup
 *
 * @brief   Allocates a buffer [with osal_mem_alloc()] and copies
 *          the src buffer into the newly allocated space.
 *
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to the new allocated buffer, or NULL if
 *          allocation problem.
 */
void *osal_memdup( const void GENERIC *src, unsigned int len )
{
  u8 *pDst;

  pDst = osal_mem_alloc( len );
  if ( pDst )
  {
    osal_memcpy( pDst, src, len );
  }

  return ( (void *)pDst );
}

/*********************************************************************
 * @fn      osal_memcmp
 *
 * @brief
 *
 *   Generic memory compare.
 *
 * @param   src1 - source 1 addrexx
 * @param   src2 - source 2 address
 * @param   len - number of bytes to compare
 *
 * @return  TRUE - same, FALSE - different
 */
u8 osal_memcmp( const void GENERIC *src1, const void GENERIC *src2, unsigned int len )
{
  const u8 GENERIC *pSrc1;
  const u8 GENERIC *pSrc2;

  pSrc1 = src1;
  pSrc2 = src2;

  while ( len-- )
  {
    if( *pSrc1++ != *pSrc2++ )
      return FALSE;
  }
  return TRUE;
}


/*********************************************************************
 * @fn      osal_memset
 *
 * @brief
 *
 *   Set memory buffer to value.
 *
 * @param   dest - pointer to buffer
 * @param   value - what to set each u8 of the message
 * @param   size - how big
 *
 * @return  value of next widget, 0 if no widget found
 */
void *osal_memset( void *dest, u8 value, int len )
{
  return memset( dest, value, len );
}

/*********************************************************************
 * @fn      osal_build_u16
 *
 * @brief
 *
 *   Build a u16 out of 2 bytes (0 then 1).
 *
 * @param   swapped - 0 then 1
 *
 * @return  u16
 */
u16 osal_build_u16( u8 *swapped )
{
  return ( BUILD_u16( swapped[0], swapped[1] ) );
}

/*********************************************************************
 * @fn      osal_build_u32
 *
 * @brief
 *
 *   Build a u32 out of sequential bytes.
 *
 * @param   swapped - sequential bytes
 * @param   len - number of bytes in the u8 array
 *
 * @return  u32
 */
u32 osal_build_u32( u8 *swapped, u8 len )
{
  if ( len == 2 )
    return ( BUILD_u32( swapped[0], swapped[1], 0L, 0L ) );
  else if ( len == 3 )
    return ( BUILD_u32( swapped[0], swapped[1], swapped[2], 0L ) );
  else if ( len == 4 )
    return ( BUILD_u32( swapped[0], swapped[1], swapped[2], swapped[3] ) );
  else
    return ( (u32)swapped[0] );
}
/*********************************************************************
 * @fn        osal_rand
 *
 * @brief    Random number generator
 *
 * @param   none
 *
 * @return  u16 - new random number
 */
u16 osal_rand( void )
{
  return Onboard_rand();
}

/*********************************************************************
 * API FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osal_msg_allocate
 *
 * @brief
 *
 *    This function is called by a task to allocate a message buffer
 *    into which the task will encode the particular message it wishes
 *    to send.  This common buffer scheme is used to strictly limit the
 *    creation of message buffers within the system due to RAM size
 *    limitations on the microprocessor.   Note that all message buffers
 *    are a fixed size (at least initially).  The parameter len is kept
 *    in case a message pool with varying fixed message sizes is later
 *    created (for example, a pool of message buffers of size LARGE,
 *    MEDIUM and SMALL could be maintained and allocated based on request
 *    from the tasks).
 *
 *
 * @param   u8 len  - wanted buffer length
 *
 *
 * @return  pointer to allocated buffer or NULL if allocation failed.
 */
u8 * osal_msg_allocate( u16 len )
{
  osal_msg_hdr_t *hdr;

  if ( len == 0 )
    return ( NULL );

  hdr = (osal_msg_hdr_t *) osal_mem_alloc( (short)(len + sizeof( osal_msg_hdr_t )) );
  if ( hdr )
  {
    hdr->next = NULL;
    hdr->len = len;
    hdr->dest_id = TASK_NO_TASK;
    return ( (u8 *) (hdr + 1) );
  }
  else
    return ( NULL );
}

/*********************************************************************
 * @fn      osal_msg_deallocate
 *
 * @brief
 *
 *    This function is used to deallocate a message buffer. This function
 *    is called by a task (or processing element) after it has finished
 *    processing a received message.
 *
 *
 * @param   u8 *msg_ptr - pointer to new message buffer
 *
 * @return  SUCCESS, INVALID_MSG_POINTER
 */
u8 osal_msg_deallocate( u8 *msg_ptr )
{
  u8 *x;

  if ( msg_ptr == NULL )
    return ( INVALID_MSG_POINTER );

  // don't deallocate queued buffer
  if ( OSAL_MSG_ID( msg_ptr ) != TASK_NO_TASK )
    return ( MSG_BUFFER_NOT_AVAIL );

  x = (u8 *)((u8 *)msg_ptr - sizeof( osal_msg_hdr_t ));

  osal_mem_free( (void *)x );

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      osal_msg_send
 *
 * @brief
 *
 *    This function is called by a task to send a command message to
 *    another task or processing element.  The sending_task field must
 *    refer to a valid task, since the task ID will be used
 *    for the response message.  This function will also set a message
 *    ready event in the destination tasks event list.
 *
 *
 * @param   u8 destination task - Send msg to?  Task ID
 * @param   u8 *msg_ptr - pointer to new message buffer
 * @param   u8 len - length of data in message
 *
 * @return  SUCCESS, INVALID_TASK, INVALID_MSG_POINTER
 */
u8 osal_msg_send( u8 destination_task, u8 *msg_ptr )
{
  if ( msg_ptr == NULL )
    return ( INVALID_MSG_POINTER );

  if ( destination_task >= tasksCnt )
  {
    osal_msg_deallocate( msg_ptr );
    return ( INVALID_TASK );
  }

  // Check the message header
  if ( OSAL_MSG_NEXT( msg_ptr ) != NULL ||
       OSAL_MSG_ID( msg_ptr ) != TASK_NO_TASK )
  {
    osal_msg_deallocate( msg_ptr );
    return ( INVALID_MSG_POINTER );
  }

  OSAL_MSG_ID( msg_ptr ) = destination_task;

  // queue message
  osal_msg_enqueue( &osal_qHead, msg_ptr );

  // Signal the task that a message is waiting
  osal_set_event( destination_task, SYS_EVENT_MSG );

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      osal_msg_receive
 *
 * @brief
 *
 *    This function is called by a task to retrieve a received command
 *    message. The calling task must deallocate the message buffer after
 *    processing the message using the osal_msg_deallocate() call.
 *
 * @param   u8 task_id - receiving tasks ID
 *
 * @return  *u8 - message information or NULL if no message
 */
u8 *osal_msg_receive( u8 task_id )
{
  osal_msg_hdr_t *listHdr;
  osal_msg_hdr_t *prevHdr = NULL;
  osal_msg_hdr_t *foundHdr = NULL;
  halIntState_t   intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  // Point to the top of the queue
  listHdr = osal_qHead;

  // Look through the queue for a message that belongs to the asking task
  while ( listHdr != NULL )
  {
    if ( (listHdr - 1)->dest_id == task_id )
    {
      if ( foundHdr == NULL )
      {
        // Save the first one
        foundHdr = listHdr;
      }
      else
      {
        // Second msg found, stop looking
        break;
      }
    }
    if ( foundHdr == NULL )
    {
      prevHdr = listHdr;
    }
    listHdr = OSAL_MSG_NEXT( listHdr );
  }

  // Is there more than one?
  if ( listHdr != NULL )
  {
    // Yes, Signal the task that a message is waiting
    osal_set_event( task_id, SYS_EVENT_MSG );
  }
  else
  {
    // No more
    osal_clear_event( task_id, SYS_EVENT_MSG );
  }

  // Did we find a message?
  if ( foundHdr != NULL )
  {
    // Take out of the link list
    osal_msg_extract( &osal_qHead, foundHdr, prevHdr );
  }

  // Release interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);

  return ( (u8*) foundHdr );
}

/**************************************************************************************************
 * @fn          osal_msg_find
 *
 * @brief       This function finds in place an OSAL message matching the task_id and event
 *              parameters.
 *
 * input parameters
 *
 * @param       task_id - The OSAL task id that the enqueued OSAL message must match.
 * @param       event - The OSAL event id that the enqueued OSAL message must match.
 *
 * output parameters
 *
 * None.
 *
 * @return      NULL if no match, otherwise an in place pointer to the matching OSAL message.
 **************************************************************************************************
 */
osal_event_hdr_t *osal_msg_find(u8 task_id, u8 event)
{
  osal_msg_hdr_t *pHdr;
  halIntState_t intState;

  HAL_ENTER_CRITICAL_SECTION(intState);  // Hold off interrupts.

  pHdr = osal_qHead;  // Point to the top of the queue.

  // Look through the queue for a message that matches the task_id and event parameters.
  while (pHdr != NULL)
  {
    if (((pHdr-1)->dest_id == task_id) && (((osal_event_hdr_t *)pHdr)->event == event))
    {
      break;
    }

    pHdr = OSAL_MSG_NEXT(pHdr);
  }

  HAL_EXIT_CRITICAL_SECTION(intState);  // Release interrupts.

  return (osal_event_hdr_t *)pHdr;
}

/*********************************************************************
 * @fn      osal_msg_enqueue
 *
 * @brief
 *
 *    This function enqueues an OSAL message into an OSAL queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message
 *
 * @return  none
 */
void osal_msg_enqueue( osal_msg_q_t *q_ptr, void *msg_ptr )
{
  void *list;
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  OSAL_MSG_NEXT( msg_ptr ) = NULL;
  // If first message in queue
  if ( *q_ptr == NULL )
  {
    *q_ptr = msg_ptr;
  }
  else
  {
    // Find end of queue
    for ( list = *q_ptr; OSAL_MSG_NEXT( list ) != NULL; list = OSAL_MSG_NEXT( list ) );

    // Add message to end of queue
    OSAL_MSG_NEXT( list ) = msg_ptr;
  }

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);
}

/*********************************************************************
 * @fn      osal_msg_dequeue
 *
 * @brief
 *
 *    This function dequeues an OSAL message from an OSAL queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 *
 * @return  void * - pointer to OSAL message or NULL of queue is empty.
 */
void *osal_msg_dequeue( osal_msg_q_t *q_ptr )
{
  void *msg_ptr = NULL;
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  if ( *q_ptr != NULL )
  {
    // Dequeue message
    msg_ptr = *q_ptr;
    *q_ptr = OSAL_MSG_NEXT( msg_ptr );
    OSAL_MSG_NEXT( msg_ptr ) = NULL;
    OSAL_MSG_ID( msg_ptr ) = TASK_NO_TASK;
  }

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);

  return msg_ptr;
}

/*********************************************************************
 * @fn      osal_msg_push
 *
 * @brief
 *
 *    This function pushes an OSAL message to the head of an OSAL
 *    queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message
 *
 * @return  none
 */
void osal_msg_push( osal_msg_q_t *q_ptr, void *msg_ptr )
{
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  // Push message to head of queue
  OSAL_MSG_NEXT( msg_ptr ) = *q_ptr;
  *q_ptr = msg_ptr;

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);
}

/*********************************************************************
 * @fn      osal_msg_extract
 *
 * @brief
 *
 *    This function extracts and removes an OSAL message from the
 *    middle of an OSAL queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message to be extracted
 * @param   void *prev_ptr  - OSAL message before msg_ptr in queue
 *
 * @return  none
 */
void osal_msg_extract( osal_msg_q_t *q_ptr, void *msg_ptr, void *prev_ptr )
{
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  if ( msg_ptr == *q_ptr )
  {
    // remove from first
    *q_ptr = OSAL_MSG_NEXT( msg_ptr );
  }
  else
  {
    // remove from middle
    OSAL_MSG_NEXT( prev_ptr ) = OSAL_MSG_NEXT( msg_ptr );
  }
  OSAL_MSG_NEXT( msg_ptr ) = NULL;
  OSAL_MSG_ID( msg_ptr ) = TASK_NO_TASK;

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);
}

/*********************************************************************
 * @fn      osal_msg_enqueue_max
 *
 * @brief
 *
 *    This function enqueues an OSAL message into an OSAL queue if
 *    the length of the queue is less than max.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message
 * @param   u8 max - maximum length of queue
 *
 * @return  TRUE if message was enqueued, FALSE otherwise
 */
u8 osal_msg_enqueue_max( osal_msg_q_t *q_ptr, void *msg_ptr, u8 max )
{
  void *list;
  u8 ret = FALSE;
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  // If first message in queue
  if ( *q_ptr == NULL )
  {
    *q_ptr = msg_ptr;
    ret = TRUE;
  }
  else
  {
    // Find end of queue or max
    list = *q_ptr;
    max--;
    while ( (OSAL_MSG_NEXT( list ) != NULL) && (max > 0) )
    {
      list = OSAL_MSG_NEXT( list );
      max--;
    }

    // Add message to end of queue if max not reached
    if ( max != 0 )
    {
      OSAL_MSG_NEXT( list ) = msg_ptr;
      ret = TRUE;
    }
  }

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);

  return ret;
}

/*********************************************************************
 * @fn      osal_set_event
 *
 * @brief
 *
 *    This function is called to set the event flags for a task.  The
 *    event passed in is OR'd into the task's event variable.
 *
 * @param   u8 task_id - receiving tasks ID
 * @param   u8 event_flag - what event to set
 *
 * @return  SUCCESS, INVALID_TASK
 */
u8 osal_set_event( u8 task_id, u16 event_flag )
{
  if ( task_id < tasksCnt )
  {
    halIntState_t   intState;
    HAL_ENTER_CRITICAL_SECTION(intState);    // Hold off interrupts
    tasksEvents[task_id] |= event_flag;  // Stuff the event bit(s)
    HAL_EXIT_CRITICAL_SECTION(intState);     // Release interrupts
    return ( SUCCESS );
  }
   else
  {
    return ( INVALID_TASK );
  }
}

/*********************************************************************
 * @fn      osal_clear_event
 *
 * @brief
 *
 *    This function is called to clear the event flags for a task.  The
 *    event passed in is masked out of the task's event variable.
 *
 * @param   u8 task_id - receiving tasks ID
 * @param   u8 event_flag - what event to set
 *
 * @return  SUCCESS, INVALID_TASK
 */
u8 osal_clear_event( u8 task_id, u16 event_flag )
{
  if ( task_id < tasksCnt )
  {
    halIntState_t   intState;
    HAL_ENTER_CRITICAL_SECTION(intState);    // Hold off interrupts
    tasksEvents[task_id] &= ~(event_flag);   // clear the event bit(s)
    HAL_EXIT_CRITICAL_SECTION(intState);     // Release interrupts
    return ( SUCCESS );
  }
   else
  {
    return ( INVALID_TASK );
  }
}

/*********************************************************************
 * @fn      osal_int_enable
 *
 * @brief
 *
 *   This function is called to enable an interrupt. Once enabled,
 *   occurrence of the interrupt causes the service routine associated
 *   with that interrupt to be called.
 *
 *   If INTS_ALL is the interrupt_id, interrupts (in general) are enabled.
 *   If a single interrupt is passed in, then interrupts still have
 *   to be enabled with another call to INTS_ALL.
 *
 * @param   u8 interrupt_id - Interrupt number
 *
 * @return  SUCCESS or INVALID_INTERRUPT_ID
 */
u8 osal_int_enable( u8 interrupt_id )
{

  if ( interrupt_id == INTS_ALL )
  {
    HAL_ENABLE_INTERRUPTS();
    return ( SUCCESS );
  }
  else
  {
    return ( INVALID_INTERRUPT_ID );
  }
}

/*********************************************************************
 * @fn      osal_int_disable
 *
 * @brief
 *
 *   This function is called to disable an interrupt. When a disabled
 *   interrupt occurs, the service routine associated with that
 *   interrupt is not called.
 *
 *   If INTS_ALL is the interrupt_id, interrupts (in general) are disabled.
 *   If a single interrupt is passed in, then just that interrupt is disabled.
 *
 * @param   u8 interrupt_id - Interrupt number
 *
 * @return  SUCCESS or INVALID_INTERRUPT_ID
 */
u8 osal_int_disable( u8 interrupt_id )
{

  if ( interrupt_id == INTS_ALL )
  {
    HAL_DISABLE_INTERRUPTS();
    return ( SUCCESS );
  }
  else
  {
    return ( INVALID_INTERRUPT_ID );
  }
}


/*********************************************************************
 * @fn      osal_start_system
 *
 * @brief
 *
 *   This function is the main loop function of the task system.  It
 *   will look through all task events and call the task_event_processor()
 *   function for the task with the event.  If there are no events (for
 *   all tasks), this function puts the processor into Sleep.
 *   This Function doesn't return.
 *
 * @param   void
 *
 * @return  none
 */
void osal_start_system( void )
{
while(1)
  {
    u8 idx = 0;
    osalTimeUpdate();
    do {
      if (tasksEvents[idx])  // Task is highest priority that is ready.
      {
        break;
      }
    } while (++idx < tasksCnt);

    if (idx < tasksCnt)
    {
      u16 events;
      halIntState_t intState;

      HAL_ENTER_CRITICAL_SECTION(intState);
      events = tasksEvents[idx];
      tasksEvents[idx] = 0;  // Clear the Events for this task.
      HAL_EXIT_CRITICAL_SECTION(intState);

      events = (tasksArr[idx])( idx, events );

      HAL_ENTER_CRITICAL_SECTION(intState);
      tasksEvents[idx] |= events;  // Add back unprocessed events to the current task.
      HAL_EXIT_CRITICAL_SECTION(intState);
    }
	
  }
}

/*********************************************************************
*********************************************************************/

void osalTimeUpdate( void )
{
  u16 elapsedMSec = 0;
  elapsedMSec = osal_tick;

  if ( elapsedMSec != 0 )
  {
    osal_tick = 0;
    osalTimerUpdate( elapsedMSec );
	while(elapsedMSec--)
	{
		osal_mutex_updata();
	}
  }
}









/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/Src/stm32l0xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   HAL MSP module.
  *
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by STM32CubeMX and eventually modified
    by the user

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_Config.h"
#include "led_board.h"
#include "oled_board.h"
#include "key_board.h"

#include "sx1276/sx1276.h"
#include "main.h"


#include "osal.h"

#include "app_osal.h"
#include "LoRaMacUsr.h"

#include "radio.h"
#include "loraMAC_osal.h"
#include "hal_osal.h"

#include "phyMac.h"
#include "LoRaMac.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
char Rx_buf[64]; //buffer for oled display
u8* RecieveBuff_flag = NULL;
__IO ITStatus UartReady = RESET;
u8 aTxBuffer[] = "uart test, hello!\n";
u8 aRxBuffer[RXBUFFERSIZE];
u8 g_number = 0;

LoRaMacAppPara_t g_appData;//定义APP 参数结构体


u8 send_num = 10;
uint8_t txuartdataflag ;
u8 uucount = 5;
u8 debugEnable = FALSE;

/* variables -----------------------------------------------------------*/
extern UART_HandleTypeDef UartHandle;
extern u8 send_num ;


/**
  * @brief  HAL task event process routine
  * @param  u8 task_id -- task ID allocated by OSAL
  					  u16 events -- happened events of this task
  * @retval u16 -- events that haven't been processed
  */
u16 HardWare_ProcessEvent( u8 task_id, u16 events )
{
    if ( events & HAL_LED_BLINK_EVENT )
    {
        #if (defined (BLINK_LEDS)) && (HAL_LED == TRUE)

        HalLedUpdate();

        #endif /* BLINK_LEDS && HAL_LED */

        return events ^ HAL_LED_BLINK_EVENT;
    }

    if (events & HAL_KEY_EVENT)
    {
        #if (defined HAL_KEY) && (HAL_KEY == TRUE)

        //TODO when key was pressed
        HalKeyPoll();

        #endif

        return events ^ HAL_KEY_EVENT;
    }

    return 0 ;
}

u16 APP_ProcessEvent( u8 task_id, u16 events )
{
 loraMAC_msg_t* pMsgSend = NULL;
 loraMAC_msg_t* pMsgRecieve = NULL;
	
 u8 len = 0 ;
	u8 communicationStatus = 0;
	
  //system event
  if(events & SYS_EVENT_MSG)
  {
		//receive msg loop
		while(NULL != (pMsgRecieve = (loraMAC_msg_t*)osal_msg_receive(APP_taskID)))
		{
		//pMsgRecieve[0] is system event type
		switch(pMsgRecieve->msgID)
		{
		//tx done
		case TXDONE :
		case TXERR_STATUS:	
				
			//	HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);
			//	HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);
//				HalKeyRead();	
				if(send_num > 0)//通过LORA MAC模式发包
				{
					send_num--;
					//send a packet to LoRaMac osal (then can be send by the radio)
					pMsgSend = (loraMAC_msg_t*)osal_msg_allocate(sizeof(loraMAC_msg_t));
					if(pMsgSend != NULL)
					{
						osal_memset(pMsgSend,0,sizeof(loraMAC_msg_t));
						pMsgSend->msgID = TXREQUEST;
						pMsgSend->msgLen = 1;
						pMsgSend->msgData[0] = 0Xf8;
					/*	for(u8 dataCount = 0; dataCount < 70; dataCount++)
						{
							pMsgSend->msgData[dataCount] = dataCount;
						}
						osal_msg_send(LoraMAC_taskID,(u8*)pMsgSend);
						osal_msg_deallocate((u8*)pMsgSend);	*/
						if(TxReady())
						{
							ClrTxStatus();
							pMsgSend->msgLen = 70;
							for(u8 dataCount = 0; dataCount < 70; dataCount++)
							{
								pMsgSend->msgData[dataCount] = transmitBuf[dataCount];
								transmitBuf[dataCount] = 0x00;
							}								
						}
						osal_msg_send(LoraMAC_taskID,(u8*)pMsgSend);
						osal_msg_deallocate((u8*)pMsgSend);		
 
						#ifdef USE_DEBUG
						UART1_Send("app send start...\n", osal_strlen("app send start...\n"));
						#endif
					}
				}
				else
				{
				
					#if  0//PHYMAC 模式发包
					
					if(mode != MODE_PHY)//通过PHY模式发包
					{
						LoRaMac_setMode(MODE_PHY);
						uucount = 5;
					}
					else
					{
						uucount--;
						if(uucount == 0)
						{
							send_num = 10;
							LoRaMac_setMode(MODE_LORAMAC);
							osal_set_event(APP_taskID,APP_PERIOD_SEND);
							//Radio.Sleep();
						}
					}
					
					#else//低功耗测试
					
					#ifdef USE_LOW_POWER_MODE
					RtcSetTimeout(20000000);
					LoRaMac_setlowPowerMode(TRUE);
					RtcEnterLowPowerStopMode();
					#endif
					LoRaMac_setMode(MODE_LORAMAC);
					
					send_num = 10;
					osal_set_event(APP_taskID,APP_PERIOD_SEND);
					Radio.Sleep();
					
					#endif
				}
				
		//		HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);
				
				break;

				//rx done
			case RXDONE:
				
			//	HalLedSet (HAL_LED_2, HAL_LED_MODE_ON);
				OLED_Clear_Half();//先把屏幕下一半清空
				APP_ShowMoteID(g_appData.devAddr);
				len = 0 ;
				g_number++ ;
				memset(Rx_buf , 0 ,sizeof(Rx_buf));                               
				osal_memcpy(Rx_buf,pMsgRecieve->msgData,pMsgRecieve->msgLen);			
				len = pMsgRecieve->msgLen;
				communicationStatus = CmdProcess(pMsgRecieve->msgData);
				if(communicationStatus == COMMUNICATION_SUCCESS)
				{
					SetTxStatus(); 
				}
			
				Rx_buf[len] = 0;
				OLED_Clear_Line(4,12);//先清空数据，再显示
				OLED_Clear_Line(5,12);
				OLED_ShowString( 0,36, (u8*)Rx_buf,12 );
				OLED_Refresh_Gram();
				#ifdef USE_DEBUG
				UART1_Send("\n",1);
				UART1_Send((uint8_t *)Rx_buf,strlen(Rx_buf));
				#endif
	//			HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);
			
				break;
				
        default:
			    break;
			}

			osal_msg_deallocate((u8*)pMsgRecieve);
		}
		return (events ^ SYS_EVENT_MSG);

	}

	//send a packet event
	if(events & APP_PERIOD_SEND)
	{
		//RedLED(OFF);
//		 HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);
		//send a packet to LoRaMac osal (then can be send by the radio)
		pMsgSend = (loraMAC_msg_t*)osal_msg_allocate(sizeof(loraMAC_msg_t));
		if(pMsgSend != NULL)
		{
			osal_memset(pMsgSend,0,sizeof(loraMAC_msg_t));
			pMsgSend->msgID = TXREQUEST;
		//	pMsgSend->msgLen = 70;
			pMsgSend->msgLen = 0;
			for(u8 dataCount = 0; dataCount < 70; dataCount++)
			{
				pMsgSend->msgData[dataCount] = dataCount;
			}
				osal_msg_send(LoraMAC_taskID,(u8*)pMsgSend);
		}

		#ifdef USE_DEBUG
		UART1_Send("app send start...\n", osal_strlen("app send start...\n"));
		#endif
	  //osal_start_timerEx(APP_taskID, APP_PERIOD_SEND,1000);//延时继续发送
		return (events ^ APP_PERIOD_SEND);
	}

	return 0 ;
}

//display NPLink mote ID on the OLED
void APP_ShowMoteID( u32 moteID )
{
	u8 	MoteIDString[32] ;
	u8* pIDString = MoteIDString;
	u32 ZeroNum = 0 ;

	//count the zero num in front of moteID string
	for(u8 i = 28; i > 0; i = i - 4)
	{
		if((moteID >> i ) % 16 == 0)
		{
			ZeroNum = ZeroNum + 1 ;
		}
		else
		{
			break;
		}
	}

	sprintf((char*)pIDString,"ID:");
	pIDString += 3;
	while(ZeroNum--)
	{
		sprintf((char*)pIDString,"0");
		pIDString++;
	}
	sprintf((char*)pIDString,"%x",moteID);

	OLED_ShowString( 0,0,MoteIDString,12 );
	OLED_Refresh_Gram();
}

u16 Onboard_rand(void) 
{
	return 0; //return TIM_GetCounter(TIM5);
}

/******************* (C) COPYRIGHT 2015 NPLink *****END OF FILE****/


bool g_lora_mac_adr_switch = true ;
/*!
 * Unique Devices IDs register set ( STM32L0xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )
/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     0  //modified by Denny  2018/5/25

#if( OVER_THE_AIR_ACTIVATION != 0 )  
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE          10000000  // 10 [s] value in us
#define APP_TX_DUTYCYCLE                            5000000  // 5 [s] value in us
#define APP_TX_DUTYCYCLE_RND                        1000000  // 1 [s] value in us
uint8_t DevEui[] ={0x70, 0xB3, 0xD5, 0x31, 0xC0, 0x00, 0x01, 0x50 };
uint8_t AppEui[] ={0x70, 0xB3, 0xD5, 0x31, 0xC0, 0x00, 0x00, 0x01 };
uint8_t AppKey[] ={0xB3, 0x19, 0x0F, 0xFF, 0x13, 0xD6, 0x93, 0x00, 0xAD, 0xA1, 0x09, 0xFF, 0x24, 0x3E, 0xA5, 0x50};

#else  
uint8_t NwkSKey[] ={0x3a, 0x1e, 0xbd, 0x00, 0x3a, 0xc8, 0x8c, 0x00, 0xc0, 0xc5, 0x1e, 0xff, 0xea, 0xd2, 0x65, 0x50  };//kiwi
uint8_t AppSKey[] ={0xe0, 0x8c, 0x28, 0xff, 0xcf, 0x7a, 0x47, 0xff, 0x7a, 0xb5, 0xeb, 0x00, 0xbd, 0x00, 0x01, 0x50  };//KIWI
#endif  

/*!
 * .
 */

#if( OVER_THE_AIR_ACTIVATION != 0 )
TimerEvent_t JoinReqTimer;  //Defines the join request timer
#endif
static u8 AppPort = 2;

u8	mode = MODE_LORAMAC; //工作模式




#if( OVER_THE_AIR_ACTIVATION != 0 )
static void OnJoinReqTimerEvent( void )  //Function executed on JoinReq Timeout event
{	  
		//LoRaMac OTAA 入网请求
		loraMAC_msg_t* pMsg = (loraMAC_msg_t*)osal_msg_allocate(1);
		if(NULL != pMsg)
		{
				osal_memset((u8*)pMsg,0,1);
				pMsg->msgID = JOINNET_REQUEST;
				osal_msg_send(APP_taskID,(u8*)pMsg);
		}
}
#endif


//PHY回调处理函数
void onPhyEvent( phyEventInfo_t* info)
{
	if( info->event == PHY_TXDONE )
	{
		loraMAC_msg_t* pMsg = (loraMAC_msg_t*)osal_msg_allocate(1+1+2);
		if(NULL != pMsg)
		{
			pMsg->msgID = TXDONE;
			pMsg->msgLen = 0;
			pMsg->frame_no = g_frame_no;
			osal_msg_send(APP_taskID,(u8*)pMsg);
		}
	}
	if( info->event == PHY_RXDONE )
	{
	if( info->size > 0)
   	{
       loraMAC_msg_t* pMsg = (loraMAC_msg_t*)osal_msg_allocate(info->size + 8);
       if(NULL != pMsg)
       {
           osal_memset(pMsg,0,info->size + 8);
           pMsg->msgID = RXDONE;
           pMsg->msgLen = info->size;
           pMsg->msgRxRssi = info->rssi;
           pMsg->msgRxSnr = info->snr;
           osal_memcpy(pMsg->msgData,info->buffer,info->size);
           osal_msg_send(APP_taskID,(u8*)pMsg);  
       }
   	}
	}
	if( info->event == PHY_TXTIMEOUT )
	{
			loraMAC_msg_t* pMsg = (loraMAC_msg_t*)osal_msg_allocate(1+1);
			if(NULL != pMsg)
			{
				pMsg->msgID = TXERR_STATUS;
				pMsg->msgLen = 0;
				osal_msg_send(APP_taskID,(u8*)pMsg);
			}
	}	
}




u16 LoRaMAC_ProcessEvent( u8 task_id, u16 events )
{
	loraMAC_msg_t* pDataMsg = NULL;
	u8 ret = 0xFF ;

	if(events & SYS_EVENT_MSG)
	{
		while(NULL != (pDataMsg = (loraMAC_msg_t*)osal_msg_receive(LoraMAC_taskID)))
		{
			//判断当前工作模式是LoRaMac 还是PHY裸奔
			//LoRaMac
			if(MODE_LORAMAC == mode)
			{
				ret = LoRaMacSendFrame( AppPort, pDataMsg->msgData, pDataMsg->msgLen);

				//ret = 5 ;
				if( 0 != ret )
				{
					loraMAC_msg_t* pMsg = (loraMAC_msg_t*)osal_msg_allocate(1);
					if(NULL != pMsg)
					{
							osal_memset((u8*)pMsg,0,1);
							pMsg->msgID = TXERR_STATUS;
							
							osal_msg_send(APP_taskID,(u8*)pMsg);
					}
				}
			}
			//PHY
			else if(MODE_PHY == mode)
			{

				phySendFrame(pDataMsg->msgData, pDataMsg->msgLen);

			}
			else
			{
				
			}

			osal_msg_deallocate((u8*)pDataMsg);
		}

		return (events ^ SYS_EVENT_MSG);
	}

	return 0 ;
}

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
	HardWare_ProcessEvent,
	LoRaMAC_ProcessEvent,
	APP_ProcessEvent,
};

const u8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
u16 myTasksEvents[tasksCnt];
u16 *tasksEvents = myTasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
bool TxNextPacket = true;



/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    TxNextPacket = true;
}



#include "osal.h"

osal_mutex_t *osal_mutex_head = NULL;
/*********************************************************************
 * @fn      osal_mutex_create
 *
 * @brief   create a muter
 *
 * @param   none
 *
 * @return  a pointer point to creater muter
 */
osal_mutex_t* osal_mutex_create(void)
{
 osal_mutex_t *ptr;
 osal_mutex_t *pseach;
 ptr = (osal_mutex_t*)osal_mem_alloc(sizeof(osal_mutex_t));
 if(ptr != NULL)
 	{
		 ptr->next_mutex = NULL;
		 ptr->mutex_value = FALSE;
		 if(osal_mutex_head == NULL)
		 	{
		 		osal_mutex_head = ptr;
		 	}
		 else
		 	{
		 		pseach = osal_mutex_head;
		 		while(pseach->next_mutex != NULL)
		 			{
		 			 pseach = pseach->next_mutex;
		 			}
				pseach->next_mutex = ptr;
		 	}
 	}
return ptr;
}

/*********************************************************************
 * @fn      osal_mutex_delete
 *
 * @brief  delete a muter
 *
 * @param  point to be delete muter
 *
 * @return none
 */
void osal_mutex_delete(osal_mutex_t** mutex)
{
	osal_mutex_t* pseach = osal_mutex_head;
	if(pseach == NULL )return;
	if(pseach == *mutex)
		{
			osal_mutex_head = (*mutex)->next_mutex;
			osal_mem_free((u8*)(*mutex));
			*mutex = NULL;

		}
	else
		{
			while( (pseach->next_mutex != *mutex)&&(pseach->next_mutex != NULL) )
				{
					pseach = pseach->next_mutex;
				}
			if(pseach->next_mutex == *mutex)
				{
					pseach->next_mutex = (*mutex)->next_mutex;
					osal_mem_free((u8*)(*mutex));
					*mutex = NULL;
				}
		}

}
/*********************************************************************
 * @fn      osal_mutex_take
 *
 * @brief  take a muter
 *
 * @param   a pointer point to be taken muter
 *
 * @return none
 */
void osal_mutex_take(osal_mutex_t** mutex,u32 mutex_overtime)
{
	if(*mutex == NULL)
		{
		 *mutex = osal_mutex_create();
		}
	if(*mutex != NULL)
		{
		(*mutex)->mutex_value = mutex_overtime;
		}

}
/*********************************************************************
 * @fn      osal_mutex_check
 *
 * @brief  cheak muter state
 *
 * @param   a pointer point to be check muter
 *
 * @return  muter state   1 - take  0 - release
 */
u32 osal_mutex_check(osal_mutex_t* mutex)
{
	if(mutex == NULL)return FALSE;
	if(mutex->mutex_value != 0) return TRUE;
	return FALSE;
}
/*********************************************************************
 * @fn      osal_mutex_release
 *
 * @brief  release a mutex
 *
 * @param   a pointer point to be  release  mutex
 *
 * @return none
 */
void osal_mutex_release(osal_mutex_t** mutex)
{
	if((*mutex) == NULL) return;
	(*mutex)->mutex_value = 0;
	osal_mutex_delete(mutex);
}


/*********************************************************************
 * @fn      osal_mutex_updata
 *
 * @brief   update mutex clock information
 *
 * @param  NONE
 *
 * @return NONE
 */
void osal_mutex_updata(void)
{
	osal_mutex_t *pseach;
	pseach = osal_mutex_head;
	while(pseach != NULL)
		{
			if(pseach->mutex_value>0)
				{
					pseach->mutex_value--;
				}
			pseach = pseach->next_mutex;
		}
}


/*********************************************************************
 * INCLUDES
 */
#include "osal.h"
// Minimum wasted bytes to justify splitting a block before allocation.
#if !defined ( OSALMEM_MIN_BLKSZ )
  #define OSALMEM_MIN_BLKSZ    4
#endif

/* Profiling memory allocations showed that a significant % of very high
 * frequency allocations/frees are for block sizes less than or equal to 16.
 */
#if !defined ( OSALMEM_SMALL_BLKSZ )
  #define OSALMEM_SMALL_BLKSZ  64 //64
#endif


#if ( OSALMEM_PROFILER )
  #define OSALMEM_INIT   'X'
  #define OSALMEM_ALOC   'A'
  #define OSALMEM_REIN   'F'
#endif

#define MAXMEMHEAP   1*1024 //14*1024


/*********************************************************************
 * TYPEDEFS
 */

typedef u32  osalMemHdr_t;

/*********************************************************************
 * CONSTANTS
 */

#define OSALMEM_IN_USE  0x80000000

/* This number sets the size of the small-block bucket. Although profiling
 * shows max simultaneous alloc of 16x18, timing without profiling overhead
 * shows that the best worst case is achieved with the following.
 */
#define SMALLBLKHEAP    128 //4*1024

// To maintain data alignment of the pointer returned, reserve the greater
// space for the memory block header.
#define HDRSZ  ( (sizeof ( halDataAlign_t ) > sizeof( osalMemHdr_t )) ? \
                  sizeof ( halDataAlign_t ) : sizeof( osalMemHdr_t ) )

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static osalMemHdr_t *ff1;  // First free block in the small-block bucket.
static osalMemHdr_t *ff2;  // First free block after the small-block bucket.

#if ( OSALMEM_METRICS )
  static u16 blkMax;  // Max cnt of all blocks ever seen at once.
  static u16 blkCnt;  // Current cnt of all blocks.
  static u16 blkFree; // Current cnt of free blocks.
  static u16 memAlo;  // Current total memory allocated.
  static u16 memMax;  // Max total memory ever allocated at once.
#endif

// Memory Allocation Heap.

  static halDataAlign_t  _theHeap[ MAXMEMHEAP / sizeof( halDataAlign_t ) ];
  static u8 *theHeap = (u8 *)_theHeap;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      osal_mem_init
 *
 * @brief   Initialize the heap memory management system.
 *
 * @param   void
 *
 * @return  void
 */
void osal_mem_init( void )
{
  osalMemHdr_t *tmp;
  unsigned int i;
  for(i=0;i<MAXMEMHEAP;i++)
  {
    *(theHeap+i)= 0x00;
  }

  // Setup a NULL block at the end of the heap for fast comparisons with zero.
  tmp = (osalMemHdr_t *)theHeap + (MAXMEMHEAP / HDRSZ) - 1;
  *tmp = 0;

  // Setup a small-block bucket.
  tmp = (osalMemHdr_t *)theHeap;
  *tmp = SMALLBLKHEAP;

  // Setup the wilderness.
  tmp = (osalMemHdr_t *)theHeap + (SMALLBLKHEAP / HDRSZ);
  *tmp = ((MAXMEMHEAP / HDRSZ) * HDRSZ) - SMALLBLKHEAP - HDRSZ;

  // Setup a NULL block that is never freed so that the small-block bucket
  // is never coalesced with the wilderness.
  ff1 = tmp;
  ff2 = osal_mem_alloc( 0 );
  ff1 = (osalMemHdr_t *)theHeap;
#if ( OSALMEM_METRICS )
  /* Start with the small-block bucket and the wilderness - don't count the
   * end-of-heap NULL block nor the end-of-small-block NULL block.
   */
  blkCnt = blkFree = 2;
#endif
}

/*********************************************************************
 * @fn      osal_mem_kick
 *
 * @brief   Kick the ff1 pointer out past the long-lived OSAL Task blocks.
 *          Invoke this once after all long-lived blocks have been allocated -
 *          presently at the end of osal_init_system().
 *
 * @param   void
 *
 * @return  void
 */
void osal_mem_kick( void )
{
  halIntState_t intState;

  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

  /* Logic in osal_mem_free() will ratchet ff1 back down to the first free
   * block in the small-block bucket.
   */
  ff1 = ff2;

  HAL_EXIT_CRITICAL_SECTION( intState );  // Re-enable interrupts.
}

/*********************************************************************
 * @fn      osal_mem_alloc
 *
 * @brief   Implementation of the allocator functionality.
 *
 * @param   size - number of bytes to allocate from the heap.
 *
 * @return  void * - pointer to the heap allocation; NULL if error or failure.
 */
void *osal_mem_alloc( u16 size )
{
  osalMemHdr_t *prev = NULL;
  osalMemHdr_t *hdr;
  halIntState_t intState;
  u32 tmp;
  u8 coal = 0;
  size += HDRSZ;

  // Calculate required bytes to add to 'size' to align to halDataAlign_t.
  if ( sizeof( halDataAlign_t ) == 2 )
  {
    size += (size & 0x01);
  }
  else if ( sizeof( halDataAlign_t ) != 1 )
  {
    const u8 mod = size % sizeof( halDataAlign_t );

    if ( mod != 0 )
    {
      size += (sizeof( halDataAlign_t ) - mod);
    }
  }

  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

  // Smaller allocations are first attempted in the small-block bucket.
  if ( size <= OSALMEM_SMALL_BLKSZ )
  {
    hdr = ff1;
  }
  else
  {
    hdr = ff2;
  }
  tmp = *hdr;

  do
  {
    if ( tmp & OSALMEM_IN_USE )
    {
      tmp ^= OSALMEM_IN_USE;
      coal = 0;
    }
    else
    {
      if ( coal != 0 )
      {
#if ( OSALMEM_METRICS )
        blkCnt--;
        blkFree--;
#endif
        *prev += *hdr;

        if ( *prev >= size )
        {
          hdr = prev;
          tmp = *hdr;
          break;
        }
      }
      else
      {
        if ( tmp >= size )
        {
          break;
        }

        coal = 1;
        prev = hdr;
      }
    }

    hdr = (osalMemHdr_t *)((u8 *)hdr + tmp);

    tmp = *hdr;
    if ( tmp == 0 )
    {
      hdr = NULL;
      break;
    }

  } while ( 1 );

  if ( hdr != NULL )
  {
    tmp -= size;

    // Determine whether the threshold for splitting is met.
    if ( tmp >= OSALMEM_MIN_BLKSZ )
    {
      // Split the block before allocating it.
      osalMemHdr_t *next = (osalMemHdr_t *)((u8 *)hdr + size);
      *next = tmp;
      *hdr = (size | OSALMEM_IN_USE);
#if ( OSALMEM_METRICS )
      blkCnt++;
      if ( blkMax < blkCnt )
      {
        blkMax = blkCnt;
      }
      memAlo += size;
#endif
    }
    else
    {
#if ( OSALMEM_METRICS )
      memAlo += *hdr;
      blkFree--;
#endif
      *hdr |= OSALMEM_IN_USE;
    }
#if ( OSALMEM_METRICS )
    if ( memMax < memAlo )
    {
      memMax = memAlo;
    }
#endif
    hdr++;
  }

  HAL_EXIT_CRITICAL_SECTION( intState );  // Re-enable interrupts.

  return (void *)hdr;
}

/*********************************************************************
 * @fn      osal_mem_free
 *
 * @brief   Implementation of the de-allocator functionality.
 *
 * @param   ptr - pointer to the memory to free.
 *
 * @return  void
 */
void osal_mem_free( void *ptr )
{
  osalMemHdr_t *currHdr;
  halIntState_t intState;

  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

  currHdr = (osalMemHdr_t *)ptr - 1;


  *currHdr &= ~OSALMEM_IN_USE;
#if ( OSALMEM_METRICS )
  memAlo -= *currHdr;
  blkFree++;
#endif
  if ( ff1 > currHdr )
  {
    ff1 = currHdr;
  }

  HAL_EXIT_CRITICAL_SECTION( intState );  // Re-enable interrupts.
}


#if ( OSALMEM_METRICS )
/*********************************************************************
 * @fn      osal_heap_block_max
 *
 * @brief   Return the maximum number of blocks ever allocated at once.
 *
 * @param   none
 *
 * @return  Maximum number of blocks ever allocated at once.
 */
u16 osal_heap_block_max( void )
{
  return blkMax;
}

/*********************************************************************
 * @fn      osal_heap_block_cnt
 *
 * @brief   Return the current number of blocks now allocated.
 *
 * @param   none
 *
 * @return  Current number of blocks now allocated.
 */
u16 osal_heap_block_cnt( void )
{
  return blkCnt;
}

/*********************************************************************
 * @fn      osal_heap_block_free
 *
 * @brief   Return the current number of free blocks.
 *
 * @param   none
 *
 * @return  Current number of free blocks.
 */
u16 osal_heap_block_free( void )
{
  return blkFree;
}

/*********************************************************************
 * @fn      osal_heap_mem_used
 *
 * @brief   Return the current number of bytes allocated.
 *
 * @param   none
 *
 * @return  Current number of bytes allocated.
 */
u16 osal_heap_mem_used( void )
{
  return memAlo;
}

u16 osal_heap_mem_max( void )
{
  return memMax;
}
#endif
/*********************************************************************
*********************************************************************/





