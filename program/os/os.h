#ifndef __BOS_H
#define __BOS_H

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "../device/device.h"
#include "../com/com.h"

typedef struct TimerEvent_s
{
    uint32_t Timestamp;         //! Current timer value
    uint32_t ReloadValue;       //! Timer delay value
    bool IsRunning;             //! Is the timer currently running
    void ( *Callback )( void ); //! Timer IRQ callback function
    struct TimerEvent_s *Next;  //! Pointer to the next Timer object.
}TimerEvent_t;
void TimerSetLowPowerEnable( bool enable );
bool TimerGetLowPowerEnable( void );
void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) );
void TimerEvent_Start( TimerEvent_t *obj );
void TimerEvent_Stop( TimerEvent_t *obj );
void TimerReset( TimerEvent_t *obj );
void TimerSetValue( TimerEvent_t *obj, uint32_t value );
void DelayMs( uint32_t ms );
uint64_t TIM2_Tick_Present( void );//Return the value on the timer counter
uint64_t TIM2_Time_Present( void );
uint64_t TimerHwGetElapsedTime( void );//Return the value on the timer Tick counter
void TimerHwEnterLowPowerStopMode( void );//Set the ARM core in Wait For Interrupt mode (only working if Debug mode is not used)
void TimerTask(void );




#endif


