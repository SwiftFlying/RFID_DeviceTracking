#include "bsp_Config.h"
#include "osal/osal.h"
#include "os.h"



static TimerEvent_t *TimerListHead = NULL;  // Timers list head pointer
             
volatile uint64_t TIM2_Tick = 1; //Hardware Timer tick counter
static uint64_t TIM2_Tick_EventStart = 0; //Saved value of the Tick counter at the start of the next event
volatile uint64_t TIM2_Tick_EventEnd = 0;  // Value trigging the IRQ




uint64_t TIM2_Tick_Present( void )
{
    uint64_t val = 0;
    __disable_irq();
    val = TIM2_Tick;
    __enable_irq( );
    return( val );
}
uint64_t TIM2_Time_Present( void )
{
    return TIM2_Tick_Present()* HW_TIMER_TIME_BASE;
}


void DelayMs( uint32_t delay )
{
		uint64_t delayValue = 0;
		uint64_t timeout = 0;
		delayValue = delay * 1000;
		timeout = TIM2_Tick_Present();
		while(((TIM2_Tick_Present()-timeout )*HW_TIMER_TIME_BASE)<delayValue)
		{
		}
}






/*!
 * This flag is used to make sure we have looped through the main several time to avoid race issues
 */
volatile uint8_t HasLoopedThroughMain = 0;

/*!
 *
 */


/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */




void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
    obj->Next = NULL;
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    uint32_t minValue = 0;
    TimerEvent_Stop( obj );
    minValue = ceil(2*HW_TIMER_TIME_BASE);  //TimerHwGetMinimumTimeout( );
    if( value < minValue )
        value = minValue;
    obj->Timestamp = value;
    obj->ReloadValue = value;
}




void TimerEvent_Start( TimerEvent_t *obj )
{
    uint32_t elapsedTime = 0;
    uint32_t remainingTime = 0;
    __disable_irq( );
		
	  bool TimerExists;
	
	  TimerEvent_t* cur = TimerListHead;   //遍历链表，找到指定的TimerEvent
    while( cur != NULL )
    {
        if( cur == obj )
            TimerExists= true;
        cur = cur->Next;
    }
    TimerExists= false;
			
    if( ( obj == NULL ) || ( TimerExists == true ) )
    {
        __enable_irq( );
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsRunning = false;

    if( TimerListHead == NULL )
    {
					obj->Next = NULL;
					obj->IsRunning = true;
					TimerListHead = obj;
					TIM2_Tick_EventStart = TIM2_Tick_Present( );   //Sets a timeout with the duration "timestamp"
					if( TimerListHead->Timestamp <= HW_TIMER_TIME_BASE + 1 ) //不足最小单位，设置为一个最小单位
							TIM2_Tick_EventEnd = TIM2_Tick_EventStart + 1;
					else
							TIM2_Tick_EventEnd = TIM2_Tick_EventStart+((TimerListHead->Timestamp-1)/HW_TIMER_TIME_BASE);
    }
    else
    {
        if( TimerListHead->IsRunning == true )
        {
            elapsedTime =( ( TIM2_Tick_Present( ) - TIM2_Tick_EventStart ) + 1 )  * HW_TIMER_TIME_BASE;
            if( elapsedTime > TimerListHead->Timestamp )
            {
                elapsedTime = TimerListHead->Timestamp; // security but should never occur
            }
            remainingTime = TimerListHead->Timestamp - elapsedTime;
        }
        else
        {
            remainingTime = TimerListHead->Timestamp;
        }

        if( obj->Timestamp < remainingTime )
        {
						TimerEvent_t* NewSecond = TimerListHead;
						NewSecond->Timestamp = remainingTime - obj->Timestamp;
						NewSecond->IsRunning = false;
						obj->Next = NewSecond;
						obj->IsRunning = true;
						TimerListHead = obj;
						TIM2_Tick_EventStart = TIM2_Tick_Present( );
						if( TimerListHead->Timestamp <= HW_TIMER_TIME_BASE + 1 )
								TIM2_Tick_EventEnd = TIM2_Tick_EventStart + 1;
						else
								TIM2_Tick_EventEnd = TIM2_Tick_EventStart + ( ( TimerListHead->Timestamp - 1 ) / HW_TIMER_TIME_BASE );
        }
        else
        {
/* 
Adds a timer to the list.The list is automatically sorted. The list head always contains the next timer to expire.
obj: Timer object to be added to the list
remainingTime:Remaining time of the running head after which the object may be added
 */
						uint32_t aggregatedTimestamp = 0;      // hold the sum of timestamps
						uint32_t aggregatedTimestampNext = 0;  // hold the sum of timestamps up to the next event
						TimerEvent_t* prev = TimerListHead;
						TimerEvent_t* cur = TimerListHead->Next;
						if( cur == NULL )
						{ // obj comes just after the head
								obj->Timestamp -= remainingTime;
								prev->Next = obj;
								obj->Next = NULL;
						}
						else
						{
								aggregatedTimestamp = remainingTime;
								aggregatedTimestampNext = remainingTime + cur->Timestamp;

								while( prev != NULL )
								{
										if( aggregatedTimestampNext > obj->Timestamp )
										{
												obj->Timestamp -= aggregatedTimestamp;
												if( cur != NULL )
												{
														cur->Timestamp -= obj->Timestamp;
												}
												prev->Next = obj;
												obj->Next = cur;
												break;
										}
										else
										{
												prev = cur;
												cur = cur->Next;
												if( cur == NULL )
												{ // obj comes at the end of the list
														aggregatedTimestamp = aggregatedTimestampNext;
														obj->Timestamp -= aggregatedTimestamp;
														prev->Next = obj;
														obj->Next = NULL;
														break;
												}
												else
												{
														aggregatedTimestamp = aggregatedTimestampNext;
														aggregatedTimestampNext = aggregatedTimestampNext + cur->Timestamp;
												}
										}
								}
						}
        }
    }
    __enable_irq( );
}


void TimerEvent_Stop( TimerEvent_t *obj )
{
    __disable_irq( );

    uint32_t elapsedTime = 0;
    uint32_t remainingTime = 0;

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead;

    // List is empty or the Obj to stop does not exist
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        __enable_irq( );
        return;
    }

    if( TimerListHead == obj ) // Stop the Head
    {
        if( TimerListHead->IsRunning == true ) // The head is already running
        {
            elapsedTime = ( ( TIM2_Tick_Present( ) - TIM2_Tick_EventStart ) + 1 )  * HW_TIMER_TIME_BASE;
            if( elapsedTime > obj->Timestamp )
            {
                elapsedTime = obj->Timestamp;
            }

            remainingTime = obj->Timestamp - elapsedTime;

            if( TimerListHead->Next != NULL )
            {
                TimerListHead->IsRunning = false;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
                TimerListHead->IsRunning = true;
                
								TIM2_Tick_EventStart = TIM2_Tick_Present( );
								if( TimerListHead->Timestamp <= HW_TIMER_TIME_BASE + 1 )
										TIM2_Tick_EventEnd = TIM2_Tick_EventStart + 1;
								else
										TIM2_Tick_EventEnd = TIM2_Tick_EventStart + ( ( TimerListHead->Timestamp - 1 ) / HW_TIMER_TIME_BASE );	
            }
            else
            {
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {
            if( TimerListHead->Next != NULL )
            {
                remainingTime = obj->Timestamp;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
            }
            else
            {
                TimerListHead = NULL;
            }
        }
    }
    else // Stop an object within the list
    {
        remainingTime = obj->Timestamp;

        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                    cur->Timestamp += remainingTime;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }
    }
    __enable_irq( );
}


void TimerReset( TimerEvent_t *obj )
{
    TimerEvent_Stop( obj );
    TimerEvent_Start( obj );
}




void TimerTask(void )
{
	if( TIM2_Tick >= TIM2_Tick_EventEnd )
	{
			if( TimerListHead == NULL )
					return;  // Only necessary when the standard timer is used as a time base
			if( ((TIM2_Tick_Present()-TIM2_Tick_EventStart)+1)*HW_TIMER_TIME_BASE > TimerListHead->Timestamp )
					TimerListHead->Timestamp = 0;
			else
					TimerListHead->Timestamp -= ((TIM2_Tick_Present()-TIM2_Tick_EventStart)+1)*HW_TIMER_TIME_BASE;
			
			while((TimerListHead!= NULL)&&(TimerListHead->Timestamp==0))// excute all the expired object of the list
			{
				TimerListHead->Callback();
				TimerListHead = TimerListHead->Next;
			}
			
			if( TimerListHead != NULL )// start the next TimerListHead if it exists
			{
					TimerListHead->IsRunning = true;		
					TIM2_Tick_EventStart = TIM2_Tick_Present( );
					if( TimerListHead->Timestamp <= HW_TIMER_TIME_BASE + 1 )
							TIM2_Tick_EventEnd = TIM2_Tick_EventStart + 1;
					else
							TIM2_Tick_EventEnd = TIM2_Tick_EventStart + ( ( TimerListHead->Timestamp - 1 ) / HW_TIMER_TIME_BASE );
			}
	}	
}

#define PWM_PERIOD    20 //period=100us*pwm_period
void TIM2_IRQHandler( void )
{	
	HAL_TIM_IRQHandler(&TimHandle);	
	
	__disable_irq( );
	static uint8_t pwm_count=0;
	static uint8_t pwm_value=0;//high level time = 100us*pwm_scale		
	/*
	if(pwm_value == 0x00)
	{
		//如果使用HalLedSet函数，在调试时程序会跑飞；
		//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,OFF);	
		 GPIOC->BRR = GPIO_PIN_13 ;
	//	 __enable_irq( );
		return;
	}*/
	if(pwm_count==0)
	{
			if(pwm_value)//pwm_value必须不为0
				GPIOC->BSRR = GPIO_PIN_13;// set I/0
			else
				GPIOC->BRR = GPIO_PIN_13 ;//clr I/O
	}
	else if(pwm_count==pwm_value)
			GPIOC->BRR = GPIO_PIN_13 ;//clr I/O
	pwm_count++;
	if(pwm_count>=PWM_PERIOD)
	{
		pwm_value = GetPwmScale();	
		pwm_count = 0;
	}	
	 __enable_irq( );

	

  __disable_irq( );
  TIM2_Tick++;		
  __enable_irq( );
}










