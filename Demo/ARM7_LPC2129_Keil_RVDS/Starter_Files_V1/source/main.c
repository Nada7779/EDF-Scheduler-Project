/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */


/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>
#include "lpc21xx.h"
#include "semphr.h"
#include <limits.h>
#include "queue.h"
/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
/* Tasks Hnadlers*/
TaskHandle_t Button_1_Monitor_Handler = NULL; 
TaskHandle_t Button_2_Monitor_Handler = NULL; 
TaskHandle_t Periodic_Transmitter_Handler = NULL;
TaskHandle_t Uart_Receiver_Handler = NULL;
TaskHandle_t Load_1_Simulation_Handler = NULL;
TaskHandle_t Load_2_Simulation_Handler = NULL;

#define BTN1_PRIODICITY     50
#define BTN2_PRIODICITY     50
#define TR_PRIODICITY       100
#define UART_PRIODICITY     20  
#define LOAD1_PRIODICITY    10  
#define LOAD2_PRIODICITY    100




/* Semaphore object */
SemaphoreHandle_t button_released;

/* Queue object */
xQueueHandle gl_queue_handle;

// changes




/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
 /* Global Variables */
 uint8_t string_size;
 
 /* PRE_DEFINES */
#define QUEUE_MAX           10
#define CONSUMER_DELAY      10
#define STRING_SIZE         17
#define SEND_DELAY          100
#define READ_BUTTON_DELAY   10
#define PRESSED             1
#define RELEASED            0
#define  BUTTON_1_PORT  PORT_0
#define  BUTTON_2_PORT  PORT_0
#define  BUTTON_1_PIN   PIN0
#define  BUTTON_2_PIN   PIN1

#define BTN1_PRIODICITY     50
#define BTN2_PRIODICITY     50
#define TR_PRIODICITY       100
#define UART_PRIODICITY     20  
#define LOAD1_PRIODICITY    10  
#define LOAD2_PRIODICITY    100
/* TaskS to be created */

/* BUTTON_1_TASK  to detect the button 1 on (port 0 pin0) rising and falling edges  Every edge is an event that will be sent to a consumer_task */
void Button_1_Monitor (void * pvParameters)
{
	/* local variables */
	uint8_t lc_u8_button_pressed=RELEASED;

	pinState_t lc_u8_button_state;
	
	const char* lc_ptr_ch_button_1_rising= "Button_1_RISING \n";
	const char* lc_ptr_ch_button_1_falling= "Button_1_FALLING\n";
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = BTN1_PRIODICITY;
	
	//vTaskSetApplicationTaskTag( NULL, (void*) PIN2);
	
	/* init the xLastWakeTime with the current time */
	xLastWakeTime = xTaskGetTickCount();
	
	for( ;; )
	{
/* Task Code*/
		GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
		lc_u8_button_state= GPIO_read(BUTTON_1_PORT, BUTTON_1_PIN);
				if (lc_u8_button_state == PIN_IS_HIGH && lc_u8_button_pressed==RELEASED )
				{ 
				  xQueueSend(gl_queue_handle,&lc_ptr_ch_button_1_rising,portMAX_DELAY);
					
					lc_u8_button_pressed=PRESSED;
				}
				else if (lc_u8_button_state == PIN_IS_LOW && lc_u8_button_pressed==PRESSED)
				{
					xQueueSend(gl_queue_handle,&lc_ptr_ch_button_1_falling,portMAX_DELAY);
					
           lc_u8_button_pressed=RELEASED;
				}				
		//vTaskDelay(READ_BUTTON_DELAY);	
				vTaskDelayUntil(&xLastWakeTime, xFrequency);
						GPIO_write(PORT_0,PIN7,PIN_IS_LOW);

	}
}
/* BUTTON_2_TASK  to detect the button 1 on (port 0 pin1) rising and falling edges  Every edge is an event that will be sent to a consumer_task */
void Button_2_Monitor (void * pvParameters)
{
	/* local variables */
	uint8_t lc_u8_button_pressed=RELEASED;

	pinState_t lc_u8_button_state;
	
	const char* lc_ptr_ch_button_1_rising= "Button_2_RISING \n";
	const char* lc_ptr_ch_button_1_falling= "Button_2_FALLING\n";
	
		TickType_t xLastWakeTime;
	const TickType_t xFrequency = BTN2_PRIODICITY;
	
	//vTaskSetApplicationTaskTag( NULL, (void*) PIN2);
	
	/* init the xLastWakeTime with the current time */
	xLastWakeTime = xTaskGetTickCount();
	
	
	for( ;; )
	{
/* Task Code*/
		lc_u8_button_state= GPIO_read(BUTTON_2_PORT, BUTTON_2_PIN);
				if (lc_u8_button_state == PIN_IS_HIGH && lc_u8_button_pressed==RELEASED )
				{ 
				  xQueueSend(gl_queue_handle,&lc_ptr_ch_button_1_rising,portMAX_DELAY);
						
					lc_u8_button_pressed=PRESSED;
				}
				else if (lc_u8_button_state == PIN_IS_LOW && lc_u8_button_pressed==PRESSED)
				{
					xQueueSend(gl_queue_handle,&lc_ptr_ch_button_1_falling,portMAX_DELAY);
					
           lc_u8_button_pressed=RELEASED;
				}				
		//vTaskDelay(READ_BUTTON_DELAY);	
					vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
/* This task will send a periodic string  every 100ms to the consumer task */
void Periodic_Transmitter (void * pvParameters)
{	
		const char* lc_ptr_ch_send_string= "periodic string \n";
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TR_PRIODICITY;
	
	vTaskSetApplicationTaskTag( NULL, (void*) PIN6);
	
	/* init the xLastWakeTime with the current time */
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
	
      xQueueSend(gl_queue_handle,&lc_ptr_ch_send_string,portMAX_DELAY);

		//vTaskDelay(SEND_DELAY);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
}
/* This task will send the strings recieved from buttons and send tasks to the uart */
void Uart_Receiver (void * pvParameters)
{	
	const char* lc_ptr_ch_receive_string;
			
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = UART_PRIODICITY;
	
	vTaskSetApplicationTaskTag( NULL, (void*) PIN4);
	
	/* init the xLastWakeTime with the current time */
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
     xQueueReceive(gl_queue_handle,&lc_ptr_ch_receive_string,portMAX_DELAY);
		
		vSerialPutString((const signed char*)lc_ptr_ch_receive_string,STRING_SIZE);
		
		//vTaskDelay(CONSUMER_DELAY);	
				vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
}
/* this task to create empty loop that loops X times to be with Execution time= 5ms*/
void Load_1_Simulation (void * pvParameters)
{	
	 int i=0; 
		TickType_t xLastWakeTime;
	const TickType_t xFrequency = LOAD1_PRIODICITY;
	
	
	vTaskSetApplicationTaskTag( NULL, (void*) PIN3);
	
	/* init the xLastWakeTime with the current time */
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{  
		GPIO_write(PORT_0, PIN3,PIN_IS_HIGH);
		for (i=0;i<37500;i++)
		{
			;
		}
				GPIO_write(PORT_0, PIN3,PIN_IS_LOW);

		
						vTaskDelayUntil(&xLastWakeTime, xFrequency);

		

	}
}
/* this task to create empty loop that loops X times to be with Execution time= 12ms */
void Load_2_Simulation (void * pvParameters)
{	
	 int i=0; 
		TickType_t xLastWakeTime;
	const TickType_t xFrequency = LOAD2_PRIODICITY;
	
	
	vTaskSetApplicationTaskTag( NULL, (void*) PIN0);
	
	/* init the xLastWakeTime with the current time */
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{   	GPIO_write(PORT_0, PIN2,PIN_IS_HIGH);
  
		for (i=0;i<90000;i++)
		{
			;
		}
			GPIO_write(PORT_0, PIN2,PIN_IS_LOW);

			vTaskDelayUntil(&xLastWakeTime, xFrequency);

		
		  /* Indicate Idle Task stop*/

	GPIO_write(PORT_0,PIN4,PIN_IS_LOW);

	}
}

/* Application tick hook callout */
void vApplicationTickHook (void)
{

	GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
	
	GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	
}

/* Application idle hook callout */

void vApplicationIdleHook (void)
{

static int TagInit = 0;
	
	if(0 == TagInit)
	{
	vTaskSetApplicationTaskTag( NULL, (void*) PIN7);
	
	TagInit = 1;
	}
	
}

	

/*****************************************************************/

extern BaseType_t  xTaskPeriodicCreate(	TaskFunction_t pxTaskCode,
							const char * const pcName,		/*lint !e971 Unqualified char types are allowed for strings and single characters only. */
							const configSTACK_DEPTH_TYPE usStackDepth,
							void * const pvParameters,
							UBaseType_t uxPriority,
							TaskHandle_t * const pxCreatedTask, TickType_t period );

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
		/* Create queue */

	
		gl_queue_handle = xQueueCreate(QUEUE_MAX, sizeof(const char*));



	    /* Create Tasks here */

/* create Button_1_Monitor */
		xTaskPeriodicCreate(
               Load_2_Simulation,                 /* function that implements the task */
	             "Load_2_Simulation",                /* task's name */
	             configMINIMAL_STACK_SIZE, /* stack size in words */
	             (void *) NULL,           /* parameter passed to the task */
		           2,                      /* task's priority */
		           &Load_2_Simulation_Handler,    /* task's handler */
							 100                      /*used to pass task periodicity */
);
							 							 /* create Load_1_Simulation */  
	xTaskPeriodicCreate(
               Load_1_Simulation,                 /* function that implements the task */
	             "Load_1_Simulation",                /* task's name */
	             configMINIMAL_STACK_SIZE, /* stack size in words */
	             (void *) NULL,           /* parameter passed to the task */
		           2,                      /* task's priority */
		           &Load_1_Simulation_Handler,    /* task's handler */
							 10                      /*used to pass task periodicity */
);
//	xTaskPeriodicCreate(
//               Button_1_Monitor,                 /* function that implements the task */
//	             "Button_1_Monitor",                /* task's name */
//	             configMINIMAL_STACK_SIZE, /* stack size in words */
//	             (void *) NULL,           /* parameter passed to the task */
//		           2,                      /* task's priority */
//		           &Button_1_Monitor_Handler, /* task's handler */
//							 50                      /*used to pass task periodicity */
//);
/* create Button_2_Monitor */
//	xTaskPeriodicCreate(
//               Button_2_Monitor,                 /* function that implements the task */
//	             "Button_2_Monitor",                /* task's name */
//	             configMINIMAL_STACK_SIZE, /* stack size in words */
//	             (void *) NULL,           /* parameter passed to the task */
//		           2,                      /* task's priority */
//		           &Button_2_Monitor_Handler,    /* task's handler */
//							 50                      /*used to pass task periodicity */
//);

/* create Uart_Receiver */  
	xTaskPeriodicCreate(
               Uart_Receiver,                 /* function that implements the task */
	             "Uart_Receiver",                /* task's name */
	             configMINIMAL_STACK_SIZE, /* stack size in words */
	             (void *) NULL,           /* parameter passed to the task */
		           2,                      /* task's priority */
		           &Uart_Receiver_Handler,    /* task's handler */
							 20                     /*used to pass task periodicity */
);
	/* create Periodic_Transmitter */
	xTaskPeriodicCreate(
               Periodic_Transmitter,                 /* function that implements the task */
	             "Periodic_Transmitter",                /* task's name */
	             configMINIMAL_STACK_SIZE, /* stack size in words */
	             (void *) NULL,           /* parameter passed to the task */
		           2,                      /* task's priority */
		           &Periodic_Transmitter_Handler,    /* task's handler */
							  100                     /*used to pass task periodicity */
);

							 /* create Load_2_Simulation */  

			
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


