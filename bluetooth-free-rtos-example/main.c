//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

char command[4] = "";
int commandIdx = 0;
SemaphoreHandle_t xTask2Semaphore;

void UART1IntHandler();

void initializePeripherals()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
}

void initializeGPIOFLeds()
{
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void initializeUART1Bluetooth()
{
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    IntPrioritySet(INT_UART1, 0xA0);
    IntRegister(INT_UART1_TM4C123, UART1IntHandler);

    UARTEnable(UART1_BASE);
    UARTStdioConfig(1, 9600, SysCtlClockGet());

    UARTprintf("UART1 Initialized Successfully!\n\r");
}

void UART1IntHandler(void)
{
	UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, true));

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(UARTCharsAvail(UART1_BASE))
    {
    	char recvChar = UARTCharGet(UART1_BASE);
    	command[commandIdx] = recvChar;
    	UARTCharPut(UART1_BASE, recvChar);
    	if(commandIdx == 2)
    	{
    		if(strcmp(command, "tog") == 0)
    		{
    		    // Give the semaphore and check if a higher-priority task was woken
    		    xSemaphoreGiveFromISR(xTask2Semaphore, &xHigherPriorityTaskWoken);
    		}
    		UARTCharPut(UART1_BASE, '\n');
    		UARTCharPut(UART1_BASE, '\r');
    	}
    	commandIdx = (commandIdx+1) % 3;
    }

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void vTask1(void* pvParameters)
{
	while(1)
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) ^ GPIO_PIN_1);

		vTaskDelay(500);
	}
}

void vTask2(void* pvParameters)
{
	while(1)
	{
        if (xSemaphoreTake(xTask2Semaphore, portMAX_DELAY) == pdTRUE)
        {
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) ^ GPIO_PIN_2);
        }
	}
}

int main(void)
{
	// Set Clock Speed to 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    initializePeripherals();

    initializeGPIOFLeds();

    initializeUART1Bluetooth();

    xTask2Semaphore = xSemaphoreCreateBinary();

    xTaskCreate(vTask1, "Task 1", 256, NULL, 10, NULL);
    xTaskCreate(vTask2, "Task 2", 256, NULL, 10, NULL);

    vTaskStartScheduler();

    while(1)
    {
    }
}
