/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. This file may only be built (assembled or compiled and linked)
**  using the Atollic TrueSTUDIO(R) product. The use of this file together
**  with other tools than Atollic TrueSTUDIO(R) is not permitted.
**
*****************************************************************************
*/

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

typedef enum {FALSE, TRUE} boolean;

boolean checkFirmwareUpdate(){
	return FALSE;
}

boolean performFirmwareUpdate(){
	return TRUE;
}


/* Application start address */
#define APPLICATION_ADDRESS        0x0800C000

typedef void (*pFunction)(void);

/**
**===========================================================================
**  Abstract: Bootloader
**===========================================================================
*/
int boot_main(void)
{

	pFunction appEntry;
	uint32_t appStack;

	/* Check if firmware update required */
	if(checkFirmwareUpdate()){

		/* Perform the update */
		performFirmwareUpdate();

	}

	/* Get the application stack pointer (First entry in the application vector table) */
	appStack = (uint32_t) *((__IO uint32_t*)APPLICATION_ADDRESS);

	/* Get the application entry point (Second entry in the application vector table) */
	appEntry = (pFunction) *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);

	/* Reconfigure vector table offset register to match the application location */
	SCB->VTOR = APPLICATION_ADDRESS;

	/* Set the application stack pointer */
	__set_MSP(appStack);

	/* Start the application */
	appEntry();

	while(1);

}






