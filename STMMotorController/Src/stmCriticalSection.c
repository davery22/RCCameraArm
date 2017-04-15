/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : stmCriticalSection.c
* Author             : MCD Tools Development
* Version            : V1.0
* Date               : 21/02/2013
* Description        : This file provides a mechanism for STMStudio host/target
*                      synchronization. Based on a critical section, using few
*                      target resources (in term of code and RAM), but
*                      potentially impacting the application runtime (possible
*                      waiting loop when enterring the critical section).
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Implementation of Peterson's algorithm for mutual exclusive access to
   data, between the STMStudio host and the target processor.
	 The use of the critical section ensures the coherence of a set of data: it is
	 not possible for the STMStudio host to read data while the target is in the
	 critical section.
	 The target should enter the critical section (waiting loop possible, if the
	 STMStudio host is being reading) before modifying data that are identified as
	 critical ones. Then leave the critical section in order to allow the
	 STMStudio host to read them.
	 The host also enters the critical section before each reading, and leaves it
	 afterwards.
	 Note that it is not mandatory for the target to protect all spied variables
	 into the critical section; and that the synchronization might generate
	 relatively long waiting loops on the target side. As a result the critical
	 section should be used only for word-variables or group of variables for
	 which the coherence is important.
*/

#include "stmCriticalSection.h"

#define TARGET_LOCK_ID 0 // Do not modify - shared with STMStudio host software
#define HOST_LOCK_ID   1 // Do not modify - shared with STMStudio host software

typedef struct petersons_t {
    volatile unsigned char flag[2]; // Do not modify - shared with STMStudio host software
    volatile unsigned char turn;    // Do not modify - shared with STMStudio host software
} petersons_t;

// stm_studio_lock symbol used by the STMStudio host software for synchronization
petersons_t stm_studio_lock = { { 0, 0 }, TARGET_LOCK_ID }; // Do not modify - shared with STMStudio host software

void enterLock (void) {
    stm_studio_lock.flag[TARGET_LOCK_ID] = 1;
    stm_studio_lock.turn = HOST_LOCK_ID;
    while (stm_studio_lock.flag[HOST_LOCK_ID] && (stm_studio_lock.turn == HOST_LOCK_ID)) {}
}

void exitLock (void) {
    stm_studio_lock.flag[TARGET_LOCK_ID] = 0;
}
