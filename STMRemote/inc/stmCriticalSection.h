/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : stmCriticalSection.h
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

#ifndef STM_CRITICAL_SECTION_H
#define STM_CRITICAL_SECTION_H

/* To call before modifying any critical data. In case the host is inside or
asked for enterring the critical section, this routine will wait for the host to
leave the critical section. */
void enterLock(void);

/* Leave the critical section. If the host is waiting, access will be granted
to him. Otherwise the first next one asking will own the turn. */
void exitLock(void);

#endif /* STM_CRITICAL_SECTION_H */
