/**
  ******************************************************************************
  * @file    STM32F072B_Ex02_Linear_DISCO\inc\tsl_user.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    04-April-2014
  * @brief   Touch-Sensing user configuration and api file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TSL_USER_H
#define __TSL_USER_H

#include "tsl.h"

//==============================================================================
//                             U S E R    S E T T I N G S
//==============================================================================

// STMStudio software usage (0=No, 1=Yes)
// Warning: The low-power mode must be disabled when STMStudio is used.
#define USE_STMSTUDIO (0)

//==============================================================================

// LEDs definition on STM32072B Discovery board
// LED3 = PC6
#define LED3_TOGGLE {GPIOC->ODR ^= (uint16_t)((uint32_t)1 << 6);}
#define LED3_ON    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)6);}
#define LED3_OFF   {GPIOC->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)6 + 16));}
// LED4 = PC8
#define LED4_TOGGLE {GPIOC->ODR ^= (uint16_t)((uint32_t)1 << 8);}
#define LED4_ON     {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)8);}
#define LED4_OFF    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)8 + 16));}
// LED5 = PC9
#define LED5_TOGGLE {GPIOC->ODR ^= (uint16_t)((uint32_t)1 << 9);}
#define LED5_ON     {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)9);}
#define LED5_OFF    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)9 + 16));}
// LED6 = PC7
#define LED6_TOGGLE {GPIOC->ODR ^= (uint16_t)((uint32_t)1 << 7);}
#define LED6_ON     {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)7);}
#define LED6_OFF    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)7 + 16));}

#if USE_STMSTUDIO > 0
#include "stmCriticalSection.h"
#define STMSTUDIO_LOCK {enterLock();}
#define STMSTUDIO_UNLOCK {exitLock();}
#else
#define STMSTUDIO_LOCK
#define STMSTUDIO_UNLOCK
#endif

//=======================
// Channel IOs definition
//=======================

#define CHANNEL_0_IO_MSK    (TSL_GROUP1_IO3)
#define CHANNEL_0_GRP_MSK   (TSL_GROUP1)
#define CHANNEL_0_SRC       (0) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_0_DEST      (0) // Index in destination result array

#define CHANNEL_1_IO_MSK    (TSL_GROUP2_IO3)
#define CHANNEL_1_GRP_MSK   (TSL_GROUP2)
#define CHANNEL_1_SRC       (1) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_1_DEST      (1) // Index in destination result array

#define CHANNEL_2_IO_MSK    (TSL_GROUP3_IO2)
#define CHANNEL_2_GRP_MSK   (TSL_GROUP3)
#define CHANNEL_2_SRC       (2) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_2_DEST      (2) // Index in destination result array

//======================
// Shield IOs definition
//======================

#define SHIELD_IO_MSK       (0) // No shield on this board

//=================
// Banks definition
//=================

#define BANK_0_NBCHANNELS    (3)
#define BANK_0_MSK_CHANNELS  (CHANNEL_0_IO_MSK  | CHANNEL_1_IO_MSK | CHANNEL_2_IO_MSK)
#define BANK_0_MSK_GROUPS    (CHANNEL_0_GRP_MSK | CHANNEL_1_GRP_MSK | CHANNEL_2_GRP_MSK) // Only these groups will be acquired

// User Parameters
extern CONST TSL_Bank_T MyBanks[];
extern CONST TSL_LinRot_T MyLinRots[];
extern CONST TSL_Object_T MyObjects[];
extern TSL_ObjectGroup_T MyObjGroup;
extern uint32_t Gv_ProcessSensor;

void TSL_user_Init(void);
TSL_Status_enum_T TSL_user_Action(void);
void TSL_user_SetThresholds(void);

#endif /* __TSL_USER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/