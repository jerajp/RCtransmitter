/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t T1statusdebounce=0;
uint32_t T2statusdebounce=0;
uint32_t T3statusdebounce=0;
uint32_t T4statusdebounce=0;

uint32_t T1statusdebounceHist=0;
uint32_t T2statusdebounceHist=0;
uint32_t T3statusdebounceHist=0;
uint32_t T4statusdebounceHist=0;

uint32_t T1status=0;
uint32_t T2status=0;
uint32_t T3status=0;
uint32_t T4status=0;

uint32_t T1count=0;
uint32_t T2count=0;
uint32_t T3count=0;
uint32_t T4count=0;

uint32_t TDstatusdebounce=0;
uint32_t TLstatusdebounce=0;
uint32_t TOGGLstatusdebounce=0;
uint32_t TOGGDstatusdebounce=0;

uint32_t TDstatus=0;
uint32_t TLstatus=0;
uint32_t TOGGLstatus=0;
uint32_t TOGGDstatus=0;

uint32_t TDcount=0;
uint32_t TLcount=0;
uint32_t TOGGLcount=0;
uint32_t TOGGDcount=0;

uint32_t BattmV=0;
uint32_t BattmVSUM=0;
uint32_t BattmVAVG=0;
uint32_t Batt1cellAVG;
uint32_t batthistindx=0;
uint32_t BAttmVhist[BATTAVERAGETIME];
uint32_t motorSTAT=0;

uint32_t i;

uint32_t LjoyUPDOWN=0;
uint32_t LjoyLEFTRIGHT=0;
uint32_t DjoyUPDOWN=0;
uint32_t DjoyLEFTRIGHT=0;

int32_t offsetLjoyUPDOWN=0;
int32_t offsetLjoyLEFTRIGHT=0;
int32_t offsetDjoyUPDOWN=0;
int32_t offsetDjoyLEFTRIGHT=0;

int32_t LjoyUPDOWNzeroOffset;
int32_t LjoyLEFTRIGHTzeroOffset;
int32_t DjoyUPDOWNzeroOffset;
int32_t DjoyLEFTRIGHTzeroOffset;

uint32_t potenc1=0;
uint32_t potenc2=0;

uint32_t offsetsetcount=0;

uint32_t TXdelay=0;
uint32_t PingRXDataFlag=0;

uint8_t Buttons;

uint32_t DroneBattUpperByte;
uint32_t DroneBattLowerByte;
uint32_t DroneBattmV;
int32_t DronePitchAngle;
int32_t DroneRollAngle;
uint32_t GyroCalibInProgress=0;

uint32_t TotalMSGsend;
uint32_t TotalMSGrecv;
uint32_t MSGcount;
uint32_t MSGprerSecond;
uint32_t MSGLowCount;
uint32_t ConnectWeakFlag;

uint32_t  LoopCounter=0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
/* USER CODE BEGIN EV */

extern uint32_t watch1;
extern uint32_t watch2;
extern uint32_t watch3;
extern uint32_t watch4;
extern uint32_t watch5;
extern uint32_t test1;
extern uint32_t test2;
extern uint32_t test3;
extern uint32_t test4;
extern uint16_t adcDataArray[7];
extern uint8_t nRF24_payloadTX[32]; //TX buffer
extern uint8_t nRF24_payloadRX[32]; //RX buffer
extern const uint8_t nRF24_ADDR[3];
extern uint8_t RXstpaketov;
extern uint32_t MainInitDoneFlag;
extern uint32_t LCDMenu;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  //Read push-buttons, internal pull-up, switch to GND
  T1status=!HAL_GPIO_ReadPin(T1_GPIO_Port,T1_Pin);
  T2status=!HAL_GPIO_ReadPin(T2_GPIO_Port,T2_Pin);
  T3status=!HAL_GPIO_ReadPin(T3_GPIO_Port,T3_Pin);
  T4status=!HAL_GPIO_ReadPin(T4_GPIO_Port,T4_Pin);
  TDstatus=!HAL_GPIO_ReadPin(DT_GPIO_Port,DT_Pin);
  TLstatus=!HAL_GPIO_ReadPin(LT_GPIO_Port,LT_Pin);
  TOGGLstatus=HAL_GPIO_ReadPin(TOGGL_GPIO_Port,TOGGL_Pin);
  TOGGDstatus=HAL_GPIO_ReadPin(TOGGD_GPIO_Port,TOGGD_Pin);
  //INTERPRET BUTTONS-----------------------------------------

  //Save old values
  T1statusdebounceHist=T1statusdebounce;
  T2statusdebounceHist=T2statusdebounce;
  T3statusdebounceHist=T3statusdebounce;
  T4statusdebounceHist=T4statusdebounce;


  //Set flag on T1 press
  if(T1status==1)
  {
	  T1count++;
  }
  else
  {
	  T1count=0;
	  T1statusdebounce=0;
  }

  if(T1count==BUTTONTHRESHOLD)
  {
	  T1statusdebounce=1;
  }

  //Set flag on T2 press
  if(T2status==1)
  {
	  T2count++;
  }
  else
  {
	  T2count=0;
	  T2statusdebounce=0;
  }

  if(T2count==BUTTONTHRESHOLD)
  {
	  T2statusdebounce=1;
  }

  //Set flag on T3 press
  if(T3status==1)
  {
	  T3count++;
  }
  else
  {
	  T3count=0;
	  T3statusdebounce=0;
  }


  if(T3count==BUTTONTHRESHOLD)
  {
	  T3statusdebounce=1;
  }

  //Set flag on T4 press
  if(T4status==1)
  {
	  T4count++;
  }
  else
  {
	  T4count=0;
	  T4statusdebounce=0;
  }

  if(T4count==BUTTONTHRESHOLD)
  {
	  T4statusdebounce=1;
  }

  //Set flag on DTpress
  if(TDstatus==1)
  {
	  TDcount++;
  }
  else
  {
	  TDcount=0;
	  TDstatusdebounce=0;
  }

  if(TDcount==BUTTONTHRESHOLD)
  {
	  TDstatusdebounce=1;
  }

  //Set flag on LT press
    if(TLstatus==1)
    {
  	  TLcount++;
    }
    else
    {
  	  TLcount=0;
  	  TLstatusdebounce=0;
    }

    if(TLcount==BUTTONTHRESHOLD)
    {
  	  TLstatusdebounce=1;
    }

    //Set flag on TOGG1 press
      if(TOGGLstatus==1)
      {
    	  TOGGLcount++;
      }
      else
      {
    	  TOGGLcount=0;
    	  TOGGLstatusdebounce=0;
      }

      if(TOGGLcount==BUTTONTHRESHOLD)
      {
    	  TOGGLstatusdebounce=1;
      }

      //Set flag on TOGG2 press
        if(TOGGDstatus==1)
        {
      	  TOGGDcount++;
        }
        else
        {
      	  TOGGDcount=0;
      	  TOGGDstatusdebounce=0;
        }

        if(TOGGDcount==BUTTONTHRESHOLD)
        {
      	  TOGGDstatusdebounce=1;
        }

     //LEDS

     //LED 1 LOW BATT
     if( (Batt1cellAVG < MINREMOTEBATT) ||  (DroneBattmV < MINDRONEBATT) ) LED1ON;
     else LED1OFF;

     //LED2 RC connection OK
     if(ConnectWeakFlag)LED2OFF;
     else LED2ON;

     //LED1ON;
     //LED1OFF;


     //LED3 GYRO CALIB INDIC
     if(GyroCalibInProgress)LED3ON;
     else LED3OFF;

     //LED4ON;
     //LED4OFF;

     // T1 button Edge to 1
     if(T1statusdebounceHist!=T1statusdebounce && T1statusdebounce==1)
     {

     }

     // T2 button Edge to 1
     if(T2statusdebounceHist!=T2statusdebounce && T2statusdebounce==1)
     {

     }

     // T3 button Edge to 1
     if(T3statusdebounceHist!=T3statusdebounce && T3statusdebounce==1)
     {

     }

     // T4 button Edge to 1
     if(T4statusdebounceHist!=T4statusdebounce && T4statusdebounce==1)
     {
    	 LCDMenu++;
    	 if(LCDMenu>LCDLASTMENU)LCDMenu=0;
     }


     //ADC interpret---------------------------------------------------------------------------------------------

     //Battery value 12k/6.8k divider----------------------------------
     BattmV=(adcDataArray[6]*3300*2.735)/4095;

     //Battery average value--------------------------------------------
     BAttmVhist[batthistindx]=BattmV;
     batthistindx++;
     if(batthistindx>=BATTAVERAGETIME)batthistindx=0;

     BattmVSUM=0;

     for(i=0;i<BATTAVERAGETIME;i++)
     {
    	 BattmVSUM+=BAttmVhist[i];
     }

     BattmVAVG=BattmVSUM/(BATTAVERAGETIME);
     Batt1cellAVG=BattmVAVG/2; //display average voltage per 1 li-ion cell


     //Joysticks---------------------------------------------------------------

     LjoyUPDOWNzeroOffset=adcDataArray[0];
     LjoyLEFTRIGHTzeroOffset=adcDataArray[1];
     DjoyUPDOWNzeroOffset=adcDataArray[2];
     DjoyLEFTRIGHTzeroOffset=adcDataArray[3];

     //Zeroing joysticks to center
     if(T1statusdebounce==1 && T3statusdebounce==1)
     {
       offsetsetcount++;

       if(offsetsetcount==5000)
       {
         offsetLjoyUPDOWN=2047-LjoyUPDOWNzeroOffset;
         offsetLjoyLEFTRIGHT=2047-LjoyLEFTRIGHTzeroOffset;
     	 offsetDjoyUPDOWN=2047-DjoyUPDOWNzeroOffset;
     	 offsetDjoyLEFTRIGHT=2047-DjoyLEFTRIGHTzeroOffset;
        }
     }
     else offsetsetcount=0;

     LjoyUPDOWNzeroOffset+=offsetLjoyUPDOWN;
     LjoyLEFTRIGHTzeroOffset+=offsetLjoyLEFTRIGHT;
     DjoyUPDOWNzeroOffset+=offsetDjoyUPDOWN;
     DjoyLEFTRIGHTzeroOffset+=offsetDjoyLEFTRIGHT;

     if(LjoyUPDOWNzeroOffset< 0)LjoyUPDOWNzeroOffset=0;
     if(LjoyLEFTRIGHTzeroOffset<0)LjoyLEFTRIGHTzeroOffset=0;
     if(DjoyUPDOWNzeroOffset<0)DjoyUPDOWNzeroOffset=0;
     if(DjoyLEFTRIGHTzeroOffset<0)DjoyLEFTRIGHTzeroOffset=0;

     if(LjoyUPDOWNzeroOffset>4095)LjoyUPDOWNzeroOffset=4095;
     if(LjoyLEFTRIGHTzeroOffset>4095)LjoyLEFTRIGHTzeroOffset=4095;
     if(DjoyUPDOWNzeroOffset>4095)DjoyUPDOWNzeroOffset=4095;
     if(DjoyLEFTRIGHTzeroOffset>4095)DjoyLEFTRIGHTzeroOffset=4095;

     //scaling 0-100%
     LjoyUPDOWN=LjoyUPDOWNzeroOffset*100/4095;
     LjoyLEFTRIGHT=LjoyLEFTRIGHTzeroOffset*100/4095;
     DjoyUPDOWN=DjoyUPDOWNzeroOffset*100/4095;
     DjoyLEFTRIGHT=DjoyLEFTRIGHTzeroOffset*100/4095;

     //Potenciometers (inverted logic-HW)---------------------------
     potenc1=100-adcDataArray[4]*100/4095;
	 potenc2=100-adcDataArray[5]*100/4095;
     //------------------------------------------------------------------------------------------------------------

	 //Save Buttons into 8bits
	 Buttons=(TOGGLstatusdebounce<<7) +  (TOGGDstatusdebounce<<6) + (T1statusdebounce<<5) + (T2statusdebounce<<4) + (T3statusdebounce<<3) + (T4statusdebounce<<2) + (TLstatusdebounce<<1) + TDstatusdebounce;


	 //NRF24---------------------------------------------------------------------------------------
	 if(MainInitDoneFlag)
	 {

		 switch(TXdelay)
		 {
	 	 	 case 0:
	 	 	 	 	 {
	 	 	 	 		//SET TX MODE
	 	 	 	 		nRF24_CE_L();//END RX
						nRF24_SetOperationalMode(nRF24_MODE_TX);
						PingRXDataFlag=0;
	 	 	 	 	 }break;

		 	 case 5:
		 	 	 	 {
		 	 	 		//TRANSMIT
		 	 	 		TotalMSGsend++;

		 	 	 		//SEND DATA TO DRONE
		 	 	 		nRF24_payloadTX[0] = (uint8_t)(LjoyUPDOWN);
		 	 	 		nRF24_payloadTX[1] = (uint8_t)(LjoyLEFTRIGHT);
		 	 	 		nRF24_payloadTX[2] = (uint8_t)(DjoyUPDOWN);
		 	 	 		nRF24_payloadTX[3] = (uint8_t)(DjoyLEFTRIGHT);
		 	 	 		nRF24_payloadTX[4] = (uint8_t)(potenc1);
		 	 	 		nRF24_payloadTX[5] = (uint8_t)(potenc2);
		 	 	 		nRF24_payloadTX[6] = (uint8_t)(Buttons);

		 	 	 		// Transmit a packet
		 	 	 		nRF24_TransmitPacket(nRF24_payloadTX, 7);
		 	 	 	 }break;

		 	 case 6:
		 	 	 	 {
		 	 	 		 //SET RX MODE
		 				nRF24_SetOperationalMode(nRF24_MODE_RX);
		 				nRF24_CE_H(); //Start RX)

		 	 	 	 }break;

		 	 case 7:
		 	 	 	 {
		 	 	 		PingRXDataFlag=1;

		 	 	 	 }break;
		 }

		 if(PingRXDataFlag)//Ping for RX data
		 {

	 	 	 if ((nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) )
	 	 	 {
	 	 	 	// Get a payload from the transceiver
	 	 	 	nRF24_ReadPayload(nRF24_payloadRX, &RXstpaketov);

	 	 	 	//Clear all pending IRQ flags
	 	 	 	nRF24_ClearIRQFlags();

	 	 	 	DroneBattLowerByte=nRF24_payloadRX[0];
	 	 	 	DroneBattUpperByte=nRF24_payloadRX[1];
	 	 	 	DroneBattmV=(DroneBattUpperByte<<8)+DroneBattLowerByte;

	 	 	 	DronePitchAngle=nRF24_payloadRX[2];
	 	 	 	DroneRollAngle=nRF24_payloadRX[3];

				if(nRF24_payloadRX[4] & 0x1)DronePitchAngle*=(-1);
				else if((nRF24_payloadRX[4]>>1)& 0x1)DroneRollAngle*=(-1);

				GyroCalibInProgress=(nRF24_payloadRX[4]>>2) & 0x1;

				motorSTAT=(nRF24_payloadRX[4]>>3) & 0x7; //last 3 bits

				TotalMSGrecv++;
	 	 	 	MSGcount++;
	 	 	 }
		 }

		 //MSG PER SECOND DIAGNOSTICS
		 LoopCounter++;
		 if(LoopCounter==1000)
		 {
			 MSGprerSecond=MSGcount;

			 if(MSGcount<MINMSGPERSEC)
		     {
				 MSGLowCount++;
				 ConnectWeakFlag=1;
		     }
			 else  ConnectWeakFlag=0;

			 MSGcount=0;
			 LoopCounter=0;
		 }
		 TXdelay++;
		 if(TXdelay==TXPERIOD)TXdelay=0;
	 }//----------------------------------------------------------------------------------------------


  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
