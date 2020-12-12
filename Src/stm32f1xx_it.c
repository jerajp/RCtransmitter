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
#include "mcp23017.h"
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

//Button inputs LEFT + - x,Y axis joystick
uint32_t TJoyLeftXPlusStatus,TJoyLeftXMinusStatus,TJoyLeftYPlusStatus,TJoyLeftYMinusStatus;
uint32_t TJoyLeftXPlusStatusDebounce,TJoyLeftXMinusStatusDebounce,TJoyLeftYPlusStatusDebounce,TJoyLeftYMinusStatusDebounce;
uint32_t TJoyLeftXPlusStatusDebounceHIST,TJoyLeftXMinusStatusDebounceHIST,TJoyLeftYPlusStatusDebounceHIST,TJoyLeftYMinusStatusDebounceHIST;
uint32_t TJoyLeftXPlusCount,TJoyLeftXMinusCount,TJoyLeftYPlusCount,TJoyLeftYMinusCount;

//Button inputs RIGHT + - x,Y axis joystick
uint32_t TJoyRightXPlusStatus,TJoyRightXMinusStatus,TJoyRightYPlusStatus,TJoyRightYMinusStatus;
uint32_t TJoyRightXPlusStatusDebounce,TJoyRightXMinusStatusDebounce,TJoyRightYPlusStatusDebounce,TJoyRightYMinusStatusDebounce;
uint32_t TJoyRightXPlusStatusDebounceHIST,TJoyRightXMinusStatusDebounceHIST,TJoyRightYPlusStatusDebounceHIST,TJoyRightYMinusStatusDebounceHIST;
uint32_t TJoyRightXPlusCount,TJoyRightXMinusCount,TJoyRightYPlusCount,TJoyRightYMinusCount;

//LCD buttons
uint32_t T1Status,T2Status,T3Status,T4Status;
uint32_t T1StatusDebounce,T2StatusDebounce,T3StatusDebounce,T4StatusDebounce;
uint32_t T1StatusDebounceHIST,T2StatusDebounceHIST,T3StatusDebounceHIST,T4StatusDebounceHIST;
uint32_t T1Count,T2Count,T3Count,T4Count;

uint32_t TUpStatus,TDownStatus,TLeftStatus,TRightStatus;
uint32_t TUpStatusDebounce,TDownStatusDebounce,TLeftStatusDebounce,TRightStatusDebounce;
uint32_t TUpStatusDebounceHIST,TDownStatusDebounceHIST,TLeftStatusDebounceHIST,TRightStatusDebounceHIST;
uint32_t TUpCount,TDownCount,TLeftCount,TRightCount;

//Toggle inputs
uint32_t TOGG1statusdebounce,TOGG2statusdebounce,TOGG3statusdebounce,TOGG4statusdebounce,TOGG5statusdebounce,TOGG6statusdebounce;
uint32_t TOGG1status,TOGG2status,TOGG3status,TOGG4status,TOGG5status,TOGG6status;
uint32_t TOGG1count,TOGG2count,TOGG3count,TOGG4count,TOGG5count,TOGG6count;

//Battery voltage
uint32_t BattmV=0;
uint32_t BattmVSUM=0;
uint32_t BattmVAVG=0;
uint32_t Batt1cellAVG;
uint32_t batthistindx=0;
uint32_t BAttmVhist[BATTAVERAGETIME];

//LED blink
uint32_t LED4blinkcount;

//Status
uint32_t motorSTAT=0;

//Joystick Values
uint32_t LjoyUPDOWN=0;
uint32_t LjoyLEFTRIGHT=0;
uint32_t DjoyUPDOWN=0;
uint32_t DjoyLEFTRIGHT=0;

int32_t LjoyUPDOWNzeroOffset;
int32_t LjoyLEFTRIGHTzeroOffset;
int32_t DjoyUPDOWNzeroOffset;
int32_t DjoyLEFTRIGHTzeroOffset;

uint32_t LjoyXmin=LJOYXMINDEF;
uint32_t LjoyXmax=LJOYXMAXDEF;
uint32_t LjoyYmin=LJOYYMINDEF;
uint32_t LjoyYmax=LJOYYMAXDEF;
//
uint32_t RjoyXmin=RJOYXMINDEF;
uint32_t RjoyXmax=RJOYXMAXDEF;
uint32_t RjoyYmin=RJOYYMINDEF;
uint32_t RjoyYmax=RJOYYMAXDEF;

uint32_t potenc1=0;
uint32_t potenc2=0;

//Buzzer
uint32_t BuzzerRCBattpanic;
uint32_t BuzzerDroneBattpanic;
uint32_t RCBattPanicCount;
uint32_t DroneBattPanicCount;

//NRF24
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
uint32_t ConnectWeakFlag=1;
uint32_t  LoopCounter=0;

uint8_t MSGParameterTX; //Parameter selector Byte
uint8_t MSGParameterRX; //Parameter selector Byte

uint32_t MSGParamDataTX; //Parameter Data multiplied for 32bit format
uint32_t MSGParamDataRX; //Parameter Data multiplied for 32bit format

uint8_t MSGCommTX=COMMCONTROLDATA;	  //MSG Type (Control Data, Active Parameter, Flash Parameter, Write/Erase Flash)
uint8_t MSGCommRX;  				  //MSG Type (Control Data, Active Parameter, Flash Parameter, Write/Erase Flash)

//RC remote commands
uint32_t CommShtdownnrf24;
uint32_t i;


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

extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c2;

extern MCP23017str	MCP23017DataStr;
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
extern uint32_t LvLUp,LvlDown;
extern uint32_t MenuPlus,MenuMinus;
extern uint32_t ValuePlus,ValueMinus;
extern uint32_t CursorUp,CursorDown;
extern struct FlashDatastruct FlashDataActive;
extern struct DroneDataStruct DroneDataActive;
extern struct DroneDataStruct DroneDataFlash;
extern struct DroneDataStruct DroneDataTemp;
extern struct DroneDataStruct DroneDataInput;

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

  //GET PUSHBUTTON DATA
  mcp23017_readA(&hi2c2,MCP23017_ADDRESS_20,&MCP23017DataStr);
  mcp23017_readB(&hi2c2,MCP23017_ADDRESS_20,&MCP23017DataStr);

  //Get current Buttons/Toggle switch status
  TOGG1status=!HAL_GPIO_ReadPin(TOGG1_GPIO_Port,TOGG1_Pin);
  TOGG2status=!HAL_GPIO_ReadPin(TOGG2_GPIO_Port,TOGG2_Pin);
  TOGG3status=!HAL_GPIO_ReadPin(TOGG3_GPIO_Port,TOGG3_Pin);
  TOGG4status=!HAL_GPIO_ReadPin(TOGG4_GPIO_Port,TOGG4_Pin);
  TOGG5status=!HAL_GPIO_ReadPin(TOGG5_GPIO_Port,TOGG5_Pin);
  TOGG6status=!HAL_GPIO_ReadPin(TOGG6_GPIO_Port,TOGG6_Pin);

  //LEFT JOYSTIC
  TJoyLeftXPlusStatus=!MCP23017DataStr.gpioB[6];
  TJoyLeftXMinusStatus=!MCP23017DataStr.gpioB[7];
  TJoyLeftYPlusStatus=!MCP23017DataStr.gpioB[3];
  TJoyLeftYMinusStatus=!MCP23017DataStr.gpioB[2];

  //RIGHT JOYSTICK
  TJoyRightXPlusStatus=!MCP23017DataStr.gpioB[1];
  TJoyRightXMinusStatus=!MCP23017DataStr.gpioB[0];
  TJoyRightYPlusStatus=!MCP23017DataStr.gpioB[5];
  TJoyRightYMinusStatus=!MCP23017DataStr.gpioB[4];

  //LCD buttons
  T1Status=!MCP23017DataStr.gpioA[2];
  T2Status=!MCP23017DataStr.gpioA[3];
  T3Status=!MCP23017DataStr.gpioA[0];
  T4Status=!MCP23017DataStr.gpioA[1];

  TUpStatus=!MCP23017DataStr.gpioA[4];
  TDownStatus=!MCP23017DataStr.gpioA[6];
  TLeftStatus=!MCP23017DataStr.gpioA[5];
  TRightStatus=!MCP23017DataStr.gpioA[7];

  //DEBOUNCE ROUTINES

  //Save old debounced values
  TJoyLeftXPlusStatusDebounceHIST=TJoyLeftXPlusStatusDebounce;
  TJoyLeftXMinusStatusDebounceHIST=TJoyLeftXMinusStatusDebounce;
  TJoyLeftYPlusStatusDebounceHIST=TJoyLeftYPlusStatusDebounce;
  TJoyLeftYMinusStatusDebounceHIST=TJoyLeftYMinusStatusDebounce;

  TJoyRightXPlusStatusDebounceHIST=TJoyRightXPlusStatusDebounce;
  TJoyRightXMinusStatusDebounceHIST=TJoyRightXMinusStatusDebounce;
  TJoyRightYPlusStatusDebounceHIST=TJoyRightYPlusStatusDebounce;
  TJoyRightYMinusStatusDebounceHIST=TJoyRightYMinusStatusDebounce;

  T1StatusDebounceHIST=T1StatusDebounce;
  T2StatusDebounceHIST=T2StatusDebounce;
  T3StatusDebounceHIST=T3StatusDebounce;
  T4StatusDebounceHIST=T4StatusDebounce;

  TUpStatusDebounceHIST=TUpStatusDebounce;
  TDownStatusDebounceHIST=TDownStatusDebounce;
  TLeftStatusDebounceHIST=TLeftStatusDebounce;
  TRightStatusDebounceHIST=TRightStatusDebounce;


  //LEFT JOYSTICK X AXIS PLUS------------------------------------------
  if(TJoyLeftXPlusStatus==1)
  {
	  TJoyLeftXPlusCount++;
  }
  else
  {
	  TJoyLeftXPlusCount=0;
	  TJoyLeftXPlusStatusDebounce=0;
  }
  if(TJoyLeftXPlusCount==BUTTONTHRESHOLD)
  {
	  TJoyLeftXPlusStatusDebounce=1;
  }
  //LEFT JOYSTICK X AXIS MINUS
  if(TJoyLeftXMinusStatus==1)
  {
	  TJoyLeftXMinusCount++;
  }
  else
  {
	  TJoyLeftXMinusCount=0;
	  TJoyLeftXMinusStatusDebounce=0;
  }
  if(TJoyLeftXMinusCount==BUTTONTHRESHOLD)
  {
	  TJoyLeftXMinusStatusDebounce=1;
  }
  //LEFT JOYSTICK Y AXIS PLUS
  if(TJoyLeftYPlusStatus==1)
  {
	  TJoyLeftYPlusCount++;
  }
  else
  {
	  TJoyLeftYPlusCount=0;
	  TJoyLeftYPlusStatusDebounce=0;
  }
  if(TJoyLeftYPlusCount==BUTTONTHRESHOLD)
  {
	  TJoyLeftYPlusStatusDebounce=1;
  }
  //LEFT JOYSTICK Y AXIS MINUS
  if(TJoyLeftYMinusStatus==1)
  {
	  TJoyLeftYMinusCount++;
  }
  else
  {
	  TJoyLeftYMinusCount=0;
	  TJoyLeftYMinusStatusDebounce=0;
  }
  if(TJoyLeftYMinusCount==BUTTONTHRESHOLD)
  {
	  TJoyLeftYMinusStatusDebounce=1;
  }
  //RIGHT JOYSTICK X AXIS PLUS-----------------------------
  if(TJoyRightXPlusStatus==1)
  {
  	  TJoyRightXPlusCount++;
  }
  else
  {
  	  TJoyRightXPlusCount=0;
  	  TJoyRightXPlusStatusDebounce=0;
  }
  if(TJoyRightXPlusCount==BUTTONTHRESHOLD)
  {
  	  TJoyRightXPlusStatusDebounce=1;
  }
  //Right JOYSTICK X AXIS MINUS
  if(TJoyRightXMinusStatus==1)
  {
  	  TJoyRightXMinusCount++;
  }
  else
  {
  	  TJoyRightXMinusCount=0;
  	  TJoyRightXMinusStatusDebounce=0;
  }
  if(TJoyRightXMinusCount==BUTTONTHRESHOLD)
  {
  	  TJoyRightXMinusStatusDebounce=1;
  }
  //Right JOYSTICK Y AXIS PLUS
  if(TJoyRightYPlusStatus==1)
  {
  	  TJoyRightYPlusCount++;
  }
  else
  {
  	  TJoyRightYPlusCount=0;
  	  TJoyRightYPlusStatusDebounce=0;
  }
  if(TJoyRightYPlusCount==BUTTONTHRESHOLD)
  {
  	  TJoyRightYPlusStatusDebounce=1;
  }
  //Right JOYSTICK Y AXIS MINUS
  if(TJoyRightYMinusStatus==1)
  {
  	  TJoyRightYMinusCount++;
  }
  else
  {
  	  TJoyRightYMinusCount=0;
  	  TJoyRightYMinusStatusDebounce=0;
  }
  if(TJoyRightYMinusCount==BUTTONTHRESHOLD)
  {
  	  TJoyRightYMinusStatusDebounce=1;
  }
  //T1------------------------------------------
  if(T1Status==1)
  {
	  T1Count++;
  }
  else
  {
	  T1Count=0;
	  T1StatusDebounce=0;
  }
  if(T1Count==BUTTONTHRESHOLD)
  {
	  T1StatusDebounce=1;
  }
  //T2--------------------------------
  if(T2Status==1)
  {
	  T2Count++;
  }
  else
  {
	  T2Count=0;
	  T2StatusDebounce=0;
  }
  if(T2Count==BUTTONTHRESHOLD)
  {
	  T2StatusDebounce=1;
  }
  //T3----------------------------------------
  if(T3Status==1)
  {
	  T3Count++;
  }
  else
  {
	  T3Count=0;
	  T3StatusDebounce=0;
  }
  if(T3Count==BUTTONTHRESHOLD)
  {
	  T3StatusDebounce=1;
  }
  //T4--------------------------------------
  if(T4Status==1)
  {
	  T4Count++;
  }
  else
  {
	  T4Count=0;
	  T4StatusDebounce=0;
  }
  if(T4Count==BUTTONTHRESHOLD)
  {
	  T4StatusDebounce=1;
  }
  //T up------------------------------------------
  if(TUpStatus==1)
  {
       TUpCount++;
  }
  else
  {
	  TUpCount=0;
	  TUpStatusDebounce=0;
  }
  if(TUpCount==BUTTONTHRESHOLD)
  {
	  TUpStatusDebounce=1;
  }
  // down--------------------------------
  if(TDownStatus==1)
  {
	  TDownCount++;
  }
  else
  {
	  TDownCount=0;
	  TDownStatusDebounce=0;
  }
  if(TDownCount==BUTTONTHRESHOLD)
  {
	  TDownStatusDebounce=1;
  }
  //T left----------------------------------------
  if(TLeftStatus==1)
  {
	  TLeftCount++;
  }
  else
  {
	  TLeftCount=0;
	  TLeftStatusDebounce=0;
  }
  if(TLeftCount==BUTTONTHRESHOLD)
  {
	  TLeftStatusDebounce=1;
  }
  //T right--------------------------------------
  if(TRightStatus==1)
  {
	  TRightCount++;
  }
  else
  {
	  TRightCount=0;
	  TRightStatusDebounce=0;
  }
  if(TRightCount==BUTTONTHRESHOLD)
  {
	  TRightStatusDebounce=1;
  }

  //Toggle 1-----------------------------------------
  if(TOGG1status==1)
  {
    TOGG1count++;
  }
  else
  {
    TOGG1count=0;
    TOGG1statusdebounce=0;
  }

  if(TOGG1count==BUTTONTHRESHOLD)
  {
	  TOGG1statusdebounce=1;
  }

  //Toggle 2
  if(TOGG2status==1)
  {
    TOGG2count++;
  }
  else
  {
    TOGG2count=0;
    TOGG2statusdebounce=0;
  }

  if(TOGG2count==BUTTONTHRESHOLD)
  {
	  TOGG2statusdebounce=1;
  }

  //Toggle 3
  if(TOGG3status==1)
  {
    TOGG3count++;
  }
  else
  {
    TOGG3count=0;
    TOGG3statusdebounce=0;
  }

  if(TOGG3count==BUTTONTHRESHOLD)
  {
	  TOGG3statusdebounce=1;
  }

  //Toggle 4
  if(TOGG4status==1)
  {
    TOGG4count++;
  }
  else
  {
    TOGG4count=0;
    TOGG4statusdebounce=0;
  }

  if(TOGG4count==BUTTONTHRESHOLD)
  {
	  TOGG4statusdebounce=1;
  }

  //Toggle 5
  if(TOGG5status==1)
  {
    TOGG5count++;
  }
  else
  {
    TOGG5count=0;
    TOGG5statusdebounce=0;
  }

  if(TOGG5count==BUTTONTHRESHOLD)
  {
	  TOGG5statusdebounce=1;
  }

  //Tpggle 6
  if(TOGG6status==1)
  {
    TOGG6count++;
  }
  else
  {
    TOGG6count=0;
    TOGG6statusdebounce=0;
  }

  if(TOGG6count==BUTTONTHRESHOLD)
  {
	  TOGG6statusdebounce=1;
  }


  //LEFT JOYSTICK X AXIS MINUS EVENT
  if(TJoyLeftXMinusStatusDebounceHIST!=TJoyLeftXMinusStatusDebounce && TJoyLeftXMinusStatusDebounce==1)
  {
	  FlashDataActive.LjoyXtrim-=TRIMJOYSTEP;
  }

  //LEFT JOYSTICK X AXIS PLUS EVENT
  if(TJoyLeftXPlusStatusDebounceHIST!=TJoyLeftXPlusStatusDebounce && TJoyLeftXPlusStatusDebounce==1)
  {
	  FlashDataActive.LjoyXtrim+=TRIMJOYSTEP;
  }

  //LEFT JOYSTICK Y AXIS MINUS EVENT
  if(TJoyLeftYMinusStatusDebounceHIST!=TJoyLeftYMinusStatusDebounce && TJoyLeftYMinusStatusDebounce==1)
  {
	  FlashDataActive.LjoyYtrim-=TRIMJOYSTEP;
  }

  //LEFT JOYSTICK Y AXIS PLUS EVENT
  if(TJoyLeftYPlusStatusDebounceHIST!=TJoyLeftYPlusStatusDebounce && TJoyLeftYPlusStatusDebounce==1)
  {
	  FlashDataActive.LjoyYtrim+=TRIMJOYSTEP;
  }
  //RIGHT JOYSTICK X AXIS MINUS EVENT
  if(TJoyRightXMinusStatusDebounceHIST!=TJoyRightXMinusStatusDebounce && TJoyRightXMinusStatusDebounce==1)
  {
	  FlashDataActive.RjoyXtrim-=TRIMJOYSTEP;
  }

  //RIGHT JOYSTICK X AXIS PLUS EVENT
  if(TJoyRightXPlusStatusDebounceHIST!=TJoyRightXPlusStatusDebounce && TJoyRightXPlusStatusDebounce==1)
  {
	  FlashDataActive.RjoyXtrim+=TRIMJOYSTEP;
  }

  //RIGHT JOYSTICK Y AXIS MINUS EVENT
  if(TJoyRightYMinusStatusDebounceHIST!=TJoyRightYMinusStatusDebounce && TJoyRightYMinusStatusDebounce==1)
  {
	  FlashDataActive.RjoyYtrim-=TRIMJOYSTEP;
  }

  //RIGHT JOYSTICK Y AXIS PLUS EVENT
  if(TJoyRightYPlusStatusDebounceHIST!=TJoyRightYPlusStatusDebounce && TJoyRightYPlusStatusDebounce==1)
  {
	  FlashDataActive.RjoyYtrim+=TRIMJOYSTEP;
  }


  //T1 EVENT
  if(T1StatusDebounceHIST!=T1StatusDebounce && T1StatusDebounce==1)
  {
	 LvLUp=1;
  }

  //T2 EVENT
  if(T2StatusDebounceHIST!=T2StatusDebounce && T2StatusDebounce==1)
  {
	 LvlDown=1;
  }

  //T3 EVENT
  if(T3StatusDebounceHIST!=T3StatusDebounce && T3StatusDebounce==1)
  {
	  ValuePlus=1;
  }

  //T4 EVENT
  if(T4StatusDebounceHIST!=T4StatusDebounce && T4StatusDebounce==1)
  {
	  ValueMinus=1;
  }

  //T UP EVENT
  if(TUpStatusDebounceHIST!=TUpStatusDebounce && TUpStatusDebounce==1)
  {
	  CursorUp=1;
  }

  //T DOWN EVENT
  if(TDownStatusDebounceHIST!=TDownStatusDebounce && TDownStatusDebounce==1)
  {
	 CursorDown=1;
  }

  //T LEFT EVENT
  if(TLeftStatusDebounceHIST!=TLeftStatusDebounce && TLeftStatusDebounce==1)
  {
	  MenuMinus=1;
  }
  //T RIGHT EVENT
  if(TRightStatusDebounceHIST!=TRightStatusDebounce && TRightStatusDebounce==1)
  {
	 MenuPlus=1;
  }


  //LEDS-----------------------------------------------------------------------------------------
  if(MainInitDoneFlag)
  {
	  //LED1 CALIB INDIC
	  if(GyroCalibInProgress)LED1ON;
	  else LED1OFF;

	  //LED2 RC connection OK
	  if(ConnectWeakFlag)LED2OFF;
	  else LED2ON;

	  //LED 3 LOW BATT
	  if( (Batt1cellAVG < MINREMOTEBATT) ||  (DroneBattmV < MINDRONEBATT) ) LED3ON;
	  else LED3OFF;

	  //LED4 blinking
	  if(LED4blinkcount>0)LED4blinkcount--;
	  if(LED4blinkcount!=0)LED4ON;
	  else LED4OFF;
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

  //HW Specific wiring
 LjoyUPDOWNzeroOffset=4095-adcDataArray[1];
 LjoyLEFTRIGHTzeroOffset=adcDataArray[0];
 DjoyUPDOWNzeroOffset=adcDataArray[3];
 DjoyLEFTRIGHTzeroOffset=4095-adcDataArray[2];

 LjoyUPDOWNzeroOffset+=FlashDataActive.LjoyYtrim;
 LjoyLEFTRIGHTzeroOffset+=FlashDataActive.LjoyXtrim;
 DjoyUPDOWNzeroOffset+=FlashDataActive.RjoyYtrim;
 DjoyLEFTRIGHTzeroOffset+=FlashDataActive.RjoyXtrim;

 //zasicenja
 if(LjoyUPDOWNzeroOffset > (int32_t)(4095) )LjoyUPDOWNzeroOffset=4095;
 if(LjoyUPDOWNzeroOffset < (int32_t)(0) )LjoyUPDOWNzeroOffset=0;
 if(LjoyLEFTRIGHTzeroOffset > (int32_t)(4095) )LjoyLEFTRIGHTzeroOffset=4095;
 if(LjoyLEFTRIGHTzeroOffset < (int32_t)(0) )LjoyLEFTRIGHTzeroOffset=0;
 if(DjoyUPDOWNzeroOffset > (int32_t)(4095) )DjoyUPDOWNzeroOffset=4095;
 if(DjoyUPDOWNzeroOffset < (int32_t)(0) )DjoyUPDOWNzeroOffset=0;
 if(DjoyLEFTRIGHTzeroOffset > (int32_t)(4095) )DjoyLEFTRIGHTzeroOffset=4095;
 if(DjoyLEFTRIGHTzeroOffset < (int32_t)(0) )DjoyLEFTRIGHTzeroOffset=0;

 //scaling
 LjoyUPDOWN=ScaleJoysticks(LjoyYmin, LjoyYmax,LjoyUPDOWNzeroOffset);
 LjoyLEFTRIGHT=ScaleJoysticks(LjoyXmin, LjoyXmax, LjoyLEFTRIGHTzeroOffset);
 DjoyUPDOWN=ScaleJoysticks(RjoyYmin, RjoyYmax, DjoyUPDOWNzeroOffset);
 DjoyLEFTRIGHT=ScaleJoysticks(RjoyXmin, RjoyXmax, DjoyLEFTRIGHTzeroOffset);


 //Potenciometers (inverted logic-HW)---------------------------
 potenc1=100-adcDataArray[4]*100/4095;
 potenc2=100-adcDataArray[5]*100/4095;
 //------------------------------------------------------------------------------------------------------------

 //Save Buttons into 8bits
 Buttons=(TOGG1statusdebounce<<7) +  (TOGG2statusdebounce<<6) + (TOGG3statusdebounce<<5) + (TOGG4statusdebounce<<4) + (TOGG5statusdebounce<<3) + (TOGG6statusdebounce<<2) + (0<<1) + 0;

 //Buzzer
 if( Batt1cellAVG < MINREMOTEBATT) BuzzerRCBattpanic=1;
 else BuzzerRCBattpanic=0;

 if(DroneBattmV < MINDRONEBATT && ConnectWeakFlag==0)BuzzerDroneBattpanic=1;
 else BuzzerDroneBattpanic=0;

 if(BuzzerRCBattpanic)
 {
	 RCBattPanicCount++;
	 if(RCBattPanicCount>10000)HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	 if(RCBattPanicCount>11000){HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2); RCBattPanicCount=0;}
 }

 else if(BuzzerDroneBattpanic)
 {
	 DroneBattPanicCount++;
	 if(DroneBattPanicCount>2000)HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	 if(DroneBattPanicCount>2500){HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2); DroneBattPanicCount=0;}
 }
 else
 {
	 HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);//buzzer stop
	 RCBattPanicCount=0;
	 DroneBattPanicCount=0;
 }

 //Calculate Parameters for sending
 if(MSGCommTX !=0 )
 {
	 switch(MSGParameterTX)
	 {
	 	 case PARAM1: {MSGParamDataTX=DroneDataInput.pid_p_gain_pitch*PARAMMULTUPLITER;}break;
	 	 case PARAM2: {MSGParamDataTX=DroneDataInput.pid_i_gain_pitch*PARAMMULTUPLITER;}break;
	 	 case PARAM3: {MSGParamDataTX=DroneDataInput.pid_d_gain_pitch*PARAMMULTUPLITER;}break;
	 	 case PARAM4: {MSGParamDataTX=DroneDataInput.pid_p_gain_roll*PARAMMULTUPLITER;}break;
	 	 case PARAM5: {MSGParamDataTX=DroneDataInput.pid_i_gain_roll*PARAMMULTUPLITER;}break;
	 	 case PARAM6: {MSGParamDataTX=DroneDataInput.pid_d_gain_roll*PARAMMULTUPLITER;}break;
	 	 case PARAM7: {MSGParamDataTX=DroneDataInput.pid_p_gain_yaw*PARAMMULTUPLITER;}break;
	 	 case PARAM8: {MSGParamDataTX=DroneDataInput.pid_i_gain_yaw*PARAMMULTUPLITER;}break;
	 	 case PARAM9: {MSGParamDataTX=DroneDataInput.pid_d_gain_yaw*PARAMMULTUPLITER;}break;
	 	 case PARAM10: {MSGParamDataTX=DroneDataInput.pid_max_pitch*PARAMMULTUPLITER;}break;
	 	 case PARAM11: {MSGParamDataTX=DroneDataInput.pid_i_max_pitch*PARAMMULTUPLITER;}break;
	 	 case PARAM12: {MSGParamDataTX=DroneDataInput.pid_max_roll*PARAMMULTUPLITER;}break;
	 	 case PARAM13: {MSGParamDataTX=DroneDataInput.pid_i_max_roll*PARAMMULTUPLITER;}break;
	 	 case PARAM14: {MSGParamDataTX=DroneDataInput.pid_max_yaw*PARAMMULTUPLITER;}break;
	 	 case PARAM15: {MSGParamDataTX=DroneDataInput.pid_i_max_yaw*PARAMMULTUPLITER;}break;
	 	 case PARAM16: {MSGParamDataTX=DroneDataInput.maxpitchdegree*PARAMMULTUPLITER;}break;
	 	 case PARAM17: {MSGParamDataTX=DroneDataInput.maxrolldegree*PARAMMULTUPLITER;}break;
	 	 case PARAM18: {MSGParamDataTX=DroneDataInput.maxyawdegree*PARAMMULTUPLITER;}break;
	 	 case PARAM19: {MSGParamDataTX=DroneDataInput.minthrottle*PARAMMULTUPLITER;}break;
	 	 case PARAM20: {MSGParamDataTX=DroneDataInput.maxthrottle*PARAMMULTUPLITER;}break;
	 }
 }

 //NRF24---------------------------------------------------------------------------------------
 if(MainInitDoneFlag && CommShtdownnrf24!=1)
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
					nRF24_payloadTX[0] = MSGCommTX;

					if(MSGCommTX==COMMCONTROLDATA)
					{
						nRF24_payloadTX[1] = (uint8_t)(LjoyUPDOWN);
						nRF24_payloadTX[2] = (uint8_t)(LjoyLEFTRIGHT);
						nRF24_payloadTX[3] = (uint8_t)(DjoyUPDOWN);
						nRF24_payloadTX[4] = (uint8_t)(DjoyLEFTRIGHT);
						nRF24_payloadTX[5] = (uint8_t)(potenc1);
						nRF24_payloadTX[6] = (uint8_t)(potenc2);
						nRF24_payloadTX[7] = (uint8_t)(Buttons);
					}
					else if(MSGCommTX==COMMPARAMACTIVE)
					{
						nRF24_payloadTX[1] = MSGParameterTX;
						nRF24_payloadTX[2] = (MSGParamDataTX & 0xFF000000)>>24;
						nRF24_payloadTX[3] = (MSGParamDataTX & 0x00FF0000)>>16;
						nRF24_payloadTX[4] = (MSGParamDataTX & 0x0000FF00)>>8;
						nRF24_payloadTX[5] = (MSGParamDataTX & 0x000000FF);
						nRF24_payloadTX[6] = 0;
						nRF24_payloadTX[7] = 0;
					}
					else if(MSGCommTX==COMMPARAMFLASH)
					{
						nRF24_payloadTX[1] = MSGParameterTX;
						nRF24_payloadTX[2] = 0;
						nRF24_payloadTX[3] = 0;
						nRF24_payloadTX[4] = 0;
						nRF24_payloadTX[5] = 0;
						nRF24_payloadTX[6] = 0;
						nRF24_payloadTX[7] = 0;
					}
					else
					{
						nRF24_payloadTX[1] = 0;
						nRF24_payloadTX[2] = 0;
						nRF24_payloadTX[3] = 0;
						nRF24_payloadTX[4] = 0;
						nRF24_payloadTX[5] = 0;
						nRF24_payloadTX[6] = 0;
						nRF24_payloadTX[7] = 0;
					}

					// Transmit a packet
					nRF24_TransmitPacket(nRF24_payloadTX, 8);
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

			MSGCommRX=nRF24_payloadRX[0];

			if(MSGCommTX!=COMMCONTROLDATA && MSGCommRX!=COMMCONTROLDATA)MSGCommTX=COMMCONTROLDATA;//reset, return message wasn't control data

			if(MSGCommRX==COMMCONTROLDATA) //Drone status
			{
				DroneBattLowerByte=nRF24_payloadRX[1];
				DroneBattUpperByte=nRF24_payloadRX[2];
				DroneBattmV=(DroneBattUpperByte<<8)+DroneBattLowerByte;

				if(nRF24_payloadRX[5] & 0x1)DronePitchAngle=(-1)*nRF24_payloadRX[3];
				else DronePitchAngle=nRF24_payloadRX[3];

				if((nRF24_payloadRX[5]>>1)& 0x1)DroneRollAngle=(-1)*nRF24_payloadRX[4];
				else DroneRollAngle=nRF24_payloadRX[4];

				GyroCalibInProgress=(nRF24_payloadRX[5]>>2) & 0x1;

				motorSTAT=(nRF24_payloadRX[5]>>3) & 0x7; //last 3 bits
			}
			else if(MSGCommRX==COMMPARAMACTIVE || MSGCommRX==COMMPARAMFLASH)
			{
				MSGParameterRX=nRF24_payloadRX[1]; //Get parameter

				LED4blinkcount=LEDSHORTBLINK;

				MSGParamDataRX=(nRF24_payloadRX[2]<<24) + (nRF24_payloadRX[3]<<16) + (nRF24_payloadRX[4]<<8) + (nRF24_payloadRX[5]); //get data

				switch (MSGParameterRX)
				{
					case  PARAM1 :{DroneDataTemp.pid_p_gain_pitch=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM2 :{DroneDataTemp.pid_i_gain_pitch=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM3 :{DroneDataTemp.pid_d_gain_pitch=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM4 :{DroneDataTemp.pid_p_gain_roll=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM5 :{DroneDataTemp.pid_i_gain_roll=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM6 :{DroneDataTemp.pid_d_gain_roll=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM7 :{DroneDataTemp.pid_p_gain_yaw=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM8 :{DroneDataTemp.pid_i_gain_yaw=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM9 :{DroneDataTemp.pid_d_gain_yaw=(float)(MSGParamDataRX)/(float)(PARAMMULTUPLITER);}break;
					case  PARAM10 :{DroneDataTemp.pid_max_pitch=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM11 :{DroneDataTemp.pid_i_max_pitch=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM12 :{DroneDataTemp.pid_max_roll=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM13 :{DroneDataTemp.pid_i_max_roll=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM14 :{DroneDataTemp.pid_max_yaw=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM15 :{DroneDataTemp.pid_i_max_yaw=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM16 :{DroneDataTemp.maxpitchdegree=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM17 :{DroneDataTemp.maxrolldegree=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM18 :{DroneDataTemp.maxyawdegree=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM19 :{DroneDataTemp.minthrottle=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
					case  PARAM20 :{DroneDataTemp.maxthrottle=(int)(MSGParamDataRX/PARAMMULTUPLITER);}break;
				}

				if(MSGCommRX==COMMPARAMACTIVE)DroneDataActive=DroneDataTemp; //Parameters are active ones
				else
				{
					DroneDataFlash=DroneDataTemp; //Save Parameters from Flash
					DroneDataInput=DroneDataTemp; //Set Parameters from flash to preset for setting new parameters
				}
			}
			else if(MSGCommRX==COMMERASEFLASHDR || MSGCommRX==COMMWRITEFLASHDR)LED4blinkcount=LEDLONGBLINK;

			TotalMSGrecv++;
			MSGcount++;
		 }
	 }

	 TXdelay++;
	 if(TXdelay==TXPERIOD)TXdelay=0;
 }//----------------------------------------------------------------------------------------------

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

uint32_t ScaleJoysticks(uint32_t min, uint32_t max, uint32_t value)
{
	uint32_t temp;

	if(value<min)value=min;

	temp=( (value-min)*100)/(max-min);

	if(temp>100)temp=100;

	return temp;

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
