/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nokia5110_LCD.h"
#include "nrf24.h"
#include "mcp23017.h"
#include "LCDprints.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t test1;
uint32_t test2;
uint32_t test3;
uint32_t test4;
uint32_t wifiOK;

uint32_t watch1;
uint32_t watch2;
uint32_t watch3;
uint32_t watch4;
uint32_t watch5;
uint32_t watch6;


char stringlcdBuffer[20];
uint16_t adcDataArray[7];

//MCP23017
MCP23017str	MCP23017DataStr;

//NRF24
uint8_t nRF24_payloadTX[32]; //TX buffer
uint8_t nRF24_payloadRX[32]; //RX buffer
const uint8_t nRF24_ADDR[3] = {5, 3, 5 }; //Address
uint8_t RXstpaketov=0;

uint32_t MainInitDoneFlag=0;
struct FlashDatastruct FlashDataDefault; //Default constant embedded in Code
struct FlashDatastruct FlashDataFlash;  //Constants read from Flash
struct FlashDatastruct FlashDataActive; //Active constants
struct DroneDataStruct DroneDataRX;
struct DroneDataStruct DroneDataTX;
float DroneTuneStep=1.0;
uint32_t DroneTuneStepInt=1;

//LCD MENUS
MenuLvL CurrentLvl;
LCDScreen CurrentScreen;
CursorPositions CurrentCursorPos;

//Buttons to move through screens, move cursor and change menu lvl
uint32_t MenuPlus,MenuMinus;
uint32_t CursorUp,CursorDown;
uint32_t ValuePlus,ValueMinus;
uint32_t LvLUp,LvlDown;

extern uint32_t CommShtdownnrf24;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//test timings DWT counter
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,adcDataArray, 7);

  //DEFAULT FLASH CONSTANTS
  FlashDataDefault.controlData=CONTROLWORD;
  FlashDataDefault.LCD_contrast=0xB8;
  FlashDataDefault.LjoyXtrim=0;
  FlashDataDefault.LjoyYtrim=0;
  FlashDataDefault.RjoyXtrim=0;
  FlashDataDefault.RjoyYtrim=0;

  if( CheckFlashData(FLASHCONSTADDR) == CONTROLWORD ) //Check if any Data is present
  {
	  //Read Data and Save parameters into ACTIVE structure
	  ReadFlashData(FLASHCONSTADDR, &FlashDataActive);

  }
  else
  {
	  //Write default values into Flash, Read back data into Active Structure
	  WriteFlashData(FLASHCONSTADDR, &FlashDataDefault);
	  ReadFlashData(FLASHCONSTADDR, &FlashDataActive);
  }

  //Lightshow
  LED1ON;
  LED2ON;
  LED3ON;
  LED4ON;
  HAL_Delay(1000);
  LED1OFF;
  LED2OFF;
  LED3OFF;
  LED4OFF;

  //default LCD
  CurrentLvl=LVL0;
  CurrentScreen=MainScreen;
  CurrentCursorPos=Line0;

  //lcd init pins
  LCD_setRST(LCD_RST_GPIO_Port, LCD_RST_Pin);
  LCD_setDC(LCD_COMM_GPIO_Port, LCD_COMM_Pin);
  LCD_setCE(LCD_CE_GPIO_Port, LCD_CE_Pin);
  LCD_setDIN(LCD_DATA_GPIO_Port, LCD_DATA_Pin);
  LCD_setCLK(LCD_CLK_GPIO_Port, LCD_CLK_Pin);

  LCD_init(&FlashDataActive); //6 lines 15 characters max per line (0,0), (0,1), (0,2), (0,3), (0,4), (0,5) line starts


  //NRF24INIT
  SPI1->CR1|=SPI_CR1_SPE; //enable SPI


  nRF24_CE_L(); // RX/TX disabled

  wifiOK=nRF24_Check();

  nRF24_Init(); //Default init

  // Disable ShockBurst for all RX pipes
  nRF24_DisableAA(0xFF);

  // Set RF channel
  nRF24_SetRFChannel(15); //2400Mhz + 15Mhz

  // Set data rate
  nRF24_SetDataRate(nRF24_DR_250kbps);

  // Set CRC scheme
  nRF24_SetCRCScheme(nRF24_CRC_1byte);

  // Set address width, its common for all pipes (RX and TX)
  nRF24_SetAddrWidth(3);

  nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 6); // Auto-ACK: disabled, payload length: 6 bytes

  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address

  // Set TX power
  nRF24_SetTXPower(nRF24_TXPWR_6dBm);

  // Set operational mode (PTX == transmitter)
  nRF24_SetOperationalMode(nRF24_MODE_TX);

  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();

  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);

  //MCP23017 intit
  mcp23017_init(&hi2c2,MCP23017_ADDRESS_20);


  //TESTING
  //writeFlashData(FLASHCONSTADDR);

  MainInitDoneFlag=1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(MenuPlus)//Menu PLUS------------------------------------------------------------------------------------------------------------------------
	  {
	  	  LCD_clrScr();

	  	  switch(CurrentScreen)
	  	  {
	  	  	  case MainScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case MenuScreen1:{CurrentScreen=TestScreen1;}break;
	  	  	  case TestScreen1:{CurrentScreen=MainScreen;}break;

	  	  	  case ButtonScreen1:{CurrentScreen=ButtonScreen2;}break;
	  	  	  case ButtonScreen2:{CurrentScreen=ButtonScreen3;}break;
	  	  	  case ButtonScreen3:{CurrentScreen=ButtonScreen1;}break;

	  	  	  case Param1TuneScreen:{CurrentScreen=Param2TuneScreen;}break;
	  	  	  case Param2TuneScreen:{CurrentScreen=Param3TuneScreen;}break;
	  	  	  case Param3TuneScreen:{CurrentScreen=Param4TuneScreen;}break;
	  	  	  case Param4TuneScreen:{CurrentScreen=Param5TuneScreen;}break;
	  	  	  case Param5TuneScreen:{CurrentScreen=Param6TuneScreen;}break;
	  	  	  case Param6TuneScreen:{CurrentScreen=Param7TuneScreen;}break;
	  	  	  case Param7TuneScreen:{CurrentScreen=Param8TuneScreen;}break;
	  	  	  case Param8TuneScreen:{CurrentScreen=Param9TuneScreen;}break;
	  	  	  case Param9TuneScreen:{CurrentScreen=Param10TuneScreen;}break;
	  	  	  case Param10TuneScreen:{CurrentScreen=Param11TuneScreen;}break;
	  	  	  case Param11TuneScreen:{CurrentScreen=Param12TuneScreen;}break;
	  	  	  case Param12TuneScreen:{CurrentScreen=Param13TuneScreen;}break;
		  	  case Param13TuneScreen:{CurrentScreen=Param14TuneScreen;}break;
		  	  case Param14TuneScreen:{CurrentScreen=Param15TuneScreen;}break;
			  case Param15TuneScreen:{CurrentScreen=Param16TuneScreen;}break;
			  case Param16TuneScreen:{CurrentScreen=Param17TuneScreen;}break;
			  case Param17TuneScreen:{CurrentScreen=Param18TuneScreen;}break;
			  case Param18TuneScreen:{CurrentScreen=Param19TuneScreen;}break;
			  case Param19TuneScreen:{CurrentScreen=Param20TuneScreen;}break;
			  case Param20TuneScreen:{CurrentScreen=Param1TuneScreen;}break;
	  	  }

		  MenuPlus=0;
		  CurrentCursorPos = Line0;//reset cursor
	  }

	  if(MenuMinus)//MENU MINUS----------------------------------------------------------------------------------------------------------------------
	  {
	  	  LCD_clrScr();

	  	  switch(CurrentScreen)
	  	  {
	  	  	  case MainScreen:{CurrentScreen=TestScreen1;}break;
	  	  	  case MenuScreen1:{CurrentScreen=MainScreen;}break;
	  	  	  case TestScreen1:{CurrentScreen=MenuScreen1;}break;

	  	  	  case ButtonScreen1:{CurrentScreen=ButtonScreen3;}break;
	  	  	  case ButtonScreen2:{CurrentScreen=ButtonScreen1;}break;
	  	  	  case ButtonScreen3:{CurrentScreen=ButtonScreen2;}break;

	  	  	  case Param1TuneScreen:{CurrentScreen=Param20TuneScreen;}break;
	  	  	  case Param2TuneScreen:{CurrentScreen=Param1TuneScreen;}break;
	  	  	  case Param3TuneScreen:{CurrentScreen=Param2TuneScreen;}break;
	  	  	  case Param4TuneScreen:{CurrentScreen=Param3TuneScreen;}break;
	  	  	  case Param5TuneScreen:{CurrentScreen=Param4TuneScreen;}break;
	  	  	  case Param6TuneScreen:{CurrentScreen=Param5TuneScreen;}break;
	  	  	  case Param7TuneScreen:{CurrentScreen=Param6TuneScreen;}break;
	  	  	  case Param8TuneScreen:{CurrentScreen=Param7TuneScreen;}break;
	  	  	  case Param9TuneScreen:{CurrentScreen=Param8TuneScreen;}break;
	  	  	  case Param10TuneScreen:{CurrentScreen=Param9TuneScreen;}break;
	  	  	  case Param11TuneScreen:{CurrentScreen=Param10TuneScreen;}break;
	  	  	  case Param12TuneScreen:{CurrentScreen=Param11TuneScreen;}break;
		  	  case Param13TuneScreen:{CurrentScreen=Param12TuneScreen;}break;
		  	  case Param14TuneScreen:{CurrentScreen=Param13TuneScreen;}break;
			  case Param15TuneScreen:{CurrentScreen=Param14TuneScreen;}break;
			  case Param16TuneScreen:{CurrentScreen=Param15TuneScreen;}break;
			  case Param17TuneScreen:{CurrentScreen=Param16TuneScreen;}break;
			  case Param18TuneScreen:{CurrentScreen=Param17TuneScreen;}break;
			  case Param19TuneScreen:{CurrentScreen=Param18TuneScreen;}break;
			  case Param20TuneScreen:{CurrentScreen=Param19TuneScreen;}break;
	  	  }

		  MenuMinus=0;
		  CurrentCursorPos = Line0;//reset cursor
	  }

	  if(LvLUp)//LVL UP------------------------------------------------------------------------------------------------------------------------------
	  {
		  LCD_clrScr();

          switch(CurrentScreen)
          {
              case MenuScreen1:
              {
            	  switch(CurrentCursorPos)
            	  {
                  	  case Line0:{CurrentScreen=MSGScreen1;}break;
                  	  case Line1:{CurrentScreen=ButtonScreen1;}break;

                  	  case Line2:{
                  		  	  	  	  CurrentScreen=FlashDataScreenRd;
                  		  	  	  	  ReadFlashData(FLASHCONSTADDR, &FlashDataFlash);
                  	  	  	  	  }break;

                  	  case Line3:{
                  		  	  	  	  CurrentScreen=FlashDataScreenWr;

                  	  	  	  	 }break;

                  	  case Line4:{CurrentScreen=CommandScreen1;}break;

                  	  case Line5:{CurrentScreen=Param1TuneScreen;}break;
            	  }
              }break;

              case CommandScreen1:
              {
            	  switch(CurrentCursorPos)
            	  {
                  	  case Line0:{CommShtdownnrf24=!CommShtdownnrf24;}break;
                  	  case Line1:{WriteFlashData(FLASHCONSTADDR, &FlashDataActive);}break;
                  	  case Line2:{EraseFlashData(FLASHCONSTADDR);}break;
            	  }
              }break;

          }
		  LvLUp=0;
		  CurrentCursorPos = Line0;//reset cursor
	  }

	  if(LvlDown)//LVL DOWN---------------------------------------------------------------------------------------------------------------------------
	  {
		  LCD_clrScr();

		  switch(CurrentScreen)
		  {
		      case MenuScreen1:{CurrentScreen=MainScreen;}break;
		      case TestScreen1:{CurrentScreen=MainScreen;}break;

		      case MSGScreen1:{CurrentScreen=MenuScreen1;}break;
		      case ButtonScreen1:{CurrentScreen=MenuScreen1;}break;
		      case ButtonScreen2:{CurrentScreen=MenuScreen1;}break;
		      case ButtonScreen3:{CurrentScreen=MenuScreen1;}break;
		      case FlashDataScreenRd:{CurrentScreen=MenuScreen1;}break;
		      case FlashDataScreenWr:{CurrentScreen=MenuScreen1;}break;
		      case CommandScreen1:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param1TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param2TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param3TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param4TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param5TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param6TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param7TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param8TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param9TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param10TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param11TuneScreen:{CurrentScreen=MenuScreen1;}break;
	  	  	  case Param12TuneScreen:{CurrentScreen=MenuScreen1;}break;
		  	  case Param13TuneScreen:{CurrentScreen=MenuScreen1;}break;
		  	  case Param14TuneScreen:{CurrentScreen=MenuScreen1;}break;
			  case Param15TuneScreen:{CurrentScreen=MenuScreen1;}break;
			  case Param16TuneScreen:{CurrentScreen=MenuScreen1;}break;
			  case Param17TuneScreen:{CurrentScreen=MenuScreen1;}break;
			  case Param18TuneScreen:{CurrentScreen=MenuScreen1;}break;
			  case Param19TuneScreen:{CurrentScreen=MenuScreen1;}break;
			  case Param20TuneScreen:{CurrentScreen=MenuScreen1;}break;
		  }
		  LvlDown=0;
		  CurrentCursorPos = Line0;//reset cursor
	  }

	  if(CursorUp)//Cursor Up-------------------------------------------------------------------------------------------------------------------------
	  {
		  if(CurrentCursorPos == Line0)CurrentCursorPos=Line5;
		  else CurrentCursorPos--;

		  CursorUp=0;
	  }

	  if(CursorDown)//Cursor Down---------------------------------------------------------------------------------------------------------------------
	  {
		  if(CurrentCursorPos == Line5)CurrentCursorPos=Line0;
		  else CurrentCursorPos++;

		  CursorDown=0;
	  }

	  if(ValuePlus)//Value Plus---------------------------------------------------------------------------------------------------------------------
	  {

		  switch(CurrentScreen)
		  {
		  	  case FlashDataScreenWr:
		  	  {
		  		switch(CurrentCursorPos)
		  		{
		  			case Line0:{FlashDataActive.controlData++;};break;
		  			case Line1:{FlashDataActive.LCD_contrast+=1;};break;
		  			case Line2:{};break;
		  			case Line3:{};break;
		  			case Line4:{};break;
		  			case Line5:{};break;
		  		}
		  	  }break;

			  case Param1TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_p_gain_pitch+=DroneTuneStep; else DroneTuneStep*=10; }break;
			  case Param2TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_gain_pitch+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param3TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_d_gain_pitch+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param4TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_p_gain_roll+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param5TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_gain_roll+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param6TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_d_gain_roll+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param7TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_p_gain_yaw+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param8TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_gain_yaw+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param9TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_d_gain_yaw+=DroneTuneStep; else DroneTuneStep*=10;}break;
			  case Param10TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_max_pitch+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param11TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_max_pitch+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param12TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_max_roll+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param13TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_max_roll+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param14TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_max_yaw+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param15TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_max_yaw+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param16TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxpitchdegree+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param17TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxrolldegree+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param18TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxyawdegree+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param19TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.minthrottle+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
			  case Param20TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxthrottle+=DroneTuneStepInt; else DroneTuneStepInt*=10;}break;
		  }

		  if(DroneTuneStep>DRONETUNESTEPMAX)DroneTuneStep=DRONETUNESTEPMAX;
		  if(DroneTuneStep<DRONETUNESTEPMIN)DroneTuneStep=DRONETUNESTEPMIN;

		  if(DroneTuneStepInt>DRONETUNESTEPINTMAX)DroneTuneStepInt=DRONETUNESTEPINTMAX;
		  if(DroneTuneStepInt<DRONETUNESTEPINTMIN)DroneTuneStepInt=DRONETUNESTEPINTMIN;

		  ValuePlus=0;
	  }

	  if(ValueMinus)//Value Minus---------------------------------------------------------------------------------------------------------------------
	  {

		  switch(CurrentScreen)
		  {
		  	  case FlashDataScreenWr:
		  	  {
		  		switch(CurrentCursorPos)
		  		{
		  			case Line0:{FlashDataActive.controlData--;};break;
		  			case Line1:{FlashDataActive.LCD_contrast-=1;};break;
		  			case Line2:{};break;
		  			case Line3:{};break;
		  			case Line4:{};break;
		  			case Line5:{};break;
		  		}
		  	  }break;

			  case Param1TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_p_gain_pitch-=DroneTuneStep; else DroneTuneStep/=10; }break;
			  case Param2TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_gain_pitch-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param3TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_d_gain_pitch-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param4TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_p_gain_roll-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param5TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_gain_roll-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param6TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_d_gain_roll-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param7TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_p_gain_yaw-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param8TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_gain_yaw-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param9TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_d_gain_yaw-=DroneTuneStep; else DroneTuneStep/=10;}break;
			  case Param10TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_max_pitch-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param11TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_max_pitch-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param12TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_max_roll-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param13TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_max_roll-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param14TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_max_yaw-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param15TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.pid_i_max_yaw-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param16TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxpitchdegree-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param17TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxrolldegree-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param18TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxyawdegree-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param19TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.minthrottle-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
			  case Param20TuneScreen:{if(CurrentCursorPos!=Line1)DroneDataTX.maxthrottle-=DroneTuneStepInt; else DroneTuneStepInt/=10;}break;
		  }

		  if(DroneTuneStep>DRONETUNESTEPMAX)DroneTuneStep=DRONETUNESTEPMAX;
		  if(DroneTuneStep<DRONETUNESTEPMIN)DroneTuneStep=DRONETUNESTEPMIN;

		  if(DroneTuneStepInt>DRONETUNESTEPINTMAX)DroneTuneStepInt=DRONETUNESTEPINTMAX;
		  if(DroneTuneStepInt<DRONETUNESTEPINTMIN)DroneTuneStepInt=DRONETUNESTEPINTMIN;

		  ValueMinus=0;
	  }



	  switch(CurrentScreen)//Display selected screen----------------------------------------------------------------------------------------------------
	  {
	  		case MainScreen:{MainScreenPrint(stringlcdBuffer);}break;
	  		case MenuScreen1:{MenuScreen1Print(stringlcdBuffer,CurrentCursorPos);}break;
	  		case MSGScreen1:{MSGScreen1Print(stringlcdBuffer);}break;
	  		case ButtonScreen1:{ButtonScreen1Print(stringlcdBuffer);}break;
	  		case ButtonScreen2:{ButtonScreen2Print(stringlcdBuffer);}break;
	  		case ButtonScreen3:{ButtonScreen3Print(stringlcdBuffer);}break;
	  		case TestScreen1:{TestScreen1Print(stringlcdBuffer);}break;
	  		case FlashDataScreenRd:{FlashDataScreenRdPrint(stringlcdBuffer);}break;
	  		case FlashDataScreenWr:{FlashDataScreenWrPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  		case CommandScreen1:{CommandScreen1Print(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param1TuneScreen:{Param1TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param2TuneScreen:{Param2TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param3TuneScreen:{Param3TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param4TuneScreen:{Param4TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param5TuneScreen:{Param5TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param6TuneScreen:{Param6TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param7TuneScreen:{Param7TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param8TuneScreen:{Param8TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param9TuneScreen:{Param9TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param10TuneScreen:{Param10TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param11TuneScreen:{Param11TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  	  	case Param12TuneScreen:{Param12TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
		  	case Param13TuneScreen:{Param13TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
		  	case Param14TuneScreen:{Param14TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
			case Param15TuneScreen:{Param15TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
			case Param16TuneScreen:{Param16TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
			case Param17TuneScreen:{Param17TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
			case Param18TuneScreen:{Param18TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
			case Param19TuneScreen:{Param19TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
			case Param20TuneScreen:{Param20TuneScreenPrint(stringlcdBuffer,CurrentCursorPos);}break;
	  }

	  //test1=DWT->CYCCNT;
	  //test2=DWT->CYCCNT-test1;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RST_Pin|LCD_CLK_Pin|LCD_CE_Pin|LCD_DATA_Pin 
                          |LED3_Pin|SPI_NR24_CSN_Pin|LED4_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_COMM_Pin|SPI_NR24_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_NR24_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI_NR24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_NR24_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_CE_Pin LED3_Pin SPI_NR24_CSN_Pin 
                           LED4_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CE_Pin|LED3_Pin|SPI_NR24_CSN_Pin 
                          |LED4_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CLK_Pin LCD_DATA_Pin */
  GPIO_InitStruct.Pin = LCD_CLK_Pin|LCD_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_COMM_Pin */
  GPIO_InitStruct.Pin = LCD_COMM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_COMM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOGG1_Pin TOGG2_Pin TOGG3_Pin TOGG4_Pin */
  GPIO_InitStruct.Pin = TOGG1_Pin|TOGG2_Pin|TOGG3_Pin|TOGG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_NR24_CE_Pin */
  GPIO_InitStruct.Pin = SPI_NR24_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_NR24_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOGG5_Pin TOGG6_Pin */
  GPIO_InitStruct.Pin = TOGG5_Pin|TOGG6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Write Data into Flash starting from given address
void WriteFlashData(uint32_t StartAddr, struct FlashDatastruct *p)
{
	FLASH_EraseInitTypeDef EraseInitStruct;

	uint32_t PageError;

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = StartAddr;
	EraseInitStruct.NbPages     = 1;

	HAL_FLASH_Unlock();

	//FLASH_PageErase(0x800FC00); //doesn't handle all registers PER regiser in CR is not cleared

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr, p->controlData);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+4, p->LCD_contrast);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+8, p->LjoyXtrim);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+12, p->LjoyYtrim);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+16, p->RjoyXtrim);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+20, p->RjoyYtrim);

	HAL_FLASH_Lock();
}

void EraseFlashData(uint32_t StartAddr)
{
	FLASH_EraseInitTypeDef EraseInitStruct;

	uint32_t PageError;

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = StartAddr;
	EraseInitStruct.NbPages     = 1;

	HAL_FLASH_Unlock();

	//FLASH_PageErase(0x800FC00); //doesn't handle all registers PER regiser in CR is not cleared

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	HAL_FLASH_Lock();
}


//Check if Data on given address matches control word
uint32_t CheckFlashData(uint32_t StartAddr)
{
	return *(( uint32_t *) (StartAddr) );
}

//Read Data from Flash
void ReadFlashData(uint32_t StartAddr, struct FlashDatastruct *p)
{
	p->controlData= *(( uint32_t *) (StartAddr) );
	p->LCD_contrast=*(( uint32_t *) (StartAddr+4) );
	p->LjoyXtrim=*(( uint32_t *) (StartAddr+8) );
	p->LjoyYtrim=*(( uint32_t *) (StartAddr+12) );
	p->RjoyXtrim=*(( uint32_t *) (StartAddr+16) );
	p->RjoyYtrim=*(( uint32_t *) (StartAddr+20) );
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
