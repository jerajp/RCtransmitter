// Functions to manage LCD printing


#include "LCDprints.h"
#include "nokia5110_LCD.h"
#include "main.h"

//external variables
extern uint32_t DroneBattmV;
extern int32_t DronePitchAngle;
extern int32_t DroneRollAngle;
extern uint32_t Batt1cellAVG;
extern uint32_t motorSTAT;
extern uint32_t TotalMSGsend;
extern uint32_t TotalMSGrecv;
extern uint32_t MSGprerSecond;
extern uint32_t MSGLowCount;
extern uint32_t TOGG1statusdebounce,TOGG2statusdebounce,TOGG3statusdebounce,TOGG4statusdebounce,TOGG5statusdebounce,TOGG6statusdebounce;
extern uint32_t potenc1,potenc2;
extern uint32_t LjoyUPDOWN,LjoyLEFTRIGHT,DjoyUPDOWN,DjoyLEFTRIGHT;
extern int32_t LjoyUPDOWNzeroOffset,LjoyLEFTRIGHTzeroOffset,DjoyUPDOWNzeroOffset,DjoyLEFTRIGHTzeroOffset;
extern uint32_t T1StatusDebounce,T2StatusDebounce,T3StatusDebounce,T4StatusDebounce;
extern uint32_t TUpStatusDebounce,TDownStatusDebounce,TLeftStatusDebounce,TRightStatusDebounce;
extern uint32_t TJoyLeftXPlusStatusDebounce,TJoyLeftXMinusStatusDebounce,TJoyLeftYPlusStatusDebounce,TJoyLeftYMinusStatusDebounce;
extern uint32_t TJoyRightXPlusStatusDebounce,TJoyRightXMinusStatusDebounce,TJoyRightYPlusStatusDebounce,TJoyRightYMinusStatusDebounce;

extern struct FlashDatastruct FlashDataDefault;
extern struct FlashDatastruct FlashDataFlash;
extern struct FlashDatastruct FlashDataActive;

extern uint32_t CommShtdownnrf24;

extern uint32_t watch1,watch2,watch3,watch4,watch5,watch6;

void MainScreenPrint(char *buffer)
{
	  	  //Main menu
	  	  switch(motorSTAT)
	  	  {
	  	  	  case 1:{
	  		  	  	  	  sprintf(buffer,"MOT OFF     ");
	  	  	  	  	  }break;

	  	  	  case 2:{
	  		  	  	  sprintf(buffer,"MOT STARTING");
	  	  	  	  	  }break;

	  	  	  case 3:{
	  		  	  	  	  sprintf(buffer,"MOT RUNNING ");
	  	  	  	 	 }break;

	  	  	  default:{
	  		  	  	  	  sprintf(buffer,"MOT INIT    ");
	  	  	  	  	  }
	  	  }
	  	  LCD_print(buffer,0,0);

	  	  sprintf(buffer,"RC: %u mV",Batt1cellAVG);
	  	  LCD_print(buffer,0,1);

	  	  sprintf(buffer,"DR: %u mV",DroneBattmV);
	  	  LCD_print(buffer,0,2);

	  	  sprintf(buffer,"Pitch: %d ",DronePitchAngle);
	  	  LCD_print(buffer,0,3);

	  	  sprintf(buffer,"Roll: %d ",DroneRollAngle);
	  	  LCD_print(buffer,0,4);

}

void MenuScreen1Print(char *buffer, CursorPositions pos)
{
	  if(pos==Line0) sprintf(buffer,"*MSG status");
	  else sprintf(buffer," MSG status");
	  LCD_print(buffer,1,0);

	  if(pos==Line1) sprintf(buffer,"*RC status");
	  else sprintf(buffer," RC status");
	  LCD_print(buffer,1,1);

	  if(pos==Line2) sprintf(buffer,"*FlashDataRd");
	  else sprintf(buffer," FlashDataRd");
	  LCD_print(buffer,1,2);

	  if(pos==Line3)  sprintf(buffer,"*FlashDataWr");
	  else sprintf(buffer," FlashDataWr");
	  LCD_print(buffer,1,3);

	  if(pos==Line4)  sprintf(buffer,"*Commands");
	  else sprintf(buffer," Commands");
	  LCD_print(buffer,1,4);

	  if(pos==Line5)  sprintf(buffer,"*");
	  else sprintf(buffer," ");
	  LCD_print(buffer,1,5);
}

void MSGScreen1Print(char *buffer)
{
  	  //Diagnostics Connection
  	  sprintf(buffer,"Total MSG");
	  LCD_print(buffer,0,0);

	  sprintf(buffer,"Send %u",TotalMSGsend);
	  LCD_print(buffer,0,1);

	  sprintf(buffer,"Recv %u",TotalMSGrecv);
	  LCD_print(buffer,0,2);

	  sprintf(buffer,"MSG/s: %u",MSGprerSecond);
	  LCD_print(buffer,0,3);

  	  sprintf(buffer,"RClow[s]: %u",MSGLowCount);
  	  LCD_print(buffer,0,4);
}

void ButtonScreen1Print(char *buffer)
{
	  sprintf(buffer,"Toggle:%u%u%u%u%u%u",TOGG1statusdebounce,TOGG2statusdebounce,TOGG3statusdebounce,TOGG4statusdebounce,TOGG5statusdebounce,TOGG6statusdebounce);
	  LCD_print(buffer,0,0);

	  sprintf(buffer,"Pot: %u %u    ",potenc1,potenc2);
	  LCD_print(buffer,0,1);

	  sprintf(buffer,"%u %u %u %u    ",LjoyUPDOWN,LjoyLEFTRIGHT,DjoyUPDOWN,DjoyLEFTRIGHT);
	  LCD_print(buffer,0,2);

	  sprintf(buffer,"LY %d %d    ",LjoyUPDOWNzeroOffset,LjoyLEFTRIGHTzeroOffset);
	  LCD_print(buffer,0,3);

	  sprintf(buffer,"RY %d %d    ",DjoyUPDOWNzeroOffset,DjoyLEFTRIGHTzeroOffset);
	  LCD_print(buffer,0,4);
}

void ButtonScreen2Print(char *buffer)
{
  	  sprintf(buffer,"Joystick Butt");
	  LCD_print(buffer,0,0);

	  sprintf(buffer,"LX- %d LX+ %d",TJoyLeftXMinusStatusDebounce,TJoyLeftXPlusStatusDebounce);
	  LCD_print(buffer,0,1);

	  sprintf(buffer,"LY- %d LY+ %d",TJoyLeftYMinusStatusDebounce,TJoyLeftYPlusStatusDebounce);
	  LCD_print(buffer,0,2);

	  sprintf(buffer,"RX- %d RX+ %d",TJoyRightXMinusStatusDebounce,TJoyRightXPlusStatusDebounce);
	  LCD_print(buffer,0,3);

	  sprintf(buffer,"RY- %d RY+ %d",TJoyRightYMinusStatusDebounce,TJoyRightYPlusStatusDebounce);
	  LCD_print(buffer,0,4);
}

void ButtonScreen3Print(char *buffer)
{
	  sprintf(buffer,"LCD Buttons");
	  LCD_print(buffer,0,0);

	  sprintf(buffer,"Tb %d %d %d %d",T1StatusDebounce,T2StatusDebounce,T3StatusDebounce,T4StatusDebounce);
	  LCD_print(buffer,0,1);

	  sprintf(buffer,"Ts %d %d %d %d",TUpStatusDebounce,TDownStatusDebounce,TLeftStatusDebounce,TRightStatusDebounce);
	  LCD_print(buffer,0,2);
}

void TestScreen1Print(char *buffer)
{
  	  //Diagnostics Inputs watch
	  sprintf(buffer,"Testing");
	  LCD_print(buffer,0,0);

  	  sprintf(buffer,"watch1 %d",watch1);
  	  LCD_print(buffer,0,1);

  	  sprintf(buffer,"watch2 %d",watch2);
	  LCD_print(buffer,0,2);

	  sprintf(buffer,"watch3 %d",watch3);
	  LCD_print(buffer,0,3);

	  sprintf(buffer,"watch4 %d",watch4);
	  LCD_print(buffer,0,4);
}

void FlashDataScreenRdPrint(char *buffer)
{
  	  sprintf(buffer,"Control: %u",FlashDataFlash.controlData);
  	  LCD_print(buffer,0,0);

  	  sprintf(buffer,"LCDcontr: %u",FlashDataFlash.LCD_contrast);
	  LCD_print(buffer,0,1);
}

void FlashDataScreenWrPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Control: %u",FlashDataActive.controlData);
	else sprintf(buffer," Control: %u",FlashDataActive.controlData);
	LCD_print(buffer,0,0);

	if(pos==Line1) sprintf(buffer,"*LCDcontr: %u",FlashDataActive.LCD_contrast);
	else sprintf(buffer," LCDcontr: %u",FlashDataActive.LCD_contrast);
	LCD_print(buffer,0,1);

	if(pos==Line2)  sprintf(buffer,"*");
	else sprintf(buffer," ");
	LCD_print(buffer,1,2);

	if(pos==Line3)  sprintf(buffer,"*");
	else sprintf(buffer," ");
	LCD_print(buffer,1,3);

	if(pos==Line4)  sprintf(buffer,"*");
	else sprintf(buffer," ");
	LCD_print(buffer,1,4);

	if(pos==Line5)  sprintf(buffer,"*");
	else sprintf(buffer," ");
	LCD_print(buffer,1,5);
}

void CommandScreen1Print(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*NRF24 TX=%d",!CommShtdownnrf24);
	else sprintf(buffer," NRF24 TX=%d",CommShtdownnrf24);
	LCD_print(buffer,1,0);

	if(pos==Line1)  sprintf(buffer,"*WriteFlData");
	else sprintf(buffer," WriteFlData");
	LCD_print(buffer,1,1);

	if(pos==Line2)  sprintf(buffer,"*EraseFlData");
	else sprintf(buffer," EraseFlData");
	LCD_print(buffer,1,2);

	if(pos==Line3)  sprintf(buffer,"*");
	else sprintf(buffer," ");
	LCD_print(buffer,1,3);

	if(pos==Line4)  sprintf(buffer,"*");
	else sprintf(buffer," ");
	LCD_print(buffer,1,4);

	if(pos==Line5)  sprintf(buffer,"*");
	else sprintf(buffer," ");
	LCD_print(buffer,1,5);

}



