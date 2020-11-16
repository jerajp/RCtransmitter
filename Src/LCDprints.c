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

	  sprintf(buffer," MSG status");
	  LCD_print(buffer,0,0);

	  sprintf(buffer," RC inputs status");
	  LCD_print(buffer,0,1);

	  sprintf(buffer," Flash Data");
	  LCD_print(buffer,0,2);

	  sprintf(buffer," Flash Commands");
	  LCD_print(buffer,0,3);

	  sprintf(buffer,"*");
	  LCD_print(buffer,0,pos);

}

//Add screens
/*switch(LCDMenu)
{
	  case 0:{
		  	  	  MainScreenPrint(stringlcdBuffer);

	  	  	 }break;

	  case 1:{
		  	  	  //Diagnostics Connection
		  	  	  sprintf(stringlcdBuffer,"Total MSG");
	  	  	  	  LCD_print(stringlcdBuffer,0,0);

	  	  	  	  sprintf(stringlcdBuffer,"Send %u",TotalMSGsend);
	  	  	  	  LCD_print(stringlcdBuffer,0,1);

	  	  	  	  sprintf(stringlcdBuffer,"Recv %u",TotalMSGrecv);
	  	  	  	  LCD_print(stringlcdBuffer,0,2);

	  	  	  	  sprintf(stringlcdBuffer,"MSG/s: %u",MSGprerSecond);
	  	  	  	  LCD_print(stringlcdBuffer,0,3);

		  	  	  sprintf(stringlcdBuffer,"RClow[s]: %u",MSGLowCount);
		  	  	  LCD_print(stringlcdBuffer,0,4);

	  	  	 }break;

	case 2:{

		  	  sprintf(stringlcdBuffer,"Toggle:%u%u%u%u%u%u",TOGG1statusdebounce,TOGG2statusdebounce,TOGG3statusdebounce,TOGG4statusdebounce,TOGG5statusdebounce,TOGG6statusdebounce);
		  	  LCD_print(stringlcdBuffer,0,0);

		  	  sprintf(stringlcdBuffer,"Pot: %u %u    ",potenc1,potenc2);
		  	  LCD_print(stringlcdBuffer,0,1);

		  	  sprintf(stringlcdBuffer,"%u %u %u %u    ",LjoyUPDOWN,LjoyLEFTRIGHT,DjoyUPDOWN,DjoyLEFTRIGHT);
		  	  LCD_print(stringlcdBuffer,0,2);

		  	  sprintf(stringlcdBuffer,"LY %d %d    ",LjoyUPDOWNzeroOffset,LjoyLEFTRIGHTzeroOffset);
			  LCD_print(stringlcdBuffer,0,3);

		  	  sprintf(stringlcdBuffer,"RY %d %d    ",DjoyUPDOWNzeroOffset,DjoyLEFTRIGHTzeroOffset);
		  	  LCD_print(stringlcdBuffer,0,4);


		  	  //sprintf(stringlcdBuffer,"Jo: %d %d %d %d  ",offsetLjoyUPDOWN,offsetLjoyLEFTRIGHT,offsetDjoyUPDOWN,offsetDjoyLEFTRIGHT);
		  	  //LCD_print(stringlcdBuffer,0,4);

		   }break;

	  case 3:{
			  	  //Diagnostics Inputs
		  	  	  sprintf(stringlcdBuffer,"Joystick Butt");
			  	  LCD_print(stringlcdBuffer,0,0);

		  	  	  sprintf(stringlcdBuffer,"LX- %d LX+ %d",TJoyLeftXMinusStatusDebounce,TJoyLeftXPlusStatusDebounce);
			  	  LCD_print(stringlcdBuffer,0,1);

		  	  	  sprintf(stringlcdBuffer,"LY- %d LY+ %d",TJoyLeftYMinusStatusDebounce,TJoyLeftYPlusStatusDebounce);
			  	  LCD_print(stringlcdBuffer,0,2);

		  	  	  sprintf(stringlcdBuffer,"RX- %d RX+ %d",TJoyRightXMinusStatusDebounce,TJoyRightXPlusStatusDebounce);
			  	  LCD_print(stringlcdBuffer,0,3);

		  	  	  sprintf(stringlcdBuffer,"RY- %d RY+ %d",TJoyRightYMinusStatusDebounce,TJoyRightYPlusStatusDebounce);
			  	  LCD_print(stringlcdBuffer,0,4);


	  	  	 }break;

	case 4:{
	  	  	  //Diagnostics Inputs
	  	  	  sprintf(stringlcdBuffer,"LCD Buttons");
	  	  	  LCD_print(stringlcdBuffer,0,0);

	  	  	  sprintf(stringlcdBuffer,"Tb %d %d %d %d",T1StatusDebounce,T2StatusDebounce,T3StatusDebounce,T4StatusDebounce);
		  	  LCD_print(stringlcdBuffer,0,1);

	  	  	  sprintf(stringlcdBuffer,"Ts %d %d %d %d",TUpStatusDebounce,TDownStatusDebounce,TLeftStatusDebounce,TRightStatusDebounce);
		  	  LCD_print(stringlcdBuffer,0,2);

		   }break;

	case 5:{
	  	  	  //Diagnostics Inputs watch
	  	  	  sprintf(stringlcdBuffer,"Control: %d",FlashDataActive.controlData);
	  	  	  LCD_print(stringlcdBuffer,0,0);

	  	  	  sprintf(stringlcdBuffer,"LCDcontr: %d",FlashDataActive.LCD_contrast);
		  	  LCD_print(stringlcdBuffer,0,1);

		   }break;



	case 6:{
	  	  	  //Diagnostics Inputs watch
	  	  	  sprintf(stringlcdBuffer,"watch1 %d",watch1);
	  	  	  LCD_print(stringlcdBuffer,0,0);

	  	  	  sprintf(stringlcdBuffer,"watch2 %d",watch2);
		  	  LCD_print(stringlcdBuffer,0,1);

		  	  sprintf(stringlcdBuffer,"watch3 %d",watch3);
		  	  LCD_print(stringlcdBuffer,0,2);

		  	  sprintf(stringlcdBuffer,"watch4 %d",watch4);
		  	  LCD_print(stringlcdBuffer,0,3);

		   }break;
}*/



