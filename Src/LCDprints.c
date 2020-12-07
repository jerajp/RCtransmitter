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
struct DroneDataStruct DroneDataRX;
struct DroneDataStruct DroneDataTX;
extern float DroneTuneStep;
extern uint32_t DroneTuneStepInt;

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

	  if(pos==Line5)  sprintf(buffer,"*DroneTune");
	  else sprintf(buffer," DroneTune");
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

	  sprintf(buffer,"%u %u %u %u    ",LjoyLEFTRIGHT,LjoyUPDOWN,DjoyLEFTRIGHT,DjoyUPDOWN);
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

	  sprintf(buffer,"Lxy:%d:%d   ",FlashDataFlash.LjoyXtrim, FlashDataFlash.LjoyYtrim );
	  LCD_print(buffer,0,2);

	  sprintf(buffer,"Rxy:%d:%d   ",FlashDataFlash.RjoyXtrim, FlashDataFlash.RjoyYtrim );
	  LCD_print(buffer,0,3);
}

void FlashDataScreenWrPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Control: %u",FlashDataActive.controlData);
	else sprintf(buffer," Control: %u",FlashDataActive.controlData);
	LCD_print(buffer,0,0);

	if(pos==Line1) sprintf(buffer,"*LCDcontr: %u",FlashDataActive.LCD_contrast);
	else sprintf(buffer," LCDcontr: %u",FlashDataActive.LCD_contrast);
	LCD_print(buffer,0,1);

	if(pos==Line2)  sprintf(buffer,"*Lxy:%d:%d   ",FlashDataActive.LjoyXtrim, FlashDataActive.LjoyYtrim);
	else sprintf(buffer," Lxy:%d:%d   ",FlashDataActive.LjoyXtrim, FlashDataActive.LjoyYtrim);
	LCD_print(buffer,1,2);

	if(pos==Line3)  sprintf(buffer,"*Rxy:%d:%d   ",FlashDataActive.RjoyXtrim, FlashDataActive.RjoyYtrim );
	else sprintf(buffer," Rxy:%d:%d   ",FlashDataActive.RjoyXtrim, FlashDataActive.RjoyYtrim );
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


void Param1TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Pitch P");
	else sprintf(buffer," Pitch P");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f     ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_p_gain_pitch);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_p_gain_pitch);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_p_gain_pitch);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_p_gain_pitch);
	LCD_print(buffer,1,5);

}
void Param2TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Pitch I");
	else sprintf(buffer," Pitch I");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f     ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_i_gain_pitch);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_i_gain_pitch);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_i_gain_pitch);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_i_gain_pitch);
	LCD_print(buffer,1,5);
}
void Param3TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Pitch D");
	else sprintf(buffer," Pitch D");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f      ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_d_gain_pitch);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_d_gain_pitch);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_d_gain_pitch);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_d_gain_pitch);
	LCD_print(buffer,1,5);
}
void Param4TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Roll P");
	else sprintf(buffer," Roll P");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f     ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_p_gain_roll);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_p_gain_roll);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_p_gain_roll);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_p_gain_roll);
	LCD_print(buffer,1,5);
}
void Param5TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Roll I");
	else sprintf(buffer," Roll I");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f     ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_i_gain_roll);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_i_gain_roll);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_i_gain_roll);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_i_gain_roll);
	LCD_print(buffer,1,5);
}
void Param6TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Roll D");
	else sprintf(buffer," Roll D");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f     ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_d_gain_roll);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_d_gain_roll);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_d_gain_roll);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_d_gain_roll);
	LCD_print(buffer,1,5);
}
void Param7TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Yaw P");
	else sprintf(buffer," Yaw P");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f      ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_p_gain_yaw);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_p_gain_yaw);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_p_gain_yaw);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_p_gain_yaw);
	LCD_print(buffer,1,5);
}
void Param8TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Yaw I");
	else sprintf(buffer," Yaw I");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f     ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_i_gain_yaw);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_i_gain_yaw);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_i_gain_yaw);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_i_gain_yaw);
	LCD_print(buffer,1,5);
}
void Param9TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Yaw D");
	else sprintf(buffer," Yaw D");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%.4f     ",DroneTuneStep);
	else sprintf(buffer," Stp:%.4f     ",DroneTuneStep);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%.4f",DroneDataRX.pid_d_gain_yaw);
	else sprintf(buffer," R:%.4f",DroneDataRX.pid_d_gain_yaw);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%.4f",DroneDataTX.pid_d_gain_yaw);
	else sprintf(buffer," T:%.4f",DroneDataTX.pid_d_gain_yaw);
	LCD_print(buffer,1,5);
}
void Param10TuneScreenPrint(char *buffer, CursorPositions pos)
{

	if(pos==Line0) sprintf(buffer,"*Pitch regMAX");
	else sprintf(buffer," Pitch regMAX");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.pid_max_pitch);
	else sprintf(buffer," R:%u",DroneDataRX.pid_max_pitch);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.pid_max_pitch);
	else sprintf(buffer," T:%u",DroneDataTX.pid_max_pitch);
	LCD_print(buffer,1,5);

}
void Param11TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Pitch intMAX");
	else sprintf(buffer," Pitch intMAX");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.pid_i_max_pitch);
	else sprintf(buffer," R:%u",DroneDataRX.pid_i_max_pitch);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.pid_i_max_pitch);
	else sprintf(buffer," T:%u",DroneDataTX.pid_i_max_pitch);
	LCD_print(buffer,1,5);
}
void Param12TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Roll regMAX");
	else sprintf(buffer," Roll regMAX");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.pid_max_roll);
	else sprintf(buffer," R:%u",DroneDataRX.pid_max_roll);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.pid_max_roll);
	else sprintf(buffer," T:%u",DroneDataTX.pid_max_roll);
	LCD_print(buffer,1,5);
}
void Param13TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Roll intMAX");
	else sprintf(buffer," Roll intMAX");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.pid_i_max_roll);
	else sprintf(buffer," R:%u",DroneDataRX.pid_i_max_roll);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.pid_i_max_roll);
	else sprintf(buffer," T:%u",DroneDataTX.pid_i_max_roll);
	LCD_print(buffer,1,5);
}
void Param14TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Yaw regMAX");
	else sprintf(buffer," Yaw regMAX");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.pid_max_yaw);
	else sprintf(buffer," R:%u",DroneDataRX.pid_max_yaw);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.pid_max_yaw);
	else sprintf(buffer," T:%u",DroneDataTX.pid_max_yaw);
	LCD_print(buffer,1,5);
}
void Param15TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Yaw intMAX");
	else sprintf(buffer," Yaw intMAX");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.pid_i_max_yaw);
	else sprintf(buffer," R:%u",DroneDataRX.pid_i_max_yaw);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.pid_i_max_yaw);
	else sprintf(buffer," T:%u",DroneDataTX.pid_i_max_yaw);
	LCD_print(buffer,1,5);
}
void Param16TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Pitch Angle");
	else sprintf(buffer," Pitch Angle");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.maxpitchdegree);
	else sprintf(buffer," R:%u",DroneDataRX.maxpitchdegree);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.maxpitchdegree);
	else sprintf(buffer," T:%u",DroneDataTX.maxpitchdegree);
	LCD_print(buffer,1,5);
}
void Param17TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Roll Angle");
	else sprintf(buffer," Roll Angle");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.maxrolldegree);
	else sprintf(buffer," R:%u",DroneDataRX.maxrolldegree);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.maxrolldegree);
	else sprintf(buffer," T:%u",DroneDataTX.maxrolldegree);
	LCD_print(buffer,1,5);
}
void Param18TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*Yaw Angle");
	else sprintf(buffer," Yaw Angle");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.maxyawdegree);
	else sprintf(buffer," R:%u",DroneDataRX.maxyawdegree);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.maxyawdegree);
	else sprintf(buffer," T:%u",DroneDataTX.maxyawdegree);
	LCD_print(buffer,1,5);
}
void Param19TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*min Throttle");
	else sprintf(buffer," min Throttle");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.minthrottle);
	else sprintf(buffer," R:%u",DroneDataRX.minthrottle);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.minthrottle);
	else sprintf(buffer," T:%u",DroneDataTX.minthrottle);
	LCD_print(buffer,1,5);
}
void Param20TuneScreenPrint(char *buffer, CursorPositions pos)
{
	if(pos==Line0) sprintf(buffer,"*max Throttle");
	else sprintf(buffer," max Throttle");
	LCD_print(buffer,1,0);

	if(pos==Line1) sprintf(buffer,"*Stp:%u     ",DroneTuneStepInt);
	else sprintf(buffer," Stp:%u     ",DroneTuneStepInt);
	LCD_print(buffer,1,1);

	if(pos==Line2) sprintf(buffer,"*Get Value");
	else sprintf(buffer," Get Value");
	LCD_print(buffer,1,2);

	if(pos==Line3) sprintf(buffer,"*Send Value");
	else sprintf(buffer," Send Value");
	LCD_print(buffer,1,3);

	if(pos==Line4) sprintf(buffer,"*R:%u",DroneDataRX.maxthrottle);
	else sprintf(buffer," R:%u",DroneDataRX.maxthrottle);
	LCD_print(buffer,1,4);

	if(pos==Line5) sprintf(buffer,"*T:%u",DroneDataTX.maxthrottle);
	else sprintf(buffer," T:%u",DroneDataTX.maxthrottle);
	LCD_print(buffer,1,5);
}

