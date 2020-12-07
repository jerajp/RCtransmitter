
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "main.h"

//Function prototypes
void MainScreenPrint(char *buffer);
void MenuScreen1Print(char *buffer,CursorPositions pos);
void MSGScreen1Print(char *buffer);
void ButtonScreen1Print(char *buffer);
void ButtonScreen2Print(char *buffer);
void ButtonScreen3Print(char *buffer);
void TestScreen1Print(char *buffer);
void FlashDataScreen1Print(char *buffer);
void FlashDataScreenRdPrint(char *buffer);
void FlashDataScreenWrPrint(char *buffer, CursorPositions pos);
void CommandScreen1Print(char *buffer, CursorPositions pos);

void Param1TuneScreenPrint(char *buffer, CursorPositions pos);
void Param2TuneScreenPrint(char *buffer, CursorPositions pos);
void Param3TuneScreenPrint(char *buffer, CursorPositions pos);
void Param4TuneScreenPrint(char *buffer, CursorPositions pos);
void Param5TuneScreenPrint(char *buffer, CursorPositions pos);
void Param6TuneScreenPrint(char *buffer, CursorPositions pos);
void Param7TuneScreenPrint(char *buffer, CursorPositions pos);
void Param8TuneScreenPrint(char *buffer, CursorPositions pos);
void Param9TuneScreenPrint(char *buffer, CursorPositions pos);
void Param10TuneScreenPrint(char *buffer, CursorPositions pos);
void Param11TuneScreenPrint(char *buffer, CursorPositions pos);
void Param12TuneScreenPrint(char *buffer, CursorPositions pos);
void Param13TuneScreenPrint(char *buffer, CursorPositions pos);
void Param14TuneScreenPrint(char *buffer, CursorPositions pos);
void Param15TuneScreenPrint(char *buffer, CursorPositions pos);
void Param16TuneScreenPrint(char *buffer, CursorPositions pos);
void Param17TuneScreenPrint(char *buffer, CursorPositions pos);
void Param18TuneScreenPrint(char *buffer, CursorPositions pos);
void Param19TuneScreenPrint(char *buffer, CursorPositions pos);
void Param20TuneScreenPrint(char *buffer, CursorPositions pos);
