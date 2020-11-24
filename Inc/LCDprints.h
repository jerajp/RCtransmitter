
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
