
#include "mcp23017.h"

// Registers
#define REGISTER_IODIRA		0x00
#define REGISTER_IODIRB		0x01
#define REGISTER_IPOLA		0x02
#define REGISTER_IPOLB		0x03
#define REGISTER_GPINTENA	0x04
#define REGISTER_GPINTENB	0x05
#define REGISTER_DEFVALA	0x06
#define REGISTER_DEFVALB	0x07
#define REGISTER_INTCONA	0x08
#define REGISTER_INTCONB	0x09

#define REGISTER_GPPUA		0x0C
#define REGISTER_GPPUB		0x0D
#define REGISTER_INTFA		0x0E
#define REGISTER_INTFB		0x0F
#define REGISTER_INTCAPA	0x10
#define REGISTER_INTCAPB	0x11
#define REGISTER_GPIOA		0x12
#define REGISTER_GPIOB		0x13
#define REGISTER_OLATA		0x14
#define REGISTER_OLATB		0x15

void mcp23017_init(I2C_HandleTypeDef* I2Cx, uint16_t addr)
{
	uint8_t data;

	data=MCP23017_IODIR_ALL_INPUT;

	HAL_I2C_Mem_Write(I2Cx, addr, REGISTER_IODIRA, 1,&data, 1, 1000);
	HAL_I2C_Mem_Write(I2Cx, addr, REGISTER_IODIRB, 1,&data, 1, 1000);
}

void mcp23017_readA(I2C_HandleTypeDef* I2Cx, uint16_t addr,MCP23017str* DataStruct)
{
	uint8_t data;
	HAL_I2C_Mem_Read (I2Cx, addr, REGISTER_GPIOA, 1, &data, 1, 1000);
	DataStruct->gpioA[0]=(data & 1);
	DataStruct->gpioA[1]=(data & 2) >> 1;
	DataStruct->gpioA[2]=(data & 4) >> 2;
	DataStruct->gpioA[3]=(data & 8) >> 3;
	DataStruct->gpioA[4]=(data & 16) >> 4;
	DataStruct->gpioA[5]=(data & 32) >> 5;
	DataStruct->gpioA[6]=(data & 64) >> 6;
	DataStruct->gpioA[7]=(data & 128) >> 7;

}

void mcp23017_readB(I2C_HandleTypeDef* I2Cx, uint16_t addr,MCP23017str* DataStruct)
{
	uint8_t data;
	HAL_I2C_Mem_Read (I2Cx, addr, REGISTER_GPIOB, 1, &data, 1, 1000);
	DataStruct->gpioB[0]=(data & 1);
	DataStruct->gpioB[1]=(data & 2) >> 1;
	DataStruct->gpioB[2]=(data & 4) >> 2;
	DataStruct->gpioB[3]=(data & 8) >> 3;
	DataStruct->gpioB[4]=(data & 16) >> 4;
	DataStruct->gpioB[5]=(data & 32) >> 5;
	DataStruct->gpioB[6]=(data & 64) >> 6;
	DataStruct->gpioB[7]=(data & 128) >> 7;
}

