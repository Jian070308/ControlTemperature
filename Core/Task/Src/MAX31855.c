/*
 * MAX31855.c
 *
 *  Created on: 16.08.2021
 *      Author: SimpleMethod
 *
 *Copyright 2021 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */


#include "MAX31855.h"

MAX31855_StateHandle MAX31855_Handle;

void MAX31855_Init(MAX31855_StateHandle *MAX31855, SPI_HandleTypeDef * hspi,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	 MAX31855->hspi=hspi;
	 MAX31855->nss.pin=GPIO_Pin;
	 MAX31855->nss.port=GPIOx;
	 MAX31855_SetNSSState(MAX31855,GPIO_PIN_SET);
}
uint8_t MAX31855_GetFault(MAX31855_StateHandle *MAX31855)
{
	if(MAX31855->ocFault)
	{
		return 1;
	}
	else if(MAX31855->scgFault)
	{
		return 2;
	}
	else if(MAX31855->scvFault)
	{
		return 3;
	}
	else
	{
		return 0;
	}

}
float MAX31855_GetTemperature(MAX31855_StateHandle *MAX31855)
{
	 return MAX31855->extTemp*0.25;
}
float MAX31855_GeInternalTemperature(MAX31855_StateHandle *MAX31855){
	 return MAX31855->intTemp*0.0625;
}

float MAX31855_GetTemperatureInFahrenheit(MAX31855_StateHandle *MAX31855)
{
 	 float temp=MAX31855_GetTemperature(MAX31855);
 	 temp*=1.8;
 	 temp += 32;
 	 return temp;
}
float MAX31855_GeInternalTemperatureInFahrenheit(MAX31855_StateHandle *MAX31855){
	 float temp=MAX31855_GeInternalTemperature(MAX31855);
	 temp*=1.8;
	 temp += 32;
	 return temp;
}

// void MAX31855_ReadData(MAX31855_StateHandle *MAX31855)
// {
// 	uint8_t payload[4];
// 	int32_t frame;
// 	MAX31855_SetNSSState(MAX31855,GPIO_PIN_RESET);
// 	HAL_SPI_Receive(MAX31855->hspi, payload, 4, 20);
// 	MAX31855_SetNSSState(MAX31855,GPIO_PIN_SET);
// 	MAX31855->scvFault=0;
// 	MAX31855->scgFault=0;
// 	MAX31855->ocFault=0;
// 	MAX31855->fault=0;
// 	MAX31855->extTemp=0;
// 	MAX31855->extTempSign=0;
// 	MAX31855->intTemp=0;
// 	MAX31855->intTempSign=0;
//
// 	frame = payload[0];
// 	frame = frame<<8;
// 	frame = frame|payload[1];
// 	frame = frame<<8;
// 	frame = frame|payload[2];
// 	frame = frame<<8;
// 	frame = frame|payload[3];
// 	int32_t extTemp=frame;
// 	int32_t intTemp=frame;
//
// 	if(frame& 0b00000000000000000000000000000100)
// 	{
// 		MAX31855->scvFault=1;
// 	}
// 	if(frame& 0b00000000000000000000000000000010)
// 	{
// 		MAX31855->scgFault=1;
// 	}
// 	if(frame& 0b00000000000000000000000000000001)
// 	{
// 		MAX31855->ocFault=1;
// 	}
// 	if(frame&0b00000000000000010000000000000000)
// 	{
// 		MAX31855->fault=1;
// 	}
// 	if(frame&0b10000000000000000000000000000000)
// 	{
// 		MAX31855->extTempSign=1;
// 	}
// 	if(frame&0b00000000000000010000000000000000)
// 	{
// 		MAX31855->intTempSign=1;
// 	}
//
// 	extTemp>>=18;
// 	if (MAX31855->intTempSign) {
// 		extTemp = ~(extTemp & 0b11111111111111);
// 	}
// 	MAX31855->extTemp=extTemp;
//
// 	intTemp = ((intTemp>>4) & 0b11111111111);
// 	if (MAX31855->intTempSign)
// 	{
// 		intTemp=~(intTemp|0b1111100000000000); //Experimental code, not tested!
// 	}
// 	MAX31855->intTemp=intTemp;
// }


void MAX31855_ReadData(MAX31855_StateHandle *MAX31855)
{
	uint8_t payload[4];
	uint32_t frame; // 使用无符号 32 位整型拼接，避免符号位干扰

	// 1. 通过 SPI 读取 4 个字节的数据
	MAX31855_SetNSSState(MAX31855, GPIO_PIN_RESET);
	HAL_SPI_Receive(MAX31855->hspi, payload, 4, 20);
	MAX31855_SetNSSState(MAX31855, GPIO_PIN_SET);

	// 2. 将 4 个字节拼接成一个 32 位的原始数据帧
	frame = ((uint32_t)payload[0] << 24) |
			((uint32_t)payload[1] << 16) |
			((uint32_t)payload[2] << 8)  |
			((uint32_t)payload[3]);

	// 3. 解析故障标志位 (使用十六进制掩码更清晰)
	MAX31855->fault    = (frame & 0x00010000) ? 1 : 0; // 第 16 位：综合故障标志
	MAX31855->scvFault = (frame & 0x00000004) ? 1 : 0; // 第 2 位：对 VCC 短路
	MAX31855->scgFault = (frame & 0x00000002) ? 1 : 0; // 第 1 位：对地短路
	MAX31855->ocFault  = (frame & 0x00000001) ? 1 : 0; // 第 0 位：热电偶开路(未连接)

	// 4. 解析热电偶实际温度 (外部温度，Bits 31:18，共 14 位)
	int16_t ext_raw = (frame >> 18) & 0x3FFF; // 提取 14 位数据
	if (ext_raw & 0x2000) {                   // 如果第 13 位(符号位)为 1，说明是负温
		ext_raw |= 0xC000;                    // 进行高位补 1，转换为标准的 C 语言负数(补码)
	}
	MAX31855->extTemp = ext_raw;
	// 注意：这里的 extTemp 还是原始值，实际温度 = extTemp * 0.25f

	// 5. 解析芯片内部参考温度 (Bits 15:4，共 12 位)
	int16_t int_raw = (frame >> 4) & 0x0FFF;  // 提取 12 位数据
	if (int_raw & 0x0800) {                   // 如果第 11 位(符号位)为 1，说明是负温
		int_raw |= 0xF000;                    // 进行符号位扩展
	}
	MAX31855->intTemp = int_raw;
	// 注意：实际内部温度 = intTemp * 0.0625f
}

void MAX31855_SetNSSState(MAX31855_StateHandle *MAX31855, GPIO_PinState state)
{
	HAL_GPIO_WritePin(MAX31855->nss.port,MAX31855->nss.pin,state);
}
