/**
 * @file OLED.h
 * @author wls
 * @brief OLED 0.96 单色 128*64 显示屏
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"


/*引脚配置*/
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_5, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))

extern const uint8_t OLED_F8x16[][16];

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif

/*#include "stm32f10x.h"                  // Device header
#include "OLED.h"


int16_t Num;

int main(void)
{
	OLED_Init();
	
	
	OLED_ShowString(1, 1, "Num:");
	
	while (1)
	{
		Num ++;
		Num%=100;
		OLED_ShowSignedNum(1, 5, Num, 5);
	}
}*/
