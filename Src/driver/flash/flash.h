#ifndef __FLASH_H
#define __FLASH_H


#include "stm32f1xx_hal.h"

/* 定义FLASH 使用的IO口 */
#define FLASH_CS_SET HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)
#define FLASH_CS_CLR {HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);\
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);}

/* 定义各个FLASH的ID */
#define EN25Q80 	0X1C13 	
#define EN25Q16 	0X1C14
#define EN25Q32 	0X1C15
#define EN25Q64 	0X1C16

/* 定义指令表 */
#define EN25X_WriteStatusReg    0x01   //写状态寄存器
#define EN25X_PageProgram		0x02   //页编辑
#define EN25X_ReadData          0x03   //读数据
#define EN25X_WriteDisable		0x04   //写失能
#define EN25X_ReadStatusReg		0x05   //读取状态寄存器
#define EN25X_WriteEnable		0x06   //写使能
#define EN25X_SectorErase		0x20   //擦除块
#define EN25X_ChipErase			0xC7   //擦除片



/* 声明外部调用函数 */
uint16_t FLASH_ReadID(void);
uint16_t FLASH_Init(void);
void FLASH_ChipErase(void);
void FLASH_ReadData(uint8_t *readBuff, uint32_t readAddr, uint16_t readByteNum);
void FLASH_WriteData(uint8_t *writeBuff, uint32_t writeAddr, uint16_t writeByteNum);















#endif
