#ifndef __FLASH_H
#define __FLASH_H


#include "stm32f1xx_hal.h"

/* ����FLASH ʹ�õ�IO�� */
#define FLASH_CS_SET HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)
#define FLASH_CS_CLR {HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);\
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);}

/* �������FLASH��ID */
#define EN25Q80 	0X1C13 	
#define EN25Q16 	0X1C14
#define EN25Q32 	0X1C15
#define EN25Q64 	0X1C16

/* ����ָ��� */
#define EN25X_WriteStatusReg    0x01   //д״̬�Ĵ���
#define EN25X_PageProgram		0x02   //ҳ�༭
#define EN25X_ReadData          0x03   //������
#define EN25X_WriteDisable		0x04   //дʧ��
#define EN25X_ReadStatusReg		0x05   //��ȡ״̬�Ĵ���
#define EN25X_WriteEnable		0x06   //дʹ��
#define EN25X_SectorErase		0x20   //������
#define EN25X_ChipErase			0xC7   //����Ƭ



/* �����ⲿ���ú��� */
uint16_t FLASH_ReadID(void);
uint16_t FLASH_Init(void);
void FLASH_ChipErase(void);
void FLASH_ReadData(uint8_t *readBuff, uint32_t readAddr, uint16_t readByteNum);
void FLASH_WriteData(uint8_t *writeBuff, uint32_t writeAddr, uint16_t writeByteNum);















#endif
