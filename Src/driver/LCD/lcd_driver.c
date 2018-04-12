#include "lcd_driver.h"
#include "touch.h"
/****************************************************************************
*��������TFT_GPIO_Config
*��  �룺��
*��  ������
*��  �ܣ���ʼ��TFT��IO�ڡ�
****************************************************************************/

//void TFT_GPIO_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	/* ��ʱ��ʹ�� */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
//	                      | RCC_APB2Periph_GPIOG, ENABLE);

//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	/* FSMC_A10(G12) ��RS��G0��*/
//	GPIO_Init(GPIOG, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
//	                              | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_8
//								  | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11
//								  | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
//								  | GPIO_Pin_15 );

//	GPIO_Init(GPIOD, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9
//	                               | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12
//								   | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

//	GPIO_Init(GPIOE, &GPIO_InitStructure);
//}

///****************************************************************************
//* Function Name  : TFT_FSMC_Config
//* Description    : ��ʼ��FSMC
//* Input          : None
//* Output         : None
//* Return         : None
//****************************************************************************/

//void TFT_FSMC_Config(void)
//{
//	/* ��ʼ������ */
//	FSMC_NORSRAMInitTypeDef        FSMC_NORSRAMInitStructure;
//	FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTiming;

//	/* ���ö�дʱ�򣬸�FSMC_NORSRAMInitStructure���� */
//	/* ��ַ����ʱ�䣬3��HCLK���� */
//	FSMC_NORSRAMTiming.FSMC_AddressSetupTime = 0x02;

//	/* ��ַ����ʱ�䣬1��HCLK���� */
//	FSMC_NORSRAMTiming.FSMC_AddressHoldTime = 0x00;

//	/* ���ݽ���ʱ�䣬6��HCLK���� */
//	FSMC_NORSRAMTiming.FSMC_DataSetupTime = 0x05;

//	/* ���ݱ���ʱ�䣬1��HCLK���� */
//	FSMC_NORSRAMTiming.FSMC_DataLatency = 0x00;

//	/* ���߻ָ�ʱ������ */
//	FSMC_NORSRAMTiming.FSMC_BusTurnAroundDuration = 0x00;
//
//	/* ʱ�ӷ�Ƶ���� */
//	FSMC_NORSRAMTiming.FSMC_CLKDivision = 0x01;

//	/* ����ģʽ������ڵ�ַ/���ݲ�����ʱ��ABCDģʽ�����𲻴� */
//	FSMC_NORSRAMTiming.FSMC_AccessMode = FSMC_AccessMode_B;

//	/*����FSMC_NORSRAMInitStructure������*/
//	/* FSMC���ĸ��洢�飨bank��������ʹ�õ�һ����bank1�� */
//	/* ͬʱ����ʹ�õ���bank����ĵ� 4��RAM�� */
//	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;

//	/* ��������ʹ��SRAMģʽ */
//	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;

//	/* ʹ�õ����ݿ��Ϊ16λ */
//	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;

//	/* ����дʹ�ܴ� */
//	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;

//	/* ѡ����չģʽʹ�ܣ������ö���д�ò�ͬ��ʱ�� */
//	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
//
//	/* ���õ�ַ�����ݸ���ʹ�ܲ��� */
//	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
//
//	/* ���ö�дʱ�� */
//	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTiming;
//
//	/* ����дʱ�� */
//	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTiming;

//	/* ��FSMC��ʱ�� */
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
//
//	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

//	/*!< Enable FSMC Bank1_SRAM Bank */
//	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
//}

/****************************************************************************
* Function Name  : TFT_WriteCmd
* Description    : LCDд������
* Input          : cmd��д���16λ����
* Output         : None
* Return         : None
****************************************************************************/

void TFT_WriteCmd(uint16_t cmd)
{
	TFT->TFT_CMD = cmd >> 8;
	TFT->TFT_CMD = cmd & 0x00FF;
}

/****************************************************************************
* Function Name  : TFT_WriteData
* Description    : LCDд������
* Input          : dat��д���16λ����
* Output         : None
* Return         : None
****************************************************************************/

void TFT_WriteData(u16 dat)
{
	TFT->TFT_DATA = dat >> 8;
	TFT->TFT_DATA = dat & 0x00FF;
}

u16 TFT_ReadData(void)
{
	volatile unsigned short dat = 0;
	dat = TFT->TFT_DATA;
	dat = dat << 8;
	dat |= TFT->TFT_DATA;
	return dat;
}

u16 TFT_ReadReg(u16 dat)
{
	TFT_WriteCmd(dat);
	dat = dat;
	return TFT_ReadData();
}

/****************************************************************************
* Function Name  : TFT_Init
* Description    : ��ʼ��LCD��
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/

void TFT_Init(void)
{
	uint16_t i;

//	TFT_GPIO_Config();
//	TFT_FSMC_Config();

	for (i = 500; i > 0; i--);

	TFT_WriteCmd(0x0001);
	TFT_WriteData(0x0100);
	TFT_WriteCmd(0x0003);
	TFT_WriteData(0x1030); //16bit 565
	TFT_WriteCmd(0x0008);
	TFT_WriteData(0x0808);
	TFT_WriteCmd(0x0090);
	TFT_WriteData(0x8000);
	TFT_WriteCmd(0x0400);
	TFT_WriteData(0x6200);
	TFT_WriteCmd(0x0401);
	TFT_WriteData(0x0001);
	TFT_WriteCmd(0x00ff);
	TFT_WriteData(0x0001);
	TFT_WriteCmd(0x0102);
	TFT_WriteData(0x01b0);
	TFT_WriteCmd(0x0710);
	TFT_WriteData(0x0016);
	TFT_WriteCmd(0x0712);
	TFT_WriteData(0x000f);
	TFT_WriteCmd(0x0752);
	TFT_WriteData(0x002f);
	TFT_WriteCmd(0x0724);
	TFT_WriteData(0x001a);
	TFT_WriteCmd(0x0754);
	TFT_WriteData(0x0018);

	for (i = 500; i > 0; i--);

	/* Gamma */
	TFT_WriteCmd(0x0380);
	TFT_WriteData(0x0000);
	TFT_WriteCmd(0x0381);
	TFT_WriteData(0x5F10);//5f10
	TFT_WriteCmd(0x0382);
	TFT_WriteData(0x0B02);//0b02
	TFT_WriteCmd(0x0383);
	TFT_WriteData(0x0614);
	TFT_WriteCmd(0x0384);
	TFT_WriteData(0x0111);
	TFT_WriteCmd(0x0385);
	TFT_WriteData(0x0000);
	TFT_WriteCmd(0x0386);
	TFT_WriteData(0xA90B);//a90b
	TFT_WriteCmd(0x0387);
	TFT_WriteData(0x0606);//0606
	TFT_WriteCmd(0x0388);
	TFT_WriteData(0x0612);
	TFT_WriteCmd(0x0389);
	TFT_WriteData(0x0111);
	/* Gamma Setting */
	/* Setting */
	TFT_WriteCmd(0x0702);
	TFT_WriteData(0x003b);//003b
	TFT_WriteCmd(0x00ff);
	TFT_WriteData(0x0000);
	/* Vcom Setting */
	TFT_WriteCmd(0x0007);
	TFT_WriteData(0x0100);

	for (i = 500; i > 0; i--);

	TFT_WriteCmd(0x0200);
	TFT_WriteData(0x0000);
	TFT_WriteCmd(0x0201);
	TFT_WriteData(0x0000);
	TOUCH_Init();
}

/****************************************************************************
* Function Name  : TFT_SetWindow
* Description    : ����Ҫ�����Ĵ��ڷ�Χ
* Input          : xStart��������ʼX����
*                * yStart��������ʼY����
*                * xEnd�����ڽ���X����
*                * yEnd�����ڽ���Y����
* Output         : None
* Return         : None
****************************************************************************/

void TFT_SetWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
	TFT_WriteCmd(0x0210);
	TFT_WriteData(xStart);
	TFT_WriteCmd(0x0211);
	TFT_WriteData(xEnd);
	TFT_WriteCmd(0x0212);
	TFT_WriteData(yStart);
	TFT_WriteCmd(0x0213);
	TFT_WriteData(yEnd);
	TFT_WriteCmd(0x0200);
	TFT_WriteData(xStart);
	TFT_WriteCmd(0x0201);
	TFT_WriteData(yStart);
	TFT_WriteCmd(0x0202);
}

/****************************************************************************
* Function Name  : TFT_ClearScreen
* Description    : ��LCD��������Ӧ����ɫ
* Input          : color��������ɫ
* Output         : None
* Return         : None
****************************************************************************/

void TFT_ClearScreen(uint16_t color)
{
	uint16_t i, j ;
	TFT_SetWindow(0, 0, TFT_XMAX, TFT_YMAX);	 //��������

	for (i = 0; i < TFT_XMAX + 1; i++)
	{
		for (j = 0; j < TFT_YMAX + 1; j++)
		{
			TFT_WriteData(color);
		}
	}
}
