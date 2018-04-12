#include "spi.h"


SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/****************************************************************************
* Function Name  : SPI1_Config
* Description    : ��ʼ��SPI2
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
void SPI1_Config(void)
{
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;

	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/****************************************************************************
* Function Name  : SPI2_Config
* Description    : ��ʼ��SPI2
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/

void SPI2_Config(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	SPI_InitTypeDef  SPI_InitStructure;
//    /* SPI��IO�ں�SPI�����ʱ�� */
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
//    /* SPI��IO������ */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/***************************************************************************/
	/************************* ����SPI�Ĳ��� ***********************************/
	/***************************************************************************/
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ѡ��ȫ˫��SPIģʽ
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;     //����ģʽ
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //8λSPI
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       //ʱ�����ոߵ�ƽ
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;      //�ڵڶ���ʱ�Ӳɼ�����
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		  //Nssʹ���������
//	/* ѡ������Ԥ��ƵΪ256 */
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//�����λ��ʼ����
//	SPI_InitStructure.SPI_CRCPolynomial = 7;
//
//	SPI_Cmd(SPI2, ENABLE);
//	SPI_Init(SPI2, &SPI_InitStructure);
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;

	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/****************************************************************************
* Function Name  : SPI1_SetSpeed
* Description    : ����SPI1�Ĵ����ٶȡ�
* Input          : �ٶȲ����ʷ�Ƶ
* Output         : None
* Return         : None
****************************************************************************/

void SPI1_SetSpeed(uint8_t speed)
{
	SPI1->CR1 &= 0xFFC7;
	SPI1->CR1 |= speed;
//	SPI_Cmd(SPI1, ENABLE);
}

/****************************************************************************
* Function Name  : SPI2_SetSpeed
* Description    : ����SPI2�ķ�Ƶ�����Ըı�SPI2���ٶ�.
* Input          : Speed����Ƶ��
* Output         : None
* Return         : None
****************************************************************************/

void SPI2_SetSpeed(uint8_t Speed)
{
	SPI2->CR1 &= 0xFFC7;
	SPI2->CR1 |= Speed;
//	SPI_Cmd(SPI2,ENABLE);
}

/****************************************************************************
* Function Name  : SPI1_WriteReadData
* Description    : ʹ��SPI1д��һ���ֽ�����ͬʱ��ȡһ���ֽ����ݡ�
* Input          : dat��Ҫд��8λ����
* Output         : None
* Return         : ��ȡ����8λ����
****************************************************************************/

uint8_t SPI1_WriteReadData(uint8_t dat)
{
//	uint16_t i = 0;
	uint8_t val = 0;
	/* �����ͻ������� */
// 	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
//	{
//		i++;
//		if(i > 10000)
//		{
//			return 0xFF;
//		}
//	}
//
//    /* �������� */
//   	SPI_I2S_SendData(SPI1, dat);
//
//	/* �ȴ����ջ�����Ϊ�ǿ� */
//	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	HAL_SPI_TransmitReceive(&hspi1, &dat, &val, 1, 100);
	/* ����ȡ������ֵ���� */
	return val;
}

/****************************************************************************
* Function Name  : SPI2_WriteReadData
* Description    : ʹ��SPI2д��һ���ֽ�����ͬʱ��ȡһ���ֽ����ݡ�
* Input          : dat��д�������
* Output         : None
* Return         : ��ȡ��������
*                * ��ȡʧ�ܷ���0xFF
****************************************************************************/

uint8_t SPI2_WriteReadData(uint8_t dat)
{
//	uint16_t i = 0;
	uint8_t val = 0;
	/* �����ͻ������� */
// 	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
//	{
//		i++;
//		if(i > 10000)
//		{
//			return 0xFF;
//		}
//	}
//
//    /* �������� */
//   	SPI_I2S_SendData(SPI2, dat);
//
//	/* �ȴ����ջ�����Ϊ�ǿ� */
//	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	HAL_SPI_TransmitReceive(&hspi1, &dat, &val, 1, 100);
	/* ����ȡ������ֵ���� */
	return val;
}



















