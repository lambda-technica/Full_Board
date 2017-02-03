//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// STM32 MCU low-level driver
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
//-------------------------------------------------------------------------------------------------
//#include "stm32f10x_lib.h"
//#include "stm32f1xx_hal_gpio.h"
#include "gpio.h"

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define KS0108_PORT  GPIOE

#define KS0108_RS    LCD_RS_Pin
#define KS0108_RW    LCD_RW_Pin
#define KS0108_EN    LCD_E_Pin
#define KS0108_CS1   LCD_CS2_Pin // CS Swapped
#define KS0108_CS2   LCD_CS1_Pin


#define GPIO_PIN_All  ((uint16_t)0xFFFF)  /* All pins selected */

#define KS0108_D0    0

#define DISPLAY_STATUS_BUSY	0x80


extern unsigned char screen_x;
extern unsigned char screen_y;

GPIO_InitTypeDef GPIO_InitStructure;


void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
   GPIOx->BRR = GPIO_Pin;
}

void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{

  GPIOx->BSRR = GPIO_Pin;
}

u16 GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  return ((u16)GPIOx->IDR);
}

void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{

  GPIOx->ODR = PortVal;
}


//-------------------------------------------------------------------------------------------------
// Delay function /for 8MHz/
//-------------------------------------------------------------------------------------------------
void GLCD_Delay(void) {
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

}
//-------------------------------------------------------------------------------------------------
// Enalbe Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_EnableController(unsigned char controller) {
	switch (controller) {
	case 0:
		GPIO_ResetBits(KS0108_PORT, KS0108_CS1);
		break;
	case 1:
		GPIO_ResetBits(KS0108_PORT, KS0108_CS2);
		break;
	//case 2:
	//	GPIO_ResetBits(KS0108_PORT, KS0108_CS3);
	//	break;


	}
}
//-------------------------------------------------------------------------------------------------
// Disable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_DisableController(unsigned char controller) {
	switch (controller) {
	case 0:
		GPIO_SetBits(KS0108_PORT, KS0108_CS1);
		break;
	case 1:
		GPIO_SetBits(KS0108_PORT, KS0108_CS2);
		break;
	//case 2:
	//	GPIO_SetBits(KS0108_PORT, KS0108_CS3);
	//	break;
	}
}
//-------------------------------------------------------------------------------------------------
// Read Status byte from specified controller (0-2)
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadStatus(unsigned char controller) {
	unsigned char status;

	//HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = 0xFF << KS0108_D0;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);

	GPIO_SetBits(KS0108_PORT, KS0108_RW);
	GPIO_ResetBits(KS0108_PORT, KS0108_RS);
	GLCD_EnableController(controller);
	GLCD_Delay();
	GPIO_SetBits(KS0108_PORT, KS0108_EN);
	GLCD_Delay();
	status = ((GPIO_ReadInputData(KS0108_PORT) >> KS0108_D0) & 0xFF);
	GPIO_ResetBits(KS0108_PORT, KS0108_EN);
	GLCD_DisableController(controller);
	return status;
}
//-------------------------------------------------------------------------------------------------
// Write command to specified controller
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite, unsigned char controller) {
	while (GLCD_ReadStatus(controller) & DISPLAY_STATUS_BUSY)
		;
	//HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = (0xFF << KS0108_D0);
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(KS0108_PORT, KS0108_RS | KS0108_RW);
	GLCD_Delay();
	GLCD_EnableController(controller);
	GLCD_Delay();
	GPIO_SetBits(KS0108_PORT, (commandToWrite << KS0108_D0));
	commandToWrite ^= 0xFF;
	GPIO_ResetBits(KS0108_PORT, (commandToWrite << KS0108_D0));
	GLCD_Delay();
	GPIO_SetBits(KS0108_PORT, KS0108_EN);
	GLCD_Delay();
	GPIO_ResetBits(KS0108_PORT, KS0108_EN);
	GLCD_Delay();
	GLCD_DisableController(controller);
}

//-------------------------------------------------------------------------------------------------
// Read data from current position
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void) {
	unsigned char tmp;
	while (GLCD_ReadStatus(screen_x / 64) & DISPLAY_STATUS_BUSY)
		;
	//HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = 0xFF << KS0108_D0;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);

	GPIO_SetBits(KS0108_PORT, KS0108_RS | KS0108_RW);

	GLCD_EnableController(screen_x / 64);
	GLCD_Delay();
	GPIO_SetBits(KS0108_PORT, KS0108_EN);
	GLCD_Delay();
	tmp = ((GPIO_ReadInputData(KS0108_PORT) >> KS0108_D0) & 0xFF);
	GPIO_ResetBits(KS0108_PORT, KS0108_EN);
	GLCD_DisableController(screen_x / 64);
	screen_x++;
	return tmp;
}
//-------------------------------------------------------------------------------------------------
// Write data to current position
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite) {
	while (GLCD_ReadStatus(screen_x / 64) & DISPLAY_STATUS_BUSY)
		;

	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.Pin = (0xFF << KS0108_D0);
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(KS0108_PORT, KS0108_RW);
	GLCD_Delay();
	GPIO_SetBits(KS0108_PORT, KS0108_RS);
	GLCD_Delay();
	GPIO_SetBits(KS0108_PORT, (dataToWrite << KS0108_D0));
	dataToWrite ^= 0xFF;
	GPIO_ResetBits(KS0108_PORT, (dataToWrite << KS0108_D0));
	GLCD_Delay();
	GLCD_EnableController(screen_x / 64);
	GLCD_Delay();
	GPIO_SetBits(KS0108_PORT, KS0108_EN);
	GLCD_Delay();
	GPIO_ResetBits(KS0108_PORT, KS0108_EN);
	GLCD_Delay();
	GLCD_DisableController(screen_x / 64);
	screen_x++;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_InitializePorts(void) {


	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_All;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;//GPIO_Speed_2MHz;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;

	HAL_GPIO_Init(KS0108_PORT, &GPIO_InitStructure);

	GPIO_Write(KS0108_PORT,
			KS0108_CS1 | KS0108_CS2 | KS0108_RS | KS0108_RW
					| (0xFF << KS0108_D0));


	HAL_GPIO_WritePin(GPIOE, LED_1_Pin | LED_2_Pin, GPIO_PIN_RESET);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadByteFromROMMemory(char * ptr) {
	return *(ptr);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
