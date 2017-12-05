#include "driver_LCD.h"
#include "stm32f4xx.h"
#include "Timer.h"

void LCD_Init(void)
{
	uint32_t tempReg;
	RCC->AHB1ENR|=(0x0000001B);
	/*
  	  PA0:PA7 are D0:D7 on the LCD screen (8 bits data bus)

	 */
	/*PA0:PA7 are set to output mode (01)*/
	tempReg = GPIOD->MODER;
	tempReg &= (0xFFFF0000);
	tempReg |= (0x00005555);
	GPIOD->MODER = tempReg;
	/*PA0:PA7 are set to push-pull mode (0)*/
	GPIOD->OTYPER &= (0xFF00);
	/*PA0:PA7 are set to Very High SPEed (11)*/
	GPIOD->OSPEEDR |= (0x0000FFFF);
	/*PA0:PA7 are set to No Pull-up or Pull-Down (00)*/
	GPIOD->PUPDR &= (0xFFFF0000);

	  /*
	  PD0 : RS -> Register Select, 0 = Instruction Register, 1 = Data Register
	  PD1 : RW -> Selects read or write, 0 = Write, 1 = Read
	  PD2 : E  -> Stars data read/write
	  */
	  /*PD0:PD2 are set to output mode (01)*/

	tempReg = GPIOE->MODER;
	tempReg &= (0xFFFFFFC0);
	tempReg |= (0x00000015);
	GPIOE->MODER = tempReg;
	/*PD0:PD2 are set to push-pull mode (0)*/
	GPIOE->OTYPER &= (0xFFF8);
	/*PD0:PD2 are set to very High Speed (11)*/
	GPIOE->OSPEEDR |= (0x0000003F);
	/*PD0:PD2 are set to No Pull-up or Pull-Down (00)*/
	GPIOE->PUPDR &= (0xFFFFFFC0);

	TIMER_Delay(1000);

	LCD_SendByteCommand(LCD_CONFIG);
	LCD_SendByteCommand(LCD_CLEAR_SCREEN);
	LCD_SendByteCommand(LCD_DISPLAY_CURSOR);
	LCD_SendByteCommand(LCD_INCREMENT_CURSOR);
	LCD_SendByteCommand(LCD_CURSOR_POSITION);
	TIMER_Delay(100);
}

void LCD_SendByteCommand(LCD_INSTRUCTION aInstruction)
{
	/*Sets enable to low*/
	GPIOE->ODR &= (0xFFFB);
	/*RW pin to 0 to write commands*/
	GPIOE->ODR &=(0xFFFD);
	/*RS pin to low to select instruction register*/
	GPIOE->ODR &=(0xFFFE);
	/*Sets the command on the BUS*/

	GPIOD->ODR &= (0xFF00);
	GPIOD->ODR |= (aInstruction);

	/*Toggle E pin to write on data BUS*/
	GPIOE->ODR |= (0x0004);
	TIMER_Delay(10);
	GPIOE->ODR &= (0xFFFB);
}

void LCD_SendByteData(char aData)
{

	/*Sets enable to HIGH*/
	GPIOE->ODR |= (0x0004);
	/*RW pin to 0 to write data*/
	GPIOE->ODR &=(0xFFFD);
	/*RS pin to high to select data register*/
	GPIOE->ODR |=(0x0001);
	/*Sets the command on the BUS*/

	GPIOD->ODR &= (0xFF00);
	GPIOD->ODR |= (aData);

	/*Toggle E pin to write on data BUS*/
	GPIOE->ODR &= (0xFFFB);
	TIMER_Delay(10);
	GPIOE->ODR |= (0x0004);
}

void LCD_SendStringData(char *aString)
{
	LCD_SendByteCommand(LCD_CONFIG);
	LCD_SendByteCommand(LCD_CLEAR_SCREEN);
	LCD_SendByteCommand(LCD_DISPLAY_CURSOR);
	LCD_SendByteCommand(LCD_INCREMENT_CURSOR);
	LCD_SendByteCommand(LCD_CURSOR_POSITION);
	TIMER_Delay(500);

	int NumberOfChar=0;
	while(*aString)
	{
		LCD_SendByteData(*aString);
		aString++;
		NumberOfChar++;
		if(NumberOfChar==16)
		{
			LCD_SendByteData(' ');
			LCD_SendByteCommand(LCD_CHANGE_LINE);
			LCD_SendByteData(' ');
		}
	}
}
