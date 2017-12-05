
#include "stm32f4xx.h"
//#include "stm32f4_discovery.h"
#include "lcd_driver_lab3.h"
#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_rcc.h"


void waitBusyFlag(void){
  
  //uint8_t bf;
 // setPortLectureLcd();
  GPIOC->MODER ^= (1 << 14); // pin7 en entrée pour lire le BF
  GPIOC->ODR = 0X200;
  //wait(2);
  while (GPIOC->IDR & 0X80);
  GPIOC->MODER |= (1 << 14);// pin7 en sortie
}


__IO uint32_t msctr;
void wait(uint32_t x){

    uint32_t now;
    now = msctr;
    while(msctr - now < x);
}

void clearLCD(void){
  
  GPIOC->ODR = 0x01;
  wait(2);
  GPIOC->ODR = 0x00;
}

uint8_t lireAdresseCursor(){
  
  uint8_t adrCursor;
  setPortLectureLcd();
  
  GPIOC->ODR = 0X200;
  wait(5);
  adrCursor = GPIOC->IDR & 0x7F;
  
  setPortEcritureLcd();  
  return adrCursor;
}

void returnHome(void){
  
  GPIOC->ODR = 0x02;
  wait(2);
  GPIOC->ODR = 0x00;
}
void entryModeSet(incrDecrLcd_Type i_d, shiftLcd_Type shift){
  
  GPIOC->ODR = (1<<2) | (i_d<<1) | (shift);
  wait(2);
  GPIOC->ODR = 0x00;
}

void displayOnOffControl( displayLcd_Type display, cursorLcd_Type cursor, blinkLcd_Type blink){

  GPIOC->ODR = (1 << 3) | (display << 2) | (cursor << 1) | blink;
  wait(2);
  GPIOC->ODR = 0x00;
}

void cursorDisplayShift (cursorOrDisplayShiftLcd_Type c_d, rightOrLeftShiftLcd_Type r_l ){
  
  GPIOC->ODR = (1 << 4) | (c_d << 3) | (r_l << 2);
  wait(2);
  GPIOC->ODR = 0x00;
}

void functionSet(dataLengthLcd_Type dataLength, nombreDeLigneLcd_Type nombreLigne, caractereFontLcd_Type caractFont){
  
  GPIOC->ODR = (1 << 5) | (dataLength << 4) | (nombreLigne << 3) | (caractFont << 2);
  wait(2);
  GPIOC->ODR = 0x00;
}

void setCGramAdrr(caracterLcd_Type caract){
  
  GPIOC->ODR = caract;
  GPIOC->ODR |=   (1<<10);
  wait(2);
  GPIOC->ODR = 0x00;
}

void setDsramAdrr(LineLcd_Type line, uint8_t offset){
  
  GPIOC->ODR = line + offset;
  GPIOC->ODR |= (1<<7);
  wait(2);
  GPIOC->ODR = 0x00;
}
void setPortLectureLcd(){

  GPIO_InitTypeDef GPIO_InitStruc;
  GPIO_InitStruc.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 
    | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruc.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruc.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruc);
}

void setPortEcritureLcd(){
  
  GPIO_InitTypeDef GPIO_InitStruc;
  GPIO_InitStruc.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 
    | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruc.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruc.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruc.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruc.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStruc);
}



