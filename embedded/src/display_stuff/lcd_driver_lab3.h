


#ifndef __LCD_DRIVER_LAB3_H
#define __LCD_DRIVER_LAB3_H

#define bin(a,b,c,d,e,f,g,h) ((a<<7)|(b<<6)|(c<<5)|(d<<4)|(e<<3)|(f<<2)|(g<<1)|(h<<0))



/* Includes ------------------------------------------------------------------*/
//#include "lcd_constantes.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
typedef enum I_DEnum{
	decr = 0,
	incr = 1
} incrDecrLcd_Type;

typedef enum shiftEnum{
	no_shift = 0,
	shift = 1
} shiftLcd_Type;

typedef enum displayEnum{
	displayOff = 0,
	displayOn = 1
} displayLcd_Type;

typedef enum cursorEnum{
	cursorOff = 0,
	cursorOn = 1
} cursorLcd_Type;

typedef enum blinkEnum{
	blinkOff = 0,
	blinkOn = 1
} blinkLcd_Type;

typedef enum cursorOrDisplayShiftEnum{
	cursorShift = 0,
	displayShift = 1
} cursorOrDisplayShiftLcd_Type;

typedef enum rightOrLeftShiftEnum{
	leftShift = 0,
	rightShift = 1
} rightOrLeftShiftLcd_Type;

typedef enum dataLengthEnum{
	length_4bits = 0,
	length_8bits = 1
} dataLengthLcd_Type;

typedef enum nombreDeLigneEnum{
	une_ligne = 0,
	deux_ligne = 1
} nombreDeLigneLcd_Type;

typedef enum caractereFontEnum{
	cinq_8 = 0,
	cinq_10 = 1
} caractereFontLcd_Type;

typedef enum caracterEnum{
	W = 0x57,
        G = 0x47,
	A = 0x41,
	B = 0x42,
	C = 0x43,
	D = 0x44,
	zero = 0x30,
	un = 0x31,
	deux = 0x32,
	trois = 0x33,
	quatre = 0x34,
	cinq = 0x35,
	six = 0x36,
	sept = 0x37,
	huit = 0x38,
	neuf = 0x39,
	etoile = 0x2A

} caracterLcd_Type;

typedef enum numeroLigneEnum{
	LINE1 = 0x00,
	LINE2 = 0x40
} LineLcd_Type;


void 	waitBusyFlag(void);
uint8_t lireAdresseCursor();
void    wait(uint32_t x);
void    clearLCD(void);
void    returnHome(void);
void    entryModeSet(incrDecrLcd_Type i_d, shiftLcd_Type shift);
void    displayOnOffControl( displayLcd_Type display, cursorLcd_Type cursor, blinkLcd_Type blink);
void    cursorDisplayShift (cursorOrDisplayShiftLcd_Type c_d, rightOrLeftShiftLcd_Type r_l );
void    functionSet(dataLengthLcd_Type dataLength, nombreDeLigneLcd_Type nombreLigne, caractereFontLcd_Type caractFont);
void    setCGramAdrr(caracterLcd_Type caract);
void    setDsramAdrr(LineLcd_Type line, uint8_t offset);
void 	setPortEcritureLcd();
void 	setPortLectureLcd();




#endif /*__LCD_DRIVER_LAB3_H*/