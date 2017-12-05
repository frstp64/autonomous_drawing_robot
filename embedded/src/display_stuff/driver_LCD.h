typedef enum _LCD_COMMAND
{
  LCD_CONFIG =          0x38,
  LCD_CHANGE_LINE =     0xC0,
  LCD_CLEAR_SCREEN =    0x0E,
  LCD_DISPLAY_CURSOR =  0x01,
  LCD_INCREMENT_CURSOR = 0x06,
  LCD_CURSOR_POSITION = 0x80

}LCD_INSTRUCTION;


void LCD_SendByteData(char aData);
void LCD_SendStringData(char *aString);
void LCD_Init(void);
void LCD_SendByteCommand(LCD_INSTRUCTION aInstruction);
