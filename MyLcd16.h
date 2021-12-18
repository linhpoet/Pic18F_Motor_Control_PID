#include <string.h>
#include <stdio.h>
#include <xc.h>

#define _XTAL_FREQ      20000000

#define Mode8bit2line   0x38
#define DisplayOn       0x0c
#define ClearDisplay    0x01
#define Goto1           0x80
#define Goto2           0xc0

#define RS              5
#define RW              6
#define E               7
#define PortData        PORTD       //port noi voi d0-d7
#define CommandPort     PORTC       //port noi voi en, rw, rs
#define DataTrist       TRISD
#define CommandTrist    TRISC
void lcd_command(char cmnd);
void lcd_char(char data);
void lcd_string(char *str);
void lcd_number(int num);
void lcd_gotoxy(char row, char pos);