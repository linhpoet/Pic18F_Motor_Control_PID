#include "MyLcd16.h"
#include <xc.h>
#include <stdio.h>


void lcd_command(char cmnd)
{
	CommandPort &= ~(1<<RW);			//RW=0,
	CommandPort &= ~(1<<RS);			//RS=0, ghi lenh

	PortData = cmnd;            //gui lenh (gui gia tri lenh vao thanh ghi cua port noi voi cac chan data (D0->D7)

	//		generate falling edge
	CommandPort |= (1<<E);
	__delay_ms(1);
    CommandPort &= ~(1<<E);
	__delay_ms(3);

}
void lcd_char(char data)
{
    CommandPort &= ~(1<<RW);			//RW=0,
	CommandPort |= (1<<RS);			//RS=1, ghi du lieu
	PortData = data;			//gui du lieu

	//		generate falling edge
	CommandPort |= (1<<E);
	__delay_ms(1);
    CommandPort &= ~(1<<E);
	__delay_ms(3);
}
void lcd_string(char *str)
{
	for(int i=0; i<strlen(str); i++)
	{
		lcd_char(str[i]);
	}
}

void lcd_number(int num)
{
    uint32_t u32TempNum;
    uint8_t u8Count = 0;
    uint8_t u8Display[10] ={};

    u32TempNum = num;

    while(u32TempNum>0)
    {
        u8Display[u8Count] = u32TempNum % 10;
        u8Count++;
        u32TempNum = u32TempNum / 10; 
    }
    if(num == 0)
    {
        u8Display[u8Count] = 0;
        u8Count = 1;
    }

    for(int i=u8Count-1; i>=0; i--)
    {
        lcd_char(u8Display[i] + 48);
    }
}

void lcd_gotoxy(char row, char pos)
{
    if(row == 1 && pos <16)
    {
        lcd_command((pos & 0x0f) | 0x80);
    }else if(row == 2 && pos<16)
    {
        lcd_command((pos & 0x0f) | 0xC0);
    }
}