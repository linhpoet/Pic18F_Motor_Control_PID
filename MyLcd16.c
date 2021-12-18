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
    int num_div = 10000;                                //ban dau = 10000, co the lay lon hon nen num>100.000
    int num_test = num;
    int num_display=0;
    if (num != 0) while (num_test/num_div <= 0 )                      
    {
        num_div /= 10;                                  //tim ra so 10xxx < num gan nhat de thuc hien phep chia 
    }
    else num_div = 1;
    
    do 
    {
        num_display = num_test/num_div;
        lcd_char(num_display + 48U);
        num_test = num_test - num_div*num_display;
        num_div /= 10;
    }
    while (num_div>=1);
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