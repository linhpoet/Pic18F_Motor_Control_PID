#pragma config OSC = RCIO6,CCP2MX = PORTC,MCLRE = OFF

#include <xc.h>
#include "MyLcd16.h"
#include <string.h>
#include <math.h>
#define _XTAL_FREQ  20000000
#define TMR2PRESCALE    4
#define Thuan 1
#define Nghich 2
long u32TempFrequency;
int tempC_Ch0;
int v_dat,v_thuc,so_xung;
int PWM, ADC;
float E0;
float Kp,Ki,Kd;
int state;

Set_Timer1_Counter(void)
{   
    TRISCbits.TRISC0 = 1;          //rc0 input
    T1CON = 0x03;
    TMR1L = 0;                  //clear tmr1l
    TMR1H = 0;                  //clear tmr1h
}

void Set_Timer2_PWM(void)               //Enable PWM o chan RC2/CCP1 va CCP2B
{
    //CCP1//
    CCP1M3 = 1;
    CCP1M2 = 1;
    /*CCP2*/
    CCP2M3 = 1;
    CCP2M2 = 1;
    
    //bo chia truoc o define_Thanh ghi T2CON//
    if (TMR2PRESCALE == 1)
    {
        T2CKPS0 = 0;
        T2CKPS1 = 0;
    }
    if (TMR2PRESCALE == 4)
    {
        T2CKPS0 = 1;
        T2CKPS1 = 0;
    }
    if (TMR2PRESCALE == 16)
    {
        T2CKPS0 = 1;
        T2CKPS1 = 1;
    }
    T2CONbits.TMR2ON = 1;       //timer 2 is on
    TRISC2 = 0;                 //tris c2 is output
}

//.........ham tinh PR2(so tick trong 1 chu ky PWM), gia tri vao u32Frequency la tan so mong muon cua PWM.......//
PWM_Frequency(long u32Frequency)
{
    PR2 = (_XTAL_FREQ/(u32Frequency*4*TMR2PRESCALE)-1);     //dung cong thuc, 0<=PR2<=255, nen chu y gia tri u32Frequency dua vao
    u32TempFrequency = u32Frequency;                        //luu lai gia tri u32Frequency de dung cho viec tinh Duty cycle
}

PWM_Duty1(unsigned int duty)                //duty dua vao la gia tri PWM mong muon, thi PWM hoat dong voi x=duty*100/1000 (%) 
{
    if (duty<1024)
    {
        duty = ((float)duty/1023)*(_XTAL_FREQ/(u32TempFrequency*TMR2PRESCALE));     //dung cong thuc
        CCP1X = duty & 2;
        CCP1Y = duty & 1;
        CCPR1L = duty >> 2;
    }
}

PWM_Duty2(unsigned int duty)                //duty dua vao la gia tri PWM mong muon, thi PWM hoat dong voi x=duty*100/1000 (%) 
{
    if (duty<1024)
    {
        duty = ((float)duty/1023)*(_XTAL_FREQ/(u32TempFrequency*TMR2PRESCALE));     //dung cong thuc
        CCP2X = duty & 2;
        CCP2Y = duty & 1;
        CCPR2L = duty >> 2;
    }
}

void ADC_Channel0_Configure()
{
    /*Port A Mode input*/
    TRISA = 0xff;
    /*ADC enable*/
    ADCON0bits.ADON = 1;
    /*0000 analog channel select bits-AN0*/
    ADCON0 &= ~(1<<2);
    ADCON0 &= ~(1<<3);
    ADCON0 &= ~(1<<4);
    ADCON0 &= ~(1<<5);
    ADRESH = 0;
    ADRESL = 0;
    /*ADC in progress*/
    ADCON0 |= 1<<1; 
    /*Port A Analog input*/
    ADCON1 = 0;
    ADCON2bits.ADFM = 1;
}

int Get_ADC()
{
    ADCON0 |= 1<<1;
    while(ADCON0bits.GO_nDONE);
    tempC_Ch0 = ((ADRESH << 8) | ADRESL);
    return tempC_Ch0;
}

/*PortB interrupt*/
void PortB_INT_Configuration()
{
    TRISB = 0xf0;               //trisB is input
    PORTB = 0xff;               
    INTCON = 0;                 //clear intcon
    INTCONbits.GIE = 1;         //enable global interrupt
    INTCONbits.PEIE = 1;        //enable peripheral interrupt
    INTCONbits.RBIE = 1;        //enable portB<7:4> interrupt
}

void LCD_Setting()
{
    DataTrist = 0;
    CommandTrist = 0;
    CommandPort = 0;
    PortData = 0;
    lcd_command(Mode8bit2line);					//8bit, 2 dong
	lcd_command(DisplayOn);						//bat hien thi
	lcd_command(ClearDisplay);					//clear
}

void Clear_Timer1()
{
    TMR1L = 0;
    TMR1H = 0;
}

void Cauhinh_LCD()
{
    LCD_Setting();
    lcd_string("v_dat  =      ");
    lcd_command(Goto2);
    lcd_string("v_thuc =      ");
}

void L298_Configuration()
{       
    TRISCbits.TRISC3 = 0;                   //trisc3 is output
    PORTCbits.RC3 = 0;                      //IN2 ouput = 0
    TRISCbits.TRISC4 = 0;                   //trisC4 is output
    PORTCbits.RC4 = 1;                      //ENA output = 1
}

void Led_Display()
{
    if(v_thuc<=50)
    {
        PORTB = 0x01;
    }
    else if(v_thuc<=300)
    {
        PORTB = 0x02;
    }
    else
    {
        PORTB = 0x04;
    }
}


void __interrupt() funtion()
{
    if (INTCONbits.RBIF == 1)
    {
        if (PORTBbits.RB5 == 0) 
        {   
            if (state == Thuan)
            {
                state = Nghich;
            } else if (state == Nghich)
            {
                state = Thuan;
            }
        }
    }
    INTCONbits.RBIF = 0;
}

void main(void) {
    Cauhinh_LCD();
    L298_Configuration();
    ADC_Channel0_Configure();
    Set_Timer2_PWM();
    PWM_Frequency(5000);
    Set_Timer1_Counter();
    PortB_INT_Configuration();
    Kp=1.535;
    TRISB = 0xf0;
    state = 1;
    while(1)
    {  
        ADC = Get_ADC();
        v_dat =(ADC/1023.0)*330;
        Led_Display();
        
        /*PID*/
        if(v_dat != v_thuc)
        {
        E0 = v_dat - v_thuc;
        PWM =PWM + Kp*E0;
        if(PWM>1023)
            PWM = 1023;
        if(PWM<0)
            PWM = 0;
        if(v_dat == 0)
            PWM = 0;
        }
        
        switch (state)
        {
            case Thuan:
                PWM_Duty1(PWM);
                PWM_Duty2(0);
                Clear_Timer1();        
                __delay_ms(200);                                    
                so_xung = ((TMR1H<<8) | TMR1L);
                v_thuc =((so_xung - 0)*10*2/22);
                /*Theo thuc nghiem de chinh toc do cho dung*/
                if(v_thuc > 165)
                   v_thuc--;
                if(v_thuc > 230)
                   v_thuc--;
                lcd_gotoxy(1,9);
                lcd_number(v_dat);
                lcd_string(" rpm   ");
                lcd_gotoxy(2,9);
                lcd_number(v_thuc);
                lcd_string(" rpm   ");
                break;
        
            case Nghich:
                PWM_Duty1(0);
                PWM_Duty2(PWM);
                Clear_Timer1();        
                __delay_ms(200);                                    
                so_xung = ((TMR1H<<8) | TMR1L);
                v_thuc =((so_xung - 0)*10*2/22);                   
                /*Theo thuc nghiem de chinh toc do cho dung*/
                if(v_thuc > 165)
                   v_thuc--;
                if(v_thuc > 230)
                   v_thuc--;
                lcd_gotoxy(1,9);
                lcd_string("-");
                lcd_number(v_dat);
                lcd_string("rpm   ");
                lcd_gotoxy(2,9);
                lcd_string("-");
                lcd_number(v_thuc);
                lcd_string("rpm   ");
                break;
        }        
    }
    return;
}