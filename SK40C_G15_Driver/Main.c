/* 
 * File:   Main.c
 * Author: inghui
 *
 */
#include "G15.h"
#define _XTAL_FREQ 20000000
#define BAUDRATE 500000
#define CTRL_G15 RC4



#define	SW1		RB0
#define	SW2		RB1

#define	LED1		RB6
#define	LED2		RB7

#include <htc.h>


void initIO(void);
unsigned char UART_REC(void);
void UART_SEND(unsigned char data);
void UART_INIT(void);
void lcdinit(void);
void delay(unsigned long data);
void send_config(unsigned char data);
void send_char(unsigned char data);
void lcd_goto(unsigned char data);
void lcd_clr(void);
void send_string(const char *s);
unsigned char lcd_num(unsigned short number, unsigned char digit);


// If PIC16F887 Configuration bits.
__CONFIG(       FOSC_HS  &		// exernal osc
                WDTE_OFF &		// Disable Watchdog Timer.
		PWRTE_OFF  &		// Enable Power Up Timer.
		BOREN_OFF &		// Disable Brown Out Reset.
		MCLRE_ON &		// MCLR function is enabled
		LVP_OFF );		// Disable Low Voltage Programming.

unsigned char ERR_FLAG=0; 
unsigned long UARTWAIT=15000;

void main(void) {
    unsigned char DATA[10];
    unsigned short pos, angle;

    initIO();
    lcdinit();
    UART_INIT();

     __delay_ms(100);

    lcd_clr();
    lcd_goto(0);
    send_string("G15 Demo ");
    lcd_goto(20);
    send_string("press SW1 or SW2 ");
   

    __delay_ms(500);
    __delay_ms(500);



    while(1){

        if(SW1==0){

            lcd_goto(20);
            send_string("Positioning     ");
            while(1){
                LED1^=1;
                __delay_ms(500);
                LED2^=1;

                SetLED(1,ON, iWRITE_DATA);
                SetPos(1, ConvertAngle2Pos(90), iWRITE_DATA);
                __delay_ms(500);

                if(SW2==0) break;

                SetPos(1, ConvertAngle2Pos(180), iWRITE_DATA);
                SetLED(1,OFF, iWRITE_DATA);
                __delay_ms(500);

                if(SW2==0) break;
            }//endwhile
        }


        if(SW2==0){
           lcd_goto(20);
           send_string("Position=      ");
           SetTorqueOnOff(1,OFF, iWRITE_DATA);
           while(1){
                GetPos(1, DATA);
                pos=0; //clear val
                pos=DATA[1];
                pos=pos<<8;
                pos|=DATA[0];
                angle=(word)ConvertPos2Angle(pos);
                lcd_goto(30);
                lcd_num(angle,3);
                if(SW1==0) break;
           }


        }

    }


  

 
  
}

//Functions: 

void initIO(void)
{


    //setup digital IO
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;

    // Initialize the I/O port direction.
    TRISA = 0b11111111;
    TRISB = 0b00000011;
    TRISC = 0b10000000;
    TRISD = 0;
    TRISE = 0b00000011;

    ANSEL=0x00;
    ANSELH=0x00;

    LED1=0; LED2=0;

}

//LCD functions----------------------------------------------------------------
#define	rs			RB4	//RS pin of the LCD display
#define	e			RB5	//E pin of the LCD display
#define	lcd_data	PORTD		//LCD 8-bit data PORT

void lcdinit(void){

    TRISD=0x00;
    TRISB4=0;
    TRISB5=0;
        //configure lcd
    send_config(0b00000001);			//clear display at lcd
    send_config(0b00000010);			//lcd return to home
    send_config(0b00000110);			//entry mode-cursor increase 1
    send_config(0b00001100);			//display on, cursor off and cursor blink off
    send_config(0b00111000);			//function set

    //display startup message
    lcd_clr();							//clear lcd

}
void delay(unsigned long data)		//delay function, the delay time
{					//depend on the given value
	for( ;data>0;data--);
}

void send_config(unsigned char data)	//send lcd configuration
{
	rs=0;					//set lcd to configuration mode
	lcd_data=data;				//lcd data port = data
	e=1;					//pulse e to confirm the data
	delay(50);
	e=0;
	delay(50);
}

void send_char(unsigned char data)		//send lcd character
{
 	rs=1;					//set lcd to display mode
	lcd_data=data;				//lcd data port = data
	e=1;					//pulse e to confirm the data
	delay(10);
	e=0;
	delay(10);
}

void lcd_goto(unsigned char data)		//set the location of the lcd cursor
{						//if the given value is (0-15) the
 	if(data<16)                             //cursor will be at the upper line
	{					//if the given value is (20-35) the
	 	send_config(0x80+data);		//cursor will be at the lower line
	}					//location of the lcd cursor(2X16):
	else					// -----------------------------------------------------
	{					// | |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15| |
	 	data=data-20;			// | |20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35| |
		send_config(0xc0+data);         // -----------------------------------------------------
	}
}

void lcd_clr(void)				//clear the lcd
{
 	send_config(0x01);
	delay(600);
}

void send_string(const char *s)			//send a string to display in the lcd
{
  	while (s && *s)send_char (*s++);
}


unsigned char lcd_num(unsigned short number, unsigned char digit){
    unsigned char num[16];
    unsigned char k=digit-1;
    unsigned char i,j;

    num[digit]=0;

    for(i=0;i<digit;i++){
        num[k-i]=number%10+'0';
        number=number/10;
        if(number==0) break;
    }

    send_string(&num[k-i]);

    for(j=0;j<k-i;j++)
        send_char(' ');

    return(i);
}



/*uart function*/
void UART_INIT(void){

      //set port direction
    TRISCbits.TRISC7=1;     //RX
    TRISCbits.TRISC6=0;     //TX
    TRISCbits.TRISC4  =0;  //EN1 or AX12_CTRL
    TRISCbits.TRISC5 =0;   //EN2  or G15_CTRL


     // Initialize UART.
    TXSTAbits.BRGH = 1;     // Select high speed baud rate.
    BAUDCTLbits.BRG16=1;    // Baud 16bits

    /*BaudRate=FOSC/(4(n+1))*/

    SPBRG =0x03; SPBRGH=0x01; //19200
    RCSTAbits.SPEN = 1;												// Enable serial port.
    TXSTAbits.TXEN = 1;
    RCSTAbits.CREN = 1;

}

void UART_SEND(unsigned char data)
{
	while(!TRMT) ; //wait for previous transmit completion
	TXREG=data;
        while(!TRMT) ;
}

unsigned char UART_REC(void)			//receive uart value
{
    unsigned long waitcount=0;
    unsigned char rec_data;

    if(RCSTAbits.OERR){
        RCSTAbits.CREN=0;
        RCSTAbits.CREN=1;
        ERR_FLAG=1;
        return(255);
    }
    // Read the received data.
    while (RCIF == 0) //wait for data
    {
        waitcount++;
        if (waitcount > UARTWAIT){ //	break if wait too long, no incoming data
            ERR_FLAG=1;
            return (255); //no line
        }
    }

    rec_data = RCREG;

    if (FERR == 1) {
        while(RCIF) rec_data=RCREG;
        ERR_FLAG=1;
        return (255);
    }
    else{
        ERR_FLAG=0;
        return rec_data;					//return the data received
    }
}
