// Funções usadas para a operação do display oled c/ controlador SSD1306
// 07/2022

#include "cmd_SSD1306.h"
#include "HT66F0195.h"
//#include "images.h"

#define SDA 	_pc4
#define SCL 	_pc5
#define SDA_io 	_pcc4
#define AckErro 1
#define AckOk 0

bit bitaux;
unsigned char ack_sts;	// indica erro na comunicação I2C (ACK=1)
						// 1 = erro - 0 = ok
//_________________________________________________________________________________________
void start_i2c()	// condição de start do I2C - high to low SDA. SCL in HIGH
{
	SDA=1; 			// as duas linhas em high
	GCC_DELAY(6);
	SCL=1;
	GCC_DELAY(6);
	SDA=0;			// SDA vai para low  
	GCC_DELAY(6);	// 15us antes de SCL ir para low também
	SCL=0;			// SCL ficará em low, pois esta é zona de trasição da SDA
	GCC_DELAY(6);	// a proxima borda de subida de SCL será o primeiro clock
}
//_________________________________________________________________________________________
void stop_i2c()		// condição de stop do I2c - trasição de SDA de low para high com SCL em high
{	
	SDA=0; 			// SDA vai para low para que ocorra a trasição low to high
	GCC_DELAY(6);	// 15us
	SCL=1;			// SCL vai para high
	GCC_DELAY(6);	// 15us
	SDA=1; 			// SDA para high (low to high)	
	GCC_DELAY(6);	// 15us	
	SCL=0;			// SCL (clock) vai ficar em low na condição de idle para garantir que não ocorra um falso start
}
//_________________________________________________________________________________________
void set_SDA()		// altera o nível lógico da linha de dados 
{
	if((SDA==1) && (bitaux==1))	// se o bit a ser enviado for de valor = 1 e a saída já esteja em 1, então não realiza comando algum
	{
		GCC_NOP();	
	}
	else
	{	
		SDA=bitaux;
	}	
}
//__________________________________________________________________________________________
unsigned char test_ack()			// a cada byte enviado o SSD responde com um bit ack = 0 caso recepção OK
{
	unsigned char value;
	
	SDA_io=1;		// define SDA como entrada para verificar o ACK bit retornado pela memória	
	GCC_DELAY(4);	// 5us
	SCL=1;			// SCL vai para high
	GCC_DELAY(6);	// 5us
	value = SDA;	// lê o valor do pino SDA. Se o display estiver conectado no I2C bus a resposta será ZERO
	SCL=0;			// SCL vai para low
	GCC_DELAY(6);	// 5us
	SDA_io=0;		// define SDA como saída novamente
	GCC_DELAY(4);	// 5us	
	
	return value;
}
//_________________________________________________________________________________________
void write_i2c(unsigned char data)	// escrita na memória do display oled. Envio de 1 byte
{	
	bit bit7 = 0 ;						// bits para enviar dados via I2C	
	bit bit6 = 0 ;
	bit bit5 = 0 ;
	bit bit4 = 0 ; 
	bit bit3 = 0 ; 
	bit bit2 = 0 ; 
	bit bit1 = 0 ; 
	bit bit0 = 0 ;
	unsigned char cont_bit=8;

 	 while(cont_bit!=0)
  	{	
		switch(cont_bit)
		{
			case 1:
					data=data>>1;
					bit0=data;
			break ; 	
			
			case 2:
					data=data>>1;
					bit1=data;
			break ;
	
			case 3:
					data=data>>1;
					bit2=data;		
			break ;
			
			case 4:
					data=data>>1;
					bit3=data;		
			break ;
			
			case 5:
					data=data>>1;
					bit4=data;		
			break ;
			
			case 6:
					data=data>>1;
					bit5=data;		
			break ;
			
			case 7:
					data=data>>1;
					bit6=data;
			break ;	
			
			case 8:	// o bit 0 da var. passa para posição 7
					bit7=data;
			break ;													
		}
	
	cont_bit--;	// decrementa a variavél de controle	 
  	}
  	
	cont_bit=1;	// seta a var. de controle para envio de byte
	
	while(cont_bit!=9)
	{	
		
		switch(cont_bit)
		{
			case 1:
					bitaux=bit0;
					set_SDA();
			break ; 	
		
			case 2:
					bitaux=bit1;
					set_SDA();
			break ;

			case 3:
					bitaux=bit2;
					set_SDA();
			break ;
		
			case 4:
					bitaux=bit3;
					set_SDA();
			break ;
		
			case 5:
					bitaux=bit4;
					set_SDA();
			break ;
		
			case 6:
					bitaux=bit5;
					set_SDA();
			break ;
		
			case 7:
					bitaux=bit6;
					set_SDA();
			break ;	
		
			case 8:	
					bitaux=bit7;
					set_SDA();
			break ;	
		}												
			
		SCL=1;			// SCL vai para high
		GCC_DELAY(6);	
		SCL=0;			// SCL vai para low
		
		cont_bit++;		
	}
	
	ack_sts = test_ack();	// verifica se houve um bit ack (zero) que confirma o recebimento do byte	
}
//_________________________________________________________________________________________
void pre_send()
{
	start_i2c();		// inicia a comunicação
	write_i2c(0x78);	// send byte via I2C (device address + Write)
	write_i2c(0x40);	// ctrl byte (DATA)	
}
//_________________________________________________________________________________________
void acess_mem(unsigned char cmd)
{
	start_i2c();					// inicia a comunicação
	write_i2c(0x78);				// send byte via I2C (device address + Write)
	write_i2c(0);					// ctrl byte
	write_i2c(SSD1306_MEMORYMODE);	// define o modo de acesso da memoria
	write_i2c(cmd);					// acesso em modo vertical V = 1, horizontal H = 0
	stop_i2c();						// encerra a comunicação I2c
}
//_________________________________________________________________________________________
void column_setup(unsigned char start_column, unsigned char end_column)	
{	// configura a coluna inicial/final e a linha inicial
	
	start_i2c();	// inicia a comunicação i2c
	
	write_i2c(0x78);					// send byte via I2C (device address + Write)
	write_i2c(0);						// ctrl byte
	write_i2c(SSD1306_COLUMNADDR); 		// set column address
	write_i2c(start_column);			// coluna inicial
	write_i2c(end_column);				// coluna final
	
	stop_i2c();		// encerra a comunicação I2c	
}
//_________________________________________________________________________________________
void page_setup(unsigned char start_page, unsigned char end_page)
{
	start_i2c();	// inicia a comunicação i2c
	
	write_i2c(0x78);					// send byte via I2C (device address + Write)
	write_i2c(0);						// ctrl byte
	write_i2c(SSD1306_PAGEADDR);		// set page address
	write_i2c(start_page);				// pagina inicial
	write_i2c(end_page);				// pagina final
	
	stop_i2c();		// encerra a comunicação I2c	
}
//_________________________________________________________________________________________
void clear_dsp()	// apaga toda memória do display
{

	start_i2c();		// inicia a comunicação

	write_i2c(0x78);	// send byte via I2C (device address + Write)
	write_i2c(0);		// ctrl byte (COMANDO)
	write_i2c(SSD1306_COLUMNADDR);	// set column address 0x21
	write_i2c(0);		// coluna inicial é a zero
	write_i2c(127);		// coluna final é a 127
	write_i2c(SSD1306_PAGEADDR);	// set page address 0x22 a zero
	write_i2c(0);
	write_i2c(7);		// pagina final é a 7

	stop_i2c();			// encerra a comunicação I2C

	pre_send();
	
	
	unsigned int i;
	
	for(i=0; i<1024; i++)
	{
		write_i2c(0);	// envia o byte	
	}	

	stop_i2c();			// encerra a comunicação I2c
	
}
//_________________________________________________________________________________________
void int_ssd()		// inicializa o display oled
{
	start_i2c();	// inicia a comunicação
	
	write_i2c(0x78);					// send byte via I2C (device address + Write)
	write_i2c(0);						// ctrl byte
	write_i2c(SSD1306_DISPLAYOFF);		// display off
	write_i2c(SSD1306_SETMULTIPLEX);	// Set Multiplex Ratio
	write_i2c(0x3f);					// set max. value for the multiplex ratio
	write_i2c(SSD1306_SETDISPLAYOFFSET);// Set Display Offset
	write_i2c(0);						// inicia na linha zero (topo da tela)
	write_i2c(SSD1306_SEGREMAP);		
	write_i2c(SSD1306_COMSCANDEC);
	write_i2c(SSD1306_MEMORYMODE);		// define o modo de acesso da memoria
	write_i2c(0);						// acesso em modo horizontal ->
	//write_i2c(1);						// acesso em modo vertical V
	write_i2c(SSD1306_SETSTARTLINE);	// Set Display Start Line
	write_i2c(SSD1306_SETPRECHARGE);	// Set Pre-charge Period
	write_i2c(0xf1);					// pre charge value
	write_i2c(SSD1306_SETVCOMDETECT);
	write_i2c(0x40);
	write_i2c(SSD1306_SETCONTRAST);		// set the contrast display
	write_i2c(48);						// nivel 120. range: 0 - 255
	write_i2c(SSD1306_DISPLAYALLON_RESUME); 
	write_i2c(SSD1306_NORMALDISPLAY);
	write_i2c(SSD1306_SETDISPLAYCLOCKDIV);
	write_i2c(0xf1);
	write_i2c(SSD1306_CHARGEPUMP);
	write_i2c(0x14);
	write_i2c(SSD1306_DISPLAYON);
	
	stop_i2c();							// encerra a comunicação I2c
}
//_________________________________________________________________________________________
void clear_linha(char first_page)
{
	unsigned int i = 0;
	column_setup(0, 127);	// alinhamento horizontal
	page_setup(first_page, first_page+1);
		
	pre_send();
	while(i<255)
	{
		write_i2c(0);		
		i++;
	}
	stop_i2c();		
}
//_________________________________________________________________________________________