// Lib para a UART do HT66F0195

#include "HT66F0195.h"

#define ID	0b10101010			// (170) ID do hardware ulizado para envio de msg via UART (CAN / BUS)
#define ID_EGT 0b01000110		// (70) ID da unidade leitora da sonda EGT
#define silent_can _pb6

//_________________________________________________________________________________________
// inicialização da UART
void uart_init(unsigned char baud)
{
	_rxps 	= 	0;		// PD1 - RX (1 = PB4)
	_txps 	= 	0;		// PD2 - TX
	_uarten = 	1;		// TX and RX pins can function as UART pins defined by TXEN and RXEN bits
	_txen 	= 	1;		// UART transmitter is enabled
	_rxen 	= 	1;		// UART receiver is enabled	
	_rxctl 	= 	1;
	_ure 	=   1;		// UART interrupt control is Enable
	_rie 	= 	1;		// Receiver related interrupt is enabled
	_brgh 	= 	1;		// High speed baud rate
	_brg 	= 	baud;	// baud rate value	
	_tx8 	= 	0;		// transmissão em 8 bits
}
//_________________________________________________________________________________________
void uart_transmit(unsigned char dat)
{	
	while(!_tidle)		
	{
		_nop();
	}
	
	_txr_rxr = dat;
	
	/* Waitting UART transmit data finished*/
	while(!_txif)
	{
		_nop();
	}
	/* transmit finished */
}
//_________________________________________________________________________________________
void send_msg(unsigned long msg)
{
	// estrutura do frame
	// ID destino (1 byte) + ID remetente (1 byte) + 4 bytes de dados (4,3,2,1)
	unsigned char data;
	
	_rxen = 0;					// desabilita o rx da UART
	GCC_DELAY(400);				// 100us
	_txen = 1;					// habilita o tx da UART
	silent_can = 0;				// TXD do CAN ativado			
	
	data = ID_EGT;
	uart_transmit(data);		// envio da ID do destino da msg
	data = ID;					
	uart_transmit(data);		// envio da ID do remetente msg
		
	data = (msg>>24);		
	uart_transmit(data);		// bits 25-32
	data = (msg>>16);		
	uart_transmit(data);		// bits 16-24
	data = (msg>>8);		
	uart_transmit(data);		// bits 8-15
	data = msg;					
	uart_transmit(data);		// bits 0-7
	
	GCC_DELAY(8000);			// 2ms antes de desativar o TX e reativar o RX
	silent_can = 1;				// apenas RXD ativado no CAN
	_txen = 0;					// desabilita o tx da UART
	GCC_DELAY(4000);			// delay para reativação do RX
	_rxen = 1;					// habilita o rx da UART
	
}
//_________________________________________________________________________________________


