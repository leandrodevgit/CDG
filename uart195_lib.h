#ifndef _uart195_lib_H_
#define	_uart195_lib_H_

void uart_init(unsigned char baud);
void uart_transmit(unsigned char dat);
void send_msg(unsigned long msg);

#endif