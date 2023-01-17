#ifndef _dsp_funcs_H_
#define	_dsp_funcs_H_

void start_i2c();
void stop_i2c();
void set_SDA();
unsigned char test_ack();
void int_ssd();
void write_i2c(unsigned char data);
void pre_send();
void column_setup(unsigned char start_column, unsigned char end_column);
void page_setup(unsigned char start_page, unsigned char end_page);
void clear_dsp();
void acess_mem(unsigned char cmd);
void clear_linha(char first_page);

#endif