// ///////////////////////////////// CDG_V111 /////////////////////////////////////
// programa p/ pci CDG-V1.1 (central diesel gnv)
// 01/2022. Tamanho do programa em linhas: 2533

// #includes_______________________________________________________________________________
#include "HT66F0195.h"
#include "images.h"
#include "dsp_funcs.h"
#include "cmd_SSD1306.h"
#include "eeprom_lib.h"
#include "uart195_lib.h"
#include "bcd.h"
#include "read_an.h"
#include "msg_dsp.h"
#include "delay_ms.h"
// fim dos includes________________________________________________________________________

// Delcarações #DEFINE_____________________________________________________________________
// Ios
#define sw1 		_pb0
#define bt 			_pb1
#define en_ind 		_pb3
#define en_hall 	_pb4
#define en_tja 		_pb5
#define silent_can 	_pb6
#define led_sts 	_pc0
#define gnv_ctrl 	_pc1
#define SDA 		_pc4
#define SCL 		_pc5
#define SDA_io 		_pcc4

#define ACK_ERRO 	1
#define LED_ON 		1
#define LED_OFF 	0
#define Diesel 		0	// modo
#define DieselGnv 	1	// modo
#define OK 			1	// pulso_sts
#define NONE 		0	// pulso_sts
#define On 			1	// sa_ctrl = 1
#define Off 		2	// sa_ctrl = 2
#define POSITIVO 	1	// fator POSITIVO para a leitura do sinal do acelerador
#define NEGATIVO 	2	// fator NEGATIVO para a leitura do sinal do acelerador
#define TRUE 		1
#define FALSE 		0
#define NACK 		1	// I2C - 1 = erro
#define ACK 		0	// I2C - 0 = ok
#define Ok  		1	// egt_manag e egt_com
#define Erro 		2	// egt_manag e egt_com
#define TimeOut 	3	// egt_manag e egt_com
#define WAINTING 	1
#define PRESS		0

#define ALL   		0 	// atualiza todas as informações
#define MODO  		1	// modo. Diesel/GNV ou GNV
#define REDUT 		2	// status redutor de pressão. ON/OFF
#define LEVEL 		3	// Nivel de GNV
#define TEMP  		4	// Indicação de temperatura alta
#define PULSE 		5	// Indicação de pulso OK
#define Tps	  		6 	// Indicação da função TPS ativa
#define MCON		7	// Mensagem do modo_config ativado

#define CURTO 		0	// botão da interface. click curto
#define LONGO 		1	// click longo
#define IDLE  		2	// idle (sem click)
#define RUN	  		1	// contagem de tempo de click do botão em curso
#define STOP  		0 	// contagem de tempo de click do botão paralizada

#define BAUD_38400	25				// baud rate de 38400 p/ 16MHz
#define READ_EGT	0xff55			// msg que solicita a leitura da sonda EGT
#define ID			0b10101010		// (170) ID do hardware ulizado para envio de msg via UART (CAN / BUS)
#define ID_EGT 		0b01000110		// (70) ID da unidade leitora da sonda EGT
#define dez_sec		80

// fim das declarações #DEFINE_____________________________________________________________


// constantes______________________________________________________________________________

// fim das constantes______________________________________________________________________

// Vetores interrupções____________________________________________________________________
void __attribute((interrupt(0x24))) int_ext1();	// vetor da int. externa 1
void __attribute((interrupt(0x20))) int_tb1();	// vetor da int. time base 1
void __attribute((interrupt(0x0C))) int_mf0();	// vetor da int. Multi Funct. 0
void __attribute((interrupt(0x2C))) UART_ISR();	// vetor da int. UART RX
//_________________________________________________________________________________________

//variaveis globais________________________________________________________________________

extern unsigned char 	ack_sts;			// indica erro na comunicação I2C (ACK=1) 1 = erro - 0 = ok
											// variavel localizada no arquivo dsp_funcs.C
unsigned char 			UART_ISR_Value;		// uart receive buff
volatile unsigned char 	error_rx_flag;		// uart receive error

unsigned char 			modo;				// indica qual o combustivel em uso: 0 GNV - 1 Diesel/GNV - 0x00
bit 					modo_config;		// flag que indica se o modo config está tivado (TRUE) ou Não FALSE
unsigned char 			s_type;				// inidica qual o tipo de sinal será lido. 0 Hall - 1 Indutivo - 0x01
volatile unsigned char 	pulso_sts;			// indica se há pulso (hall ou indutivo) presente. OK ou NONE
volatile unsigned int 	periodo;			// variavel p/ armazenar o periodo(t) do pulso medido via stm
volatile unsigned int 	periodo_val;		// variavel p/ armazenar o periodo valido
volatile unsigned char 	pval_saved;			// flag usada na calibração da RPM. Informa quendo um perido valido foi salvo
volatile unsigned char  indice;				// 4 leituras de periodo validas em sequencia devem ser computadas
volatile unsigned char 	media_ok;			// flag que sinaliza que a média dos 4 periodos foi calculada
unsigned int t_buffer[4];					// buffer para armazenar 4 periodos (tempos medidos via smt)
volatile unsigned char 	tout_erro;			// indica TimeOut ou periodo invalido na medição
unsigned char 			temp_sts;			// indica se a temperatura EGT está acima do nivel selecionado
volatile unsigned char 	timeout_menu;		// inatividade dentro do menu gera a saída do mesmo na ocorrência de timeout
volatile unsigned char 	click;				// indica se o click do botão da interface foi curto, longo ou está em idle
volatile unsigned char 	time_bt;			// contagem de tempo para a medição de tempo do click do botão da interface
volatile unsigned char 	start_tbt;			// indica que a contagem do tempo de click do botão está em curso
char 					sa_flag = 2;		// flag que indica que o pedal do acelerador não está em repouso (posição neutra)
unsigned int 			sa_xvalue;			// contem o valor que será comparado com o valor sa_value (lido em AN4)	
unsigned int 			sa_value;			// valor de tensão do acelerador
unsigned char 			sa_ctrl;			// ON/OFF p/ a função sensor do acelerador
unsigned char 			sa_fator;			// fator 1(+) = a tensão AUMENTA conforme o pelal vai ao fim de curso
											// fator 2(-) = a tensão DIMINUI conforme o pedal vai ao fim de curso
unsigned char 			rpm_sel;			// indica qual a rpm que está selecionada. 7(700), 9(900), 11(1100), 13(1300), 15(1500)
unsigned int 			t_target;			// periodo alvo com base na rpm selecionada
unsigned int 			t_800;				// periodo de 800 RPM usado para corte do GNV através da RPM marcha lenta. Usado quando TPS = OFF
unsigned char 			last_atl;			// armazena qual foi o último valor atualizado para determindado parâmetro		
volatile unsigned char 	poweron;			// utilizada para o primeiro acionamento do GNV. 1 = 1º acionamento, 0 = 2º ou >	
unsigned char 			cnt;	
unsigned long 			msg;				// var. de 4 bytes p/ msg a ser enviada p/ a unidade de leitura da sonda EGT
volatile unsigned char 	r_egt;				// liberação p/ a leitura da sonda EGT	
volatile unsigned char 	w_ret;				// 1 = esperando resposta da unidade EGT, 0 = não está aguardando reposta	
unsigned int 			egt_temp;			// temperatura da sonda EGT. Dado recebido via UART
unsigned int 			egt_temp_target;	// temperatura limite de operação
unsigned char 			temp;				// temperatura limite EGT (menu)
unsigned int 			temp_base;			// temperatura que serve de base para se cacular as temperaturas (opções) do menu	
bit 					TermoPar_status;	// 1 = termopar aberto ou cabeamento rompido
char 					egt_status;			// relativo a comunicação com a unidade EGT. OK, ERRO ou TimeOut
volatile unsigned char 	buffer[6];			// buffer para armazenar a msg via UART / CAN
volatile unsigned char 	ctrl_byte;			// controle da recepção dos 6 bytes	
unsigned char 			go_atl;				// liberação p/ a atualização do display
unsigned int 			cnt_leituras;		// leituras consecutivas devem ser capturas para evitar ruido na zona de transição		
// fim das declarações de variaveis gloabais_______________________________________________

// funções_________________________________________________________________________________
void print_logo()
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
		write_i2c(logo[i]);	// envia o byte	
	}	

	stop_i2c();			// encerra a comunicação I2c
}
//_________________________________________________________________________________________
void print_level(unsigned char b1, unsigned char b2, unsigned char b3, unsigned char b4)
{
	
	unsigned char i;	// indica qual das 4 barras que será printada
	unsigned char j;
	unsigned char bx[4] = {b1,b2,b3,b4};
	unsigned char startcol = 40;
	unsigned char endcol = 55;
	page_setup(3,7);	// alinhamento vertical da barra. O mesmo para as 4 barras
	
	for(i=0; i<4; i++)	// laço para o print das 4 barras
	{	
		startcol += 18;
		endcol += 18;
		column_setup(startcol, endcol);	// alinhamento horizontal da barra
		j = 0;
		
		pre_send();
		while(j<80)
		{
			if(bx[i] == 1)	
			{
				write_i2c(255);				// barra cheia
			}
			else
			{
				write_i2c(barra_oca[j]);	// barra oca
			}
			j++;
		}
		stop_i2c();		
	}
	
}
//_________________________________________________________________________________________
void w_text(char word[12], unsigned char startpage, unsigned char startcol, unsigned char size)
{
	// a palavra/frase é sempre escrita na mesma linha
	// uma linha é composta por um pár de paginas. Ex: page 0 e 1
	unsigned char x;				// acesso ao array bi-dimensional dos caracteres
	unsigned char y;
	unsigned char i=0;
	unsigned char endpage = startpage+1;
	unsigned char endcol;
	unsigned char c_md = 1;
	
	page_setup(startpage, endpage);	// seta pagina inicial e final apenas uma vez. Escrita na mesma linha
	// modo vertical de acesso a memoria p/ impressão de texto.
	acess_mem(c_md);	// vertical
	
	while(i<size)
	{
		endcol = startcol+7;
		column_setup(startcol,endcol);	// seleciona nova coluna p/ editar pixels
		
		pre_send();		// comando para envio de dados
		x = word[i]-32;	// X indica qual é o caracter
		y = 7;			// byte inicial do caracter. São 16 bytes
		
		// laço while para envio de um caracter
		while(y<15)
		{
			y = y-7;
			write_i2c(font_8x16[x] [y]);	// page 1
			y = y+8;
			write_i2c(font_8x16[x] [y]);	// page 2
		}
		
		stop_i2c();
		
		i++;			// proximo caracter da string
		startcol = startcol+10;
	}
	
	// retorna para o modo de acesso horizontal
	c_md = 0;
	acess_mem(c_md);	// horizontal
}
//_________________________________________________________________________________________
void page_ind(unsigned char bola, unsigned char nbolas)
{	// indica através das bolinhas qual é a pagina que está sendo exibida no display

	unsigned char horiz  = (128 - (nbolas*13)) / 2;
	unsigned char j;
	unsigned char i;
	unsigned char end;
	
	clear_linha(7);						// limpa a linha antes de printar as bolinhas

	for(i=1; i<=nbolas; i++)			// laço para printar as boloas
	{	
		end = horiz+7;
		column_setup(horiz, end);		// alinhamento horizontal
		page_setup(7, 7);				// alinhamento vertical
		j = 0;
			
		pre_send();
		while(j<8)						// loop para enviar os dados imagem
		{
			if(i == bola)				// a variavel bola indica qual é a bola que é cheia
			{
				write_i2c(bola_cheia[j]);
			}
			else						// somente uma bola cheia é impresa
			{
				write_i2c(bola_vazia[j]);	// bola vazia
			}
			
			j++;
		}
		stop_i2c();	
		
		horiz += 14;					// pula dois pixels para separação das bolas
	}
}
//_________________________________________________________________________________________
void atl_mscreen(unsigned char i)		// atualização periodia da tela principal
{
	// 0 ALL   = atualiza todas as informações - tempo para atualização de todos os elementos da tela: 193mS
	// 1 MODO  = modo. Diesel/GNV ou GNV
	// 2 REDUT = status redutor de pressão. ON/OFF
	// 3 LEVEL = Nivel de GNV
	// 4 TEMP  = Indicação de temperatura alta
	// 5 PULSE = Indicação de pulso OK
	// 6 Tps   = Indicação da função TPS
	// 7 MCON  = Mensagem do modo_config
	
	if( ((i==MODO) || (i==ALL)) && (modo != last_atl) )
	{
		
		clear_linha(0);				// limpa o campo de informação
		if(modo == DieselGnv)
		{
			w_text(DIESELGNV,0,10,10);
		}
		else
		{
			w_text(DIESEL,0,34,6);	
		}
		last_atl = modo;
	}
	
	if((i==REDUT) || (i==ALL))
	{
		if(gnv_ctrl == 1)
		{
			w_text("ON ",3,22,3);
		}	
		else
		{
			w_text(OFF,3,22,3);
		}
	}
	
	if((i==LEVEL) || (i==ALL))
	{
		unsigned int value;	
		
		value = read_an(5);	// faz a leitura do sensor manômetro. AN5 MANO.
		

		if(value<=1310) 	// atualiza o nivel de GNV com base no valor lido no AN4
		{
			print_level(1,1,1,1);				//nivel 5 full - 0,88v
		}
		else
		{
			if(value<=1515) 					
			{
				print_level(1,1,1,0);			//nivel 4  - 1,85v	
			}
			else
			{
				if(value<=2285)
				{
					print_level(1,1,0,0);		//nivel 3  - 2,79v	
				}
				else
				{
					if(value<=3300)
					{
						print_level(1,0,0,0);	//nivel 2 - 4,38v	
					}
					else
					{
						if(value>3301) 	
						{
							print_level(0,0,0,0);// 1 reserva - Maior que 4,38v			
						}	
					}								
				}				
			}			
		}
		
			
	}
	
	if((i==TEMP) || (i==ALL))
	{
		unsigned char j = 0;
		column_setup(0, 15);	// alinhamento horizontal
		page_setup(3, 7);		// alinhamento vertical
		
		pre_send();
		while(j<80)
		{
			if((egt_temp > egt_temp_target) && (egt_status == Ok))
			{
				write_i2c(thermo[j]);	// icone overheat
			}
			else if((TermoPar_status == 1) && (egt_status == Erro))	// se o termopar estiver desconectado exibe o alerta o display
			{
				write_i2c(alerta[j]);	// icone alterta	
			}
			else if((egt_temp < egt_temp_target) && (egt_status == Ok))
			{							// EGT indica que há comunicação com a unidade EGT. 
				write_i2c(egt_icon[j]);	// icone egt
			}
			else if(egt_status == TimeOut)
			{
				write_i2c(0);			// local vazio indica que não há comunicação com a unidade
			}
			
			j++;
		}
		stop_i2c();	
	}	
	
	if((i==PULSE) || (i==ALL))
	{
		if(periodo != 0xffff)			// sem ausencia de pulso
		{
			w_text("P",0,117,1);
		}
		else
		{
			w_text(" ",0,117,1);
		}
	}	
	
	if((i==Tps) || (i==ALL))
	{
		if(sa_ctrl == On)
		{
			w_text(TPS,6,22,3);	
		}
		else
		{
			w_text("   ",6,22,3);
		}
	}
	
	if(i == MCON)
	{
		w_text("MODO CONFIG.",1,0,12);
		w_text("ATIVADO!",3,0,8);
	}
		
}
//_________________________________________________________________________________________
unsigned char check_periodo(unsigned char i)
{
	unsigned char status = FALSE;
	
	if(i <= 3)
	{
		t_buffer[i] = periodo_val;		
	}
	
	if(i == 3)	
	{	// com os periodos salvos é possível fazer a verificação e média		
		// verificação dos periodos salvos
		unsigned int t_mais = (t_buffer[0] * 40)/100;		// 40%
		unsigned int t_menos = t_buffer[0] - t_mais;		// t0 - 40%
		t_mais += t_buffer[0];								// t0 + 40%
			
		if(	((t_buffer[1] < t_mais) && (t_buffer[1] > t_menos)) &&
			((t_buffer[2] < t_mais) && (t_buffer[2] > t_menos)) &&	
			((t_buffer[3] < t_mais) && (t_buffer[3] > t_menos)) )
		{
			periodo_val = (t_buffer[0] + t_buffer[1] + t_buffer[2] + t_buffer[3]) / 4;
			status = TRUE;	// sinaliza que a média está  ok
		}
	}
	
	return status;
}
//_________________________________________________________________________________________
void main_scan()		// scan principal
{
		_clrwdt();
		
		// verifica se o modo foi alterado
		// GNV ON_____________________________________________________________________________
		if((pval_saved == TRUE) && (poweron == TRUE))
		{
			pval_saved = FALSE;
			media_ok = check_periodo(indice);
			
			if(indice<3)
			{
				indice++;
			}
			else
			{
				indice=0;
			}
		}
		
		if( (modo == DieselGnv) &&
			(gnv_ctrl == 0) &&
			(pulso_sts == OK) &&
			(poweron == TRUE) && 
			((egt_temp < egt_temp_target) || (egt_status != Ok)) &&
			((periodo_val <= t_target)&&(media_ok == TRUE)) &&
			( ((sa_ctrl == On) && (sa_flag == TRUE)) || ((sa_ctrl == Off) && (periodo_val < t_800))	)
																									  )
			//
		{
			poweron = FALSE, pval_saved = FALSE;
			gnv_ctrl = 1;		// ativa o GNV
		}
		else if( (modo == DieselGnv) &&
				 (gnv_ctrl == 0) &&
				 (pulso_sts == OK) &&
				 (poweron == FALSE) &&
				 ((egt_temp < egt_temp_target) || (egt_status != Ok)) &&
				 ( ((sa_ctrl == On) && (sa_flag == TRUE)) || ((sa_ctrl == Off) && (periodo_val < t_800))  )
																											)
				//
		{
			gnv_ctrl = 1;		// ativa o GNV
		}
		
		// GNV OFF___________________________________________________________________________
		if( (modo == DieselGnv) &&
			(gnv_ctrl == 1) &&
			
			((pulso_sts == NONE) ||
			 ((sa_ctrl == On) && (sa_flag == FALSE)) ||
			 ((sa_ctrl == Off) && (periodo_val > t_800)) ||
			 ((egt_temp > egt_temp_target) && (egt_status == Ok)) )
			 															)
		{
			gnv_ctrl = 0;		// desativa o GNV
		}
		else if( (modo == Diesel) && (gnv_ctrl == 1) )
		{
			gnv_ctrl = 0;		// desativa o GNV
		}
		
		// leitura do pedal do ACELERADOR - TPS_______________________________________________
		if(sa_ctrl == On)
		{
			sa_value = read_an(4);	// leitura do pedal do acelerador. AN4
			
			if( (sa_fator == POSITIVO) && (sa_value >= sa_xvalue) )
			{
				if(cnt_leituras < 50)
				{
					cnt_leituras++;	
				}
				else
				{
					sa_flag = TRUE;
					
				}
			}
			else if( (sa_fator == NEGATIVO) && (sa_value <= sa_xvalue) )
			{
				if(cnt_leituras < 50)
				{
					cnt_leituras++;	
				}
				else
				{
					sa_flag = TRUE;
				}
			}	
			else
			{
				sa_flag = FALSE;
				cnt_leituras = 0;	// reset no contador
			}
		}
}
//_________________________________________________________________________________________
void atl_temps(unsigned char temp)
{
	switch(temp)					// atualiza a temperatura alvo
	{
		case 1:	
				egt_temp_target = temp_base-30;	
		break;	
								
		case 2:	
				egt_temp_target = temp_base-15;	
		break;	
								
		case 3:	
				egt_temp_target = temp_base;	
		break;	
								
		case 4:	
				egt_temp_target = temp_base+15;
		break;
								
		case 5:
				egt_temp_target = temp_base+30;
		break;	
	}
}
//_________________________________________________________________________________________
void callback(char cmd)
{
	clear_dsp();
	if(cmd == 0)
	{
		w_text(SALVO, 3, 34, 6);	// SALVO!
	}
	else
	{
		w_text("ERRO AO", 3, 24, 7);
		w_text("SALVAR!", 6, 24, 7);
	}
	delay_ms(2000);
	clear_linha(3);
	_clrwdt();
}
//_________________________________________________________________________________________
void print_digt(char digit, char pos)
{
		switch(digit)
		{
			case 0:
					w_text("0", 3, pos, 1);
			break;
					
			case 1:
					w_text("1", 3, pos, 1);
			break;
					
			case 2:
					w_text("2", 3, pos, 1);
			break;
					
			case 3:
					w_text("3", 3, pos, 1);
			break;
					
			case 4:
					w_text("4", 3, pos, 1);
			break;
					
			case 5:
					w_text("5", 3, pos, 1);
			break;
					
			case 6:
					w_text("6", 3, pos, 1);
			break;
					
			case 7:
					w_text("7", 3, pos, 1);
			break;
					
			case 8:
					w_text("8", 3, pos, 1);
			break;
					
			case 9:
					w_text("9", 3, pos, 1);
			break;
		}	
		
		w_text(" ", 3, 19, 1);
		w_text(" ", 3, 39, 1);
		w_text(" ", 3, 59, 1);
		
		switch(pos)
		{
			case 29:
					w_text(">", 3, 19, 1);		
			break;
			
			case 49:
					w_text(">", 3, 39, 1);
			break;
			
			case 69:
					w_text(">", 3, 59, 1);
			break;
		}
}
//_________________________________________________________________________________________
void edit_tb()						// edição da tempratura base - EGT
{
	char i = 0;
	char digts[7] = {0};
	unsigned long digitos;
	char loop = TRUE;
	char pos = 29;
	unsigned int centena;
	unsigned char dezena;
	_clrwdt();
	delay_ms(3000);
	click = IDLE;
	clear_dsp();
	_clrwdt();
	start_i2c();					// inicia a comunicação
	write_i2c(0x78);				// send byte via I2C (device address + Write)
	write_i2c(0);					// ctrl byte
	write_i2c(SSD1306_INVERTDISPLAY);
	stop_i2c();	
	
	digitos = bcd_convert(temp_base);	
	digts[0] = (digitos>>16);		// centena
	digts[1] = 32;					// espaço
	digts[2] = (digitos>>8);		// dezena
	digts[3] = 32;					// espaço
	digts[4] = digitos;				// unidade
	digts[5] = 32;					// espaço
	digts[6] = 91;					// °C
	w_text(">", 3, 19, 1);
	w_text(digts, 3, 29, 7);
	
	while(loop == TRUE)
	{	
		_clrwdt();
		if(click == CURTO)
		{
			click = IDLE;
				
			if(digts[i]<9)
			{
				digts[i]++;	
			}
			else
			{
				digts[i] = 0;	
			}
				
			print_digt(digts[i], pos);
		}	
		else if(click == LONGO)
		{
			click = IDLE;
			if(i < 4)
			{
				i+=2;
				pos +=20;
				print_digt(digts[i], pos);
			}
			else
			{
				centena = digts[0] * 100;
				dezena = digts[2] * 10;
				centena += (dezena+digts[4]);
				clear_dsp();
				
				if(centena <= 994)
				{
					char check;
					char total_check = 0;
					unsigned char data = centena;
					temp_base = centena;
					atl_temps(temp);
					check = grava_eeprom(0x11, data);		// LSByte
					
					if(check == 1)
					{
						total_check++;
					}
						
					data = (centena>>8);
					check = grava_eeprom(0x10, data);		// MSByte
					
					if(check == 1)
					{
						total_check++;	
					}
						
					callback(total_check);
				}
				else
				{
					w_text("ATENCAO", 0, 29, 7);
					w_text("TEMPERATURA", 2, 9, 11);
					w_text("INVALIDA!", 4, 19, 9);
					_clrwdt();
					delay_ms(2500);
				}
				
				clear_dsp();	
				loop = FALSE;
				clear_dsp();
				start_i2c();					// inicia a comunicação
				write_i2c(0x78);				// send byte via I2C (device address + Write)
				write_i2c(0);					// ctrl byte
				write_i2c(SSD1306_NORMALDISPLAY);
				stop_i2c();	
			}
		}
	}
}
//_________________________________________________________________________________________
void egt_options(char opt)
{
	char t_option[5];		// texto relativo a opção de tempratura dentro de: Menu >> EGT
	t_option[4] = 91;		// °C
	unsigned long digitos;
	
	switch(opt)
	{
		case 1:	
				digitos = bcd_convert(temp_base-30);	
				t_option[0] = (digitos>>24);	// milhar
				t_option[1] = (digitos>>16);	// centena
				t_option[2] = (digitos>>8);		// dezena
				t_option[3] = digitos;			// unidade
				w_text(t_option, 3, 44, 5);		// 1ª opção: xxxx°C
		break ;
		
		case 2:	
				digitos = bcd_convert(temp_base-15);	
				t_option[0] = (digitos>>24);	
				t_option[1] = (digitos>>16);	
				t_option[2] = (digitos>>8);	
				t_option[3] = digitos;			
				w_text(t_option, 3, 44, 5);		// 2ª opção: xxxx°C
		break ;
		
		case 3:
				digitos = bcd_convert(temp_base);	
				t_option[0] = (digitos>>24);	
				t_option[1] = (digitos>>16);	
				t_option[2] = (digitos>>8);	
				t_option[3] = digitos;		
				w_text(t_option, 3, 44, 5);		// 3ª opção: xxxx°C
		break ;
		
		case 4:
				digitos = bcd_convert(temp_base+15);	
				t_option[0] = (digitos>>24);	
				t_option[1] = (digitos>>16);	
				t_option[2] = (digitos>>8);		
				t_option[3] = digitos;			
				w_text(t_option, 3, 44, 5);		// 4ª opção: xxxx°C
		break ;
		
		case 5:
				digitos = bcd_convert(temp_base+30);	
				t_option[0] = (digitos>>24);	
				t_option[1] = (digitos>>16);	
				t_option[2] = (digitos>>8);	
				t_option[3] = digitos;		
				w_text(t_option, 3, 44, 5);		// 5ª opção: xxxx°C
		break ;
		
		case 6:
				w_text(DIAG, 3, 9, 11);			// 5ª opçao: DIAGNOSTICO
		break ;
		
		case 7:
				w_text(VOLTAR, 3, 34, 6);		// 6ª opção: VOLTAR
		break ;
	}
	
	page_ind(opt,7);
}
//_________________________________________________________________________________________
void fast_blink()		// led_sts em 4Hz
{
	char i;
	
	for(i=0; i<4; i++)
	{
		led_sts=~led_sts;
		delay_ms(25);	
	}	
}
//_________________________________________________________________________________________
char egt_manag()
{
	char result = Ok;
	egt_temp = 0;
	TermoPar_status = 0;
	
	// destino e origem da msg
	if((buffer[0] == ID) && (buffer[1] == ID_EGT))
	{
		// byte 2 e 3 serão descartados		
		egt_temp = buffer[4]; 		// MSByte
		egt_temp = (egt_temp<<8);	// desloca o valor para a parte alta

		egt_temp += buffer[5]; 		// LSByte
		TermoPar_status = (egt_temp>>2);
		egt_temp = (egt_temp>>3);	// ajuste para colocar os 12 bits alinhados a direita
		egt_temp = egt_temp/4;		// converte para °C
		
		// um valor acima de 4089 (12 bits) é um valor invalido.
		if((egt_temp > 4089) || (TermoPar_status == 1))		
		{
			result = Erro;
		}
	}
	else
	{
		result = Erro;	
	}
	
	if(result == Erro)
	{
		fast_blink();
	}
	
	return result;
}
//_________________________________________________________________________________________
char egt_com()
{
	char atl = FALSE;
	
		// envio de msg (leitura de temperatura) para unidade EGT
		if((r_egt == 1) && (w_ret == IDLE))
		{	
			r_egt++;
			msg = READ_EGT;			// mensagem de 4 bytes que será enviada + 1 byte de destino + 1 byte de origem
			send_msg(msg);			// envia mensagem para a unidade EGT solicitando a temperatura
			w_ret = WAINTING;		// sinaliza que está aguardando resposta
			ctrl_byte = 0;			// recebe 6 bytes como resposta
		}
		
		// erro na recepção da UART
		if( (w_ret == WAINTING) && (error_rx_flag == 1) )
		{
			w_ret = IDLE;			// cancela a recepção da mensagem que está em curso
			egt_status = Erro;		// status é de erro devido ruido, frame ou overrun.
			error_rx_flag = 0;		// limpa a flag
			fast_blink();			// sinalização através do led_sts
		}
		
		// 6 bytes recebidos dentro do tempo limite para resposta
		if((w_ret == WAINTING) && (ctrl_byte == 6) && (r_egt < 8))
		{
			w_ret = IDLE;			// liberaçao para um novo envio de msg
			egt_status = egt_manag();	
			atl = TRUE;				// autoriza a atualização das informações na tela de diagnóstico EGT
		}
		// ocorrência de timeout
		else if( (w_ret == WAINTING) && (r_egt > 8) )
		{
			w_ret = IDLE;			// liberaçao para um novo envio de msg
			egt_status = TimeOut;	// status de timeout. Sem comunicação com a unidade EGT
			atl = TRUE;				// autoriza a atualização das informações na tela de diagnóstico
			fast_blink();			// sinalização através do led_sts
		}
		
	return atl;	
}
//_________________________________________________________________________________________
void sr_options(char opt)
{
	switch(opt)
	{
		case 1:
				w_text(HALL, 3, 44, 4);		// 1ª opção: HALL
		break ;
		
		case 2:
				w_text(INDUTIVO, 3, 24, 8);	// 2ª opção: INDUTIVO
		break ;
		
		case 3:
				w_text(VOLTAR, 3, 34, 6);	// 3ª opção: VOLTAR
		break ;
	}
	
	page_ind(opt,3);
}
//_________________________________________________________________________________________
void sa_options(char opt)
{
	switch(opt)
	{
		case 1:
				w_text(ON, 3, 54, 2);			// 1ª opção: ON
		break ;
		
		case 2:
				w_text(OFF, 3, 49, 3);			// 2ª opção: OFF
		break ;
		
		case 3:
				w_text(CALIBRACAO, 3, 14, 10);	// 3ª opção: CALIBRACAO
		break ;
		
		case 4:
				w_text(VOLTAR, 3, 34, 6);		// 4ª opção: VOLTAR
		break ;
	}
	
	page_ind(opt,4);
}
//_________________________________________________________________________________________
void sa_calibracao()	// calibração função S.ACELERADOR
{
	unsigned int max_course;
	unsigned int min_course;
	unsigned int dif;
	char status = 0;
	
	_clrwdt();
	clear_dsp();		// limpa o display
	w_text("PISE NO", 		0, 24, 7);
	w_text("PEDAL ATE O", 	3, 8, 11);
	w_text("FIM DO CURSO", 	6, 0, 12);
	delay_ms(3500);		// tempo de exibição da mensagem
	_clrwdt();
	max_course = read_an(4);	// lê o valor de tensão com o pedal no fundo. pé no fundo!
	
	_clrwdt();
	clear_dsp();		// limpa o display
	w_text("REMOVA O PE", 	0, 9, 11);
	w_text("DO PEDAL", 		3, 24, 8);
	w_text("TOTALMENTE", 	6, 14, 10);
	delay_ms(3000);		// tempo de exibição da mensagem
	_clrwdt();
	clear_dsp();		// limpa o display
	
	min_course = read_an(4);	// lê o valor de tensão com o pedal em repouso. pé fora do pedal!
	
	// verifica se o fator é + ou -
	if(max_course > (min_course*2))	// a tensão do pedal no fundo deve ser > que pelo menos 2x -
	{								// a tensão referente ao pedal em repouso
		sa_fator = 	POSITIVO;
		dif = max_course - min_course;
		sa_xvalue = ((8*dif)/100) + min_course; 
		status = 1;
	}
	else if(min_course > (max_course*2))
	{
		sa_fator = NEGATIVO;
		dif = min_course - max_course;
		sa_xvalue = min_course - ((8*dif)/100);
		status = 1;
	}
	else
	{
		_clrwdt();
		w_text(ERRO, 0, 24, 5);	// erro ao calibrar
		w_text(" NA", 0, 74, 3);
		w_text(CALIBRACAO, 3, 14, 10);
		delay_ms(2000);		// tempo de exibição da mensagem
	}
	
	if(status == 1)
	{
		w_text(SUCESSO, 0, 14, 7);
		w_text(" NA", 0, 84, 3);
		w_text(CALIBRACAO, 3, 14, 10);
		_clrwdt();
		delay_ms(2000);		// tempo de exibição da mensagem
		clear_dsp();		// limpa o display
		_clrwdt();
		unsigned char dat;
		char check;
		char total_check = 0;
		check = grava_eeprom(0x06, sa_xvalue); // LSByte
		
		if(check == 1)	// 1 = ERRO
		{
			total_check++;
		}
		
		dat = (sa_xvalue>>8);
		check = grava_eeprom(0x05, dat); 		// MSByte
		
		if(check == 1)
		{
			total_check++;
		}
		
		check = grava_eeprom(0x04, sa_fator);
		
		if(check == 1)
		{
			total_check++;
		}
		
		_clrwdt();
		callback(total_check);
	}
	
	clear_dsp();		// limpa o display
	
}
//_________________________________________________________________________________________
unsigned int tt_calc(unsigned int t_1500, unsigned char sel)
{
	unsigned long t = 15 * t_1500;
	unsigned int v = 0;
	
	switch(sel)
	{
		case 7:
				t = t/7;
				v = t;
		break ;
		
		case 8:
				t = t/8;
				v = t;
		break ;
		
		case 9:
				t = t/9;
				v = t;
		break ;
		
		case 11:
				t = t/11;
				v = t;
		break ;
		
		case 13:
				t = t/13;
				v = t;
		break ;
		
		case 15:
				v = t_1500;
		break ;
	}	
	
	return v;
}
//_________________________________________________________________________________________
void rpm_calibracao()
{
	
	clear_dsp();
	
	if((pulso_sts == OK) && (periodo_val < 62000) && (periodo_val > 3))
	{
		w_text("ACELERE ATE", 	0, 9, 11);
		w_text("1500 RPM E", 	3, 14, 10);
		w_text("MANTENHA!", 	6, 19, 9);
		_clrwdt();
		delay_ms(3500);		// tempo de exibição da mensagem
		clear_dsp();
		w_text("PRESSIONE", 	0, 19, 9);	
		w_text("O BOTAO", 		3, 29, 7);
		w_text("AO ATINGIR", 	6, 14, 10);
		_clrwdt();
		delay_ms(2000);		// tempo de exibição da mensagem
		click = IDLE;
		time_bt = 0;
		
		// aguarda um click. O timeout é 5
		while((click == IDLE) && (time_bt <39))
		{
			_clrwdt();
		}
		
		if(click != IDLE)
		{								
			time_bt = 0;
			unsigned char m_ok = FALSE;
			unsigned char i = 0;
			
			// aguarda a media ser calculada ou timeout
			while((m_ok == FALSE) && (time_bt<23))
			{
				_clrwdt();
				if(pval_saved == TRUE)	// periodo valido foi salvo?
				{
					pval_saved = FALSE;
					m_ok = check_periodo(i);	// salva o periodo no buffer. Caso já tenha sido salvo os 4 periodos a média sera feita
					
					if(i<3)
					{
						i++;
					}
					else
					{
						i=0;
					}
				}	
			}
			
			if((time_bt < 23) && ( m_ok == TRUE))
			{
				//novo t_target
				t_target = tt_calc(periodo_val, rpm_sel);
					
				// salva o novo rpm 1500
				unsigned char d = periodo_val;
					
				char check;						// retorno da gravação da eeprom
				char total_check = 0;
				check = grava_eeprom(0x09, d);	// LSByte
				if(check == 1)
				{
					total_check++;
				}
					
				d = (periodo_val>>8);
				check = grava_eeprom(0x08, d);	// MSByte
				if(check == 1)
				{
					total_check++;	
				}
					
				callback(total_check);
			}
			else								// ocorreu erro na comparação dos periodos
			{
				clear_dsp();
				w_text("ERRO NA", 0, 29, 7);
				w_text("CALIBRACAO", 3, 14, 10);
				_clrwdt();
				delay_ms(3500);
			}
			
		}
		else
		{
			clear_dsp();
			w_text(ERRO, 0, 39, 5);
			w_text("BOTAO N. FOI", 3, 0, 12);
			w_text("PRESSIONADO", 6, 9, 11);
			_clrwdt();
			delay_ms(3500);
		}
	}
	else
	{
		w_text(ERRO, 0, 39, 5);
		w_text("SEM SINAL", 3, 19, 9);
		w_text("DE ROTACAO", 6, 14, 10);
		_clrwdt();
		delay_ms(3500);		// tempo de exibição da mensagem
		
	}
	
	clear_dsp();	
}
//_________________________________________________________________________________________
void rpm_options(char opt)
{
	switch(opt)
	{
		
		case 7:
				w_text(R900, 3, 29, 3);
				w_text(RPM, 3, 69, 3);
				page_ind(1,7);
		break ;
		
		case 9:
				w_text(R1000, 3, 29, 4);
				w_text(RPM, 3, 74, 3);
				page_ind(2,7);
		break ;
		
		case 11:
				w_text(R1100, 3, 24, 4);
				w_text(RPM, 3, 74, 3);
				page_ind(3,7);
		break ;
		
		case 13:
				w_text(R1300, 3, 24, 4);
				w_text(RPM, 3, 74, 3);
				page_ind(4,7);
		break ;
		
		case 15:
				w_text(R1500, 3, 24, 4);
				w_text(RPM, 3, 74, 3);
				page_ind(5,7);
		break ;
		
		case 17:
				w_text(CALIBRACAO, 3, 14, 10);	// 6ª opção: CALIBRACAO
				page_ind(6,7);
		break ;
		
		case 19:
				w_text(VOLTAR, 3, 34, 6);		// 7ª opção: VOLTAR
				page_ind(7,7);
		break ;
	}
	
}
//_________________________________________________________________________________________
char out_menu()
{
	last_atl = 3;
	clear_dsp();
	atl_mscreen(ALL);			// atualiza o display
	return 0;
}
//_________________________________________________________________________________________
void menu()
{
	char menu_pos = 1;	 
	clear_dsp();			// limpa o display
	page_ind(1,5);
	w_text(RPM, 3, 49, 3);	// primeira opção: RPM
	
	// main while do menu
	while(menu_pos>0)
	{
		_clrwdt();
		if(timeout_menu > dez_sec)	// 10s
		{
			menu_pos = out_menu();
		}
			
		char options = rpm_sel;
		
		if(click == CURTO)
		{
			click = IDLE;
			menu_pos++;						// proxima opção
			clear_linha(3);					// limpa a linha inteira
			page_ind(2,5);
			w_text(TPS, 3, 49, 3);			// SEGUNDA opção: TPS
		}
		else if(click == LONGO)
		{
			click = IDLE;
			menu_pos += 10;	
			clear_linha(3);	
			rpm_options(options);
		}
		
		while(menu_pos == 11)
		{
				_clrwdt();
			if(timeout_menu > dez_sec)
			{
				menu_pos = out_menu();
			}
			
			if(click == CURTO)
			{
				if(options == 19)		// 19 = SAIR
				{
					options = 7;		// volta p/ a 1ª opção da lista
				}
				else
				{
					options += 2;
				}
				
				click = IDLE;
				clear_linha(3);
				rpm_options(options);
			}
			else if(click == LONGO)
			{
				clear_linha(3);
				
				if(options <= 15)
				{
					char check;
					rpm_sel = options;
					check = grava_eeprom(0x0A, rpm_sel);
					callback(check);
				}
				else if(options == 17)	// CALIBRACAO
				{
					rpm_calibracao();	// chama a função que faz a calibração com base nos 1500 RPM
				}
				
				click = IDLE;
				menu_pos -= 10;
				w_text(RPM, 3, 49, 3);	// primeira opção: RPM
				page_ind(1,5);
			}
		}
		
		// Sensor TPS
		while(menu_pos == 2)
		{
			_clrwdt();
			if(timeout_menu > dez_sec)
			{
				menu_pos = out_menu();
			}
			
			char options = sa_ctrl;
			
			if(click == CURTO)
			{
				click = IDLE;
				menu_pos++;					// proxima opção
				clear_linha(3);				// limpa a linha inteira
				page_ind(3,5);
				w_text(EGT, 3, 49, 3);		// TERCEIRA opção: EGT
			}
			else if(click == LONGO)
			{
				click = IDLE;
				clear_linha(3);				// limpa a linha inteira
				menu_pos += 19;
				sa_options(options);
			}	
			
			while(menu_pos == 21)
			{
				_clrwdt();
				if(timeout_menu > dez_sec)	// 10s
				{
					menu_pos = out_menu();
				}
				
				if(click == CURTO)
				{
					
					if(options == 4)		// a 4ª opção é SAIR
					{
						options = 1;		// volta p/ a primeira opção	
					}
					else
					{
						options++;			// proxima opçao	
					}
					
					click = IDLE;
					clear_linha(3);			// limpa a linha inteira
					sa_options(options);
				}
				else if(click == LONGO)
				{
					clear_linha(3);			// limpa a linha inteira
					
					if(options <= 2)		// 1 = On, 2 = Off
					{
						sa_ctrl = options;
						char check;
						check = grava_eeprom(0x03, sa_ctrl);
						callback(check);
					}
					else if(options == 3)	// calibração
					{
						sa_calibracao();	// chama a rotina de calibração
						timeout_menu = 0;	// reset no timeout_menu
					}
					
					click = IDLE;
					menu_pos -= 19;
					w_text(TPS, 3, 49, 3);	// SEGUNDA opção: S.ACELERADOR
					page_ind(2,5);
				}
			}
		}
		
		// EGT
		while(menu_pos == 3)
		{
			_clrwdt();
			if(timeout_menu > dez_sec)
			{
				menu_pos = out_menu();
			}
			
			unsigned char options = temp;
			
			if(click == CURTO)
			{
				click = IDLE;
				menu_pos++;					// proxima opção
				clear_linha(3);				// limpa a linha inteira
				page_ind(4,5);
				w_text(ROTACAO, 3, 19, 9);	// QUARTA opção: S.ROTAÇÃO
			}	
			
			if(click == LONGO)
			{
				click = IDLE;
				menu_pos += 28;
				clear_linha(3);				// limpa a linha inteira
				egt_options(temp);			// vai mostrar qual a atual temperatura que está selecionada
			}
			
			while(menu_pos == 31)
			{
				_clrwdt();
				if(timeout_menu > dez_sec)	// 10s
				{
					menu_pos = out_menu();
				}
				
				if(click == CURTO)
				{
					click = IDLE;
					if(options == 7)		// a 7ª opção é SAIR
					{
						options = 1;		// volta p/ a primeira opção	
					}
					else
					{
						options++;			// proxima opçao	
					}
					clear_linha(3);			// limpa a linha inteira
					egt_options(options);	
				}
				else if(click == LONGO)
				{
					clear_linha(3);			// limpa a linha inteira
					
					if(options == 6)		// 6 = DIAGNOSTICO
					{
						char atl;
						unsigned long digitos;
						char digts[12] = "-TEMP: ";
							
						clear_dsp();				// limpa o display
						click = IDLE;
							
						// qualquer click fecha essa tela
						while(click == IDLE)
						{
							_clrwdt();
							main_scan();			// monitorara as condições para GNV ON ou OFF
							atl  = egt_com();		// a autorização para atualização da tela é gerada com o recebimento dos 6 bytes ou TimeOut
								
							if(atl == TRUE)			// autorização para atualizar
							{	
								atl = FALSE;
									
								if((egt_status == Ok) && (TermoPar_status == 0))
								{
									digitos = bcd_convert(egt_temp);	
									digts[7] = (digitos>>24);			// milhar
									digts[8] = (digitos>>16);			// centena
									digts[9] = (digitos>>8);			// dezena
									digts[10] = digitos;				// unidade
									digts[11] = 91;						// °C
										
									w_text(digts, 			1, 0, 12);	// temperatura
									w_text("-COM: OK  ", 	3, 0, 10);	// comunicação com a unidade EGT
									w_text("-T.PAR: OK  ", 	5, 0, 12);	// sonda (termopar) conectada/desconectada
								}
								else if((egt_status == Erro) && (TermoPar_status == 1))
								{
									w_text("-TEMP: ****[", 1, 0, 12);
									w_text("-COM: OK  ",   3, 0, 10);
									w_text("-T.PAR: ERRO", 5, 0, 12);	
								}
								else if(egt_status == TimeOut)
								{
									w_text("-TEMP: ****[", 1, 0, 12);
									w_text("-COM: ERRO",   3, 0, 10);
									w_text("-T.PAR: ****", 5, 0, 12);
								}	
							}
						}
							
						clear_dsp();			// limpa o display
						egt_options(options);	// informação do nivel anterior
						click = IDLE;
					}
					
					if(options <= 5)		// 1...5 temperatura
					{	
						char check;
						temp = options;		// atualiza a variavel que é usada para verificar a temperatura limite
						check = grava_eeprom(0x02, temp);	// salva a alteração na eeprom	
						callback(check);
						
						atl_temps(temp);
					}
					
					if((options <= 5) || (options == 7))	// ao retornar da tela de diagnostico apenas retrocede a ultima posição
					{
						click = IDLE;
						menu_pos -= 28;			// volta p/ o nivel anterior
						w_text(EGT, 3, 49, 3);	// TERCEIRA opção: EGT
						page_ind(3,5);
					}
				}
			}
		}
		
		// S.ROTAÇÃO
		while(menu_pos == 4)
		{
			_clrwdt();
			if(timeout_menu > dez_sec)
			{
				menu_pos = out_menu();
			}
			
			char options = s_type;;
			
			if(click == CURTO)
			{
				click = IDLE;
				menu_pos++;					// proxima opção
				clear_linha(3);				// limpa a linha inteira
				page_ind(5,5);
				w_text(SAIR, 3, 44, 4);		// quinta opção: SAIR
			}
			else if(click == LONGO)
			{
				click = IDLE;
				clear_linha(3);				// limpa a linha inteira
				menu_pos += 37;				// avança um nivel dentro do menu
				sr_options(options);		// vai mostrar qual o tipo de sinal está selecionado			
			}	
			
			while(menu_pos == 41)
			{
				_clrwdt();
				if(timeout_menu > dez_sec)	// 10s
				{
					menu_pos = out_menu();
				}
				
				if(click == CURTO)
				{
					click = IDLE;
					if(options == 3)		// a 3ª opção é SAIR
					{
						options = 1;		// volta p/ a primeira opção	
					}
					else
					{
						options++;			// proxima opçao	
					}
					clear_linha(3);			// limpa a linha inteira
					sr_options(options);
				}
				else if(click == LONGO)
				{
					clear_linha(3);			// limpa a linha inteira
					
					if(options <= 2)		// 3=SAIR
					{
						char check;	
						s_type = options;	// atualiza a variavel que indica qual o tipo de sinal está sendo utilizado
							
						if(s_type == 1)
						{
							en_hall = 1, en_ind = 0;	// enable read hall signal
						}
						else
						{
							en_hall = 0, en_ind = 1;	// enabl read inductive signal AC
						}
							
						check = grava_eeprom(0x01, s_type);	// salva a alteração na eeprom	
						callback(check);
					}
					
					click = IDLE;
					menu_pos -= 37;				// volta p/ o nivel anterior
					w_text(ROTACAO, 3, 19, 9);	// 4ª opção: S.ROTAÇÃO	
					page_ind(4,5);
				}
			}
		}
		
		// SAIR
		while(menu_pos == 5)
		{
			_clrwdt();
			if(timeout_menu > dez_sec)
			{
				menu_pos = out_menu();
			}
			
			if(click == CURTO)
			{
				click = IDLE;
				menu_pos = 1;				// proxima opção
				clear_linha(3);				// limpa a linha inteira
				page_ind(1,5);
				w_text(RPM, 3, 49, 3);		// primeira opção: RPM
				page_ind(1,5);
			}
			
			if(click == LONGO)
			{
				click = IDLE;
				menu_pos = 0;				// sai do menu
				clear_dsp();
				last_atl=3;
				atl_mscreen(ALL);			// atualiza o display	
			}
		}
	}
}
//_________________________________________________________________________________________
void setup()
{
	//_wdtc = 0b10101101;		// watchdog timer OFF - 2¹6 = 64536/32KHz = timeout de ~2s
	_wdtc = 0b01010110;			// watchdog timer OFF - 2¹8 = 64536/32KHz = timeout de ~8s
	
	// definições dos IOs
	_pac = 	0b11111111;
	_pbc = 	0b0000111;
	_pcc = 	0b1001000;
	_pdc = 	0b1111;
	_papu = 0b10001110;			// resistor pullup interno para inputs
	_pbpu = 0b0000011;
	_pcpu = 0b1001000;
	_pdpu = 0b1111;
	_cpc = 	0b00000000;			// comparador desativado
	_ace4 = 1, _ace5 = 1;		// PA5/AN4 ACEL. PA6/AN5 MANO.
	_adrfs = 1;					// formato dos dados na conversão AD. SADOH=D[11:8]; SADOL=D[7:0]
	_sadc1 = 0b00000100;		// External signal - External analog channel input, ANn. clock fsys/16
	_sadc2 = 0b00000000;		// Internal A/D converter power supply voltage VDD
	
	// low voltage detector
	_lvden = 1;					// ativa o monitoramento <= 4,0V
	_vlvd2=1, _vlvd1=1, _vlvd2=1;
		
	// config. interrrupções
	_int1e = 1;					// ativa a int. externa 1. Flag _INT1F. Int. vector: 0x20
	_int1s1 = 1, _int1s0 = 1;	// interrupção ocorre na borda de subida e descida
	_tb1e = 1;					// interrupção do time base 1 ativada
	
	
	// config. timer			*** frequancia maxima do sinal para que o programa consiga ser executado é 7KHz ***
	_stmc0 = 0b01001000;		// tm em run, clock fsub (32KHz), tm on. Max: 2048ms (0,5Hz) - Min: 62,5uS (16KHz)
	_stmc1 = 0b01010000;		// capture mode, falling edge, STM counter clear condition = Comparator P match 
	_stmrp = 0;					// overflow em 65536
	_t0cp = 1;					// STP pin enable
	
	// config. time base
	_tbc = 0b10000100;			// time base zero: on, clock: ftbc, time-out periodo tb1: 2¹²(4096), tb0: 2¹²(4096)
	
	// carrega dados da eeprom
	modo  = 	le_eeprom(0x00);	// Diesel ou Diesel/GNV 
	s_type = 	le_eeprom(0x01);	// Sinal Hall(1) ou Indutivo

	if(s_type == 1)
	{
		en_hall = 1, en_ind = 0;	// enable read hall signal
	}
	else
	{
		en_hall = 0, en_ind = 1;	// enable read inductive signal AC
	}
	
	temp = 	le_eeprom(0x02);		// temperatura limite EGT (menu)
	temp_base = le_eeprom(0x10);	// temperatura base para definiçoes das opções de temperatura em: Menu >> EGT
	temp_base = (temp_base<<8);		// MSByte
	temp_base += le_eeprom(0x11);	// LSByte
	
	atl_temps(temp);				// seleciona a temperatura limite com base na opção (temp 1...4)
	
	sa_ctrl = 	le_eeprom(0x03);	// indica se a função TPS está ON ou OFF
	sa_fator =  le_eeprom(0x04);	// fator da função S.ACELERADOR
	sa_xvalue = le_eeprom(0x05);	// MSByte do valor de 16 bits
	sa_xvalue = (sa_xvalue<<8);		// desloca o valor para a parte alta dos 16bits da variavel int.
	sa_xvalue += le_eeprom(0x06);	// LSByte
	unsigned int t_1500;
	t_1500 = le_eeprom(0x08);		// MSByte do valor de 16 bits
	t_1500 = (t_1500<<8);
	t_1500 += le_eeprom(0x09);		// LSByte
	rpm_sel = le_eeprom(0x0a);		// rpm selecionada (700, 900, 1100, 1300 ou 1500)
	t_target = tt_calc(t_1500, rpm_sel);
	t_800 = tt_calc(t_1500, 8);		// calcula qual é o periodo para 800 RPM
	
	// inicialização de variaveis e outputs
	gnv_ctrl = 0;					// GNV off
	led_sts = LED_OFF;				// led off
	click = IDLE;					// sem click
	periodo = 0xffff;				// timeout forçado
	periodo_val = periodo;
	pval_saved = FALSE;
	w_ret = IDLE;
	error_rx_flag = 0;
	pulso_sts = NONE;				// sem sinal pulsante valido
	egt_temp = 0;
	last_atl = 0xff;
	poweron = TRUE;
	tout_erro = TRUE;
	indice = 0;
	time_bt = 0;
	
	// incialização do display e criação da tela principal
	delay_ms(700);					// tempo para estabilização da tensão antes de inicializar o display
	
	// Low voltage reset
	_lvrc = 0b10011001;				// reset ao detectar tensão < 3,15V
	
	// caso seja detectada tensão abaixo de 3,6V, aguarda a estabilização da mesma
	// timeout de 1,4s
	while((_lvdo == 1)&&(time_bt<11))						
	{
		GCC_NOP();
	}
	
	int_ssd();						// inicializa o display oled
	clear_dsp();					// limpa o display
	
	// incialização da UART. 8N1
	uart_init(BAUD_38400);
	
	// com o botão da interface pressionado no poweron 
	if(bt == PRESS)
	{
		modo_config = TRUE;
		atl_mscreen(MCON);	// mensagem do modo_config
	}
	else
	{	print_logo();
		delay_ms(2000);
		_clrwdt();
		clear_dsp();
		w_text(INFO,0,0,11);
		w_text(DATA,2,0,11);
		delay_ms(1500);
		_clrwdt();
		clear_dsp();
		modo_config = FALSE;
		atl_mscreen(ALL);
		// ativa a int. M Funct.0, int. dos camparadores A e P do stm (timer que mede t do pulso)
		_mf0e = 1, _stmae = 1, _stmpe = 1;
	}
			
	// config. interrrupções
	_int1e = 1;					// ativa a int. externa 1 (bt). Flag _INT1F. Int. vector: 0x20	
}
// fim das funções_________________________________________________________________________

// main____________________________________________________________________________________
void main()
{
	setup();
	
	// main while
	while(1)
	{	
		_clrwdt();
		
		if(modo_config == TRUE)
		{
			click = IDLE;	
			edit_tb();		// função p/ edição
			modo_config = FALSE;
			atl_mscreen(ALL);
			// ativa a int. M Funct.0, int. dos camparadores A e P do stm (timer que mede t do pulso)
			_mf0e = 1, _stmae = 1, _stmpe = 1;
		}					
		
		
		// verifica se ocrreu um click curto para que seja realizada a troca de modo_______
		if(click == CURTO)
		{
			// alteração do modo de operação
			if(modo == Diesel)
			{
				modo = DieselGnv;	
			}
			else
			{
				modo = Diesel;
			}
			
			grava_eeprom(0x00, modo);
			
			click = IDLE;		// reset da variavel
		}
		else if(click == LONGO) // ativação do menu somente no modo Diesel
		{
			click = IDLE;		// limpa o status do click
			menu();				// abre o menu
		}
		
		// atulização periódia do display_____________________________________________________
		if(go_atl == 0)
		{	
			if(ack_sts == NACK)			// NACK = Sem reposta do display
			{
				led_sts = LED_ON;
				delay_ms(50);
				int_ssd();				// reinicializa o display oled
				clear_dsp();			// limpa o display
				last_atl=3;
				led_sts = LED_OFF;
				cnt = TRUE;
			}
			else if(cnt == TRUE)
			{
				cnt = FALSE;
				int_ssd();				// reinicializa o display oled
				clear_dsp();			// limpa o display	
				last_atl=3;
			}
			atl_mscreen(ALL);
		}
		
		egt_com();		// comunicação com a unidade EGT___________________________________

		main_scan();	// scan para on/off do GNV_________________________________________
	}
}
// fim do main_____________________________________________________________________________

// Vetores interrupções____________________________________________________________________
void int_ext1()			// interrupção externa 1. A flag é auto-reset
{
	// detecta qual a borda. subida ou descida
	if(bt == PRESS)
	{
		time_bt = 0;	// zera a contagem de tempo
		start_tbt = RUN;// sinaliza que a contagem de tempo está em curso
	}
	else				// borda de descida
	{
		// o tempo que o botão ficou pressionado é menor que 3s e maior que 256ms ???
		if((time_bt<23) && (time_bt>=1) && (start_tbt == RUN))
		{
			click = CURTO;
			timeout_menu = 0;	// click do botão reseta a contagem de tempo (inatividade no menu)
			start_tbt = STOP;
		}
	}
}
//_________________________________________________________________________________________
void int_mf0()			// interrupções dos comparadores A e P do stm
{
	if(_stmaf == 1)
	{
		_stmaf = 0;		// limpa o flag da int.	
		_ston = 0;		// para a contagem de tempo
		
		// primeira borda de descida após a ocorrência de timout (ausência de pulso). Medição é descartada.
		// da segunda borda de descida em diante as medições são feitas
		if(tout_erro == FALSE)
		{
			// se == NONE significa que houve um timeout (ausencia de pulso) antes de desta borda de descida
			// capturada. Sendo assim, só começa a reliazar a medição do (t) do pulso na proxima borda de descida
			periodo = _stmah;			// salva o periodo (número de 16 bits)
			periodo = (periodo<<8);
			periodo += _stmal;
			_ston = 1;					// reinicia a contagem
			
			// Range - 7KHz ~ 0,51Hz
			// o periodo deve ser maior que 142us. 7KHz é a frequencia maxima aceitavel
			if((periodo > 3) && (periodo < 62000))		
			{
				pulso_sts = OK;			// sinaliza que há um sinal pulsante valido
				periodo_val = periodo;
				pval_saved = TRUE;
			}
			else
			{
				if(poweron == TRUE)		// somente no primeiro acionamento
				{
					pulso_sts = NONE;	// periodo (pulso) invalido
					tout_erro = TRUE;	// indica a ocorrência de timeout. Na 1ª borda de descida a medição do periodo será descartada
				}
				pval_saved = FALSE;
			}
		}
		else
		{
			_ston = 1;					// reinicia a contagem
			tout_erro = FALSE;			// na próxima borda de descida será feita a medição do periodo 
		}
	}
	
	// int. do comparador P. Ausência de pulso
	if(_stmpf == 1)
	{
		_stmpf = 0;					// limpa o flag da int.	
		pulso_sts = NONE;
		periodo = 0xffff;
		periodo_val = periodo;
		tout_erro = TRUE;			// indica a ocorrência de timeout. Na 1ª borda de descida a medição do periodo será descartada
		indice = 0;	
	}
}
//_________________________________________________________________________________________
void int_tb1()			// base de tempo de 128ms. A flag é auto-reset
{
	time_bt++;			// contagem maxima é 255 (32,3s)
	timeout_menu++;		// incrementa a contagem de tempo (inatividade no menu)
	
	// verifica se a contagem de tempo de click do botão está em curso
	if((start_tbt == RUN) && (time_bt>20))
	{
		click = LONGO;
		timeout_menu = 0;	// click do botão reseta a contagem de tempo (inatividade no menu)
		start_tbt = STOP;
	}
	
	// tempo para atualização do display - taxa de atualização: 2,6Hz 384ms
	if(go_atl > 2)	
	{
		go_atl = 0;
	}
	else
	{
		go_atl++;	
	}
	
	// tempo para leitura da sonda EGT - taxa de leitura: 1s (1Hz)
	if(r_egt > 11)
	{
		r_egt = 1;
	}
	else
	{
		r_egt++;	
	}
}
//_________________________________________________________________________________________
void UART_ISR()			// recepção UART
{
	UART_ISR_Value 	= 0;		// clear receive value				
	
	if((_nf) || (_ferr) || (_oerr))
	{
		error_rx_flag = 1;
		_nf = 0;				// limpa o flag - noise error flag
		_ferr = 0;				// limpa o flag - Framing error flag	
		_oerr = 0;				// limpa o flag - Overrun error flag
	}
	
	_rxif = 0;					// data register has available data
	UART_ISR_Value = _txr_rxr;	// move o byte para a variavel de usuario
	
	// verifica se está aguardando resposta
	if((w_ret == WAINTING) && (ctrl_byte<=5))
	{
		buffer[ctrl_byte] = UART_ISR_Value;
		ctrl_byte++;
	}
}
// fim dos vetores de interrupções_________________________________________________________
