// leitura de entrada analogica. retorna um inteiro
#include "HT66F0195.h"

unsigned int read_an(char channel)	// AN3 p/ leitura do sensor do pedal acelerador.
{
	unsigned int value = 0;
	unsigned int data_ad = 0;		// PA5/AN4 ACEL. PA6/AN5 MANO.
	char cnt = 0;					// contagem de 4 leituras para fazer a m�dia do valor
	
	while(cnt<8)
	{
	
		// sele��o do canal AN que ser� lido
		if(channel == 4)
		{
			_sacs3 = 0, _sacs2 = 1, _sacs1 = 0, _sacs0 = 0;
		}
		else if(channel == 5)
		{
			_sacs3 = 0, _sacs2 = 1, _sacs1 = 0, _sacs0 = 1;
		}
		
		// ativa o conversor AD
		_adcen = 1; 			
		
		//condi��o de start para convers�o
		_start=0;				
		_start=1;
		_start=0;
		
		// enquanto == 1 a convers�o est� em curso
		while(_adbz == 1)		
		{
			GCC_NOP();	
		}
		
		// ao final da convers�o move os valores MSB e LSB p/ a variavel inteira
		if(_adbz == 0)
		{
			data_ad = _sadoh;		// move para para a var. 16 bits o valor de 8 bits MSB do AD
			data_ad = (data_ad<<8); // desloca o valor 8 bits para esquerda. vai ficar nos 8 bits MSB da var. 16 bits
			data_ad += _sadol;		// concatena na var. 16 bits os 8bits LSB do AD
		}
		
		cnt++;						// incrementa o contador
		value += data_ad;
	}
	
	value = value/8;				// m�dia das 8 leituras
	
	return value;					// retorna o valor lido. 12 bits (0000...4095)
}