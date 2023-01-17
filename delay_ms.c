#include "HT66F0195.h"

void delay_ms(unsigned int i)   	//	delay base 1m
{
  	while(i!=0)	  			//	enquanto a var. for diferente de zero...
    {
	    GCC_DELAY(4000);	//	tempo base de 1ms, ciclo de maquina 0,25uS	
	    i--;	          	//	decrementa 1 da var.
    }
}