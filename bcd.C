
unsigned long bcd_convert(unsigned int value)	// converte um valor int(16) em 4 digitos (bytes) BCD
{
	unsigned long digitos = 0;			// ASCII 0000. (byte4) 31...24; (byte3) 23...16; (byte2) 15...8; (byte1) 7...0
	unsigned char res;
	unsigned char milhar  = 0x30;		// milhar é ascII "0"
	unsigned char centena = 0x30;
	unsigned char dezena  = 0x30;
	unsigned char unidade = 0x30;
	
	
	if(value > 1000)
	{
		res = value/1000;
		
		if(res < 10)
		{
			milhar += res;
		}
		
		while(value > 1000)
		{
			value -= 1000;	
		}
	}
	
	if(value >= 100)
	{
		res = value/100;
		
		if(res < 10)
		{
			centena += res;		// centena
		}
		
		while(value > 100)
		{
			value -= 100;	
		}
	}
	
	if(value >= 10)
	{
		res = value/10;
		
		if(res < 10)
		{
			dezena += res;		// dezena
		}
		
		while(value > 10)
		{
			value -= 10;	
		}	
		
		if(value < 10)
		{
			unidade += value;	// unidade
		}
		
	}
	else if(value < 10)
	{
		unidade += value;	// unidade
	}
	
	digitos = milhar;	
	digitos = (digitos<<8);
	digitos += centena;
	digitos = (digitos<<8);
	digitos += dezena;
	digitos = (digitos<<8);
	digitos += unidade;
	
	return digitos;
}