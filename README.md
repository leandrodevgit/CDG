# CDG
Central Diesel GNV

Este é o firmware desenvolvido para a central de gerenciamento Diesel GNV que faz o controle do uso dos combustíveis diesel e GNV em motores diesel
linha pesada de veículos.

Abaixo as principais características da CDG (Central Diesel GNV):
 
Interface gráfica em display OLED 0,96" que permite viualisação de parâmetros do sistema como:
 
1 - Modo de operação em uso (DIESEL / DIESEL GNV);

2 - Indicação de sinal de rotação válido;

3 - Status do redutor de pressão (ON/OFF);

4 - Status da função TPS (Ativada / desativada);

5 - Status da unidade EGT (Conectada, desconectada, erro no termopar e overheat);

6 - Indicação do nível de GNV do sistema.  

Funções de auto diagnóstico de falhas, calibração e monitoramento de parâmetros do sistema;

Barramento CAN exclusivo para comunicação com a unidade EGT;    

A unidade EGT é um hardware auxiliar da CDG que tem como função medir a temperatura dos gases EGT
do motor diesel através de sonda termopar tipo K e informar essa temperatura à CDG. A sua comunicação é feita através do barramento CAN da CDG;

Corte do combustível GNV em caso de super aquecimento do motor (informação proveniente da unidade EGT);

Corte do combustível GNV em baixa rotação. A CDG lê o sinal TPS (sensor de posição do pedal do acelerador) e desta forma permite ou não o uso do GNV pelo motor do veículo.

Mais detalhes podem ser consultados no manual do equipamento.
