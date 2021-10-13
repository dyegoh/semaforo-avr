#define F_CPU 16000000UL
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nokia5110.h"
#include <avr/eeprom.h>

#define LED 1


//declaraão das variáveis para o sensor de temperatura
float a = 0.001129241;
float b = 0.0002341077;
float c = 0.00000008775468;
float T_Kelvin;
float Rntc;
float Vntc;
float Celsius;



//Novos tipos
typedef enum enum_parametros {Sel_modo, Sel_tempo_verde, Sel_tempo_vermelho, Sel_tempo_amarelo, Size_enum_parametros} enum_parametros;
typedef struct stc_semaforo // STRUCT para armazenar os tempos dos estados do semaforo
{
	uint8_t modo;
	uint16_t tempo_verde_ms;
	uint16_t tempo_vermelho_ms;
	uint16_t tempo_amarelo_ms; 
	uint16_t carros_por_min;
	uint16_t sensor_lux; 
	uint16_t sensor_temp; 
} stc_semaforo; //Define o nome do novo tipo criado

//Variáveis globais
stc_semaforo semaforo = {.modo=0, .tempo_verde_ms=5000, .tempo_vermelho_ms=3000, .tempo_amarelo_ms=1000, .carros_por_min=0, .sensor_lux=0, .sensor_temp=0}; //Inicializa as variáveis
enum_parametros selecao_parametro = Sel_modo;
uint8_t flag_5000ms = 0, flag_500ms = 0; // flags para disparar tarefas cíclicas
uint32_t tempo_ms = 0;   // contador geral de tempo decorrido em ms
uint16_t num_carros = 0; // contador do número de carros que passou pelo semaforo em 5seg

//Protótipos
void anima_semaforo(stc_semaforo semaforo, uint32_t tempo_ms);
void anima_LCD(stc_semaforo semaforo);
void estima_carros_por_min(uint8_t *flag_disparo);
void leituraADC_sensor_LUX(uint8_t *flag_disparo);
void leituraADC_sensor_TEMP(uint8_t *flag_disparo);

//Tratamento de interrupções
ISR(TIMER0_COMPA_vect) //interrupção DO tc0 A CADA 1MS = (64*(249+1)/16mHZ
{
	tempo_ms++;
	//PORTD ^= 0b01000000;
	if((tempo_ms % 5000)==0)//true a cada 5000ms
		flag_5000ms = 1;
	if((tempo_ms % 500)==0)//true a cada 5000ms
		flag_500ms = 1;
}

ISR(INT0_vect) //interrupção pino D2
{
	//anima_LCD(semaforo);
}

/*ISR(ADC_vect)
{
	leitura_ADC = ADC;
}*/

ISR(INT1_vect) //interrupção externa 1, PIN D3, Sensor de presença de carros
{
	num_carros++;
}

ISR(PCINT2_vect){//interupção 2 por mudança de pino na porta D (PD4, PD5, PD6) - BOTÃO "+", "-", E "S"
	
	if((PIND&0b00010000)==0)// BOTÃO "+" PD4
	{
		
		switch (selecao_parametro)
		{
			case Sel_modo:
				semaforo.modo = !semaforo.modo;
				break;
			case Sel_tempo_verde:
				if(semaforo.tempo_verde_ms <= 8000)
					semaforo.tempo_verde_ms += 1000;
				break;
			case Sel_tempo_vermelho:
				if(semaforo.tempo_vermelho_ms <= 8000)
					semaforo.tempo_vermelho_ms += 1000;
				break;
			case Sel_tempo_amarelo:
				if(semaforo.tempo_amarelo_ms <= 8000)
					semaforo.tempo_amarelo_ms += 1000;
				break;			
		}
	}
			
	if((PIND&0b00100000)==0)// BOTÃO "-" PD5
	{
		
		switch(selecao_parametro){
			
			case Sel_modo:
				semaforo.modo = !semaforo.modo;
				break;
			case Sel_tempo_verde:
				if(semaforo.tempo_verde_ms >= 2000)
					semaforo.tempo_verde_ms -= 1000;
				break;
			case Sel_tempo_vermelho:
				if(semaforo.tempo_vermelho_ms >= 2000)
					semaforo.tempo_vermelho_ms -= 1000;
				break;
			case Sel_tempo_amarelo:
				if(semaforo.tempo_amarelo_ms >= 2000)
					semaforo.tempo_amarelo_ms -= 1000;
				break;
		}
	}

	if((PIND&0b01000000)==0)// BOTÃO "S" PD6
	{
		if(selecao_parametro < (Size_enum_parametros-1))
			selecao_parametro++;
		else
			selecao_parametro = Sel_modo;
	}
	
	anima_LCD(semaforo);
	
}

void anima_semaforo(stc_semaforo semaforo, uint32_t tempo_ms )
{
	const uint16_t estados[9]={0b000001111, 0b000000111, 0b000000011, 0b000000001, 0b100000000, 0b011110000, 0b001110000, 0b000110000, 0b000010000};
	static int8_t i=0;
	static uint32_t tempo_anterior_ms=0;
	
	PORTB = estados[i] & 0b011111111;
	if(estados[i] & 0b100000000)
		PORTD |= 0b10000000;
	else
		PORTD &= 0b01111111;
	
	if(i<=3)
	{
		//_delay_ms(Tempo_verde_ms/4);
		if((tempo_ms - tempo_anterior_ms)>= (semaforo.tempo_verde_ms/4))
		{
			i++;
			tempo_anterior_ms += (semaforo.tempo_verde_ms/4);
		}
	}
	else
	{
		if(i<=4)
		{
			
			if((tempo_ms - tempo_anterior_ms) >= (semaforo.tempo_amarelo_ms))
			{
				i++;
				tempo_anterior_ms += (semaforo.tempo_amarelo_ms);
			}
		}
		else
		{
			if(i<=8)
			{
				
				if((tempo_ms - tempo_anterior_ms) >= (semaforo.tempo_vermelho_ms/4))
				{
					i++;
					tempo_anterior_ms +=(semaforo.tempo_vermelho_ms/4);
				}
			}
			else
			{				
				i=0;
				tempo_anterior_ms = tempo_ms;
			}
		}
	}
}


void anima_LCD(stc_semaforo semaforo)
{
	
	unsigned char modo_string[2];
	unsigned char tempo_verde_s_string[2];
	unsigned char tempo_vermelho_s_string[2];
	unsigned char tempo_amarelo_s_string[2];
	unsigned char carros_por_min_string[4];
	unsigned char sensor_lux_string[5];
	unsigned char sensor_temp_string[5];
	modo_string[0] = (semaforo.modo) ? 'A' : 'M'; modo_string[1]='\0';
	sprintf(tempo_verde_s_string, "%u", semaforo.tempo_verde_ms/1000);
	sprintf(tempo_vermelho_s_string, "%u", semaforo.tempo_vermelho_ms/1000);
	sprintf(tempo_amarelo_s_string, "%u", semaforo.tempo_amarelo_ms/1000);
	sprintf(carros_por_min_string, "%u", semaforo.carros_por_min/500);
	sprintf(sensor_lux_string, "%u", semaforo.sensor_lux );
	sprintf(sensor_temp_string, "%u", semaforo.sensor_temp );
	
	nokia_lcd_clear();
	
	nokia_lcd_set_cursor(0,5);
	nokia_lcd_write_string("Modo", 1);
	nokia_lcd_set_cursor(30, 5);
	nokia_lcd_write_string(modo_string, 1);
	nokia_lcd_set_cursor(0, 15);
	nokia_lcd_write_string("T.Vd", 1);
	nokia_lcd_set_cursor(30, 15);
	nokia_lcd_write_string(tempo_verde_s_string, 1); nokia_lcd_write_string("s", 1);
	nokia_lcd_set_cursor(0, 25);
	nokia_lcd_write_string("T.Vm", 1);
	nokia_lcd_set_cursor(30,25);
	nokia_lcd_write_string(tempo_vermelho_s_string, 1); nokia_lcd_write_string("s", 1);
	nokia_lcd_set_cursor(0, 35);
	nokia_lcd_write_string("T.AM", 1);
	nokia_lcd_set_cursor(30, 35);
	nokia_lcd_write_string(tempo_amarelo_s_string, 1); nokia_lcd_write_string("s", 1);
	
	nokia_lcd_set_cursor(42, 5 + selecao_parametro*10);
	nokia_lcd_write_string("<", 1);
	
	//nokia_lcd_draw_Vline(50, 2, 47);
	
	nokia_lcd_set_cursor(52,20);	
	nokia_lcd_write_string(carros_por_min_string, 1);
	nokia_lcd_set_cursor(55, 30);
	nokia_lcd_write_string("c/min", 1);
	
	nokia_lcd_set_cursor(52, 0);
	nokia_lcd_write_string(sensor_lux_string, 1);
	nokia_lcd_set_cursor(55, 10);
	nokia_lcd_write_string("lux", 1);
	
	nokia_lcd_set_cursor(52, 40);
	nokia_lcd_write_string(sensor_temp_string, 1);
	nokia_lcd_set_cursor(65, 40);
	nokia_lcd_write_string("*C", 1);
	
	nokia_lcd_render();

}

void estima_carros_por_min(uint8_t *flag_disparo)
{
	static uint16_t aux = 0;
	
	if(*flag_disparo)
	{
		*flag_disparo = 0;
		aux = num_carros;
		num_carros = 0;
		semaforo.carros_por_min = aux*12; // *60/5  -> converter de carros/seg para carros/min
		
		if(semaforo.modo)
		{
			semaforo.tempo_verde_ms = 1000 + ((uint16_t)(semaforo.carros_por_min*16.7)/1000)*1000;
			if(semaforo.tempo_verde_ms > 9000)
				semaforo.tempo_verde_ms = 9000;
			semaforo.tempo_vermelho_ms = 9000 -  ((uint16_t)(semaforo.carros_por_min*16.7)/1000)*1000;
			if(semaforo.tempo_vermelho_ms > 32000)
				semaforo.tempo_vermelho_ms =  1000;
		}
		
		//anima_LCD(semaforo);
	}
}

void leituraADC_sensor_LUX(uint8_t *flag_disparo)
{
	if(*flag_disparo)
	{
		semaforo.sensor_lux = (1023000/ADC - 1000)*3.16;
		if (semaforo.sensor_lux > 300) // liga a luminária caso a medição seja menor que 300 lux
		{
			//PORTC |= 0b1000000;
			OCR2B = 0;
		}
		else
		{
			if(((PINC & 0b1000000)==0)  || (semaforo.carros_por_min > 0))
			{
				
				OCR2B = 255;
			}
			else
			{
				OCR2B = 85;
				//PORTC &= 0b0111111;
			}
		}
		
		*flag_disparo = 0;
		anima_LCD(semaforo);
	}
}

void leituraADC_sensor_TEMP(uint8_t *flag_disparo)
{
	if(*flag_disparo)
	{
		//função para converter a resistencia para temperatura
		Vntc = ((5.0*ADC)/1023);
		Rntc = (Vntc*(10000.0/(5.0-Vntc)));
		
		T_Kelvin = 1/(a+b*log(Rntc)+c*pow(log(Rntc), 3));
		Celsius = (T_Kelvin - 273);
				
		semaforo.sensor_temp = Celsius;
		*flag_disparo = 0;
		anima_LCD(semaforo);
	}
}



int main(void)
{
	//Definições de GPIO
	DDRB = 0b11111111;
	DDRD = 0b10000000;
	DDRC |= 0b1000000;   // habilita PC6 como saída
	PORTD = 0b01111101;
	
	DDRD = (1<<LED);
	
	EICRA  = 0b00001010;
	EIMSK  = 0b00000011;
	PCICR  = 0b00000100;
	PCMSK2 = 0b01110001;
	
	TCCR0A = 0b00000010;
	TCCR0B = 0b00000011;
	OCR0A = 249;
	TIMSK0 = 0b00000010;
	
	/*PORTC = 0xFE; //desabilita o pullup do PC0
	ADMUX = 0b11000000; //Tensão interna de ref (1.1V) canal 0
	ADCSRA = 0b11101111; //Habilita o AD, habilita interrupção, modo de conversão continua, prescaler = 128
	ADCSRB = 0x00; //modo de conversão continua
	DIDR0 = 0b00111110; //habilita pino PC0 como entrada do ADC0
	*/
	
	//Configuração do ADC
	ADMUX = 0b01000000;  // VCC como ref, canal 0
	ADCSRA = 0b11100111; // habilita o AD, modo de conversão continua, prescaler = 128
	ADCSRB = 0b00000000; // modo de conversão contínua
	DIDR0 = 0b00000001; // desabilita pino PC0 como entrada digital
	
	
	
	nokia_lcd_init();
	anima_LCD(semaforo);
	
	sei(); // habilita interrupções globais
	
	while (1)
	{
		PORTD ^= (1<<LED); //aciona o speaker
		_delay_ms(1000);
		
		anima_semaforo(semaforo, tempo_ms);
		estima_carros_por_min(&flag_5000ms);
		leituraADC_sensor_LUX(&flag_500ms);
		leituraADC_sensor_TEMP(&flag_500ms);
	}
}

