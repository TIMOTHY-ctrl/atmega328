/*
 * embeded-code.c
 *
 * Created: 22 Oct 2025 08:37:41
 * Author : CODEZ
 */ 

/*
 * Smart Plant Monitoring System (Pure AVR-GCC)
 * MCU: ATmega328P
 * Author: John Son
 *
 * Components:
 *  - Soil Moisture Sensor ? ADC0 (PC0)
 *  - Tilt Switch ? INT0 (PD2)
 *  - Relay ? PD7
 *  - Yellow LED ? PB0
 *  - Red LED ? PB1
 *  - Green LED ? PB2
 *  - I2C LCD 16x2 (SDA=A4/PC4, SCL=A5/PC5)
 *  - USART for debugging (9600 baud)
 */

#define F_CPU 4000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

// ====== I2C (TWI) LCD LIBRARY PROTOTYPES ======
void TWI_Init(void);
void TWI_Start(void);
void TWI_Stop(void);
void TWI_Write(unsigned char data);
void LCD_I2C_Init(void);
void LCD_I2C_Cmd(unsigned char cmd);
void LCD_I2C_Char(unsigned char data);
void LCD_I2C_String(char *str);
void LCD_I2C_Clear(void);
void LCD_I2C_SetCursor(uint8_t row, uint8_t col);

// ====== CONFIG ======
#define SOIL_ADC_CHANNEL 0
#define TILT_PIN PD2
#define RELAY_PIN PD7
#define LED_YELLOW PB0
#define LED_RED PB1
#define LED_GREEN PB2

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)

// ====== CONSTANTS ======
#define WARN_THRESHOLD 10
#define ALERT_THRESHOLD 4
#define PUMP_OFF_THRESHOLD 20

volatile uint8_t tiltCount = 0;
volatile uint16_t firstTiltTime = 0;
uint8_t pumpOn = 0;

// ====== USART ======
void USART_Init(void) {
    UBRR0H = (unsigned char)(MYUBRR>>8);
    UBRR0L = (unsigned char)MYUBRR;
    UCSR0B = (1<<TXEN0);   // TX only
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8N1
}
void USART_TxChar(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}
void USART_SendString(const char *str) {
    while (*str) USART_TxChar(*str++);
}

// ====== ADC ======
void ADC_Init(void) {
    ADMUX = (1<<REFS0); // AVcc reference
    ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); // Enable ADC, prescale 8
}
uint16_t ADC_Read(uint8_t ch) {
    ch &= 0x07;
    ADMUX = (ADMUX & 0xF8) | ch;
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    return ADCW;
}

// ====== I2C LCD SIMPLE IMPLEMENTATION ======
#define LCD_ADDR 0x27

void TWI_Init(void) {
    TWSR = 0x00;
    TWBR = 0x0C; // 100kHz for 4MHz
}

void TWI_Start(void) {
    TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void TWI_Stop(void) {
    TWCR = (1<<TWSTO)|(1<<TWEN)|(1<<TWINT);
}

void TWI_Write(unsigned char data) {
    TWDR = data;
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

// --- LCD helper macros ---
void LCD_I2C_WriteNibble(unsigned char nibble, unsigned char mode) {
    unsigned char data_u = nibble & 0xF0;
    unsigned char data_l = (nibble << 4) & 0xF0;
    unsigned char data_t[4];
    data_t[0] = data_u | 0x0C | mode;
    data_t[1] = data_u | 0x08 | mode;
    data_t[2] = data_l | 0x0C | mode;
    data_t[3] = data_l | 0x08 | mode;
    TWI_Start();
    TWI_Write(LCD_ADDR<<1);
    for(uint8_t i=0;i<4;i++) TWI_Write(data_t[i]);
    TWI_Stop();
}

void LCD_I2C_Cmd(unsigned char cmd) {
    LCD_I2C_WriteNibble(cmd,0);
}

void LCD_I2C_Char(unsigned char data) {
    LCD_I2C_WriteNibble(data,1);
}

void LCD_I2C_Init(void) {
    _delay_ms(50);
    LCD_I2C_Cmd(0x02);
    LCD_I2C_Cmd(0x28);
    LCD_I2C_Cmd(0x0C);
    LCD_I2C_Cmd(0x06);
    LCD_I2C_Cmd(0x01);
    _delay_ms(2);
}

void LCD_I2C_String(char *str) {
    while(*str) LCD_I2C_Char(*str++);
}

void LCD_I2C_SetCursor(uint8_t row, uint8_t col) {
    unsigned char pos = (row==0)?(0x80+col):(0xC0+col);
    LCD_I2C_Cmd(pos);
}

void LCD_I2C_Clear(void) {
    LCD_I2C_Cmd(0x01);
    _delay_ms(2);
}

// ====== INTERRUPT HANDLER (Tilt Switch) ======
ISR(INT0_vect) {
    if (tiltCount == 0) {
        firstTiltTime = 0;
    }
    tiltCount++;
}

// ====== RELAY CONTROL ======
void setRelay(uint8_t state) {
    if (state) PORTD |= (1<<RELAY_PIN);
    else PORTD &= ~(1<<RELAY_PIN);
    pumpOn = state;
}

// ====== MAIN ======
int main(void) {
    USART_Init();
    ADC_Init();
    TWI_Init();
    LCD_I2C_Init();
    sei();

    // Configure pins
    DDRB |= (1<<LED_YELLOW)|(1<<LED_RED)|(1<<LED_GREEN);
    DDRD |= (1<<RELAY_PIN);
    PORTD |= (1<<TILT_PIN); // pull-up for tilt

    // Enable external interrupt on INT0
    EICRA = (1<<ISC00); // trigger on any change
    EIMSK = (1<<INT0);

    LCD_I2C_SetCursor(0,0);
    LCD_I2C_String("Smart Plant Init");
    _delay_ms(1000);
    LCD_I2C_Clear();

    char buffer[20];
    uint16_t raw;
    uint8_t percent;

    while(1) {
        raw = ADC_Read(SOIL_ADC_CHANNEL);
        percent = (uint8_t)(((float)raw/1023.0)*100.0);

        // Debug
        sprintf(buffer,"Soil=%u (%u%%)\r\n",raw,percent);
        USART_SendString(buffer);

        LCD_I2C_SetCursor(0,0);
        LCD_I2C_String("Soil:");
        itoa(percent,buffer,10);
        LCD_I2C_String(buffer);
        LCD_I2C_String("%   ");

        // --- logic ---
        if (percent <= ALERT_THRESHOLD) {
            PORTB |= (1<<LED_RED);
            PORTB &= ~(1<<LED_YELLOW);
            setRelay(1);
            PORTB |= (1<<LED_GREEN);
            LCD_I2C_SetCursor(1,0);
            LCD_I2C_String("WATER THE PLANT ");
        }
        else if (percent <= WARN_THRESHOLD) {
            PORTB |= (1<<LED_YELLOW);
            PORTB &= ~(1<<LED_RED);
            if (pumpOn && percent >= PUMP_OFF_THRESHOLD) setRelay(0);
            LCD_I2C_SetCursor(1,0);
            LCD_I2C_String("Moisture is low ");
        }
        else {
            PORTB &= ~((1<<LED_YELLOW)|(1<<LED_RED));
            if (pumpOn && percent >= PUMP_OFF_THRESHOLD) setRelay(0);
            LCD_I2C_SetCursor(1,0);
            LCD_I2C_String("Moisture OK     ");
        }

        // Tilt detection (simplified)
        if (tiltCount >= 10) {
            LCD_I2C_Clear();
            LCD_I2C_SetCursor(0,0);
            LCD_I2C_String("Wind available");
            tiltCount = 0;
        }

        _delay_ms(1000);
    }
}


