/* full_smart_plant.c
 *
 * Smart Plant Monitoring System (AVR-GCC)
 * MCU: ATmega328P, F_CPU = 4 MHz
 * Author: adapted for CODEZ / John Son project
 *
 * Wiring (summary):
 *  - Soil moisture analog -> A0 (PC0)
 *  - Tilt switch -> D2 (PD2 / INT0) with pull-up
 *  - Relay module IN -> D7 (PD7); COM -> +5V, NO -> pump +
 *  - Yellow LED -> D8 (PB0) via 220? to GND
 *  - Red LED -> D9 (PB1) via 220? to GND
 *  - Green LED -> D10 (PB2) via 220? to GND
 *  - I2C LCD (PCF8574 backpack): SDA->A4 (PC4), SCL->A5 (PC5)
 *  - USART TX -> PD1 (used for debug)
 *
 * Notes:
 *  - If your relay module is active-LOW, invert the logic in setRelay().
 *  - If your LCD is at address 0x3F, change LCD_ADDR or let scanner detect it.
 */

#define F_CPU 4000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/*** CONFIG ***/
#define SOIL_ADC_CHANNEL 0

// pins (use bit numbers)
#define TILT_PIN PD2
#define RELAY_PIN PD7

#define LED_YELLOW PB0
#define LED_RED    PB1
#define LED_GREEN  PB2

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)

#define WARN_THRESHOLD 10
#define ALERT_THRESHOLD 4
#define PUMP_OFF_THRESHOLD 20

volatile uint8_t tiltCount = 0;
volatile uint8_t pumpOn = 0;

/*** USART (debug) ***/
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

/*** ADC ***/
void ADC_Init(void) {
    // AVcc reference, ADC0 initially selected
    ADMUX = (1<<REFS0);
    // Enable ADC, prescaler = 64 => ADC clock = 4MHz/64 = 62.5kHz (good)
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
}
uint16_t ADC_Read(uint8_t ch) {
    ch &= 0x07;
    ADMUX = (ADMUX & 0xF8) | ch;
    _delay_us(5); // let MUX settle
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    return ADCW;
}

/*** TWI (I2C) ***/
void TWI_Init(void) {
    TWSR = 0x00;     // prescaler = 1
    TWBR = 0x0C;     // ~100kHz for F_CPU=4MHz
}

void TWI_Start(void) {
    TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void TWI_Stop(void) {
    TWCR = (1<<TWSTO)|(1<<TWEN)|(1<<TWINT);
    _delay_us(10);
}

// Write single byte on bus (assumes start + address already sent)
void TWI_WriteByte(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

/*** LCD over PCF8574 helper (common mapping used by many backpacks)
    We use the mapping that worked in your earlier code:
      - BACKLIGHT bit = 0x08
      - EN bit = 0x04 (toggled high->low for pulse)
      - RS bit = 0x01 (mode)
    If your backpack is mapped differently, adjust these bits.
***/
#define LCD_BACKLIGHT 0x08
#define LCD_EN        0x04
#define LCD_RS        0x01

// Default LCD address - we'll try to detect if needed
uint8_t LCD_ADDR = 0x27; // common; switch to 0x3F if needed

void LCD_I2C_WriteNibble(uint8_t nibble, uint8_t mode) {
    // nibble is the full byte; we'll take high nibble and low nibble (shifted)
    uint8_t data_u = nibble & 0xF0;
    uint8_t data_l = (nibble << 4) & 0xF0;
    uint8_t data_t[4];

    // Build sequence: (high nibble with EN=1) -> (high nibble with EN=0)
    // then same for low nibble.
    data_t[0] = data_u | LCD_BACKLIGHT | LCD_EN | (mode?LCD_RS:0);
    data_t[1] = data_u | LCD_BACKLIGHT | (mode?LCD_RS:0);
    data_t[2] = data_l | LCD_BACKLIGHT | LCD_EN | (mode?LCD_RS:0);
    data_t[3] = data_l | LCD_BACKLIGHT | (mode?LCD_RS:0);

    // Send sequence
    TWI_Start();
    TWI_WriteByte((LCD_ADDR<<1) | 0); // address + write (R/W=0)
    for (uint8_t i=0;i<4;i++) {
        TWI_WriteByte(data_t[i]);
    }
    TWI_Stop();
}

void LCD_I2C_Cmd(uint8_t cmd) {
    LCD_I2C_WriteNibble(cmd, 0);
    _delay_ms(2);
}
void LCD_I2C_Char(uint8_t data) {
    LCD_I2C_WriteNibble(data, 1);
    _delay_us(50);
}

void LCD_I2C_Init(void) {
    _delay_ms(50);
    // Init sequence for 4-bit mode
    // Some modules require specific sequence; do safe sequence
    LCD_I2C_Cmd(0x33);
    LCD_I2C_Cmd(0x32);
    LCD_I2C_Cmd(0x28); // 4-bit, 2 line, 5x8 dots
    LCD_I2C_Cmd(0x0C); // display on, cursor off
    LCD_I2C_Cmd(0x06); // entry mode, auto-increment
    LCD_I2C_Cmd(0x01); // clear
    _delay_ms(2);
}

void LCD_I2C_Clear(void) {
    LCD_I2C_Cmd(0x01);
    _delay_ms(2);
}

void LCD_I2C_SetCursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_I2C_Cmd(pos);
}

void LCD_I2C_String(const char *s) {
    while (*s) LCD_I2C_Char(*s++);
}

/*** Simple I2C scanner (prints results via USART) ***/
void i2c_scan_and_set_address(void) {
    // Try to find known addresses 0x27, 0x3F — but do a quick scan
    char buf[40];
    USART_SendString("I2C scan start...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        TWI_Start();
        TWDR = (addr<<1) | 0; // write
        TWCR = (1<<TWINT)|(1<<TWEN);
        while (!(TWCR & (1<<TWINT)));
        uint8_t status = TWSR & 0xF8;
        TWI_Stop();
        // 0x18 (SLA+W + ACK) / implementation may vary; easiest is to check if not NACK
        // The TWSR value for SLA+W transmitted and ACK received is 0x18.
        if (status == 0x18) {
            sprintf(buf, "Found I2C device at 0x%02X\r\n", addr);
            USART_SendString(buf);
            // If it's a common LCD address, pick it
            if (addr == 0x27 || addr == 0x3F) {
                LCD_ADDR = addr;
                sprintf(buf, "Using LCD address 0x%02X\r\n", LCD_ADDR);
                USART_SendString(buf);
                return;
            }
        }
    }
    // fallback if nothing explicit found: keep default 0x27 but report
    sprintf(buf, "Scan done. Using LCD address 0x%02X (default)\r\n", LCD_ADDR);
    USART_SendString(buf);
}

/*** Interrupt (tilt) ***/
ISR(INT0_vect) {
    // simple counter increment: count any change
    tiltCount++;
}

/*** Relay control (assume active HIGH by default)
    If your relay board is active LOW, invert the logic here.
*/
void setRelay(uint8_t state) {
    if (state) PORTD |= (1<<RELAY_PIN);
    else PORTD &= ~(1<<RELAY_PIN);
    pumpOn = state;
}

/*** Main ***/
int main(void) {
    USART_Init();
    _delay_ms(25);
    USART_SendString("Smart Plant Booting...\r\n");

    ADC_Init();
    TWI_Init();

    // Configure ports
    DDRB |= (1<<LED_YELLOW)|(1<<LED_RED)|(1<<LED_GREEN); // outputs for LEDs
    DDRD |= (1<<RELAY_PIN); // relay control output
    // Make sure LEDs/relay start OFF
    PORTB &= ~((1<<LED_YELLOW)|(1<<LED_RED)|(1<<LED_GREEN));
    PORTD &= ~(1<<RELAY_PIN);

    // Tilt switch pin as input with pull-up
    DDRD &= ~(1<<TILT_PIN);
    PORTD |= (1<<TILT_PIN); // enable pull-up

    // Enable INT0 on any logical change
    EICRA = (1<<ISC00); // ISC01=0, ISC00=1 => any logical change
    EIMSK = (1<<INT0);

    sei();

    // Try to detect LCD address (optional but helpful)
    i2c_scan_and_set_address();
    _delay_ms(50);

    // Initialize LCD
    LCD_I2C_Init();
    LCD_I2C_Clear();
    LCD_I2C_SetCursor(0,0);
    LCD_I2C_String("Smart Plant Init");
    _delay_ms(1000);
    LCD_I2C_Clear();

    char dbgbuf[64];
    uint16_t raw;
    uint8_t percent;

    while (1) {
        // Read ADC (soil)
        raw = ADC_Read(SOIL_ADC_CHANNEL);
        // compute percentage without float:
        percent = (uint8_t)((raw * 100UL) / 1023UL);

        // Debug print
        sprintf(dbgbuf, "Soil=%u (%u%%)\r\n", raw, percent);
        USART_SendString(dbgbuf);

        // Display on LCD (top line)
        LCD_I2C_SetCursor(0,0);
        LCD_I2C_String("Soil:");
        // print percentage right after
        itoa(percent, dbgbuf, 10);
        LCD_I2C_String(dbgbuf);
        LCD_I2C_String("%   "); // padding to erase leftover chars

        // Logic
        if (percent <= ALERT_THRESHOLD) {
            // Critical: water now
            PORTB |= (1<<LED_RED);
            PORTB &= ~((1<<LED_YELLOW));
            setRelay(1); // pump on
            PORTB |= (1<<LED_GREEN); // indicate pump running
            LCD_I2C_SetCursor(1,0);
            LCD_I2C_String("WATER THE PLANT ");
        }
        else if (percent <= WARN_THRESHOLD) {
            // Warning
            PORTB |= (1<<LED_YELLOW);
            PORTB &= ~(1<<LED_RED);
            // Turn pump off if we've gone above PUMP_OFF_THRESHOLD
            if (pumpOn && percent >= PUMP_OFF_THRESHOLD) {
                setRelay(0);
                PORTB &= ~(1<<LED_GREEN);
            } else if (!pumpOn) {
                PORTB &= ~(1<<LED_GREEN);
            }
            LCD_I2C_SetCursor(1,0);
            LCD_I2C_String("Moisture is low ");
        }
        else {
            // OK
            PORTB &= ~((1<<LED_YELLOW)|(1<<LED_RED));
            if (pumpOn && percent >= PUMP_OFF_THRESHOLD) {
                setRelay(0);
            }
            PORTB &= ~(1<<LED_GREEN);
            LCD_I2C_SetCursor(1,0);
            LCD_I2C_String("Moisture OK     ");
        }

        // Tilt detection: if toggled >= 10 times show wind message
        if (tiltCount >= 10) {
            LCD_I2C_Clear();
            LCD_I2C_SetCursor(0,0);
            LCD_I2C_String("Wind available ");
            // optionally show moisture on 2nd line after small delay
            _delay_ms(2500);
            LCD_I2C_Clear();
            tiltCount = 0;
        }

        _delay_ms(1000);
    }
    return 0;
}
