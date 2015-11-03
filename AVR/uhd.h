/* 
 * Universal hardware driver for microcontrollers
 * 
 * File:     uhd.h
 * Descript: Universal hardware driver head file.
 *
 * Platform: AVR
 * Board:    Arduino Nano 3
 * Compiler: GNU Toolchain for Atmel AVR8/WinAVR
 * Version:  1.0
 * 
 * Author:   shaoziyang
 * Email:    shaoziyang@outlook.com
 * Date:     2015-Octo
 *
 */

#ifndef _UHD_H_
#define _UHD_H_

// Enable/Disable cutdown
#define _UHD_CUTDOWN_ 0

#if (_UHD_CUTDOWN_ == 1)
#include "uhdcutdown.h"
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <stdio.h>

#define _UHD_VER_       1.0
#define _UHD_BUILD_     1060
#define _UHD_COMPILE_   avr8-gnu-toolchain
#define _UHD_PLATFORM_  atMega328P

// MCU maximum OSC(INTOSC) frequency
#define MCU_OSC_FREQ_MAX        16000000L

// MCU default OSC frequency
#define MCU_OSC_FREQ_DEFAULT    16000000L

// MCU OSC default division
#define MCU_OSC_DIV_DEFAULT     OSC_DIV_1

#ifndef F_CPU
#define F_CPU          MCU_OSC_FREQ_DEFAULT
#endif

#include <util/delay.h>

#define IO_OUTPUT           1
#define IO_INPUT            0
#define IO_HIGH             1
#define IO_LOW              0

#define PULLUP_ENABLE       1
#define PULLUP_DISABLE      0

#define PORT_OUTPUT         0xFF
#define PORT_INPUT          0
#define PORT_HIGH           0xFF
#define PORT_LOW            0

#define MACRO_AB_EXPAND(A, B)       A ## B
#define MACRO_AB(A, B)      MACRO_AB_EXPAND(A, B)
#define MACRO_ARG21(a, b)   a
#define MACRO_ARG22(a, b)   b

#define ENABLE      1
#define DISABLE     0

#define TRUE        1
#define FALSE       0

#ifndef NULL
#define NULL        0
#endif

// --------------------------------------------------------
// COMMON
// --------------------------------------------------------
#define NOP() __asm__ __volatile__("nop")
//#define SLEEP()       SLEEP()

#define ENABLE_interrupt()      sei()
#define DISABLE_interrupt()     cli()

#define delay_MS            _delay_ms
#define delay_US            _delay_us

#define TMR_0       0
#define TMR_1       1
#define TMR_2       2

#define UART_0      0
#define UART_1      1

#define SPI_0       0

#define I2C_0       0

#define PWM_1       1
#define PWM_2       2
#define PWM_3       3
#define PWM_4       4

#define PWM0_OC0A   0
#define PWM0_OC0B   1
#define PWM1_OC1A   0
#define PWM1_OC1B   1
#define PWM2_OC2A   0
#define PWM2_OC2B   1

#define PININT_0    0
#define PININT_1    1
#define PININT_2    2
#define PININT_3    3
#define PININT_4    4
#define PININT_5    5
#define PININT_6    6
#define PININT_7    7
#define PININT_8    8
#define PININT_9    9
#define PININT_10   10
#define PININT_11   11
#define PININT_12   12
#define PININT_13   13
#define PININT_14   14
#define PININT_15   15
#define PININT_16   16
#define PININT_17   17
#define PININT_18   18
#define PININT_19   19
#define PININT_20   20
#define PININT_21   21
#define PININT_22   22
#define PININT_23   23

// Compatible define
#if !defined(UDR0)
#define UBRR0H      UBRRH
#define UBRR0L      UBRRL
#define UBRR0       UBRR
#define UCSR0A      UCSRA
#define UCSR0B      UCSRB
#define UCSR0C      UCSRC
#define UDR0        UDR
#define TXC0        TXC
#define RXC0        RXC
#define RXEN0       RXEN
#define TXEN0       TXEN
#define UCSZ01      UCSZ1
#define UCSZ00      UCSZ0
#define URSEL0      URSEL
#define U2X0        U2X
#define RXCIE0      RXCIE
#define TXCIE0      TXCIE
#endif

#ifdef USART_RX_vect
#define USART0_RX_vect  USART_RX_vect
#define USART0_TX_vect  USART_TX_vect
#endif

// --------------------------------------------------------
// GPIO
// --------------------------------------------------------
// set IO input/output
//  IO: GPIO, must define as below
//      #define LED    B, 5
//  DIR: IO_INPUT, IO as input
//       IO_OUTPUT, IO as output
#define IO_dir(IO, DIR) \
        {\
            MACRO_AB(DDR, MACRO_ARG21(IO)) &= ~(1 << MACRO_ARG22(IO));\
            MACRO_AB(DDR, MACRO_ARG21(IO)) |= (DIR << MACRO_ARG22(IO));\
        }
// set IO output high level
#define IO_set(IO)          (MACRO_AB(PORT, MACRO_ARG21(IO)) |= (1 << MACRO_ARG22(IO)))
// set IO output low level
#define IO_clr(IO)          (MACRO_AB(PORT, MACRO_ARG21(IO)) &= ~(1 << MACRO_ARG22(IO)))
// set IO output level invert
#define IO_inv(IO)          (MACRO_AB(PORT, MACRO_ARG21(IO)) ^= (1 << MACRO_ARG22(IO)))
// get IO input level
#define IO_in(IO)           ((MACRO_AB(PIN, MACRO_ARG21(IO)) & (1 << MACRO_ARG22(IO))) >> MACRO_ARG22(IO))

// set IO pullup resistance statue
//  PULLUP: PULLUP_ENABLE, enable pullup
//          PULLUP_DISABLE, disable pullup
#define IO_pullup(IO, PULLUP)      MACRO_AB(DDR, MACRO_ARG21(IO)) &= ~(1 << MACRO_ARG22(IO));\
                                   MACRO_AB(DDR, MACRO_ARG21(IO)) |= (1 << MACRO_ARG22(IO));\
                                   MACRO_AB(PORT, MACRO_ARG21(IO)) &= ~(1 << MACRO_ARG22(IO));\
                                   MACRO_AB(PORT, MACRO_ARG21(IO)) |= (PULLUP << MACRO_ARG22(IO))

// enable IO digital input function
#define IO_digital(IO)     MACRO_ABC(ANS, MACRO_ARG21(IO), MACRO_ARG22IO)) = 0
// enable IO analog input function
#define IO_analog(IO)      MACRO_ABC(ANS, MACRO_ARG21(IO), MACRO_ARG22(IO)) = 1

// set whole PORT direction
//  PORT: port name, such as A/B/C
//  DIR: port direction
#define PORT_dir(PORT, DIR) \
        {\
            MACRO_AB(DDR, PORT) = PORT_INPUT;\
            MACRO_AB(DDR, PORT) = DIR;\
        }

// set whole port output high level
#define PORT_set(_PORT)     MACRO_AB(PORT, _PORT) = PORT_HIGH
// set whole port output low level
#define PORT_clr(_PORT)     MACRO_AB(PORT, _PORT) = PORT_LOW
// set whole port output invert
#define PORT_inv(_PORT)     MACRO_AB(PORT, _PORT) ^= 0xFF
// get port input level
#define PORT_in(_PORT)      MACRO_AB(PIN, _PORT)

// --------------------------------------------------------
// OSC
// --------------------------------------------------------
#if (_UHD_OSC_CUTDOWN != 1) && defined(CLKPR)

// oscillator frequency divisor
enum OSC_FREQ_DIV
{
    OSC_DIV_1   = 0,
    OSC_DIV_2   = 1,
    OSC_DIV_4   = 2,
    OSC_DIV_8   = 3,
    OSC_DIV_16  = 4,
    OSC_DIV_32  = 5,
    OSC_DIV_64  = 6,
    OSC_DIV_128 = 7,
    OSC_DIV_256 = 8
};

// set OSC frequency divisor, it will effect all timer function
//  freq_div: MCU frequency divisor
extern void OSC_freq_div(enum OSC_FREQ_DIV freq_div);

#endif

// --------------------------------------------------------
// WDT
// --------------------------------------------------------
#if (_UHD_WDT_CUTDOWN != 1)
/*
WDTO_15MS
WDTO_30MS
WDTO_60MS
WDTO_120MS
WDTO_250MS
WDTO_500MS
WDTO_1S
WDTO_2S
WDTO_4S
WDTO_8S
*/
#define WDTO_DISABLE        255

// set WDT timeout interval
//  WDTO: timeout interval
extern void WDT_set(uint8_t WDTO);

#ifdef WDIE
// WDT delay, use WDT to wake from sleep
//  cnt: delay counter, delay time is (15ms * cnt)
extern void WDT_sleep(uint16_t cnt);
#endif

// WDT sleep in millisecond
#define WDT_sleep_MS(MS)    WDT_sleep(MS/16)
// Disable WDT
#define WDT_disable()       wdt_disable()
// reset/clear WDT
#define WDT_clr()           wdt_reset()
#endif

// --------------------------------------------------------
// UART
// --------------------------------------------------------
#if (_UHD_UART_CUTDOWN_ != 1)

#if (_UHD_UART0_CUTDOWN_ != 1)

// macro to control UART0
#define UART0_txisr_enable()    UCSR0B |= (1<<TXCIE0)
#define UART0_txisr_disable()   UCSR0B &= ~(1<<TXCIE0)
#define UART0_rxisr_enable()    UCSR0B |= (1<<RXCIE0)
#define UART0_rxisr_disable()   UCSR0B &= ~(1<<RXCIE0)
                                
#define UART0_tx_enable()       UCSR0B |= (1<<TXEN0)
#define UART0_tx_disable()      UCSR0B &= ~(1<<TXEN0)
#define UART0_rx_enable()       UCSR0B |= (1<<RXEN0)
#define UART0_rx_disable()      UCSR0B &= ~(1<<RXEN0)

// put a byte to UART0
#define UART0_put(dat)          UDR0 = dat
// get a byte from UART0
#define UART0_get()             UDR0
// if data in UART0 buffer? 
#define UART0_readable()        (UCSR0A&(1<<RXC0))
// if UART0 write buffer empty?
#define UART0_writeable()       (UCSR0A&(1<<UDRE0))

// initial UART0
//  baudrate: UART0 baudrate, such as 9600/115200 etc.
//            if baudrate euq zero, UART0 is disabled, TXD/RXD io will act as GPIO
//  tx_isr: user tx isr, if no tx isr used, set it to NULL.
//  rx_isr: user rx isr, if no rx isr used, set it to NULL.
//  err_isr: user err isr, not use in AVR.
extern void UART0_init(uint32_t baudrate, void (* tx_isr)(), void (* rx_isr)(), void (* err_isr)());

// put a string to UART0
//  msg: string to be send
extern void UART0_puts(char *msg);

#endif

#endif

// --------------------------------------------------------
// TIMER
// --------------------------------------------------------
#if (_UHD_TMR_CUTDOWN_ != 1)

// set TMR0 ticker
//  interval: ticker interval (ms). if interval equ zero, TMR0 ticker will be disable.
//  isr: user's isr function for TMR0 ticker.
extern void TMR0_ticker(uint16_t interval, void (* isr)());

// set TMR1 ticker
//  interval: ticker interval (ms). if interval equ zero, TMR1 ticker will be disable.
//  isr: user's isr function for TMR1 ticker.
extern void TMR1_ticker(uint16_t interval, void (* isr)());

// set TMR2 ticker
//  interval: ticker interval (ms). if interval equ zero, TMR2 ticker will be disable.
//  isr: user's isr function for TMR2 ticker.
extern void TMR2_ticker(uint16_t interval, void (* isr)());
#endif

// --------------------------------------------------------
// EXTINT
// --------------------------------------------------------
#if (_UHD_EXTINT_CUTDOWN_ != 1)
enum EXTINT_MODE
{
    EXTINT_OFF,
    EXTINT_ONRISING,
    EXTINT_ONFALLING
};

// macro to control EXTINT
#define EXTINT0_enable()    EIMSK |= (1<<INT0)
#define EXTINT0_disable()   EIMSK &= ~(1<<INT0)
#define EXTINT0_flag()      EIFR | (1<<INTF0)
#define EXTINT0_flag_clr()  EIFR |= (1<<INTF0)

#define EXTINT1_enable()    EIMSK |= (1<<INT1)
#define EXTINT1_disable()   EIMSK &= ~(1<<INT1)
#define EXTINT1_flag()      EIFR | (1<<INTF1)
#define EXTINT1_flag_clr()  EIFR |= (1<<INTF1)

// set EXTINT0/1 mode and isr
//  mode: EXTINT mode
//  isr: user EXTINT isr
extern void EXTINT0_init(enum EXTINT_MODE mode, void (* isr)());
extern void EXTINT1_init(enum EXTINT_MODE mode, void (* isr)());

#endif


// --------------------------------------------------------
// PININT
// --------------------------------------------------------
#if (_UHD_PININT_CUTDOWN_ != 1)

// IOC INT(PIN INT)
enum PININT_MODE
{
    PININT_NULL = 0,
    PININT_POSITIVE_NEGATIVE_EDGE = 3
};

// set PININT user isr
//  isr: user PININT isr
extern void PININT_init(void (* isr)());

// set PININT IO
//  index: PININT IO index
//  mode: PININT IO mode
extern void PININT_set(uint8_t index, enum PININT_MODE mode);

#define PININT_flag(index)        
#define PININT_flag_clr(index)     
#define PININT_enable()         PCICR = (1<<PCIE2)|(1<<PCIE1)|(1<<PCIE0)
#define PININT_disable()        PCICR = 0

#endif

// --------------------------------------------------------
// ADC
// --------------------------------------------------------
#if (_UHD_ADC_CUTDOWN_ != 1)
enum ADC_MODE{
    ADC_OFF,
    AREF_EXT,
    AREF_AVCC,
    AREF_INT
};

// initial ADC, set ADC clock and reference
//  mode: set ADC mode
//        AREF_ADCOFF, disable ADC
//        AREF_EXT, use external reference
//        AREF_AVCC, use AVVC as reference
//        AREF_INT, use internal reference
extern void ADC_init(enum ADC_MODE mode);

// get ADC result
//  chn: ADC channel
extern uint16_t ADC_get(uint8_t chn);

#endif

// --------------------------------------------------------
// PWM
// --------------------------------------------------------
#if (_UHD_PWM_CUTDOWN_ != 1)

// PWM output mode
enum PWM_MODE
{
    PWM_OFF,
    PWM_ON,
    PWM_INVERT_PHASE
};

#if (_UHD_PWM0_CUTDOWN_ != 1)

// PWM0 frequency divisor 
enum PWM0_FREQ_SYSDIV_MODE
{
    PWM0_OFF             = 0,
    PWM0_FREQ_SYSDIV_1   = 1,
    PWM0_FREQ_SYSDIV_8   = 2,
    PWM0_FREQ_SYSDIV_64  = 3,
    PWM0_FREQ_SYSDIV_256 = 4,    
    PWM0_FREQ_SYSDIV_1024= 5    
};

// set PWM0 frequency, division from system clock
//  sysdiv: system clock divisor
extern void PWM0_freq_sysdiv(enum PWM0_FREQ_SYSDIV_MODE sysdiv);

// set PWM0 output
//  no: PWM0 output pin
//  mode: PWM0 output mode
//  PluseWidth: PWM0 output plusewidth, it must be less than PWM width 
extern void PWM0_out(uint8_t no, enum PWM_MODE mode, uint8_t PluseWidth);

// PWM0 duty output, use percent instead of PluseWidth
#define PWM0_duty(no, percent)      PWM0_out(no, PWM_ON, percent * 256 / 100)
// PWM0 duty invert output
#define PWM0_duty_inv(no, percent)  PWM0_out(no, PWM_INVERT_PHASE, percent * 256 / 100)

#endif

#if (_UHD_PWM1_CUTDOWN_ != 1)
enum PWM1_FREQ_SYSDIV_MODE
{
    PWM1_OFF             = 0,
    PWM1_FREQ_SYSDIV_1   = 1,
    PWM1_FREQ_SYSDIV_8   = 2,
    PWM1_FREQ_SYSDIV_64  = 3,
    PWM1_FREQ_SYSDIV_256 = 4,
    PWM1_FREQ_SYSDIV_1024= 5    
};

// set PWM1 frequency, division from system clock
//  sysdiv: system clock divisor
extern void PWM1_freq_sysdiv(enum PWM1_FREQ_SYSDIV_MODE sysdiv);

// set PWM1 frequency
//  freq: PWM frequency (ms)
//        if freq equ zero, PWM1 will be disable.
extern void PWM1_freq(uint32_t freq);

// set PWM1 output
//  no: PWM1 output pin
//  mode: PWM1 output mode
//  PluseWidth: PWM1 output plusewidth, it must be less than PWM width 
extern void PWM1_out(uint8_t no, enum PWM_MODE mode, uint8_t PluseWidth);

// PWM1 duty output, use percent instead of PluseWidth
#define PWM1_duty(no, percent)      PWM1_out(no, PWM_ON, percent * ICR1 / 100)
// PWM1 duty invert output
#define PWM1_duty_inv(no, percent)  PWM1_out(no, PWM_INVERT_PHASE, percent * ICR1 / 100)
#endif

#if (_UHD_PWM2_CUTDOWN_ != 1)
enum PWM2_FREQ_SYSDIV_MODE
{
    PWM2_OFF             = 0,
    PWM2_FREQ_SYSDIV_1   = 1,
    PWM2_FREQ_SYSDIV_8   = 2,
    PWM2_FREQ_SYSDIV_32  = 3,
    PWM2_FREQ_SYSDIV_64  = 4,
    PWM2_FREQ_SYSDIV_128 = 5,
    PWM2_FREQ_SYSDIV_256 = 6,
    PWM2_FREQ_SYSDIV_1024= 7    
};

// set PWM2 frequency, division from system clock
//  sysdiv: system clock divisor
extern void PWM2_freq_sysdiv(enum PWM2_FREQ_SYSDIV_MODE sysdiv);

// set PWM2 output
//  no: PWM2 output pin
//  mode: PWM2 output mode
//  PluseWidth: PWM2 output plusewidth, it must be less than PWM width 
extern void PWM2_out(uint8_t no, enum PWM_MODE mode, uint8_t PluseWidth);

// PWM2 duty output, use percent instead of PluseWidth
#define PWM2_duty(no, percent)      PWM2_out(no, PWM_ON, percent * 256 / 100)
// PWM2 duty invert output
#define PWM2_duty_inv(no, percent)  PWM2_out(no, PWM_INVERT_PHASE, percent * 256 / 100)

#endif

#endif

#endif
