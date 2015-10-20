/* 
 * Universal hardware driver for microcontrollers
 * 
 * File:     uhd.h
 * Descript: Universal hardware driver head file.
 *
 * Platform: PIC10
 * Board:    PIC10F32x Development Board
 * Compiler: MPLAB XC8
 * Version:  1.0
 * 
 * Author:   shaoziyang
 * Email:    shaoziyang@126.com
 * Date:     2015-Sept
 *
 */

#ifndef _UHD_H_
#define _UHD_H_ 1

// Enable/Disable cutdown
#define _UHD_CUTDOWN_ 0

#if (_UHD_CUTDOWN_ == 1)
#include "uhdcutdown.h"
#endif

#include <xc.h>
#include <stdint.h>

#define _UHD_VER_       1.0
#define _UHD_BUILD_     1060
#define _UHD_COMPILE_   XC8
#define _UHD_PLATFORM_  PIC10

// MCU maximum OSC(INTOSC) frequency
#define MCU_OSC_FREQ_MAX        16000000L

// MCU default OSC frequency
#define MCU_OSC_FREQ_DEFAULT    8000000L

// MCU OSC default division
#define MCU_OSC_DIV_DEFAULT     1

#ifndef _XTAL_FREQ
#define _XTAL_FREQ       MCU_OSC_FREQ_DEFAULT
#endif


#define IO_OUTPUT   0
#define IO_INPUT    1

#define IO_HIGH     1
#define IO_LOW      0

#define PULLUP_ENABLE  1
#define PULLUP_DISABLE 0

#define ENABLE      1
#define DISABLE     0

#define TRUE        1
#define FALSE       0

#define NULL        0

#define MACRO_AB(A, B)      MACRO_AB_EX(A, B)
#define MACRO_ABC(A, B, C)  MACRO_ABC_EX(A, B, C)
#define MACRO_ABCD(A, B, C, D)  MACRO_ABCD_EX(A, B, C, D)

#define MACRO_AB_EX(A, B)       A ## B
#define MACRO_ABC_EX(A, B, C)   A ## B ## C
#define MACRO_ABCD_EX(A, B, C, D)   A ## B ## C ## D

#define MACRO_ARG21(a, b)   a
#define MACRO_ARG22(a, b)   b


// --------------------------------------------------------
// COMMON
// --------------------------------------------------------
#define WDT_clr()   CLRWDT()
//#define NOP()     NOP()
//#define SLEEP()       SLEEP()
#define delay_US    __delay_us
#define delay_MS    __delay_ms

#define ENABLE_interrupt()  ei();PEIE=1
#define DISABLE_interrupt() di()

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


// --------------------------------------------------------
// IO
// --------------------------------------------------------
#define IO_dir(PIN, DIR)    MACRO_ABC(TRIS, MACRO_ARG21(PIN), MACRO_ARG22(PIN)) = DIR
#define IO_set(PIN)         MACRO_ABC(LAT, MACRO_ARG21(PIN), MACRO_ARG22(PIN)) = 1
#define IO_clr(PIN)         MACRO_ABC(LAT, MACRO_ARG21(PIN), MACRO_ARG22(PIN)) = 0
#define IO_inv(PIN)         MACRO_ABC(LAT, MACRO_ARG21(PIN), MACRO_ARG22(PIN)) = \
                                !MACRO_ABC(LAT, MACRO_ARG21(PIN), MACRO_ARG22(PIN))
#define IO_in(PIN)          MACRO_ABC(R, MACRO_ARG21(PIN), MACRO_ARG22(PIN))
#define IO_pullup(PIN, PULLUP)      MACRO_ABC(WPU, MACRO_ARG21(PIN), MACRO_ARG22(PIN)) = PULLUP

#define IO_digital(PIN)     MACRO_ABC(ANS, MACRO_ARG21(PIN), MACRO_ARG22(PIN)) = 0
#define IO_analog(PIN)      MACRO_ABC(ANS, MACRO_ARG21(PIN), MACRO_ARG22(PIN)) = 1


// --------------------------------------------------------
// INTOSC
// --------------------------------------------------------
#if (_UHD_OSC_CUTDOWN != 1)

enum INTOSC_FREQ
{
    INTOSC_16M  = 0,
    INTOSC_8M   = 1,
    INTOSC_4M   = 2,
    INTOSC_2M   = 3,
    INTOSC_1M   = 4,
    INTOSC_500K = 5,
    INTOSC_250K = 6,
    INTOSC_31K  = 7
};

enum OSC_FREQ_DIV
{
    OSC_DIV_1   = 0,
    OSC_DIV_2   = 1,
    OSC_DIV_4   = 2,
    OSC_DIV_8   = 3,
    OSC_DIV_16  = 4,
    OSC_DIV_32  = 5,
    OSC_DIV_64  = 6,
    OSC_DIV_512 = 7
};
// set INTOSC frequency, it will effect all timer function
// It has no effect for ext OSC
//  freq_div: frequency division
extern void OSC_freq_div(enum OSC_FREQ_DIV freq_div);

#endif

// --------------------------------------------------------
// ADC
// --------------------------------------------------------
#if (_UHD_ADC_CUTDOWN_ != 1)

#define ADC_enable()        ADON = 1
#define ADC_disable()       ADON = 0

// get ADC value
//  chn: ADC channel
extern uint8_t ADC_get(uint8_t chn);

#endif

// --------------------------------------------------------
// TIMER
// --------------------------------------------------------
#if (_UHD_TMR_CUTDOWN != 1)

// TIMER0
#define TMR0_interrupt_enable()     TMR0IE = 1
#define TMR0_interrupt_disable()    TMR0IE = 0

// TIMER2
#define TMR2_on()   TMR2ON = 1
#define TMR2_off()  TMR2ON = 0
#define TMR2_interrupt_enable()     TMR2IE = 1
#define TMR2_interrupt_disable()    TMR2IE = 0

// set timer interval and isr
//  interval: Timer interrupt interval 
//    when interval is 0, timer is disable
//  isr: user Timer isr
//    when isr is NULL, tmr isr is disable
extern void TMR0_ticker(uint16_t interval, void (* isr)());
extern void TMR2_ticker(uint16_t interval, void (* isr)());

#endif

// --------------------------------------------------------
// WDT
// --------------------------------------------------------
#if (_UHD_WDT_CUTDOWN != 1)

#define WDTO_1MS    0
#define WDTO_2MS    1
#define WDTO_4MS    2
#define WDTO_8MS    3
#define WDTO_16MS   4
#define WDTO_32MS   5
#define WDTO_64MS   6
#define WDTO_128MS  7
#define WDTO_256MS  8
#define WDTO_512MS  9
#define WDTO_1S     10
#define WDTO_2S     11
#define WDTO_4S     12
#define WDTO_8S     13
#define WDTO_16S    14
#define WDTO_32S    15
#define WDTO_64S    16
#define WDTO_128S   17
#define WDTO_256S   18
#define WDTO_DISABLE    255

// set WDT timeout interval
//  WDTO: timeout interval
extern void WDT_set(uint8_t WDTO);

// WDT delay, use WDT to wake from sleep
//  cnt: delay counter, delay time is (8ms * cnt)
extern void WDT_sleep(uint16_t cnt);

#define WDT_sleep_MS(MS)    WDT_sleep(MS/8)

#endif

// --------------------------------------------------------
// PWM
// --------------------------------------------------------
#if (_UHD_PWM_CUTDOWN_ != 1)

#define PWM_FREQ_DISABLE    0
enum PWM_MODE
{
    PWM_OFF,
    PWM_ON,
    PWM_INVERT_PHASE
};

// set PWM frequency, if freq == 0, PWM is disable
//  freq: PWM frequency
extern void PWM_freq(uint32_t freq);

// set PWM out
//  no: PWM number (1/2/3/4)
//  mode: PWM output mode
//  PluseWidth: PWM pluse width
extern void PWM_out(uint8_t no, enum PWM_MODE mode, uint16_t PluseWidth);

// PWM macro
#define PWM_duty(no, percent)       PWM_out(no, PWM_ON, (percent * (PR2+1)/ 25))
#define PWM_duty_inv(no, percent)   PWM_out(no, PWM_INVERT_PHASE, (percent * (PR2+1)/ 25))
#define PWM_disable(no)             PWM_out(no, PWM_OFF, 0)

#endif

// --------------------------------------------------------
// EXTINT
// --------------------------------------------------------
#if (_UHD_EXTINT_CUTDOWN_ != 1)

#define EXTINT_flag()       INTF
#define EXTINT_flag_clr()   INTF = 0

enum EXTINT_MODE
{
    EXTINT_OFF,
    EXTINT_ONRISING,
    EXTINT_ONFALLING
};
// set EXTINT mode and isr
//  mode: EXTINT mode
//  isr: user EXTINT isr
extern void EXTINT_init(enum EXTINT_MODE mode, void (* isr)());

#define EXTINT_enable()     INTE = 1
#define EXTINT_disable()    INTE = 0

#endif

// --------------------------------------------------------
// PININT
// --------------------------------------------------------
#if (_UHD_PININT_CUTDOWN_ != 1)

// IOC INT(PIN INT)
enum PININT_MODE
{
    PININT_NULL=0,
    PININT_POSITIVE_EDGE=1,
    PININT_NEGATIVE_EDGE=2,
    PININT_POSITIVE_NEGATIVE_EDGE=3
};

// set PININT user isr
//  isr: user PININT isr
extern void PININT_init(void (* isr)());
#define PININT_set(PIN, MODE)   {\
                                    MACRO_ABCD(IOC, MACRO_ARG21(PIN), P, MACRO_ARG22(PIN)) = (MODE%2);\
                                    MACRO_ABCD(IOC, MACRO_ARG21(PIN), N, MACRO_ARG22(PIN)) = (MODE>>1);\
                                }
#define PININT_flag(PIN)        MACRO_ABCD(IOC, MACRO_ARG21(PIN), F, MACRO_ARG22(PIN))
#define PININT_flag_clr(PIN)    MACRO_ABCD(IOC, MACRO_ARG21(PIN), F, MACRO_ARG22(PIN)) = 0
#define PININT_enable()         IOCIE = 1
#define PININT_disable()        IOCIE = 0

#endif

#endif
