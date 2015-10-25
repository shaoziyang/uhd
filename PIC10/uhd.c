/* 
 * Universal hardware driver for microcontrollers
 * 
 * File:     uhd.c
 * Descript: Universal hardware driver file.
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
 
#include "uhd.h"

// --------------------------------------------------------
// OSC
// --------------------------------------------------------
#if (_UHD_OSC_CUTDOWN_ != 1)

uint8_t MCU_div = MCU_OSC_DIV_DEFAULT;
uint16_t MCU_freq = MCU_OSC_FREQ_DEFAULT / 1000;

// set INTOSC frequency, it will effect all timer function
// It has no effect for ext OSC
//  freq_div: frequency division
void OSC_freq_div(enum OSC_FREQ_DIV freq_div)
{
    // check range
    if(freq_div > 7)
        return;

    OSCCONbits.IRCF = 7 - freq_div;
    MCU_div = freq_div;
    
    if(freq_div == 7)
    {
        MCU_freq = 31;
    }
    else
    {
        MCU_freq = ((uint16_t)(MCU_OSC_FREQ_MAX/1000)) >> (freq_div);
    }
}
#else

#define MCU_freq    MCU_OSC_FREQ_DEFAULT/1000

#endif

// --------------------------------------------------------
// ADC
// --------------------------------------------------------
#if (_UHD_ADC_CUTDOWN_ != 1)

#define ADCON_MASK  0xE1

// get ADC value
//  chn: ADC channel
uint8_t ADC_get(uint8_t chn)
{
    if(chn > 7)
        return 0;
    
    ADCON = ADCON_MASK | (chn << 2);
    GO_nDONE = 1;
    while(GO_nDONE);

    return ADRES;
}
#endif

// --------------------------------------------------------
// TIMER
// --------------------------------------------------------
#if (_UHD_TMR_CUTDOWN != 1)

#if (_UHD_TMR0_CUTDOWN != 1)
void (* _UHD_TMR0_ISR_USERFUNC)(void);
volatile uint16_t TMR0CNT, TMR0MAX;
volatile uint8_t TMR0PR;

// set timer interval and isr
//  interval: Timer interrupt interval 
//    when interval is 0, timer is disable
//  isr: user Timer isr
//    when isr is NULL, tmr isr is disable
void TMR0_ticker(uint16_t interval, void (* isr)())
{
    OPTION_REG &= 0xC0;
    TMR0_interrupt_disable();
    TMR0IF = 0;
    if(interval)
    {
        TMR0PR = 258 - MCU_freq / (4 * 16);
        TMR0 = TMR0PR;
        OPTION_REGbits.PS = 3;  // 1:16
        TMR0CNT = TMR0MAX = interval - 1;
        _UHD_TMR0_ISR_USERFUNC = isr;
        if(_UHD_TMR0_ISR_USERFUNC)
            TMR0_interrupt_enable();
    }
}
#endif

#if (_UHD_TMR2_CUTDOWN != 1)
void (* _UHD_TMR2_ISR_USERFUNC)(void);
volatile uint16_t TMR2CNT, TMR2MAX;

// set timer interval and isr
//  interval: Timer interrupt interval 
//    when interval is 0, timer is disable
//  isr: user Timer isr
//    when isr is NULL, tmr isr is disable
void TMR2_ticker(uint16_t interval, void (* isr)())
{
    T2CON = 0;
    TMR2_interrupt_disable();
    TMR2IF = 0;
    TMR2 = 0;
    if(interval)
    {
        if(MCU_freq > 1000)
        {
            PR2 = (MCU_freq >> 6) - 1;
            T2CONbits.T2CKPS = 1;   // Prescaler is 4
            T2CONbits.TOUTPS = 3;   // 1:4 Postscaler
        }
        else
        {
            PR2 = (MCU_freq >> 2) - 1;
            T2CONbits.T2CKPS = 0;   // Prescaler is 1
            T2CONbits.TOUTPS = 0;   // 1:1 Postscaler
        }
        TMR2_on();
        TMR2CNT = TMR2MAX = interval - 1;
        _UHD_TMR2_ISR_USERFUNC = isr;
        if(_UHD_TMR2_ISR_USERFUNC)
            TMR2_interrupt_enable();
    }
}
#endif

#endif

// --------------------------------------------------------
// WDT
// --------------------------------------------------------
#if (_UHD_WDT_CUTDOWN != 1)

// WDT delay, use WDT to wake from sleep
//  cnt: delay counter, delay time is (8ms * cnt)
void WDT_sleep(uint16_t cnt)
{
    // 128ms
    WDTCONbits.WDTPS = 0B00011;
    WDTCONbits.SWDTEN = 1;
    
    while(cnt > 0)
    {
        cnt--;
        SLEEP();
    }
}

// set WDT timeout interval
//  WDTO: timeout interval
void WDT_set(uint8_t WDTO)
{
    if(WDTO == WDTO_DISABLE)
    {
        SWDTEN = 0;
    }
    else
    {
        WDTCONbits.WDTPS = WDTO;
        SWDTEN = 1;
    }
}

#endif

// --------------------------------------------------------
// PWM
// --------------------------------------------------------
#if (_UHD_PWM_CUTDOWN_ != 1)

// use system clock set PWM frequency, and max duty is 100
//  sysdiv: system clock prescale
void PWM_freq_sysdiv(enum PWM_FREQ_SYSDIV_MODE sysdiv)
{
    T2CON = 0;
    TMR2_interrupt_disable();

    if(sysdiv == PWM_FREQ_SYSDIV_OFF)
        return;
    
    T2CONbits.T2CKPS = sysdiv - 1;
    // max duty is (PR2 + 1)*4 = 100
    PR2 = 24;
    TMR2_on();
}

// set PWM frequency, if freq == 0, PWM is disable
//  freq: PWM frequency
void PWM_freq(uint32_t freq)
{
    T2CON = 0;
    TMR2_interrupt_disable();
 
    if(freq == 0)
        return;
    
    if(freq >= ((uint32_t)MCU_freq << 6))
    {
        PR2 = 3;
    }
    else if(freq > MCU_freq)
    {   
        PR2 = ((uint32_t)MCU_freq << 8) / freq - 1; // Prescaler is 1
    }
    else if(freq >= (MCU_freq >> 4))
    {
        T2CONbits.T2CKPS = 2;   // Prescaler is 16
        PR2 = ((uint32_t)MCU_freq << 4) / freq - 1;
    }
    else if(freq > (MCU_freq >> 6))
    {
        T2CONbits.T2CKPS = 3;   // Prescaler is 64
        PR2 = (MCU_freq << 2) / freq - 1;
    }
    else
    {
        T2CONbits.T2CKPS = 3;   // Prescaler is 64
        PR2 = 255;
    }
    TMR2_on();
}

// set PWM out
//  no: PWM number
//  mode: PWM output mode
//  PluseWidth: PWM pluse width
void PWM_out(uint8_t no, enum PWM_MODE mode, uint16_t PluseWidth)
{
    switch(no)
    {
        case PWM_1:
            PWM1CON = 0;
            if(mode != PWM_OFF)
            {
                PWM1DCH = PluseWidth>>2;
                PWM1DCL = PluseWidth<<6;
                if(mode == PWM_INVERT_PHASE)
                    PWM1POL = 1;
                PWM1OE = 1;
                PWM1EN = 1;
            }
            break;
        case PWM_2:
            PWM2CON = 0;
            if(mode != PWM_OFF)
            {
                PWM2DCH = PluseWidth>>2;
                PWM2DCL = PluseWidth<<6;
                if(mode == PWM_INVERT_PHASE)
                    PWM2POL = 1;
                PWM2OE = 1;
                PWM2EN = 1;
            }
            break;
    }
}
#endif

// --------------------------------------------------------
// EXTINT
// --------------------------------------------------------
#if (_UHD_EXTINT_CUTDOWN_ != 1)

void (* _UHD_EXTINT_ISR_USERFUNC)(void);

// set EXTINT mode and isr
//  mode: EXTINT mode
//  isr: user EXTINT isr
void EXTINT_init(enum EXTINT_MODE mode, void (* isr)())
{
    EXTINT_disable();
    if(mode == EXTINT_OFF)
        return;

    if(mode == EXTINT_ONRISING)
    {
        INTEDG = 1;
    }
    else
    {
        INTEDG = 0;
    }
    EXTINT_flag_clr();
    _UHD_EXTINT_ISR_USERFUNC = isr;
    if(_UHD_EXTINT_ISR_USERFUNC)
        EXTINT_enable();
}
#endif

// --------------------------------------------------------
// PININT
// --------------------------------------------------------
#if (_UHD_PININT_CUTDOWN_ != 1)

void (* _UHD_PININT_ISR_USERFUNC)(void);
void PININT_init(void (* isr)())
{
    PININT_disable();
    //IOCIF = 0;
    _UHD_PININT_ISR_USERFUNC = isr;
    if(_UHD_PININT_ISR_USERFUNC)
        PININT_enable();
}

#endif

// --------------------------------------------------------
// ISR
// --------------------------------------------------------
#if (_UHD_ISR_CUTDOWN_ != 1)

/*
    PIC10 interrupt server routinue
*/
void interrupt _UHD_ISR(void)
{
    if(INTE && INTF)
    {
        INTF = 0;
        if(_UHD_EXTINT_ISR_USERFUNC)
            _UHD_EXTINT_ISR_USERFUNC();
    }
    
    if(IOCIE && IOCIF)
    {
        //IOCIF = 0;
        if(_UHD_PININT_ISR_USERFUNC)
            _UHD_PININT_ISR_USERFUNC();
    }
    
    if(TMR2IE && TMR2IF)
    {
        TMR2IF = 0;
        if(TMR2CNT)
        {
            TMR2CNT--;
        }
        else
        {
            TMR2CNT = TMR2MAX;
            if(_UHD_TMR2_ISR_USERFUNC)
                _UHD_TMR2_ISR_USERFUNC();
        }
    }
    
    if(TMR0IE && TMR0IF)
    {
        TMR0IF = 0;
        TMR0 = TMR0PR;
        if(TMR0CNT)
        {
            TMR0CNT--;
        }
        else
        {
            TMR0CNT = TMR0MAX;
            if(_UHD_TMR0_ISR_USERFUNC)
                _UHD_TMR0_ISR_USERFUNC();
        }
    }
}

#endif
