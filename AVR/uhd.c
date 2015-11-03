/* 
 * Universal hardware driver for microcontrollers
 * 
 * File:     uhd.h
 * Descript: Universal hardware driver file.
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
#include "uhd.h"

// --------------------------------------------------------
// OSC
// --------------------------------------------------------
#if (_UHD_OSC_CUTDOWN != 1) && defined(CLKPR)

uint32_t MCU_freq = MCU_OSC_FREQ_DEFAULT;
uint8_t MCU_div = MCU_OSC_DIV_DEFAULT;

// set OSC frequency divisor, it will effect all timer function
//  freq_div: MCU frequency divisor
void OSC_freq_div(enum OSC_FREQ_DIV freq_div)
{
    // out of range
    if(freq_div > 8)
        return;

    // set clock divisor
    clock_prescale_set((clock_div_t)freq_div);
    
    // calculate MCU frequency
    MCU_freq = MCU_OSC_FREQ_MAX >> freq_div;
    MCU_div = freq_div;
}
#else

#endif

// --------------------------------------------------------
// WDT
// --------------------------------------------------------
#if (_UHD_WDT_CUTDOWN != 1)

uint8_t __WDTO = WDTO_DISABLE;

#ifdef WDIE
ISR(WDT_vect)
{
    return;
}

// WDT delay, use WDT to wake from sleep
//  cnt: delay counter, delay time is (15ms * cnt)
void WDT_sleep(uint16_t cnt)
{
    uint8_t _sav_sreg = SREG;
    
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    MCUSR &= ~(1<<WDRF);    // clear WDRF
    DISABLE_interrupt();
    
    wdt_enable(WDTO_15MS);
    WDTCSR &= ~(1<<WDE);    // clear WDE
    ENABLE_interrupt();     // Enable interrupt
    
    while(cnt > 0)
    {
        cnt--;
		WDTCSR |= (1<<WDIE);// Enable WDT interrupt again
		sleep_cpu();        // sleep mcu, and wait WDT timeout
    }
    
    wdt_enable(__WDTO);     // restore WDT
    
    SREG = _sav_sreg;       // restore interrupt status
}
#endif

// set WDT timeout interval
//  WDTO: timeout interval
void WDT_set(uint8_t WDTO)
{
    __WDTO = WDTO;
    if(WDTO == WDTO_DISABLE)
        WDT_disable();
    else
        wdt_enable(WDTO);
}
#endif

// --------------------------------------------------------
// UART
// --------------------------------------------------------
#if (_UHD_UART_CUTDOWN != 1)

// TX/RX isr function point
void (* _UHD_UART0_TX_ISR_USERFUNC)(void);
void (* _UHD_UART0_RX_ISR_USERFUNC)(void);

// AVR RX isr
ISR(USART0_RX_vect)
{
    if(_UHD_UART0_RX_ISR_USERFUNC)
        _UHD_UART0_RX_ISR_USERFUNC();
}

// AVR TX isr
ISR(USART0_TX_vect)
{
    if(_UHD_UART0_TX_ISR_USERFUNC)
        _UHD_UART0_TX_ISR_USERFUNC();  
}

// put a string to UART0
//  msg: string to be send
void UART0_puts(char *msg)
{
    while(*msg)
    {
        UART0_put(*msg);            // send a char
        msg++;
        while(!UART0_writeable());  // wait send finished
    }
}

// initial UART0
//  baudrate: UART0 baudrate, such as 9600/115200 etc.
//            if baudrate euq zero, UART0 is disabled, TXD/RXD io will act as GPIO
//  tx_isr: user tx isr, if no tx isr used, set it to NULL.
//  rx_isr: user rx isr, if no rx isr used, set it to NULL.
//  err_isr: user err isr, not use in AVR.
void UART0_init(uint32_t baudrate, void (* tx_isr)(), void (* rx_isr)(), void (* err_isr)())
{
    UCSR0A = UCSR0B = UCSR0C = 0;
    UBRR0 = 0;

    // disable UART
    if(baudrate == 0)
    {
        return;
    }

    // set user isr function
    _UHD_UART0_RX_ISR_USERFUNC = rx_isr;
    _UHD_UART0_TX_ISR_USERFUNC = tx_isr;       

    // set UART0
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
    UCSR0A = (1<<U2X0);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    if(rx_isr)
    {
        UCSR0B |= (1<<RXCIE0);
    }
    if(tx_isr)
    {
        UCSR0B |= (1<<TXCIE0);
    }
    
    // set baudrate
    UBRR0 = MCU_freq / (8L * baudrate) -1;
}

#endif

// --------------------------------------------------------
// TIMER
// --------------------------------------------------------
#if (_UHD_TMR_CUTDOWN != 1)

#if (_UHD_TMR0_CUTDOWN != 1)
volatile uint16_t TMR0ISRCNT, TMR0ISRMAX;;
void (* _UHD_TMR0_ISR_USERFUNC)(void);

// AVR TMR0 isr, CTC mode
ISR(TIMER0_COMPA_vect)
{
    if(TMR0ISRCNT)
    {
        TMR0ISRCNT--;
    }
    else
    {
        TMR0ISRCNT = TMR0ISRMAX;
        _UHD_TMR0_ISR_USERFUNC();
    }
}

// set TMR0 ticker
//  interval: ticker interval (ms). if interval equ zero, TMR0 ticker will be disable.
//  isr: user's isr function for TMR0 ticker.
void TMR0_ticker(uint16_t interval, void (* isr)())
{
    TCCR0A = 0;
    TCCR0B = 0;
    TIMSK0 &= ~(1<<OCIE0A);
    TCNT0 = 0;
    
    // disable TMR0
    if(interval == 0)
        return;

    _UHD_TMR0_ISR_USERFUNC = isr;
    if(isr)
    {
        TIMSK0 |= (1<<OCIE0A);
    }
    TMR0ISRCNT = TMR0ISRMAX = interval - 1;

    // start TMR0
    TCCR0A = (1<<WGM01); // CTC
    if(MCU_freq > 16000000L)
    {
        OCR0A = MCU_freq / 256000L - 1;
        TCCR0B = (1<<CS02)|(0<<CS01)|(0<<CS00);  // CLK/256
    }
    else
    {
        OCR0A = MCU_freq / 64000L - 1;
        TCCR0B = (0<<CS02)|(1<<CS01)|(1<<CS00);  // CLK/64
    }
}
#endif

#if (_UHD_TMR1_CUTDOWN != 1)
volatile uint16_t TMR1ISRCNT, TMR1ISRMAX;;
void (* _UHD_TMR1_ISR_USERFUNC)(void);

// AVR TMR1 isr
ISR(TIMER1_COMPA_vect)
{
    if(TMR1ISRCNT)
    {
        TMR1ISRCNT--;
    }
    else
    {
        TMR1ISRCNT = TMR1ISRMAX;
        _UHD_TMR1_ISR_USERFUNC();
    }
}

// set TMR1 ticker
//  interval: ticker interval (ms). if interval equ zero, TMR1 ticker will be disable.
//  isr: user's isr function for TMR1 ticker.
void TMR1_ticker(uint16_t interval, void (* isr)())
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TIMSK1 &= ~(1<<OCIE1A);
    TCNT1 = 0;
    
    // disable TMR1
    if(interval == 0)
        return;
    
    _UHD_TMR1_ISR_USERFUNC = isr;
    if(isr)
    {
        TIMSK1 |= (1<<OCIE1A);
    }
    TMR1ISRCNT = TMR1ISRMAX = interval - 1;
    
    if(MCU_freq > 16000000)
    {
        OCR1A = MCU_freq / 256000L - 1;
        TCCR1B = (1<<WGM12)|(1<<CS12);    // Prescale = 256
    }
    else
    {
        OCR1A = MCU_freq / 64000L - 1;
        TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10);    // Prescale = 64
    }

}
#endif

#if (_UHD_TMR2_CUTDOWN != 1)
volatile uint16_t TMR2ISRCNT, TMR2ISRMAX;;
void (* _UHD_TMR2_ISR_USERFUNC)(void);

// AVR TMR2 isr
ISR(TIMER2_COMPA_vect)
{
    if(TMR2ISRCNT)
    {
        TMR2ISRCNT--;
    }
    else
    {
        TMR2ISRCNT = TMR2ISRMAX;
        _UHD_TMR2_ISR_USERFUNC();
    }
}

// set TMR2 ticker
//  interval: ticker interval (ms). if interval equ zero, TMR2 ticker will be disable.
//  isr: user's isr function for TMR2 ticker.
void TMR2_ticker(uint16_t interval, void (* isr)())
{
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 &= ~(1<<OCIE2A);
    TCNT2 = 0;
    
    // disable TMR2
    if(interval == 0)
        return;

    _UHD_TMR2_ISR_USERFUNC = isr;
    if(isr)
    {
        TIMSK2 |= (1<<OCIE2A);
    }
    TMR2ISRCNT = TMR2ISRMAX = interval - 1;
    
    TCCR2A = (1<<WGM21); // CTC
    if(MCU_freq > 16000000)
    {
        OCR2A = MCU_freq / 128000L - 1;
        TCCR2B = (1<<CS22)|(0<<CS21)|(1<<CS20);  // CLK/128
    }
    else
    {
        OCR2A = MCU_freq / 64000L - 1;
        TCCR2B = (1<<CS22)|(0<<CS21)|(0<<CS20);  // CLK/64
    }
}
#endif

#endif

// --------------------------------------------------------
// EXTINT
// --------------------------------------------------------
#if (_UHD_EXTINT_CUTDOWN_ != 1)
#if (_UHD_EXTINT0_CUTDOWN_ != 1)
void (* _UHD_EXTINT0_ISR_USERFUNC)(void);

ISR(INT0_vect)
{
    if(_UHD_EXTINT0_ISR_USERFUNC)
        _UHD_EXTINT0_ISR_USERFUNC();
}

// set EXTINT0 mode and isr
//  mode: EXTINT mode
//  isr: user EXTINT isr
void EXTINT0_init(enum EXTINT_MODE mode, void (* isr)())
{
    EXTINT0_disable();
    
    if(mode == EXTINT_OFF)
        return;
    
    if(mode == EXTINT_ONRISING)
    {
        EICRA |= (1<<ISC01)|(1<<ISC00);
    }
    else
    {
        EICRA |= (1<<ISC01)|(0<<ISC00);
    }
    
    EXTINT0_flag_clr();
    _UHD_EXTINT0_ISR_USERFUNC = isr;
    if(_UHD_EXTINT0_ISR_USERFUNC)
        EXTINT0_enable();
}
#endif

#if (_UHD_EXTINT1_CUTDOWN_ != 1)
void (* _UHD_EXTINT1_ISR_USERFUNC)(void);

ISR(INT1_vect)
{
    if(_UHD_EXTINT1_ISR_USERFUNC)
        _UHD_EXTINT1_ISR_USERFUNC();
}

// set EXTINT1 mode and isr
//  mode: EXTINT mode
//  isr: user EXTINT isr
void EXTINT1_init(enum EXTINT_MODE mode, void (* isr)())
{
    EXTINT1_disable();
    
    if(mode == EXTINT_OFF)
        return;
    
    if(mode == EXTINT_ONRISING)
    {
        EICRA |= (1<<ISC11)|(1<<ISC10);
    }
    else
    {
        EICRA |= (1<<ISC11)|(0<<ISC10);
    }
    
    EXTINT1_flag_clr();
    _UHD_EXTINT1_ISR_USERFUNC = isr;
    if(_UHD_EXTINT1_ISR_USERFUNC)
        EXTINT1_enable();
}
#endif
#endif

// --------------------------------------------------------
// PININT
// --------------------------------------------------------
#if (_UHD_PININT_CUTDOWN_ != 1)

void (* _UHD_PININT_ISR_USERFUNC)(void);

ISR(PCINT0_vect)
{
    if(_UHD_PININT_ISR_USERFUNC)
        _UHD_PININT_ISR_USERFUNC();
}
ISR_ALIAS(PCINT1_vect, PCINT0_vect);
ISR_ALIAS(PCINT2_vect, PCINT0_vect);

// set PININT IO
//  index: PININT IO index
//  mode: PININT IO mode
void PININT_set(uint8_t index, enum PININT_MODE mode)
{
    if(index < 8)
    {
        if(mode != PININT_NULL)
            PCMSK0 |= (1<<index);
        else
            PCMSK0 &= ~(1<<index);
    }
    else if(index < 16)
    {
        if(mode != PININT_NULL)
            PCMSK1 |= (1<<(index-8));
        else
            PCMSK1 &= ~(1<<(index-8));
    }
    else
    {
        if(mode != PININT_NULL)
            PCMSK2 |= (1<<(index-16));
        else
            PCMSK2 &= ~(1<<(index-16));
    }
}

// set PININT user isr
//  isr: user PININT isr
void PININT_init(void (* isr)())
{
    PININT_disable();
    PCIFR = 0x07;
    _UHD_PININT_ISR_USERFUNC = isr;
    if(_UHD_PININT_ISR_USERFUNC)
        PININT_enable();
}

#endif

// --------------------------------------------------------
// ADC
// --------------------------------------------------------
#if (_UHD_ADC_CUTDOWN_ != 1)
uint8_t _ADMUX = (1<<ADEN)|(1<<REFS0);

// initial ADC, set ADC clock and reference
//  mode: set ADC mode
//        AREF_ADCOFF, disable ADC
//        AREF_EXT, use external reference
//        AREF_AVCC, use AVVC as reference
//        AREF_INT, use internal reference
void ADC_init(enum ADC_MODE mode)
{
    uint8_t ADPS;
    
    ADCSRA = 0;
    ADCSRB = 0;
    switch(mode)
    {
        case ADC_OFF:
            return;
        case AREF_EXT:
            _ADMUX = 0x00;
            break;
        case AREF_INT:
            _ADMUX = (1<<REFS1)|(1<<REFS0);
            break;
        default:// AREF_VACC
            _ADMUX = (1<<REFS0);
            break;
    }
    
    ADCSRA = (1<<ADEN);
    ADPS = MCU_freq / 100000L;
    if(ADPS > 128)
        ADPS = 7;
    else if(ADPS > 64)
        ADPS = 6;
    else if(ADPS > 32)
        ADPS = 5;
    else if(ADPS > 16)
        ADPS = 4;
    else if(ADPS > 8)
        ADPS = 3;
    else if(ADPS > 4)
        ADPS = 2;
    else ADPS = 1;
        
    ADCSRA |= ADPS;
    
}

// get ADC result
//  chn: ADC channel
uint16_t ADC_get(uint8_t chn)
{
    chn = chn % 32;
    ADMUX = _ADMUX | chn;
    
    ADCSRA |= (1<<ADSC)|(1<<ADIF);
    while(!(ADCSRA & (1<<ADIF)))
        NOP();
    
    return ADC;
}
#endif

// --------------------------------------------------------
// PWM
// --------------------------------------------------------
#if (_UHD_PWM_CUTDOWN_ != 1)
#if (_UHD_PWM0_CUTDOWN_ != 1)

// set PWM0 frequency, division from system clock
//  sysdiv: system clock divisor
//          PWM0_OFF, PWM0 is disable
void PWM0_freq_sysdiv(enum PWM0_FREQ_SYSDIV_MODE sysdiv)
{
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    
    // disable PWM0
    if(sysdiv == PWM0_OFF)
        return;
    
    TCCR0A = (1<<WGM01)|(1<<WGM00);  // FAST PWM
    TCCR0B = sysdiv;
}

// set PWM0 output
//  no: PWM0 output pin
//  mode: PWM0 output mode
//  PluseWidth: PWM0 output plusewidth, it must be less than PWM width 
void PWM0_out(uint8_t no, enum PWM_MODE mode, uint8_t PluseWidth)
{
    switch(no)
    {
        case PWM0_OC0A:
            TCCR0A &= ~((1<<COM0A1)|(1<<COM0A0));
            if(mode == PWM_ON)
                TCCR0A |= (1<<COM0A1);
            else if(mode == PWM_INVERT_PHASE)
                TCCR0A |= (1<<COM0A1)|(1<<COM0A0);
            OCR0A = PluseWidth;
            break;
        case PWM0_OC0B:
            TCCR0A &= ~((1<<COM0B1)|(1<<COM0B0));
            if(mode == PWM_ON)
                TCCR0A |= (1<<COM0B1);
            else if(mode == PWM_INVERT_PHASE)
                TCCR0A |= (1<<COM0B1)|(1<<COM0B0);
            OCR0B = PluseWidth;
            break;
        default:
            return;
    }
}


#endif

#if (_UHD_PWM1_CUTDOWN_ != 1)
// set PWM1 frequency
//  freq: PWM frequency (ms)
//        if freq equ zero, PWM1 will be disable.
void PWM1_freq(uint32_t freq)
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // disable PWM1
    if(freq == 0)
        return;
    
    // Fast PWM: mode 14
    TCCR1A = (1<<WGM11);
    TCCR1B = (1<<WGM13)|(1<<WGM12);
    
    if(freq > (MCU_freq>>4))
    {
        ICR1 = 15;
        TCCR1B |= (1<<CS10);  // CLK/1
    }
    else if(freq > (MCU_freq>>10))
    {
        ICR1 = MCU_freq / freq - 1;
        TCCR1B |= (1<<CS10);  // CLK/1
    }
    else if(freq > (MCU_freq>>20))
    {
        ICR1 = (MCU_freq>>6) / freq - 1;
        TCCR1B |= (1<<CS11)|(1<<CS10);  // CLK/64
    }
    else
    {
        ICR1 = 1023;
        TCCR1B |= (1<<CS12)|(1<<CS10);
    }
}

// set PWM1 frequency, division from system clock
//  sysdiv: system clock divisor
void PWM1_freq_sysdiv(enum PWM1_FREQ_SYSDIV_MODE sysdiv)
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // disable PWM1
    if(sysdiv == PWM1_OFF)
        return;
    
    ICR1 = 256;
    TCCR1A = (0<<WGM11)|(1<<WGM10);  // FAST PWM
    TCCR1B = (1<<WGM12)|sysdiv;
}

// set PWM1 output
//  no: PWM1 output pin
//  mode: PWM1 output mode
//  PluseWidth: PWM1 output plusewidth, it must be less than PWM width 
void PWM1_out(uint8_t no, enum PWM_MODE mode, uint8_t PluseWidth)
{
    switch(no)
    {
        case PWM1_OC1A:
            TCCR1A &= ~((1<<COM1A1)|(1<<COM1A0));
            if(mode == PWM_ON)
                TCCR1A |= (1<<COM1A1);
            else if(mode == PWM_INVERT_PHASE)
                TCCR1A |= (1<<COM1A1)|(1<<COM1A0);
            OCR1A = PluseWidth;
            break;
        case PWM1_OC1B:
            TCCR1A &= ~((1<<COM1B1)|(1<<COM1B0));
            if(mode == PWM_ON)
                TCCR1A |= (1<<COM1B1);
            else if(mode == PWM_INVERT_PHASE)
                TCCR1A |= (1<<COM1B1)|(1<<COM1B0);
            OCR1B = PluseWidth;
            break;
        default:
            return;
    }
}
#endif

#if (_UHD_PWM2_CUTDOWN_ != 1)
// set PWM2 frequency, division from system clock
//  sysdiv: system clock divisor
void PWM2_freq_sysdiv(enum PWM2_FREQ_SYSDIV_MODE sysdiv)
{
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    
    // disable PWM2
    if(sysdiv == PWM2_OFF)
        return;
    
    TCCR2A = (1<<WGM21)|(1<<WGM20);  // FAST PWM
    TCCR2B = sysdiv;
}

// set PWM2 output
//  no: PWM2 output pin
//  mode: PWM2 output mode
//  PluseWidth: PWM2 output plusewidth, it must be less than PWM width 
void PWM2_out(uint8_t no, enum PWM_MODE mode, uint8_t PluseWidth)
{
    switch(no)
    {
        case PWM2_OC2A:
            TCCR2A &= ~((1<<COM2A1)|(1<<COM2A0));
            if(mode == PWM_ON)
                TCCR2A |= (1<<COM2A1);
            else if(mode == PWM_INVERT_PHASE)
                TCCR2A |= (1<<COM2A1)|(1<<COM2A0);
            OCR2A = PluseWidth;
            break;
        case PWM2_OC2B:
            TCCR2A &= ~((1<<COM2B1)|(1<<COM2B0));
            if(mode == PWM_ON)
                TCCR2A |= (1<<COM2B1);
            else if(mode == PWM_INVERT_PHASE)
                TCCR2A |= (1<<COM2B1)|(1<<COM2B0);
            OCR2B = PluseWidth;
            break;
        default:
            return;
    }
}
#endif

#endif

