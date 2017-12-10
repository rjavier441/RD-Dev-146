#include "LPC17xx.h"    // LPC_GPIOx, PINSELX
#include "stdint.h"     // uintX_t
#include "printf_lib.h" // u0_dbg_printf()
#include "adc0.h"       // ADC0 interface

class keypad {
public:
    keypad();   // default
    char key(uint16_t raw_adc_val);
private:
};