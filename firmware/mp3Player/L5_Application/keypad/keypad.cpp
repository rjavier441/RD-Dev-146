#include "keypad.h"

keypad::keypad () {

}

/*
    @function   key
    @parameter  raw_adc_val - the raw adc value (tested to be a number from 0-4000)
    @returns    The character repesented by the key being pressed.
    @details    This function (though more inefficient for the lower character keys) determines the value being pressed.
    @note       In the future, I plan to improve the efficiency of this key search by using a binary search tree data structure.
*/
char keypad::key (uint16_t raw_adc_val) {
    char temp = '\0';
    if (raw_adc_val > 3700) {
        temp = '1';
    } else if (raw_adc_val > 3600) {
        temp = '2';
    } else if (raw_adc_val > 3530) {
        temp = '3';
    } else if (raw_adc_val > 3470) {
        temp = 'A';
    } else if (raw_adc_val > 3400) {
        temp = '4';
    } else if (raw_adc_val > 3315) {
        temp = '5';
    } else if (raw_adc_val > 3260) {
        temp = '6';
    } else if (raw_adc_val > 3200) {
        temp = 'B';
    } else if (raw_adc_val > 3120) {
        temp = '7';
    } else if (raw_adc_val > 3070) {
        temp = '8';
    } else if (raw_adc_val > 3020) {
        temp = '9';
    } else if (raw_adc_val > 2960) {
        temp = 'C';
    } else if (raw_adc_val > 2900) {
        temp = '*';
    } else if (raw_adc_val > 2850) {
        temp = '0';
    } else if (raw_adc_val > 2810) {
        temp = '#';
    } else if (raw_adc_val > 2750) {
        temp = 'D';
    }
    return temp;
}
