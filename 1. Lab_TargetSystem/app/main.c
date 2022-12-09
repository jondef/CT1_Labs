#include "utils_ctboard.h"



#define S0  0x60000200
#define L0  0x60000100
#define P11 0x60000211
#define DS0 0x60000110



const uint8_t patterns[16] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E};



int main(void) {

    while (1) {
        int inputValue = read_word(S0);
        write_word(L0, inputValue);
        
        int rotaryValue = read_word(P11);
        int rotaryValueLowerNibble = rotaryValue & 0x0F;
     
        write_byte(DS0, patterns[rotaryValueLowerNibble]);
}
}