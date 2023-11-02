#include <stdint.h>
#include "lpc824.h"

void delay(uint32_t counts);

int main(void) {
    SYSCON_SYSAHBCLKCTRL |= (1 << 6);
    GPIO_DIR0 |= (1 << 7);
    GPIO_DIR0 |= (1 << 4);
    GPIO_DIR0 |= (1 << 28);
    GPIO_DIR0 &= ~(1 << 20);
    GPIO_DIR0 &= ~(1 << 11);

    volatile unsigned char state = 0;
    uint8_t button1_state = 1;
    uint8_t button2_state = 1;

    
    GPIO_B7 = 0;
    GPIO_B4 = 1;
    GPIO_B28 = 0;
    state = 0;

    while (1) {
          
        if (GPIO_B20 == 0 && !button1_state) {
            button1_state = 1;  // Buton 1 basıldı olarak işaretle

            state = (state + 1) ;  // Bir sonraki duruma geç
            if (state > 4)
            {
                state = state-5;
            }
        } else if (GPIO_B20 == 1) {
            button1_state = 0;  // Buton 1 serbest bırakıldı olarak işaretle
        }

        if (GPIO_B11 == 0 && !button2_state) {
            button2_state = 1;  // Buton 2 basıldı olarak işaretle

            state = (state + 2);  // İkinci düğme ile 2 durumu ilerlet
             if (state > 4)
            {
                state = state-5;
            }
        } else if (GPIO_B11 == 1) {
            button2_state = 0;  // Buton 2 serbest bırakıldı olarak işaretle
        }

        // State actions
        switch (state) {
            case 0:
                GPIO_B4 = 1;
                GPIO_B28 = 0;
                GPIO_B7 = 0;
                delay(100000);
                break;
            case 1:
                GPIO_B7 = 1;
                GPIO_B4 = 0;
                GPIO_B28 = 0;
                delay(100000);
                break;
            case 2:
                GPIO_B7 = 0;
                GPIO_B4 = 0;
                GPIO_B28 = 1;
                delay(100000);
                break;
            case 3:
                GPIO_B7 = 0;
                GPIO_B4 = 0;
                GPIO_B28 = 0;
                delay(100000);
                break;
            case 4:
                GPIO_B7 = 1;
                GPIO_B4 = 1;
                GPIO_B28 = 1;
                delay(100000);
                break;
            default:
                break;
        }
    }

    return 0;
}

void delay(uint32_t counts) {
    uint32_t wait;
    for (wait = counts; wait > 0; --wait) {}
}