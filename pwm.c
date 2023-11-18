#include "fsl_sctimer.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "board.h"
#include <stdint.h>
#include <stdbool.h>
#include "lpc824.h"
#include "stdio.h"

void delay(int counts);

// Function to introduce a delay for debouncing
void debounceDelay() {
    delay(100000); // Adjust the delay value as needed
}

int main(void) {
    sctimer_config_t sctimerConfig;
    sctimer_pwm_signal_param_t pwmParam0, pwmParam1, pwmParam2;
    uint32_t event0, event1, event2;
    uint32_t sctimerClock;
    uint8_t state = 0; // 0: Red LED, 1: Yellow LED, 2: Green LED
    uint8_t prevDutyCycle0 = 85; // Initial value for Red LED
    uint8_t prevDutyCycle1 = 100; // Initial value for Yellow LED
    uint8_t prevDutyCycle2 = 100; // Initial value for Green LED

    BOARD_InitBootPins();       // Initialize board pins.
    BOARD_InitBootClocks();     // Initialize clock signals.

    CLOCK_EnableClock(kCLOCK_Sct);              // Enable clock of SCTimer peripheral
    sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);   // The SCTimer gets 12 MHz frequency
    SCTIMER_GetDefaultConfig(&sctimerConfig);   // Set it to a default value. See the library function.
    SCTIMER_Init(SCT0, &sctimerConfig);         // Initialize SCTimer module:

    pwmParam0.output = kSCTIMER_Out_0;    // This is the Red LED.
    pwmParam0.level = kSCTIMER_LowTrue;   // High duty ratio will produce a lower average signal.
    pwmParam0.dutyCyclePercent = prevDutyCycle0; // Use the initial value
    // Configure the PWM for Red LED.
    SCTIMER_SetupPwm(SCT0,                  // Use SCT0 timer.
                     &pwmParam0,            // Use the pwmParam struct.
                     kSCTIMER_CenterAlignedPwm,  // Generate center aligned PWM
                     1000U,                        // Freq is 1kHz
                     sctimerClock,                 // Use the clock from sctimer
                     &event0);

    pwmParam1.output = kSCTIMER_Out_1;    // This is the Green LED.
    pwmParam1.level = kSCTIMER_LowTrue;   // High duty ratio will produce a lower average signal.
    pwmParam1.dutyCyclePercent = prevDutyCycle1; // Use the initial value
    // Configure the PWM for Green LED.
    SCTIMER_SetupPwm(SCT0,                  // Use SCT0 timer.
                     &pwmParam1,            // Use the pwmParam struct.
                     kSCTIMER_CenterAlignedPwm,  // Generate center aligned PWM
                     1000U,                        // Freq is 1kHz
                     sctimerClock,                 // Use the clock from sctimer
                     &event1);

    pwmParam2.output = kSCTIMER_Out_2;    // This is the Blue LED.
    pwmParam2.level = kSCTIMER_LowTrue;   // High duty ratio will produce a lower average signal.
    pwmParam2.dutyCyclePercent = prevDutyCycle2; // Use the initial value
    // Configure the PWM for Blue LED.
    SCTIMER_SetupPwm(SCT0,                  // Use SCT0 timer.
                     &pwmParam2,            // Use the pwmParam struct.
                     kSCTIMER_CenterAlignedPwm,  // Generate center aligned PWM
                     1000U,                        // Freq is 1kHz
                     sctimerClock,                 // Use the clock from sctimer
                     &event2);
    // Initialize LED duty cycles
    //pwmParam0.dutyCyclePercent = 15; // Red LED
    //pwmParam1.dutyCyclePercent = 0;  // Yellow LED
    //pwmParam2.dutyCyclePercent = 0;  // Green LED

    // Configure PWM signals with frequency 1kHz
    SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_0, pwmParam0.dutyCyclePercent, event0);
    SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_1, pwmParam1.dutyCyclePercent, event1);
    SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_2, pwmParam2.dutyCyclePercent, event2);

    // ...

    while (1) {
        // If button 1 is pressed once
        if (GPIO_PinRead(GPIO, 0, 19)) {
            debounceDelay(); // Introduce a delay for debouncing

            while (GPIO_PinRead(GPIO, 0, 19)) {}

            // Increase or decrease PWM duty cycle ratios by 5% based on the current state
            switch (state) {
                case 0:
                    prevDutyCycle0 = pwmParam0.dutyCyclePercent;
                    pwmParam0.dutyCyclePercent = (pwmParam0.dutyCyclePercent < 90U) ? (pwmParam0.dutyCyclePercent + 5U) : 90U;
                    SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_0, pwmParam0.dutyCyclePercent, event0);
                    delay(1000000);
                break;
                case 1:
                    prevDutyCycle1 = pwmParam1.dutyCyclePercent;
                    pwmParam1.dutyCyclePercent = (pwmParam1.dutyCyclePercent < 90U) ? (pwmParam1.dutyCyclePercent + 5U) : 90U;
                    SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_1, pwmParam1.dutyCyclePercent, event1);
                    delay(1000000);
                break;
                case 2:
                    prevDutyCycle2 = pwmParam2.dutyCyclePercent;
                    pwmParam2.dutyCyclePercent = (pwmParam2.dutyCyclePercent < 90U) ? (pwmParam2.dutyCyclePercent + 5U) : 90U;
                    SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_2, pwmParam2.dutyCyclePercent, event2);
                    delay(1000000);
                    break;
                    }
                    }
                    // If button 2 is pressed once
                    else if (GPIO_PinRead(GPIO, 0, 20)) {
                    debounceDelay(); // Introduce a delay for debouncing
        while (GPIO_PinRead(GPIO, 0, 20)) {}

        // Decrease PWM duty cycle ratios by 5% down to 0% based on the current state
        switch (state) {
            case 0:
                prevDutyCycle0 = pwmParam0.dutyCyclePercent;
                pwmParam0.dutyCyclePercent = (pwmParam0.dutyCyclePercent > 40U) ? (pwmParam0.dutyCyclePercent - 5U) : 40U;
                SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_0, pwmParam0.dutyCyclePercent, event0);
                delay(1000000);
                break;
            case 1:
                prevDutyCycle1 = pwmParam1.dutyCyclePercent;
                pwmParam1.dutyCyclePercent = (pwmParam1.dutyCyclePercent > 40U) ? (pwmParam1.dutyCyclePercent - 5U) : 40U;
                SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_1, pwmParam1.dutyCyclePercent, event1);
                delay(1000000);
                break;
            case 2:
                prevDutyCycle2 = pwmParam2.dutyCyclePercent;
                pwmParam2.dutyCyclePercent = (pwmParam2.dutyCyclePercent > 40U) ? (pwmParam2.dutyCyclePercent - 5U) : 40U;
                SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_2, pwmParam2.dutyCyclePercent, event2);
                delay(1000000);
                break;
        }
    }
    // If button 3 is pressed once
    else if (GPIO_PinRead(GPIO, 0, 21)) {
        debounceDelay(); // Introduce a delay for debouncing

        while (GPIO_PinRead(GPIO, 0, 21)) {}

        state = (state + 1); // Use modulo to cycle through states
        if (state > 2) {
            state = state - 3;
        }

        switch (state) {
            case 0:
                SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_0, pwmParam0.dutyCyclePercent, event0);
                delay(1000000);
                break;
            case 1:
                SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_1, pwmParam1.dutyCyclePercent, event1);
                delay(1000000);
                break;
            case 2:
                SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_2, pwmParam2.dutyCyclePercent, event2);
                delay(1000000);
                break;
        }

        // Set other LED duty cycles to 0
        pwmParam0.dutyCyclePercent = (state == 0) ? prevDutyCycle0 : 100; // 15% for Red LED initially
        pwmParam1.dutyCyclePercent = (state == 1) ? prevDutyCycle1 : 100;
        pwmParam2.dutyCyclePercent = (state == 2) ? prevDutyCycle2 : 100;

        // Update PWM outputs based on the current state
        SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_0, pwmParam0.dutyCyclePercent, event0);
        SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_1, pwmParam1.dutyCyclePercent, event1);
        SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_2, pwmParam2.dutyCyclePercent, event2);
    }
    //
}
}
void delay(int counts){
        int wait;
        for (wait = counts; wait > 0; --wait) {}
}