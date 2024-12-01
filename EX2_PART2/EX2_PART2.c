#include <stdbool.h>
#include <stdint.h>
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_mrt.h"
#include "fsl_sctimer.h"

#define CORE_CLOCK 30000000U // Set CPU Core clock frequency (Hz)

#define DESIRED_INT_FREQ 100 // Desired number of INT's per second.

volatile uint32_t counter = 0; // Variable that counts to 50 in ISR

// Declare variables for event handling
uint32_t event_red, event_yellow, event_green;
uint32_t sctimerClock;  // For SCTIMER clock frequency
sctimer_config_t sctimerConfig;
sctimer_pwm_signal_param_t pwm_signal_red, pwm_signal_yellow, pwm_signal_green;  // PWM signals for LEDs
uint32_t pwm_frequency = 1000;  // 1 kHz PWM frequency
volatile uint8_t Duty1 = 0, Duty2 = 60, Duty3 = 100;  // Duty cycles

void MRT0_IRQHandler(void);
void Duty_Changee(uint8_t Duty, uint32_t *Freq, uint32_t event, uint32_t sctimerClock, sctimer_config_t *sctimerConfig, sctimer_pwm_signal_param_t *pwm_signal);

int main(void)
{
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Sct);
    CLOCK_EnableClock(kCLOCK_Mrt);

    // Initialize SCT clock
    sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(&sctimerConfig);
    SCTIMER_Init(SCT0, &sctimerConfig);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

    // Configure SCT PWM for Red LED
    pwm_signal_red.output = kSCTIMER_Out_2;
    pwm_signal_red.dutyCyclePercent = Duty1;  // Initial duty cycle (0%)
    pwm_signal_red.level = kSCTIMER_HighTrue;
    SCTIMER_SetupPwm(SCT0, &pwm_signal_red, kSCTIMER_EdgeAlignedPwm, pwm_frequency, sctimerClock, &event_red);

    // Configure SCT PWM for Yellow LED
    pwm_signal_yellow.output = kSCTIMER_Out_3;
    pwm_signal_yellow.dutyCyclePercent = Duty1;  // Initial duty cycle (0%)
    pwm_signal_yellow.level = kSCTIMER_HighTrue;
    SCTIMER_SetupPwm(SCT0, &pwm_signal_yellow, kSCTIMER_EdgeAlignedPwm, pwm_frequency, sctimerClock, &event_yellow);

    // Configure SCT PWM for Green LED
    pwm_signal_green.output = kSCTIMER_Out_4;
    pwm_signal_green.dutyCyclePercent = Duty1;  // Initial duty cycle (0%)
    pwm_signal_green.level = kSCTIMER_HighTrue;
    SCTIMER_SetupPwm(SCT0, &pwm_signal_green, kSCTIMER_EdgeAlignedPwm, pwm_frequency, sctimerClock, &event_green);

    // MRT config
    mrt_config_t mrtConfig;
    MRT_GetDefaultConfig(&mrtConfig);
    MRT_Init(MRT0, &mrtConfig);
    MRT_SetupChannelMode(MRT0, kMRT_Channel_0, kMRT_RepeatMode);

    uint32_t mrt_clock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    uint32_t mrt_count_val = mrt_clock / DESIRED_INT_FREQ;

    MRT_StartTimer(MRT0, kMRT_Channel_0, mrt_count_val);
    MRT_EnableInterrupts(MRT0, kMRT_Channel_0, kMRT_TimerInterruptEnable);

    EnableIRQ(MRT0_IRQn);

    while (1)
    {
        pwm_signal_red.dutyCyclePercent = Duty3;
        SCTIMER_SetupPwm(SCT0, &pwm_signal_red, kSCTIMER_EdgeAlignedPwm, pwm_frequency, sctimerClock, &event_red);

        //__WFI(); // Waiting for interrupt
    }
}

///////////////////////////////////////////////////////////////////////
// This is the MRT interrupt service routine. /////////////////////////
///////////////////////////////////////////////////////////////////////

void MRT0_IRQHandler(void)
{
    MRT_ClearStatusFlags(MRT0, kMRT_Channel_0, kMRT_TimerInterruptFlag);
/*
    // Change the PWM duty cycle and switch LEDs
    if (counter == 0)
    {
        Duty_Changee(Duty2, &pwm_frequency, event_red, sctimerClock, &sctimerConfig, &pwm_signal_red);  // Red LED off
        Duty_Changee(Duty1, &pwm_frequency, event_green, sctimerClock, &sctimerConfig, &pwm_signal_green);  // Green LED on
    }
    else if (counter == 50)
    {
        Duty_Changee(Duty2, &pwm_frequency, event_green, sctimerClock, &sctimerConfig, &pwm_signal_green);  // Green LED off
        Duty_Changee(Duty1, &pwm_frequency, event_yellow, sctimerClock, &sctimerConfig, &pwm_signal_yellow);  // Yellow LED on
    }
    else if (counter == 100)
    {
        Duty_Changee(Duty2, &pwm_frequency, event_yellow, sctimerClock, &sctimerConfig, &pwm_signal_yellow);  // Yellow LED off
        Duty_Changee(Duty1, &pwm_frequency, event_red, sctimerClock, &sctimerConfig, &pwm_signal_red);  // Red LED on
    }

    counter = (counter + 1) % 150;  // Reset counter after 150 iterations
    */
}

void Duty_Changee(uint8_t Duty, uint32_t *Freq, uint32_t event, uint32_t sctimerClock, sctimer_config_t *sctimerConfig, sctimer_pwm_signal_param_t *pwm_signal)
{
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);

    // Update SCTimer configuration with the new settings
    SCTIMER_GetDefaultConfig(sctimerConfig);
    SCTIMER_Init(SCT0, sctimerConfig);

    pwm_signal->dutyCyclePercent = Duty;  // Set the duty cycle
    SCTIMER_SetupPwm(SCT0, pwm_signal, kSCTIMER_EdgeAlignedPwm, *Freq, sctimerClock, &event);  // Apply PWM configuration

    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);  // Start the SCTimer
}
