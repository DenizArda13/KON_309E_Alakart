#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "lpc824.h"
#include "pin_mux.h"
#include "fsl_mrt.h"
#include "fsl_power.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_syscon.h"

#include "fsl_swm.h"
#include "fsl_swm_connections.h"
#include "fsl_sctimer.h"
#include "fsl_reset.h"
#include "fsl_pint.h"
#include "fsl_common.h"
#include "fsl_common_arm.h"
#include "fsl_debug_console.h"

#define CORE_CLOCK 30000000U // Set CPU Core clock frequency (Hz)

#define DESIRED_INT_FREQ 100 // Desired number of INT's per second.

#define LED_PORT 0U

volatile uint32_t counter = 0; // Variable that counts to 50 in ISR
volatile uint32_t millis = 0; // millisecond counter

uint32_t event;
volatile uint32_t sctimerClock;  // For SCTIMER clock frequency
sctimer_config_t sctimerConfig;
sctimer_pwm_signal_param_t pwmParam[3];  // PWM signals for LEDs
uint32_t pwm_frequency = 1000U;  // 1 kHz PWM frequency
volatile uint8_t Duty1 = 0x00, Duty2 = 0x60;  // Duty cycles

void clock_init(void);
void SysTick_Handler(void);   // our systick interrupt handler
void delay_ms(uint32_t ms);   // delay (ms)

void MRT0_IRQHandler(void);
void Duty_Changee(volatile uint8_t *Duty, uint32_t *Freq, uint32_t *const event,volatile uint32_t *sctimerClock, sctimer_config_t *const sctimerConfig, sctimer_pwm_signal_param_t *p_1);

int main(void)
{
    GPIO_PortInit(GPIO, LED_PORT);

    InitPins();
    clock_init();

    SysTick_Config(CORE_CLOCK / 1000); // setup systick clock interrupt @1ms

    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Sct);
    CLOCK_EnableClock(kCLOCK_Mrt);

    // Initialize SCT clock
    sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(&sctimerConfig);
    SCTIMER_Init(SCT0, &sctimerConfig);

    // Configure SCT PWM for Red LED
    pwmParam[0].output = kSCTIMER_Out_2;
    pwmParam[0].level = kSCTIMER_HighTrue;
    pwmParam[0].dutyCyclePercent = Duty1;
    SCTIMER_SetupPwm(SCT0, &pwmParam[0], kSCTIMER_CenterAlignedPwm, 1000U, sctimerClock, &event);

    // Configure SCT PWM for Yellow LED
    pwmParam[1].output = kSCTIMER_Out_3;
    pwmParam[1].level = kSCTIMER_HighTrue;
    pwmParam[1].dutyCyclePercent = Duty1;
    SCTIMER_SetupPwm(SCT0, &pwmParam[1], kSCTIMER_CenterAlignedPwm, 1000U, sctimerClock, &event);

    // Configure SCT PWM for Green LED
    pwmParam[2].output = kSCTIMER_Out_4;
    pwmParam[2].level = kSCTIMER_HighTrue;
    pwmParam[2].dutyCyclePercent = Duty1;
    SCTIMER_SetupPwm(SCT0, &pwmParam[2], kSCTIMER_CenterAlignedPwm, 1000U, sctimerClock, &event);

    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

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
        __WFI(); // Waiting for interrupt
    }
}

///////////////////////////////////////////////////////////////////////
// This is the MRT interrupt service routine. /////////////////////////
///////////////////////////////////////////////////////////////////////

void MRT0_IRQHandler(void)
{
    MRT_ClearStatusFlags(MRT0, kMRT_Channel_0, kMRT_TimerInterruptFlag);

    // Change the PWM duty cycle and switch LEDs
    if (counter == 0)
    {
        Duty_Changee(&Duty1, &pwm_frequency, &event, &sctimerClock, &sctimerConfig, &pwmParam[1]);  // Yellow LED off
        Duty_Changee(&Duty2, &pwm_frequency, &event, &sctimerClock, &sctimerConfig, &pwmParam[0]);  // Red LED on        
    }
    else if (counter == 50)
    {
        Duty_Changee(&Duty1, &pwm_frequency, &event, &sctimerClock, &sctimerConfig, &pwmParam[0]);  // Red LED off
        Duty_Changee(&Duty2, &pwm_frequency, &event, &sctimerClock, &sctimerConfig, &pwmParam[2]);  // Green LED on
    }
    else if (counter == 100)
    {
        Duty_Changee(&Duty1, &pwm_frequency, &event, &sctimerClock, &sctimerConfig, &pwmParam[2]);  // Green LED off
        Duty_Changee(&Duty2, &pwm_frequency, &event, &sctimerClock, &sctimerConfig, &pwmParam[1]);  // Yellow LED on
    }
    else if (counter >= 150)
    {
        counter = -1;
    }
    counter++;
    
}

void Duty_Changee(volatile uint8_t *Duty, uint32_t *Freq, uint32_t *event,volatile uint32_t *sctimerClock, sctimer_config_t *const sctimerConfig, sctimer_pwm_signal_param_t *p_1)
{
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);
    *sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(sctimerConfig);
    SCTIMER_Init(SCT0, sctimerConfig);

    p_1->dutyCyclePercent = *Duty;  // Set the duty cycle
    SCTIMER_SetupPwm(SCT0, p_1, kSCTIMER_CenterAlignedPwm, *Freq, *sctimerClock, event);  // Apply PWM configuration

    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);  // Start the SCTimer
}

void clock_init(void)
{

    // Set up using Internal RC clock (IRC) oscillator:
    POWER_DisablePD(kPDRUNCFG_PD_IRC_OUT); // Turn ON IRC OUT
    POWER_DisablePD(kPDRUNCFG_PD_IRC);     // Turn ON IRC

    CLOCK_Select(kSYSPLL_From_Irc); // Connect IRC to PLL input.

    clock_sys_pll_t config;
    config.src = kCLOCK_SysPllSrcIrc;   // Select PLL source as IRC.
    config.targetFreq = CORE_CLOCK * 2; // set pll target freq

    CLOCK_InitSystemPll(&config); // set parameters

    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcSysPll); // Select PLL as main clock source.
    CLOCK_Select(kCLKOUT_From_Irc);               // select IRC for CLKOUT
    CLOCK_SetCoreSysClkDiv(1U);

    // Check processor registers and calculate the
    // Actual clock speed. This is stored in the
    // global variable SystemCoreClock
    SystemCoreClockUpdate();

}

void SysTick_Handler(void)
{ // our systick interrupt handler
    millis++;
}

void delay_ms(uint32_t ms)
{ // delay (ms)
    uint32_t now = millis;
    while ((millis - now) < ms)
        ;
}

