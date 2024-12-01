#include <stdbool.h>
#include <stdint.h>
#include "lpc824.h"
#include "pin_mux.h"
#include "fsl_mrt.h"
#include "fsl_power.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_swm.h"
#include "fsl_sctimer.h"

#define CORE_CLOCK 30000000U // Set CPU Core clock frequency (Hz)
#define DESIRED_INT_FREQ 100 // Desired number of INT's per second.

#define LED_PORT 0U
#define RED_LED_PIN 13U
#define YELLOW_LED_PIN 17U
#define GREEN_LED_PIN 18U

#define LED_ON 1U // LED will be ON when GPIO output is '1'
#define LED_OFF 0U

// PWM parameters

volatile uint32_t sctimerClock;
sctimer_config_t sctimerConfig;
sctimer_pwm_signal_param_t pwm_signal_red, pwm_signal_yellow, pwm_signal_green; // PWM signals for every LED

volatile uint32_t counter = 0; // Variable that counts to 50 in ISR
volatile uint32_t millis = 0; // Millisecond counter

void MRT0_IRQHandler(void);
void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void clock_init(void);
void Duty_Changee(volatile uint8_t duty, uint32_t pwmFreq, uint32_t srcClock, uint32_t *event, sctimer_pwm_signal_param_t *pwm_signal);

int main(void)
{
    uint32_t pwm_frequency = 1000; // 1 kHz PWM freq
    uint8_t Duty1 = 60 ,Duty2 = 0;
    uint32_t event_red, event_yellow, event_green;

    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Sct);
    CLOCK_EnableClock(kCLOCK_Mrt);

    // LED struct and config
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};
    GPIO_PinInit(GPIO, LED_PORT, GREEN_LED_PIN, &led_config);
    GPIO_PinInit(GPIO, LED_PORT, YELLOW_LED_PIN, &led_config);
    GPIO_PinInit(GPIO, LED_PORT, RED_LED_PIN, &led_config);

    // Turning off the leds 
    GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
    GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_OFF);
    GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_OFF);

    // Initialize SCT clock
    sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(&sctimerConfig);
    SCTIMER_Init(SCT0, &sctimerConfig);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

    // Configure SCT PWM for Red LED
    pwm_signal_red.output = kSCTIMER_Out_2;
    pwm_signal_red.dutyCyclePercent = Duty2;  // Initial duty cycle (0%)
    pwm_signal_red.level = kSCTIMER_HighTrue;
    SCTIMER_SetupPwm(SCT0, &pwm_signal_red, kSCTIMER_EdgeAlignedPwm, pwm_frequency, sctimerClock, &event_red);

    // Configure SCT PWM for Yellow LED
    pwm_signal_yellow.output = kSCTIMER_Out_3;
    pwm_signal_yellow.dutyCyclePercent = Duty2;  // Initial duty cycle (0%)
    pwm_signal_yellow.level = kSCTIMER_HighTrue;
    SCTIMER_SetupPwm(SCT0, &pwm_signal_yellow, kSCTIMER_EdgeAlignedPwm, pwm_frequency, sctimerClock, &event_yellow);

    // Configure SCT PWM for Green LED
    pwm_signal_green.output = kSCTIMER_Out_4;
    pwm_signal_green.dutyCyclePercent = Duty2;  // Initial duty cycle (0%)
    pwm_signal_green.level = kSCTIMER_HighTrue;
    SCTIMER_SetupPwm(SCT0, &pwm_signal_green, kSCTIMER_EdgeAlignedPwm, pwm_frequency, sctimerClock, &event_green);

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
        Duty_Changee(&Duty2, &pwm_frequency, &event_red, &sctimerClock, &sctimerConfig, &pwm_signal_red); // Turn off Red LED
        Duty_Changee(&Duty1, &pwm_frequency, &event_green, &sctimerClock, &sctimerConfig, &pwm_signal_green); // Turn on Green LED

    }
    else if (counter == 50)
    {
        Duty_Changee(&Duty2, &pwm_frequency, &event_green, &sctimerClock, &sctimerConfig, &pwm_signal_green); // Turn off Green LED
        Duty_Changee(&Duty1, &pwm_frequency, &event_yellow, &sctimerClock, &sctimerConfig, &pwm_signal_yellow); // Turn on Yellow LED
    }
    else if (counter == 100)
    {
        Duty_Changee(&Duty2, &pwm_frequency, &event_yellow, &sctimerClock, &sctimerConfig, &pwm_signal_yellow); // Turn off Yellow LED
        Duty_Changee(&Duty1, &pwm_frequency, &event_red, &sctimerClock, &sctimerConfig, &pwm_signal_red); // Turn on Red LED
    }
    

    counter = (counter + 1) % 150; // Cycle through every 150 ticks
}

void Duty_Changee(volatile uint8_t *Duty, volatile uint32_t *Freq, uint32_t *const event, volatile uint32_t *sctimerClock, sctimer_config_t *const sctimerConfig, sctimer_pwm_signal_param_t *const pwm_signal)
{
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);
    *sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(sctimerConfig);
    SCTIMER_Init(SCT0, sctimerConfig);

    pwm_signal.dutyCyclePercent = *Duty; 
    SCTIMER_SetupPwm(SCT0, pwm_signal); 
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U); 
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
{
    millis++;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis;
    while ((millis - start) < ms)
        ;
}
