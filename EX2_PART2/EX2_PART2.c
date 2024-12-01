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
#define RED_LED_PIN 13U
#define YELLOW_LED_PIN 17U
#define GREEN_LED_PIN 18U

#define LED_ON 1U // LED will be ON when GPIO output is '1'
#define LED_OFF 0U

// PWM parameters
volatile uint32_t sctimerClock;
sctimer_config_t sctimerConfig;
sctimer_pwm_signal_param_t pwm_signal_red, pwm_signal_yellow, pwm_signal_green; // pwm signals for every led
uint32_t event;
volatile uint8_t Duty1 = 60, Duty2 = 60, Duty3 = 60; // 60% brightness
volatile uint32_t Freq1 = 1000, Freq2 = 1000, Freq3 = 1000; // LED's frequency

volatile uint32_t counter = 0; // variable that counts to 50 in Interrupt Service Routine

void MRT0_IRQHandler(void);
void clock_init(void);
volatile uint32_t millis = 0; // millisecond counter
void SysTick_Handler(void);   // our systick interrupt handler
void delay_ms(uint32_t ms);   // delay (ms)

int main(void)
{ 
    CLOCK_EnableClock(kCLOCK_Gpio0);
    
    // LED struct and config
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};
    GPIO_PinInit(GPIO, LED_PORT, GREEN_LED_PIN, &led_config);
    GPIO_PinInit(GPIO, LED_PORT, YELLOW_LED_PIN, &led_config);
    GPIO_PinInit(GPIO, LED_PORT, RED_LED_PIN, &led_config);

    // Turning off the leds 
    GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
    GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_OFF);
    GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_OFF);

    // Configure SCT for PWM output
    CLOCK_EnableClock(kCLOCK_Mrt);
    CLOCK_EnableClock(kCLOCK_Sct);

    // Initialize SCT timer and configure PWM for LED control
    sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);  // Use IRC clock for SCT
    SCTIMER_GetDefaultConfig(&sctimerConfig);
    SCTIMER_Init(SCT0, &sctimerConfig);

    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

    // Configure and setup PWM signal for each LED
    pwm_signal_red.output = kSCTIMER_Out_2;   // Red LED connected to Out0
    pwm_signal_yellow.output = kSCTIMER_Out_3; // Yellow LED connected to Out1
    pwm_signal_green.output = kSCTIMER_Out_4;  // Green LED connected to Out2
    
    pwm_signal_red.dutyCyclePercent = Duty1;   // Set initial duty cycle (60%)
    pwm_signal_yellow.dutyCyclePercent = Duty2; // Set initial duty cycle (60%)
    pwm_signal_green.dutyCyclePercent = Duty3;  // Set initial duty cycle (60%)

    // PWM frequency = 1kHz (Periyot hesaplama)
    uint32_t pwm_frequency = 1000; // 1 kHz PWM freq

    // PWM çıkışlarını başlat
    SCTIMER_SetupPwm(SCT0, &pwm_signal_red);   // Red LED PWM setup
    SCTIMER_SetupPwm(SCT0, &pwm_signal_yellow); // Yellow LED PWM setup
    SCTIMER_SetupPwm(SCT0, &pwm_signal_green);  // Green LED PWM setup
    
    // Set up the MRT for interrupt (timing control)
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
        __WFI(); // waiting for interrupt
    } // END while (1)
} // END main()

///////////////////////////////////////////////////////////////////////
// This is the MRT interrupt service routine. /////////////////////////
///////////////////////////////////////////////////////////////////////

void MRT0_IRQHandler(void)
{
    MRT_ClearStatusFlags(MRT0, kMRT_Channel_0, kMRT_TimerInterruptFlag);

    if (counter == 0) {
        GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_ON);
        
        pwm_signal_red.dutyCyclePercent = 0; 
        SCTIMER_SetupPwm(SCT0, &pwm_signal_red);   
    } 
    else if (counter == 50) {
        GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_ON);
        
  
        pwm_signal_green.dutyCyclePercent = 0; 
        SCTIMER_SetupPwm(SCT0, &pwm_signal_green); 
    } 
    else if (counter == 100) {
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_ON);
        
        
        pwm_signal_yellow.dutyCyclePercent = 0; 
        SCTIMER_SetupPwm(SCT0, &pwm_signal_yellow); 
    }
    else if (counter >= 150)
    {
        counter = -1;
    }

 
    if (counter == 0) {
        pwm_signal_green.dutyCyclePercent = 60; 
        SCTIMER_SetupPwm(SCT0, &pwm_signal_green); 
    } 
    else if (counter == 50) {
        pwm_signal_yellow.dutyCyclePercent = 60; 
        SCTIMER_SetupPwm(SCT0, &pwm_signal_yellow); 
    } 
    else if (counter == 100) {
        pwm_signal_red.dutyCyclePercent = 60; 
        SCTIMER_SetupPwm(SCT0, &pwm_signal_red); 
    }
    counter++;
}

void Duty_Changee(volatile uint8_t *Duty, volatile uint32_t *Freq, uint32_t *const event, volatile uint32_t *sctimerClock, sctimer_config_t *const sctimerConfig, sctimer_pwm_signal_param_t *const p_1)
{
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);
    *sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(sctimerConfig);
    SCTIMER_Init(SCT0, sctimerConfig);

    p_1->dutyCycle = *Duty; 
    SCTIMER_SetupPWM(SCT0, p_1); 
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U); 
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
