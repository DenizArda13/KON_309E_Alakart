#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

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
#define GREEN_LED_PIN 7U 
#define YELLOW_LED_PIN 8U
#define RED_LED_PIN 9U
#define BUTTON_PIN 10U


#define USART_INSTANCE 0U
#define USART_BAUDRATE 115200

void clock_init(void);
volatile uint32_t millis = 0; // millisecond counter
void SysTick_Handler(void);   // our systick interrupt handler
void delay_ms(uint32_t ms);   // delay (ms)
void Duty_Changee(volatile uint8_t *Duty, volatile uint32_t *Freq, uint32_t *const event, volatile uint32_t *sctimerClock, sctimer_config_t *const sctimerConfig, sctimer_pwm_signal_param_t *const p_1);

static volatile bool mrtIsrFlag = false; // MRT ISR sets this flag to true.
static volatile bool on_off = false;
volatile uint8_t Duty1, Duty2, Duty3;
volatile uint32_t Freq1, Freq2, Freq3;

sctimer_config_t sctimerConfig;
uint32_t event;
volatile uint32_t sctimerClock;
uint8_t brightness[4] = {0x04, 0x10, 0x40, 0x63};
uint32_t freq[4] = {10000,5000,2000,1000};

void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    static int i = 0;
    static bool flag = false;
    /* Required changes*/

    flag = !flag;

    // PINT_PinInterruptClrRiseFlagAll(PINT);
    PINT_PinInterruptClrFallFlagAll(PINT);
}

int main(void)
{
    uint32_t mrt_clock;
    uint32_t mrt_count_val;
    // This struct stores several parameters of SCTimer:

    // This struct stores several parameters of the  PWM signal:

    mrt_config_t mrtConfig; // Struct for configuring the MRT:

    InitPins();
    clock_init();

    SysTick_Config(CORE_CLOCK / 1000); // setup systick clock interrupt @1ms
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt1, kSYSCON_GpioPort0Pin25ToPintsel);

    PINT_Init(PINT); // Initialize PIN Interrupts

    // Setup Pin Interrupt 1:
    //  falling edge triggers the INT
    //  register the name of the callback function (as the last argument).

    PINT_PinInterruptConfig(PINT,                       // Pin INT base register address
                            kPINT_PinInt1,              // Use Pin INT 1
                            kPINT_PinIntEnableFallEdge, // At falling edge.
                            pint_intr_callback);        // Name of the callback function

    // Enable callbacks for PINT0 by Index:
    // This clears the pending INT flags
    //  and enables the interrupts for this PIN INT.
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt1);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Mrt);
    CLOCK_EnableClock(kCLOCK_Sct);
    sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(&sctimerConfig);
    SCTIMER_Init(SCT0, &sctimerConfig);
    
    Duty1 = brightness[0];
    Duty2 = 0x00;
    Duty3 = 0x00;

    pwmParam[0].output = kSCTIMER_Out_2;
    pwmParam[0].level = kSCTIMER_HighTrue;
    pwmParam[0].dutyCyclePercent = Duty1;

    SCTIMER_SetupPwm(SCT0,                      // Use SCT0 timer.
                     &pwmParam[0],              // Use the pwmParam struct.
                     kSCTIMER_CenterAlignedPwm, // Generate center aligned PWM
                     1U,
                     sctimerClock, // Use the clock from sctimer
                     &event);

    pwmParam[1].output = kSCTIMER_Out_3;
    pwmParam[1].level = kSCTIMER_HighTrue;
    pwmParam[1].dutyCyclePercent = Duty2;

    SCTIMER_SetupPwm(SCT0,                      // Use SCT0 timer.
                     &pwmParam[1],              // Use the pwmParam struct.
                     kSCTIMER_CenterAlignedPwm, // Generate center aligned PWM
                     1U,
                     sctimerClock, // Use the clock from sctimer
                     &event);

    pwmParam[2].output = kSCTIMER_Out_4;
    pwmParam[2].level = kSCTIMER_HighTrue;
    pwmParam[2].dutyCyclePercent = Duty3;

    SCTIMER_SetupPwm(SCT0,                      // Use SCT0 timer.
                     &pwmParam[2],              // Use the pwmParam struct.
                     kSCTIMER_CenterAlignedPwm, // Generate center aligned PWM
                     1U,
                     sctimerClock, // Use the clock from sctimer
                     &event);

    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

    MRT_GetDefaultConfig(&mrtConfig);
    MRT_Init(MRT0, &mrtConfig);
    MRT_SetupChannelMode(MRT0, kMRT_Channel_0, kMRT_RepeatMode);

    mrt_clock = CLOCK_GetFreq(kCLOCK_CoreSysClk);

    mrt_count_val = mrt_clock / DESIRED_INT_FREQ;

    MRT_StartTimer(MRT0, kMRT_Channel_0, mrt_count_val);
    MRT_EnableInterrupts(MRT0, kMRT_Channel_0, kMRT_TimerInterruptEnable);
    EnableIRQ(MRT0_IRQn);

    while (1)
    {
        __WFI();
    } // END while (1)

} // END main()

///////////////////////////////////////////////////////////////////////
////////// This is the MRT interrupt service routine. /////////////////
///////////////////////////////////////////////////////////////////////

// It was declared in the file startup_LPC824.S
// as the 10th entry of the vector table.
// See Table 5 of Sec. 4.3.1 Interrupt sources

void MRT0_IRQHandler(void)
{
    static uint8_t i = 0;
    MRT_ClearStatusFlags(MRT0, // Clear interrupt flag:
                         kMRT_Channel_0,
                         kMRT_TimerInterruptFlag);
}

// Setup processor clock source:
// Internal RC clock with the PLL set to 30MHz frequency.
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

    /*
    // The following is for convenience and not necessary. AO.
    // It outputs the system clock frequency on Pin 27
    //    so that we can check using an oscilloscope:

    // First activate the clock out function:
    SYSCON->CLKOUTSEL = (uint32_t)3; //set CLKOUT source to main clock.
    SYSCON->CLKOUTUEN = 0UL;
    SYSCON->CLKOUTUEN = 1UL;
    // Divide by a reasonable constant so that it is easy to view on an oscilloscope:
    //SYSCON->CLKOUTDIV = 100;
    SYSCON->CLKOUTDIV = 2000;

    // Using the switch matrix, connect clock out to Pin 27:
    CLOCK_EnableClock(kCLOCK_Swm);     // Enables clock for switch matrix.
    SWM_SetMovablePinSelect(SWM0, kSWM_CLKOUT, kSWM_PortPin_P0_27);
    CLOCK_DisableClock(kCLOCK_Swm); // Disable clock for switch matrix.
    */
}

void Duty_Changee(volatile uint8_t *Duty, volatile uint32_t *Freq, uint32_t *const event, volatile uint32_t *sctimerClock, sctimer_config_t *const sctimerConfig, sctimer_pwm_signal_param_t *const p_1)
{
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);
    *sctimerClock = CLOCK_GetFreq(kCLOCK_Irc);
    SCTIMER_GetDefaultConfig(sctimerConfig);
    SCTIMER_Init(SCT0, sctimerConfig);
    /*
    /*
    //
    //
    /*
    //
    */
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
