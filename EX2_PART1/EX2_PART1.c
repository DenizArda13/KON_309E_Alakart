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
#define GREEN_LED_PIN 18U
#define YELLOW_LED_PIN 17U
#define RED_LED_PIN 13U

#define LED_ON 1U // LED will be ON when GPIO output is '1'
#define LED_OFF 0U

#define BUTTON_PIN 10U

#define USART_INSTANCE 0U
#define USART_BAUDRATE 115200

volatile uint32_t counter; //variable that counts to 50 in Interrupt Service Routine

volatile uint32_t millis = 0; // millisecond counter
void clock_init(void);
void SysTick_Handler(void);   // our systick interrupt handler
void delay_ms(uint32_t ms);   // delay (ms)

static volatile bool mrtIsrFlag = false; // MRT ISR sets this flag to true.

// callback function
void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status) {
    // callback function   
  
}

int main(void)
{
    uint32_t mrt_clock;
    uint32_t mrt_count_val;  

    mrt_config_t mrtConfig; // Struct for configuring the MRT:  
    CLOCK_EnableClock(kCLOCK_Gpio0);
    
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0}; // led config
    GPIO_PinInit(GPIO, LED_PORT, GREEN_LED_PIN, &led_config);
    GPIO_PinInit(GPIO, LED_PORT, YELLOW_LED_PIN, &led_config);
    GPIO_PinInit(GPIO, LED_PORT, RED_LED_PIN, &led_config);
    

    InitPins();
    clock_init();

    SysTick_Config(CORE_CLOCK / 1000); // setup systick clock interrupt @1ms
    
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt1, kSYSCON_GpioPort0Pin10ToPintsel); // defining input pin as pin interrupt which is pin 10

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
    
    CLOCK_EnableClock(kCLOCK_Mrt);

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
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_ON);
        delay_ms(500);
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
        delay_ms(500);        
        //__WFI();
    
    } // END while (1)

} // END main()

///////////////////////////////////////////////////////////////////////
////////// This is the MRT interrupt service routine. /////////////////
///////////////////////////////////////////////////////////////////////

void MRT0_IRQHandler(void)
{
    MRT_ClearStatusFlags(MRT0, // Clear interrupt flag:
                         kMRT_Channel_0,
                         kMRT_TimerInterruptFlag);
    
    counter++;
    /*
    if (counter == 0){
        GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_ON);
    }
    if (counter == 50){
        GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_ON);
    }
    if (counter == 100){
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_ON);
    }
    if(counter == 150){
        counter = 0;
    }
    */
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


