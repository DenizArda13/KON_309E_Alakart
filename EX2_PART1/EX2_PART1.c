#include <stdbool.h>
#include <stdint.h>
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_mrt.h"


#define CORE_CLOCK 30000000U // Set CPU Core clock frequency (Hz)

#define DESIRED_INT_FREQ 100 // Desired number of INT's per second.

#define LED_PORT 0U
#define RED_LED_PIN 13U
#define YELLOW_LED_PIN 17U
#define GREEN_LED_PIN 18U

#define LED_ON 1U // LED will be ON when GPIO output is '1'
#define LED_OFF 0U

volatile uint32_t counter = 0; //variable that counts to 50 in Interrupt Service Routine

void MRT0_IRQHandler(void);
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
    //MRT config
    CLOCK_EnableClock(kCLOCK_Mrt);
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
        __WFI();// waiting for interrupt
    
    } // END while (1)

} // END main()

///////////////////////////////////////////////////////////////////////
////////// This is the MRT interrupt service routine. /////////////////
///////////////////////////////////////////////////////////////////////

void MRT0_IRQHandler(void)
{
    MRT_ClearStatusFlags(MRT0, kMRT_Channel_0, kMRT_TimerInterruptFlag);

    // turning on each led by 500ms delay
    if (counter == 0) {
        GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_ON);
    } else if (counter == 50) {
        GPIO_PinWrite(GPIO, LED_PORT, RED_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_ON);
    } else if (counter == 100) {
        GPIO_PinWrite(GPIO, LED_PORT, GREEN_LED_PIN, LED_OFF);
        GPIO_PinWrite(GPIO, LED_PORT, YELLOW_LED_PIN, LED_ON);
    }
    else if (counter >= 150)
    {
        counter = -1;
    }
    counter++;

}