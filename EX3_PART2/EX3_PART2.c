
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_adc.h"
#include "fsl_sctimer.h"
#include "fsl_clock.h"
#include "fsl_power.h"
#include "fsl_syscon.h"
#include "fsl_usart.h"
#include <stdbool.h>
#include <stdio.h>
#include "xprintf.h"
#include "lpc824.h"

#define ADC_CHANNEL_0 0U     // Channel 1 will be used in this example.
#define ADC_CHANNEL_1 1U     // Channel 2 will be used in this example.
#define CORE_CLOCK 24000000U // Set core clock to 24 MHz
#define ADC_CLOCK_DIVIDER 1U // See Fig 52. ADC clocking in Ref Manual.
#define USART_INSTANCE 0U
#define Conversion_Rate 4095U / 4095U // This may be used to convert results to different units
#define REF_VOLTAGE_MV 3300 // Reference voltage in millivolts
#define LED_PIN 20U // LED connected to PIO0_20
#define LED_PORT 0U // LED Port

// The pointer and flag are global so that ISR can manipulate them:
volatile adc_result_info_t ADCResultStruct[2]; // ADC results structure array (and pointer as well) for 2 ADC channels
volatile bool adc_conversion_done = false; // Flag for ADC conversion completion
volatile uint32_t led_status = 0; // Variable that controls the led 
uint32_t eventCounterL;
//uint32_t counter = 0;
void uart_init(void);
void ADC_Configuration(void);
void SCT_Configuration(void);
void clock_init(void);
void uart_putch(uint8_t character);

int main(void)
{
  uint32_t frequency;

  BOARD_InitPins();
  BOARD_InitBootClocks();
  uart_init(); 
  xdev_out(uart_putch); 

  CLOCK_EnableClock(kCLOCK_Sct);      
  SCT_Configuration();                
  CLOCK_EnableClock(kCLOCK_Adc);      // Enable ADC clock 
  POWER_DisablePD(kPDRUNCFG_PD_ADC0); // Power on ADC0

  // Hardware calibration is required after each chip reset.
  // See: Sec. 21.3.4 Hardware self-calibration
  frequency = CLOCK_GetFreq(kCLOCK_Irc);

  // LED Configuration 
  gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};
  GPIO_PinInit(GPIO, LED_PORT, LED_PIN, &led_config);
  // Led is off at the begining
  GPIO_PinWrite(GPIO, LED_PORT, LED_PIN, 0);

  if (true == ADC_DoSelfCalibration(ADC0, frequency)){
    xprintf("ADC Calibration Done.\r\n");}
  else{
    xprintf("ADC Calibration Failed.\r\n");}

  ADC_Configuration(); // Configure ADC and operation mode.

  // Enable the interrupt the for Sequence A Conversion Complete:
  ADC_EnableInterrupts(ADC0, kADC_ConvSeqAInterruptEnable); // Within ADC0
  NVIC_EnableIRQ(ADC0_SEQA_IRQn);                           // Within NVIC  

  xprintf("Configuration Done.\n");

  while (1){
    if (adc_conversion_done){

        adc_conversion_done = false; // Reset the flag

        // Voltage calculations
        int16_t voltage_ch0 = (ADCResultStruct[0].result * REF_VOLTAGE_MV) / 4095;
        int16_t voltage_ch1 = (ADCResultStruct[1].result * REF_VOLTAGE_MV) / 4095;

        // Printring the voltage values in mV
        xprintf("ADC0=%d mV, ADC1=%d mV\r\n", voltage_ch0, voltage_ch1);

        // Added a counter to see whether Serial Monitor is broker or not
        //xprintf("counter=%d\r\n", counter);
        //counter ++;
        
        // Toggle the led
        led_status = !led_status;
        GPIO_PinWrite(GPIO, LED_PORT, LED_PIN, led_status);  
    }
    __WFI();  // Wait For Interrupt to save power
  }
}

// ISR for ADC conversion sequence A done.
void ADC0_SEQA_IRQHandler(void)
{

  if (kADC_ConvSeqAInterruptFlag & ADC_GetStatusFlags(ADC0)) {

    // Read the conversion results from channel 0 and 1
    ADC_GetChannelConversionResult(ADC0, ADC_CHANNEL_0, &ADCResultStruct[0]);
    ADC_GetChannelConversionResult(ADC0, ADC_CHANNEL_1, &ADCResultStruct[1]);
    // Set flag to indicate conversion is complete
    adc_conversion_done = true;
    // Clear the status flag
    ADC_ClearStatusFlags(ADC0, kADC_ConvSeqAInterruptFlag);
    }    
}

// Configure and initialize the ADC
void ADC_Configuration(void)
{

  adc_config_t adcConfigStruct;
  adc_conv_seq_config_t adcConvSeqConfigStruct;

  adcConfigStruct.clockDividerNumber = ADC_CLOCK_DIVIDER; // Defined above.
  adcConfigStruct.enableLowPowerMode = false;
  adcConfigStruct.voltageRange = kADC_HighVoltageRange;

  ADC_Init(ADC0, &adcConfigStruct); // Initialize ADC0 with this structure.

  adcConvSeqConfigStruct.channelMask = 3U; // Mask the least significant bit0 and bit1 for ADC channel0 and channel1 respectively;
  adcConvSeqConfigStruct.triggerMask = 3U;//Trigger with timer
  adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
  adcConvSeqConfigStruct.enableSingleStep = false;
  adcConvSeqConfigStruct.enableSyncBypass = false;
  adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence;

  // Initialize the ADC0 with the sequence defined above:
  ADC_SetConvSeqAConfig(ADC0, &adcConvSeqConfigStruct);
  ADC_EnableConvSeqA(ADC0, true); // Enable the conversion sequence A.

  // Make the first ADC conversion so that
  // the result register has a sensible initial value.
  ADC_DoSoftwareTriggerConvSeqA(ADC0);
}

void uart_init(void)
{
  uint32_t uart_clock_freq;
  usart_config_t config;
 
  CLOCK_EnableClock(kCLOCK_Uart0);

  // 2. Set speed (baud rate) to 115200bps:
  // See Sec. 13.7.1.1 and 13.6.9 in User Manual.
  // Obtain a preliminary clock by first dividing the processor main clock
  // Processor main clock is 24MHz. (240000000)
  // Divide by 26 to obtain 24000000/26 Hz. intermediate clock.
  CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 26U); // USART clock div register.

  // Baud rate generator value is calculated from:
  // Intermediate clock /16 (always divided) = 24000000/(16*divisor)Hz
  // To obtain 57600 baud transmission speed 24000000/(16*26) is closer to 57600
  // This applies to other baud rates as well
  USART0->BRG = 1;
  USART_GetDefaultConfig(&config);
  config.enableRx = true;
  config.enableTx = true;
  USART_Init(USART0, &config, uart_clock_freq);

  USART0->CFG = 0b00000101; 
}

void SCT_Configuration(void)
{
  sctimer_config_t sctimerConfig;
  uint32_t eventCounterL;
  uint16_t matchValueL;

  SCTIMER_GetDefaultConfig(&sctimerConfig);

  // Set the configuration struct for the timer:
  // For more information, see:  Xpresso_SDK/devices/LPC824/drivers/fsl_sctimer.h
  sctimerConfig.enableCounterUnify = false; // Use as two 16 bit timers.

  sctimerConfig.clockMode = kSCTIMER_System_ClockMode; // Use system clock as SCT input
  matchValueL = 24000U; // This is in: 16.6.20 SCT match registers 0 to 7
  sctimerConfig.enableBidirection_l = false; // Use as single directional register.
  // Prescaler is 8 bit, in: CTRL. See: 16.6.3 SCT control register
  // sctimerConfig.prescale_l = 999U; // For this value +1 is used.
  sctimerConfig.prescale_l = 249U;
  SCTIMER_Init(SCT0, &sctimerConfig); // Initialize SCTimer module

  // Configure the low side counter.
  // Schedule a match event for the 16-bit low counter:
  SCTIMER_CreateAndScheduleEvent(SCT0,
                                 kSCTIMER_MatchEventOnly,
                                 matchValueL,
                                 0, // Not used for "Match Only"
                                 kSCTIMER_Counter_L,
                                 &eventCounterL);

  // TODO: Rather than toggle, it should set the output:
  // Toggle output_3 when the 16-bit low counter event occurs:
  SCTIMER_SetupOutputToggleAction(SCT0, kSCTIMER_Out_3, eventCounterL);
  // Reset Counter L when the 16-bit low counter event occurs
  SCTIMER_SetupCounterLimitAction(SCT0, kSCTIMER_Counter_L, eventCounterL);

  // Setup the 16-bit low counter event active direction
  //  See fsl_sctimer.h
  SCTIMER_SetupEventActiveDirection(SCT0,
                                    kSCTIMER_ActiveIndependent,
                                    eventCounterL);

  // Start the 16-bit low counter
  SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
}
void uart_putch(uint8_t character)
{
  // Check if transmission has ended. See: 13.6.3 USART Status register:
  while ((USART0_STAT & 0b0100) == 0)
    ;
  USART0_TXDAT = character;
}

