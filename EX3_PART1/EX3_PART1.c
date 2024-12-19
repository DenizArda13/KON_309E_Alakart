// This is an example program where the timer is used to directly initiate an
// ADC conversion sequence.
// At the end of the sequence, the ADC triggers the
// "ADC0 Sequence A conversion complete interrupt" and the corresponding ISR
// prints out the conversion result to the terminal.

// Modified from AO 2023

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

// The pointer and flag are global so that ISR can manipulate them:
volatile adc_result_info_t ADCResultStruct[2]; // ADC results structure array (and pointer as well) for 2 ADC channels
volatile bool gAdcConversionDone = false; // Flag to indicate ADC conversion done
volatile bool adc_conversion_done = false; // Flag for ADC conversion completion
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
  CLOCK_EnableClock(kCLOCK_Adc);      
  POWER_DisablePD(kPDRUNCFG_PD_ADC0); 

  frequency = CLOCK_GetFreq(kCLOCK_Irc);
  if (true == ADC_DoSelfCalibration(ADC0, frequency)){
    xprintf("ADC Calibration Done.\r\n");}
  else{
    xprintf("ADC Calibration Failed.\r\n");}

  ADC_Configuration(); // starting ADC 

  // ADC interrupt
  ADC_EnableInterrupts(ADC0, kADC_ConvSeqAInterruptEnable); 
  NVIC_EnableIRQ(ADC0_SEQA_IRQn);                           

  xprintf("Configuration Done.\r\n");

  while (1){

    xprintf("Press a key to start conversion.\r\n");
    GETCHAR(); // Kullanıcıdan giriş al
    ADC_DoSoftwareTriggerConvSeqA(ADC0); // ADC Sequence A başlat

    while (!adc_conversion_done)
    {
      // Wait for conversion to complete
    }

    adc_conversion_done = false; // Reset flag

    // Calculate voltages
    int16_t voltage_ch0 = (ADCResultStruct[0].result * REF_VOLTAGE_MV) / 4095;
    int16_t voltage_ch1 = (ADCResultStruct[1].result * REF_VOLTAGE_MV) / 4095;

    // Print results
    xprintf("ADC0=%d mV, ADC1=%d mV\r\n", voltage_ch0, voltage_ch1);
  }
    
} // END: main()


// ISR for ADC conversion sequence A done.
void ADC0_SEQA_IRQHandler(void)
{

  if (kADC_ConvSeqAInterruptFlag & ADC_GetStatusFlags(ADC0)) {

    // Kanal 0 ve 1 dönüşüm sonuçlarını oku
    ADC_GetChannelConversionResult(ADC0, ADC_CHANNEL_0, &ADCResultStruct[0]);
    ADC_GetChannelConversionResult(ADC0, ADC_CHANNEL_1, &ADCResultStruct[1]);
    // Set flag to indicate conversion is complete
    adc_conversion_done = true;

    // Interrupt bayrağını temizle
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
  adcConvSeqConfigStruct.triggerMask = 1U;//Trigger with interrupt
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
  // Divide by x to obtain 24000000/x Hz. intermediate clock.
  CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 13U); // USART clock div register.

  // Baud rate generator value is calculated from:
  // Intermediate clock /16 (always divided) = 24000000/(16*divisor)Hz
  // To obtain 115200 baud transmission speed 24000000/(16*divisor) must be closer to 115200
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
// ayarlanıp ayarlanmadığını hocaya sor
  matchValueL = 48000U; // This is in: 16.6.20 SCT match registers 0 to 7
                       ////////// This value is incorrect
  ////////// You must calculate your match value for 500ms interupt you may use the previous versions
  /////////// where we calculated 1ms counter for 60mhz so calculate for 24mhz /////////////
  ////////// You must adjust prescaler and match value accordingly
  sctimerConfig.enableBidirection_l = false; // Use as single directional register.
  // Prescaler is 8 bit, in: CTRL. See: 16.6.3 SCT control register
  // sctimerConfig.prescale_l = 249U; // For this value +1 is used.
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
void config_uart0 (void){


  // The following steps set up the serial port USART0:
   // See User Manual
  // 13.3 Basic Configuration (USART)

  // 1. Turn the peripheral on:
  SYSCON_SYSAHBCLKCTRL |= 0x4000; // Enable clock for USART0.

  // 2. Set speed (baud rate) to 38500bps:
  // See Sec. 13.7.1.1 and 13.6.9 in User Manual.
  // Obtain a preliminary clock by first dividing the processor main clock
  // Processor main clock is 24MHz. (24,000,000)
  // Divide by 16 to obtain 3,000,000 Hz. intermediate clock.
  SYSCON_UARTCLKDIV=16; // USART clock div register.
 
  // Baud rate generator value is calculated from:
  // Intermediate clock /16 (always divided) = 468750Hz
  // To obtain 115200 baud transmission speed, we must divide further:
  // 3,000,000/18500=4.8 ~= 5
  // Baud rate generator should be set to one less than this value.
  USART0_BRG=4;  //(4-1)  Baud rate generator register value.

  // 3. Enable USART & configure byte format for 8 bit, no parity, 1 stop bit:
  // (See 13.6.1 USART Configuration register)
  // (Bit 0) Enable USART 
  // (Bit 1) not used.
  // (Bit 2:3) Data Length 00 => 8 bits.
  // (Bit 4:5) Parity 00 => No parity (default)
  // (Bit 6) Stop bit  0 => 1:  (default)
  // (Bit 7) Reserved
  // The remaining bits are left at default values.
  USART0_CFG=0b00000101;

}