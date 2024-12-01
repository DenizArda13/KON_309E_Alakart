#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_swm.h"
#include "pin_mux.h"

void InitPins(void) {
  
  uint32_t IOCON_PIO_config;
  
  // Enable clocks for IOCON and SWM (Switch Matrix)
  CLOCK_EnableClock(kCLOCK_Iocon);   // Enable clock for IOCON block.
  CLOCK_EnableClock(kCLOCK_Swm);     // Enables clock for switch matrix.

  // Configure P0_13 for PWM output (Red LED)
  IOCON_PIO_config = (IOCON_PIO_MODE_PULLUP |  // Select pull-up function
                      IOCON_PIO_HYS_EN |       // Enable hysteresis
                      IOCON_PIO_INV_DI |       // Do not invert input
                      IOCON_PIO_OD_DI |        // Disable open-drain function
                      IOCON_PIO_SMODE_BYPASS | // Bypass the input filter
                      IOCON_PIO_CLKDIV0);      // IOCONCLKDIV = 0
  IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_13, IOCON_PIO_config);
  
  // Set SCT OUT0 (PWM for Red LED) to P0_13
  SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT2, kSWM_PortPin_P0_13);

  // Configure P0_17 for PWM output (Yellow LED)
  IOCON_PIO_config = (IOCON_PIO_MODE_PULLUP |  // Select pull-up function
                      IOCON_PIO_HYS_EN |       // Enable hysteresis
                      IOCON_PIO_INV_DI |       // Do not invert input
                      IOCON_PIO_OD_DI |        // Disable open-drain function
                      IOCON_PIO_SMODE_BYPASS | // Bypass the input filter
                      IOCON_PIO_CLKDIV0);      // IOCONCLKDIV = 0
  IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_17, IOCON_PIO_config);
  
  // Set SCT OUT1 (PWM for Yellow LED) to P0_17
  SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT3, kSWM_PortPin_P0_17);

  // Configure P0_18 for PWM output (Green LED)
  IOCON_PIO_config = (IOCON_PIO_MODE_PULLUP |  // Select pull-up function
                      IOCON_PIO_HYS_EN |       // Enable hysteresis
                      IOCON_PIO_INV_DI |       // Do not invert input
                      IOCON_PIO_OD_DI |        // Disable open-drain function
                      IOCON_PIO_SMODE_BYPASS | // Bypass the input filter
                      IOCON_PIO_CLKDIV0);      // IOCONCLKDIV = 0
  IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_18, IOCON_PIO_config);
  
  // Set SCT OUT2 (PWM for Green LED) to P0_18
  SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT4, kSWM_PortPin_P0_18);

  // Disable the clock for Switch Matrix after configuration
  CLOCK_DisableClock(kCLOCK_Swm); // Disable clock for switch matrix.
}


