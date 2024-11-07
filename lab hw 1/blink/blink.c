#include <stdint.h>
#include "lpc824.h"

#define SYSTEM_CORE_CLOCK 30000000UL   //Declare system clock as 30MHz
// (The clock speed has been set in "init.c" file to 30MHz.)

static inline uint32_t SysTickConfig(uint32_t ticks);
void SysTick_Handler(void);  //our systick interrupt handler
void delay_ms(uint32_t ms);//delay (ms)

volatile uint32_t delaytime; // This is decremented by SysTick_Handler.


int main(void) {
  int buttonState = 0;

  delaytime=0;
  
  SYSCON_SYSAHBCLKCTRL |= 0x400C0; // Enable IOCON, SWM & GPIO clocks.
  
  SYSCON_PRESETCTRL &= ~(0x400);  // Peripheral reset control to gpio/gpio int
  SYSCON_PRESETCTRL |=   0x400;   // AO: Check.
  
  //Make Pin 9,8,7 an output and Pin 10, 11 an input.
  GPIO_DIR0 &= (!(1<<10));//Button 1 input
  GPIO_DIR0 &= (!(1<<11));//Button 2 input

  GPIO_DIR0 |= (1<<9);//Yellow Led output
  GPIO_DIR0 |= (1<<8);//Green Led output
  GPIO_DIR0 |= (1<<7);//Red Led output

  SysTickConfig(SYSTEM_CORE_CLOCK/1000);  //setup systick clock interrupt @1ms 

  while (1) { //infinite loop
  // delay for taking input 
  delay_ms(1000); 
 
    // Button 1
     if(GPIO_B10 == 1){
      buttonState = 1;
     } 

    // Button 2
    else if (GPIO_B11 == 1){
      buttonState = 2; 
     }

     switch (buttonState){

      // Red,green,yellow,all ON, all OFF.
      case 1:

      GPIO_B7 = 1;    //set pin high (Red LED is ON)
      delay_ms(1000);
      GPIO_B7 = 0;    //set pin low (Red LED is OFF) 
      delay_ms(1000);

      GPIO_B8 = 1;    //set pin high (Green LED is ON)
      delay_ms(1000);   
      GPIO_B8 = 0;    //set pin low (Green LED is OFF)
      delay_ms(1000); 

      GPIO_B9 = 1;    //set pin high. (Yellow LED is ON)
      delay_ms(1000);
      GPIO_B9 = 0;    //set pin low. (Yellow LED is OFF)
      delay_ms(1000);

      GPIO_B9 = 1;    //set pin high. (Yellow LED is ON)
      GPIO_B8 = 1;    //set pin high (Green LED is ON)  
      GPIO_B7 = 1;    //set pin high (Red LED is ON)
      delay_ms(1000);
      GPIO_B9 = 0;    //set pin low (Yellow LED is OFF)
      GPIO_B8 = 0;    //set pin low (Green LED is OFF)
      GPIO_B7 = 0;    //set pin low (Red LED is OFF)
      delay_ms(1000);       

      GPIO_B9 = 0;    //set pin low (Yellow LED is OFF)
      GPIO_B8 = 0;    //set pin low (Green LED is OFF)
      GPIO_B7 = 0;    //set pin low (Red LED is OFF)
      delay_ms(1000);
       
      buttonState = 0;

      break;

      //red, green, all ONN,yellow,all OFF.
      case 2:
      
      GPIO_B7 = 1;    //set pin high (Red LED is ON)
      delay_ms(1000);
      GPIO_B7 = 0;    //set pin low (Red LED is OFF) 
      delay_ms(1000);

      GPIO_B8 = 1;    //set pin high (Green LED is ON)
      delay_ms(1000);   
      GPIO_B8 = 0;    //set pin low (Green LED is OFF)
      delay_ms(1000);

      GPIO_B9 = 1;    //set pin high. (Yellow LED is ON)
      GPIO_B8 = 1;    //set pin high (Green LED is ON)  
      GPIO_B7 = 1;    //set pin high (Red LED is ON)
      delay_ms(1000);
      GPIO_B9 = 0;    //set pin low (Yellow LED is OFF)
      GPIO_B8 = 0;    //set pin low (Green LED is OFF)
      GPIO_B7 = 0;    //set pin low (Red LED is OFF)
      delay_ms(1000);               
      
      GPIO_B9 = 1;    //set pin high. (Yellow LED is ON)
      delay_ms(1000);
      GPIO_B9 = 0;    //set pin low. (Yellow LED is OFF)
      delay_ms(1000);

      GPIO_B9 = 0;    //set pin low (Yellow LED is OFF)
      GPIO_B8 = 0;    //set pin low (Green LED is OFF)
      GPIO_B7 = 0;    //set pin low (Red LED is OFF)
      delay_ms(1000); 

      buttonState = 0;
 
      break;
      
      //only Red ON.     
      default: 

      GPIO_B7 = 1;    //set pin high (Red LED is ON)
      delay_ms(1000);

      break;
     }
     

  }
}

 //The interrupt handler for SysTick system time-base timer.
void SysTick_Handler(void) { 
  if (delaytime!=0){ // If delaytime has been set somewhere in the program,
    --delaytime;     //  decrement it every time SysTick event occurs (1ms).
  }
}


void delay_ms(uint32_t ms) {//delay (ms)

  delaytime=ms;        // Set the delay time to the number of millisecs of wait
  while(delaytime!=0){}// Wait here until the delay time expires.

}
 
// System Tick Configuration:
// Initializes the System Timer and its interrupt, and
// Starts the System Tick Timer.
// ticks = Number of ticks between two interrupts.

static inline uint32_t SysTickConfig(uint32_t ticks) {
  if (ticks > 0xFFFFFFUL) // Timer is only 24 bits wide.
    return (1); //Reload value impossible
  
  SYST_RVR = (ticks & 0xFFFFFFUL) - 1;  //Set reload register

  SYST_CVR = 0;   //Load the initial count value.

  SYST_CSR = 0x07;  // Counter ENABLE, INT ENABLE, CLK source=system clock.

  return (0);
}         // AO!: Check OK.
