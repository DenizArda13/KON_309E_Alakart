
This program demonstrates the use of system tick interrupt to implement a precisely timed busy wait.

System tick (SysTick) is a simple timer common to ARM M processors that is used to produce a periodic interrupt to implement system services such as in an RTOS.

In this example, SysTick is set to produce an INT every millisecond. A busy wait causes a precisely timed delay with the timing precision of the SysTick interrupt. It is used to blink a LED.

Note that in init.c, for SysTick INT, Default_Handler
 has been changed to SysTick_Handler. 


To compile and program into the processror, open a terminal and type the following commands:

1. Compile blink.c to the object file blink.o
arm-none-eabi-gcc -c -mcpu=cortex-m0 -mthumb -g  blink.c -o blink.o

2. Compile init.c to the object file init.o
arm-none-eabi-gcc -c -mcpu=cortex-m0 -mthumb -g  init.c -o init.o

3. Link the files to a relocatable executable file (.elf) using the linker srcipt that describes the memory map of LPC824:
arm-none-eabi-ld blink.o init.o -T lpc824_linker_script.ld --cref -Map blink.map -o blink.elf

4. Convert the .elf file to another format; Intel Hex (.hex)
arm-none-eabi-objcopy -O ihex  blink.elf blink.hex

5. Program the hex file into the processor. (Connect Alakart to your PC)
lpc21isp -control blink.hex /dev/ttyUSB0 115200 12000

Microsoft Windows and Apple MacOS: Use Flash Magic program.
