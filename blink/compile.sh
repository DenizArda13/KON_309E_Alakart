#!/bin/bash
arm-none-eabi-gcc -c -mcpu=cortex-m0 -mthumb -g  blink.c -o blink.o
arm-none-eabi-gcc -c -mcpu=cortex-m0 -mthumb -g  init.c -o init.o
arm-none-eabi-ld blink.o init.o -T lpc824_linker_script.ld --cref -Map blink.map -o blink.elf
~/elektronik/ARM/LPC/lpcpatchelf/lpcpatchelf -f blink.elf
arm-none-eabi-objcopy -O ihex  blink.elf blink.hex
lpc21isp -control blink.hex /dev/ttyUSB0 115200 12000
