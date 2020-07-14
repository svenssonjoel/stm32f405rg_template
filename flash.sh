#!/bin/sh
openocd -f stm32f405.cfg -c "program ./build/usb_test0.elf verify reset exit"
