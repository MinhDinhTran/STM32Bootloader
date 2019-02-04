@echo off
echo AXF to BIN file generation ...

"C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe" --bin --output ..\MDK-ARM\STM3210C_EVAL_SysTick.bin ..\MDK-ARM\bootloader_STM32F1xx\bootloader_STM32F1xx.axf 

echo AXF to BIN file generation completed.

fromelf --bin --output GT400VN_V2.90.bin .\flash\bootloader_STM32F1xx.axf

"C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe" --bin --output STM3210C_EVAL_SysTick.bin bootloader_STM32F1xx.axf
