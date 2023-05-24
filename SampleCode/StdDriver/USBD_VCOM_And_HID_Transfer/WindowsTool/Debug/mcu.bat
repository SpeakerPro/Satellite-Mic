@echo off
set isp_file=mcu.bin
copy /b ..\..\KEIL\obj\*.bin %isp_file%
HIDTransferTest.exe -isp %isp_file% -uap -vid 6127 -pid 41076
del %isp_file%
