B1: Create new empty C++ project with your STM32 device.
B2: Copy Core, Driver, Source, main folder to your project (Source folder should be rename from Src, don't delete syscalls.c and sysmem.c, delete default main.c file).
B3: Replace all file in Core folder by your stm32 cmsis and device file.
B4: Go to Project -> Properties -> C/C++ Build -> Settings -> MCU GCC Compiler and MCU G++ Compiler, add:
	- ../Core/CMSIS/
	- ../Core/Device/
	- ../Driver/freertos/include/
	- ../Driver/config/
	- ../Driver/include/
	- ../Source/
	- ../main/
B5: Goto C/C++ General -> Paths and Symbols -> Source Location and Core, Driver, Source, main to source folder.
B6: Open Driver/config/sdkconfig.h and replace macros by your settings.
And finish!
 
