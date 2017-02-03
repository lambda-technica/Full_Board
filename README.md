# Full Board

Project is an STM32 embedded system using FreeRTOS which connects to various external probes using I2C protocol and displays results on LCD screen. The following component have been used:
* STM32 ARM Cortex M-3 F107 series chip
* Graphical LCD based on KS0108 with 5x8 font library 
* L3G4200D gyroscope
* DP83848 ethernet
* MCP98003 temperature chip

The connection between sensors and the system is based on I2C protocol. The system is driven by FreeRTOS with appropriate locking mechanisma and synchronisation. Inter-task communication uses queques provided by RTOS. Project was built using free version of TrueStudio. Code base was established with STMCube (attached to repository). The STMCube also provides the visual representation of pin layout. Segger J-link SWD was used for debugging purposes with free version of Tracealyzer. 
