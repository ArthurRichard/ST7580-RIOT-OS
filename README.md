# ST7580-RIOT-OS
 Port of the ST X-CUBE-PLM1 driver to RIOT-OS

This code should work as is on any Nucleo-64 board.

## ST7580-RIOT-OS
To use it, drop the "st7580" folder into RIOT-OS's "driver" folder.
Then, copy the "st7580.h" present in the "drivers/st7580/include" to "drivers/include"

then, add USEMODULE += st7580 to your application's Makefile.

For additional documentation, look at https://www.st.com/en/embedded-software/x-cube-plm1.html