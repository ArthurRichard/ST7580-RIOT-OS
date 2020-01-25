# name of your application
APPLICATION = st7580_official
# If no BOARD is found in the environment, use this default:
BOARD ?= nucleo-l073rz

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += xtimer
USEMODULE += periph_gpio_irq
USEMODULE += printf_float
USEMODULE += periph_uart
USEMODULE += st7580


include $(RIOTBASE)/Makefile.include


