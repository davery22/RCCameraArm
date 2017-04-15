#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!   
#
##############################################################################################
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#
# Included originally in the yagarto projects. Original Author : Michael Fischer
# Modified to suit our purposes by Hussam Al-Hertani
# Use at your own risk!!!!!
##############################################################################################
# Start of default section
#
CCPREFIX = arm-none-eabi-
CC   = $(CCPREFIX)gcc
CP   = $(CCPREFIX)objcopy
AS   = $(CCPREFIX)gcc -x assembler-with-cpp
GDBTUI = $(CCPREFIX)gdb
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary -S
MCU  = cortex-m0
 
# List all C defines here
DDEFS = 
#
# Define project name and Ram/Flash mode here
PROJECT        = cameraarm
 
# List C source files here

PERIPHS = ./Drivers/STM32F0xx_StdPeriph_Driver
TOUCH = ./Drivers/STMTouch_Driver

SRC  = ./src/main.c
SRC += ./src/stm32f0xx_it.c
SRC += ./src/system_stm32f0xx.c
SRC += ./src/stmCriticalSection.c
SRC += ./src/tsl_user.c
#SRC += $(PERIPHS)/src/stm32f0xx_adc.c
#SRC += $(PERIPHS)/src/stm32f0xx_can.c
#SRC += $(PERIPHS)/src/stm32f0xx_cec.c
#SRC += $(PERIPHS)/src/stm32f0xx_comp.c
#SRC += $(PERIPHS)/src/stm32f0xx_crc.c
#SRC += $(PERIPHS)/src/stm32f0xx_crs.c
#SRC += $(PERIPHS)/src/stm32f0xx_dac.c
#SRC += $(PERIPHS)/src/stm32f0xx_dbgmcu.c
#SRC += $(PERIPHS)/src/stm32f0xx_dma.c
SRC += $(PERIPHS)/src/stm32f0xx_exti.c
SRC += $(PERIPHS)/src/stm32f0xx_flash.c
SRC += $(PERIPHS)/src/stm32f0xx_gpio.c
SRC += $(PERIPHS)/src/stm32f0xx_i2c.c
#SRC += $(PERIPHS)/src/stm32f0xx_iwdg.c
SRC += $(PERIPHS)/src/stm32f0xx_misc.c
SRC += $(PERIPHS)/src/stm32f0xx_pwr.c
SRC += $(PERIPHS)/src/stm32f0xx_rcc.c
#SRC += $(PERIPHS)/src/stm32f0xx_rtc.c
#SRC += $(PERIPHS)/src/stm32f0xx_spi.c
SRC += $(PERIPHS)/src/stm32f0xx_syscfg.c
SRC += $(PERIPHS)/src/stm32f0xx_tim.c
SRC += $(PERIPHS)/src/stm32f0xx_usart.c
#SRC += $(PERIPHS)/src/stm32f0xx_wwdg.c
SRC += $(TOUCH)/src/tsl_acq.c
SRC += $(TOUCH)/src/tsl_filter.c
SRC += $(TOUCH)/src/tsl.c
SRC += $(TOUCH)/src/tsl_dxs.c
SRC += $(TOUCH)/src/tsl_ecs.c
SRC += $(TOUCH)/src/tsl_globals.c
SRC += $(TOUCH)/src/tsl_linrot.c
SRC += $(TOUCH)/src/tsl_object.c
SRC += $(TOUCH)/src/tsl_time.c
SRC += $(TOUCH)/src/tsl_time_stm32f0xx.c
SRC += $(TOUCH)/src/tsl_touchkey.c
SRC += $(TOUCH)/src/tsl_acq_stm32f0xx.c

# List assembly startup source file here
STARTUP = ./startup/startup_stm32f0xx.s
 
# List all include directories here
INCDIRS = ./inc ./Drivers/CMSIS/Include ./Drivers/CMSIS/Device/ST/STM32F0xx/Include ./MDK-ARM/RTE $(PERIPHS)/inc $(TOUCH)/inc 
              
# List the user directory to look for the libraries here
LIBDIRS += 
 
# List all user libraries here
LIBS =
 
# Define optimisation level here
OPT = -Os
 

# Define linker script file here
LINKER_SCRIPT = ./linker/stm32f0_linker.ld

 
INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))
##reference only flags for run from ram...not used here
##DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM

## run from Flash
DEFS    = $(DDEFS) -DRUN_FROM_FLASH=1 -DUSE_STDPERIPH_DRIVER -DSTM32F0XX -DSTM32F072 -DSTM32F072xB -D_RTE_ -DUSE_LINUX

OBJS  = $(STARTUP:.s=.o) $(SRC:.c=.o)
MCFLAGS = -mcpu=$(MCU)
 
ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.s=.lst) 
CPFLAGS = $(MCFLAGS) $(OPT) -g -gdwarf-2 -mthumb   -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS) 
LDFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb -nostartfiles -T$(LINKER_SCRIPT) -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)
 
#
# makefile rules
#
 
all: $(OBJS) $(PROJECT).elf  $(PROJECT).hex $(PROJECT).bin
	$(TRGT)size $(PROJECT).elf
 
%.o: %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%.o: %.s
	$(AS) -c $(ASFLAGS) $< -o $@

%.elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%.hex: %.elf
	$(HEX) $< $@
	
%.bin: %.elf
	$(BIN)  $< $@
	
flash_openocd: $(PROJECT).bin
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase $(PROJECT).bin 0x08000000" -c "sleep 100" -c "verify_image $(PROJECT).bin 0x08000000" -c "sleep 100" -c "reset run" -c shutdown

erase_openocd:
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "stm32f1x mass_erase 0" -c "sleep 100" -c shutdown 

debug_openocd: $(PROJECT).elf flash_openocd
	xterm -e openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "halt" -c "reset halt" &
	$(GDBTUI) --eval-command="target remote localhost:3333" $(PROJECT).elf 

clean:
	-rm -rf $(OBJS)
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASRC:.s=.lst)
# *** EOF ***
