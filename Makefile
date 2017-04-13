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
GDBTUI = $(CCPREFIX)gdbtui
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary -S
MCU  = cortex-m0
 
# List all C defines here
DDEFS = 
#
# Define project name and Ram/Flash mode here
PROJECT        = cameraarm
 
# List C source files here

SRC  = ./STMRemote/src/main.c
SRC += ./STMRemote/src/stm32f0xx_it.c
SRC += ./STMRemote/src/system_stm32f0xx.c
SRC += ./STMRemote/src/stmCriticalSection.c
SRC += ./STMRemote/src/tsl_user.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_adc.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_can.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_cec.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_comp.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_crc.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_crs.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dbgmcu.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dma.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_exti.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_flash.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_gpio.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_i2c.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_iwdg.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_misc.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_pwr.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rcc.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rtc.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_spi.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_syscfg.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_tim.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_usart.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_wwdg.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_acq.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_filter.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_dxs.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_ecs.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_globals.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_linrot.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_object.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_time.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_time_stm32f0xx.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_touchkey.c
SRC += ./STMRemote/Drivers/STMTouch_Driver/src/tsl_acq_stm32f0xx.c
SRC += ./STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dac.c

# List assembly startup source file here
STARTUP = ./STMRemote/startup/startup_stm32f0xx.s
 
# List all include directories here
ABSPATH = .#~/EmbeddedArm/RCCameraArm
INCDIRS = $(ABSPATH)/STMRemote/inc $(ABSPATH)/STMRemote/Drivers/CMSIS/Include $(ABSPATH)/STMRemote/Drivers/CMSIS/Device/ST/STM32F0xx/Include $(ABSPATH)/STMRemote/MDK-ARM/RTE $(ABSPATH)/STMRemote/Drivers/STM32F0xx_StdPeriph_Driver/inc $(ABSPATH)/STMRemote/Drivers/STMTouch_Driver/inc 
              
# List the user directory to look for the libraries here
LIBDIRS += 
 
# List all user libraries here
LIBS =
 
# Define optimisation level here
OPT = -Os
 

# Define linker script file here
LINKER_SCRIPT = ./STMRemote/linker/stm32f0_linker.ld

 
INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))
##reference only flags for run from ram...not used here
##DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM

## run from Flash
DEFS    = $(DDEFS) -DRUN_FROM_FLASH=1 -DUSE_STDPERIPH_DRIVER -DSTM32F0XX

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

flash_stlink: $(PROJECT).bin
	st-flash write $(PROJECT).bin 0x8000000

erase_openocd:
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "stm32f1x mass_erase 0" -c "sleep 100" -c shutdown 

erase_stlink:
	st-flash erase

debug_openocd: $(PROJECT).elf flash_openocd
	xterm -e openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "halt" -c "reset halt" &
	$(GDBTUI) --eval-command="target remote localhost:3333" $(PROJECT).elf 

debug_stlink: $(PROJECT).elf
	xterm -e st-util &
	$(GDBTUI) --eval-command="target remote localhost:4242"  $(PROJECT).elf -ex 'load'
		
clean:
	-rm -rf $(OBJS)
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASRC:.s=.lst)
# *** EOF ***
