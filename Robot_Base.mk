SHELL=cmd
COMPORT=$(shell type COMPORT.inc)
OBJS = Robot_Base.o serial3.o startup_samd20.o init_Clock48.o

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS += -W -Wall --std=gnu99 -Os -fno-diagnostics-show-caret -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields -mcpu=cortex-m0plus -mthumb -MD -MP

LDFLAGS += -mcpu=cortex-m0plus -mthumb -Wl,--gc-sections -Wl,--script=../common/Linker_Scripts/samd20e16b_flash.ld

INCLUDES = -I..\common\include

DEFINES = -D__SAMD20E16B__ -DDONT_USE_CMSIS_INIT -DF_CPU=48000000 

CFLAGS += $(INCLUDES) $(DEFINES)

Robot_Base.hex: Robot_Base.elf
	$(OBJCOPY) -O ihex Robot_Base.elf Robot_Base.hex
	@$(SIZE) -t Robot_Base.elf

Robot_Base.elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o Robot_Base.elf

Robot_Base.o: Robot_Base.c
	$(CC) $(CFLAGS) Robot_Base.c -c -o Robot_Base.o

startup_samd20.o: ..\common\source\gcc\startup_samd20.c
	$(CC) $(CFLAGS) ..\common\source\gcc\startup_samd20.c -c -o startup_samd20.o

serial3.o: ..\common\source\gcc\serial3.c
	$(CC) $(CFLAGS) ..\common\source\gcc\serial3.c -c -o serial3.o

init_Clock48.o: ..\common\source\gcc\init_Clock48.c
	$(CC) $(CFLAGS) ..\common\source\gcc\init_Clock48.c -c -o init_Clock48.o

load_flash:
	@Taskkill /IM putty.exe /F 2>NUL | wait 500
	..\Prog_SAMD\Prog_SAMD -p Robot_Base.hex
	cmd /c start putty.exe -serial $(COMPORT) -sercfg 115200,8,n,1,N

verify_flash:
	@Taskkill /IM putty.exe /F 2>NUL | wait 500
	..\Prog_SAMD\Prog_SAMD Robot_Base.hex

putty:
	@Taskkill /IM putty.exe /F 2>NUL | wait 500
	cmd /c start putty.exe -serial $(COMPORT) -sercfg 115200,8,n,1,N

size: Robot_Base.elf
	@echo size:
	@$(SIZE) -t Robot_Base.elf

Dummy: Robot_Base.hex
	@echo Nothing to see here!
	
explorer:
	cmd /c start explorer .

clean:
	@echo clean
	del *.o *.d *.elf *.hex
