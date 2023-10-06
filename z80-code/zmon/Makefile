C=z80asm
CFLAGS=-I ../include/

all: zmon

zmon: zmon.asm ../include/zmachine.inc ../include/ascii.inc
	$(C) $(CFLAGS) zmon.asm -o zmon.bin --list=zmon.lst --label=../include/zmon.inc
	srec_cat zmon.bin -binary -o zmon.hex -intel

clean:
	rm *.bin *.hex *.lst