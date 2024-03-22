Zmachine is a single board computer with the following features:
- RP2040 based console that provides 8-bit color VGA output, PS2 keyboard interface, PWM based sound, etc
- Z80 CPU, clock is provided by the RP2040 so it can be set to any value the chip supports
- 512k of SRAM with MMU to support paging
- 128, 256, or 512k of flash ROM
- Dual serial ports (one RS232, one FTDI compatible TTL interface)
- Two microSD cards for storage (SPI mode, with real hardware shift registers)
- Floppy controller
- Vectored interrupts
- Two RC2014 compatible expansion slots
- SPI interface (for RTC, etc)
- I2C and VGA DDC support

Hardware tested and working:
- CPU, memory, basic system functionality
- VGA (though the sync drops out from time to time still - probably a RP2040 software issue)
- Sound
- PS2 keyboard
- SD cards
- MMU

Hardware that doesn't yet have software support, therefore untested so far:
- Floppy interface
- Serial ports
- SPI for things other than the SD cards
- I2C/DDC
- RC2014 slots
- Interrupts

Software status:
- CPM3 non-banked system up and running
- Still need to write a banked BIOS for CPM3
- VGA console is still very basic. Monochrome text mode only, so far.
- CPM treats the SD cards as one 8MB disk each, but with a few changes it would be possible to
  use them as multiple 8MB partitions.

Also provided is "Zemu", which is hardware and software for development. It implements the Z80 in
software on a Raspberry Pi to allow for detailed debugging and testing. The RP2040 console and SD cards
are in place to allow for development of the console software as well. The Raspberry Pi can flash the
RP2040 directly and interface with the SD cards to read and write disk images.

This project is released under the GPL license. It is primarily a personal project but contributions
are welcomed. Support is limited but feel free to open a GitHub issue.
