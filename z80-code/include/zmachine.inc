; Pico console equates
PORT_CON_BASE:      equ 00h                 ;Base addresss of the Pico console
PORT_CON_TMR:       equ PORT_CON_BASE       ;Timer status
PORT_CON_CON:       equ PORT_CON_BASE + 1   ;Console status
PORT_CON_CTL:       equ PORT_CON_BASE + 2   ;Console control
PORT_CON_DAT:       equ PORT_CON_BASE + 3   ;Console data
PORT_CON_STS:       equ PORT_CON_BASE + 4   ;Status flags
BIT_CON_CRR:        equ 0                   ;Control read ready
BIT_CON_DRR:        equ 1                   ;Data read ready
BIT_CON_CSR:        equ 2                   ;Control send ready
BIT_CON_DSR:        equ 3                   ;Data send ready
BIT_CON_RDY:        equ 4                   ;Console ready
BIT_TMR_RDY:        equ 5                   ;Timer ready

; SPI controller equates
PORT_SPI_BASE:      equ 10h;                ;Base address of SPI controller
PORT_SPI_DATA:      equ PORT_SPI_BASE       ;Data port
PORT_SPI_CTRL:      equ PORT_SPI_BASE + 1   ;Control port
BIT_SPI_CS0:        equ 0                   ;CS0, 1 to enable
BIT_SPI_CS1:        equ 1                   ;CS1
BIT_SPI_CS2:        equ 2                   ;CS2
BIT_SPI_INTEN:      equ 5                   ;SPI interrupt enable
BIT_SPI_FAST:       equ 7                   ;Fast SPI clock enable
BIT_SPI_BUSY:       equ 0                   ;SPI busy flag

; Interrupt controller equates
PORT_PIC_BASE:      equ 040h                ;Base address of the 8259
; port 20
; port 21

; Floppy controller equates
PORT_FLP_BASE:      equ 030h                ; Base address of floppy controller
; ports 30-37

; MMU equates
PORT_MMU_BASE:      equ 040h                ;Base address of the MMU
PORT_MMU_PG0L:      equ PORT_MMU_BASE       ;Page 0 low bits register (A14-15, ROM, WP)
PORT_MMU_PG0H:      equ PORT_MMU_BASE + 1   ;Page 0 high bits regsiter (A16-23)
PORT_MMU_PG1L:      equ PORT_MMU_BASE + 2   ;Page 1 low
PORT_MMU_PG1H:      equ PORT_MMU_BASE + 3   ;Page 1 high
PORT_MMU_PG2L:      equ PORT_MMU_BASE + 4   ;Page 2 low
PORT_MMU_PG2H:      equ PORT_MMU_BASE + 5   ;Page 2 high
PORT_MMU_PG3L:      equ PORT_MMU_BASE + 6   ;Yep you guessed it... Page 3 low
PORT_MMU_PG3H:      equ PORT_MMU_BASE + 7   ;Page 3 high, probably.
PORT_MMU_ENA:       equ PORT_MMU_BASE + 8   ;A write here enables the MMU
BIT_MMU_ROM:        equ 0                   ;Bit 0 of low register maps the page to ROM
BIT_MMU_WP:         equ 1                   ;Bit 1 write protects the page
BIT_MMU_A14:        equ 6                   ;Bit 6 of low register is A14
BIT_MMU_A15:        equ 7                   ;  and Bit 7 is A15
                                            ; D0-D7 map to A16-A23 on high register

; UART equates
PORT_UART_BASE:     equ 050h                ;Base port of UART
; ports 50-5F

; Debug stuff
PORT_ZEMU_CON:      equ 08h                 ;Local console of ZEMU board
PORT_LED:           equ 0FFh                ;Onboard LEDs
