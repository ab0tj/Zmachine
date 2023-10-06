; ZMon v0.01
; For Zmachine SBC
; AB0TJ, 2023

include 'zmachine.inc'
include 'ascii.inc'

; at boot, RAM occupies the top two pages
startmem:   equ 08000h
endmem:     equ 0ffffh
bs_ld_addr: equ endmem-03ffh    ; where to load the boot sector

; varable storage locations
stack:      equ endmem - 14 ; top of stack
dma:        equ endmem - 12 ; current DMA address
temp:       equ endmem - 10 ; temporary space
tm_hrs:     equ endmem - 6	; current hour
tm_min:     equ endmem - 5	; current minute
tm_sec:     equ endmem - 4 	; current second	
tm_day:     equ endmem - 3	; current day
tm_mo:      equ endmem - 2 	; current month
tm_yr:      equ endmem - 1 	; current year
sd_cap:     equ endmem      ; SD card capacity flag

    org 0

start:
    ld sp, stack        ; Initialize stack pointer
    di                  ; disable interrupts

; clear LEDs
    xor a
	out (PORT_LED), a

; initialize MMU
    ; ld a, 00100000b ; RAM @0000
    ; out (MMU_PG0), a
    ; ld a, 00100001b ; RAM @4000
    ; out (MMU_PG1), a
    ; ld a, 00100010b ; RAM @8000
    ; out (MMU_PG2), a
    ; ld a, 00100011b ; RAM @C000
    ; out (MMU_PG3), a
    ; ld a, 1
    ; out (MMU_EN), a

; initialize UART
    ; ld a, 10111010b         ; reset MRA pointer, disable TX and RX
    ; out (UART_CRA), a
    ; out (UART_CRB), a
    ; ld a, 00101010b         ; Flush RX FIFO
    ; out (UART_CRA), a
    ; out (UART_CRB), a
    ; ld a, 00111010b         ; Reset transmitter
    ; out (UART_CRA), a
    ; out (UART_CRB), a
    ; ld a, 00000100b         ; MR0A: Extended mode 2 baud rates, 1 byte interrupts
    ; out (UART_MRA), a
    ; ld a, 01000100b         ; MR0B: Extended mode 2 baud rates, 6 byte interrupts
    ; out (UART_MRB), a
    ; ld a, 10010011b         ; MR1A: RTS, no parity, 8 bits, 1 byte interrupts
    ; out (UART_MRA), a
    ; ld a, 00010011b         ; MR1B: No RTS, no parity, 8 bits, 6 byte interrupts
    ; out (UART_MRB), a
    ; ld a, 00010111b         ; MR2A: normal mode, CTS, 1 stop bit
    ; out (UART_MRA), a
    ; ld a, 00000111b         ; MR2B: normal mode, no CTS, 1 stop bit
    ; out (UART_MRB), a
    ; ld a, 01100110b         ; CSRA: 115200 baud
    ; out (UART_CSRA), a
    ; ld a, 10111011b         ; CSRB: 9600 baud
    ; out (UART_CSRB), a
    ; ld a, 00110000b         ; ACR: Select first baud rate option
    ; out (UART_ACR), a
    ; ld a, 00100000b         ; IMR: unmask RxB interrupt
    ; out (UART_IMR), a
    ; ld a, 10000101b         ; UARTA: Set RTS, Enable TX and RX
    ; out (UART_CRA), a
    ; ld a, 00000101b         ; UARTB: Enable TX and RX
    ; out (UART_CRB), a

; initialize PIC
    ; ld a, 0C3h      ; JP opcode
    ; ld (030h), a    ; RST6 calls here
    ; ld (066h), a    ; NMI calls here
    ; ld hl, int_handler
    ; ld (031h), hl
    ; ld hl, nmi
    ; ld (067h), hl

    ; ld a, 00010010b         ; ICW1
    ; out (PIC_BASE), a
    ; ld a, 0                 ; ICW2
    ; out (PIC_BASE+1), a
    ; ld a, 11111011b         ; OCW1: unmask UART interrupt
    ; out (PIC_BASE+1), a
    ; im 0
    ; call int_handler        ; handle any pending interrupts

; Clear the display
    xor a
    out (PORT_CON_CTL), a
wait_clr:
    in a, (PORT_CON_STS)
    bit BIT_CON_CSR, a
    jp nz, wait_clr
    
    ld a, 4
    out (PORT_CON_CTL), a    ; enable cursor
wait_cur:
    in a, (PORT_CON_STS)
    bit BIT_CON_CSR, a
    jp nz, wait_cur

; Show signon message
    ld hl, str_signon
    call outstr             ; Show signon message

prompt:
    ld sp, stack   ; just in case there should have been a ret
    ;ei
    ld hl, str_prompt
    call outstr
    call chrin
    ld d, a                 ; save for later
    ld c, a
    call outc               ; echo
    call crlf

    ld a, d
    cp 'm'                 ; modify memory
    jp z, memin
    cp ':'			        ; intel hex
    jp z, memin_intel_p
    cp 'd'                 ; dump memory
    jp z, memout
    cp 'j'                 ; jump to memory address
    jp z, jump
    cp 'i'                 ; input from i/o
    jp z, portin
    cp 'o'                 ; output to i/o
    jp z, portout
    cp 'b'                  ; boot from SD card
    jp z, boot
    cp 'f'                  ; fill memory
    jp z, fill
    cp 'p'                  ; program flash
    jp z, program

    ld c, '?'
    call outc
    jp prompt              ; didn't understand cmd

memin:
    xor a
    ld (temp), a            ; clear error flag
    ld hl, str_hextype
    call outstr             ; raw or intel?
    call chrin
    ld c, a                ; save the input
    call outc              ; echo
    ld a, c
    cp 'r'                 ; raw
    jp z, memin_raw
    cp 'i'                 ; intel
    jp z, memin_intel
    jp prompt

memin_raw:
    ld hl, str_start
    call outstr       ; "start:"
    call in_hexaddr
    call crlf

    ld a, l
    and 0fh                 ; calculate offset
    ld b, a
    ld a, l
    and 0f0h                ; round down
    ld l, a

    call out_mempos
memin_raw_1:
    ld a, l
    and 0fh
    cp b                   ; end of offset?
    jp z, memin_raw_2
    ld c, ' '
    call outc
    ld d, (hl)
    call out_hex
    inc hl
    jp memin_raw_1

memin_raw_2:
    ld c, ' '
    call outc
    call in_hex
    ld (hl), a
    inc hl
    inc b

    ld a, b
    cp 010h                ; end of line?
    jp nz, memin_raw_2     ; nope

    call crlf
    call out_mempos
    ld b, 0
    jp memin_raw_2

memin_intel_p:
    xor a
    ld (temp), a            ; clear error flag
    jp memin_intel_1

memin_intel:
    call crlf
memin_intel_start:
    call chrin
    cp ':'                  ; wait for start code
    jp z, memin_intel_1
    cp CtrlC                ; bail if user sends Ctrl-C
    jp z, prompt
    jp memin_intel_start
memin_intel_1:
    call in_hex             ; byte count
    ld b, a
    call in_hex             ; address high
    ld h, a
    call in_hex             ; address low
    ld l, a

    ld a, b
    add a, h
    add a, l                ; start of checksum
    ld e, a

    push de
    call in_hex             ; record type
    pop de
    cp 0                    ; data
    jp z, memin_intel_2
    cp 1                    ; eof
    jp z, memin_intel_eof
    push hl
    ld hl, str_unsup
    call outstr
    xor a
    ld (temp), a
    pop hl
    jp memin_intel          ; not supported, wait for another record
memin_intel_2:
    ld a, b
    and a
    jp z, memin_intel_cksum ; b=0, checksum comes next
    push de                 ; e will get clobbered by input
    call in_hex
    pop de
    ld (hl), a              ; save byte
    add a, e                ; add to checksum
    ld e, a
    inc hl                  ; increment pointer
    dec b                   ; decrement counter
    jp memin_intel_2        ; next byte
memin_intel_cksum:
    push de                 ; input clobbers e
    call in_hex             ; get checksum
    pop de
    add a, e
    jp z, memin_intel       ; checksum is good, get next record
    ld hl, str_ckerr
    call outstr             ; show error on console
    ld a, 1                 ; set error flag
    ld (temp), a
    jp memin_intel          ; get next record
memin_intel_eof:
    call in_hex             ; checksum
    ld a, (temp)
    and a                   ; check if we had errors
    jp z, prompt            ; none - return to prompt
    ld hl, str_error
    call outstr             ; warn the user
    jp prompt

memout:
    ld hl, str_hextype      ; raw or intel?
    call outstr
    call chrin
    ld c, a                ; save the input
    call outc
    ld a, c
    cp 'r'
    jp z, memout_raw
    cp 'i'
    jp z, memout_intel
    jp prompt

memout_prompt:
    ld hl, str_start        ; "start"
    call outstr
    call in_hexaddr
    push hl
    ld hl, str_end          ; "end:"
    call outstr
    call in_hexaddr
    call crlf
    ex de, hl               ; end addr in de
    pop hl                  ; start addr in hl
    ret

memout_raw:
    call memout_prompt
    ld a, l
    and 0f0h                ; round down
    ld l, a
    ld a, e
    and 0f0h
    ld e, a
    dec hl

memout_raw_1:
    inc hl
    ld b, 0
    push de
    call out_mempos
    ld c, ' '
    call outc
memout_raw_2:
    ld d, (hl)                ; spit out a byte
    call out_hex
    ld c, ' '
    call outc

    inc hl                   ; increment the pointer
    inc b                    ; increment the byte counter
    ld a, b
    cp 010h                 ; time for a new line?
    jp nz, memout_raw_2        ; not yet

    call crlf
memout_raw_3:
    in a, (PORT_CON_STS)    ; check console status
    bit BIT_CON_RDY, a      ; check if a char is waiting
    jp z, memout_raw_4
    in a, (PORT_CON_CON)
    cp CtrlC
    jp z, prompt
    jp memout_raw_3
memout_raw_4:
    pop de
    dec hl
    ld a, h                ; see if we're at the end addr
    cp d
    jp c, memout_raw_1
    ld a, l
    cp e
    jp c, memout_raw_1
    jp prompt              ; done

memout_intel:
    call memout_prompt
    ld a, e
    and a                   ; clear carry
    sub l
    ld e, a
    ld a, d
    sbc a, h
    ld d, a                ; de=de-hl
memout_intel_1:
    ld a, d
    or a
    jp nz, memout_intel_2      ; d>0
    ld a, e
    cp 020h
    jp nc, memout_intel_2      ; e>20h
    ld b, e                ; b=e
    jp memout_intel_3
memout_intel_2:
    ld b, 020h             ; b=20h
memout_intel_3:
    and a                   ; clear carry
    ld a, e
    sub b                   ; e=e-b
    ld e, a
    ld a, d
    sbc a, 0                ; d=d-(borrow)
    ld d, a
    push de

    ld c, ':'
    call outc           ; start code
    ld d, b
    call out_hex       ; byte count
    ld d, h
    call out_hex       ; address high
    ld d, l
    call out_hex       ; address low
    ld d, 0
    call out_hex       ; record type

    ld a, b                ; checksum=b
    add a, h
    add a, l
    ld e, a                ; store checksum
memout_intel_4:
    ld a, b
    and a
    jp z, memout_intel_5       ; end of line
    ld d, (hl)                ; grab hex byte
    ld a, e
    add a, d
    ld e, a                ; checksum+=d
    call out_hex            ; send to console
    dec b                   ; decrement byte counter
    inc hl                  ; increment memory pointer
    jp memout_intel_4
memout_intel_5:
    ld a, e
    cpl                     ; compliment checksum
    inc a                   ; a++
    ld d, a
    call out_hex           ; checksum
    call crlf              ; newline
memout_intel_6:
    in a, (PORT_CON_STS)    ; check console status
    bit BIT_CON_RDY, a      ; check if a char is waiting
    jp z, memout_intel_7
    in a, (PORT_CON_CON)
    cp CtrlC
    jp z, prompt
    jp memout_intel_6
memout_intel_7:
    pop de
    ld a, d
    and a                   ; d=0
    jp nz, memout_intel_1      ; next line
    ld a, e
    and a                   ; e=0
    jp z, memout_intel_8       ; done
    jp memout_intel_1      ; next line
memout_intel_8:
    ld hl, str_hexeof
    call outstr
    jp prompt

jump: ; jump to user-specified memory address
    ld hl, str_addr
    call outstr             ; "addr:"
    call in_hexaddr
    call crlf
    jp (hl)                 ; jump!

portin: ; input from i/o port, display result
    ld hl, str_port         ; "port:"
    call outstr
    call in_hex
    ld c, a
    in a, (c)               ; read port
    ld d, a
    call crlf
    call out_hex
    jp prompt

portout: ; output specified value to i/o port
    ld hl, str_port         ; "port:"
    call outstr
    call in_hex
    ld b, a
    ld hl, str_byte         ; "byte:"
    call outstr
    call in_hex
    ld c, b
    out (c), a
    jp prompt

fill:   ; fill memory with provided value
    ld hl, str_start
    call outstr
    call in_hexaddr     ; get start address
    push hl
    ld hl, str_end
    call outstr
    call in_hexaddr     ; get end address
    push hl
    ld hl, str_byte
    call outstr
    call in_hex         ; get fill byte
    ld c, a
    pop de              ; recall end addr
    pop hl              ; recall start addr
fill_loop:
    ld a, h
    cp d                ; done?
    jp nz, fill_next
    ld a, l
    cp e
    jp z, prompt        ; return if so
fill_next:
    ld (hl), c
    inc hl
    jp fill_loop

boot:   ; boot from SD card
SDC: macro val
    ld a, val
    out (PORT_SPI_CTRL), a
endm

    SDC 0               ; Slow, no CS
    ld b, 10
    call sd_dummy       ; 80 SCK pulses with CS high
    SDC 1<<BIT_SPI_CS0
    xor a               ; CMD0: go idle
    ld b, a
    ld c, a
    ld d, a
    ld e, a             ; arg=0
    ld h, 095h          ; checksum
    call sd_cmd
    ld a, e
    cp 1                ; ack?
    jp nz, sd_error

; We don't use the response from CMD8, but apparently
;   most cards need this to initialize correctly.
    ld a, 8             ; CMD8
    ld bc, 0
    ld de, 01AAh
    ld h, 087h
    call sd_cmd
sd_init:
    ld hl, temp
    ld bc, 4
    call sd_rx          ; finish receiving CMD8 response

sd_init_1:
    ld a, 55            ; Next cmd is ACMD
    ld bc, 0
    ld de, 0
    ld h, 065h
    call sd_cmd
    ld a, 41            ; ACMD41 (init)
    ld bc, 04000h
    ld de, 0
    ld h, 077h
    call sd_cmd
    ld a, e
    cp 5
    jp nz, sd_init_2
    ld a, 1             ; Old card - try sending CMD1 instead.
    ld bc, 0
    ld h, 0F9h
    call sd_cmd
    ld a, e
sd_init_2:
    and a
    jp nz, sd_init_1
    ld hl, str_init
    call outstr

; get card OCR
    ld a, 58
    ld bc, 0
    ld de, 0
    call sd_cmd
    ld bc, 4
    ld hl, temp
    call sd_rx
    ld a, (temp)
    bit 6, a
    jp z, sd_std_size   ; Card uses byte addressing
    ld a, 1
    ld (sd_cap), a      ; Card uses LBA addressing
    jp sd_setblock
sd_std_size:
    xor a
    ld (sd_cap), a

sd_setblock:    ; set block size to 512 bytes
    ld a, 16
    ld bc, 0
    ld de, 512
    call sd_cmd
    SDC 0
    ld a, e
    and a
    jp nz, sd_error

; load first sector into memory
    ld hl, bs_ld_addr
    ld (dma), hl
    ld bc, 0
    ld de, 0
    call sd_getblock
; check boot signature
    ld a, (bs_ld_addr + 510)
    cp 0AAh
    jp nz, sd_not_bootable
    ld a, (bs_ld_addr + 511)
    cp 055h
    jp nz, sd_not_bootable
    ld hl, str_boot
    call outstr
    di
    jp bs_ld_addr

sd_not_bootable:
    ld hl, str_nosdboot
    call outstr
    jp prompt

sd_error:
    ld d, e
    ld hl, str_sderr
    call outstr
    call out_hex
    SDC 0
    jp prompt

program:
    jp prompt               ; not yet implemented

nmi:                        ; NMI: dump registers and go to monitor prompt
    push hl
    push de
    push bc
    push af
    call crlf
    ld hl, str_af
    call outstr
    pop hl
    call out_hexword
    ld hl, str_bc
    call outstr
    pop hl
    call out_hexword
    ld hl, str_de
    call outstr
    pop hl
    call out_hexword
    ld hl, str_hl
    call outstr
    pop hl
    call out_hexword
    ld hl, str_pc
    call outstr
    pop hl
    call out_hexword
    jp prompt
    
; Hex functions
in_hexaddr: ; get an address from the console in hex, return in hl
    call in_hex             ; high byte
    ld h, a
    call in_hex             ; low byte
    ld l, a
    ret

in_hex: ; get a byte from the console in ascii hex
    call in_hexnib          ; get high nibble
    rla
    rla
    rla
    rla                     ; shift to the high nibble
    and 0f0h                ; mask off low nibble
    ld e, a                 ; store the first nibble
    call in_hexnib          ; get low nibble
    or e                    ; combine high and low nibble
    ret                     ; return with byte in a

in_hexnib: ; get a single hex digit from the console, return in a
    call chrin              ; get digit from console
    cp CtrlC                ; ctrl-c?
    jp z, prompt            ; bail if so
    cp 061h                 ; lowercase?
    jp c, hexnib1           ; nope
    sub 20h                 ; lower -> upper
hexnib1:
    ld c, a                 ; save ascii version
    call hex2bin            ; convert to bin
    cp 010h                 ; carry if 0-F
    jp nc, in_hexnib        ; try again
    ld d, a                 ; a will get clobbered by outc
    call outc               ; echo
    ld a, d
    ret

hex2bin: ; convert ascii hex digit to binary nibble in a
    cp 041h                 ; >=A?
    jp c, hex2bin1          ; <A.
    sub 7                   ; correct for A-F
hex2bin1:
    sub 030h                ; get binary value
    ret

bin2hex: ; convert low nibble of a to ascii hex digit
    and 0fh                 ; mask off high nibble
    add 90h
    daa
    adc a, 40h
    daa
    ret

out_mempos: ; address in hl, plus a colon
    ld d, h
    call out_hex
    ld d, l
    call out_hex
    ld c, ':'
    call outc
    ret

out_hex: ; send byte in d as ascii hex digits
    ld a, d
    rra
    rra
    rra
    rra                    ; get the high nibble
    call bin2hex
    ld c, a
    call outc           ; send first digit
    ld a, d
    call bin2hex
    ld c, a
    call outc           ; send second digit
    ret

out_hexword:
    ld d, h
    call out_hex
    ld d, l
    call out_hex
    ret

; Console functions
outc:   ; sent a single byte to console
    in a, (PORT_CON_STS)    ; get console status
    bit BIT_CON_DSR, a  ; check if DSR is clear
    jp nz, outc         ; loop if not ready
    ld a, c             ; get byte to send
    out (PORT_CON_DAT), a ; send to console
    ret                 ; done

outstr: ; send string at hl to the console
    ld a, (hl)          ; get a byte to send
    or a                ; is it null?
    ret z               ; we're done if so
    ld c, a
    call outc           ; send this byte
    inc hl              ; increment the pointer
    jp outstr           ; loop until we find a null

crlf: ; send newline to UART
    ld c, CR
    call outc
    ld c, LF
    call outc
    ret

chrin: ; get a byte from the console, store in a
    in a, (PORT_CON_STS)    ; check input status
    bit BIT_CON_RDY, a    ; check if a char is available
    jp z, chrin           ; loop until a byte comes in
    in a, (PORT_CON_CON)  ; get the byte
    ret

; SD card functions
sd_getblock:    ; read a block specified by bcde (LBA) into the buffer
    ld a, (sd_cap)  ; we need to shift the block value left by 9 if card is not LBA
    bit 0, a
    call z, lba_adj
    SDC 1<<BIT_SPI_CS0|1<<BIT_SPI_FAST
    ld a, 17            ; CMD17: read block
    call sd_cmd         ; address already in bcde
sd_getblock_wait:      ; wait for start token
    call spi_rx
    ld e, a
    cp 0FEh
    jp z, sd_getblock_read  ; start token
    cp 0FFh
    jp z, sd_getblock_wait  ; still waiting
    jp sd_getblock_done
sd_getblock_read:
    ld bc, 512
    ld hl, (dma)
    call sd_rx      ; get block
    call spi_rx
    call spi_rx     ; get crc (and discard)
sd_getblock_done:
    SDC 1<<BIT_SPI_FAST
    ret

sd_dummy:               ; send dummy bytes, number of bytes in b
    call spi_rx
    dec b
    jp nz, sd_dummy     ; loop until done
    ret

sd_cmd:
    push af
    call spi_rx          ; send a dummy byte
    pop af
    set 6, a
    call spi_xfer        ; command | 0x40
    ld a, b
    call spi_xfer        ; arg>>24
    ld a, c
    call spi_xfer        ; arg>>16
    ld a, d
    call spi_xfer        ; arg>>8
    ld a, e
    call spi_xfer        ; arg
    ld a, h
    call spi_xfer        ; checksum
sd_cmd_wait:
    call spi_rx
    cp 0FFh
    jp z, sd_cmd_wait   ; loop until we get reply
    ld e, a
    ret

sd_rx:  ; recieve bytes from SD card into memory.
        ; number of bytes in bc, start address in hl
sd_rx_1:
    ld a, b
    and a
    jp nz, sd_rx_2
    ld a, c
    and a
    ret z
sd_rx_2:
    call spi_rx
    ld (hl), a
    inc hl
    dec bc
    jp sd_rx_1

lba_adj:    ; LBA -> byte addr
    ld b, c ; shift left by 8
    ld c, d
    ld d, e
    ld e, 0
    sla d   ; shift left by one
    rl c
    rl b
    ret

; SPI functions
spi_rx:
    ld a, 0FFh      ; enter here to just receive a byte
spi_xfer:
    out (PORT_SPI_DATA), a    ; enter here to exchange a byte
spi_xfer_wait:
    in a, (PORT_SPI_CTRL)
    bit BIT_SPI_BUSY, a
    jp nz, spi_xfer_wait
    in a, (PORT_SPI_DATA)
    ret

; interrupt stuff
int_handler:    ; Jump here on RST6
    ex af, af'
    exx
; int_handler_loop:
;     in a, (UART_SRB)    ; check fifo status
;     bit 0, a            ; check bit 0 (RxRDYA)
;     jp z, int_handler_done    ; loop until a byte comes in
;     call parse_nmea
;     jp int_handler_loop
int_handler_done:
    ; ld a, 00100000b     ; non-specific EOI
    ; out (PIC_BASE), a
    exx
    ex af, af'
    ei
    ret

; strings (null terminated)
str_crlf: db CR,LF,0
str_prompt: db CR,LF,'>',0
str_signon: db CR,LF,"ZMachine ROM Monitor",CR,LF,"V2.1 by AB0TJ, 2021-2023",CR,LF,0
str_hextype: db "(r)aw or (i)ntel:",0
str_start: db CR,LF,"start:",0
str_end: db CR,LF,"  end:",0
str_addr: db "addr:",0
str_hexeof: db ":00000001FF",CR,LF,0
str_ckerr: db " cksum err",0
str_unsup: db " unsupported", 0
str_port: db "port:",0
str_byte: db CR,LF,"byte:",0
str_error: db CR,LF,CR,LF,"WARNING: Errors detected.",CR,LF,0
str_init: db "SD card initialized. Loading boot sector...",CR,LF,0
str_boot: db "Done. Jumping to boot code...",CR,LF,CR,LF,0
str_sderr: db "Error initializing SD card!",CR,LF,0
str_nosdboot: db "SD card is not bootable.",CR,LF,0
str_af: db "AF:",0
str_bc: db " BC:",0
str_de: db " DE:",0
str_hl: db " HL:",0
str_pc: db " PC:",0