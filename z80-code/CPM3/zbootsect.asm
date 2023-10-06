; CP/M boot sector for ZMachine and SD card
;   2023, AB0TJ
; Boot ROM / Monitor will bring us here with the SD card initialized but MMU still disabled.

include 'zmachine.inc'

bs_addr:    equ 0FC00h      ; Where does the boot sector itself get loaded to?
first_sect: equ 1           ; Start loading at sector 1 since 0 is the boot sector
n_sect:     equ 31          ; Number of sectors to load from SD card
load_addr:  equ 0100h       ; Where to load the sectors in memory
pc_addr:    equ 0100h       ; Where to jump to after loading
sd_cap:     equ 0FFFFh      ; SD capacity flag variable
dma:        equ sd_cap-2    ; Current DMA address variable
stack:      equ sd_cap-3    ; Top of stack

SDC: macro val
    ld a, val
    out (PORT_SPI_CTRL), a
endm

    org bs_addr

; Setup stuff
    di                      ; Disable interrupts just in case
    ld sp, stack            ; Set stack pointer
    ld hl, load_addr        ; Set initial load address
    ld (dma), hl            ; Put it in DMA variable

    xor a
    out (PORT_MMU_PG0H), a    ; Set A16-A23 to zero
    out (PORT_MMU_PG1H), a
    out (PORT_MMU_PG2H), a
    out (PORT_MMU_PG3H), a    ; High bits of other pages are zero, too.
    inc a
    out (PORT_MMU_PG0L), a    ; Page 0 = 0000, RAM, no WP
    ld a, 041h
    out (PORT_MMU_PG1L), a    ; Page 1 = 4000
    ld a, 081h
    out (PORT_MMU_PG2L), a    ; Page 2 = 8000
    ld a, 0C1h
    out (PORT_MMU_PG3L), a    ; Page 3 = C000
    out (PORT_MMU_ENA), a     ; Enable the MMU

; Load n_sect sectors to load_addr
    ld hl, load_addr
    ld (dma), hl
    ld bc, 0
    ld de, first_sect   ; start LBA... 16 bits so this has to be in the first 32MB of the SD card.
                        ;   load 32 bits to to bcde if >32MB support is necessary.
    ld l, n_sect        ; note... using an 8 bit register here limits us to 128k of loading.
                        ;   not really a problem since we don't support paging in this boot sector
load:
    push bc             ; save values
    push de
    push hl
    call sd_getblock    ; get a block
    ld a, '.'
    out (PORT_CON_DAT), a
    pop hl              ; restore values
    pop de
    pop bc
    dec l               ; done loading?
    jr z, done
    inc de              ; note... this only supports the first 32MB of the card
                        ;   do a full increment of bcde here if >32MB support is necessary.
    jr load             ; not done, keep loading.

done:
    ld a, 10
    out (PORT_CON_DAT), a
    jp pc_addr

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
    jr z, sd_getblock_read  ; start token
    cp 0FFh
    jr z, sd_getblock_wait  ; still waiting
    jr sd_getblock_done     ; something went wrong...
sd_getblock_read:
    ld bc, 512
    ld hl, (dma)
    call sd_rx          ; get block
    ld (dma), hl        ; update DMA address
    call spi_rx
    call spi_rx         ; get crc (and discard)
sd_getblock_done:
    SDC 1<<BIT_SPI_FAST
    ret

sd_dummy:               ; send dummy bytes, number of bytes in b
    call spi_rx
    dec b
    jr nz, sd_dummy     ; loop until done
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
    jr z, sd_cmd_wait   ; loop until we get reply
    ld e, a
    ret

sd_rx:  ; recieve bytes from SD card into memory.
        ; number of bytes in bc, start address in hl
    ld a, b
    or c
    ret z
    call spi_rx
    ld (hl), a
    inc hl
    dec bc
    jr sd_rx

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
    jr nz, spi_xfer_wait
    in a, (PORT_SPI_DATA)
    ret

ds 510-($-bs_addr)    ; fill remaining sector bytes
db 0AAh, 055h       ; boot sector signature