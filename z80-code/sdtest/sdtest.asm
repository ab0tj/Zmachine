include 'zmachine.inc'
include 'ascii.inc'

    org 0

    xor a
    out (PORT_SPI_CTRL), a  ; No CS, slow clock

    ld b, 10
    call sd_dummy       ; Send 80 clock pulses with CS deasserted

    ld a, 1<<BIT_SPI_CS0
    out (PORT_SPI_CTRL), a  ; CS0, slow
    xor a               ; CMD0: go idle
    ld b, a
    ld c, a
    ld d, a
    ld e, a             ; arg=0
    ld h, 095h          ; checksum
    call sd_cmd
    ld a, e
    cp 1                ; ack?
    jp nz, error
    ld hl, str_idleok
    call outstr

    ld a, 8             ; CMD8
    ld bc, 0
    ld de, 01AAh
    ld h, 087h
    call sd_cmd
    ld a, e
    cp 1                ; 1=SDv2
    jp nz, sd_v1
    ld hl, str_v2
    call outstr
    jp sd_init
sd_v1:
    ld hl, str_v1
    call outstr
sd_init:
    ld hl, buffer
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
    ld hl, buffer
    call sd_rx
    ld hl, str_ocr
    call outstr
    ld b, 4
    ld hl, buffer
    call out_hex_str
    call crlf
    ld a, (buffer)
    bit 6, a
    jp z, sd_std_size
    ld hl, str_hicap
    call outstr
    ld a, 1
    ld (sd_cap), a
    jp sd_setblock
sd_std_size:
    ld hl, str_stdcap
    call outstr
    xor a
    ld (sd_cap), a

sd_setblock:    ; set block size to 512 bytes
    ld a, 16
    ld bc, 0
    ld de, 512
    call sd_cmd
    ld a, e
    and a
    jp nz, error

    ld a, 1<<BIT_SPI_FAST
    out (PORT_SPI_CTRL), a ; no CS, fast clock

; let's read some blocks
    call chrin
    ld de, 0        ; start with block 0
read_loop:
    ld bc, 0
    push de
    call sd_getblock
    ld a, e
    cp 0FEh
    jp nz, error
    ld hl, buffer
    ld b, 32
read_loop_1:
    ld a, b
    and a
    jp z, read_loop_next
read_loop_2:
    push bc
    ld b, 16
    call out_hex_str
    call crlf
    pop bc
    dec b
    jp read_loop_1
read_loop_next:
    pop de
    inc de
    jp read_loop

error:
    ld d, e
    ld hl, str_err
    call outstr
    call out_hex

stop:
    halt
    jp stop

sd_getblock:    ; read a block specified by bcde (LBA) into the buffer
    ld a, (sd_cap)  ; we need to shift the block value left by 9 if card is not LBA
    bit 0, a
    call z, lba_adj
    ld a, (1<<BIT_SPI_CS0) | (1<<BIT_SPI_FAST)
    out (PORT_SPI_CTRL), a  ; Fast, CS0
    ld a, 17            ; CMD17: read block
    call sd_cmd
sd_getblock_wait:      ; wait for start token
    ld a, 0FFh
    call mid_xfer
    ld e, a
    cp 0FEh
    jp z, sd_getblock_read  ; start token
    cp 0FFh
    jp z, sd_getblock_wait  ; still waiting
    jp sd_getblock_done
sd_getblock_read:
    ld bc, 514
    ld hl, buffer
    call sd_rx
sd_getblock_done:
    ld a, 1<<BIT_SPI_FAST
    out (PORT_SPI_CTRL), a
    ret

sd_dummy:               ; send clock pulses with idle data line
    ld a, 0FFh          ;   number of bytes in b
    call mid_xfer
    dec b
    jp nz, sd_dummy     ; send dummy byte
    ret

sd_cmd:
    push af
    ld a, 0FFh
    call mid_xfer       ; send a dummy byte
    pop af
    set 6, a
    call mid_xfer        ; command | 0x40
    ld a, b
    call mid_xfer        ; arg>>24
    ld a, c
    call mid_xfer        ; arg>>16
    ld a, d
    call mid_xfer        ; arg>>8
    ld a, e
    call mid_xfer        ; arg
    ld a, h
    call mid_xfer        ; checksum
sd_cmd_wait:
    ld a, 0FFh
    call mid_xfer
    cp 0FFh
    jp z, sd_cmd_wait   ; loop until we get reply
    ld e, a
    ret

sd_rx:  ; recieve bytes from SD card into memory.
        ; number of bytes in bc, start address in hl
    ld a, b
    or c
    ret z
    ld a, 0FFh
    call mid_xfer
    ld (hl), a
    inc hl
    dec bc
    jp sd_rx

; MID functions
mid_xfer:
    out (PORT_SPI_DATA), a
mid_xfer_wait:
    in a, (PORT_SPI_CTRL)
    bit 0, a
    jp nz, mid_xfer_wait
    in a, (PORT_SPI_DATA)
    ret

; Hex functions
bin2hex: ; convert low nibble of a to ascii hex digit
    and 0fh                 ; mask off high nibble
    add 90h
    daa
    adc a, 40h
    daa
    ret

out_hex_str:    ; output a string of hex bytes. location in hl, count in b
    ld a, b
    and a
    ret z
    ld d, (hl)
    call out_hex
    ld c, ' '
    call outc
    inc hl
    dec b
    jp out_hex_str

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

; Math
lba_adj:    ; LBA -> byte addr
    ld b, c ; shift left by 8
    ld c, d
    ld d, e
    ld e, 0
    sla d   ; shift left by one
    rl c
    rl b
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

; Strings
str_idleok: db "SD go idle ok!",CR,LF,0
str_err: db "Error: ",0
str_v2: db "Card is SDv2",CR,LF,0
str_v1: db "Card is SDv1 or MMC",CR,LF,0
str_hicap: db "Card uses LBA",CR,LF,0
str_stdcap: db "Card uses byte addressing",CR,LF,0
str_init: db "SD card initialized",CR,LF,0
str_ocr: db "OCR: ",0

    org 8000h
; Storage
sd_cap: ds 1
buffer: