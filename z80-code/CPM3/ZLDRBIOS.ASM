
;	CP/M 3.0 LOADER BIOS FOR THE ZMachine SBC
;	Originally by John Monahan
;
;	AB0TJ, 2023

BELL	EQU	07H
CR	EQU	0DH
LF	EQU	0AH
NBYTES  EQU	512			;(512 byte sectors)
SD$CAP EQU	0FFFFH		;SD capacity flag, left in memory by initial bootloader

; Pico console equates
PORT$CON$BASE:      equ 00h                 ;Base addresss of the Pico console
PORT$CON$TMR:       equ PORT$CON$BASE       ;Timer status
PORT$CON$CON:       equ PORT$CON$BASE + 1   ;Console status
PORT$CON$CTL:       equ PORT$CON$BASE + 2   ;Console control
PORT$CON$DAT:       equ PORT$CON$BASE + 3   ;Console data
PORT$CON$STS:       equ PORT$CON$BASE + 4   ;Status flags
BT$CON$CRR:        equ 0                   ;Control read ready
BT$CON$DRR:        equ 1                   ;Data read ready
BT$CON$CSR:        equ 2                   ;Control send ready
BT$CON$DSR:        equ 3                   ;Data send ready
BT$CON$RDY:        equ 4                   ;Console ready
BT$TMR$RDY:        equ 5                   ;Timer ready
CMD$CON$ENACUR		equ 4					;Enable cursor

; SPI controller equates
PORT$SPI$BASE:      equ 10h                 ;Base address of SPI controller
PORT$SPI$DATA:      equ PORT$SPI$BASE       ;Data port
PORT$SPI$CTRL:      equ PORT$SPI$BASE + 1   ;Control port
FLAG$SPI$CS0:        equ 1                   ;CS0, 1 to enable
FLAG$SPI$CS1:        equ 2                   ;CS1
FLAG$SPI$CS2:        equ 4                   ;CS2
FLAG$SPI$INTEN:      equ 020h                ;SPI interrupt enable
FLAG$SPI$FAST:       equ 080h               ;Fast SPI clock enable
BT$SPI$BUSY:       equ 0                   ;SPI busy flag

; MMU equates
PORT$MMU$BASE:      equ 040h                ;Base address of the MMU
PORT$MMU$PG0L:      equ PORT$MMU$BASE       ;Page 0 low bits register (A14-15, ROM, WP)
PORT$MMU$PG0H:      equ PORT$MMU$BASE + 1   ;Page 0 high bits regsiter (A16-23)
PORT$MMU$PG1L:      equ PORT$MMU$BASE + 2   ;Page 1 low
PORT$MMU$PG1H:      equ PORT$MMU$BASE + 3   ;Page 1 high
PORT$MMU$PG2L:      equ PORT$MMU$BASE + 4   ;Page 2 low
PORT$MMU$PG2H:      equ PORT$MMU$BASE + 5   ;Page 2 high
PORT$MMU$PG3L:      equ PORT$MMU$BASE + 6   ;Yep you guessed it... Page 3 low
PORT$MMU$PG3H:      equ PORT$MMU$BASE + 7   ;Page 3 high, probably.
BT$MMU$ROM:        equ 0                   ;Bit 0 of low register maps the page to ROM
BT$MMU$WP:         equ 1                   ;Bit 1 write protects the page
BT$MMU$A14:        equ 6                   ;Bit 6 of low register is A14
BT$MMU$A15:        equ 7                   ;  and Bit 7 is A15
                                            ; D0-D7 map to A16-A23 on high register


	; INCLUDE CP/M 3.0 DISK DEFINITION MACROS:
	MACLIB	CPM3
	MACLIB  Z80

;	CODE BEGINS HERE:	

	JMP	BOOT			;INITIAL ENTRY ON COLD START
	JMP	WBOOT			;REENTRY ON PROGRAM EXIT, WARM START
	JMP	CONST			;RETURN CONSOLE INPUT STATUS
	JMP	CONIN			;RETURN CONSOLE INPUT CHARACTER
	JMP	CONOUT			;SEND CONSOLE OUTPUT CHARACTER
	JMP	LIST			;SEND LIST OUTPUT CHARACTER
	JMP	AUXOUT			;SEND AUXILLIARY OUTPUT CHARACTER
	JMP	AUXIN			;RETURN AUXILLIARY INPUT CHARACTER
	JMP	HOME			;SET DISKS TO LOGICAL HOME
	JMP	SELDSK			;SELECT DISK DRIVE, RETURN DISK PARAMETER INFO
	JMP	SETTRK			;SET DISK TRACK
	JMP	SETSEC			;SET DISK SECTOR
	JMP	SETDMA			;SET DISK I/O MEMORY ADDRESS
	JMP	READ			;<----------- READ PHYSICAL BLOCK(S)
	JMP	FWRITEA			;WRITE PHYSICAL BLOCK(S)
	JMP	LISTST			;RETURN LIST DEVICE STATUS
	JMP	SECTRN			;TRANSLATE LOGICAL TO PHYSICAL SECTOR
	JMP	CONOST			;RETURN CONSOLE OUTPUT STATUS
	JMP	AUXIST			;RETURN AUXILLIARY INPUT STATUS
	JMP	AUXOST			;RETURN AUXILLIARY OUTPUT STATUS
	JMP	DEVTBL			;RETURN ADDRESS OF DEVICE DEFINITION TABLE
	JMP	?CINIT			;CHANGE BAUD RATE OF DEVICE
	JMP	GETDRV			;RETURN ADDRESS OF DISK DRIVE TABLE
	JMP	MULTIO			;SET MULTIPLE RECORD COUNT FOR DISK I/O
	JMP	FLUSH			;FLUSH BIOS MAINTAINED DISK CACHING
	JMP	?MOVE			;BLOCK MOVE MEMORY TO MEMORY
	JMP	?TIME			;SIGNAL TIME AND DATE OPERATION
	JMP	BNKSEL			;SEL BANK FOR CODE EXECUTION AND DEFAULT DMA
	JMP	SETBNK			;SELECT DIFFERENT BANK FOR DISK I/O DMA OPS.
	JMP	?XMOVE			;SET SOURCE AND DEST. BANKS FOR ONE OPERATION
	JMP	0				;RESERVED FOR FUTURE EXPANSION
	JMP	0				;     DITTO
	JMP	0				;     DITTO


CONST:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

LISTST:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

AUXIST:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

AUXOST:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

FLUSH:	XRA	A			; ROUTINE HAS NO FUNCTION IN LOADER BIOS:
	RET					; RETURN A FALSE STATUS

LIST:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

AUXOUT:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

DEVTBL:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

?CINIT:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

MULTIO:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

?TIME:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

BNKSEL:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

SETBNK:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

?XMOVE:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

CONIN:	MVI   A,'Z'-40H			; ROUTINE HAS NO FUNCTION IN LOADER BIOS:
	RET

AUXIN:	MVI   A,'Z'-40H			; ROUTINE HAS NO FUNCTION IN LOADER BIOS:
	RET

CONOUT:	IN PORT$CON$STS			; ROUTINE OUTPUTS A CHARACTER IN [C] TO THE CONSOLE:
	BIT BT$CON$DSR, A
	JRNZ	CONOUT
	MOV	A,C
	OUT	PORT$CON$DAT
	RET	

CONOST:	IN	PORT$CON$STS		; RETURN CONSOLE OUTPUT STATUS:
	BIT BT$CON$DSR, A
	JRZ	CONOST$RDY			;1 IF NOT READY
	XRA	A
	RET
CONOST$RDY:
	XRA A
	DCR A
	RET

?MOVE:
	RET

SELDSK:	LXI	H,DPH0			; RETURN DPH ADDRESS FOR DRIVE A:
	RET

HOME:	LXI	B,0			; HOME SELECTED DRIVE -- TREAT AS SETTRK(0):

SETTRK:	SBCD	@TRK			; ROUTINE SETS TRACK TO ACCESS ON NEXT READ
	RET
	
SETSEC:	SBCD	@SECT			; ROUTINE SETS SECTOR TO ACCESS ON NEXT READ
	RET

SETDMA:	SBCD	@DMA			; ROUTINE SETS DISK MEMORY ADDRESS FOR READ
	RET
					;Arrive here with [BC] = Sec #,  [DE] Translation table address.
SECTRN:	
	MOV H,B
	MOV L,C 		; null sector skew
	RET

GETDRV:	LXI	H,@DTBL			; RETURN ADDRESS OF DISK DRIVE TABLE:
	RET

FINITA:	RET				; ROUTINE HAS NO FUNCTION IN LOADER BIOS:

FLOGA:	RET				; DOES NOT SEEM TO BE USED IN LOADER BIOS

FWRITEA:XRA	A			; RETURN GOOD RESULT CODE
	RET


WBOOT:	RET				; WARM BOOT IS NOT USED IN LOADER BIOS

	;;;;; BOOT
					; ROUTINE DOES COLD BOOT INITIALIZATION 

BOOT:
	MVI A,CMD$CON$ENACUR
	OUT PORT$CON$CTL	; Enable cursor
	RET

;	READ A SECTOR			;READ A SECTOR

READ:	
	SSPD	OLDSTACK		;See bottom of this module
	LXI	SP,NEWSTACK
	LHLD @TRK
	LDED @SECT
	LXI B,0
    DAD H	; Track*2
    DAD H	; *4
	DAD H	; *8
	DAD H	; *16
	DAD H	; *32 (sectors per track)
	DAD D	; add sector number (0-based)
	XCHG
	; don't forget to adjust for drive number in final BIOS
	; will need 32-bit math for that if going past 32MB
	CALL SD$GETBLOCK
	LSPD	OLDSTACK		;VIP!	
	RET


;============== SUPPORT ROUTINES FOR HARDWARE =================================
;
CSH MACRO
	MVI A, FLAG$SPI$FAST
	OUT PORT$SPI$CTRL
	ENDM

CSL MACRO
	MVI A, FLAG$SPI$FAST+FLAG$SPI$CS0
	OUT PORT$SPI$CTRL
	ENDM

SD$GETBLOCK:				; read a block specified by bcde (LBA) into memory
	LDA SD$CAP  			; we need to shift the block value left by 9 if card is not LBA
    BIT 1, A
    CZ LSHIFT9				; Adjust for byte address if necessary

	CSL
    MVI A,17            	; CMD17: read block
    CALL SD$CMD
SD$GETBLOCK$WAIT:      		; wait for start token
    CALL SPI$RX
    CPI 0FEh
    JRZ RX				  	; start token
    CPI 0FFh
    JRZ SD$GETBLOCK$WAIT  	; still waiting
    JR SD$GETBLOCK$ERROR	; error
RX:
	LHLD @DMA				;Get DMA address
	LXI D, NBYTES			;Bytes/sector for this disk format (128,256,512 or 1024)
RX$1:
    MOV A,D
    ORA E
    JZ RX$DONE
    CALL SPI$RX
    MOV M,A
    INX H
    DCX D
    JMP RX$1
RX$DONE:
	CALL SPI$RX
	CALL SPI$RX			; get CRC bytes and discard
SD$GETBLOCK$DONE:
    CSH
	XRA A				; 0 = no error
    RET

SD$GETBLOCK$ERROR:
    CSH
	MVI A,1				; 1 = error
	RET

SD$CMD:
    MOV L,A
    CALL SPI$RX			; give the card time to do things
    MOV A,L
    SETB 6,A			; command | 0x40
    CALL SPI$XFER
    MOV A,B
    CALL SPI$XFER        ; arg>>24
    MOV A,C
    CALL SPI$XFER        ; arg>>16
    MOV A,D
    CALL SPI$XFER        ; arg>>8
    MOV A,E
    CALL SPI$XFER        ; arg
    XRA A
    CALL SPI$XFER        ; checksum
SD$CMD$WAIT:
    CALL SPI$RX
    CPI 0FFh
    JRZ SD$CMD$WAIT   ; loop until we get reply
    RET

LSHIFT9:    ; Multiply bcde by 512
    MOV B,C ; shift left by 8
    MOV C,D
    MOV D,E
    MVI E,0
    SLAR D  ; shift left by one
    RALR C
    RALR B
    RET

; MID functions
SPI$RX:			; Enter here if only receiving a byte
	MVI A,0FFH
SPI$XFER:		; Enter here to send byte in A
    OUT PORT$SPI$DATA
SPI$XFER$WAIT:
    IN PORT$SPI$CTRL
    BIT BT$SPI$BUSY, A
    JRNZ SPI$XFER$WAIT
    IN PORT$SPI$DATA
    RET

;Print the acccumulator value on CRT in HEX-ASCII
;PACC:	PUSH	PSW
;	PUSH	B

;	PUSH	PSW
;	RRC
;	RRC
;	RRC
;	RRC
;	CALL	ZCONV
;	POP	PSW
;	CALL	ZCONV
;	POP	B
;	POP	PSW
;	RET

;ZCONV:	ANI	0FH		;HEX to ASCII
;	ADI	90H
;	DAA
;	ACI	40H
;	DAA
;	MOV	C,A
;	CALL	CONOUT
;	RET

;SPECIAL$PMSG:				;Cannot use @PMSG in LOADERBIOS
;	MOV	A,M
;	INX	H
;	ANA A
;	RZ	
;	MOV	C,A
;	CALL	CONOUT			;Hardware send to consol
;	JMP	SPECIAL$PMSG

@TRK:		DW	0		;2 BYTES FOR NEXT TRACK TO READ OR WRITE
@DMA:		DW	0		;2 BYTES FOR NEXT DMA ADDRESS
@SECT		DW	0		;2 BYTES FOR SECTOR
@SECTOR$BYTE$COUNT DW	0		;2 BYTES FOR Bytes/Sector

OLDSTACK:DW	0
	 DS	40
NEWSTACK:DW	0


               	; DISK DRIVE TABLE:
@DTBL:	DW	DPH0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0


DPH0:	DPH	0,DISK$DPB

	
DISK$DPB:
	DPB	512,32,512,2048,1024,1,8000H	; 512b/sec, 32 SPT, 512 tracks, 2k alloc, 1k dir entries, 1 sys trk, non-removable = 8MB

SKEW0:	SKEW	32,0,0		;<-- Note start with sector 0
	DB	'<----- END OF LOADER BIOS  >' ;Useful for debugging
