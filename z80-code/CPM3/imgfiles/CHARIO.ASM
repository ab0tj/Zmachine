	title 'Character I/O handler for z80 chip based system'

; Character I/O for the Modular CP/M 3 BIOS

	; limitations:

	;		baud rates 19200,7200,3600,1800 and 134
	;			are approximations.

	;		9600 is the maximum baud rate that is likely
	;			to work.

	;		baud rates 50, 75, and 110 are not supported


	public	?cinit,?ci,?co,?cist,?cost
	public	@ctbl

	maclib Z80	; define Z80 op codes
	maclib ports	; define port addresses
	maclib modebaud	; define mode bits and baud equates

max$devices	equ 6

	cseg

?cinit:
	mov a,c ! cpi max$devices ! jz cent$init ; init parallel printer
	rnc				; invalid device
	mov l,c ! mvi h,0		; make 16 bits from device number
	push h				; save device in stack
	dad h ! dad h ! dad h		; *8 
	lxi d,@ctbl+7 ! dad d ! mov l,m	; get baud rate
	mov a,l ! cpi baud$600		; see if baud > 300
	mvi a,44h ! jnc hi$speed	; if >= 600, use *16 mode
	mvi a,0C4h			;    else, use *64 mode
hi$speed:
	sta sio$reg$4	
	mvi h,0 ! lxi d,speed$table ! dad d	; point to counter entry
	mov a,m ! sta speed		; get and save ctc count
	pop h				; recover 
	lxi d,data$ports ! dad d	; point at SIO port address
	mov a,m ! inr a ! sta sio$port	; get and save port
	lxi d,baud$ports-data$ports ! dad d	; offset to baud rate port
	mov a,m ! sta ctc$port		; get and save
	lxi h,serial$init$tbl
	jmp stream$out

cent$init:
	lxi h,pio$init$tbl

stream$out:
	mov a,m ! ora a ! rz
	mov b,a ! inx h ! mov c,m ! inx h
	outir
	jmp stream$out


?ci:		; character input

	mov a,b ! cpi 6 ! jnc null$input ; can't read from centronics
ci1:
	call ?cist ! jz ci1		; wait for character ready
	dcr c ! inp a			; get data
	ani 7Fh				; mask parity
	ret

null$input:
	mvi a,1Ah			; return a ctl-Z for no device
	ret

?cist:		; character input status

	mov a,b ! cpi 6 ! jnc null$status ; can't read from centronics
	mov l,b ! mvi h,0		; make device number 16 bits
	lxi d,data$ports ! dad d	; make pointer to port address
	mov c,m ! inr c			; get SIO status port
	inp a				; read from status port
	ani 1			; isolate RxRdy
	rz				; return with zero
	ori 0FFh
	ret

null$status:
	xra a ! ret

?co:		; character output
	mov a,b ! cpi 6 ! jz centronics$out
	jnc null$output
	mov a,c ! push psw		; save character from <C>
	push b				; save device number
co$spin:
	call ?cost ! jz co$spin		; wait for TxEmpty
	pop h ! mov l,h ! mvi h,0	; get device number in <HL>
	lxi d,data$ports ! dad d	; make address of port address
	mov c,m				; get port address
	pop psw ! outp a		; send data
null$output:
	ret

centronics$out:
	in p$centstat ! ani 20h ! jnz centronics$out
	mov a,c ! out p$centdata	; give printer data
	in p$centstat ! ori 1 ! out p$centstat	; set strobe
	ani 7Eh ! out p$centstat		; clear strobe
	ret

?cost:		; character output status
	mov a,b ! cpi 6 ! jz cent$stat
	jnc null$status
	mov l,b ! mvi h,0
	lxi d,data$ports ! dad d
	mov c,m ! inr c 
	inp a				; get input status
	ani 4 ! rz			; test transmitter empty
	ori 0FFh ! ret			; return true if ready


cent$stat:
	in p$centstat ! cma
	ani 20h ! rz
	ori 0FFh ! ret	

baud$ports:		; CTC ports by physical device number
	db	p$baud$con1,p$baud$lpt1,p$baud$con2,p$baud$con34
	db	p$baud$con34,p$baud$lpt2

data$ports:		; serial base ports by physical device number
	db	p$crt$data,p$lpt$data,p$con2data,p$con3data
	db	p$con4data,p$lpt2data


@ctbl	db 'CRT   '	; device 0, CRT port 0
	db mb$in$out+mb$serial+mb$softbaud
	db baud$9600
	db 'LPT   '	; device 1, LPT port 0
	db mb$in$out+mb$serial+mb$softbaud+mb$xonxoff
	db baud$9600
	db 'CRT1  '	; device 2, CRT port 1
	db mb$in$out+mb$serial+mb$softbaud
	db baud$9600
	db 'CRT2  '	; device 3, CRT port 2
	db mb$in$out+mb$serial+mb$softbaud
	db baud$9600
	db 'CRT3  '	; device 4, CRT port 3
	db mb$in$out+mb$serial+mb$softbaud
	db baud$9600
	db 'VAX   '	; device 5, LPT port 1 used for VAX interface
	db mb$in$out+mb$serial+mb$softbaud
	db baud$9600
	db 'CEN   '	; device 6, Centronics parallel printer
	db mb$output
	db baud$none
	db 0			; table terminator


speed$table	db	0,255,255,255,233,208,104,208,104,69,52,35,26,17,13,7

serial$init$tbl
		db 2		; two bytes to CTC
ctc$port	ds 1		; port address of CTC
		db 47h		; CTC mode byte
speed		ds 1		; baud multiplier
		db 7		; 7 bytes to SIO
sio$port	ds 1		; port address of SIO
		db 18h,3,0E1h,4
sio$reg$4	ds 1
		db 5,0EAh 	
		db 0		; terminator

pio$init$tbl	db	2,p$zpio$2b,0Fh,07h
		db	3,p$zpio$2a,0CFh,0F8h,07h
		db 0

	end
