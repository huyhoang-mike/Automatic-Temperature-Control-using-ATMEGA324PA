;
; CoolingProject.asm

;--------------------------------------------
;define
;--------------------------------------------
;init the LCD
;LCD_D7..LCD_D4 connect to PA7..PA4
;LCD_RS connect to PA0
;LCD_RW connect to PA1
;LCD_EN connect to PA2
.equ LCDPORT = PORTD ; Set signal port reg to PORTA
.equ LCDPORTDIR = DDRD ; Set signal port dir reg to PORTA
.equ LCDPORTPIN = PIND ; Set clear signal port pin reg to PORTA
.equ LCD_RS = PINA0
.equ LCD_RW = PINA1
.equ LCD_EN = PINA2
.equ LCD_D7 = PINA7
.equ LCD_D6 = PINA6
.equ LCD_D5 = PINA5
.equ LCD_D4 = PINA4

.def shiftData = r20 ; Define the shift data register
.equ DRPort = DDRC
.equ clearSignalPort = PORTC ; Set clear signal port to PORTB
.equ clearSignalPin = 3 ; Set clear signal pin to pin 0 of PORTB
.equ shiftClockPort = PORTC ; Set shift clock port to PORTB
.equ shiftClockPin = 0 ; Set shift clock pin to pin 1 of PORTB
.equ latchPort = PORTC ; Set latch port to PORTB
.equ latchPin = 2 ; Set latch pin to pin 0 of PORTB
.equ shiftDataPort = PORTC ; Set shift data port to PORTB
.equ shiftDataPin = 1 ; Set shift data pin to pin 3 of PORTB

.def sochia=r22
.def sobichia_l=r16
.def sobichia_h=r17
.def du=r11
.def count=r18

.equ DC_dr=ddrb
.equ DC_port=portb
.equ in1=0
.equ in2=1

.equ kp_port=portc
.equ kp_pin=pinc
.equ kp_dr=ddrc
.def count_c=r17
.def count_r=r20
.def mask=r18


.org 0
rjmp main
;.org $0002
;rjmp int0_isr
.org $40

;;;;TEST FIELD
main:
ldi r16,$ff
out ddrb,r16
out ddrc,r16
	call initport
	rcall LCD_Init
;stack
	ldi r16,high(ramend)
	out sph,r16
	ldi r16,low(ramend)
	out spl,r16		
manual:
;hien huong dan max s timer1
;moi nhap so lieu
;nhan button de onled
	sbi ddrc,0
	cbi ddrc,7
	sbi portc,7
	cbi portc,0
	rcall lcd_init
	ldi ZH, high(mn)	;point to the information that is to be displayed
	ldi ZL, low(mn)
	call LCD_Send_String
	ldi r16,1
	ldi r17,0
	call LCD_Move_Cursor 
	ldi ZH, high(moi)	;point to the information that is to be displayed
	ldi ZL, low(moi)
	call LCD_Send_String
	rcall timer1_init
	rcall delay_max	
	ldi r16,1
	rcall lcd_send_command
	rcall keypad_sram
	;cbi portc,0
rjmp ADC_LOOP	
	;in r16,pinc
	;sbrc r16,7
	;rjmp cho
	;sbi portc,0 


;shift data from keypad to sram
;6 bit from keypad -> $100:$105
keypad_sram:
	push r22
	ldi xh,high($105)
	ldi xl,low($105)
	ldi yh,high($107)
	ldi yl,low($107)
	ldi r22,2
	ldi ZH, high(tl)	;point to the information that is to be displayed
	ldi ZL, low(tl)
	call LCD_Send_String
main0:
	rcall delay10ms		;de man hinh khong chop tat lien tuc
	rcall delay10ms
	rcall delay10ms
	rcall key_pad
	brcc main0
	st x+,r17
	dec r22
	brne main0
tieptuc: 
	lds r16,($105)
	subi r16,-48
	rcall lcd_send_data
	lds r16,($106)
	subi r16,-48
	rcall lcd_send_data
	ldi r16,1
	ldi r17,0
	call LCD_Move_Cursor 
	ldi r22,2
	ldi ZH, high(th) ; point to the information that is to be displayed
	ldi ZL, low(th)
	call LCD_Send_String
line2:	
	rcall delay10ms		;de man hinh khong chop tat lien tuc
	rcall delay10ms
	rcall delay10ms
	rcall key_pad
	brcc line2
	st y+,r17
	dec r22
	brne line2
xuatdata:
	lds r16,($107)
	subi r16,-48
	rcall lcd_send_data
	lds r16,($108)
	subi r16,-48
	rcall lcd_send_data
	pop r22
	rcall delay1s
	ret

;nibble cao = input
;nibble thap = output
;ket qua luu vao r17
key_pad:
	ldi r16,$0f		
	out kp_dr,r16		;pb7:pb4=input & pb3:pb0=output
	ldi mask,0b11111110
	ldi count_c,4
scan_col:
	out kp_port,mask		;dien tro keo len
	in r19,kp_pin
	in r19,kp_pin
	andi r19,$f0
	cpi r19,$f0
	brne number
nextk:
	lsl mask			;kiem tra hang ke
	inc mask			;xuat 111
	dec count_c			;quet cho den khi het cot
	brne scan_col
	clc					;chua co phim nhan
	rjmp exit
number:
	subi count_c,4
	neg count_c		;lay gia tri cot
	swap r19
	ldi r20,4
scan_row:
	ror r19
	brcc set_flag
	subi r17,-4		;sau 1 hang thi tang 4
	dec count_r
	brne scan_row
	clc				;khong co phim nhan
	rjmp exit
set_flag:
	sec
exit:
	ret	
;;	
ADC_LOOP:
	;adc
	ldi r16,0
	out ddra,r16
	ldi r16,0b01000000			;Vcc & ADC0
	sts admux,r16
	ldi r16,0b10000110			;enable adc & clk/64	
	sts adcsra,r16
loop:
	ldi r16,$01				;clear lcd
	call lcd_send_command
	lds r16,adcsra
	ori r16,(1<<adsc)		;start conversion
	sts adcsra,r16
wait: 
	lds r16,adcsra
	sbrs r16,adif
	rjmp wait
	sts adcsra,r16			;xoa co
	lds r16,adcl			;get Dout_L
	lds r17,adch			;get Dout_H
/*
	ldi r18,160
	ldi r19,0
	sub r16,r18
	sbc r17,r19
	brcc duong
am:
	neg r16
	com r17
	andi r17,0b00000011		;chi dao 2 bit cuoi
	adc r17,r19				;neu neg bi tran thi cong vao 2 bit MSB
	ldi sochia,8
	rcall div16_8
duong:
	ldi sochia,8
	rcall div16_8
;
*/
	push r16
	push r17
;;xu ly Dout	
	rcall bin16_bcd_5dg
;dieu chinh mode
	lds r16,($101)	;chu so hang ngan
	cpi r16,1
	breq maxspeed
	lds r16,($102)	;chu so hang tram
	cpi r16,7
	brcc maxspeed
	cpi r16,3
	brcc half
off:
	cbi DC_port,in1
	;cbi DC_port,in2
	ldi ZH, high(off_c) ; point to the information that is to be displayed
	ldi ZL, low(off_c)
	call LCD_Send_String
	rjmp outlcd
half:
	sbi DC_port,in1
	;cbi DC_port,in2
	ldi ZH, high(normal_c) ; point to the information that is to be displayed
	ldi ZL, low(normal_c)
	call LCD_Send_String
	rcall initTimer0_half
	rjmp outlcd
maxspeed:
	sbi DC_port,in1
	;cbi DC_port,in2
	rcall initTimer0_max
	ldi ZH, high(on_c) ; point to the information that is to be displayed
	ldi ZL, low(on_c)
	call LCD_Send_String
outlcd:
	;xu ly Temp
	;xu ly voltage lcd
	ldi r16,1
	ldi r17,0
	call LCD_Move_Cursor 
	ldi ZH, high(temp) ; point to the information that is to be displayed
	ldi ZL, low(temp)
	call LCD_Send_String
	;xu ly voltage	
	pop r17
	pop r16
	ldi r18,160
	ldi r19,0
	sub r16,r18
	sbc r17,r19
	ldi sochia,8
	rcall div16_8
;
	rcall bin16_bcd_5dg
	lds r16,($103)
	subi r16,-48
	call LCD_Send_data
	lds r16,($104)
	subi r16,-48
	call LCD_Send_data
;
	ldi r16,' '
	call LCD_Send_data
	ldi r16,'d'
	call LCD_Send_data
	ldi r16,'o'
	call LCD_Send_data
	ldi r16,' '
	call LCD_Send_data
	ldi r16,'C'
	call LCD_Send_data
	/*
	pop r17 
	pop r16
	rcall tru_16bit
	ldi sochia,4
	rcall div16_8
	rcall bin16_bcd_5dg
	lds r16,($103)
	subi r16,-48
	call LCD_Send_data
	lds r16,($104)
	subi r16,-48
	call LCD_Send_data
	ldi r16,'o'
	call LCD_Send_data
	ldi r16,'C'
	call LCD_Send_data
	*/
	rcall delay1sl
	rjmp loop

;--------------------------------------------
;LCD
;--------------------------------------------
reset_handler:
	; display the first line of information
	ldi r16,$ff
	out ddrc,r16
	sbi portd,2
	call LCD_init
	ldi ZH, high(line1) ; point to the information that is to be displayed
	ldi ZL, low(line1)
	call LCD_Send_String
	ldi r16,1
	ldi r17,0
	call LCD_Move_Cursor 
	ldi ZH, high(Dout) ; point to the information that is to be displayed
	ldi ZL, low(Dout)
	call LCD_Send_String
;here:
;rjmp here

; Function to move the cursor to a specific position on the LCD
; Assumes that the LCD is already initialized
; Input: Row number in R16 (0-based), Column number in R17 (0-based)

LCD_Move_Cursor:
	cpi r16,0 ;check if first row
	brne LCD_Move_Cursor_Second
	andi r17, 0x0F 
	ori r17,0x80 ;CHUYEN CON TRO VE DONG 2, VI TRI TRONG BYTE THAP R17
	mov r16,r17 ;CHUYEN LENH VAO R16 DE XUAT
	; Send command to LCD
	call LCD_Send_Command ;XUAT LENH TRONG R16
	ret

LCD_Move_Cursor_Second:
	cpi r16,1 ;check if second row
	brne LCD_Move_Cursor_Exit ;else exit 
	andi r17, 0x0F
	ori r17,0xC0 
	mov r16,r17 
	; Send command to LCD
	call LCD_Send_Command

	LCD_Move_Cursor_Exit:
	; Return from function
	ret
	;Subroutine to send string to LCD
	;address of the string on ZH-ZL
	;string end with Null
	.def LCDData = r16

LCD_Send_String:
	push ZH ; preserve pointer registers
	push ZL
	push LCDData
	; fix up the pointers for use with the 'lpm' instruction
	lsl ZL ; shift the pointer one bit left for the lpm instruction
	rol ZH
	; write the string of characters

LCD_Send_String_01:
	lpm LCDData, Z+ ; get a character
	cpi LCDData, 0 ; check for end of string
	breq LCD_Send_String_02 ; done
	; arrive here if this is a valid character
	call LCD_Send_Data ; display the character
	rjmp LCD_Send_String_01 ; not done, send another character
	; arrive here when all characters in the message have been sent to the LCD module
	LCD_Send_String_02:
	pop LCDData
	pop ZL ; restore pointer registers
	pop ZH
	ret
	; Subroutine to send command to LCD
	;Command code in r16
	;LCD_D7..LCD_D4 connect to PA7..PA4
	;LCD_RS connect to PA0
	;LCD_RW connect to PA1
	;LCD_EN connect to PA2

LCD_Send_Command:
	push r17
	call LCD_wait_busy ; check if LCD is busy 
	mov r17,r16 ;save the command
	; Set RS low to select command register
	; Set RW low to write to LCD
	andi r17,0xF0
	; Send command to LCD
	out LCDPORT, r17
	nop
	nop
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	nop
	cbi LCDPORT, LCD_EN
	swap r16
	andi r16,0xF0
	; Send command to LCD
	out LCDPORT, r16 
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	nop
	cbi LCDPORT, LCD_EN
	pop r17
	ret

LCD_Send_Data:
	push r17
	call LCD_wait_busy ;check if LCD is busy
	mov r17,r16 ;save the command
	; Set RS high to select data register
	; Set RW low to write to LCD
	andi r17,0xF0
	ori r17,0x01
	; Send data to LCD
	out LCDPORT, r17 
	nop
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	cbi LCDPORT, LCD_EN
	; Delay for command execution
	;send the lower nibble
	nop
	swap r16
	andi r16,0xF0
	; Set RS high to select data register
	; Set RW low to write to LCD
	andi r16,0xF0
	ori r16,0x01
	; Send command to LCD
	out LCDPORT, r16
	nop
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	cbi LCDPORT, LCD_EN
	pop r17
	ret

LCD_wait_busy:
	push r16
	ldi r16, 0b00000111 ; set PA7-PA4 as input, PA2-PA0 as output
	out LCDPORTDIR, r16
	ldi r16,0b11110010 ; set RS=0, RW=1 for read the busy flag
	out LCDPORT, r16
	nop

LCD_wait_busy_loop:
	sbi LCDPORT, LCD_EN
	nop
	nop
	in r16, LCDPORTPIN
	cbi LCDPORT, LCD_EN
	nop
	sbi LCDPORT, LCD_EN
	nop
	nop
	cbi LCDPORT, LCD_EN
	nop
	andi r16,0x80
	cpi r16,0x80
	breq LCD_wait_busy_loop
	ldi r16, 0b11110111 ; set PA7-PA4 as output, PA2-PA0 as output
	out LCDPORTDIR, r16
	ldi r16,0b00000000 ; set RS=0, RW=1 for read the busy flag
	out LCDPORT, r16
	pop r16
	ret

LCD_Init:
	; Set up data direction register for Port A
	ldi r16, 0b11110111 ; set PA7-PA4 as outputs, PA2-PA0 as output
	out LCDPORTDIR, r16
	; Wait for LCD to power up
	call DELAY10MS
	call DELAY10MS
	; Send initialization sequence
	ldi r16, 0x02 ; RETURN HOME
	call LCD_Send_Command
	ldi r16, 0x28 ; Function Set: GIAO TI?P 4 BIT CAO, 2 DONG, 5X8 DOT
	call LCD_Send_Command
	ldi r16, 0x0E ; Display Control: MAN HINH BAT, CON TRO NHAP NHAY TAT
	call LCD_Send_Command
	ldi r16, 0x01 ; Clear Display
	call LCD_Send_Command
	ldi r16, 0x80 ; CHUYEN CON TRO VE DAU DONG 1
	call LCD_Send_Command
	ret

DELAY1MS: 
	LDI R16, 8 ;1MC
LP1: 
	LDI R17, 250 ;1MC
LP2: 
	DEC R17 ;1MC
	NOP ;1MC
	BRNE LP2;2/1MC
	DEC R16 ;1MC
	BRNE LP1;2/1MC
	RET ;4MC

DELAY10MS: LDI R18, 10
LP3: CALL DELAY1MS
	 DEC R18
	 BRNE LP3
	 RET
;------------------------------------------------------
;interrupt
;------------------------------------------------------
interrupt_init:
	;cho phep ngat ngoai int0
	ldi r16,(1<<isc01)		;canh xuong
	sts eicra,r16
	ldi r16,(1<<int0)		;enable int0
	out eimsk,r16
	sei
	ret

int0_isr:
	in r17,pinc
	ldi r18,1
	eor r17,r18
	out portc,r17
	reti

;------------------------------------------------------
;shiftregister
;------------------------------------------------------

//////Initialize ports as outputs
initport:
ldi r24,(1<<clearSignalPin)|(1<<shiftClockPin)|(1<<latchPin)|(1<<shiftDataPin) 
out DRPort, r24 ; Set DDRB to output
ret


////reset
cleardata:
cbi clearSignalPort, clearSignalPin ; Set clear signal pin to low 
; Wait for a short time
sbi clearSignalPort, clearSignalPin ; Set clear signal pin to high
ret

//////Shift out data
shiftoutdata:
cbi shiftClockPort, shiftClockPin ;
ldi r18, 8 ; Shift 8 bits
shiftloop:
 sbrc shiftData, 7 ; Check if the MSB of shiftData is 1
 sbi shiftDataPort, shiftDataPin ; Set shift data pin to high
 sbi shiftClockPort, shiftClockPin ; Set shift clock pin to high
 lsl shiftData ; Shift left
 cbi shiftClockPort, shiftClockPin ; Set shift clock pin to low
 cbi shiftDataPort, shiftDataPin ; Set shift data pin to low
 dec r18
 brne shiftloop
; Latch data
sbi latchPort, latchPin ; Set latch pin to high
cbi latchPort, latchPin ; Set latch pin to low
ret

;------------------------------------------------------
;ADC
;------------------------------------------------------
adc_init:
	ldi r16,0
	out ddra,r16
	ldi r16,0b01000000			;Vcc & ADC0
	sts admux,r16
	ldi r16,0b10000110			;enable adc & clk/64	
	sts adcsra,r16
	ret

;--------------------------------------------
;CALCULATION
;--------------------------------------------
bin16_bcd_5dg:
	push xh
	push xl
	push count
	push r20
	push du
	push sobichia_l
	ldi xh,high($100)
	ldi xl,low($100)
	ldi count,5
	ldi r20,0
loop_cl:
	st x+,r20
	dec count
	brne loop_cl
	ldi sochia,10
div_nxt:
	rcall div16_8
	st -x,du
	cpi sobichia_l,0
	brne div_nxt

	pop sobichia_l
	pop du
	pop r20
	pop count
	pop xl
	pop xh
	ret



div16_8:
push count
push sochia
	ldi count,16
	clr du
sh_nxt:
	clc
	lsl sobichia_l
	rol sobichia_h
	rol du
	brcs oc_v
	sub du,sochia
	brcc gt_th
	add du,sochia
	rjmp next
oc_v:
	sub du,sochia
gt_th:
	sbr sobichia_l,1
next:
	dec count
	brne sh_nxt
pop sochia
pop count
	ret


;input/output r17:r16
mul_16bit:
	push r21
	push r10
	push r11
	ldi r21,49			;gia tri nhan
	mov r10,r16
	mov r11,r17
loop_m:
	add r16,r10
	adc r17,r11
	dec r21
	brne loop_m
	pop r11
	pop r10
	pop r21
	ret


;so bi tru r17:r16
;so tru r7:r6
;ket qua r17:r16
tru_16bit:
	push r21
	ldi r21,high(82)		;gia tri tai 0 do C
	mov r7,r21
	ldi r21,low(82)
	mov r6,r21
	sub r16,r6
	sbc r17,r7
	pop r21
	ret

;-----------------------------
;FPWM
;-----------------------------
dc: 

sbi portc,4
cbi portc,5
call initTimer0_max

start:
rjmp start

initTimer0_half:
// Set OC0A (PB3) and OC0B (PB4) pins as outputs
ldi r16, (1 << PB3) | (1 << PB4); 
out DDRB,r16
ldi r16, (1 << COM0B1)|(1 << COM0A1) | (1 << WGM00)|(1 << WGM01)
out TCCR0A,r16 // setup TCCR0A
ldi r16, (1 << CS01)
out TCCR0B,r16 // setup TCCR0B
ldi r16, 180
out OCR0A,r16 //OCRA = 100
ldi r16, 0  
out OCR0B,r16 //OCRB = 75
ret

timer1_init:
	ldi r16,0
	sts tccr1a,r16
	ldi r16,0
	sts tccr1b,r16
	ret

delay_max:
	ldi r16,3		;clk/1024
	sts tccr1b,r16
waitt:
	in r16,tifr1
	sbrs r16,tov1
	rjmp waitt
	out tifr1,r16	;xoa co
	ldi r16,0
	sts tccr1b,r16	;dung timer
	ret

initTimer0_max:
// Set OC0A (PB3) and OC0B (PB4) pins as outputs
ldi r16, (1 << PB3) | (1 << PB4); 
out DDRB,r16
ldi r16, (1 << COM0B1)|(1 << COM0A1) | (1 << WGM00)|(1 << WGM01)
out TCCR0A,r16 // setup TCCR0A
ldi r16, (1 << CS01)
out TCCR0B,r16 // setup TCCR0B
ldi r16, 255
out OCR0A,r16 //OCRA = 100
ldi r16, 0  
out OCR0B,r16 //OCRB = 75
ret

DELAY1Sl: 
LDI R16,30 
LP_1l: LDI R17,160 
LP_2l: LDI R18,50 
LP_3l: DEC R18
NOP
BRNE LP_3l 
DEC R17 
BRNE LP_2l 
DEC R16 
BRNE LP_1l 
RET 

DELAY1S: 
LDI R16,150 
LP_1: LDI R17,160 
LP_2: LDI R18,50 
LP_3: DEC R18
NOP
BRNE LP_3 
DEC R17 
BRNE LP_2 
DEC R16 
BRNE LP_1 
RET 

Dout: .db "Dout =  ",0
line1: .db "Dout = ",0
off_c: .db "OFF",0
on_c: .db "MAX SPEED",0
normal_c: .db "NORMAL SPEED",0
temp: .db "Temp = ",0
tl: .db "Temp low = ",0
th: .db "Temp high = ",0
mn: .db "OFF<1>NOR<2>MAX = ",0
moi: .db "NHAP MUC 1 & 2",0