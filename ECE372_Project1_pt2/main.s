.text
.global _start
.global INT_DIRECTOR

_start:
@@@@@@@@SET UP THE STACK@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R13,=STACK1  @Point to base of STACK for svc mod
	ADD R13,R13,#0x1000 @Point to top of STACK
	CPS #0x12    @Switch to IRQ mode
	LDR R13,=STACK2  @Point to IRQ mode
	ADD R13,R13,#0x1000  @Point to top of STACK
	CPS #0x13  @Back to SVC mode
@@@@@@@ Turn on UART2 CLK @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x02 @Enable clock for GPIO
	LDR R1, =0x44E00070 @CM_PER_UART2_CLKCTRL
	STR R0, [R1] @wake up the clock
@@@@@@@@ Turn on GPIO1 CLK @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R1,=0x44E000AC @Address of GPIO1_CLKCTRL register
	STR R0,[R1] @Enable GPIO1
	@Detect falling edge on GPIO1_31 which is pin 20
	ldr R0,=0x4804C000 @base address of GPIO1 register
	ADD R1,R0,#0x14C @R1 = address of GPIO1_FALLINGDETECT register
	MOV R2,#0x80000000 @Load value for bit 31
	LDR R3,[R1] @Read GPIO1_FALLINGDETECT register
	ORR R3,R3,R2 @Modify (set bit 21)
	STR R3,[R1] @Write back
	ADD R1,R0,#0x34 @Address of GPIO1_IRQSTATUS_SET_0 register
	STR R2, [R1] @Enable GPIO1_21 request on POINTRPEND1
@@@@@@@@@@@@@@ TIMER5 @@@@@@@@@@@@@@@@@@@@@@@@@@
	@turn on Timer5 CLK
	mov R2,#0x2 @value to enable Timer5 CLK
	ldr R1,=0x44E000EC  @address of CM_PER_TIMER2_CLKCTRL
	str R2,[R1] @turn on
	ldr R1,=0x44E00518  @address of PRCMCLKSEL_TIMER5 register
	str R2,[R1] @select 32Khz CLK for Timer5
	@initialize timer5 registers, with count, overflow interrupt generation
	ldr R1,=0x48046000  @Base address for Timer5 register
	mov R2,#0x1 @value to reset Timer5
	str R2,[R1,#0x10] @write to Timer5 CFG register
	mov R2,#0x2 @value to enable overflow interrupt
	str R2,[R1,#0x2C] @write to Timer5 IRQENABLE_SET
	ldr R2,=0xFFFF8000 @count value for 1 seconds
	str R2,[R1,#0x40] @Timer5 TLDR load register
	str R2,[R1,#0x3C] @write to Timer5 TCRR count register


@@@@@@@@@@@Switch to mode 1@@@@@@@@@@@@@@@@@
	LDR R1, =0x44E10954 @address of the  control module register + Spi0_d0 = 0x954
	LDR R2,=0x09 @value to switch it to mode 1
	STR R2, [R1] @switch the pin to mode 1
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	@initialize UART2
	LDR R1, =0x48024000 @address of UART2
	LDR R2, =0x4802400C @address of UART2_LCR
	MOV R3,#0x83 @the value to switch UART2 to mode A
	STR R3,[R2] @switch to mode A by writing 0x83 to the LCR
	LDR R2, =0x48024004 @address of UART2 DLH
	MOV R3,#0x01
	STR R3, [R2] @write 0x01 to DLH for a 9600 baud rate
	LDR R2, =0x48024000 @address of UART2 DLL
	MOV R3, #0x38
	STR R3, [R2] @write 0x38 to DLL for a 9600 baud rate
	ADD R2, R1, #0x20 @address of MDR1 register for UART2
	MOV R3, #0x00
	STR R3, [R2] @store 0x00 to MDR1 register to reset the bits.
	LDR R2, =0x4802400C @address of UART2_LCR
	MOV R3, #0x03
	STR R3,[R2] @store 0x00 to LCR to switch the UART2 back to operation mode and keep the 8-bit data word length
	LDR R2, =0x48024008 @address of the FIFO Control Register
	MOV R3, #0x06
	STR R3,[R2] @disable RX FIFO and TX FIFO

	@initialize INTC
	ldr R1,=0x48200000 @base address for INTC
	mov R2,#0x2 @value to reset INTC
	str R2,[R1,#0x10] @write to INTC Config register
	mov R2,#0x20000000    @unmask INTC INT 93, Timer5 interrupt
	str R2,[R1,#0xC8] @write to INTC_MIR_CLEAR2
	mov R2,#0x04 @value to unmask INTC INT 98, GPIOINTA
	str R2,[R1,#0xE8] @write to INTC_MIR_CLEAR3 register

	@initialize INTC for UART2
	LDR R1,=0x482000C8 @Address of INTC_MIR_CLEAR2 register
	MOV R2,#0x400 @value to unmask INTC INT 74 UART2
	STR R2,[R1] @Write to INTC_MIR_CLEAR2 register

	@Make sure processor IRQ enabled in CPSR
	MRS R3,CPSR @Copy CPSR to R3
	BIC R3,#0x80 @clear bit 7
	MSR CPSR_c,R3 @Write back to CPSR
	@Wait for interrupt
Loop: NOP
	B Loop

INT_DIRECTOR:
	STMFD SP!,{R0-R3,LR} @Push registers on stack
@@@@@@@@@@@@@ Check if it was UART2 @@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x482000D8 @value of the INTC_PENDING_IRQ2
	LDR R1, [R0] @load the value of the INTC_PENDING_IRQ2 register
	CMP R1,#0x00000400 @check the eleven bit to see if it from UART2
	BEQ SEND_WORD @go to send if it is else go check the button
@@@@@@@@@@@@ Check if it was timer @@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x482000D8 @Address of INTC-PENDING_IRQ2 register
	LDR R1,[R0] @read INTC-PENDING_IRQ3 register
	cmp R1,#0x20000000 @test to see if it from timer5.
	BEQ TCHK  @Not from GPIOINT1A, check for timer5, else
@@@@@@@@@@@@@ Check if it was the button @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x4804C02C @load GPIO1_IRQSTATUS_0 register address
	LDR R1,[R0] @read STATUS register
	TST R1,#0x80000000 @Check if bit 31 = 1
	BNE BUTTON_SVC @if bit 31 = 1, then button pushed
	BEQ PASS_ON @if bit 31 = 0, then go back to wait loop
TCHK:
	ldr R1,=0x482000D8 @address of INTC_PENDING_IRQ2 register
	ldr R0,[R1] @read value
	TST R0,#0x20000000 @check if interrupt from Timer5
	BEQ PASS_ON @if no then go back to wait loop, if yes then check for overflow
	ldr R1,=0x48046028 @address of Timer5 IRQSTATUS register
	ldr R0,[R1] @read the value
	TST R0,#0x2 @check bit 1
	BNE TURN_THRIT @if overflow then go SEND
	B PASS_ON @else go back to wait loop

PASS_ON:
	LDMFD SP!,{R0-R3,LR} @restore register
	SUBS PC,LR,#4 @pass execution on to wait Loop for now
BUTTON_SVC:
	LDR R0,=0x4804C02C @load GPIO1_IRQSTATUS_0 register address
	MOV R1,#0x80000000 @Value turns off GPIO1_31 interrupt request and also turn off INTC interrupt request
	STR R1,[R0] @write to GPIO1_IRQSTATUS_0 register
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	B TURN_THRIT
TURN_THRIT:
	@@@@@@@@@@@@ Enable the IER UART  for interrupt
	LDR R2, =0x48024004 @address of IER_UART2 interrupt Enable register
	MOV R3, #0x00000002 @Bit THRIT
	STR R3, [R2] @Enable the interrupt by write 1 to bit 1 of THRIT
	@turn off timer 5 interrupt request and enable INTC for next IRQ
	ldr R1,=0x48046028  @load address of Timer5 IRQSTATUS register
	mov R2,#0x2 @value to reset Timer5 Overflow IRQ request
	str R2,[R1] @write
	B SEND_WORD

TURN_TIMER:
	@@@@@@@@@@@@ start the timer5
	ldr R1,=0x48046000 @address of Timer5 TCLR register
	ldr R2,=0xFFFF8000 @value load for 2 seconds
	str R2,[R1,#0x40] @Timer5 TLDR load register
	str R2,[R1,#0x3C] @write to Timer5 TCRR count register
	mov R2,#0x01  @value to make timer wait for 2 seconds
	ldr R1,=0x48046038 @address of Timer5 TCLR register
	str R2,[R1] @write to TCLR register
	B PASS_ON

SEND_WORD:
	LDR R1, =MESSAGE_COUNT @read in the address of the pointer for message count
	LDR R2, [R1] @ read in the value that is being store there
	cmp R2, #0 @compare to see what message to sent
	BEQ SEND @if it 0 then call SEND
	cmp R2,#1 @if it 1 then call SEND2
	BEQ SEND2
	cmp R2,#2 @if it 2 then call SEND3
	BEQ SEND3
	cmp R2,#3 @if it 3 then call SEND4
	BEQ SEND4
	cmp R2,#4 @if it 4 then call SEND5
	BEQ SEND5
	cmp R2,#5 @if it 4 then call SEND5
	BEQ SEND6
	cmp R2,#6
	BEQ SEND7
	cmp R2,#7
	BEQ SEND8
	B PASS_ON @go back to wait loop

SEND8:
	LDR R0, =CHAR_PTR8 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT8 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND8

LAST_SEND8:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR8 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT8 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE8 @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #18 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send
@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R1, =MESSAGE_COUNT @load in what message it is
	mov R2, #0 @reset the counter
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER

SEND7:
	LDR R0, =CHAR_PTR7 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT7 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND7

LAST_SEND7:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR7 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT7 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE7 @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #18 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send
@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R1, =MESSAGE_COUNT @load in what message it is
	LDR R2, [R1] @read the value
	ADD R2, R2, #1 @increment the value
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER

SEND6:
	LDR R0, =CHAR_PTR6 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT6 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND6

LAST_SEND6:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR6 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT6 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE6 @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #18 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send
@@@@@@@@@@@@@@@@@@@@@@@@@@
	@LDR R1, =MESSAGE_COUNT @load in what message it is
	@mov R2, #0 @reset the counter
	@STR R2, [R1] @write back to MESSAGE_COUNT
	LDR R1, =MESSAGE_COUNT @load in what message it is
	LDR R2, [R1] @read the value
	ADD R2, R2, #1 @increment the value
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER

SEND5:
	LDR R0, =CHAR_PTR5 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT5 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND5

LAST_SEND5:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR5 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT5 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE5 @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #16 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send

@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R1, =MESSAGE_COUNT @load in what message it is
	LDR R2, [R1] @read the value
	ADD R2, R2, #1 @increment the value
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER

SEND4:
	LDR R0, =CHAR_PTR4 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT4 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND4

LAST_SEND4:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR4 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT4 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE4 @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #14 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send

@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R1, =MESSAGE_COUNT @load in what message it is
	LDR R2, [R1] @read the value
	ADD R2, R2, #1 @increment the value
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER

SEND3:
	LDR R0, =CHAR_PTR3 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT3 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND3

LAST_SEND3:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR3 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT3 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE3 @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #12 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send

@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R1, =MESSAGE_COUNT @load in what message it is
	LDR R2, [R1] @read the value
	ADD R2, R2, #1 @increment the value
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER

SEND2:
	LDR R0, =CHAR_PTR2 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT2 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND2

LAST_SEND2:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR2 @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT2 @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE2 @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #10 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send

@@@@@@@@@@@@@@@
	LDR R1, =MESSAGE_COUNT @load in what message it is
	LDR R2, [R1] @read the value
	ADD R2, R2, #1 @increment the value
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER

SEND:
	LDR R0, =CHAR_PTR @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	CMP R3,#0
	BNE PASS_ON @geater than or equal zero, more characters
	BEQ LAST_SEND

LAST_SEND:
	@Turn off NEWIRQA bit in INTC_CONTROL, so processor can respondto new IRQ
	LDR R0,=0x48200048 @address of INTC_CONTROL register
	MOV R1,#01 @value to clear bit 0
	STR R1,[R0] @write to INTC_CONTROL register
	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0, =CHAR_PTR @send character, R0 = address of pointer store
	LDR R1, [R0] @R1 = address of desire character in text string
	LDR R2, =CHAR_COUNT @R2 = address of count store location
	LDR R3, [R2] @get current character count value
	LDRB R4, [R1], #1 @read char to send from string, inc ptr in R1 by 1
	STR R1, [R0] @put incremented address back in CHAR_PTR location
	LDR R5, =0x48024000 @point at UART transmit buffer
	STRB R4, [R5] @write CHARACTER to transmit buffer.THrit send a signal
	SUBS R3, R3, #1 @decrement character counter by 1
	STR R3, [R2] @store character value counter back in memory
	LDR R3, =MESSAGE @done, reload. get address of start of string
	STR R3, [R0] @write in char pointer store location in memory
	MOV R3, #10 @load original number of char in string again
	STR R3, [R2] @write back to memory for next message send
@@@@@@@@@@@@@@@@@@@@@@
	LDR R1, =MESSAGE_COUNT @load in what message it is
	LDR R2, [R1] @read the value
	ADD R2, R2, #1 @increment the value
	STR R2, [R1] @write back to MESSAGE_COUNT
@@@@@@@@@@@@@@@ Shut off the UART INTERRUPTS @@@@@@@@@@@@@@@@@@@@@@@@
	LDR R0,=0x48024004 @load address of IER_UART2
	LDR R1,=0x00 @disable the THRIT bit
	STR R1,[R0]
	B TURN_TIMER


.data
.align 2
STACK1: .rept 1024
		.word 0x0000
		.endr
STACK2: .rept 1024
		.word 0x0000
		.endr
MESSAGE:
.byte 0x7C
.byte 0x2D
.byte 0x7C
.byte 0x9E
.ascii "Martin"
.align 2
CHAR_PTR: .word MESSAGE
CHAR_COUNT: .word 10

MESSAGE2:
.byte 0x7C
.byte 0x2D
.ascii "  Martin"
.align 2
CHAR_PTR2: .word MESSAGE2
CHAR_COUNT2: .word 10

MESSAGE3:
.byte 0x7C
.byte 0x2D
.ascii "    Martin"
.align 2
CHAR_PTR3: .word MESSAGE3
CHAR_COUNT3: .word 12

MESSAGE4:
.byte 0x7C
.byte 0x2D
.ascii "      Martin"
.align 2
CHAR_PTR4: .word MESSAGE4
CHAR_COUNT4: .word 14

MESSAGE5:
.byte 0x7C
.byte 0x2D
.ascii "        Martin"
.align 2
CHAR_PTR5: .word MESSAGE5
CHAR_COUNT5: .word 16

MESSAGE6:
.byte 0x7C
.byte 0x2D
.ascii "          Martin"
.align 2
CHAR_PTR6: .word MESSAGE6
CHAR_COUNT6: .word 18

MESSAGE7:
.byte 0x7C
.byte 0x2D
.ascii "in          Mart"
.align 2
CHAR_PTR7: .word MESSAGE7
CHAR_COUNT7: .word 18

MESSAGE8:
.byte 0x7C
.byte 0x2D
.ascii "rtin          Ma"
.align 2
CHAR_PTR8: .word MESSAGE8
CHAR_COUNT8: .word 18

.align 2
MESSAGE_COUNT: .word 0
.END
