;--------------------------------------------------------------------------------------------------
; Project:  Noah's Robot #1
; Date:     7/15/10
; Revision: 1.0
;
; Overview:
;
; This program controls a robot.
;
;
;--------------------------------------------------------------------------------------------------
;
; Revision History:
;
; 1.0   First code.
;
;--------------------------------------------------------------------------------------------------
;
;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A
;
; RA0   Out - Drive Motor Negative
; RA1   Out - Drive Motor Positive
; RA2   Out - Right LED
; RA3   Out - Left LED
; RA4
; RA5
; RA6   Out - Steering Motor Negative
; RA7   Out - Steering Motor Positive
;
; Port B
;
; RBO
; RB1
; RB2
; RB3
; RB4   To Black Pushbutton (Switch Between Menu Options / Start-Stop)
; RB5   To Red Pushbutton (Menu/Enter)
; RB6   In -  PIC Programmer Interface
; RB7   Out - PIC Programmer Interface
;
; Function by System
;
; Motors
;
; RA0   Out - Drive Motor Negative
; RA1   Out - Drive Motor Positive
; RA6   Out - Steering Motor Negative
; RA7   Out - Steering Motor Positive
;
; Buttons
;
; RB4   To Black Pushbutton (Switch Between Menu Options / Start-Stop)
; RB5   To Red Pushbutton (Menu/Enter)
;
; LEDs
;
; RA2   Out - Right LED
; RA3   Out - Left LED
;
;end of Control Control Description
;--------------------------------------------------------------------------------------------------
;
; User Inputs
;
; On startup, ...
;
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F648a	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

#INCLUDE <P16f648a.inc> 		; Microchip Device Header File

#INCLUDE <STANDARD.MAC>     	; include standard macros

; Specify Device Configuration Bits

  __CONFIG _INTOSC_OSC_NOCLKOUT & _WDT_OFF & _PWRTE_ON & _MCLRE_OFF & _LVP_OFF & _BOREN_OFF


;_INTOSC_OSC_NOCLKOUT = (see Clock Info below)
;_WDT_OFF = watch dog timer is off
;_PWRTE_ON = device will delay startup after power applied to ensure stable operation
;_MCLRE_OFF = RA5/MCLR/VPP pin function is digital input, MCLR internally to VDD 
;_LVP_OFF = RB4/PGM is digital I/O, HV on MCLR must be used for programming
;           (device cannot be programmed in system with low voltage)
;_BOREN_OFF = brown out reset is disabled - cannot be used for 3.3V supply because the
;			  brownout threshold is higher than 3.3V
;
; Clock Info 
; _INTOSC_OSC_NOCLKOUT control clock configuration
; clock config bits are FOSC<2:0> = 4,1-0 of config register
; above settings = 100 = internal oscillator, I/O on RA6/OSC2/CLKOUT and I/O on RA7/OSC1/CLKIN
;
; for improved reliability, Watch Dog code can be added and the Watch Dog Timer turned on - _WDT_ON
; turn on code protection to keep others from reading the code from the chip - _CP_ON
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Hardware Definitions

LEDS            EQU     0x05		; PORTA
LED_RIGHT       EQU     0x02		; RA2
LED_LEFT        EQU     0x03		; RA3

MOTORS          EQU     0x05		; PORTA
DRIVE_NEG       EQU     0x00		; RA0
DRIVE_POS       EQU     0x01		; RA1
STEER_NEG       EQU     0x06		; RA6
STEER_POS       EQU     0x07		; RA7


BUTTONS			EQU     0x06		; PORTB
ACTION			EQU		0X04		; RB4		To Black Pushbutton (Switch Between Menu Options / Start-Stop)
MENU			EQU		0X05		; RB5		To Red Pushbutton (Menu/Enter)

; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions

; bits in flags variable

Aa   	EQU     0x0			; undefined so far
Bb     	EQU     0x1			; undefined so far
Cc      EQU     0x2			; undefined so far

; end of Software Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in RAM
;
; Note that you cannot use a lot of the data definition directives for RAM space (such as DB)
; unless you are compiling object files and using a linker command file.  The cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
; 

; Assign variables in RAM - Bank 0 - must set RP0:RP1 to 0:0 to access

 cblock 0x20                ; starting address

    flags                   ; bit 0: 0 = undefined, 1 = undefined

	debounce0
	debounce1

    eepromAddress		    ; use to specify address to read or write from EEprom
    eepromCount	        	; use to specify number of bytes to read or write from EEprom

    scratch0                ; these can be used by any function
    scratch1
    scratch2
    scratch3
    scratch4
    scratch5
    scratch6
    scratch7
    scratch8
    scratch9
    scratch10

 endc


; Assign variables in RAM - Bank 1 - must set RP0:RP1 to 0:1 to access

 cblock 0xa0                ; starting address

    flags2	                ; bit 0: 0 = undefined, 1 = undefined
    
 endc
 
; Define variables in the memory which is mirrored in all 4 RAM banks.  This area is usually used
; by the interrupt routine for saving register states because there is no need to worry about
; which bank is current when the interrupt is invoked.
; On the PIC16F628A, 0x70 thru 0x7f is mirrored in all 4 RAM banks.

 cblock	0x70
    W_TEMP
    FSR_TEMP
    STATUS_TEMP
    PCLATH_TEMP	
 endc

; end of Variables in RAM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in EEprom
;
; Assign variables in EEprom
;

 cblock 	0x0      	; Variables start in RAM at 0x0

	eeFlags	
	var1				; undefined
	var2				; undefined

 endc

; end of Variables in EEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Power On and Reset Vectors
;

	org 0x00                ; Start of Program Memory

	goto start              ; jump to main code section

	nop			            ; Pad out so interrupt
	nop			            ; service routine gets
	nop			            ; put at address 0x0004.


; interrupt vector at 0x0004
; NOTE: You must save values (PUSH_MACRO) and clear PCLATH before jumping to the interrupt
; routine - if PCLATH has bits set it will cause a jump into an unexpected program memory
; bank.

	PUSH_MACRO              ; MACRO that saves required context registers
	clrf	STATUS          ; set to known state
    clrf    PCLATH          ; set to bank 0 where the ISR is located
    goto interruptHandler	; points to interrupt service routine

; end of Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; start
;

start:

    call    setup           ; preset variables and configure hardware

    bcf     STATUS,RP0      ; select bank 0

	bcf		MOTORS, DRIVE_POS	; turn off drive motor by setting setting both lines to GND
	bcf		MOTORS, DRIVE_NEG

	bcf		MOTORS, STEER_POS	; turn off steer motor by setting setting both lines to GND
	bcf		MOTORS, STEER_NEG


	bsf		MOTORS, DRIVE_NEG	; drive motor forward

    goto    runLoop
    
; end of start
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; runLoop
;

runLoop:

	btfss	BUTTONS, ACTION
	call	flashAlternatingFast

	btfss	BUTTONS, MENU
	call	flashAlternatingSlow

	goto	runLoop
    
; end of runLoop
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flashAlternatingFast
;
; Flashes the LEDs back and forth rapidly.
;

flashAlternatingFast:


fAF1:

	bsf		MOTORS, DRIVE_NEG	; drive motor forward
    movlw   0x10
    movwf   scratch4

fAF2:
	
	decfsz  scratch4,F     		; count down direction change timer
	goto	fAF4

    movlw   0x10
    movwf   scratch4

	btfss	MOTORS, DRIVE_NEG
	goto	fAF3

	bcf		MOTORS, DRIVE_NEG	; drive motor reverse
	bsf		MOTORS, DRIVE_POS

	goto	fAF4

fAF3:

	bsf		MOTORS, DRIVE_NEG	; drive motor forward
	bcf		MOTORS, DRIVE_POS

fAF4:

    bcf     STATUS,RP0      ; select bank 0
	bcf     PORTA,LED_LEFT	; turn on the left LED (low output turns on)
	bsf     PORTA,LED_RIGHT ; turn on the right LED (low output turns on)							

    movlw   0x0
    movwf   scratch1
    movlw   0x45
    call    bigDelayA       ; delay

    bcf     STATUS,RP0      ; select bank 0
	bsf     PORTA,LED_LEFT	; turn on the left LED (low output turns on)
	bcf     PORTA,LED_RIGHT ; turn on the right LED (low output turns on)							

    movlw   0x0
    movwf   scratch1
    movlw   0x45
    call    bigDelayA       ; delay

   	bcf     STATUS,RP0      ; select bank 0
	btfss	BUTTONS, MENU
	return

    goto    fAF2

; end of flashAlternatingFast
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flashAlternatingSlow
;
; Flashes the LEDs back and forth slowly.
;

flashAlternatingSlow:

fAS1:

	bcf		MOTORS, DRIVE_NEG	; stop drive motor

    bcf     STATUS,RP0      ; select bank 0
	bcf     PORTA,LED_LEFT	; turn on the left LED (low output turns on)
	bsf     PORTA,LED_RIGHT ; turn on the right LED (low output turns on)							

    movlw   0x0
    movwf   scratch1
    movlw   0x90
    call    bigDelayA       ; delay

    bcf     STATUS,RP0      ; select bank 0
	bsf     PORTA,LED_LEFT	; turn on the left LED (low output turns on)
	bcf     PORTA,LED_RIGHT ; turn on the right LED (low output turns on)							

    movlw   0x0
    movwf   scratch1
    movlw   0x90
    call    bigDelayA       ; delay

   	bcf     STATUS,RP0      ; select bank 0
	btfss	BUTTONS, ACTION
	return

    goto    fAS1

; end of flashAlternatingSlow
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;

setup:

;start of hardware configuration
    
    clrf    INTCON          ; disable all interrupts
                            
    movlw   0x7             ; turn off comparator, PortA pins set for I/O
    movwf   CMCON           ; 7 -> Comparator Control Register - see note in function header
    movlw   0xff
    movwf   PORTB           ; 0xff -> Port B
    bsf     STATUS,RP0      ; select RAM bank 1
    movlw   0x78
    movwf   TRISB           ; 0x78 -> TRISB = PortB I/O 0111 1000 (1=input, 0=output)
    bcf     STATUS,RP0      ; select bank 0
    movlw   0xff
    movwf   PORTA           ; 0xff -> Port A
    bsf     STATUS,RP0      ; select RAM bank 1
    movlw   0x00
    movwf   TRISA           ; 0x00 -> TRISA = PortA I/O 0000 0000 (1=input, 0=output)

    movlw   0x58
    movwf   OPTION_REG      ; Option Register = 0x58   0101 1000b
                            ; bit 7 = 0 : PORTB pullups active
                            ; bit 6 = 1 : RBO/INT interrupt on rising edge
                            ; bit 5 = 0 : Timer 0 clock source = internal instruction clock
                            ; bit 4 = 1 : Timer 0 inc on HiToLo (not used since source is internal)
                            ; bit 3 = 0 : Prescaler used by Timer 0
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    000 = 1:2 for Timer 0
                            ; bit 0 = 0 :
    
    bcf     STATUS,RP0      ; select bank 0

;end of hardware configuration

    movlw   0x3
    movwf   scratch1
    movlw   0xe8
    call    bigDelayA       ; delay 1000
 
    ; read the value stored for flags from the EEProm

    movlw   flags           ; address in RAM
    movwf   FSR
    movlw   eeFlags         ; address in EEprom
    movwf   eepromAddress
    movlw   .1
    movwf   eepromCount     ; read 1 byte
    call    readFromEEprom

 
    ; example of reading two bytes from eeporm
    ; note: these two variables must be contiguous in RAM memory

    movlw   scratch0     ; address in RAM
    movwf   FSR
    movlw   scratch0     ; address in EEprom
    movwf   eepromAddress
    movlw   .2
    movwf   eepromCount         ; read 2 bytes
    call    readFromEEprom
    	
; enable the interrupts

	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enabe TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

	bcf	    STATUS,RP0	    ; back to Bank 0
    bcf     STATUS,RP1	    ; back to Bank 0

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; saveFlagsToEEprom
;
; Saves the flags value to eeprom.
;

saveFlagsToEEprom:

    movlw   flags           ; address in RAM
    movwf   FSR
    movlw   eeFlags         ; address in EEprom
    movwf   eepromAddress
    movlw   .1
    movwf   eepromCount     ; write 1 byte
    call    writeToEEprom

    return

; end of saveFlagsToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; interruptHandler
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;

interruptHandler:

	btfsc 	INTCON,T0IF     	; Timer0 overflow interrupt?
	goto 	timer0Interrupt	    ; YES, so process Timer0
           
; Not used at this time to make interrupt handler as small as possible.
;	btfsc 	INTCON, RBIF      	; NO, Change on PORTB interrupt?
;	goto 	portB_interrupt       	; YES, Do PortB Change thing

INT_ERROR_LP1:		        	; NO, do error recovery
	;GOTO INT_ERROR_LP1      	; This is the trap if you enter the ISR
                               	; but there were no expected interrupts

endISR:

	POP_MACRO               	; MACRO that restores required registers

	retfie                  	; Return and enable interrupts

; end of interruptHandler
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; timer0Interrupt function^
;
; This function is called when the timer 0 registers overflow.
;

timer0Interrupt:            ; Routine when the Timer1 overflows

	bcf 	INTCON,T0IF     ; Clear the Timer0 overflow interrupt flag

    bcf     STATUS,RP0      ; select data bank 0 to access variables

    movf    debounce0,W
    iorwf   debounce1,W
    btfsc   STATUS,Z
    goto    t0I

    decfsz  debounce0,F     ; count down debounce timer
    goto    t0I
    decf    debounce1,F

t0I:

    goto    endISR

; end of timer0Interrupt
;--------------------------------------------------------------------------------------------------
   
;--------------------------------------------------------------------------------------------------
; SetBank0ClrWDT        Set Bank 0, IRP = Bank 0/1, Clear WatchDog timer
; 

SetBank0ClrWDT:

    bcf     STATUS,IRP      ;bank 0/1 for indirect addressing
    bcf     STATUS,RP1      ;bank 0 for direct addressing
    bcf     STATUS,RP0
    clrwdt                  ;keep watchdog from triggering
    return

;end of SetBank0ClrWDT
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; bigDelay
;
; Delays while periodically clearing WDT to prevent watch dog timer trigger.
;
; On entry at bigDelay, W holds LSB of delay value, MSB will be 0.
; On entry at bigDelayA, scratch1:W holds delay value.
;
; Notes on code for decrementing to 0:
;  subtract 1 from LSB by adding 0xff (two's comp for -1)
;  C bit will be set until LSB is 0 - only value added to 0xff that won't carry
;  When C bit not set, subtract 1 from MSB until it reaches 0 (carry will be 0)
;
; Uses W, scratch0, scratch1, scratch2, scratch3
;

bigDelay:

    clrf    scratch1

bigDelayA:

    movwf   scratch0        ; store W

L9:

    movlw   0xff            ; decrement scratch1:0 (see note above for details)
    addwf   scratch0,F
    btfss   STATUS,C
    addwf   scratch1,F
    btfss   STATUS,C
    goto    SetBank0ClrWDT  ; when counter = 0, reset stuff and return

    movlw   0x3
    movwf   scratch3
    movlw   0xe6            ; scratch2:W = 0x3e6
    call    smallDelayA
    goto    L9              ; loop

; end of bigDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; smallDelay
;
; Delays while periodically clearing WDT to prevent watch dog timer trigger.
;
; On entry at smallDelay, W holds LSB of delay value, MSB will be 0.
; On entry at smallDelayA, scratch3:W holds delay value.
;
; For some reason, 5 is subtracted from the delay value upon entry and if
; the LSB was <5 the WDT is cleared immediately instead of waiting for
; the first period to expire.
;
; Uses scratch2, scratch3
;

smallDelay:
    clrf    scratch3
smallDelayA:
    addlw   0xfb
    movwf   scratch2        ; scratch2 = W - 5 
    comf    scratch3,F      ; complement scratch3 (so can use inc to decrement it)
    movlw   0xff            ; W = -1
    btfss   STATUS,C
    goto    L10             ; if (var20 <= 4) exit

L11:
    addwf   scratch2,F      ; dec variable
    btfsc   STATUS,C        
    goto    L11             ; loop until 0

L10:
    addwf   scratch2,F      ; dec scratch2 again - starts it over at 0xff
    clrwdt                  ; keep watch dog timer from triggering
    incfsz  scratch3,F      ; increment scratch3
    goto    L11             ; loop until scratch2 is zero
    nop
    return

; end of smallDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readFromEEprom
; 
; Read block of bytes from EEprom.
;
; Address in EEprom from which to read first byte should be in eepromAddress.
; Number of bytes to read should be in eepromCount.
; Indirect register (FSR) should point to first byte in RAM to be written into.
; The block of bytes will be copied from EEprom to RAM.
;
; NOTE: This function must be modified for use with PIC16F876 - look for
;  notes in the function referring to PIC16F876 for details
; 

readFromEEprom:

loopRFE1:

	movf	eepromAddress,W	; load EEprom address from which to read
	incf	eepromAddress,F	; move to next address in EEprom	
	    
    bsf	    STATUS,RP0		; select bank 1 [for PIC16F648]
    ;bsf    STATUS,RP1		; select bank 2 [for PIC16F876]
	
    movwf	EEADR			; place in EEprom read address register

	;bsf    STATUS,RP0		; select bank 3 [for PIC16F876]
	;bcf    EECON1,EEPGD	; read from EEprom (as opposed to Flash) [for PIC16F876]
	bsf	    EECON1,RD		; perform EEProm read

	;bcf	STATUS,RP0	    ; select bank 2 [for PIC16F876]
	movf	EEDATA,W		; move data read into w
	movwf   INDF			; write to RAM
	incf	FSR,F			; move to next address in RAM

	bcf	STATUS,RP0		    ; select bank 0
	bcf	STATUS,RP1

	decfsz	eepromCount,F	; count down number of bytes transferred
	goto	loopRFE1		; not zero yet - read more bytes

	return

; end of readFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeToEeprom
;
; Writes block of bytes to EEprom.
;
; Address in EEprom to store first byte should be in eepromAddress.
; Number of bytes to write should be in eepromCount.
; Indirect register (FSR) should point to first byte in RAM to be written.
; The block of bytes will be copied from RAM to EEprom.
;

writeToEEprom:

loopWTE1:	
    
    movf	eepromAddress,W	; load EEprom address at which to store
	incf	eepromAddress,F	; move to next address in EEprom	

    bsf	    STATUS,RP0		; select bank 1 [for PIC16F648]
    ;bsf    STATUS,RP1		; select bank 2 [for PIC16F876]
	
	movwf	EEADR			; place in EEprom write address register

	movf	INDF,W			; get first byte of block from RAM
	incf	FSR,F			; move to next byte in RAM
	movwf	EEDATA			; store in EEprom write data register

	;bsf	    STATUS,RP0	; select bank 3 [for PIC16F876]
    ;bcf	EECON1,EEPGD	; write to EEprom (as opposed to Flash) [for PIC16F876]
	bsf	    EECON1,WREN		; enable EEprom write
	bcf	    INTCON,GIE		; disable all interrupts
	
	movlw	0x55
	movwf	EECON2			; put 0x55 into EECON2
	movlw	0xaa
	movwf	EECON2			; put 0xaa into EECON2
	bsf	    EECON1,WR	  	; begin the write process

waitWTE1:	
    
    btfsc	EECON1,WR		; loop until WR bit goes low (write finished)
	goto	waitWTE1

	bcf	EECON1,WREN		    ; disable writes
	bsf	INTCON,GIE		    ; re-enable interrupts

	bcf	STATUS,RP0		    ; select bank 0
	bcf	STATUS,RP1

	decfsz	eepromCount,F	; count down number of bytes transferred
	goto	loopWTE1        ; not zero yet - write more bytes

	return

; end of writeToEEprom
;--------------------------------------------------------------------------------------------------

    END
