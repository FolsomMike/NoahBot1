;--------------------------------------------------------------------------------------------------
; Project:  NoahBot1 -- Main PIC software
; Date:     5/1/16
; Revision: See Revision History notes below.
;
; Overview:
;
; This program controls a robot.
;                                     
; The program controls a drive motor, a steering motor, and monitors several button inputs and
; displays data on an LCD display.
; 
;--------------------------------------------------------------------------------------------------
; Notes on PCLATH
;
; The program counter (PC) is 13 bits. The lower 8 bits can be read and written as register PCL.
; The upper bits cannot be directly read or written.
;
; When the PCL register is written, PCLATH<4:0> is copied at the same time to the upper 5 bits of
; PC.
;
; When a goto is executed, 11 bits embedded into the goto instruction are loaded into the PC<10:0>
; while bits PCLATH<4:3> are copied to bits PC<12:11>
;
; Changing PCLATH does NOT instantly change the PC register. The PCLATH will be used the next time
; a goto is executed (or similar opcode) or the PCL register is written to. Thus, to jump farther
; than the 11 bits (2047 bytes) in the goto opcode will allow, the PCLATH register is adjusted
; first and then the goto executed.
;
;--------------------------------------------------------------------------------------------------
;
; Revision History:
;
; 1.0   Some code and concepts used from Notch Cutter main PIC 3.
;
;--------------------------------------------------------------------------------------------------
; Miscellaneous Notes
;
; incf vs decf rollover
;
; When incrementing multi-byte values, incf can be used because it sets the Z flag - then the Z
; flag is set, the next byte up should then be incremented.
; When decrementing multi-byte values, decf CANNOT be used because it sets the Z flag but NOT the
; C flag.  The next byte up is not decremented when the lower byte reaches zero, but when it rolls
; under zero.  This can be caught by loading w with 1 and then using subwf and catching the C flag
; cleared. (C flag is set for a roll-over with addwf, cleared for roll-under for subwf.)
; For a quickie but not perfect count down of a two byte variable, decf and the Z flag can be used
; but the upper byte will be decremented one count too early. WARNING: checking both bytes for
; zero to end the countdown will result in stopping 255 counts too early.
;
;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RA0   I/*,IOC,USB-D+                  ~ In ~ Jog Down switch input
; RA1   I/*,IOC,USB-D-                  ~ In ~ unused, pulled high
; RA2   not implemented in PIC16f1459   ~ 
; RA3   I/*,IOC,T1G,MSSP-SS,Vpp,MCLR    ~ In ~ Vpp, short detect
; RA4   I/O,IOC,T1G,CLKOUT,CLKR, AN3    ~ In ~ cutting current high limit trigger
; RA5   I/O,IOC,T1CKI,CLKIN             ~ Out ~ cutting current power supply on/off
; RA6   not implemented in PIC16f1459
; RA7   not implemented in PIC16f1459
;
; On version 1.0, RA0 is connected to Serial_Data_To_Master and RB5 is connected to the
; Jog Down switch. Those boards are modified with jumpers to switch RA0 and RB5 so that the
; EUSART RX on RB5 can be used to read serial data. From version 1.1 forward, the boards are
; redesigned and do not need modification.
;
; Port B        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RB0   not implemented in PIC16f1459
; RB1   not implemented in PIC16f1459
; RB2   not implemented in PIC16f1459
; RB3   not implemented in PIC16f1459
; RB4   I/O,IOC,MSSP-SDA/SDI,AN10       ~ I ~ I2CSDA, I2C bus data line
; RB5   I/O,IOC,EUSART-RX/DX,AN11       ~ I ~ EUSART-RX, serial port data in
; RB6   I/O,IOC,MSSP-SCL/SCK            ~ I ~ I2CSCL, I2C bus clock line
; RB7   I/O,IOC,EUSART-TX/CK            ~ O ~ EUSART-TX, serial port data out
;
; Port C        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RC0   I/O,AN4,C1/2IN+,ICSPDAT,Vref    ~ Out ~ ICSPDAT ~ in circuit programming data, Motor Enable
; RC1   I/O,AN5,C1/2IN1-,ICSPCLK,INT    ~ In  ~ ICSPCLK ~ in circuit programming clock, Mode Switch
; RC2   I/O,AN6,C1/2IN2-,DACOUT1        ~ In  ~ Jog Up Switch
; RC3   I/O,AN7,C1/2IN3-,DACOUT2,CLKR   ~ Out ~ Motor Direction
; RC4   I/O,C1/2OUT                     ~ Out ~ Motor Step
; RC5   I/O,T0CKI,PWM1                  ~ In  ~ cutting current low limit trigger
; RC6   I/O,AN8,PWM2,MSSP-SS            ~ Out ~ Motor Mode Step Size
; RC7   I/O,AN9,MSSP-SDO                ~ In ~ Select switch
;
;end of Hardware Control Description
;--------------------------------------------------------------------------------------------------
;
; User Inputs
;
;
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; Defines
;

; COMMENT OUT "#define DEBUG_MODE" line before using code in system.
; Defining DEBUG_MODE will insert code which simplifies simulation by skipping code which waits on
; stimulus and performing various other actions which make the simulation run properly.
; Search for "DEBUG_MODE" to find all examples of such code.

;#define DEBUG_MODE 1     ; set DEBUG_MODE testing "on" ;//debug mks -- comment this out later

    
;LCD Screen Position Codes                
                
LINE1_COL1  EQU     0x00                
LINE2_COL1  EQU     0x40

;LINE1_COL7  EQU     0x86  
;LINE2_COL2  EQU     0xc1  

; end of Defines
;--------------------------------------------------------------------------------------------------

;-------------------------------------------------------------------------------------------------- 
; LCD Display Defines
;
 
; commands
 
LCD_CLEARDISPLAY    EQU 0x01
LCD_RETURNHOME      EQU 0x02
LCD_ENTRYMODESET    EQU 0x04
LCD_DISPLAYCONTROL  EQU 0x08
LCD_CURSORSHIFT     EQU 0x10
LCD_FUNCTIONSET     EQU 0x20
LCD_SETCGRAMADDR    EQU 0x40
LCD_SETDDRAMADDR    EQU 0x80

; flags for display entry mode
 
LCD_DEC                 EQU 0x00
LCD_INC                 EQU 0x02
LCD_SHIFT               EQU 0x01
LCD_NO_SHIFT            EQU 0x00

; flags for display on/off control
 
LCD_DISPLAYON   EQU 0x04
LCD_DISPLAYOFF  EQU 0x00
LCD_CURSORON    EQU 0x02
LCD_CURSOROFF   EQU 0x00
LCD_BLINKON     EQU 0x01
LCD_BLINKOFF    EQU 0x00

; flags for display/cursor shift

LCD_DISPLAYMOVE EQU 0x08
LCD_CURSORMOVE  EQU 0x00
LCD_MOVERIGHT   EQU 0x04
LCD_MOVELEFT    EQU 0x00

; flags for function set
    
LCD_8BITMODE    EQU 0x10
LCD_4BITMODE    EQU 0x00
LCD_2LINE       EQU 0x08
LCD_1LINE       EQU 0x00
LCD_5x10DOTS    EQU 0x04
LCD_5x8DOTS     EQU 0x00

; control flags
     
LCD_BACKLIGHT   EQU .3
LCD_EN          EQU .2                    ; high pulse to write or read data from LCD
LCD_RW          EQU .1                    ; 1 = read 0 = write
LCD_RS          EQU .0                    ; 1 = data register 0 = instruction register
 
; end of LCD Display Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F1459	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

	errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
					 ;	(this is displayed when a RAM address above bank 1 is used -- it is
					 ;	 expected that the lower bits will be used as the lower address bits)

#INCLUDE <p16f1459.inc> 		; Microchip Device Header File


;#include <xc.h>

; Specify Device Configuration Bits

; CONFIG1
; __config 0xF9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_DISABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_INTOSC -> internal oscillator, I/O function on CLKIN pin
; _WDTE_OFF -> watch dog timer disabled
; _PWRTE_OFF -> Power Up Timer disabled
; _MCLRE_OFF -> MCLR/VPP pin is digital input
; _CP_OFF -> Flash Program Memory Code Protection off
; _BOREN_OFF -> Power Brown-out Reset off
; _CLKOUTEN_OFF -> CLKOUT function off, I/O or oscillator function on CLKOUT pin
; _IESO_OFF -> Internal/External Oscillator Switchover off
;   (not used for this application since there is no external clock)
; _FCMEN_OFF -> Fail-Safe Clock Monitor off
;   (not used for this application since there is no external clock)
; _WRT_ALL -> Flash Memory Self-Write Protection on -- no writing to flash
;
; _CPUDIV_NOCLKDIV -> CPU clock not divided
; _USBLSCLK_48MHz -> only used for USB operation
; _PLLMULT_4x -> sets PLL (if enabled) multiplier -- 4x allows software override
; _PLLEN_DISABLED -> the clock frequency multiplier is not used
;
; _STVREN_ON -> Stack Overflow/Underflow Reset on
; _BORV_LO -> Brown-out Reset Voltage Selection -- low trip point
; _LPBOR_OFF -> Low-Power Brown-out Reset Off
; _LVP_OFF -> Low Voltage Programming off
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Hardware Definitions
;
; NOTE: Need to change all output port definitions to the port latch instead as per the next
; note...but all code needs to be examined because the latches are in a different bank and the
; banksel commands need to be updated.
; NOTE: All ports for outputs are defined as the latches for the port. Writing to the latches
; avoids problems with the read-modify-write system of the PIC.
;
; For inputs, the port must be read.
;

; Port A

SWITCH_P        EQU     PORTA
RED_SW		EQU	RA0
BLK_SW		EQU	RA1
		
LEDS_L		EQU	LATA
LEFT_LED	EQU	RA4
RIGHT_LED	EQU	RA5
		
; Port B		

UNUSED_B_P	EQU	PORTB
UNUSED_B_L	EQU	LATB
UNUSED_RB5	EQU	RB5
UNUSED_RB7	EQU	RB7

I2CSDA_LINE     EQU     RB4
I2CSCL_LINE     EQU     RB6

; Port C

MOTORS_L	EQU	LATC
     
ICSPDAT		EQU	RC0
ICSPCLK		EQU	RC1
UNUSED_RC2	EQU	RC2
UNUSED_RC3	EQU	RC3
STEER_MOTOR_A	EQU	RC4
STEER_MOTOR_B	EQU	RC5
DRIVE_MOTOR_A	EQU	RC6
DRIVE_MOTOR_B	EQU	RC7

;bits in switchStates variable

MODE_SW_FLAG            EQU     0
JOG_UP_SW_FLAG          EQU     1
JOG_DOWN_SW_FLAG        EQU     2          
SELECT_SW_FLAG          EQU     3
ELECTRODE_PWR_SW_FLAG   EQU     4
AC_OK_FLAG              EQU     5

;bits in outputStates variable

UNUSED1                 EQU     0
UNUSED2                 EQU     1
UNUSED3                 EQU     2
       
; I2C bus ID bytes for writing and reading to LCD
; bits 7-1 = device address of 0x3f                 
; upper nibble = 0111 (bits 7-4)
; chip A2-A0 inputs = 111 (bits 3-1)
; R/W bit set to 0 (bit 0) for writing
; R/W bit set to 1 (bit 0) for reading
                 
LCD_WRITE_ID    EQU     b'01111110'
LCD_READ_ID     EQU     b'01111111'

; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions

; bits in flags variable

UNUSED_1_0  EQU     0x0
UNUSED_1_1  EQU     0x1
UNUSED_1_2  EQU     0x2
UNUSED_1_3  EQU     0x3
UNUSED_1_4  EQU     0x4
UNUSED_1_5  EQU     0x5
UNUSED_1_6  EQU     0x6
UNUSED_1_7	EQU     0x7

; bits in flags2 variable

HEADER_BYTE_1_RCVD  EQU 0
HEADER_BYTE_2_RCVD  EQU 1
LENGTH_BYTE_VALID   EQU 2
SERIAL_PACKET_READY EQU 3    
LCD_REG_SEL         EQU 4
LCD_BACKLIGHT_SEL   EQU 5
WARNING_ERROR       EQU 6
UNUSED_2_3          EQU 7
    
; bits in flags3 variable

UNUSED_3_0      EQU     0
DEBOUNCE_ACTIVE EQU     1
TIME_CRITICAL   EQU     2    
 
; bits in statusFlags variable

SERIAL_COM_ERROR    EQU 0
I2C_COM_ERROR       EQU 1

SERIAL_RCV_BUF_LEN  EQU .10

SERIAL_XMT_BUF_LEN  EQU .64

; Serial Port Packet Commands

NO_ACTION_CMD               EQU .0
ACK_CMD                     EQU .1
SET_OUTPUTS_CMD             EQU .2
SWITCH_STATES_CMD           EQU .3
LCD_DATA_CMD                EQU .4
LCD_INSTRUCTION_CMD         EQU .5
LCD_BLOCK_CMD               EQU .6

; LCD Display Commands

CLEAR_SCREEN_CMD	EQU		0x01

; LCD Display On/Off Command bits

;  bit 3: specifies that this is a display on/off command if 1
;  bit 2: 0 = display off, 1 = display on
;  bit 1: 0 = cursor off, 1 = cursor on
;  bit 0: 0 = character blink off, 1 = blink on

DISPLAY_ONOFF_CMD_FLAG	EQU		0x08
DISPLAY_ON_FLAG			EQU		0x04
CURSOR_ON_FLAG			EQU		0x02
BLINK_ON_FLAG			EQU		0x01

; end of Software Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in RAM
;
; Note that you cannot use a lot of the data definition directives for RAM space (such as DB)
; unless you are compiling object files and using a linker command file.  The cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
; 

; Assign variables in RAM - Bank 0
; Bank 0 has 80 bytes of free space

 cblock 0x20                ; starting address

    flags                   ; bit 0: 0 =
                            ; bit 1:

    flags2                  ; bit 0: 1 = first serial port header byte received
                            ; bit 1: 1 = second serial port header byte received
                            ; bit 2: 1 = serial port packet length byte received and validated
                            ; bit 3: 1 = data packet ready for processing
                            ; bit 4: 0 = LCD instruction register, 1 = LCD data register
                            ; bit 5: 0 = LCD backlight off, 1 = on
							; bit 6: 0 = Warning message, 1 = Error message
							; bit 7: 0 =
    
    flags3                  ; bit 0:
                            ; bit 1: 0 = debounce timer, inactive 1 = timer active
                            ; bit 2: 0 = no time criticality, 1 = time critical operation
                            ; bit 3:
                            ; bit 4:
                            ; bit 5:
                            ; bit 6:
                            ; bit 7:
        
    statusFlags             ; bit 0: 1 =
                            ; bit 1: 1 = 
                            
    menuOption              ; tracks which menu option is currently selected                            
                            
    xmtDataTimer            ; used to control rate of data sending
    
    switchStates

    switchStatesPrev        ; state of switches the last time they were scanned
                            ; bit assignments same as for buttonState

    switchStatesRemote      ; switch states reported by remote device such as User Interface Board

    outputStates            ; state of the outputs
    
    debounceH               ; switch debounce timer decremented by the interrupt routine
    debounceL

    secDelayCnt
    msDelayCnt
    bigDelayCnt
    smallDelayCnt
    
    cursorPos               ; contains the location of the cursor
                            ; NOTE: LCD addressing is screwy - the lines are not in sequential order:
                            ; line 1 column 1 = 0x80  	(actually address 0x00)
                            ; line 2 column 1 = 0xc0	(actually address 0x40)
                            ; line 3 column 1 = 0x94	(actually address 0x14)
                            ; line 4 column 1 = 0xd4	(actually address 0x54)
							;
							; To address the second column in each line, use 81, C1, 95, d5, etc.
							;
							; The two different columns of values listed above are due to the fact that the address
							; is in bits 6:0 and control bit 7 must be set to signal that the byte is an address
							; byte.  Thus, 0x00 byte with the control bit set is 0x80.  The 0x80 value is what is
							; actually sent to the LCD to set address 0x00.
							;
							;  Line 3 is actually the continuation in memory at the end of line 1
							;    (0x94 - 0x80 = 0x14 which is 20 decimal -- the character width of the display)
							;  Line 4 is a similar extension of line 2.
                            ;
							; Note that the user manual offered by Optrex shows the line addresses
							; for 20 character wide displays at the bottom of page 20.

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

    I2CScratch0                ; these are used by I2C functions
    I2CScratch1
    I2CScratch2
    I2CScratch3
    I2CScratch4
    I2CScratch5
    
	; next variables ONLY written to by interrupt code

	intScratch0				; scratch pad variable for exclusive use by interrupt code

	; end of variables ONLY written to by interrupt code

    serialPortErrorCnt      ; number of com errors from Rabbit via serial port
    slaveI2CErrorCnt        ; number of com errors from Slave PICs via I2C bus

    serialRcvPktLenMain     ; used by main to process completed packets
    serialRcvPktCntMain     ; used by main to process completed packets    
    
    usartScratch0
    usartScratch1
    serialIntScratch0
    
    ; used by serial receive interrupt    
    
    serialRcvPktLen
    serialRcvPktCnt
    serialRcvBufPtrH
    serialRcvBufPtrL
    serialRcvBufLen
    
    ; used by serial transmit interrupt
    
    serialXmtBufNumBytes
    serialXmtBufPtrH
    serialXmtBufPtrL
    serialXmtBufLen

 endc
               
;-----------------

; Assign variables in RAM - Bank 1
; Bank 1 has 80 bytes of free space

 cblock 0xa0                ; starting address


 endc

 ;-----------------

; Assign variables in RAM - Bank 2
; Bank 2 has 80 bytes of free space
 
; WARNING: These buffers may be large enough to overrun the following banks. Linear indirect
; addressing is used to access them. They may be moved to a higher bank if necessary to make room
; for variables in this bank.

 cblock 0x120                ; starting address

    serialRcvBuf:SERIAL_RCV_BUF_LEN

    serialXmtBuf:SERIAL_XMT_BUF_LEN
 
    endc

; Compute address of serialRcvBuf in linear data memory for use as a large buffer
RCV_BUF_OFFSET EQU (serialRcvBuf & 0x7f) - 0x20
SERIAL_RCV_BUF_LINEAR_ADDRESS   EQU ((serialRcvBuf/.128)*.80)+0x2000+RCV_BUF_OFFSET
SERIAL_RCV_BUF_LINEAR_LOC_H     EQU high SERIAL_RCV_BUF_LINEAR_ADDRESS
SERIAL_RCV_BUF_LINEAR_LOC_L     EQU low SERIAL_RCV_BUF_LINEAR_ADDRESS
    
; Compute address of serialXmtBuf in linear data memory for use as a large buffer
XMT_BUF_OFFSET EQU (serialXmtBuf & 0x7f) - 0x20
SERIAL_XMT_BUF_LINEAR_ADDRESS   EQU ((serialXmtBuf/.128)*.80)+0x2000+XMT_BUF_OFFSET
SERIAL_XMT_BUF_LINEAR_LOC_H     EQU high SERIAL_XMT_BUF_LINEAR_ADDRESS
SERIAL_XMT_BUF_LINEAR_LOC_L     EQU low SERIAL_XMT_BUF_LINEAR_ADDRESS

;-----------------
 
; Define variables in the memory which is mirrored in all RAM banks.
;
; On older PICs, this section was used to store context registers during an interrupt as the
; current bank was unknown upon entering the interrupt. Now, the section can be used for any
; purpose as the more powerful PICs automatically save the context on interrupt.
;
;	Bank 0		Bank 1		Bank 2		Bank3
;	70h-7fh		f0h-ffh		170h-17fh	1f0h-1ffh
;

 cblock	0x70

    BANKSEL_TEMP
    FSR0H_TEMP
    FSR0L_TEMP
    FSR1H_TEMP
    FSR1L_TEMP

 endc

; end of Variables in RAM
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

; NOTE: You must set PCLATH before jumping to the interrupt routine - if PCLATH is wrong the
; jump will fail.
 
    movlp   high handleInterrupt
    goto    handleInterrupt	; points to interrupt service routine

    ; end of Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; start
;

start:

    movlp   high setup      ; preset variables and configure hardware
    call    setup           
    movlp   high start

    goto    debugFunction   ;debug mks
    
    movlw   .10                 ; alternating LED flash for 10 cycles
    call    alternateFlashLEDs
        
loop623:
        
    ;call    driveForward
    call    steerLeft

    movlw   .3
    call    delayWSeconds
    
    ;call    stopDrive
    call    steerCenter
    
    movlw   .2
    call    delayWSeconds
    
    ;call    driveReverse
    call    steerRight
    
    movlw   .3
    call    delayWSeconds

    ;call    stopDrive
    call    steerCenter

    movlw   .3                 ; alternating LED flash for 10 cycles
    call    alternateFlashLEDs
    
    goto    loop623
    
menuLoop:
    
    call    doMainMenu      ; display and handle the main menu

    goto    menuLoop
    
; end of start
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; debugFunction
;
; Debugging functions.
;    
    
debugFunction:
    
;debug mks    
 
    movlw   high string0                ; "NoahBot 1.0"
    movwf   FSR1H
    movlw   low string0
    movwf   FSR1L
    call    printStringI2CUnbuffered
  
    call    gotoLCDLine2Col1
            
    movlw   high string1                ; "Noah = tater tot"
    movwf   FSR1H
    movlw   low string1
    movwf   FSR1L
    call    printStringI2CUnbuffered
        
    movlw   .4
    call    delayWSeconds
    
    movlw   high string2                ; "Destructo Mode"
    movwf   FSR1H
    movlw   low string2
    movwf   FSR1L
    movlw   .10                         ; flash warning x number of times
    call    displayWarningOnLCD
    
    movlw   high string3                ; "I'm f*ing lost"
    movwf   FSR1H
    movlw   low string3
    movwf   FSR1L
    movlw   .255                        ; flash warning x number of times
    call    displayErrorOnLCD
        
debugLoop:
    goto    debugLoop
    
;debug mks end    
    
    return
    
; end of debugFunction
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearLCD
;
; Clears the LCD display and moves the cursor to line 1 column 1.
;
        
clearLCD:

    movlw   LCD_CLEARDISPLAY
    call    sendControlCodeToLCD
    
    movlw   .3                      ; delay at least 1.53 ms after Clear Display command
    call    delayWms
 
    return
    
; end of clearLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; gotoLCDLine1Col1
;
; Moves the cursor to line 1 column 1 on the LCD.
;
        
gotoLCDLine1Col1:
    
    movlw   LCD_SETDDRAMADDR | 0x00     ; move to row 1, col 1
    goto    sendControlCodeToLCD
    
; end of gotoLCDLine1Col1
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; gotoLCDLine2Col1
;
; Moves the cursor to line 2 column 1 on the LCD.
;
        
gotoLCDLine2Col1:
    
    movlw   LCD_SETDDRAMADDR | 0x40     ; move to row 2, col 1
    goto    sendControlCodeToLCD
    
; end of gotoLCDLine2Col1
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayWarningOnLCD
;
; Flashes "* Warning *" on line 1 while displaying the string pointed by FSR1 on line 2 of the LCD.
;
; On entry:
;
; WREG contains number of times to flash
; FSR1 points to string to be displayed
;
    
displayWarningOnLCD:

    banksel flags2                      
    bcf     flags2,WARNING_ERROR        ; clear flag to indicate a warning message
    goto    dWOLCD1
    
displayErrorOnLCD:    

    banksel flags2
    bsf     flags2,WARNING_ERROR        ; set flag to indicate an error message
        
dWOLCD1:
    
    banksel I2CScratch5                 ; store the flash count
    movwf   I2CScratch5
    
    call    clearLCD
    
    call    gotoLCDLine2Col1            ; display the specified string on line 2
    call    printStringI2CUnbuffered

dWOLCDLoop:
    
    call    gotoLCDLine1Col1

    banksel flags2
    btfsc   flags2,WARNING_ERROR
    goto    dWOLCD2
    
    movlw   high warningStr             ; display "* Warning *" on line 1
    movwf   FSR1H
    movlw   low warningStr
    movwf   FSR1L
    goto    dWOLCD3
    
dWOLCD2:    

    movlw   high errorStr               ; display "* Error *" on line 1
    movwf   FSR1H
    movlw   low errorStr
    movwf   FSR1L
        
dWOLCD3:
    
    call    printStringI2CUnbuffered

    movlw   .255                        ; delay 255 ms
    call    delayWms

    call    gotoLCDLine1Col1            ; erase line 1
    movlw   high clearStr
    movwf   FSR1H
    movlw   low clearStr
    movwf   FSR1L
    call    printStringI2CUnbuffered

    movlw   .255                        ; delay 255 ms
    call    delayWms
        
    banksel I2CScratch5
    decfsz  I2CScratch5,F
    goto    dWOLCDLoop    
    
    return
    
; end of displayWarningOnLCD
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; longCallSendNybblesToLCDViaI2C
;
; Sets the PCLATH register to allow a call to the function which is in a different memory page. Sets
; the register back to the local page after the call returns.
;
        
longCallSendNybblesToLCDViaI2C:
    
    movlp   high sendNybblesToLCDViaI2C
    call    sendNybblesToLCDViaI2C
    movlp   longCallSendNybblesToLCDViaI2C
    
    return
    
; end of longCallSendNybblesToLCDViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendControlCodeToLCD
;
; Sends a control code in WREG to the LCD via the I2C bus.
;
; On entry:
;
; WREG = control code to be sent.
;
; On exit:
;
; The flags2.LCD_REG_SEL bit is cleared which selects the LCD's instruction register.
;    
        
sendControlCodeToLCD:
    
    banksel flags2
    bcf     flags2, LCD_REG_SEL         ; select the LCD's instruction register    
    
    movlp   high sendNybblesToLCDViaI2C
    call    sendNybblesToLCDViaI2C
    movlp   sendControlCodeToLCD
    
    return
    
; end of sendControlCodeToLCD
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; driveForward
;
; Drives rover forward. Sets one drive control line high and one low to apply differential voltage
; to the drive motor.
;    
    
driveForward:

    
    banksel MOTORS_L
    
    bsf	    MOTORS_L,DRIVE_MOTOR_A
    bcf	    MOTORS_L,DRIVE_MOTOR_B	

    return
    
; end of driveForward
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; driveReverse
;
; Drives rover in reverse. Sets one drive control line high and one low to apply differential
; voltage to the drive motor.
;    
    
driveReverse:

    banksel MOTORS_L
    
    bcf	    MOTORS_L,DRIVE_MOTOR_A
    bsf	    MOTORS_L,DRIVE_MOTOR_B	

    return
    
; end of driveReverse
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; stopDrive
;
; Stops rover drive motor.
;    
    
stopDrive:

    
    banksel MOTORS_L
    
    bcf	    MOTORS_L,DRIVE_MOTOR_A	; set both lines low to turn motor off
    bcf	    MOTORS_L,DRIVE_MOTOR_B

    return
    
; end of stopDrive
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; steerLeft
;
; Turns front wheels to the left. Sets one drive control line high and one low to apply
; differential voltage to the drive motor.
;    
    
steerLeft:

    
    banksel MOTORS_L
    
    bsf	    MOTORS_L,STEER_MOTOR_A
    bcf	    MOTORS_L,STEER_MOTOR_B	

    return
    
; end of steerLeft
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; steerRight
;
; Turns front wheels to the right. Sets one drive control line high and one low to apply
; differential voltage to the drive motor.
;    
    
steerRight:

    
    banksel MOTORS_L
    
    bcf	    MOTORS_L,STEER_MOTOR_A
    bsf	    MOTORS_L,STEER_MOTOR_B	

    return
    
; end of steerRight
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; steerCenter
;
; Removes voltage from the steering direction motor so the return spring can return the front
; wheels to the straight ahead direction.
;    
    
steerCenter:

    banksel MOTORS_L
    
    bcf	    MOTORS_L,STEER_MOTOR_A	; set both lines low to turn motor off
    bcf	    MOTORS_L,STEER_MOTOR_B	

    return
    
; end of steerCenter
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; alternateFlashLEDs
;
; Flashes the LEDs back and forth.
;    
    
alternateFlashLEDs:
    
    banksel scratch0
    movwf   scratch0
        
aFLLoop:    
        
    banksel LEDS_L
    bcf     LEDS_L,LEFT_LED             ; left LED on
    bsf     LEDS_L,RIGHT_LED            ; right LED off
    
    movlw   .255
    call    delayWms
    
    banksel LEDS_L
    bsf     LEDS_L,LEFT_LED             ; left LED off
    bcf     LEDS_L,RIGHT_LED            ; right LED on

    movlw   .255
    call    delayWms

    banksel scratch0
    decfsz  scratch0,F
    goto    aFLLoop
    
    return
    
; end of start
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; trapSwitchInputs
;
; Checks each switch input and sets the associated flag for each if it is active.
;
; All other code uses the flags to determine the state of the switches. This allows the switch
; states to be easily read from an alternative source, such as the User Interface board which
; transmits switch states via serial line.
;
; This function also allows streamlining of debounce code.
;

trapSwitchInputs:

    banksel PORTC

    btfss   PORTC,0
    bcf     switchStates,0

    btfss   PORTC,0
    bcf     switchStates,0

; Select switch input is ignored here...for boards connected to a User Interface board, R84 is
; installed which allows the Cutting Current Power Supply's AC OK output to be read via this input.
; The signal is no longer valid as the Select switch. That switch is now connected to the User
; Interface board which will report its state via serial transmission.
;
;    btfss   MODE_JOGUP_SEL_EPWR_P,SELECT_SW
;    bcf     switchStates,SELECT_SW_FLAG

    btfss   PORTC,0
    bcf     switchStates,0

    banksel PORTA

    btfss   PORTA,0
    bcf     switchStates,0

    return

; end of trapSwitchInputs
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; flipSign
;
; Flips the sign (positive to negative or vice verse) of the word in scratch1:scratch0
;
; On entry:
;
; var scratch1:scratch0 = word to be flipped
;
; Returns flipped word in scratch1:scratch0
;

flipSign:

    comf    scratch0,F
    comf    scratch1,F
    incf    scratch0,F				; see note "incf vs decf rollover"
    btfsc   STATUS,Z		
    incf    scratch1,F
    return

; end of flipSign
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; processIO
;
; Handles inputs and outputs.
;
; Sends the output states to the remote devices if the serial transmit buffer is ready.
;
; Scans the switch inputs, returning their state and providing debouncing delay.
;
; Each switch input is turned back off by this function on the following call so the calling
; functions only see a single pulse for each switch closure. If the user holds the button down long
; enough, it will begin to repeat at a rate controlled by the debounce timer.
;
; Call processIOQ to skip sending of Output States.
;
; On entry: no values required
;
; On exit:
;
; switch states (local & remote combined) in switchStates, previous state in switchStatesPrev
; last reported switch states from remote device in switchStatesRemote
;

processIO:

processIOQ:    
    
    banksel switchStates
    movlw   0x0e                    ; only allow one pulse per switch closure
    iorwf   switchStates,F
    iorwf   switchStatesRemote,F                                                                        
                                    
    movlp   high handleReceivedDataIfPresent
    call    handleReceivedDataIfPresent    
    movlp   high processIO

    btfsc   flags3,DEBOUNCE_ACTIVE  ; bail out if debounce timer is active
    return
        
    movf    switchStates,W          ; store the previous state of the buttons
    movwf   switchStatesPrev

    movlw   0xff                    ; preset states -- flags will be set low for active switches
    movwf   switchStates

    call    trapSwitchInputs        ; check all switches and store their states
    
    movf    switchStatesRemote,W    ; combine the states from the local and remote switches
    andwf   switchStates,F          ;  a zero from either will result in zero (switch active)

    ; if the switch states changed, start debounce timer -- switches will not be checked again
    ; until the timer controlled by an interrupt routine is finished
    
    movf    switchStates,W                  ; check if new state same as old state
    subwf   switchStatesPrev,W
    btfsc   STATUS,Z
    return

    movlw   0x21                          ; set and start the debounce timer
    movwf   debounceH    
    movlw   0x75                            
    movwf   debounceL
    
    bsf     flags3,DEBOUNCE_ACTIVE

    return

; end of processIO
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitUntilIORefresh
;
; Waits until the current debounce timeout is finished and the next IO packet is received and
; applied to the switchStates.
;
; Use this at the beginning of functions to make sure the Select button and all other inputs are
; released so they won't trigger anything until the user presses again.
;
; This is especially useful if a delay is used at the beginning of the function. This delay will
; generally cause the next call to processIO to use old data from the remote as only the first
; packet received during the delay will be processed...all subsequent packets will be tossed.
;
   
waitUntilIORefresh:
    
wUIRLoop:        
    
    call    processIO                   ; allow handling of IO packets so the latest is applied
        
    btfsc   flags3,DEBOUNCE_ACTIVE      ; loop until debounce finished
    goto    wUIRLoop
    
    return

; end of waitUntilIORefresh
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; setupLCDBlockPkt
;
; Prepares the serial transmit buffer with header, length byte of 3, and command LCD_BLOCK_CMD.
; The data to be sent can then be added to the packet.
;
; Before the packet is transmitted, the length byte should be replaced with the actual number of
; bytes which have been added to the packet.
;
; On Entry:
;
; On Exit:
;
; packet is stuffed with header, length value of 3, and command byte
; FSR0 and serialXmtBufPtrH:serialXmtBufPtrL will point to the location for the next data byte
;
    
setupLCDBlockPkt:

    movlw   LCD_BLOCK_CMD               ; prepare to write a block of data to LCD
    
    movlp   high setupSerialXmtPkt
    call    setupSerialXmtPkt
    movlp   high setupLCDBlockPkt

    banksel flags

    return

; end of setupLCDBlockPkt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; printStringWaitPrep
;
; Prints the string pointed to by FSR1.  The string is placed after any data already in the print
; buffer.
;
; All data in the buffer is then begun to be transmitted to the LCD.
;
; Waits for transmission to be completed.
;
; Sets up the transmission buffer in preparation for the next printString (LCD_BLOCK_CMD
;  packet).
;
; On entry:
;
; FSR1 points to the desired string
;
; After placing the string characters in the LCD print buffer, the buffer is flushed to force
; transmission of the buffer.  Any characters placed in the buffer before the string will also
; be printed, so place control codes in the buffer first and then call this function to print
; everything.
;
; wip mks -- printString is a buffered method used for serial port connection -- needs updated to
;           work with the I2C
;

printStringWaitPrep:

    call    printString
    call    waitSerialXmtComplete   ; wait until buffer printed
    
    call    setupLCDBlockPkt

    return

; end of printStringWaitPrep
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flushXmtWaitPrep
;
; Starts transmission of the serial transmit buffer and then waits until the entire buffer has been
; sent and is ready for more data.
;
; Setups up the serial transmission buffer in preparation for the next printString (LCD_BLOCK_CMD
;  packet).
;

flushXmtWaitPrep:
    
    movlp   high startSerialPortTransmit
    call    startSerialPortTransmit

    movlp   high waitSerialXmtComplete
    call    waitSerialXmtComplete

    call    setupLCDBlockPkt

    return

; end of flushXmtWaitPrep
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitXmtPrep
;
; Waits until the entire serial transmit buffer has been sent and is ready for more data.
;
; Setups up the serial transmission buffer in preparation for the next printString (LCD_BLOCK_CMD
;  packet).
;

waitXmtPrep:

    call    waitSerialXmtComplete

    call    setupLCDBlockPkt

    return

; end of waitXmtPrep
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doMainMenu
;
; Displays and handles the main menu page 1.  If the user moves cursor down while on bottom option
; page 2 is displayed.
;
; If doMainMenu is called, the menu is displayed with option 1 highlighted.
; If doMainMenuA is called, the cursorPos and menuOption should be preloaded by the calling
;  function to set the default highlighted option.
;
; It then processes user input.
;
; NOTE: LCD addressing is screwy - second line first column is 0xC0, third line is 0x94,
;       fourth line is 0xd4.
;

doMainMenu:

	; call here to default to option 1

    movlw   LINE2_COL1      ; set display position    
    movwf   cursorPos       ; option 1 highlighted
    movlw   0x1
    movwf   menuOption      ; option 1 currently selected

doMainMenuA:				; call here if default option has already been set by caller

;print the strings of the menu

    call    setupLCDBlockPkt    ; prepare block data packet for LCD

    call    clearScreen     ; clear the LCD screen (next print will flush this to LCD)
    
    movlw   high string4   ; "OPT EDM Notch Cutter"
    movwf   FSR1H
    movlw   low string4
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE2_COL1      ; set display position
    call    writeControl

    movlw   high string5  ; "1 - ????"
    movwf   FSR1H
    movlw   low string5
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

;position the cursor on the default selection

    movf    cursorPos,W		; load the cursor position to highlight the current choice
    call    writeControl
	call	turnOnBlink
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

loopDMM1:

	bcf		menuOption,7		; clear the menu page change flags
	bcf		menuOption,6

; scan for button inputs, highlight selected option, will return when Reset/Enter/Zero pressed

    movlw   .02                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    movwf   scratch0

    decfsz  scratch0,F
    goto    skipDMM1

    ; handle option 1 - DO SOMETHING HERE
    
    goto    doMainMenu      ; repeat main menu

    return 
   
skipDMM1:

    decfsz  scratch0,F
    goto    skipDMM3

    ; handle option 2 - DO SOMETHING HERE


    goto    doMainMenu      ; repeat main menu
    
skipDMM3:

	btfsc	menuOption,7
	goto	loopDMM1			; no previous menu page, ignore

	btfsc	menuOption,6
	goto    doMainMenuPage2     ; display next menu page

	; this part should never be reached unless there is a programming error
	; just redo the menu in this case

	goto	doMainMenu
    
; end of doMainMenu
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doMainMenuPage2
;
; Displays and handles the main menu page 2.  If the user moves cursor up while on top option
; page 1 is displayed.
;
; If doMainMenuPage2 is called, the menu is displayed with option 1 highlighted.
; If doMainMenuPage2A is called, the cursorPos and menuOption should be preloaded by the calling
;  function to set the default highlighted option.
;
; It then processes user input.
;
; NOTE: LCD addressing is screwy - second line first column is 0xC0, third line is 0x94,
;       fourth line is 0xd4.
;

doMainMenuPage2:

	; call here to default to option 1

    movlw   LINE1_COL1
    movwf   cursorPos       ; option 1 highlighted
    movlw   0x1
    movwf   menuOption      ; option 1 currently selected

doMainMenuPage2A:			; call here if selected option has already been set by caller

;print the strings of the menu

    call    setupLCDBlockPkt    ; prepare block data packet for LCD

    call    clearScreen     ; clear the LCD screen (next print will flush this to LCD)

; display the first option

    movlw   high string19   ; "Cycle Test"
    movwf   FSR1H
    movlw   low string19
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

; display the second option

    movlw   LINE2_COL1      ; set display position
    call    writeControl

    movlw   high string20   ; "5 - Motor Dir "
    movwf   FSR1H
    movlw   low string20
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done
    
;position the cursor on the default selection

    movf    cursorPos,W		; load the cursor position to highlight the current choice
    call    writeControl
	call	turnOnBlink
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

loopDMMP21:

	bcf		menuOption,7		; clear the menu page change flags
	bcf		menuOption,6

; scan for button inputs, highlight selected option, will return when Reset/Enter/Zero pressed
; if handleMenuInputs returns with bit 7 or 6 of menuOptions set, the parsing will fall all the
; way to the bottom where those bits are checked and handled. The bits could be checked at the
; beginning instead?

    movlw   .02                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    movwf   scratch0

    decfsz  scratch0,F
    goto    skipDMMP27

    ; handle option 1 - DO SOMETHING HERE
    
    goto    doMainMenuPage2 ; refresh menu
 
skipDMMP27:

    decfsz  scratch0,F
    goto    skipDMMP28

    ; handle option 2 - DO SOMETHING HERE

    goto    doMainMenuPage2         ; refresh menu

skipDMMP28:

	btfss	menuOption,7
	goto	skipDMMP29

	;go back to previous menu with last option defaulted	
    movlw   LINE2_COL1      ; set display position
    movwf   cursorPos
    movlw   0x3
    movwf   menuOption      ; last option currently selected
	goto	doMainMenuA		; display previous menu page

skipDMMP29:

	btfsc	menuOption,6
	goto    loopDMMP21			; no next menu page, ignore

	; this part should never be reached unless there is a programming error
	; just redo the menu in this case

	goto	doMainMenuPage2
    
; end of doMainMenuPage2
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; pushFSR1
;
; Saves FSR1 to temporary variables.
;
  
pushFSR1:    

    movf    FSR1H,W
    movwf   FSR1H_TEMP
    movf    FSR1L,W
    movwf   FSR1L_TEMP

    return
    
; pushFSR1
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; popFSR1
;
; Loads FSR1 from temporary variables.
;
  
popFSR1:

    movf    FSR1H_TEMP,W
    movwf   FSR1H
    movf    FSR1L_TEMP,W    
    movwf   FSR1L

    return
    
; popFSR1
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearScreen
;
; Clears the LCD screen.
;

clearScreen:

; reset LCD modes to known and clear the screen

    call    turnOffBlink		; turn off character blinking

    movlw   CLEAR_SCREEN_CMD	; send Clear Display control code to the LCD
    call    writeControl    
    
    movlw   LINE1_COL1          ; set display position
    call    writeControl    	

    return                  	; exit menu on error

; end of clearScreen
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleMenuInputs
;
; Handles menu inputs, moving the cursor to highlight the selected option. Returns when
; Reset/Enter/Zero button pressed.
;
; If the user attempts to move above the first option, bit 7 of menuOption is set,
; if the user attempts to move below the last option, bit 6 of menuOption is set.
; The function exits in either of those cases to allow the calling function to change the menu
; page as appropriate.
;
; On entry:
;
; menuOption contains the number of the currently selected option.
; W should contain the maximum menu option number.
;

handleMenuInputs:

    movwf   scratch4        ; store the maximum menu item number   

loopHMMI:

	; check to see if bit 7 or 6 of menuOption has been set by selectHigherOption or
    ; selectLowerOption calls in the loop below
	; this flags that the menu page needs to be changed so this function exits so the calling
    ; function can perform that task

	btfsc	menuOption,7
	return
	btfsc	menuOption,6
	return

    call    processIO        ; watch for user input

    btfsc   switchStates,JOG_UP_SW_FLAG
    goto    skip_upHMMI     ; skip if Up switch not pressed

    call    selectHigherOption  ; adjusts menu_option to reflect new selection
    
    goto    loopHMMI

skip_upHMMI:

    btfsc   switchStates,JOG_DOWN_SW_FLAG
    goto    skip_dwnHMMI    ; skip if Down switch not pressed

    movf    scratch4,W          ; maximum number of options
    call    selectLowerOption   ; adjusts menu_option to reflect new selection

    goto    loopHMMI

skip_dwnHMMI:

    btfsc   switchStates,SELECT_SW_FLAG
    goto    loopHMMI        ; loop if Reset/Select switch not pressed

    return                  ; return when Reset/Enter/Zero button pressed

; end of handleMenuInputs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; selectHigherOption
;
; Decrements menu_option if it is not already 1, returns selected option in menuOption.
;
; If the user attempts to move above option 1, bit 7 of menuOption is set upon return.
; This flag bit can be used to trigger a menu page change if there are more than one.
; The lower 6 bits are able to handle 63 menu options - more than enough.
;
; On entry:
;
; menuOption contains the number of the currently selected option.
;
; On return, menuOption contains the newly selected option.
;

selectHigherOption:

    decfsz  menuOption,F    ; move to option listed higher on the screen (lower number)
    goto    moveCursorSHO

; reached end of options

    movlw   .1
    movwf   menuOption      ; don't allow option less than 1

	bsf     menuOption,7	; set bit 7 to show that user attempted to move curser
						    ; above first option - can be used to change menu pages

    return

; move cursor
; Note that if the first option is not on line one or the last option not on the last line,
; this section won't be reached because the code above prevents moving beyond option 1 or
; the max option - thus constraining the cursor to the proper lines.
; Note that the numbering for the lines is screwy - 0x00, 0x40

moveCursorSHO:

    movlw   LINE2_COL1
    subwf   cursorPos,W         ; is cursor at last line?
    btfss   STATUS,Z    
    goto    line2SHO

    movlw   LINE1_COL1          ; move cursor up one line
    movwf   cursorPos
    call    writeControl        ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    return

line2SHO:                       ; don't move cursor if at the top

    return

; end of selectHigherOption
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; selectLowerOption
;
; Increments menu_option if it is not already at maximum specifed in W, returns selected option in
; menuOption.
;
; If the user attempts to move below the max option, bit 6 of menuOption is set upon return.
; This flag bit can be used to trigger a menu page change if there are more than one.
; The lower 6 bits are able to handle 63 menu options - more than enough.
;
; On entry:
;
; menu_option contains the number of the currently selected option
; W contains the maximum allowable option number.
;
; On return, menuOption contains the newly selected option.
;

selectLowerOption:

    movwf   scratch0        ; store the maximum

    addlw   .1              ; needs to be max option + 1 for the equal 

    incf    menuOption,F    ; move to option listed lower on the screen (higher number)
    
    subwf   menuOption,W
    btfss   STATUS,Z        ; jump if menuOption = W
    goto    moveCursorSLO

    movf    scratch0,W		; already at max option, so exit
    movwf   menuOption     	; set menuOption to the maximum option allowed

	bsf     menuOption,6	; set bit 6 to show that user attempted to move curser
						    ; above first option - can be used to change menu pages

    return

; move cursor
; Note that if the first option is not on line one or the last option not on the last line,
; this section won't be reached because the code above prevents moving beyond option 1 or
; the max option - thus constraining the cursor to the proper lines.
; Note that the numbering for the lines is screwy - 0x00, 0x40

moveCursorSLO:

    movlw   LINE1_COL1
    subwf   cursorPos,W     ; is cursor at top line?
    btfss   STATUS,Z    
    goto    line2SLO

    movlw   LINE2_COL1      ; set display position
    movwf   cursorPos
    call    writeControl        ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    return

line2SLO:                   ; don't move cursor if at the bottom

    return

; end of selectLowerOption
;--------------------------------------------------------------------------------------------------
   
;--------------------------------------------------------------------------------------------------
; SetBank0ClrWDT        
;
; Set Bank 0, Clear high byte of FSR pointers, Clear WatchDog timer
; 

SetBank0ClrWDT:

    banksel flags

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
;  C bit will be set until LSB goes from 0 to 255; 0 is only value added to 0xff that won't carry
;  When C bit not set, subtract 1 from MSB until it reaches 0
;
;

bigDelay:
    clrf    scratch1
bigDelayA:
    movwf   scratch0        ; store W

    ifdef DEBUG_MODE        ; if debugging, don't delay
    return
    endif

loopBD1:

	; call inner delay for each count of outer delay

    movlw   0x1
    movwf   scratch3
    movlw   0x6a            ; scratch3:W = delay value
    call    smallDelayA

    movlw   0xff            ; decrement LSByte by adding -1
    addwf   scratch0,F
    btfss   STATUS,C		; did LSByte roll under (0->255)?
	decf	scratch1,F		; decrement MSByte after LSByte roll under
	movf	scratch0,W		; check MSB:LSB for zero
	iorwf	scratch1,W
	btfsc	STATUS,Z
	goto    SetBank0ClrWDT  ; counter = 0, reset stuff and return

    goto    loopBD1         ; loop until outer counter is zero

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

smallDelay:
    clrf    scratch3
smallDelayA:
    movwf   scratch2        ; store W

    ifdef DEBUG_MODE        ; if debugging, don't delay
    return
    endif

loopSD1:

    clrwdt                  ; keep watch dog timer from triggering

    movlw   0xff            ; decrement LSByte by adding -1
    addwf   scratch2,F
    btfss   STATUS,C		; did LSByte roll under (0->255)?
	decf	scratch3,F		; decrement MSByte after LSByte roll under
	movf	scratch2,W		; check MSB:LSB for zero
	iorwf	scratch3,W
	btfsc	STATUS,Z
	return

    goto    loopSD1         ; loop until outer counter is zero

; end of smallDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; delayWms
;
; Creates a delay of x milliseconds where x is specified in the W register.
;
; On Entry:
;
; W contains number of milliseconds to delay.
;
; Values to achieve 1 millisecond for various Fosc:
; 
; for 4Mhz  Fosc -> bigDelayCnt = .2, smallDelayCnt = .166
; for 16Mhz Fosc -> bigDelayCnt = .6, smallDelayCnt = .222
;
; Note: these values do not take into account interrupts processing which will increase the delay.
;

delayWms:

    banksel msDelayCnt

    ifdef DEBUG_MODE            ; if debugging, don't delay
    return
    endif

    movwf   msDelayCnt          ; number of milliseconds

msD1Loop1:

	movlw	.6                  ; smallDelayCnt * bigDelayCnt give delay of 1 millisecond
	movwf	bigDelayCnt

msD1Loop2:

	movlw	.222                
	movwf	smallDelayCnt

msD1Loop3:

	decfsz	smallDelayCnt,F
    goto    msD1Loop3

	decfsz	bigDelayCnt,F
    goto    msD1Loop2

	decfsz	msDelayCnt,F
    goto    msD1Loop1

	return

; end of delayWms
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; delayWSeconds
;
; Creates a delay of x seconds where x is specified in the W register. Uses delayWms repeatedly
; to create proper delay.
;
; On Entry:
;
; W contains number of seconds to delay.
;
;

delayWSeconds:

    banksel secDelayCnt

    movwf   secDelayCnt          ; number of seconds

sDLoop1:    
    
    movlw   .250
    call    delayWms

    movlw   .250
    call    delayWms

    movlw   .250
    call    delayWms
    
    movlw   .250
    call    delayWms

    decfsz  secDelayCnt,F
    goto    sDLoop1
    
    return
    
; end of delayWSeconds
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; printString
;
; Prints the string pointed to by FSR1.  The string is placed after any data already in the print
; buffer, all data in the buffer is then begun to be transmitted to the LCD.
;
; Does not wait for the data to be printed - call waitLCD after calling this function to wait
; until string is printed.
;
; On entry:
;
; FSR1 points to the desired string
;
; After placing the string characters in the LCD print buffer, the buffer is flushed to force
; transmission of the buffer.  Any characters placed in the buffer before the string will also
; be printed, so place control codes in the buffer first and then call this function to print
; everything.
;

printString:

loopPS:
    
    moviw   FSR1++

    btfss   STATUS,Z
    goto    pS1

    movlp   high startSerialPortTransmit
    call    startSerialPortTransmit     ; force buffer to print
    movlp   high printString
    banksel flags
    return

pS1:

    call    writeChar       ; write the character to the LCD print buffer
    
    goto    loopPS          ; loop until length of string reached 

; end of printString
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; printStringI2CUnbuffered
;
; Prints the string pointed to by FSR1 to the I2C port.
;
; On entry:
;
; FSR1 points to the desired string
;

printStringI2CUnbuffered:

    banksel flags2
    bsf     flags2, LCD_REG_SEL         ; select the LCD's data register
    
loopPSI:
    
    moviw   FSR1++

    btfss   STATUS,Z
    goto    pSI1

    return

pSI1:

    call    longCallSendNybblesToLCDViaI2C
    
    goto    loopPSI          ; loop until length of string reached 

; end of printStringI2CUnbuffered
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; waitSerialXmtComplete
;
; Waits until the serial transmit buffer has been sent and is ready for more data.
;

waitSerialXmtComplete:

    banksel serialXmtBufNumBytes

loopWBL1:                   ; loop until interrupt routine finished writing character

    movf    serialXmtBufNumBytes,W
    btfss   STATUS,Z
    goto    loopWBL1

    banksel flags

    return

; end of waitSerialXmtComplete
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flushAndWaitSerialXmt
;
; Starts transmission of the serial transmit buffer and then waits until the entire buffer has been
; sent and is ready for more data.
;

flushAndWaitSerialXmt:

    call    startSerialPortTransmit ; start serial buffer transmission
    call    waitSerialXmtComplete

    return

; end of flushAndWaitSerialXmt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; turnOffBlink
;
; Turns off blink function in the LCD. The cursor is also turned off.
;
; The command stored in the LCD print buffer after any data already in the buffer.
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

turnOffBlink:

    movlw   DISPLAY_ONOFF_CMD_FLAG | DISPLAY_ON_FLAG ; display on, cursor off, blink off

    call    writeControl

    return

; end of turnOffBlink
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; turnOnBlink
;
; Turns on blink function in the LCD - the current at the address in the address register blinks.
; The cursor is also turned off.
;
; The command is stored in the LCD print buffer after any data already in the buffer.
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

turnOnBlink:

    movlw   DISPLAY_ONOFF_CMD_FLAG | DISPLAY_ON_FLAG | BLINK_ON_FLAG ; display on, cursor off, blink on

    call    writeControl

    return

; end of turnOnBlink
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeChar
;
; Writes an ASCII character to the serial transmit buffer : writes 0x00 followed by the byte in W.
; This is stored after any data already in the buffer.
;
; On entry:
;
; W contains byte to write
;
; NOTE: The data is placed in the buffer but is not submitted to be sent.  After using this
; function, call startSerialPortTransmit or printString to initiate transmission.
;

writeChar:

    movwf    scratch0       ; store character

    clrf    scratch1        ; scratch1 = 0

    call    writeWordToSerialXmtBuf     ; write 0 followed by scratch0 to LCD
    
    return

; end of writeChar
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeControl
;
; Writes an LCD control code to the serial transmit buffer : writes 0x01 followed by the byte in W.
; This is stored after any data already in the buffer.
;
; On entry:
;
; W contains byte to write
;
; NOTE: The data is placed in the buffer but is not submitted to be sent.  After using this
; function, call startSerialPortTransmit or printString to initiate transmission.
;

writeControl:

    movwf    scratch0       ; store character

    movlw   0x1
    movwf   scratch1        ; scratch1 = 1
    
    call    writeWordToSerialXmtBuf         ; write 1 followed by scratch0 to transmit buffer

    return

; end of writeControl
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeWordToSerialXmtBuf
; 
; This subroutine writes the word in scratch1:0 to the serial transmit buffer.
;
; On entry:
; 
; scratch1 contains first byte to write
; scratch0 contains second byte to write
;
; NOTE: The data is placed in the buffer but is not submitted to be sent.  After using this
; function, call startSerialPortTransmit or printString to initiate transmission.
;

writeWordToSerialXmtBuf:
    
    movf    scratch1,W                  ; get first byte to write
    call    writeByteToSerialXmtBuf

    movf    scratch0,W                  ; get second byte to write
    call    writeByteToSerialXmtBuf
    
    return

; end of writeWordToSerialXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeByteToSerialXmtBuf
;
; This subroutine writes the byte in W to the serial transmit buffer.
;
; On entry:
; 
; W contains byte to write
;
; NOTE: The data is placed in the buffer but is not submitted to be sent.  After using this
; function, call startSerialPortTransmit or printString to initiate transmission.
;

writeByteToSerialXmtBuf:

    banksel scratch2
    movwf   scratch2                        ; store character

    banksel serialXmtBufPtrH                ; load FSR0 with buffer pointer
    movf    serialXmtBufPtrH, W
    movwf   FSR0H
    movf    serialXmtBufPtrL, W
    movwf   FSR0L

    movf    scratch2,W                      ; retrieve character

    movwf   INDF0                           ; store character in buffer

    banksel serialXmtBufNumBytes            ; increment packet byte count
    incf    serialXmtBufNumBytes,f

    banksel serialXmtBufPtrH
    incf    serialXmtBufPtrL,F              ; point to next buffer position
    btfsc   STATUS,Z
    incf    serialXmtBufPtrH,F
    
    banksel flags

    return    
 
; end of writeByteToSerialXmtBuf
;--------------------------------------------------------------------------------------------------
 
;--------------------------------------------------------------------------------------------------
; generateI2CStart
;
; Generates a start condition on the I2C bus.
;

generateI2CStart:

    banksel SSP1CON2
    bsf     SSP1CON2,SEN

    return

; end of generateI2CStart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CRestart
;
; Generates a restart condition on the I2C bus.
;

generateI2CRestart:

    banksel SSP1CON2
    bsf     SSP1CON2,RSEN

    return

; end of generateI2CRestart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CStop
;
; Generates a stop condition on the I2C bus.
;

generateI2CStop:

    banksel SSP1CON2
    bsf     SSP1CON2,PEN

    return

; end of generateI2CStop
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSP1IF
;
; Sets the SSP1IF bit in register PIR1 to 0.
;

clearSSP1IF:

    banksel PIR1
    bcf     PIR1, SSP1IF

    return

; end of clearSSP1IF
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHigh
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high.
;

waitForSSP1IFHigh:

    ifdef DEBUG_MODE  ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    return            ; simulated by the IDE
    endif

    banksel PIR1

wfsh1:
    btfss   PIR1, SSP1IF
    goto    wfsh1

    return

; end of waitForSSP1IFHigh
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHighThenClearIt
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high and then clears that bit.
;
; The bit must be cleared at some point after it is set before performing most actions with the I2C
; but. In most cases, it can be cleared immediately for which this function is useful.
;
; If the bit is not cleared before an operation, then checking the bit immediately after the
; operation will make it appear that the operation completed immediately and the code will not
; wait until the MSSP module sets the bit after actual completion.
;

waitForSSP1IFHighThenClearIt:

    call    waitForSSP1IFHigh

    call    clearSSP1IF

    return

; end of waitForSSP1IFHighThenClearIt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendI2CByte
;
; Waits until the SSP1IF bit in register PIR1 goes high and then transmits the byte in the W
; register on the I2C bus.
;

sendI2CByte:

    ; wait for SSP1IF to go high

    call    waitForSSP1IFHigh

    ; put byte in transmit buffer

    banksel SSP1BUF
    movwf   SSP1BUF

    ; clear interrupt flag

    call    clearSSP1IF

    return

; end of sendI2CByte
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSP1OV
;
; Clears the MSSP overflow bit to allow new bytes to be read.
;
; The bit is set if a byte was received before the previous byte was read from the buffer.
;

clearSSP1OV:

    banksel SSP1CON1
    bcf     SSP1CON1,SSPOV

    return

; end of clearSSP1OV
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearWCOL
;
; Clears the MSSP write collision bit to allow new bytes to be written
;
; The bit is set if a byte was placed in SSPBUF at an improper time.
;

clearWCOL:

    banksel SSP1CON1
    bcf     SSP1CON1, WCOL

    return

; end of clearWCOL
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; reallyBigDelay
;
; Delays for a second or two.
;

reallyBigDelay:

    ifdef DEBUG_MODE       ; if debugging, don't delay
    return
    endif

    banksel scratch6

    movlw   .50
    movwf   scratch6

rbd1:

    movlw   .255
    movwf   scratch7

rbd2:

    movlw   .255
    movwf   scratch8

rbd3:

    decfsz  scratch8,F
    goto    rbd3

    decfsz  scratch7,F
    goto    rbd2

    decfsz  scratch6,F
    goto    rbd1

    return

; end of reallyBigDelay
;--------------------------------------------------------------------------------------------------
    
    org 0x800	;start code on page 2 of 2047 byte boundary
    
;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;
; NOTE: The system does not use the internal comparators even though it appears there are analog
; inputs to some of the comparator pins.  Those inputs are the output of external op-amps
; acting as comparators - their outputs are read as digital inputs on RA3 and RA4
;

setup:

    clrf    INTCON          ; disable all interrupts

    call    setupClock      ; set system clock source and frequency

    call    setupPortA      ; prepare Port A for I/O

    call    setupPortB      ; prepare Port B for I/O

    call    setupPortC      ; prepare Port C  for I/O

;DEBUG NPS    call    setupSerialPort ; prepare serial port for sending and receiving

    call    initializeOutputs

    call    setupI2CMaster7BitMode ; prepare the I2C serial bus for use

    call    initLCD
    
;start of hardware configuration

    banksel OPTION_REG
    movlw   0x58
    movwf   OPTION_REG      ; Option Register = 0x58   0101 1000 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 0 : TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
                            ; bit 4 = 1 : TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
                            ; bit 3 = 1 : PSA ~ Prescaler disabled; Timer0 will be 1:1 with Fosc/4
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    000 = 1:2 scaling for Timer0 (if enabled)
                            ; bit 0 = 0 :
    
;end of hardware configuration

    banksel flags

    movlp   high reallyBigDelay     ; ready PCLATH for the calls below
    call    reallyBigDelay
    movlp   high setup              ; set PCLATH back to what it was

    clrf   flags3
    clrf    xmtDataTimer
    
    movlw   0xff
    movwf   switchStates
    movwf   switchStatesPrev
    movwf   switchStatesRemote
    
    ; reset some values to a default state
    
    call    resetSerialPortRcvBuf           ; re-init flags2 variable after loading from eeprom
    call    resetSerialPortXmtBuf
	
; enable the interrupts

	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 and serial port are peripherals)
    bsf     INTCON,T0IE     ; enable TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

    ; the resetLCD function is still coded for the parallel connected LCD -- won't work for I2C
    ;call    resetLCD        ; resets the LCD PIC and positions at line 1 column 1

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupClock
;
; Sets up the system clock source and frequency.
;
; Assumes clock related configuration bits are set as follows:
;
;   _FOSC_INTOSC,  _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_DISABLED
;
; Assumes all programmable clock related options are at Reset default values.
;
; NOTE: Adjust I2C baud rate generator value when Fosc is changed.
;

setupClock:

    ; choose internal clock frequency of 16 Mhz

    banksel OSCCON

    bsf     OSCCON, IRCF0
    bsf     OSCCON, IRCF1
    bsf     OSCCON, IRCF2
    bsf     OSCCON, IRCF3

    return

; end of setupClock
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortA
;
; Sets up Port A for I/O operation.
;
; NOTE: Writing to PORTA is same as writing to LATA for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RA0, RA1 and RA3 can only be inputs on the PIC16f1459 device. 
;       RA2, RA6, RA7 are not implemented.
;

setupPortA:

    banksel WPUA
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUA

    banksel PORTA
    clrf    PORTA                       ; init port value

    banksel LATA                        ; init port data latch
    clrf    LATA

    banksel ANSELA
    clrf    ANSELA                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISA
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISA

    ; set direction for each pin used

    bsf     TRISA, RED_SW               ; input    
    bsf     TRISA, BLK_SW               ; input
    
    bcf	    TRISA, LEFT_LED             ; output
    bcf	    TRISA, RIGHT_LED            ; output
    
    return

; end of setupPortA
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortB
;
; Sets up Port B for I/O operation.
;
; NOTE: Writing to PORTB is same as writing to LATB for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RB0, RB1, RB2, RB3 are not implemented on the PIC16f1459 device.
;

setupPortB:

    banksel WPUB
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUB

    banksel PORTB
    clrf    PORTB                       ; init port value

    banksel LATB                        ; init port data latch
    clrf    LATB

    banksel ANSELB
    clrf    ANSELB                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISB
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISB
    
    bsf	    TRISB, RB4                  ; set as input for use as I2C bus
    bsf	    TRISB, RB6                  ; set as input for use as I2C bus
    
    bcf	    TRISB, UNUSED_RB5           ; output
    bcf	    TRISB, UNUSED_RB7           ; output

    return

; end of setupPortB
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortC
;
; Sets up Port C for I/O operation.
;
; NOTE: Writing to PORTC is same as writing to LATC for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;

setupPortC:

    ; Port C does not have a weak pull-up register

    banksel PORTC
    clrf    PORTC                       ; init port value

    banksel LATC                        ; init port data latch
    clrf    LATC

    banksel ANSELC
    clrf    ANSELC                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISC
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISC

    bcf     TRISC, ICSPDAT              ; output
    bcf     TRISC, ICSPCLK  	        ; output
    bcf     TRISC, UNUSED_RC2	        ; output
    bcf     TRISC, UNUSED_RC3	        ; output
    bcf     TRISC, STEER_MOTOR_A        ; output
    bcf     TRISC, STEER_MOTOR_B        ; output
    bcf     TRISC, DRIVE_MOTOR_A        ; output
    bcf     TRISC, DRIVE_MOTOR_B        ; output
    
    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; initializeOutputs
;
; Initializes all outputs to known values.
;
; Write to port latches to avoid problems with read-modify-write modifying port bits due to
; line noise.
;

initializeOutputs:
    
    banksel LEDS_L
    bsf	    LEDS_L, LEFT_LED	    ; set cathode high to turn off LED
    bsf	    LEDS_L, RIGHT_LED		; set cathode high to turn off LED
    
    banksel UNUSED_B_L
    clrf    UNUSED_B_L              ; set all port B pins low
    
    banksel MOTORS_L
    bcf	    MOTORS_L,ICSPDAT		; PICKIT programming line
    bcf	    MOTORS_L,ICSPCLK		; PICKIT programming line
    bcf	    MOTORS_L,UNUSED_RC2		; set unused low
    bcf	    MOTORS_L,UNUSED_RC3		; set unused low
    bcf	    MOTORS_L,STEER_MOTOR_A	; set both lines low to turn motor off
    bcf	    MOTORS_L,STEER_MOTOR_B	
    bcf	    MOTORS_L,DRIVE_MOTOR_A	; set both lines low to turn motor off
    bcf	    MOTORS_L,DRIVE_MOTOR_B
    
    return

; end of initializeOutpus
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; setupI2CMaster7BitMode
;
; Sets the MASTER SYNCHRONOUS SERIAL PORT (MSSP) MODULE to the I2C Master mode using the 7 bit
; address mode.
;
; NOTE: RB4 and RB6 must have been configured elswhere as inputs for this mode.
;

setupI2CMaster7BitMode:

    movlw   0x27			; set baud rate at 100kHz for oscillator frequency of 16 Mhz
    banksel SSP1ADD
    movwf   SSP1ADD

    banksel SSP1CON1
    bcf	SSP1CON1,SSP1M0		; SSPM = b1000 ~ I2C Master mode, clock = FOSC / (4 * (SSPADD+1))(4)
    bcf	SSP1CON1,SSP1M1
    bcf	SSP1CON1,SSP1M2
    bsf	SSP1CON1,SSP1M3

    bsf	SSP1CON1,SSPEN		;enables the MSSP module

    return

; end setupI2CMaster7BitMode
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; resetLCD
;
; Resets the LCD screen.
;

resetLCD:

    movlp   high setupLCDBlockPkt
    call    setupLCDBlockPkt    ; prepare block data packet for LCD

    movlp   high writeControl   ; readies the PCLATH for the calls below

    movlw   CLEAR_SCREEN_CMD
    call    writeControl        ; send Clear Display control code to the LCD

    movlw   LINE1_COL1          ; set display position
    call    writeControl

    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    movlp   high resetLCD       ; set PCLATH back to what it was on entry
    
    return
    
; end of resetLCD
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; initLCD
;
; Initialize the LCD display.
;
; See Dmcman_full-user manual.pdf from www.optrex.com for details.
;

initLCD:

    banksel flags2
    bcf     flags2, LCD_REG_SEL         ; select the LCD's instruction register
        
    bsf     flags2, LCD_BACKLIGHT_SEL   ; turn on the backlight
    
    movlw   0x00                    ; init the I2C expander chip
    call    sendNybbleToLCDViaI2C   ; RW = 0; BL = 0; EN = 0; RS = 0
    
    movlw   .200                    ; delay at least 15 ms after Vcc = 4.5V
    call    longCallDelayWms        ; delay plenty to allow power to stabilize
   
	movlw	0x03                    ; 1st send of Function Set Command: (8-Bit interface)
    call    sendNybbleToLCDViaI2C   ; (BF cannot be checked before this command.)
                                
    movlw   .6                      ; delay at least 4.1 mS
    call    longCallDelayWms

	movlw	0x03                    ; 2nd send of Function Set Command: (8-Bit interface)
    call    sendNybbleToLCDViaI2C   ; (BF cannot be checked before this command.)

    movlw   .6                      ; delay at least 100uS
    call    longCallDelayWms

	movlw	0x03                    ; 3rd send of Function Set Command: (8-Bit interface)
    call    sendNybbleToLCDViaI2C   ; (BF can be checked after this command)

    ; it's too complicated to check the LCD busy flag via the I2C expander bus, so care must be
    ; taken not to send data too quickly -- as the I2C bus is relatively slow, this shouldn't be
    ; a problem
    
    movlw   0x02                    ; set to 4-bit interface
    call    sendNybbleToLCDViaI2C
    
    ; from here on, two nybbles are sent for each transmission to send a complete byte
    
    movlw   (LCD_FUNCTIONSET | LCD_2LINE)  ; 2 line display, 5x8 font
    call    sendNybblesToLCDViaI2C

    movlw	(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    call    sendNybblesToLCDViaI2C
    
    movlw   LCD_CLEARDISPLAY        ; debug mks -- make this a function
    call    sendNybblesToLCDViaI2C   
    
    movlw   .3                      ; delay at least 1.53 ms after Clear Display command
    call    longCallDelayWms
    
	; set the entry mode -- cursor increments position (moves right) after each character
    ; display does not shift
    
	movlw   (LCD_ENTRYMODESET | LCD_INC | LCD_NO_SHIFT);
    call    sendNybblesToLCDViaI2C       

    movlw   LCD_RETURNHOME          ; set cursor at home position    debug mks -- make this a function
    call    sendNybblesToLCDViaI2C       

    movlw   .3                      ; delay at least 1.53 ms after Return Home command
    call    longCallDelayWms
        
	return

; end of initLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendNybblesToLCDViaI2C
;
; Sends a single byte to the LCD via the I2C bus one nybble at a time, upper nybble first.
;
; On entry:
;
; W should contain the nybbles to be written.
; Flags2.LCD_REG_SEL should be set 0 to access the instruction register or 1 for the data register
;
; On exit:
; 
; The proper address~R/W code will be stored in scratch0.    
; Variable scratch1 will be set to 2 to indicate a single byte.
; The nybbles to be written will be stored in scratch2:scratch3
; Indirect register FSR0 will point to scratch2.
;
    
sendNybblesToLCDViaI2C:
   
   banksel  I2CScratch0
      
   movwf    I2CScratch2             ; save for swapping and later retrieval of lower nybble
   swapf    I2CScratch2,W           ; swap upper to lower
   
   call     sendNybbleToLCDViaI2C   ; send upper nybble (now in lower, upper will be ignored)
   
   banksel  I2CScratch2
   movf     I2CScratch2,W              ; retrieve to send lower nybble
   
   goto     sendNybbleToLCDViaI2C   ; send lower nybble (upper will be ignored)
    
; end of sendNybblesToLCDViaI2C
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendNybbleToLCDViaI2C
;
; Sends a single nybble to the LCD via the I2C bus. The value is sent once to latch it into the
; outputs of the I2C expander chip and allow for setup time before strobing, then sent again with
; the LCD Enable line high, then again with the Enable line low to strobe the value into the LCD.
; The nybble being sent is in the upper half of WREG, the Enable line is controlled by a bit in the
; lower nybble.
;
; On entry:
;
; WREG lower nybble should contain value to be written. The upper nybble of WREG will be ignored.
; Flags2.LCD_REG_SEL should be set 0 to access the instruction register or 1 for the data register   
;
; On exit:
; 
; The proper address~R/W code will be stored in scratch0.    
; Variable scratch1 will be set to 1 to indicate a single byte.
; The byte to be written will be stored in scratch2.
; Indirect register FSR0 will point to scratch2.
;

sendNybbleToLCDViaI2C:
      
   banksel  I2CScratch0
   
   andlw    0x0f                ; upper nybble to zeroes (these end up being the control lines)

   movwf    I2CScratch4         ; store the value to be transmitted
 
   swapf    I2CScratch4,F       ; swap so value is in upper nybble and control lines are in lower
   
   btfsc    flags2,LCD_REG_SEL
   bsf      I2CScratch4,LCD_RS
   
   btfsc    flags2,LCD_BACKLIGHT_SEL
   bsf      I2CScratch4,LCD_BACKLIGHT
  
   movlw    high I2CScratch4    ; set pointer to location of nybble to be sent
   movwf    FSR0H
   movlw    low I2CScratch4
   movwf    FSR0L
      
   movlw    LCD_WRITE_ID        ; LCD's I2C address with R/W bit set low
   movwf    I2CScratch0
   
   movlw    .1
   movwf    I2CScratch1         ; set number of bytes to be transmitted    
   call     sendBytesViaI2C     ; send value to preset the data lines

   bsf      I2CScratch4, LCD_EN    ; set the Enable line high in the value
   
   movlw    .1                  ; send value again to set the Enable line high
   movwf    I2CScratch1    
   addfsr   FSR0,-.1
   call     sendBytesViaI2C
   
   bcf      I2CScratch4, LCD_EN    ; set the Enable line low in the value
   
   movlw    .1                  ; send again to set the Enable line low (strobes nybble into LCD)
   movwf    I2CScratch1    
   addfsr   FSR0,-.1   
   call     sendBytesViaI2C
    
   return
   
; end of sendNybbleToLCDViaI2C
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendBytesToLCDViaI2C
;
; Sends bytes to the LCD via the I2C bus.
;
; The number of bytes to be written should be in scratch1.
; Indirect register FSR0 should point to first byte in RAM to be written.
;
; The proper address~R/W code will be stored in scratch0.
;

sendBytesToLCDViaI2C:
   
   banksel  I2CScratch0
   movlw    LCD_WRITE_ID        ; LCD's I2C address with R/W bit set low
   movwf    I2CScratch0
    
   goto     sendBytesViaI2C
    
; end of sendBytesToLCDViaI2C
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendBytesViaI2C
;
; Sends byte to the LED PIC via the I2C bus.
;
; On entry:
;
; The I2C address of the destination device with R/W bit (bit 0) set low should be in scratch0
; The number of bytes to be written should be in scratch1.
; Indirect register FSR0 should point to first byte in RAM to be written.
;

sendBytesViaI2C:

    movlp   high clearSSP1IF        ; ready PCLATH for the calls below

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    banksel I2CScratch0
    movf    I2CScratch0,W           ; I2C device address and R/W bit
    call    sendI2CByte             ; send byte in W register on I2C bus after SSP1IF goes high

loopSBLP1:

    movlp   high sendI2CByte        ; ready PCLATH for the call to sendI2CByte
    moviw   FSR0++                  ; load next byte to be sent
    call    sendI2CByte
    
    movlp   high loopSBLP1          ; set PCLATH for the goto
    banksel I2CScratch1
	decfsz	I2CScratch1,F           ; count down number of bytes transferred
	goto	loopSBLP1               ; not zero yet - transfer more bytes

    movlp   high waitForSSP1IFHighThenClearIt   ; ready PCLATH for the calls below
    call    waitForSSP1IFHighThenClearIt        ; wait for high flag upon transmission completion
    call    generateI2CStop
    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished
    movlp   high sendBytesViaI2C            ; set PCLATH back to what it was on entry

    return

; end of sendBytesViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; longCallDelayWms
;
; Sets the PCLATH register to allow a call to delayWms which is in a different memory page. Sets
; the register back to the local page after the call returns.
;
        
longCallDelayWms:
    
    movlp   high delayWms
    call    delayWms
    movlp   longCallDelayWms
    
    return
    
; end of longCallDelayWms
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; setupSerialXmtPkt
;
; Prepares the serial transmit buffer with header, a length byte of 3 and the command byte. The
; data to be sent can then be added to the packet.
;
; The default length of 3 is useful for many LCD commands which require 2 data bytes and a
; checksum. The length can be adjusted before transmission.
;
; On Entry:
;
; W contains the packet command.
;
; On Exit:
;
; packet is stuffed with header, command, and length value of 3.
; FSR0 and serialXmtBufPtrH:serialXmtBufPtrL will point to the location for the next data byte
;

setupSerialXmtPkt:

    banksel usartScratch0

    movwf   usartScratch1       ; store the command byte
    
    movlw   .3
    movwf   usartScratch0       ; default number of data bytes plus checksum byte
                                ;       can be changed before transmission

    goto    setUpSerialXmtBuf

; end of setupSerialXmtPkt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------
;   EUSART Serial Port Core Functions
;
; Copy this block of code for all basic functions required for serial transmit. Build code, then
; copy all variables and defines which are shown to be missing.
;
;--------------------------------------------------------------------------------------------------
; setUpSerialXmtBuf
;
; Adds the header bytes, length byte, command byte, and various values from this Master PIC to the
; start of the serial port transmit buffer and sets serialXmtBufPtrH:L ready to add data bytes.
;
; Notes on packet length:
;
;   Example with 1 data bytes...
;
;   2 bytes (command byte + data byte)
;   ---
;   2 total (value passed to calcAndStoreCheckSumSerPrtXmtBuf; number bytes checksummed)
;
;   ADD (to determine length byte to insert into packet)
;
;   +1 checksum byte for the overall packet
;   3 total (value passed to setUpSerialXmtBuffer (this function) for packet length)
;
;   ADD (to determine actual number of bytes to send)
;
;   +2 header bytes
;   +1 length byte
;   ---
;   6 total (value passed to startSerialPortTransmit)
;
; On Entry:
;
; usartScratch0 should contain the number of data bytes plus one for the checksum byte in the packet
; usartScratch1 should contain the command byte
;
; On Exit:
;
; FSR0 and serialXmtBufPtrH:serialXmtBufPtrL will point to the location for the next data byte
; serialXmtBufNumBytes will be zeroed
;

setUpSerialXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    banksel usartScratch0

    movlw   0xaa
    movwi   FSR0++                          ; store first header byte

    movlw   0x55
    movwi   FSR0++                          ; store first header byte

    movf    usartScratch0,W                 ; store length byte
    movwi   FSR0++

    movf    usartScratch1,W                 ; store command byte
    movwi   FSR0++

    banksel serialXmtBufPtrH                ; point serialXmtBufPtrH:L at next buffer position
    movf    FSR0H,W
    movwf   serialXmtBufPtrH
    movf    FSR0L,W
    movwf   serialXmtBufPtrL

    clrf   serialXmtBufNumBytes             ; tracks number of bytes added -- must be adjusted
                                            ; later to include the header bytes, length, command

    return

; end of setUpSerialXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleReceivedDataIfPresent
;
; Processes data in the serial receive buffer if a packet has been received.
;

handleReceivedDataIfPresent:

    banksel flags2                          ; handle packet in serial receive buffer if ready
    btfsc   flags2, SERIAL_PACKET_READY
    goto    handleSerialPacket

    return

; end of handleReceivedDataIfPresent
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPacket
;
; Processes a packet in the serial receive buffer.
;

handleSerialPacket:

    banksel serialRcvPktLen

    movf    serialRcvPktLen,W           ; store the packet length variable so the receive interrupt
    movwf   serialRcvPktLenMain         ; can overwrite it if a new packet arrives
        
    call    resetSerialPortRcvBuf       ; allow the serial receive interrupt to start a new packet
                                        ; see "Serial Data Timing" notes at the top of this page
    
    ;verify the checksum

    banksel serialRcvPktLenMain
  
    movf    serialRcvPktLenMain, W      ; copy number of bytes to variable for counting
    movwf   serialRcvPktCntMain

    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H ; point FSR0 at start of receive buffer
    movwf   FSR0H
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   FSR0L

    clrw                                ; preload W with zero

hspSumLoop:

    addwf   INDF0, W                    ; sum each data byte and the checksum byte at the end
    addfsr  FSR0,1
    decfsz  serialRcvPktCntMain, F
    goto    hspSumLoop

    movf    WREG, F                         ; test for zero
    btfsc   STATUS, Z                       ; error if not zero
    ;debug mksgoto    parseCommandFromSerialPacket    ; checksum good so handle command

hspError:

    incf    serialPortErrorCnt, F           ; track errors
    bsf     statusFlags,SERIAL_COM_ERROR

    return

; end of handleSerialPacket
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; startSerialPortTransmit
;
; Initiates sending of the bytes in the transmit buffer. The transmission will be performed by an
; interrupt routine.
;
; The command byte and all following data bytes are used to compute the checksum which is inserted
; at the end.
;
; On Entry:
;
; serialXmtBufNumBytes should contain the number of bytes to send.
; The bytes to be sent should be in the serial port transmit buffer serialXmtBuf.
;

startSerialPortTransmit:

    ; get number of bytes stored in buffer, add one for the command byte, calculate checksum

    banksel serialXmtBufNumBytes
    movf    serialXmtBufNumBytes,W
    addlw   .1
    movwf   serialXmtBufNumBytes
    movwf   usartScratch0

    call    calcAndStoreCheckSumSerPrtXmtBuf

    banksel serialXmtBufPtrH                ; set FSR0 and pointer to start of transmit buffer
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H
    movwf   serialXmtBufPtrH
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   serialXmtBufPtrL
    movwf   FSR0L

    ; add 1 to length to account for checksum byte, store in packet length byte
    ;  packet length = command byte + data bytes + checksum

    banksel serialXmtBufNumBytes
    movf    serialXmtBufNumBytes,W
    addlw   .1
    movwi   2[FSR0]

    ; add 3 to length to account for two header bytes and length byte, store for xmt routine
    ; this is the total number of bytes to transmit

    addlw   .3
    movwf   serialXmtBufNumBytes

    banksel PIE1                            ; enable transmit interrupts
    bsf     PIE1, TXIE                      ; interrupt will trigger when transmit buffers empty

    return

; end of startSerialPortTransmit
;--------------------------------------------------------------------------------------------------
 
;--------------------------------------------------------------------------------------------------
; clearSerialPortXmtBuf
;
; Sets all bytes up to 255 in the Serial Port transmit buffer to zero. If the buffer is larger
; than 255 bytes, only the first 255 will be zeroed.
;

clearSerialPortXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    banksel usartScratch0                       ; get buffer size to count number of bytes zeroed
    movlw   SERIAL_XMT_BUF_LEN
    movwf   usartScratch0

    movlw   0x00

cSPXBLoop:

    movwi   FSR0++
    decfsz  usartScratch0,F
    goto    cSPXBLoop

    return

; end of clearSerialPortXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupSerialPort
;
; Sets up the serial port for communication.
; Also prepares the receive and transmit buffers for use.
;

setupSerialPort:

    banksel serialRcvBufLen     ;store buffer length constants in variables for easier maths

    movlw   SERIAL_RCV_BUF_LEN
    movwf   serialRcvBufLen
    movlw   SERIAL_XMT_BUF_LEN
    movwf   serialXmtBufLen

    clrf    serialPortErrorCnt
    bcf     statusFlags,SERIAL_COM_ERROR

    ;to set the baud rate to 57,600 (will actually be 57.97K with 0.64% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 68

    ;to set the baud rate to 19,200 (will actually be 19.23K with 0.16% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 207

    ;to set the baud rate to 9,600 (will actually be 9592 with 0.08% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 416 (0x1a0)
    
    ;to set the baud rate to 2,400 (will actually be 2399.5 with 0.02% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 1666 (0x682)
    
    banksel TXSTA
    bsf     TXSTA, BRGH
    banksel BAUDCON
    bsf     BAUDCON, BRG16
    banksel SPBRGH
    movlw   0x01
    movwf   SPBRGH
    banksel SPBRGL
    movlw   0xa0
    movwf   SPBRGL

    ;set UART mode and enable receiver and transmitter

    banksel ANSELB          ; RB5/RB7 digital I/O for use as RX/TX
    bcf     ANSELB,RB5
    bcf     ANSELB,RB7

    banksel TRISB
    bsf     TRISB, TRISB5   ; set RB5/RX to input
    bcf     TRISB, TRISB7   ; set RB7/TX to output

    banksel TXSTA
    bcf     TXSTA, SYNC     ; clear bit for asynchronous mode
    bsf     TXSTA, TXEN     ; enable the transmitter
    bsf     RCSTA, CREN     ; enable the receiver
    bsf     RCSTA, SPEN     ; enable EUSART, configure TX/CK I/O pin as an output

    call    resetSerialPortRcvBuf
    call    resetSerialPortXmtBuf

    ; enable the receive interrupt; the transmit interrupt (PIE1/TXIE) is not enabled until data is
    ; ready to be sent
    ; for interrupts to occur, INTCON/PEIE and INTCON/GIE must be enabled also

    banksel PIE1
    bsf     PIE1, RCIE      ; enable receive interrupts
    bcf     PIE1, TXIE      ; disable transmit interrupts (re-enabled when data is ready to xmt)

    return

; end of setupSerialPort
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForTXIFHigh
;
; Waits in a loop for TXIF bit in register PIR1 to go high. This signals that the EUSART serial
; port transmit buffer is empty and a new byte can be sent.
;

waitForTXIFHigh:

    ifdef DEBUG_MODE  ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    return            ; simulated by the IDE
    endif

    banksel PIR1

wfth1:
    btfss   PIR1, TXIF
    goto    wfth1

    return

; end of waitForTXIFHigh
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetSerialPortRcvBuf
;
; Resets all flags and variables associated with the serial port receive buffer.
;

resetSerialPortRcvBuf:

    banksel flags2

    clrf    serialRcvPktLen
    clrf    serialRcvPktCnt
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H
    movwf   serialRcvBufPtrH
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   serialRcvBufPtrL

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    RSPRBnoOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN
        
RSPRBnoOERRError:

    banksel RCREG           ; clear any pending interrupt by clearing both bytes of the buffer
    movf    RCREG, W
    movf    RCREG, W
    
    banksel flags2

    bcf     flags2, HEADER_BYTE_1_RCVD
    bcf     flags2, HEADER_BYTE_2_RCVD
    bcf     flags2, LENGTH_BYTE_VALID
    bcf     flags2, SERIAL_PACKET_READY
    
    return

; end of resetSerialPortRcvBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetSerialPortXmtBuf
;
; Resets all flags and variables associated with the serial port transmit buffer.
;

resetSerialPortXmtBuf:

    banksel flags2

    clrf    serialXmtBufNumBytes
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H
    movwf   serialXmtBufPtrH
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   serialXmtBufPtrL

    return

; end of resetSerialPortXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calcAndStoreCheckSumSerPrtXmtBuf
;
; Calculates the checksum for a series of bytes in the serial port transmit buffer. The two
; header bytes and the length byte are not included in the checksum.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series, not including the 2 header bytes and 1 length
; byte
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calcAndStoreCheckSumSerPrtXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    addfsr  FSR0,3                              ; skip 2 header bytes and 1 length byte
                                                ; command byte is part of checksum

    goto    calculateAndStoreCheckSum

; end calcAndStoreCheckSumSerPrtXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calculateAndStoreCheckSum
;
; Calculates the checksum for a series of bytes.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series
; FSR0 points to first byte in series.
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calculateAndStoreCheckSum:

    call    sumSeries                       ; add all bytes in the buffer

    comf    WREG,W                          ; use two's complement to get checksum value
    addlw   .1

    movwi   FSR0++                          ; store the checksum at the end of the summed series

    return

; end calculateAndStoreCheckSum
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sumSeries
;
; Calculates the sum of a series of bytes. Only the least significant byte of the sum is retained.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series.
; FSR0 points to first byte in series.
;
; On Exit:
;
; The least significant byte of the sum will be returned in WREG.
; Z flag will be set if the LSB of the sum is zero.
; FSR0 points to the location after the last byte summed.
;

sumSeries:

    banksel usartScratch0

    clrf    WREG

sumSLoop:                       ; sum the series

    addwf   INDF0,W
    addfsr  INDF0,1             ; debug mks -- this should be FSR0 not INDF0???

    decfsz  usartScratch0,F
    goto    sumSLoop

    return

; end sumSeries
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInterrupt
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleInterrupt:

                                    ; INTCON is a core register, no need to banksel
	btfsc 	INTCON, T0IF     		; Timer0 overflow interrupt?
	call 	handleTimer0Int         ; call so the serial port interrupts will get checked
                                    ;  if not, the timer interrupt can block them totally

    banksel PIR1
    btfsc   PIR1, RCIF              ; serial port receive interrupt
    goto    handleSerialPortReceiveInt

    banksel PIE1                    ; only handle UART xmt interrupt if enabled
    btfss   PIE1, TXIE              ;  the TXIF flag is always set whenever the buffer is empty
    retfie                          ;  and should be ignored unless the interrupt is enabled
    
    banksel PIR1
    btfsc   PIR1, TXIF              ; serial port transmit interrupt
    goto    handleSerialPortTransmitInt


; Not used at this time to make interrupt handler as small as possible.
;	btfsc 	INTCON, RBIF      		; NO, Change on PORTB interrupt?
;	goto 	portB_interrupt       	; YES, Do PortB Change thing

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

endISR:

	retfie                  	; Return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Int
;
; This function is called when the Timer0 register overflows.
;
; TMR0 is never reloaded -- thus it wraps around and does a full count for each interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Int:

	bcf 	INTCON,T0IF     ; clear the Timer0 overflow interrupt flag

    banksel flags3

    btfss   flags3,DEBOUNCE_ACTIVE  ; if debounce active, decrement counter
    goto    noDebounceDec

    movlw   .1
    subwf   debounceL,F
    btfsc   STATUS,C        ; carry clear = borrow
    goto    noDecMSB
    
    decf	debounceH,F

noDecMSB:
    
    movf    debounceH,W     ; catch both bytes zeroed
    iorwf   debounceL,W
    btfsc   STATUS,Z
    bcf     flags3,DEBOUNCE_ACTIVE    
    
noDebounceDec:
    
    return

; end of handleTimer0Int
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortReceiveInt
;
; This function is called when a byte(s) has been received by the serial port. The byte(s) will be
; checked to see if it is a header byte, a packet length byte, or a data byte. Data bytes will be
; stored in a buffer. If an error occurs in receiving a packet, the function will ignore data
; received before the error and begin watching for the next packet signature. Upon receiving a
; complete packet, a flag will be set to notify the main loop.
;
; The receive register is a two byte fifo, so two bytes could be ready. This function will process
; all bytes available.
;
; The RCIF flag is cleared by reading all data from the two byte receive FIFO.
;
; This code check each byte sequence to see if it starts with a header prefix (0xaa,0x55) followed
; by a valid length byte. If these are found, the bytes after the length byte are stored in a
; buffer. If the sequence is not matched or the supposed length byte is larger than the buffer,
; all flags are reset and the search for the first header byte starts over.
;
; Packet format:
;   0xaa, 0x55, length, data1, data2, data3,...checksum.
;
; This interrupt function does not verify the checksum; the main loop should do that if required.
; Once a packet has been received, a flag is set to alert the main loop that it is ready for
; processing. All further data will be ignored until the main loop clears that flag. If an error
; occurs, the data received to that point will be discarded and the search for the next packet
; begun anew.
;
; The packet length byte is the number of data bytes plus one for the checksum byte. It does not
; include the two header bytes or the length byte itself. If the length byte value is 0 or is
; greater than the buffer size, the packet will be ignored. If the length byte value is greater
; than the actual number of bytes sent (but still less than the buffer size), the current packet
; AND the next packet(s) will be discarded as the interrupt routine will wait until enough bytes
; are received from subsequent packets to equal the erroneously large length byte value.
;
; Thus, only one packet at a time can be handled. The processing required is typically minimal, so
; the main loop should be able to process each packet before another is received. Some care should
; be taken by the receiver to not flood the line with packets.
;
; The main loop does all the actual processing in order to minimize the overhead of the interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleSerialPortReceiveInt:

    ; if the packet ready flag is set, ignore all data until main loop clears it

    banksel flags2
    btfss   flags2, SERIAL_PACKET_READY
    goto    readSerialLoop

    ; packet ready flag set means last packet still being processed, read byte to clear interrupt
    ; or it will result in an endless interrupt loop, byte is tossed and a resync will occur

    banksel RCREG
    movf    RCREG, W
    goto    rslExit

    ;RCREG is a two byte FIFO and may contain two bytes; read until RCIF flag is clear

readSerialLoop:

    banksel RCREG
    movf    RCREG, W        ; get byte from receive fifo

    banksel flags2

    btfsc   flags2, HEADER_BYTE_1_RCVD      ; header byte 1 already received?
    goto    rsl1                            ; if so, check for header byte 2

    bsf     flags2, HEADER_BYTE_1_RCVD      ; preset the flag, will be cleared on fail

    sublw   0xaa                            ; check for first header byte of 0xaa
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 1, reset all to restart search

rsl1:
    btfsc   flags2, HEADER_BYTE_2_RCVD      ; header byte 2 already received?
    goto    rsl2                            ; if so, check for length byte

    bsf     flags2, HEADER_BYTE_2_RCVD      ; preset the flag, will be cleared on fail

    sublw   0x55                            ; check for second header byte of 0x55
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 2, reset all to restart search

rsl2:
    btfsc   flags2, LENGTH_BYTE_VALID       ; packet length byte already received and validated?
    goto    rsl3                            ; if so, jump to store data byte

    movwf   serialRcvPktLen                 ; store the packet length
    movwf   serialRcvPktCnt                 ; store it again to count down number of bytes stored

    bsf     flags2, LENGTH_BYTE_VALID       ; preset the flag, will be cleared on fail

    movf    serialRcvPktLen, F              ; check for invalid packet size of 0
    btfsc   STATUS, Z
    goto    rslError

    subwf   serialRcvBufLen, W              ; check if packet length < buffer length
    btfsc   STATUS, C                       ; carry cleared if borrow was required
    goto    rsllp                           ; continue on, leaving flag set
                                            ; if invalid length, reset all to restart search

rslError:

    incf    serialPortErrorCnt, F           ; track errors
    bsf     statusFlags,SERIAL_COM_ERROR
    call    resetSerialPortRcvBuf    
    goto    rsllp

rsl3:

    movwf   serialIntScratch0               ; store the new character

    movf    serialRcvBufPtrH, W             ; load FSR0 with buffer pointer
    movwf   FSR0H
    movf    serialRcvBufPtrL, W
    movwf   FSR0L

    movf    serialIntScratch0, W            ; retrieve the new character
    movwi   INDF0++                         ; store in buffer

    movf    FSR0H, W                        ; save adjusted pointer
    movwf   serialRcvBufPtrH
    movf    FSR0L, W
    movwf   serialRcvBufPtrL

    decfsz  serialRcvPktCnt, F              ; count down number of bytes stored
    goto    rsllp                           ; continue collecting until counter reaches 0

rsl4:

    bsf     flags2, SERIAL_PACKET_READY     ; flag main loop that a data packet is ready
    goto    rslExit

rsllp:

    banksel PIR1                            ; loop until receive fifo is empty
    btfsc   PIR1, RCIF
    goto    readSerialLoop

rslExit:

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    noOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN

noOERRError:

    goto    endISR

; end of handleSerialPortReceiveInt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortTransmitInt
;
; This function is called when a byte is to be transmitted to the host via serial port. After
; data is placed in the transmit buffer, the TXIE flag is enabled so this routine gets called
; as an interrupt whenever the transmit buffer is empty. After all bytes in the buffer have been
; transmitted, this routine clears the TXIE flag to disable further interrupts.
;
; Before the TXIE flag is set to start the process, serialXmtBufNumBytes should be set to value
; > 0, i.e. the number of valid bytes in the transmit buffer.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;
; The TXIF flag is cleared in the second instruction cycle after writing data to TXREG.
;

handleSerialPortTransmitInt:

    banksel serialXmtBufPtrH                ; load FSR0 with buffer pointer
    movf    serialXmtBufPtrH, W
    movwf   FSR0H
    movf    serialXmtBufPtrL, W
    movwf   FSR0L

    moviw   FSR0++                          ; send next byte in buffer
    banksel TXREG
    movwf   TXREG

    banksel serialXmtBufPtrH                ; store updated FSR0 in buffer pointer
    movf    FSR0H, W
    movwf   serialXmtBufPtrH
    movf    FSR0L, W
    movwf   serialXmtBufPtrL

    decfsz  serialXmtBufNumBytes, F
    goto    endISR                          ; more data to send, exit with interrupt still enabled

    banksel PIE1                            ; no more data, disable further transmit interrupts
    bcf     PIE1, TXIE

    goto    endISR

; end of handleSerialPortTransmitInt
;--------------------------------------------------------------------------------------------------    
    
;--------------------------------------------------------------------------------------------------    
;--------------------------------------------------------------------------------------------------
;
;   End of EUSART Serial Port Core Functions
;
;--------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; Constants in Program Memory
;

; end of Constants in Program Memory
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; Strings in Program Memory
;
    
warningStr  dw  ' ',' ','*',' ', 'W','a','r','n','i','n','g',' ','*',0x00
errorStr    dw  ' ',' ',' ','*',' ', 'E','r','r','o','r',' ','*',0x00
clearStr    dw  ' ',' ', ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0x00
  
string0	    dw	'N','o','a','h','B','o','t',' ','1','.','1',0x00
string1	    dw	'N','o','a','h',' ','=',' ','t','a','t','e','r',' ','t','o','t',0x00
string2	    dw	'D','e','s','t','r','u','c','t','o',' ','M','o','d','e',0x00
string3	    dw	'I','\'','m',' ','f','*','i','n','g',' ','l','o','s','t','!',0x00
string4	    dw	'O',0x00
string5	    dw	'1',0x00
string6	    dw	'1',0x00
string7	    dw	'2',0x00
string8	    dw	'3',0x00
string9	    dw	' ',0x00
string10    dw	'0',0x00
string11    dw	'J',0x00
string12    dw	'Z',0x00
string13    dw	'O',0x00
string14    dw	'T',0x00
string15    dw	'U',0x00
string16    dw	'D',0x00
string17    dw	'N',0x00
string18    dw	'W',0x00
string19    dw	'C',0x00
string20    dw	'5',0x00
string21    dw	'4',0x00
string22    dw	'N',0x00
string23    dw	'R',0x00
string24    dw	'6',0x00
string25    dw	'7',0x00
string26    dw	'N',0x00
string27    dw	'1',0x00
string28    dw	'O',0x00
string29    dw	'O',0x00
    
; end of Strings in Program Memory
;--------------------------------------------------------------------------------------------------

    END
