;--------------------------------------------------------------------------------------------------
; Project:  OPT EDM Notch Cutter -- Main PIC software
; Date:     2/29/12
; Revision: See Revision History notes below.
;
; IMPORTANT: When programming the PIC in the notch cutter, turn the Electrode Current switch to
; Off and the Electrode Motion switch to Setup.
;
; Normally, the programming header for the LCD PIC is not installed on the board.  It can be
; installed in the Main PIC socket, programmed, and then moved to the LCD PIC socket.
;
; Overview:
;
; This program controls an EDM cutting device by manipulating a motor which moves the head up and
; down and a high current power supply which provides voltage to the cutting blade.  The current
; output of the electrode cutting blade is monitored to adjust the height of the blade above the
; material being cut so that the current maintains optimum value.
;
; The program monitors several button inputs and displays data on an LCD display.
;
; There are two PIC controllers on the board -- the Main PIC and the LCD PIC.  This code is
; for the Main PIC.  The Main PIC sends data to the LCD PIC via a serial data line for display
; on the LCD.
;
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
; 1.0   Some code and concepts used from source code disassembled from hex object code version 6.4 
;       from original author.
; 7.6e	Version distributed.  Has aggression control.
; 7.7a	Fixed bug: asterisk wasn't displaying by the "Up" label when doing fast retract from
;		 over current condition in auto cut mode.
;		Increased responsiveness in auto cut mode.
;		Changed motor step size back to Full.  This was the original setting used by the designer.
; 7.7b  Motor direction reversed so motor wiring makes more sense.
;		The "R" prefix removed from the displayed version so that the letter suffix can fit.
; 7.7c	Added "repeat cycle" test function for testing for proper operation.  The head will be
;		driven down until the low current input signal is cleared and then retracted quickly
;		back to the starting position.  The cycle is repeated until the user exits.
; 7.7d	Fixed bug in "repeat cycle" test where it was locking up in the retreat mode because
;		the power supply wasn't coming up fast enough at the start.
;		Improved the input button handling - better debounce, faster response.
;		Improved the multiple page menu handling - cursor starts on bottom option when moving
;		back to a previous menu page.
; 7.7e  Fixed incrementing/decrementing of multibyte variables -- decrementing was skipping a
;		count when crossing the zero thresold of an upper byte.
; 7.7f	Fixed comments explaining commands to the LCD screen. Cleaned up superfluous code.
; 7.7g	Refactoring for clarity.
;
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
; Operational Notes
;
; Standard Reach Tool
;
; The cam should turn clockwise when the head is being driven down.
; The cam should make a full rotation in 65 seconds while jogging up or down in Setup mode.
;
; The motor used for the OPT standard reach head is:
;
; manufactured by Faulhaber @ http://www.faulhaber.com/
; sold by www.micromo.com
;
; Printed on motor: 
;
;	ARSAPE
;	swiss made
;	AM1524-A-025-125-71
;
; 	AM1524 	  ~ AM motor design, 15 mm diameter, 24 steps per revolution
;	A-025-125 ~ -A-0,25-12,5 Winding Type
;	71	  ~ 71 (-73)  ~ Plain shaft, L=4,3mm (Designation for assembly with gearhead 15A)
;
; On the sticker on the motor:
;
;   Faulhaber
;   P/N: AM15P0199
;   PO: 956952-003
;   JO: 30212/2     KW36/13
;   www.micromo.com
;
;
; A planetary gearhead is attached: Faulhaber Series 15A.  There are no identifying numbers for
; the gearhead printed on the assembly.
; 
; Using Full Step mode, the motor has 24 steps per revolution.
;
; Empirical testing shows the gear reduction ratio to be 69:1
;
; Using Full Step mode, this results in 1656 steps per revolution of the cam:
;	24 steps/rev (motor shaft) * 69 (in revs / out revs) = 1656 steps per cam rev
;
; On the working side of the cam, each degree of rotation moves the head one mil.
; 1 degree = 1 mil
; Rate is 4.6 steps / degree (1656 / 360), which is 4.6 steps / mil of head travel.
;
; Thus, one step is 0.000217391"
;
; Extended Reach Tool
;
; The Standard Tool Ratio is 9.14 times the Extended Tool Ratio
;
; The same motor is used as for the Standard Reach Tool, but a lead screw is used which changes
; the final gear ratio.
;
; One motor revolution = 1mm of blade travel
; 1mm = 0.0393700787"
; One motor revolution is 1656 steps
; 0.0393700787" / 1656 = .000023774"
;
; Thus, one step is .000023774"
;
;--------------------------------------------------------------------------------------------------
;
; J8 on the schematics is implemented as switches labeled "microstep set" on the board.  The
; step size inputs to the motor controller chip are defined as follows:
;
; MS1 MS2
;  L   L  = Full Step
;  H   L  = Half
;  L   H  = Quarter
;  H   H =  Eighth
;
; Typical J8 switch settings on the board:
; 1 (MS1) = Off  2 (MS2) = On
;
; When a J8 switch is off, it forces the associated input (MS1 or MS2) to be low.
; When a J8 switch is on, it allows the associated input to be controlled by RB2.
;
; Thus, with MS1 Off, it is always low.
; With MS1 On, it is set by RB2.
; If RB2 is set high, MS2 = high - step size will be 1/4.  If set low, step size will be full.
;
; The full step size is currently used - as of version 7.7a
;
; The PIC chip's internal comparators are NOT used to detect over/under current conditions - see
; note in header of "setup" function.
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
; On startup, Mode button used to select between Standard Cut Depth and Extended Cut Depth
;
; Mode button used during operation to select Cut, Jog, & Setup modes.
;
; Jog Up and Jog Down buttons used to move cutting head up & down.
;
; Select button used to accept an entry or selection, stop a function, or return to a menu.
;  (also referred to as "Reset" on the schematic and "Zero Reset" on wiring harness"
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

; Values for the digital pot settings.
;
; Each count equals approximately 0.0196V when power is at +5V
;

HI_CURRENT_LIMIT_POT    EQU     .69     ; .69/.59 cuts .060 notch in 1.5 minutes
LO_CURRENT_LIMIT_POT    EQU     .59

CURRENT_LIMIT_DIFFERENCE    EQU .10     ; difference between high and low current pot values

CURRENT_LIMIT_POT_OFFSET    EQU .49     ; base level for HI_CURRENT_LIMIT_POT when power level
                                        ; is 0 (0 not actually used -- 1 is lowest value)
CURRENT_LIMIT_POT_INCREMENT EQU .10     ; increment between each pot value setting for each
                                        ; count of power level

VOLTAGE_MONITOR_POT         EQU     .255
CURRENT_MONITOR_POT         EQU     .255


; Cutting Current Pulse Controller Values (cycle width and duty cycle)
;
; with oscillator frequency of 16 Mhz and Timer 2 prescaler set to 4:
;
; each count of PWM_PERIOD equals 1uS of period with an offset of +1
;
;    period uS = (PWM_PERIOD + 1) * 1 uS
;
; each count of PWM_DUTY_CYCLE_LO_BYTE_DEFAULT:PWM_DUTY_CYCLE_LO_BYTE_DEFAULT equals 0.25 uS
;  for the high pulse width -- note that signal is inverted before it reaches testpoing J23 so this
;  is actually a low pulse at that point
;
;   pulse width uS = PWM_DUTY_CYCLE_LO_BYTE_DEFAULT:PWM_DUTY_CYCLE_LO_BYTE_DEFAULT * 0.25 uS
;


PWM_PERIOD_DEFAULT EQU  .217                ; gives 218 uS PWM period
                                            ; period uS = (PWM_PERIOD + 1) * 1 uS

PWM_DUTY_CYCLE_HI_BYTE_DEFAULT      EQU     0x00    ; 00:b8 gives 46 uS PWM high pulse width
PWM_DUTY_CYCLE_LO_BYTE_DEFAULT      EQU     0xb8    ; (high:lo) * 0.25 uS = high pulse width
                                                    ; note that signal is inverted befor it reaches
                                                    ; testpoint J23

PWM_POLARITY_DEFAULT                EQU     0x00


; LED PIC Commands

LEDPIC_SET_LEDS                 EQU 0x00    ; sets the on/off states of the LED arrays
LEDPIC_SET_PWM                  EQU 0x01    ; sets the PWM values
LEDPIC_START                    EQU 0x02    ; starts normal operation
LEDPIC_SET_RESET                EQU 0xff    ; resets to a known state

    
;LCD Screen Position Codes                
                
LINE1_COL1  EQU     0x80                
LINE2_COL1  EQU     0xc0
LINE3_COL1  EQU     0x94
LINE4_COL1  EQU     0xd4

LINE1_COL7  EQU     0x86  
LINE2_COL2  EQU     0xc1  
LINE2_COL5  EQU     0xc4
LINE2_COL14 EQU     0xcd
LINE3_COL2  EQU     0x95  
LINE4_COL12 EQU     0xdf
LINE4_COL18 EQU     0xe5

; end of Defines
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

#INCLUDE <STANDARD 2.MAC>     	; include standard macros

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

UNUSED_RA1_P    EQU     PORTA
UNUSED_RA1      EQU     RA1         ; input ~ RA1 can only be input on PIC16f1459
;NA_RA2         EQU     RA2         ; RA2 not implemented on PIC16f1459
SHORT_DETECT_P  EQU     PORTA
SHORT_DETECT    EQU     RA3         ; input ~ RA3 can only be input on PIC16f1459
HI_LIMIT_P      EQU     PORTA
HI_LIMIT        EQU     RA4         ; input ~ cutting current hight limit
POWER_ON_L      EQU     LATA
POWER_ON        EQU     RA5         ; output
;NA_RA6         EQU     RA6         ; RA6 not implemented on PIC16f1459
;NA_RA7         EQU     RA7         ; RA7 not implemented on PIC16f1459

; Port B

I2CSDA_LINE     EQU     RB4
I2CSCL_LINE     EQU     RB6

; Port C

MOTOR_ENABLE_L  EQU     LATC
MOTOR_ENABLE    EQU     RC0         ; output
MOTOR_DIR_L     EQU     LATC
MOTOR_DIR       EQU     RC3         ; output
MOTOR_STEP_L    EQU     LATC
MOTOR_STEP      EQU     RC4         ; output
LO_LIMIT_P      EQU     PORTC
LO_LIMIT		EQU     RC5         ; input ~ cutting current low limit
MOTOR_MODE_L    EQU     LATC
MOTOR_MODE      EQU     RC6         ; output ~ motor step size selection

; Switches

MODE_JOGUP_SEL_EPWR_P   EQU     PORTC
JOGDWN_P                EQU     PORTA

MODE_SW                 EQU     RC1
JOG_UP_SW               EQU     RC2
SELECT_SW               EQU		RC7
AC_PWR_OK_SIGNAL        EQU     RC7     ; only when using User Interface Board               
ELECTRODE_PWR_SW        EQU     RC4
JOG_DOWN_SW             EQU     RA0

;bits in switchStates variable

MODE_SW_FLAG            EQU     0
JOG_UP_SW_FLAG          EQU     1
JOG_DOWN_SW_FLAG        EQU     2          
SELECT_SW_FLAG          EQU     3
ELECTRODE_PWR_SW_FLAG   EQU     4
AC_OK_FLAG              EQU     5

;bits in outputStates variable

AC_OK_LED_FLAG          EQU     0
BUZZER_FLAG             EQU     1
SHORT_LED_FLAG          EQU     2
       

; I2C bus ID byte for writing to digital pot 1
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 001 (bits 3-1)
; R/W bit set to 0 (bit 0)

DIGITAL_POT1_WRITE_ID       EQU     b'10100010'

HI_LIMIT_POT_ADDR           EQU     0x0
LO_LIMIT_POT_ADDR           EQU     0x1
VOLTAGE_MONITOR_POT_ADDR    EQU     0x2
CURRENT_MONITOR_POT_ADDR    EQU     0x3

; I2C bus ID bytes for writing and reading to EEprom 1
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 000 (bits 3-1)
; R/W bit set to 0 (bit 0) for writing
; R/W bit set to 1 (bit 0) for reading

EEPROM1_WRITE_ID            EQU     b'10100000'
EEPROM1_READ_ID             EQU     b'10100001'


; I2C bus ID bytes for writing and reading to the LED PIC
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 010 (bits 3-1)
; R/W bit set to 0 (bit 0) for writing
; R/W bit set to 1 (bit 0) for reading

LED_PIC_WRITE_ID            EQU     b'10100100'
LED_PIC_READ_ID             EQU     b'10100101'


; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions

; bits in flags variable

EXTENDED_MODE   EQU     0x0
CUT_STARTED     EQU     0x1
AT_DEPTH        EQU     0x2
WALL_MODE       EQU     0x3
DATA_MODIFIED   EQU     0x4
UPDATE_DISPLAY  EQU     0x5
UPDATE_DIR_SYM  EQU     0x6
MOTOR_DIR_MODE	EQU     0x7

; bits in flags2 variable

HEADER_BYTE_1_RCVD  EQU 0
HEADER_BYTE_2_RCVD  EQU 1
LENGTH_BYTE_VALID   EQU 2
SERIAL_PACKET_READY EQU 3
PACKET_SEND_TYPE    EQU 4

; bits in flags3 variable

EROSION_MODE    EQU     0
DEBOUNCE_ACTIVE EQU     1
TIME_CRITICAL   EQU     2
ALARM_ENABLED   EQU     3
    
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

; WARNING: flags, flags2, and flags3 should all be contiguous for eeprom saving/loading

    flags                   ; bit 0: 0 = standard mode, 1 = extended cut depth mode
                            ; bit 1: 0 = cut not started, 1 = cut started
                            ; bit 2: 0 = depth not reached, 1 = depth reached
                            ; bit 3: 0 = notch mode, 1 = wall reduction mode
                            ; bit 4: 0 = data not modified, 1 = data modified (used by various functions)
                            ; bit 5: 0 = no display update, 1 = display update (used by various functions)
							; bit 6: 0 = no update direction symbol, 1 = update (used by various functions)
							; bit 7: 0 = normal motor rotation, 1 = reverse motor direction

    flags2                  ; bit 0: 1 = first serial port header byte received
                            ; bit 1: 1 = second serial port header byte received
                            ; bit 2: 1 = serial port packet length byte received and validated
                            ; bit 3: 1 = data packet ready for processing
                            ; bit 4: 0 = send LCD data packet, 1 = send Output States packet
                            ; bit 5:
							; bit 6:
							; bit 7:

    flags3                  ; bit 0: 0 = no erosion factor, 1 = use erosion factor
                            ; bit 1: 0 = debounce timer, inactive 1 = timer active
                            ; bit 2: 0 = no time criticality, 1 = time critical operation
                            ; bit 3: 0 = alarm disabled, 1 = alarm enabled
                            ; bit 4:
                            ; bit 5:
                            ; bit 6:
                            ; bit 7:

    statusFlags             ; bit 0: 0 = one or more com errors from serial have occurred
                            ; bit 1: 0 = one or more com errors from I2C have occurred
                            ; bit 2: 0 =
                            ; bit 3: 0 =
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =
                            
    menuOption              ; tracks which menu option is currently selected

    xmtDataTimer            ; used to control rate of data sending
    
    switchStates

    switchStatesPrev        ; state of switches the last time they were scanned
                            ; bit assignments same as for buttonState

    switchStatesRemote      ; switch states reported by remote device such as User Interface Board

    outputStates            ; state of the outputs
    
    eepromAddressL		    ; use to specify address to read or write from EEprom
    eepromAddressH          ; high byte
    eepromCount	        	; use to specify number of bytes to read or write from EEprom

    hiCurrentLimitPot       ; value for digital pot which sets the high current limit value
    loCurrentLimitPot       ; value for digital pot which sets the high current limit value
    powerLevel              ; store the power level of the high/low current values in use

    speedValue              ; stores sparkLevel converted to a single digit

    sparkLevel              ; current value being used - copy from sparkLevelNotch or sparkLevelWall
                            ; depending on flags.WALL_MODE

    sparkLevelNotch         ; specifies the low amount of spark to trigger an advance in notch mode,
                            ;  smaller number makes advance more aggressive
    sparkLevelWall          ; same as above, but for the wall reduction mode
                            ; NOTE: keep sparkLevelNotch and sparkLevelWall contiguous

    sparkTimer1             ; tracks time between sparks (sensed by voltage change on comparator input)
    sparkTimer0

    overCurrentTimer1	    ; tracks time between over current spikes (sensed by voltage change on comparator input)
    overCurrentTimer0

    pwmSetCommandByte       ; convenience variable for the LED PIC set LEDs command byte
    pwmDutyCycleHiByte      ; cutting current pulse controller duty cycle time
    pwmDutyCycleLoByte
    pwmPeriod               ; cutting current pulse controller period time
    pwmPolarity             ; polarity of the PWM output -- only lsb used
    pwmCheckSum             ; used to verify PWM values read from eeprom

    debounceH               ; switch debounce timer decremented by the interrupt routine
    debounceL

    msDelayCnt
    bigDelayCnt
    smallDelayCnt
    
    normDelay               ; delay between motor pulses for normal mode (slow)
    setupDelay              ; delay between motor pulses for setup mode (fast)

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

	cycleTestRetract0		; wip mks - use a scratch variable instead?
	cycleTestRetract1		; wip mks - use a scratch variable instead?

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

    ; Current depth of electrode. This is the distance travelled since last zeroing.
    ; unpacked BCD decimal value, each can be 0-9
    ; The decimal point is two digits from the left for all depth related values: xx.xxxxxxxxx

    depth10                 ; most significant digit
    depth9
    depth8
    depth7
    depth6
    depth5
    depth4
    depth3
    depth2                  
    depth1
    depth0                  ; least significant digit
    depthSign               ; zero if value is positive or zero, non-zero if negative

    ; Target depth for electrode. This is the distance at which to stop cutting.
    ; unpacked BCD decimal value, each can be 0-9
    ; The decimal point is two digits from the left for all depth related values: xx.xxxxxxxxx

    target10                ; most significant digit
    target9
    target8
    target7
    target6
    target5
    target4
    target3
    target2                  
    target1
    target0                 ; least significant digit
    targetSign

    ; Number of inches per motor step -- used to update the depth value for each step.
    ; The appropriate step value for the selected head and erosion rate are copied here for
    ; use during operation.
    ; unpacked BCD decimal value, each can be 0-9
    ; The decimal point is two digits from the left for all depth related values: xx.xxxxxxxxx

    step10                 ; most significant digit
    step9
    step8
    step7
    step6
    step5
    step4
    step3
    step2                  
    step1
    step0                  ; least significant digit
    stepSign
    
    scratchc0               ; scratches defined in bank c(2) so that we don't have to switch
    scratchc1               ; banks all the time
    scratchc2

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
; Variables in EEprom
;
; Assign variables in EEprom
;

 cblock 	0x0      	; Variables start in RAM at 0x0
	
	eeTarget3           ; storage for target depth value
    eeTarget2
    eeTarget1
    eeTarget0

    eeFlags             ; flags2 not useful to save, but is in between flags and flags3, so...
    eeFlags2
    eeFlags3

    eeSparkLevelNotch       ; NOTE: keep eeSparkLevelNotch and eeSparkLevelWall contiguous
    eeSparkLevelWall

    eePWMDutyCycleHiByte    ; NOTE: keep PWM values contigious
    eePWMDutyCycleLoByte
    eePWMPeriod
    eePWMPolarity
    eePWMCheckSum


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

menuLoop:

    call    doExtModeMenu   ; display and handle the Standard / Extended Mode menu
    
    call    doMainMenu      ; display and handle the main menu

    goto    menuLoop
    
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

    banksel MODE_JOGUP_SEL_EPWR_P

    btfss   MODE_JOGUP_SEL_EPWR_P,MODE_SW
    bcf     switchStates,MODE_SW_FLAG

    btfss   MODE_JOGUP_SEL_EPWR_P,JOG_UP_SW
    bcf     switchStates,JOG_UP_SW_FLAG

; Select switch input is ignored here...for boards connected to a User Interface board, R84 is
; installed which allows the Cutting Current Power Supply's AC OK output to be read via this input.
; The signal is no longer valid as the Select switch. That switch is now connected to the User
; Interface board which will report its state via serial transmission.
;
;    btfss   MODE_JOGUP_SEL_EPWR_P,SELECT_SW
;    bcf     switchStates,SELECT_SW_FLAG

    btfss   MODE_JOGUP_SEL_EPWR_P,ELECTRODE_PWR_SW
    bcf     switchStates,ELECTRODE_PWR_SW_FLAG

    banksel JOGDWN_P

    btfss   JOGDWN_P,JOG_DOWN_SW
    bcf     switchStates,JOG_DOWN_SW_FLAG

    return

; end of trapSwitchInputs
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; setHighCurrentLimitDigitalPot
;
; Sets the digital pot value for the high current limit comparator to the value in hiCurrentLimit.
;

setHighCurrentLimitDigitalPot:

    banksel hiCurrentLimitPot
    movf    hiCurrentLimitPot,W

    banksel scratch0
    movwf   scratch1                ; the pot value
    movlw   HI_LIMIT_POT_ADDR
    movwf   scratch0

    goto    setDigitalPotInChip1

; end of setHighCurrentLimitDigitalPot
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLowCurrentLimitDigitalPot
;
; Sets the digital pot value for the low current limit comparator to the value in loCurrentLimit.
;

setLowCurrentLimitDigitalPot:

    banksel loCurrentLimitPot
    movf    loCurrentLimitPot,W

    banksel scratch0
    movwf   scratch1                ; the pot value
    movlw   LO_LIMIT_POT_ADDR
    movwf   scratch0

    goto    setDigitalPotInChip1

; end of setLowCurrentLimitDigitalPot
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; saveDepthValueToEEprom
;
; Saves the user set cut depth value from eeprom. These are BCD digits.
;
; Note: these variables must be kept contiguous in memory and eeprom.
;

saveDepthValueToEEprom:

    banksel eeTarget3

    movlw   high target9     ; address in RAM
    movwf   FSR0H
    movlw   low target9
    movwf   FSR0L
        
    clrf    eepromAddressH
    movlw   eeTarget3        ; address in EEprom
    movwf   eepromAddressL
    movlw   .4
    movwf   eepromCount     ; write 4 bytes
    call    writeToEEprom

    return

; end of saveDepthValueToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; saveSparkLevelsToEEprom
;
; Saves the spark Wall/Notch aggression/speed values to eeprom.
;
; Note: these variables must be kept contiguous in memory and eeprom.

saveSparkLevelsToEEprom:

    banksel sparkLevelNotch

    movlw   high sparkLevelNotch     ; address in RAM
    movwf   FSR0H
    movlw   low sparkLevelNotch
    movwf   FSR0L
        
    clrf    eepromAddressH
    movlw   eeSparkLevelNotch   ; address in EEprom
    movwf   eepromAddressL
    movlw   .2
    movwf   eepromCount         ; write 2 bytes
    call    writeToEEprom

    return

; end of saveSparkLevelsToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; saveFlagsToEEprom
;
; Saves the flags, flags2, and flags3 values to eeprom.
;
; The flags2 variable does not contain values which need to be saved, but it is between flags
; and flags3 so it is easier to save it than to skip it. 
;

saveFlagsToEEprom:

    banksel flags

    movlw   high flags      ; address in RAM
    movwf   FSR0H
    movlw   low flags
    movwf   FSR0L
    
    clrf    eepromAddressH
    movlw   eeFlags         ; address in EEprom
    movwf   eepromAddressL
    movlw   .3
    movwf   eepromCount     ; write 3 bytes
    call    writeToEEprom

    return

; end of saveFlagsToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; savePWMValuesToEEprom
;
; Saves PWM module time period and duty cycle and output polarity values to the eeprom.
;
; These values are sent to the LED PIC to specify the period and duty cycle of the pulse
; controlling the on/off times of the cutting current.
;
; Before saving, the bytes are added together to create a checksum which is also stored in the
; eeprom. This allows the reading function to verify that the values are valid and also that the
; eeprom was not empty and the values had never been stored.
;
; The value of 1 is added to the checksum to catch cases where the eeprom values have never been
; set and are all zeros in which case the data bytes would sum to zero and then match the zero
; value also read for the checksum -- this would make the zero values appear to be valid. Adding
; 1 means the checksum would also have to be 1 to create a match, which is possible if eeprom is
; filled with random values but not as likely as an all zero case.
;

savePWMValuesToEEprom:
    
    banksel pwmDutyCycleHiByte

    clrw                                ; calculate the checksum for all PWM values
    addwf   pwmDutyCycleHiByte,W
    addwf   pwmDutyCycleLoByte,W
    addwf   pwmPeriod,W
    addwf   pwmPolarity,W
    addlw   1                           ; see note in function header
    movwf   pwmCheckSum

    movlw   high pwmDutyCycleHiByte     ; address in RAM
    movwf   FSR0H
    movlw   low pwmDutyCycleHiByte      ; address in RAM
    movwf   FSR0L
  
    clrf    eepromAddressH
    movlw   eePWMDutyCycleHiByte        ; address in EEprom
    movwf   eepromAddressL
    movlw   .5
    movwf   eepromCount                 ; write 4 bytes
    call    writeToEEprom

    return

; end of savePWMValuesToEEprom
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

    call    sendOutputStatesIfReady ; send output states to remote devices if xmt buffer is ready    
    
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
; sendOutputStatesIfReady
;
; Sends output state data when the serial transmit buffer is empty. A counter is used to limit the
; rate at which the data is sent to avoid overrunning the remotes with data.
;
; Sets up buffer for a SET_OUTPUTS_CMD and adds appropriate data. The data in the buffer is then
; begun to be transmitted.
;
; Waits for transmission to be completed.
;
; Sets up the serial transmission buffer in preparation for the next printString (LCD_BLOCK_CMD
;  packet).
;
; Since LCD_BLOCK_CMD packets the most common packet sent and the buffer must be prepared for
; various data insertions, this function leaves the buffer prepared for that packet type.
;
; Call sendOutputStatesIfReadyQ to force a send without regard to the send rate control timer.
;
; On entry:
;
; FSR1 points to the desired string
;

sendOutputStatesIfReady:

    banksel flags                   ; only send when counter reaches zero to limit send rate
    decfsz  xmtDataTimer,F
    return

sendOutputStatesIfReadyQ:
    
    banksel serialXmtBufNumBytes    ; exit if buffer busy
    movf    serialXmtBufNumBytes,W
    btfsc   STATUS,Z
    goto    sendSOSIR
    
    movlw   .1                      ; if timed out but buffer was busy, set to 1 so there will not
    movwf   xmtDataTimer            ; be another delay and the data will be transmitted the next
    return                          ; time the buffer is ready
    
sendSOSIR:    

    movlw   0x10                    ; flip the send packet type selector bit
    xorwf   flags2,F

    movlp   high sendOutputStates
    call    sendOutputStates
    movlp   high sendOutputStatesIfReady

    banksel flags3
    btfsc   flags3,TIME_CRITICAL    ; do not wait or set up buffer if time critical flag set
    return
    
    call    waitSerialXmtComplete   ; wait until buffer printed

    call    setupLCDBlockPkt        ; prepare xmt buffer for the next block transmit
    
    return

; end of sendOutputStatesIfReady
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; zeroTarget
;
; Zeroes the target depth variable.
;
; On entry:
;

zeroTarget:
    
    movlw   high target10
    movwf   FSR0H
    movlw   low target10
    movwf   FSR0L
    
    goto    zeroVariable
    
; end of zeroTarget
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; zeroDepth
;
; Zeroes the depth position variable.
;
; On entry:
;

zeroDepth:

    movlw   high depth10
    movwf   FSR0H
    movlw   low depth10
    movwf   FSR0L

zeroVariable:
    
    clrw                    ; W = 0

    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++
    movwi   FSR0++          ; clear sign

    return

; end of zeroDepth
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
; Sets up the serial transmission buffer in preparation for the next printString (LCD_BLOCK_CMD
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
; doExtModeMenu
;
; Displays and handles the Standard / Extended depth mode menu.
;
; If doExtModeMenu is called, the menu is displayed with option 1 highlighted.
; If doExtModeMenuA is called, the cursorPos and menuOption should be preloaded by the calling
;  function to set the default highlighted option.
;
; Menu display:
;
; "OPT AutoNotcher x.x"
;
; 0x1, 0xC0
; "CHOOSE CONFIGURATION"
;
; 0x1, 0x94
; "1 - EDM Notch Cutter"
;
; 0x1, 0xd4
; "2 - EDM Extend Reach"
;
; 0x1, 0x94
; Carriage Return
;
; It then processes user input.
;
; NOTE: LCD addressing is screwy - second line first column is 0xC0, third line is 0x94,
;       fourth line is 0xd4.
;

doExtModeMenu:

    ; call here to default to option 1

    movlw   LINE3_COL1      ; set display position
    movwf   cursorPos       ; option 1 highlighted
    movlw   0x1
    movwf   menuOption      ; option 1 currently selected

doExtModeMenuA:				; call here if default option has already been set by caller

;print the strings of the menu

    movlp   high setupLCDBlockPkt    
    call    setupLCDBlockPkt    ; prepare block data packet for LCD
    
    movlw   high string0   ; "OPT AutoNotcher x.x"
    movwf   FSR1H
    movlw   low string0
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done
    
    movlw   LINE2_COL1      ; set display position
    call    writeControl
    movlw   high string1    ; "CHOOSE CONFIGURATION"
    movwf   FSR1H
    movlw   low string1
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done
    
    movlw   LINE3_COL1      ; set display position
    call    writeControl
    movlw   high string2    ; "1 - EDM Notch Cutter"
    movwf   FSR1H
    movlw   low string2
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE4_COL1      ; set display position
    call    writeControl
    movlw   high string3    ; "2 - EDM Extend Reach"
    movwf   FSR1H
    movlw   low string3
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

; position the cursor on the default selection
    
    movf    cursorPos,W		; load the cursor position to highlight the current choice
    call    writeControl
    call    turnOnBlink
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

; scan for button inputs, highlight selected option, will return when Reset/Enter/Zero pressed

LoopDEMM1:

    bcf		menuOption,7		; clear the menu page change flags
    bcf		menuOption,6

    movlw   .02                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    
    movwf   scratch0
    
    decfsz  scratch0,F
    goto    skipDEMM6

    ; handle option 1 - standard reach cutting head installed

    bcf     flags,EXTENDED_MODE ; set flag to 0

    movlp   high setStepDistance
    call    setStepDistance
    movlp   high exitDEMM
    
    ; standard head delay values
    
    movlw   .32
    movwf   setupDelay
    movlw   .100
    movwf   normDelay

    goto    exitDEMM
   
skipDEMM6:

    decfsz  scratch0,F
    goto    skipDEMM7

    ; handle option 2 - extended reach cutting head installed

    bsf     flags,EXTENDED_MODE ; set flag to 1    
    
    movlp   high setStepDistance
    call    setStepDistance
    movlp   high exitDEMM   
    
    ; extended head delay values

    movlw   .4
    movwf   setupDelay
    movlw   .12
    movwf   normDelay
    
    goto    exitDEMM

skipDEMM7:
	
	; this part reached if a menu page change flag bit is set in menuOption
	; since there is only one page for this menu, ignore
	goto	LoopDEMM1

exitDEMM:
    
    return    

; end of doExtModeMenu
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
; Screen displayed:
;
; "OPT EDM Notch Cutter"
;
; 0x1, 0xc0
; "1 - Set Cut Depth" or "1 - Depth = "
;
; 0x1, 0x94
; "2 - Cut Notch"
;
; 0x1, 0xd4
; "3 - Jog Electrode"
;
; 0x1, 0xc0
; Carriage Return (to place cursor on first option)
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
    
    btfsc  flags,EXTENDED_MODE  ; check for extended mode
    goto    extendedModeDMM

; display the menu header for standard mode

    movlw   high string4   ; "OPT EDM Notch Cutter"
    movwf   FSR1H
    movlw   low string4
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done
    goto    skipDMM    

extendedModeDMM:

; display the menu header for extended mode

    movlw   high string13   ; "OPT EDM Extend Reach"
    movwf   FSR1H
    movlw   low string13
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

skipDMM:

; if target depth not already set (=0), display this string

    movlw   LINE2_COL1      ; set display position
    call    writeControl

    call    isTargetZero    ; is target variable zero?
    btfss   STATUS,Z
    goto    skipString5     ; if not zero, jump to display the target

    movlw   high string5  ; "1 - Set Cut Depth"
    movwf   FSR1H
    movlw   low string5
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    goto    skipString6

; if target depth already set (!=0), display the target depth value

skipString5:

    movlw   high string6            ; "1 - Depth = "
    movwf   FSR1H
    movlw   low string6
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done
    
    call    displayTarget           ; display the target depth to cut
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

skipString6:

    movlw   LINE3_COL1      ; set display position
    call    writeControl

    movlw   high string7    ; "2 - Cut Notch"
    movwf   FSR1H
    movlw   low string7
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE4_COL1      ; set display position
    call    writeControl

    movlw   high string8    ; "3 - Jog Electrode"
    movwf   FSR1H
    movlw   low string8
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

    movlw   .03                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    movwf   scratch0

    decfsz  scratch0,F
    goto    skipDMM1

    ; handle option 1 - set cut depth
    
    call    setTarget        ; allow user to adjust the depth value

    goto    doMainMenu      ; repeat main menu

    return 
   
skipDMM1:

    decfsz  scratch0,F
    goto    skipDMM2

    ; handle option 2 - Cut Notch

	; flag is ignored because this was a pain - better to be able to zero after a cut - can start a new
	; cut this way without powering down
    bsf     flags,CUT_STARTED ; set flag that cut started so unit cannot be zeroed

    call    cutNotch		; start the auto notch cut function

    goto    doMainMenu      ; repeat main menu
    
skipDMM2:

    decfsz  scratch0,F
    goto   	skipDMM3

    ; handle option 3 - Jog Electrode

    call    jogMode         ; allow user to manually position the height of the cutting blade

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
; Screen displayed:
;
; 0x1, 0x80
; "4 - Cycle Test"
;
; 0x1, 0xc0
; "5 - Motor Dir Normal" or "5 - Motor Dir Reverse"
;
; 0x1, 0x94
; "6 - Erosion None" or "6 - Erosion 17%"
;
; 0x1, 0x80
; Carriage Return (to place cursor on first option)
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

    movlw   high string21   ; "4 - " prefix so can re-use "Cycle Test" string
    movwf   FSR1H
    movlw   low string21
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done
    
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

    btfsc  	flags,MOTOR_DIR_MODE    ; check for reverse motor direction
	goto	revDirDMMP2

    movlw   high string22   ; add "Normal" suffix to motor dir line
    movwf   FSR1H
    movlw   low string22
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    goto    option3DMMP2

revDirDMMP2:

    movlw   high string23   ; add "Reverse" suffix to motor dir line
    movwf   FSR1H
    movlw   low string23
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

; display the third option

option3DMMP2:

    movlw   LINE3_COL1      ; set display position
    call    writeControl

    movlw   high string24   ; "6 - Erosion "
    movwf   FSR1H
    movlw   low string24
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    btfsc  	flags3,EROSION_MODE    ; check erosion mode
	goto	useErosionDMMP2

    movlw   high string26           ; add "None" suffix to erosion line
    movwf   FSR1H
    movlw   low string26
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    goto    option4DMMP2

useErosionDMMP2:

    movlw   high string27   ; add "17%" suffix to erosion line
    movwf   FSR1H
    movlw   low string27
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done
    
; display the fourth option

option4DMMP2:

    movlw   LINE4_COL1      ; set display position
    call    writeControl

    movlw   high string25           ; "7 - Alarm "
    movwf   FSR1H
    movlw   low string25
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    btfsc  	flags3,ALARM_ENABLED    ; check mode flag
	goto	alarmDisabledDMMP2

    movlw   high string28           ; add "Off" suffix
    movwf   FSR1H
    movlw   low string28
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    goto    placeCursorDMMP2

alarmDisabledDMMP2:

    movlw   high string29           ; add "On"
    movwf   FSR1H
    movlw   low string29
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

placeCursorDMMP2:
    
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

    movlw   .04                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    movwf   scratch0

    decfsz  scratch0,F
    goto    skipDMMP21

    ; handle option 4 - Cycle Test (menuOption value is 1)
    
    call    cycleTest       ; enter the "Cycle Test" function

    goto    doMainMenuPage2 ; refresh menu
 
skipDMMP21:

    decfsz  scratch0,F
    goto    skipDMMP24

    ; handle option 5 - Motor Direction Change (menuOption value is 2)

    btfss	flags,MOTOR_DIR_MODE    ; motor direction is reverse?
	goto	skipDMMP23

	bcf		flags,MOTOR_DIR_MODE    ; set motor direction to normal
	call    saveFlagsToEEprom       ; save the new setting to EEprom
    goto    doMainMenuPage2         ; refresh menu

skipDMMP23:

	bsf		flags,MOTOR_DIR_MODE	; set motor direction to reverse
    call    saveFlagsToEEprom       ; save the new setting to EEprom
    goto    doMainMenuPage2         ; refresh menu

skipDMMP24:

    decfsz  scratch0,F
    goto    skipDMMP26

    ; handle option 6 - Erosion Change (menuOption value is 3)

    btfss	flags3,EROSION_MODE     ; erosion factor is 17%?
	goto	skipDMMP25

	bcf		flags3,EROSION_MODE     ; set erosion factor to none
    
    movlp   high setStepDistance
    call    setStepDistance
    
    movlp   high saveFlagsToEEprom
    call    saveFlagsToEEprom       ; save the new setting to EEprom
    
    goto    doMainMenuPage2         ; refresh menu

skipDMMP25:

	bsf		flags3,EROSION_MODE     ; set erosion mode to 17%

    movlp   high setStepDistance
    call    setStepDistance
    
    movlp   high saveFlagsToEEprom
    call    saveFlagsToEEprom       ; save the new setting to EEprom

    goto    doMainMenuPage2         ; refresh menu

skipDMMP26:

    decfsz  scratch0,F
    goto    skipDMMP28

    ; handle option 7 - Alarm (menuOption value is 4)

    btfss	flags3,ALARM_ENABLED    ; Alarm on?
	goto	skipDMMP27

	bcf		flags3,ALARM_ENABLED    ; set to off
	call    saveFlagsToEEprom
    goto    doMainMenuPage2         ; refresh menu

skipDMMP27:

	bsf		flags3,ALARM_ENABLED    ; set to on
    call    saveFlagsToEEprom
    goto    doMainMenuPage2         ; refresh menu

skipDMMP28:

	btfss	menuOption,7
	goto	skipDMMP29

	;go back to previous menu with last option defaulted	
    movlw   LINE4_COL1      ; set display position
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
; cutNotch
;
; Automatically cuts the notch.  Moves the cutting blade down until the over-current limit input
; triggers then moves head back up until limit condition is removed.
;
; NOTE: The system monitors the voltage of the cutting supply, NOT the current.  When the voltage
; is high, the current must be low so the blade is lowered.  When the voltage is low, the current
; must be high so the blade is raised.  This makes some of the logic appear to be backwards if
; it is erroneously assumed that the current is being monitored instead of the voltage.
;
; The user can adjust the agressiveness of the advance using the "Jog Up" and "Jog Down" buttons.
; This affects the time between sparks to trigger an advance.
;
; NOTE: If the cutting power supply is turned off, the head will be raised due to the fact that
;       the voltage appears to be low.
;
; On entry:
;
; position contains current height of the cutting blade
; depth contains the desired depth of the cut
;
; For notch cutting mode switch out the smart code on retract.
;
; For wall reduction cutting mode, switch in the smart code on retract.
; A good value for the overCurrentTimer retract is 1fff (hard coded).
;
; This function does not disable switch debouncing like the jogMode function. Since the Up/Down
; switch is used to adjust values during the cut, it must be debounced. It does enable the
; TIME_CRITICAL flag to prevent data transmissions from waiting until completion so that they
; occur in the background.
;

cutNotch:
    
    call    waitUntilIORefresh    ; make sure all buttons released to avoid errant response
    
    banksel flags3
    bsf     flags3,TIME_CRITICAL ; configure functions to minimize waits and delays
        
    call    setupLCDBlockPkt    ; prepare block data packet for LCD
    
    call    clearScreen     ; clear the LCD screen

    movlw   high string14   ; "Turn on Cut Voltage"
    movwf   FSR1H
    movlw   low string14
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    call    setupCutNotchAndCycleTest	; finish the screen setup

    bcf     flags,AT_DEPTH  ; clear the depth reached flag

    movlw   ' '				; these variables store the direction symbol (an asterisk)
    movwf   scratch7		; for display to show which direction the head is going
    movlw   ' '				; clear them so garbage won't be displayed first time through
    movwf   scratch8

    call    handlePowerSupplyOnOff  ; turn cutting current power supply on/off per switch setting

    movlw   .255            ; delay to allow the power supply to stabilize
    call    msDelay
    movlw   .255
    call    msDelay
    
cutLoop:

    call    handleFastIO            ; set switch flags, display depth, and send output states

    btfss   switchStates,SELECT_SW_FLAG
    goto    exitCN                  ; exit the notch cutting mode if the Select button pressed

    btfsc   flags,AT_DEPTH          ; displayPosLUCL sets flags:AT_DEPTH if target reached
    goto    exitCN

checkUPDWNButtons:
    
    btfss   switchStates,JOG_UP_SW_FLAG
    call    adjustSpeedOrPowerUp   ; increment the speed (sparkLevel) value or Power Level

    btfss   switchStates,JOG_DOWN_SW_FLAG
    call    adjustSpeedOrPowerDown ; decrement the speed (sparkLevel) value or Power Level

checkHiLimit:

    call    sparkTimer
    btfss   STATUS,Z
    goto    checkLoLimit

moveDownLUCL:

; voltage too high (current too low) - move cutting blade down

    call    incDepth        ; going down increments the position by one step distance

    movlw   ' '
    movwf   scratch7
    movlw   '*'
    movwf   scratch8        ; display asterisk by "Down" label

    call    pulseMotorDownWithDelay  	; move motor one step
                                 		; no delay before stepping because enough time wasted above

checkLoLimit:
 
    btfsc   flags,WALL_MODE         ; no retract smart code for notch, use it for Wall (see header notes)
    goto    wallModeCN

notchModeCN:                        ; next two lines for notch mode
    
    btfsc   LO_LIMIT_P,LO_LIMIT     ; is voltage too low? (current too high)
    goto    cutLoop
    goto    moveUpLUCL              ; time to retract

wallModeCN:                         ; next two lines for wall reduction mode
   
    call    overCurrentTimer
    btfss   STATUS,Z
    goto    cutLoop

moveUpLUCL:

; voltage too low (current too high) - move cutting blade up

    call    pulseMotorUpWithDelay  	; move motor one step - delay to allow motor to move

    call    decDepth				; going up decrements the position by one step distance

    movlw   '*'
    movwf   scratch7                ; display asterisk by "Up" label
    movlw   ' '
    movwf   scratch8

; enter a fast loop without much overhead to retract quickly until over-current is gone

quickRetractCN:

    call    handleFastIO            ; set switch flags, display depth, and send output states

    btfss   switchStates,SELECT_SW_FLAG
    goto    exitCN                  ; exit the notch cutting mode if the Select button pressed

    call    pulseMotorWithDelay    	; move motor one step - delay to allow motor to move
    
    call    decDepth				; going up decrements the position by one step distance

    banksel LO_LIMIT_P

    btfss   LO_LIMIT_P,LO_LIMIT     ; check again, loop quickly until current is within
    goto    quickRetractCN          ; limits to avoid glow plugging
    
    goto    cutLoop

displayPosLUCL:             		; updates the display if it is time to do so

    banksel serialXmtBufNumBytes    ; update the display if serial transmit not in progress
    movf    serialXmtBufNumBytes,W
    btfss   STATUS,Z
    goto    checkPositionCN 		; don't display if print buffer not ready      

    banksel flags

    movlw   0x10                    ; flip the send packet type selector bit
    xorwf   flags2,F
        
    bcf     flags,UPDATE_DISPLAY 	; clear flag so no update until data changed again

    ; display asterisk at "Up" or "Down" label depending on blade direction or erase if no movement

    call    setupLCDBlockPkt    ; prepare block data packet for LCD

    movlw   LINE2_COL1      ; set display position
    call    writeControl
    movf    scratch7,W
    call    writeChar       ; write asterisk or space by "Up" label
    movlw   LINE3_COL1      ; set display position
    call    writeControl
    movf    scratch8,W
    call    writeChar       ; write asterisk or space by "Down" label

    call    displaySpeedAndPower    ; display the current advance speed and power level

    movlw   LINE4_COL12     ; set display position
    call    writeControl
    call    displayPos      ; display the location of the head relative to the zero point
    movlp   high startSerialPortTransmit
    call    startSerialPortTransmit ; force buffer to print, don't wait due to time criticality

    movlp   high checkPositionCN

checkPositionCN:        ; compare position with desired cut depth, exit when reached

    call    isDepthGreaterThanOrEquToTarget
    banksel flags
    bcf     flags,AT_DEPTH
    btfsc   STATUS,C
    bsf     flags,AT_DEPTH  ; if isDepthGreaterThanTarget returned C=1, depth reached so set flag

    return                  ; NOTE: this returns from displayPosLUCL, NOT cutNotch! 

exitCN:

    banksel POWER_ON_L
    bcf     POWER_ON_L,POWER_ON    ; turn off the cutting voltage

    banksel flags
    bcf     flags3,TIME_CRITICAL    ; configure functions for normal waits and delays
        
    call    waitXmtPrep            ; wait until buffer printed

    banksel flags
    btfsc   flags,DATA_MODIFIED     ; if data has been modified, save to eeprom
    call    saveSparkLevelsToEEprom

    bcf     flags,DATA_MODIFIED

    return

; end of cutNotch
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handlePowerSupplyOnOff
;
; Turns the Cutting Current power supply on or off depending on the state of the Electrode switch.
;

handlePowerSupplyOnOff:
    
    btfss   switchStates,ELECTRODE_PWR_SW_FLAG
    goto    psIsOff
    
    banksel POWER_ON_L
    bsf     POWER_ON_L,POWER_ON    ; turn on the cutting voltage

    banksel flags
    return
    
psIsOff:    

    banksel POWER_ON_L
    bcf     POWER_ON_L,POWER_ON    ; turn on the cutting voltage

    banksel flags
    return
        
; end of handlePowerSupplyOnOff
;--------------------------------------------------------------------------------------------------
        
;--------------------------------------------------------------------------------------------------
; setupCutNotchAndCycleTest
;
; Prepares the screen for the cutNotch and cycleTest functions.  These share nearly identical
; screens.
;

setupCutNotchAndCycleTest:

    movlw   LINE2_COL2      ; set display position
    call    writeControl

    movlw   high string15   ; "Up Speed>"
    movwf   FSR1H
    movlw   low string15
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    call    displaySpeedAndPower    ; display the current advance speed and power level
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    movlw   LINE3_COL2          ; set display position
    call    writeControl

    movlw   high string16   ; "Down  Stop>"
    movwf   FSR1H
    movlw   low string16
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    call    displayTarget   ; display the target depth to cut

    movlw   0x22
    call    writeChar       ; write '"' for inch mark

    movlw   LINE4_COL18     ; set display position
    call    writeControl
    movlw   0x22
    call    writeChar       ; write '"' for inch mark

    movlw   LINE4_COL12     ; set display position
    call    writeControl
    call    displayPos      ; display the location of the head relative to the zero point

    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    return

; end of setupCutNotchAndCycleTest
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; cycleTest
;
; Performs a test to verify the proper operation of the system.  The head is lowered until the
; high current limit is inactive - this occurs when the blade nearly touches the pipe and
; current begins to flow.  The head is then retracted back to the original position.  This cycle
; is repeated until the user exits the test.  With each touchdown, only a tiny cut is made so the
; depth should only change very slowly after many touchdowns.
;
; For the best test, the operator should use the jog mode to raise the head as high as possible
; so that the maximum range is used in each cycle.
;
; NOTE: The system monitors the voltage of the cutting supply, NOT the current.  When the voltage
; is high, the current must be low so the blade is lowered.  When the voltage is low, the current
; must be high so the blade is raised.  This makes some of the logic appear to be backwards if
; it is erroneously assumed that the current is being monitored instead of the voltage.
;
; Although the target depth to cut and the cutting speed labels are displayed, they are not
; used by this function.
;
; NOTE: If the cutting power supply is turned off, the head will be raised due to the fact that
;       the voltage appears to be low.
;
; This function does not disable switch debouncing like the jogMode function as it does not
; monitor the inputs. It does enable the TIME_CRITICAL flag to prevent data transmissions from
; waiting until completion so that they occur in the background.
;
; On entry:
;
; On exit:
;

cycleTest:

    call    waitUntilIORefresh    ; make sure all buttons released to avoid errant response
    
    banksel flags3
    bsf     flags3,TIME_CRITICAL ; configure functions to minimize waits and delays
    
    call    setupLCDBlockPkt    ; prepare block data packet for LCD    

    call    clearScreen     ; clear the LCD screen

    movlw   high string19   ; "Cycle Test"
    movwf   FSR1H
    movlw   low string19
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

	call	setupCutNotchAndCycleTest	; finish the screen setup

    movlw   ' '				; these variables store the direction symbol (an asterisk)
    movwf   scratch7		; for display to show which direction the head is going
    movlw   ' '				; clear them so garbage won't be displayed first time through
    movwf   scratch8

    call    handlePowerSupplyOnOff  ; turn cutting current power supply on/off per switch setting

    movlw   .255            ; delay to allow the power supply to stabilize
    call    msDelay
    movlw   .255
    call    msDelay

restartCycleCT:

    movlw   .0				; clear the cycle distance counter to track distance lowered
    movwf   cycleTestRetract0
	movwf   cycleTestRetract1

cycleLoopCT:

    call    handleFastIO            ; set switch flags, display depth, and send output states

    btfss   switchStates,SELECT_SW_FLAG
    goto    exitCT          ; exit the notch cutting mode if the Select button pressed
            
checkHiLimitCT:

    btfsc   HI_LIMIT_P,HI_LIMIT     ; is voltage too high? (current too low, not touching yet)
    goto    upCycleCT             	; voltage drop - blade touching - begin up cycle
									; unlike the cutNotch function, the blade never sits still

moveDownCT:

; voltage too high (current too low, not touching yet) - move cutting blade down

	;count how many ticks the blade travels down before touchdown - on the up cycle this
	;count will be used to stop the blade in approximately the original start position
	;each time - as a cut is made, the up and down positions will move down slowly
    
	incf	cycleTestRetract0,F	; increment low byte ~ see note "incf vs decf rollover"
    btfsc   STATUS,Z
	incf	cycleTestRetract1,F	; increment high byte

    call    incDepth            ; going down increments the position by one step distance

    movlw   ' '
    movwf   scratch7
    movlw   '*'
    movwf   scratch8        ; display asterisk by "Down" label

    call    pulseMotorDownWithDelay	; move motor one step
                                 	; no delay before stepping because enough time wasted above

	goto	cycleLoopCT

upCycleCT:

; voltage too low (current too high - blade touching) - move cutting blade up

    movlw   '*'
    movwf   scratch7                ; display asterisk by "Up" label
    movlw   ' '
    movwf   scratch8

quickRetractCT:

    call    handleFastIO            ; set switch flags, display depth, and send output states
            
    btfss   switchStates,SELECT_SW_FLAG
    goto    exitCT                  ; exit the notch cutting mode if the Select button pressed

    call    pulseMotorUpWithDelay  	; move motor up one step - delay to allow motor to move

    call    decDepth				; going up decrements the position by one step distance

    banksel cycleTestRetract0

	; user counter to return blade to original start position

    movlw   .1                      ; decrement LSByte
    subwf   cycleTestRetract0,F		; see note "incf vs decf rollover"
    btfss   STATUS,C                ; did LSByte roll under (0->255)?
	decf	cycleTestRetract1,F		; decrement MSByte after LSByte roll under
	movf	cycleTestRetract0,W		; check MSB:LSB for zero
	iorwf	cycleTestRetract1,W
	btfsc	STATUS,Z
	goto    restartCycleCT	; restart loop when counter reaches zero

    goto    quickRetractCT
	
exitCT:

    banksel POWER_ON_L
    bcf     POWER_ON_L,POWER_ON    ; turn off the cutting voltage

    banksel flags
    bcf     flags3,TIME_CRITICAL    ; configure functions for normal waits and delays
        
    call    waitXmtPrep            ; wait until buffer printed

    return

; end of cycleTest
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleFastIO
;
; Handles setting of switch input flags, sending of depth display to LCD PIC and sending of output
; states to the Switch PIC, alternating between each type of send packet and limiting the number
; of sends to decrease overhead.
;
; Since this function makes a call to sendOutputStatesIfReadyQ (which skips the send rate limiting
; controls), it calls processIOQ to skip that function's call to sendOutputStatesIfReady which
; is an unnecessary time waste as the output states would be sent extra times.
;
; Used by cutNotch and cycleTest.
;

handleFastIO:
    
    call    processIOQ              ; check local & remote switch states

    call    handlePowerSupplyOnOff  ; turn cutting current power supply on/off per switch setting
        
    btfsc   flags2,PACKET_SEND_TYPE
    goto    sendOutputStatesHFIO
    
    call    displayPosLUCL
    return
   
sendOutputStatesHFIO:    
    
    call    sendOutputStatesIfReadyQ
    return
    
; end of handleFastIO
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sparkTimer
;
; Tracks time between sparking of the blade.  Uses high limit comparator input - if this signal
; is 1, then voltage and current are good - if 0 then the blade needs to move down.  If the blade
; moves down every time this signal goes to 0, it advances too quickly - it should only advance
; when no spark has occurred for a specified time.
;
; The timer counts down and is reset any time the voltage/current are good.  If the timer makes
; it to zero without being reset, then an advance is required.
;
; Returns Z = 0 if not time to advance, Z = 1 if time to advance (lower blade).
;
; A smaller value makes the advance more aggressive.
;
; The forced 1 value in the lower nibble of sparkLevel and in the lower byte of the counter word
; are required to avoid 0 values as decfsz is used for which a starting value of 0 would result in
; 255 loops.
;

sparkTimer:

    btfsc   HI_LIMIT_P,HI_LIMIT     ; is voltage too high? (current too low)
    goto    noAdvanceST             ; voltage is good, don't advance

; check the advance timer

    bcf     STATUS,Z        ; preset flag - decfsz does not affect Z

    decfsz  sparkTimer0,F   ; count down
    return                  ; returns Z = 0, no advance due to not timed out
    decfsz  sparkTimer1,F
    return                  ; returns Z = 0, no advance due to not timed out

    movlw   0x01            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer0
    movf    sparkLevel,W    ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer1
   
    bsf     STATUS,Z        ; return Z = 1, blade advance required

    return

noAdvanceST:

    ; voltage is good, so reset timer

    movlw   0x01            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer0
    movf    sparkLevel,W    ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer1

    bcf     STATUS,Z        ; return Z = 0, no blade advance

    return

; end of sparkTimer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; overCurrentTimer
;
; Tracks time between over-current spikes.  Uses low limit comparator input - if this signal
; is 0, then voltage and current are good - if 1 then the blade needs to up .  If the blade
; moves up every time this signal goes to 1, it rises too quickly - it should only retract
; when an overvoltage has occurred for a specified time.
;
; The timer counts down and is reset any time the voltage/current are good.  If the timer makes
; it to zero without being reset, then a retract is required.
;
; Returns Z = 0 if not time to retract, Z = 1 if time to retract (raise blade).
;
; A smaller value makes the retract more responsive - will retract more often.
;
; Further explanation:
; The system has to be in overcurrent for the entire duration of the timer to force a retract.  This
; prevents retracts on short spikes of overcurrent.
;

overCurrentTimer:

    btfsc   LO_LIMIT_P,LO_LIMIT     ; is voltage too low? (current too high)
    goto    noRetractOCT            ; voltage is good, don't retract

; check the retract timer

    bcf     STATUS,Z        ; preset flag - decfsz does not affect Z

    decfsz  overCurrentTimer0,F	; count down
    return             		    ; returns Z = 0, no retract due to not timed out
    decfsz  overCurrentTimer1,F
    return                  	; returns Z = 0, no retract due to not timed out

    movlw   0xff            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   overCurrentTimer0
    movlw   0x1f            ; NOTE - don't use less than 0x01 (0 = 0xff)
                            ; ver 7.6b = 0
    movwf   overCurrentTimer1
   
    bsf     STATUS,Z        ; return Z = 1, blade retract required

    return

noRetractOCT:

    ; voltage is good, so reset timer

    movlw   0xff            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   overCurrentTimer0
    movlw   0x1f            ; NOTE - don't use less than 0x01 (0 = 0xff)
                            ; ver 7.6b = 0
    movwf   overCurrentTimer1

    bcf     STATUS,Z        ; return Z = 0, no blade advance

    return

; end of overCurrentTimer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; adjustSpeedOrPowerUp
;
; If Mode switch is in "Setup" position, jumps to adjustSpeedUp.
; If switch is in "Normal" position, jumps to adjustPowerUp.
;

adjustSpeedOrPowerUp:

    btfss   switchStates,MODE_SW_FLAG  ; in Setup mode?
    goto    adjustSpeedUp              ; adjust Speed setting if in "Setup" mode
    goto    adjustPowerUp              ; adjust Power Level if in "Normal" mode

; end of adjustSpeedOrPowerUp
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; adjustSpeedOrPowerDown
;
; If Mode switch is in "Setup" position, jumps to adjustSpeedDown.
; If switch is in "Normal" position, jumps to adjustPowerDown.
;

adjustSpeedOrPowerDown:

    btfss   switchStates,MODE_SW_FLAG   ; in Setup mode?
    goto    adjustSpeedDown             ; adjust Speed setting if in "Setup" mode
    goto    adjustPowerDown             ; adjust Power Level if in "Normal" mode

; end of adjustSpeedOrPowerDown
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; adjustSpeedUp and adjustSpeedDown
;
; Call adjustSpeedUp if jog up button toggled, adjustSpeedDown if jog down button toggled.
;
; If jog up then speed is increased by one, rolling from 9 to 1 if appropriate.
; If jog down then speed is decreased by one, rolling from 1 to 9 if appropriate.
;
; speedValue range is 1-9 which is converted to sparkLevel range of 0x01-0x11
;
; Thus, speedValue of 1 gives sparkLevel of 0x01; 2 gives 0x03; 3 gives 0x05, etc.
;
; In actual use, sparkLevel is then used as the upper byte of a counter word:
;
;   sparkLevel:0x01
;
; The forced 1 value in the lower nibble of sparkLevel and in the lower byte of the counter word
; are required to avoid 0 values as decfsz is used for which a starting value of 0 would result in
; 255 loops.
;

adjustSpeedUp:

; jog up button press    

    incf    speedValue,F    ; increment the value
    
    movlw   .10
    subwf   speedValue,W    ; check if 10 reached
    btfss   STATUS,Z
    goto    updateAS        ; display the digit

    movlw   .1
    movwf   speedValue      ; roll around to 1 after 9

    goto    updateAS        ; display the digit

adjustSpeedDown:

; jog down button press

    decf    speedValue,F    ; decrement the value
    
    movlw   .0
    subwf   speedValue,W    ; check if less than 1
    btfss   STATUS,Z
    goto    updateAS        ; display the digit

    movlw   .9
    movwf   speedValue      ; roll around to 9 after 1

updateAS:

; update and set dirty flag so values will be saved to eeprom later
; set display flag to trigger cutNotch function to update the values on the display

    btfsc   flags,WALL_MODE ; Notch or Wall mode?
    goto    wallModeAS

    movlw   high sparkLevelNotch ; transfer value to Notch variable
    movwf   FSR0H
    movlw   low sparkLevelNotch
    movwf   FSR0L

    goto    processValueAS

wallModeAS:

    movlw   high sparkLevelWall  ; transfer value to Wall variable
    movwf   FSR0H
    movlw   low sparkLevelWall
    movwf   FSR0L

processValueAS:

    ; convert speedValue from 1-9 to 0x01-0x11 and store in sparkLevel and Notch or Wall
    ; variable (pointed by FSR0) -- see notes in function header for details

    movlw   0x01
    subwf   speedValue,W
    movwf   sparkLevel
    rlf     sparkLevel,F    ; shift left 1 bit
    movf    sparkLevel,W    ; get the rotated value
    andlw   0xfe            ; mask off lower bit(s)
    addlw   0x01            ; force value to be at least value of 1
    movwf   sparkLevel      ; store value in sparkLevel
    movwf   INDF0           ; store value in appropriate variable
 
    bsf     flags,DATA_MODIFIED     ; set flag so values will be saved
    bsf     flags,UPDATE_DISPLAY    ; set flag so display will be updated

    return

; end of adjustSpeedUp and adjustSpeedDown
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; adjustPowerUp and adjustPowerDown
;
; Call adjustPowerUp if jog up button toggled, adjustPowerDown if jog down button toggled.
;
; If jog up then speed is increased by one, rolling from 5 to 1 if appropriate.
; If jog down then speed is decreased by one, rolling from 1 to 5 if appropriate.
;
; Power level is range of 1-5 which is converted to a value for hiCurrentLimitPot.
; Value of loCurrentLimitPot is then set to CURRENT_LIMIT_DIFFERENCE lower than hiCurrentLimitPot.
;
; The Power Level is converted to a value for hiCurrentLimitPot using this formula:
;
;   hiCurrentLimitPot = CURRENT_LIMIT_POT_OFFSET + (powerLevel * CURRENT_LIMIT_POT_INCREMENT)
;   loCurrentLimitPot = hiCurrentLimitPot - CURRENT_LIMIT_DIFFERENCE
;

adjustPowerUp:

; jog up button press

    incf    powerLevel,F    ; increment the value

    movlw   .6
    subwf   powerLevel,W    ; check if upper limit reached
    btfss   STATUS,Z
    goto    updateAP        ; display the digit

    movlw   .1
    movwf   powerLevel      ; roll around if limit reached

    goto    updateAP        ; display the digit

adjustPowerDown:

; jog down button press

    decf    powerLevel,F    ; decrement the value

    movlw   .0
    subwf   powerLevel,W    ; check if less than lower limit
    btfss   STATUS,Z
    goto    updateAP        ; display the digit

    movlw   .5
    movwf   powerLevel      ; roll around if limit reached

updateAP:

    ; convert Power Level to values for the digital pots
    ; see notes in function header for details

    movlw   CURRENT_LIMIT_POT_OFFSET
    movwf   hiCurrentLimitPot

    movf    powerLevel,W
    movwf   scratch0

apuLoop1:

    movlw   CURRENT_LIMIT_POT_INCREMENT     ; add an increment for each count of Power Level
    addwf   hiCurrentLimitPot,F

    decfsz  scratch0,F
    goto    apuLoop1

apuExit:

    movlw   CURRENT_LIMIT_DIFFERENCE        ; calculate low setting from the high setting
    subwf   hiCurrentLimitPot,W
    movwf   loCurrentLimitPot

    call    setHighCurrentLimitDigitalPot
    call    setLowCurrentLimitDigitalPot

    banksel flags
    
    bsf     flags,DATA_MODIFIED             ; set flag so values will be saved
    bsf     flags,UPDATE_DISPLAY            ; set flag so display will be updated

    return

; end of adjustPowerUp and adjustPowerDown
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; pulseMotorDownWithDelay
;
; Moves the motor down one click, delaying as necessary for proper motor operation.
;
; Whether the direction bit is set or cleared depends upon the state of the Normal/Reverse
; direction flag.  This option can be set by the user to adjust for systems with different wiring.
;

pulseMotorDownWithDelay:
    
    banksel MOTOR_DIR_L
    bsf     MOTOR_DIR_L,MOTOR_DIR       ; motor down for normal direction option
	
    banksel flags
    btfss	flags,MOTOR_DIR_MODE        ; is motor direction option reverse?
    goto    pMDWD1

    banksel MOTOR_DIR_L
    bcf     MOTOR_DIR_L,MOTOR_DIR       ; motor down for reverse direction option

pMDWD1:

	goto	pulseMotorWithDelay

; end of pulseMotorDownWithDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; pulseMotorUpWithDelay
;
; Moves the motor up one click, delaying as necessary for proper motor operation.
;
; Whether the direction bit is set or cleared depends upon the state of the Normal/Reverse
; direction flag.  This option can be set by the user to adjust for systems with different wiring.
;

pulseMotorUpWithDelay:

    banksel MOTOR_DIR_L
    bcf     MOTOR_DIR_L,MOTOR_DIR       ; motor up for normal direction option
	
    banksel flags
    btfss	flags,MOTOR_DIR_MODE        ; is motor direction option reverse?
    goto    pMUWD1
 
    banksel MOTOR_DIR_L
    bsf     MOTOR_DIR_L,MOTOR_DIR       ; motor up for reverse direction option
	
pMUWD1:

    goto	pulseMotorWithDelay

; end of pulseMotorUpWithDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; pulseMotorWithDelay
;
; Delays and then pulses the motor.  The motor will not respond if there is not enough delay
; between pulses.
;
; For no delay in cases where enough time is wasted between pulses, call pulseMotorNoDelay.
;
; On entry:
;
; Desired motor direction bit set, ie:
;       bcf     MOTOR,DIR_SEL
;   or
;       bsf     MOTOR,DIR_SEL
;
; NOTE: The delay value of .15 worked for both standard and extended heads.  The cut rate
; for the extended head was about 50% too slow compared to desired rate of 0.001 per minute.
; The burn was very consistent and customer wanted to try paralleling the power supplies
; to increase burn rate rather than experiment with program changes.  Originally, this delay
; value was loaded from normDelay.  If it is decided that each head needs a different value,
; a new variable needs to be created (ex: cutDelay) and loaded specifically for each head type
; because normDelay and setupDelay are already used to control the speeds in jog mode.  The values
; for jog mode may not work for cut mode because of the difference in the code exec time.
;

pulseMotorWithDelay:

    banksel scratch1

    movlw   0x0
    movwf   scratch1
    ;movf    normDelay,W
    movlw   .15             ; see notes in header regarding this value (use .15 for normal head)
    call    bigDelayA

    banksel MOTOR_STEP_L
    bcf     MOTOR_STEP_L,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_L,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    banksel flags

    return

pulseMotorNoDelay:

    banksel MOTOR_STEP_L
    bcf     MOTOR_STEP_L,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_L,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    banksel flags

    return

; end of pulseMotorAndDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setTarget
;
; This function allows the user to set the depth of the cut.
; The most significant digit is not edited, so it is always zero. The next four digits are
; editable. The editable/ignored digits are:
;       nEEEEnnnnnn
;   where n represents non-editable digits while E represents editable digits.
; 

setTarget:

; set up the display

    call    setupLCDBlockPkt    ; prepare block data packet for LCD    
    
    call    clearScreen     ; clear the LCD screen

    movlw   high string9   ; "   Set Cut Depth"
    movwf   FSR1H
    movlw   low string9
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE2_COL5      ; set display position
    call    writeControl
    movlw   high string10   ; "0.000 inches"
    movwf   FSR1H
    movlw   low string10
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE3_COL1      ; set display position
    call    writeControl
 
    movlw   high string17   ; load the high of string17 to be used with low of string17 or string18
    movwf   FSR1H
    
    movlw   low string17    ; print "Notch Mode" if in notch mode
    btfsc   flags,WALL_MODE ; which mode?
    movlw   low string18    ; print "Wall Mode" if in wall mode
    
    movwf   FSR1L	    ; put the low of either string17 or string18 -- depends on mode
   
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE2_COL5      ; set display position
    call    writeControl

    call    displayTarget   ; display the value over the "0.000 inches" string so it can be edited

    movlw   LINE2_COL5      ; set display position
    call    writeControl
	call	turnOnBlink
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

; handle editing

    clrf    scratch7        ; track character being modified -- 0 for leftmost character
                            ; 4 characters -- 4 digits, decimal point gets skipped

    movlw   high target9    ; first digit in variable being edited
    movwf   FSR1H
    movlw   low target9          
    movwf   FSR1L
    
    movlw   LINE2_COL5      ; set display position
    movwf   cursorPos

loopSD:

    call    adjustBCDDigit  ; handle editing of the digit

    addfsr  FSR1,1          ; point to the next digit
    incf    scratch7,f      ; track character being modified

    movlw   .4              ; exit if past the last character
    subwf   scratch7,W
    btfsc   STATUS,Z
    goto    endSD

    movlw   .1              ; if on second digit, move cursor again to skip decimal point
    subwf   scratch7,W
    btfsc   STATUS,Z
    incf    cursorPos,F

    incf    cursorPos,F     ; move the cursor to the next digit
    
    movf    cursorPos,W
    call    writeControl
    call    turnOnBlink
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    goto    loopSD

endSD:

    call    saveDepthValueToEEprom  ; save the value stored for depth in the EEProm

    goto    setCutMode

; end of setTarget
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCutMode
;
; This function allows the user to select the Notch or Wall Reduction mode.  The appropriate
; string is displayed on line 3 (Notch or Wall Mode) and the first character is highlighted. If
; the up or down switch is pressed (or toggled), the mode will flip.  Pressing the Reset/Enter
; button will save the mode and exit.
; 

setCutMode:

    movlw   LINE3_COL1      ; set display position
    call    writeControl
    
    movlw   high string17   ; load the high of string17 to be used with low of string17 or string18
    movwf   FSR1H
    
    movlw   low string17    ; print "Notch Mode" if in notch mode
    btfsc   flags,WALL_MODE ; which mode?
    movlw   low string18    ; print "Wall Mode" if in wall mode
    
    movwf   FSR1L	    ; put the low of either string17 or string18 -- depends on mode
   
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE3_COL1      ; set display position
    call    writeControl
	call	turnOnBlink
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

loopSCM:
    
    call    processIO       ; watch for user input

    btfsc   switchStates,JOG_UP_SW_FLAG
    goto    skip_upSCM      ; skip if Up switch not pressed

; jog up button press    

    movlw   b'00001000'     ; flip the Notch/Wall cut mode flag
    xorwf	flags,F

    goto    setCutMode      ; update the display

skip_upSCM:

    btfsc   switchStates,JOG_DOWN_SW_FLAG
    goto    skip_dwnSCM     ; skip if Down switch not pressed

; jog down button press

    movlw   b'00001000'     ; flip the Notch/Wall cut mode flag
    xorwf	flags,F
    
    goto    setCutMode      ; update the display

skip_dwnSCM:

    btfsc   switchStates,SELECT_SW_FLAG
    goto    loopSCM             ; loop without updating display if no button pressed

; set sparkLevel to value for Notch or Wall mode depending on current mode

    movf    sparkLevelNotch,W   ; use Notch mode value if in Wall Reduction mode
    btfsc   flags,WALL_MODE
    movf    sparkLevelWall,W    ; use Wall mode value if in Wall Reduction mode
    movwf   sparkLevel          ; save the selected value

    call    saveFlagsToEEprom   ; save the flags variable to eeprom so that the mode will be
                                ; recalled on power up

    return                  ; return when Reset/Enter/Zero button pressed

; end of setCutMode
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
; adjustBCDDigit
;
; This function allows the user to adjust the value of a BCD digit.
;
; On entry
;
; FSR1 = memory address of the digit
; cursorPos = screen location of the digit
;

adjustBCDDigit:

loopABD:
    
    call    pushFSR1
    call    processIO       ; watch for user input
    call    popFSR1
    
    btfsc   switchStates,JOG_UP_SW_FLAG
    goto    skip_upABD      ; skip if Up switch not pressed

; jog up button press    

    incf    INDF1,F         ; increment the digit
    
    movlw   .10
    subwf   INDF1,W         ; check if 10 reached
    btfss   STATUS,Z
    goto    updateABD       ; display the digit

    movlw   .0
    movwf   INDF1           ; roll around to 0 after 9

    goto    updateABD       ; display the digit

skip_upABD:

    btfsc   switchStates,JOG_DOWN_SW_FLAG
    goto    skip_dwnABD    ; skip if Down switch not pressed

; jog down button press

    decf    INDF1,F         ; decrement the digit
    
    movlw   0xff            
    subwf   INDF1,W         ; check if less than 0
    btfss   STATUS,Z
    goto    updateABD       ; display the digit

    movlw   .9
    movwf   INDF1           ; roll around to 9 after 0

    goto    updateABD       ; display the digit

skip_dwnABD:

    btfsc   switchStates,SELECT_SW_FLAG
    goto    loopABD        ; loop if Reset/Select switch not pressed

; reset/enter/zero button press - digit finished, so exit

    return                  ; return when Reset/Enter/Zero button pressed

updateABD:

; update the character on the screen

    movf    cursorPos,W
    call    writeControl    ; prepare to overwrite the digit
    
    movf    INDF1,W
    addlw   0x30            ; convert to ASCII
    call    writeChar       ; write the digit

    movf    cursorPos,W
    call    writeControl
	call	turnOnBlink

    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next
    
    goto    loopABD         ; continue editing
    
; end of adjustBCDDigit
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; jogMode
;
; This function allows the user to manually adjust the height of the cutting blade using
; the Up/Down buttons.  If the Mode switch is in "Normal" position, the head moves slowly, if in
; "Setup" position, the head moves more quickly.  In "Normal" mode, the Reset/Enter/Zero button
; exits the Jog Mode, in "Setup" mode, the Reset/Enter/Zero button zeroes the displayed position
; of the height.
; 

jogMode:

    movlw   .255                ; delay 255 milliseconds to allow user to release the Select button
    call    msDelay             ;  as this function disables switch debouncing
    call    processIO           ; call to clear out old switch flags from local and remote
    
    banksel flags3
    bsf     flags3,TIME_CRITICAL ; configure functions to minimize waits and delays
                                
; set up the display

    call    setupLCDBlockPkt    ; prepare block data packet for LCD
    
    call    clearScreen     ; clear the LCD screen

    movlw   LINE1_COL7      ; set display position
    call    writeControl
    movlw   high string11   ; "Jog Mode"
    movwf   FSR1H
    movlw   low string11
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE2_COL5      ; set display position
    call    writeControl
    movlw   high string12   ; "Zero or Exit"
    movwf   FSR1H
    movlw   low string12
    movwf   FSR1L
    call    printStringWaitPrep     ; print the string and wait until done

    movlw   LINE4_COL18     ; set display position
    call    writeControl
    movlw   0x22
    call    writeChar       ; write '"' for inch mark

    movlw   LINE4_COL12     ; set display position
    call    writeControl
    call    displayPos      ; display the location of the head relative to the zero point
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    movlw   0x0
    movwf   scratch1
    movlw   0xff
    call    bigDelayA       ; delay - give user chance to release button

    banksel POWER_ON_L
    bsf     POWER_ON_L,POWER_ON    ; turn on the cutting voltage
    
    banksel flags

loopJM:

    bcf     flags3,DEBOUNCE_ACTIVE      ; bypass switch debouncing for better response
    call    processIO                   ; process inputs and outputs
    
    call    handlePowerSupplyOnOff    
    
    movlw   0x0                         ; delay to control motor speed
    movwf   scratch1
    movlw   0x1
    call    bigDelayA

    btfsc   switchStates,JOG_UP_SW_FLAG
    goto    chk_dwnJM       ; skip if Up switch not pressed

; jog up button press    

    banksel MOTOR_DIR_L
    bcf     MOTOR_DIR_L,MOTOR_DIR   ; motor up for normal direction option
	
    banksel flags
	btfss	flags,MOTOR_DIR_MODE    ; is motor direction option reverse?
    goto    jM1

    banksel MOTOR_DIR_L
    bsf     MOTOR_DIR_L,MOTOR_DIR   ; motor up for reverse direction option
    
jM1:

    nop
    nop
    nop
    banksel MOTOR_STEP_L
    bcf     MOTOR_STEP_L,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_L,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    banksel flags

    call    decDepth        ; going up decrements the position by one step distance

    goto    updateJM        ; display the new location

chk_dwnJM:

    btfsc   switchStates,JOG_DOWN_SW_FLAG
    goto    not_dwnJM      ; skip if Down switch not pressed

; jog down button press

    banksel MOTOR_DIR_L
    bsf     MOTOR_DIR_L,MOTOR_DIR   ; motor down for normal direction option
    
    banksel flags
	btfss	flags,MOTOR_DIR_MODE    ; is motor direction option reverse?
    goto    jM2

    banksel MOTOR_DIR_L
    bcf     MOTOR_DIR_L,MOTOR_DIR   ; motor up for reverse direction option

jM2:

    nop
    nop
    nop
    banksel MOTOR_STEP_L
    bcf     MOTOR_STEP_L,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_L,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    banksel flags

    call    incDepth        ; going down increments the position by one step distance

    goto    updateJM        ; display the new location

not_dwnJM:

    btfsc   switchStates,SELECT_SW_FLAG
    goto    loopJM          ; loop if Reset/Select switch not pressed

; handle Select button press

    btfsc   switchStates,MODE_SW_FLAG   ; in Setup mode?
    goto    exitJM                      ;exit Jog Mode if not when Select button pressed

; set the current height as zero

    call    zeroDepth       ; clear the depth position variable

    call    waitXmtPrep     ; wait until buffer printed

    movlw   LINE4_COL12     ; set display position
    call    writeControl
    call    displayPos      ; display the position
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    goto    loopJM          ; stay in jog mode after zeroing

updateJM:

    movlw   0x0             ; delay between each motor step or motor won't respond
    movwf   scratch1
    movf    setupDelay,W
    call    bigDelayA

    btfsc   switchStates,MODE_SW_FLAG   ; in Setup mode?
    goto    noExtraDelayJM      ; extra pause if not in setup speed mode to slow down the head

    movlw   0x0             ; delay extra in normal speed mode
    movwf   scratch1
    movf    normDelay,W
    call    bigDelayA

noExtraDelayJM:

    banksel serialXmtBufNumBytes    ; update the display if serial transmit not in progress
    movf    serialXmtBufNumBytes,W
    btfsc   STATUS,Z
    goto    displayJM

    banksel flags

    goto    loopJM                  ; continue without updating display if print buffer not ready      

displayJM:

    banksel flags                   ; update the display

    call    setupLCDBlockPkt        ; prepare block data packet for LCD    
    
    movlw   LINE4_COL12             ; set display position
    call    writeControl
    call    displayPos      ; display the location of the head relative to the zero point
    movlp   high startSerialPortTransmit
    call    startSerialPortTransmit ; force buffer to print, don't wait due to time criticality
    movlp   high displayJM                                   
                                    
    banksel flags

    goto    loopJM          ; stay in jog mode

exitJM:

    banksel POWER_ON_L
    bcf     POWER_ON_L,POWER_ON     ;  turn off the cutting voltage

    banksel flags
    bcf     flags3,TIME_CRITICAL    ; configure functions for normal waits and delays
        
    call    waitXmtPrep             ; wait until buffer printed

    return

; end of jogMode
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayPos
;
; Displays the current height of the head relative to the zero position.  The variables are in
; unpacked BCD format, the smaller is always subtracted from the bigger and the sign applied
; afterwards.  The resulting difference is displayed on the LCD.
;
; On entry:
;
; Cursor should be positioned at desired print location of the LCD
; Bank should point to flags.
;
; On exit:
;
; Bank will point to flags.
;

displayPos:

        banksel depthSign
        movf    depthSign,F     ; sign of depth position variable
        banksel flags           ; restore bank

        btfss   STATUS,Z
        goto    negativeDP      ; jump if position is negative

; position is positive

        movlw   0x20
        call    writeChar       ; display a space instead of negative sign

        goto    displayDepth

negativeDP:

; position is positive

        movlw   0x2d        ; ASCII '-'
        call    writeChar   ; display a negative sign

        goto    displayDepth

; end of displayPos
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
; Note that the numbering for the lines is screwy - 0x80, 0xc0, 0x94, 0xd4

moveCursorSHO:

    movlw   LINE4_COL1
    subwf   cursorPos,W         ; is cursor at 0xd4?
    btfss   STATUS,Z    
    goto    line2SHO

    movlw   LINE3_COL1          ; move cursor up one line
    movwf   cursorPos
    call    writeControl        ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    return

line2SHO:

    movlw   LINE3_COL1
    subwf   cursorPos,W     ; is cursor at 0x94?
    btfss   STATUS,Z    
    goto    line3SHO

    movlw   LINE2_COL1      ; set display position
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

	return

line3SHO:

    movlw   LINE2_COL1
    subwf   cursorPos,W     ; is cursor at LINE2_COL1?
    btfss   STATUS,Z    
    goto    line4SHO

    movlw   LINE1_COL1          ; move cursor up one line
    movwf   cursorPos
    call    writeControl        ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

	return

line4SHO:                   ; don't move cursor if at the top

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
; Note that the numbering for the lines is screwy - 0x80, 0xc0, 0x94, 0xd4

moveCursorSLO:

    movlw   LINE1_COL1
    subwf   cursorPos,W     ; is cursor at 0x80?
    btfss   STATUS,Z    
    goto    line2SLO

    movlw   LINE2_COL1      ; set display position
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

    return

line2SLO:

    movlw   LINE2_COL1
    subwf   cursorPos,W     ; is cursor at LINE2_COL1?
    btfss   STATUS,Z    
    goto    line3SLO

    movlw   LINE3_COL1          ; move cursor down one line
    movwf   cursorPos
    call    writeControl        ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

	return

line3SLO:

    movlw   LINE3_COL1
    subwf   cursorPos,W     ; is cursor at 0x94?
    btfss   STATUS,Z    
    goto    line4SLO

    movlw   LINE4_COL1          ; move cursor down one line
    movwf   cursorPos
    call    writeControl        ; write the cursor to the LCD
    call    flushXmtWaitPrep    ; force the buffer to print and wait until done then prep for next

	return

line4SLO:                   ; don't move cursor if at the bottom

    return

; end of selectLowerOption
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayDepth
;
; Displays 4 digits of the depth position variable in format x.xxx.
; The most significant digit is not displayed, nor are the least significant digits.
;

displayDepth:

    movlw   high depth9         ; depth10 not displayed
    movwf   FSR1H
    movlw   low depth9
    movwf   FSR1L

    goto    displayBCDVar

; end of displayDepth
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayTarget
;
; Displays 4 digits of the target depth variable in format x.xxx.
; The most significant digit is not displayed, nor are the least significant digits.
;

displayTarget:

    movlw   high target9        ; target10 not displayed
    movwf   FSR1H
    movlw   low target9
    movwf   FSR1L

    goto    displayBCDVar

; end of displayTarget
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayBCDVar
;
; Displays 4 digits of the BCD variable addressed by FSR1.  A decimal point is placed between
; the first and second digits
;
; On entry:
;
; FSR1 contains address of BCD variable.
; Bank points to scratch* variables.
;

displayBCDVar:
    
    moviw   FSR1++
    addlw   0x30            ; convert BCD digit to ASCII
    call    writeChar       ; write the first digit

    movlw   '.'            
    call    writeChar       ; write decimal point

    movlw   .3              
    movwf   scratch4        ; three bytes after the decimal point

loopDBV1:

    moviw   FSR1++
    addlw   0x30            ; convert BCD digit to ASCII
    call    writeChar       ; write the first digit

    decfsz  scratch4,F      ; stop when count is zero
    goto    loopDBV1

    return

; end of displayBCDVar
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displaySpeedAndPower
;
; Displays the current speed value (stored as sparkLevel) and the current power setting. The power
; setting is the cutting current range in use -- represetns the values used for the high and low
; level comparator digital pot values.
;
; The value in sparkLevel is converted to ASCII 1 - 9 and displayed.
; A '-' (dash) separator is displayed to separate the values.
; The value powerLeve is converted to ASCII 1-5 and displayed.
;
; Search for other comments regarding sparkLevel for more info on how its value relates to the
; ASCII values 1-9.
;
; Search for other comments regarding hiCurrentLimitPot and loCurrentLimitPot for more info on how
; their values relate to the ASCII values 1-5.
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;
; On entry:
;
; no presets required
;
; On exit:
;
; WIP NOTE: The speed display value is parsed from the sparkLevel setting while the power display
; is parsed much more simple from the powerLevel variable.
; Couldn't speedLevel be used in a similar manner (and faster) rather than parsing from sparkLevel?
; Would need to set speedLevel when sparkLevel is loaded from eeprom in the beginning.
;

displaySpeedAndPower:

    movlw   LINE2_COL14     ; set display position
    call    writeControl

    ; display the speed value

    ; parse the ASCII number from the sparkLevel value

    movf    sparkLevel,W    ; get the current speed/sparkLevel value
    movwf   speedValue      ; store in variable so we can manipulate it
    rrf     speedValue,F    ; shift value to the right
    movf    speedValue,W    ; get the rotated value
    andlw   0x0f            ; mask off upper nibble
    addlw   0x01            ; shift up one
    movwf   speedValue      ; store speed value for use by other functions

    addlw   0x30            ; convert BCD digit to ASCII
    call    writeChar       ; write to the LCD buffer

    ; write a '-' (dash) separator

    movlw   0x2d            ; write a dash to separate speed from power level
    call    writeChar       ; write to the LCD buffer

    ; display the power value

    movf    powerLevel,W

    addlw   0x30            ; convert BCD digit to ASCII
    call    writeChar       ; write to the LCD buffer

    return

; end of displaySpeedAndPower
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
; msDelay
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

msDelay:

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

; end of msDelay
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; isXgtY    Compare X word with Y word
;
; Entry point isXgtY checks if X > Y
; Entry point isXltY checks if X < Y
;
; If condition is true,  Z = 0 and W = 0xff
; If condition is false, Z = 1 and W = 0x00
;
; On entry:
;
; X word : scratch1:scratch0
; Y word : scratch3:w
;

isXgtY:

    movwf   scratch2        ; scratch2 = W
    movlw   0x1             ; W = 1 (use this later to mask for less than flag)
    goto    L12

isXltY:
    movwf   scratch2        ; scratch2 = w
    movlw   0x4             ; W = 4 (use this later to mask for greater than flag)
    goto    L12

L12:
    movwf   scratch4        ; store flag mask
    movf    scratch3,W
    subwf   scratch1,W      ; W = var21 - scratch0 (compare upper bytes of word)
    btfss   STATUS,Z        ; upper bytes are same, compare lower bytes
    goto    L13
    movf    scratch2,W
    subwf   scratch0,W

; determine if result means x > y, x < y, or x == y -- REMEMBER: Borrow flag is inverse:
;                  0 = borrow, 1 = no borrow

L13:
    movlw   0x4             ; W = 4  preload flag for less than (Borrow (B=0) & not Z flags)
    btfsc   STATUS,C
    movlw   0x1             ; W = 1  replace with greater than (No Borrow (B=1))
    btfsc   STATUS,Z
    movlw   0x2             ; W = 2  replace with equal flag because of Z being set
    
    andwf   scratch4,W      ; compare flag with the mask selected by the entry point
    btfss   STATUS,Z
    movlw   0xff            ; W = 0xff if condition true, 0 if not

    return

; end of isXgtY
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isDepthGreaterThanOrEquToTarget
;
; Compares the top five most significant digits of the current depth against the top five most
; significant digits of the target depth. Note that the depth value can be negative, but the target
; must always be positive.
;
; The bytes in depth and target compared are:
;   EEEEEnnnnnn
; where n represents ignored digits while E represents digits to be compared.
;
; For comparing depth and target, only xx.xxx digits are necessary. The lowest digits are required
; to achieve accuracy while tracking the position of the head, but for checking to see if target 
; reached, only the five digits are needed.
;
; ON ENTRY:
;   no requirements
;
; ON EXIT:
;   C = 0 if Depth < Target 
;   C = 1 if Depth >=  Target
;   

isDepthGreaterThanOrEquToTarget:
    
    bcf     STATUS,C        ; clear for operations below
    
    banksel depth10         ; target and depth should always be in the same bank
    
    movf    depthSign,W     ; load the sign byte (affects Z flag but not C)
    btfss   STATUS,Z        ; if set, depth < target because depth sign is negative
    return                  ; return C=0 because depth < target

    movf    target10,W      ; compare next least significant digits
    subwf   depth10,W
    btfss   STATUS,C
    return                  ; return C=0 because depth <= target
    
    movf    target9,W
    subwf   depth9,W
    btfss   STATUS,C 
    return
    
    movf    target8,W
    subwf   depth8,W
    btfss   STATUS,C
    return
    
    movf    target7,W
    subwf   depth7,W
    btfss   STATUS,C
    return
    
    movf    target6,W
    subwf   depth6,W
    
    return
    
; end of isDepthGreaterThanOrEquToTarget
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; incDepth
;
; Increments the signed unpacked BCD depth position variable by one step distance.
;
; If depth is negative, the condition depth + step <= 0 must always be true in order for the code to
; work. This means that the step value can NEVER change during operation.
;
; On entry:
;
; Bank should point to flags.
;
; On exit:
;
; Bank will point to flags.
;

incDepth:

    banksel depthSign
    movf    depthSign,F
    banksel flags           ; restore bank
    btfss   STATUS,Z        ; check pos/neg
    goto    negativeIBV

; value is positive or 0 (sign is always + for zero), so add to increment

    goto    incDepthAbs     ; add one step distance

negativeIBV:

; value is negative so subtract to increment it

    call    decDepthAbs     ; subtract one step distance

    call    isDepthZero     ; check for zero
    
    btfss   STATUS,Z        
    return                  ; value is not zero

    banksel depthSign       ; set sign positive (zero value should always have positive sign)
    clrf    depthSign
    banksel flags           ; restore bank

    return

; end of incDepth
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; decDepth
;
; Decrements the signed unpacked BCD depth position variable by one step distance.
;
; If depth is positive, the condition depth - step >= 0 must always be true in order for the code to
; work. This means that the step value can NEVER change during operation.
;
; On entry:
;
; Bank should point to flags.
;
; On exit:
;
; Bank will point to flags.
;

decDepth:

    call    isDepthZero     ; check if zero
    btfsc   STATUS,Z
    goto    negativeDBV     ; if it is zero, add to decrement

    banksel depthSign
    movf    depthSign,F
    banksel flags           ; restore bank
    btfss   STATUS,Z        ; check pos/neg
    goto    negativeDBV     ; if depth is negative, add to decrement
    
; value is positive, so subtract to decrement

    goto    decDepthAbs     ; subtract one step distance from depth position

negativeDBV:

; value is negative or zero so add to decrement
; since value is negative or zero, decrementing will always result in negative

    banksel depthSign
    movlw   0x01
    movwf   depthSign       ; set sign negative
    banksel flags           ; restore bank

    call    incDepthAbs     ; add one step distance to depth position

    return

; end of decDepth
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; incDepthAbs
;
; Adds one step distance to depth position BCD variable ignoring its sign.
;
; On entry:
;

incDepthAbs:

    movlw   high depth10
    movwf   FSR0H
    movlw   low depth10
    movwf   FSR0L

    movlw   high step10
    movwf   FSR1H
    movlw   low step10
    movwf   FSR1L

    movlw   .11             ; 11 digits in the operands

    goto    addBCDVars
    
; end of incDepthAbs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; decDepthAbs
;
; Subtracts one step distance from depth position BCD variable ignoring its sign.
;
; On entry:
;

decDepthAbs:

    movlw   high depth10
    movwf   FSR0H
    movlw   low depth10
    movwf   FSR0L

    movlw   high step10
    movwf   FSR1H
    movlw   low step10
    movwf   FSR1L

    movlw   .11             ; 11 digits in the operands

    goto    subtractBCDVars
    
; end of decDepthAbs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; addBCDVars
;
; Adds two unpacked BCD variables together, ignoring sign.
; Carry from most significant digit will be ignored.
;
; On entry:
;
; W contains the number of digits in the unpacked BCD variables
; FSR0 points to MSB of operand1/destination variable
; FSR1 points to MSB of operand2/variable
;

addBCDVars:

    banksel scratch1

    movwf   scratch1        ; use scratch variable as loop counter

    decf    WREG,F          ; adjust for use to move to digit 0
    
    addwf   FSR0L,F         ; point to digit 0 of operand 1
    btfsc   STATUS,C
    incf    FSR0H,F
    
    addwf   FSR1L,F         ; point to digit 0 of operand 2
    btfsc   STATUS,C
    incf    FSR1H,F

    bcf     STATUS,C        ; clear the carry bit for the first addition

aB6BV1Loop1:

    moviw   FSR1--          ; add digit of the two operands
    addwfc  INDF0,F
    movlw   .10
    subwf   INDF0,W         ; compare with 10 (W = f - W)
    btfss   STATUS,C        
    goto    aB6BV1          ; C = 0 if borrow ~ digit < 10

    movlw   .10             ; adjust digit to valid BCD
    subwf   INDF0,F         ; (F = f - W) (this will always leave Carry bit set to carry over)

aB6BV1:

    addfsr   FSR0,-.1       ; point to next digit

    decfsz  scratch1,F
    goto    aB6BV1Loop1

    ; any carry from most significant digit will be ignored

    return

; end of addBCDVars
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; subtractBCDVars
;
; Subtracts two unpacked BCD variables, ignoring sign.
; Borrow to most significant digit will be ignored.
;
; On entry:
;
; W contains the number of digits in the unpacked BCD variables
; FSR0 points to MSB of operand1/destination variable
; FSR1 points to MSB of operand2/variable
; destination(operand1) = operand1 - operand2
;

subtractBCDVars:

    banksel scratch1

    movwf   scratch1        ; use scratch variable as loop counter
    
    decf    WREG,F          ; adjust for use to move to digit 0
    addwf   FSR0L,F         ; point to digit 0 of operand 1
    btfsc   STATUS,C
    incf    FSR0H,F
    addwf   FSR1L,F         ; point to digit 0 of operand 2
    btfsc   STATUS,C
    incf    FSR1H,F
 
    bsf     STATUS,C        ; set the carry/borrow bit for the first subtraction (no borrow)

sB6BV1Loop1:

    moviw   FSR1--          ; subtract digit of the two operands
    subwfb  INDF0,F
    btfss   INDF0,7         ; if bit 7 set, result is negative and requires adjustment
    goto    sB6BV1          ; positive result needs no adjustment

    movlw   .10             ; adjust digit to valid BCD
    addwf   INDF0,F
    bcf     STATUS,C        ; clear carry/borrow so it will borrow on the next subtraction
                            ;    (the inverse of the flag is used for borrowing)
sB6BV1:

    addfsr   FSR0,-.1       ; point to next digit

    decfsz  scratch1,F
    goto    sB6BV1Loop1

    ; any borrow from most significant digit will be ignored

    return

; end of subtractBCDVars
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isDepthZero
;
; Checks if the depth position variable is zero.
;
; On entry:
;
; On return, Z = 1 if variable is zero.
;

isDepthZero:

    movlw   high depth10    ; point to depth position variable
    movwf   FSR0H
    movlw   low depth10
    movwf   FSR0L

    goto    isZero

; end of isDepthZero
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isTargetZero
;
; Checks if the target depth position variable is zero.
;
; On entry:
;
; On return:
;
;   Z = 1 if variable is zero.
;

isTargetZero:

    movlw   high target10    ; point to target depth position variable
    movwf   FSR0H
    movlw   low target10
    movwf   FSR0L

    goto    isZero

; end of isTargetZero
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isZero
;
; Checks if the variable is zero.
;
; On entry:
;
; FSR0 points to first byte of variable.
;
; On return, Z = 1 if variable is zero.
;

isZero:

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 10 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 9 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 8 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 7 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 6 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 5 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 4 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 3 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 2 for zero
    return

    moviw   FSR0++
    btfss   STATUS,Z        ; check digit 1 for zero
    return

    moviw   FSR0++          ; check digit 0 for zero

    return

; end of isZero
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
; writeToEeprom
;
; Writes block of bytes to EEprom.
;
; Address in EEprom to store first byte should be in eepromAddressH/L.
; Number of bytes to write should be in eepromCount.
; Indirect register FSR0 should point to first byte in RAM to be written.
; The block of bytes will be copied from RAM to EEprom.
;

writeToEEprom:

loopWTE1:

    moviw   FSR0++              ; store byte to be written in scratch0
    banksel scratch0
    movwf   scratch0

    call    writeByteToEEprom1ViaI2C

    banksel eepromAddressL
	incf	eepromAddressL,F    ; move to next address in EEprom
    btfsc   STATUS,Z
	incf	eepromAddressH,F	; increment high byte on low byte rollover

    call    waitForEEprom1WriteCycleFinished

    banksel eepromCount
	decfsz	eepromCount,F       ; count down number of bytes transferred
	goto	loopWTE1            ; not zero yet - trasfer more bytes

	return

; end of writeToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setDigitalPotInChip1
;
; Sets the pot specified by scratch0 in digital pot chip 1 to the value in scratch1.
;

setDigitalPotInChip1:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   DIGITAL_POT1_WRITE_ID   ; send proper ID to write to digital pot chip 1
    call    sendI2CByte             ; send byte in W register on I2C bus after SP1IF goes high

    banksel scratch0                ; send the address of the pot in the chip to access
    movf    scratch0,W
    call    sendI2CByte

    banksel scratch1                ; send the pot value
    movf    scratch1,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    return

; end of setDigitalPotInChip1
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeByteToEEprom1ViaI2C
;
; Writes a byte to EEprom1 via the I2C bus.
;
; The byte to be written should be in scratch0.
; The EEprom target address word should be in eepromAddressL/H
;

writeByteToEEprom1ViaI2C:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   EEPROM1_WRITE_ID        ; send proper ID to write to EEprom 1
    call    sendI2CByte             ; send byte in W register on I2C bus after SSP1IF goes high

    banksel eepromAddressH          ; send the address high byte after SSP1IF goes high
    movf    eepromAddressH,W
    call    sendI2CByte

    banksel eepromAddressL          ; send the address low byte after SSP1IF goes high
    movf    eepromAddressL,W
    call    sendI2CByte

    banksel scratch0                ; send the byte to be written after SSP1IF goes high
    movf    scratch0,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    return

; end of writeByteToEEprom1ViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForEEprom1WriteCycleFinished
;
; Waits for EEprom1 to finish its write cycle.
;
; The EEprom will not respond to a write command with an ACK during a write cycle, so a write
; command is repeatedly sent until the EEprom responds with an ACK. A stop condition is then
; generated to abort the write command.
;

waitForEEprom1WriteCycleFinished:

wfewcf1:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   EEPROM1_WRITE_ID        ; send proper ID to write to EEprom 1
    call    sendI2CByte             ; send byte in W register on I2C bus after SP1IF goes high

    call    waitForSSP1IFHighThenClearIt; wait for high flag upon transmission completion

    ; abort the write operation regardless of success
    ; it was only used to check for ACK from slave

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    banksel SSP1CON2
    btfsc   SSP1CON2,ACKSTAT        ; check for low ACK from slave, repeat loop if NACK
    goto    wfewcf1

    return

; end of waitForEEprom1WriteCycleFinished
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
; applyASCIINumDigitLimits
;
; Limits an ASCII value in scratch0 between ASCII '0' and '9'.
;
; Compares the value in scratch0 with ASCII for '0' and '9'. If the value is less than that for
; the '0' character, it is replaced with that value. If greater than the '9' value, it is replaced
; with that value.
;

applyASCIINumDigitLimits:

    banksel scratch0

    movlw   0x30                        ; lower limit is ASCII for '0'
    movwf   scratch1

    movlw   0x39                        ; upper limit is ASCII for '9'
    movwf   scratch2

    goto    applyLimitsToByteValue

; end applyASCIINumDigitLimits
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; applyBCDDigitLimits
;
; Limits an unpacked BCD value in scratchc2 between 0 and 9.
;
; Compares the value in scratchc2 with 0 and 9. If the value is less than that for the 0, it is 
; replaced with that value. If greater than 9, it is replaced with that value.
;
; ON ENTRY:
    
;   scratchc2 = original value to check
;
; ON EXIT:
;
;   If 0 < value < 9        W = original value      [FSR1+2]
;   If value < 0            W = low limit           [FSR1+1]
;   If value > 9            W = high limit          [FSR1]
;

applyBCDDigitLimits:

    movlw   high scratchc0               ; point at the high limit
    movwf   FSR1H
    movlw   low scratchc0
    movwf   FSR1L

    movlw   .9                          ; upper limit for unpacked BCD
    movwf   scratchc0
    movlw   .0                          ; lower limit for unpacked BCD
    movwf   scratchc1

    goto    applyLimitsToByteValue

; end applyBCDDigitLimits
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; applyLimitsToByteValue
;
; Clips a value between a lower and upper limit.
;
; Compares the value in [FSR1+2] with the limits pointed to by FSR1. FSR1 points to the high limit
; value, the low limit should be in [FSR1+1].
;
; ON ENTRY:
;
;   [FSR1]      =   high limit
;   [FSR1+1]    =   low limit
;   [FSR1+2]    =   original value to check against limits
;
; ON EXIT:
;
;   If low limit < value < high limit     W = original value      [FSR1+2]
;   If value < low limit                  W = low limit           [FSR1+1]
;   If value > high limit,                W = high limit          [FSR1]
;

applyLimitsToByteValue:
   
    moviw   2[FSR1]             ; load value into W
    subwf   INDF1,W             ; check if value greater than high limit
    moviw   2[FSR1]             ; load value into W
    btfsc   STATUS,C            ; skip next line if value greater than high limit
    goto    notHigher

    movf    INDF1,W             ; was greater than so replace W with the high limit
    
    return

notHigher:

    ADDFSR  FSR1,1              ; FSR1 will now point to the low limit
    subwf   INDF1,W             ; check if value less than or equal to low limit
    moviw   1[FSR1]             ; load value into W
    btfss   STATUS,C            ; skip next line if value lower than or equal to low limit
    goto    notLower
    
    movf    INDF1,W             ; was less than so replace W with the low limit

notLower:

    decf    FSR1,F              ; decrement FSR1 so that it points at the high limit upon exit again
    
    return

; end applyLimitsToByteValue
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

    call    setupSerialPort ; prepare serial port for sending and receiving

    call    initializeOutputs

    call    setupI2CMaster7BitMode ; prepare the I2C serial bus for use

    call    initHighLowCurrentLimitPotValues

    call    setDigitalPots  ; set digital pot values to stored values

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
    call    zeroDepth               ; clear the depth position variable
    movlp   high setup              ; set PCLATH back to what it was

    clrf   flags3
    clrf    xmtDataTimer
    
    movlw   0xff
    movwf   switchStates
    movwf   switchStatesPrev
    movwf   switchStatesRemote
    
    call    readFlagsFromEEprom     ; read the value stored for flags from the EEProm

    ; reset some values to a default state
    ; leave WALL_MODE as read from eeprom - this state is saved
	; leave MOTOR_DIR_MODE as read from eeprom - this state is saved

    bcf     flags,EXTENDED_MODE
    bcf     flags,CUT_STARTED
    bcf     flags,AT_DEPTH
    bcf     flags,DATA_MODIFIED
    bcf     flags,UPDATE_DISPLAY
    
    call    resetSerialPortRcvBuf           ; re-init flags2 variable after loading from eeprom
    call    resetSerialPortXmtBuf

    call    readSparkLevelsFromEEprom

    ; set sparkLevel to value loaded for Notch or Wall mode depending on current mode

    movf    sparkLevelNotch,W   ; use Notch mode value if in Wall Reduction mode
    btfsc   flags,WALL_MODE
    movf    sparkLevelWall,W    ; use Wall mode value if in Wall Reduction mode
    movwf   sparkLevel          ; save the selected value

    movlp   high zeroTarget
    call    zeroTarget
    movlp   high setup
    
    call    readTargetValueFromEEprom    ; read the value stored for depth from the EEProm

    call    readPWMValsFrmEEpromSendLEDPIC

    banksel MOTOR_ENABLE_L
    bcf     MOTOR_ENABLE_L, MOTOR_ENABLE    ; enable the motor
	
; enable the interrupts

	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 and serial port are peripherals)
    bsf     INTCON,T0IE     ; enable TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

    call    resetLCD        ; resets the LCD PIC and positions at line 1 column 1

    call    setLEDArrays    ; set the on/off states for the LED arrays

    call    sendLEDPICStart     ; command the LED PIC to display Current/Voltage on LED arrays

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

    bsf     TRISA, JOG_DOWN_SW          ; input    
    bsf     TRISA, SHORT_DETECT         ; input
    bsf     TRISA, HI_LIMIT             ; input
    bcf     TRISA, POWER_ON             ; output

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

    bcf     TRISC, MOTOR_ENABLE         ; output
    bsf     TRISC, MODE_SW              ; input
    bsf     TRISC, JOG_UP_SW            ; input
    bcf     TRISC, MOTOR_DIR            ; output
    bcf     TRISC, MOTOR_STEP           ; output
    bsf     TRISC, LO_LIMIT             ; input
    bcf     TRISC, MOTOR_MODE           ; output
    bsf     TRISC, SELECT_SW            ; input

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

    banksel LATA
    bcf     LATA, POWER_ON

    banksel LATC
    bsf     LATC, MOTOR_STEP

    banksel LATC
    bsf     LATC, MOTOR_DIR

    banksel LATC              ; disable the motor
    bsf     LATC, MOTOR_ENABLE

    banksel LATC
	bcf     LATC, MOTOR_MODE    ; choose full step if J8-1 (MS1) = Off and J8-2 (MS2) = On
                                        ; see notes at top of page for more info

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
; initHighLowCurrentLimitPotValues
;
; Sets the high and low current comparator digital pot variables to default settings.
;
; Sets the Power Level value to "2" for the default settings.
;
; Does not send the values to the digital pots.
;

initHighLowCurrentLimitPotValues:

    movlw   HI_CURRENT_LIMIT_POT
    banksel hiCurrentLimitPot
    movwf   hiCurrentLimitPot

    movlw   LO_CURRENT_LIMIT_POT
    banksel loCurrentLimitPot
    movwf   loCurrentLimitPot

    movlw   .2                  ; default power level for the default pot settings is 2
    banksel powerLevel
    movwf   powerLevel

    return

; end of initHighLowCurrentLimitPotValues
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; setDigitalPots
;
; Sets the digital pot values to their stored settings.
;

setDigitalPots:

    movlp   high setHighCurrentLimitDigitalPot      ; readies the PCLATH for the calls below
    
    call    setHighCurrentLimitDigitalPot

    call    setLowCurrentLimitDigitalPot

    banksel scratch0
    movlw   VOLTAGE_MONITOR_POT_ADDR
    movwf   scratch0
    movlw   VOLTAGE_MONITOR_POT
    movwf   scratch1
    call    setDigitalPotInChip1

    banksel scratch0
    movlw   CURRENT_MONITOR_POT_ADDR
    movwf   scratch0
    movlw   CURRENT_MONITOR_POT
    movwf   scratch1
    call    setDigitalPotInChip1
    
    movlp   high setDigitalPots             ; set PCLATH back to what it was on entry    

    return

; end of setDigitalPots
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; readFlagsFromEEprom
;
; Reads the flags, flags2, and flags3 values from eeprom.
;

readFlagsFromEEprom:

    banksel flags
    
    movlw   high flags      ; address in RAM
    movwf   FSR0H
    movlw   low flags
    movwf   FSR0L
        
    clrf    eepromAddressH
    movlw   eeFlags         ; address in EEprom
    movwf   eepromAddressL
    movlw   .3
    movwf   eepromCount     ; read 3 bytes
    call    readFromEEprom

    return

; end of readFlagsFromEEprom
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; readSparkLevelsFromEEprom
;
; Reads the spark Wall/Notch aggression/speed values from eeprom.
;
; Note: these variables must be kept contiguous in memory and eeprom.
;

readSparkLevelsFromEEprom:

    banksel sparkLevelNotch

    movlw   high sparkLevelNotch    ; address in RAM
    movwf   FSR0H
    movlw   low sparkLevelNotch
    movwf   FSR0L

    clrf    eepromAddressH
    movlw   eeSparkLevelNotch   ; address in EEprom
    movwf   eepromAddressL
    movlw   .2
    movwf   eepromCount         ; read 2 bytes
    call    readFromEEprom

    ; apply limits to the values to prevent illegal values read from eeprom

    banksel scratch0

    movlw   high scratch0                   ; point FSR0 at the high limit
    movwf   FSR1H
    movlw   low scratch0
    movwf   FSR1L

    movlw   0x81                            ; upper limit
    movwf   scratch0
    movlw   0x01                            ; lower limit
    movwf   scratch1

    movlp   high applyLimitsToByteValue     ; set PCLATH for calls to applyLimitsToByteValue
    
    movf    sparkLevelNotch,W               ; limit notch spark level value
    movwf   scratch2
    call    applyLimitsToByteValue
    movwf   sparkLevelNotch                 ; update variable with clipped value

    movf    sparkLevelWall,W                ; limit wall spark level value
    movwf   scratch2
    call    applyLimitsToByteValue
    movwf   sparkLevelWall                  ; update variable with clipped value
    
    
    movlp   high readSparkLevelsFromEEprom  ; set PCLATH back to what it was on entry
    
    return

; end of readSparkLevelsFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readTargetValueFromEEprom
;
; Reads the user set target depth value from eeprom. These are unpacked BCD digits.
;
; Each digit is checked for valid range of 0-9 and forced to that range.
;
; Note: these variables must be kept contiguous in memory and eeprom.
;

readTargetValueFromEEprom:

    movlw   high target9
    movwf   FSR0H
    movlw   low target9      ; address in RAM
    movwf   FSR0L
    
    banksel eepromAddressH
    clrf    eepromAddressH
    movlw   eeTarget3        ; address in EEprom
    movwf   eepromAddressL
    movlw   .4
    movwf   eepromCount     ; read 4 bytes
    call    readFromEEprom

    ; check each digit for illegal BCD value (0-9)
    
    movlp   high applyBCDDigitLimits    ; ready PCLATH for the calls below
    
    banksel target9
    movf    target9,W
    movwf   scratchc2
    call    applyBCDDigitLimits
    movwf   target9

    movf    target8,W
    movwf   scratchc2
    call    applyBCDDigitLimits
    movwf   target8

    movf    target7,W
    movwf   scratchc2
    call    applyBCDDigitLimits
    movwf   target7

    movf    target6,W
    movwf   scratchc2
    call    applyBCDDigitLimits
    movwf   target6
    
    movlp   high readTargetValueFromEEprom  ; set PCLATH back to what it was on entry

    return

; end of readTargetValueFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readPWMValsFrmEEpromSendLEDPIC
;
; Reads the Pulse Width Module (PWM) values for the Cutting Current pulse controller and
; transmits the values to the LED PIC which handles that function via the I2C bus.
;
; If the values have never previously been saved to the eeprom, the checksum validation should
; fail and default values will be used.
;

readPWMValsFrmEEpromSendLEDPIC:

    call    readPWMValuesFromEEprom

    call    sendPWMValuesToLEDPIC

    return

; end of readPWMValsFrmEEpromSendLEDPIC
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; readPWMValuesFromEEprom
;
; Reads PWM module time period and duty cycle and output polarity values from the eeprom.
;
; These values are sent to the LED PIC to specify the period and duty cycle of the pulse
; controlling the on/off times of the cutting current.
;
; The least significant bit of the polarity value is used to set the PWM output polarity.
;
; Before saving, the bytes are added together to create a checksum which is also stored in the
; eeprom. This allows the reading function to verify that the values are valid and also that the
; eeprom was not empty and the values had never been stored.
;
; The value of 1 is added to the checksum...(see note in the header for function
;   savePWMValuesToEEprom for details).
;
; Upon reading, if the checksum is invalid then safe default PWM values are used instead.
;
; NOTE: Improper PWM values can destroy the MOSFETs.
;

readPWMValuesFromEEprom:

    banksel pwmDutyCycleHiByte

    movlw   high pwmDutyCycleHiByte      ; address in RAM
    movwf   FSR0H
    movlw   low pwmDutyCycleHiByte
    movwf   FSR0L
        
    clrf    eepromAddressH
    movlw   eePWMDutyCycleHiByte         ; address in EEprom
    movwf   eepromAddressL
    movlw   .5
    movwf   eepromCount                  ; read 4 bytes
    call    readFromEEprom

    banksel pwmDutyCycleHiByte

    clrw                                ; calculate the checksum for all PWM values
    addwf   pwmDutyCycleHiByte,W
    addwf   pwmDutyCycleLoByte,W
    addwf   pwmPeriod,W
    addwf   pwmPolarity,W
    addlw   1                           ; see note in function header

    subwf   pwmCheckSum,W               ; compare calculated checksum with that read from eeprom
    btfsc   STATUS,Z
    goto    pwmValuesReadAreValid       ; zero flag set, checksum matched, skip

    ; checksum read did not match calculated checksum, so use default values

    movlw   PWM_DUTY_CYCLE_HI_BYTE_DEFAULT
    movwf   pwmDutyCycleHiByte

    movlw   PWM_DUTY_CYCLE_LO_BYTE_DEFAULT
    movwf   pwmDutyCycleLoByte

    movlw   PWM_PERIOD_DEFAULT
    movwf   pwmPeriod

    movlw   PWM_POLARITY_DEFAULT
    movwf   pwmPolarity

    clrw                                ; calculate the checksum for all PWM values
    addwf   pwmDutyCycleHiByte,W
    addwf   pwmDutyCycleLoByte,W
    addwf   pwmPeriod,W
    addwf   pwmPolarity,W
    addlw   1                           ; see note in function header
    movwf   pwmCheckSum

pwmValuesReadAreValid:

    return

; end of readPWMValuesFromEEprom
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendPWMValuesToLEDPIC
;
; Sends the PWM values to the LED PIC.
;

sendPWMValuesToLEDPIC:

    banksel scratch0
    movlw   .6                      ; send command byte and two values
    movwf   scratch0

    movlw   LEDPIC_SET_PWM         ; precede values with command byte
    banksel pwmSetCommandByte
    movwf   pwmSetCommandByte

    movlw   high pwmSetCommandByte          ; point to first byte to be sent
    movwf   FSR0H
    movlw   low pwmSetCommandByte
    movwf   FSR0L

    call    sendBytesToLEDPICViaI2C

    return

; end of sendPWMValuesToLEDPIC
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; readFromEEprom
; 
; Read block of bytes from EEprom.
;
; Address in EEprom from which to read first byte should be in eepromAddressH/L.
; Number of bytes to read should be in eepromCount.
; Indirect register FSR0 should point to first byte in RAM to be written into.
; The block of bytes will be copied from EEprom to RAM.
; 

readFromEEprom:

loopRFE1:

    call    readByteFromEEprom1ViaI2C

    banksel scratch0            ; store byte read to the buffer
    movwf   scratch0
    movwi   FSR0++

    banksel eepromAddressL
	incf	eepromAddressL,F    ; move to next address in EEprom
    btfsc   STATUS,Z
	incf	eepromAddressH,F	; increment high byte on low byte rollover

    banksel eepromCount
	decfsz	eepromCount,F       ; count down number of bytes transferred
	goto	loopRFE1            ; not zero yet - transfer more bytes

	return

; end of readFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readByteFromEEprom1ViaI2C
;
; Reads a byte from EEprom1 via the I2C bus.
;
; The byte read will be returned in scratch0.
; The EEprom source address word should be in eepromAddressL/H
;
; To read a byte from the EEprom, the source address is first set by using a write command. A
; restart condition is then generated and the byte read using a read command.
; After the read, a NACK is sent to the slave followed by a stop condition.
;

readByteFromEEprom1ViaI2C:

    movlp   high clearSSP1IF        ; ready PCLATH for the calls below
    
    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   EEPROM1_WRITE_ID        ; send proper ID to write to EEprom 1 (used to set address)
    call    sendI2CByte             ; send byte in W register on I2C bus after SP1IF goes high

    banksel eepromAddressH          ; send the address high byte
    movf    eepromAddressH,W
    call    sendI2CByte

    banksel eepromAddressL          ; send the address low byte
    movf    eepromAddressL,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CRestart

    movlw   EEPROM1_READ_ID        ; send proper ID to write to EEprom 1
    call    sendI2CByte            ; send byte in W register via I2C bus after SP1IF goes high
    
    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    banksel SSP1CON2
    bsf     SSP1CON2,RCEN

    call    waitForSSP1IFHighThenClearIt;    wait for high flag upon reception completion

    banksel SSP1BUF                 ; store the received byte in scratch0
    movf    SSP1BUF,W
    banksel scratch0
    movwf   scratch0

    banksel SSP1CON2                ; send NACK to terminate read
    bsf     SSP1CON2,ACKDT          ; send high bit (NACK)
    bsf     SSP1CON2,ACKEN          ; enable NACK transmission

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon NACK transmission complete

    banksel SSP1CON2
    bcf     SSP1CON2,ACKDT          ; reset to send ACKs

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    movlp   high readByteFromEEprom1ViaI2C  ; set PCLATH back to what it was on entry
    
    return

; end of readByteFromEEprom1ViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setStepDistance
;
; Sets the step distance based on standard/extended reach mode and erosion/no erosion mode.
;
; The step distance is the distance the head moves with each step of the motor.
;
; On entry:
;
; bank set to flags
;
; On exit:
;
       
setStepDistance:
    
    movlw   high step10         ; destination variable for step value
    movwf   FSR1H
    movlw   low step10
    movwf   FSR1L

    btfsc   flags,EXTENDED_MODE
    goto    handleExtendedModeSSD

    ; copy value for the standard tool to the inches/motor step variable    
    
    btfsc   flags3,EROSION_MODE
    goto    useStdErosionFactor

    movlw   high stdNoErosion   ; source is constant in program memory -- no erosion factor
    movwf   FSR0H               ;  ("high" directive will automatically set bit 7 for program space)
    movlw   low stdNoErosion
    movwf   FSR0L

    goto    copyConstantToVarSSD

useStdErosionFactor:

    movlw   high std17Erosion   ; source is constant in program memory -- with erosion factor
    movwf   FSR0H               ;  ("high" directive will automatically set bit 7 for program space)
    movlw   low std17Erosion
    movwf   FSR0L    
    
    goto    copyConstantToVarSSD
    
handleExtendedModeSSD:    

    ; copy value for the extended tool to the inches/motor step variable

    btfsc   flags3,EROSION_MODE
    goto    useExtErosionFactor

    movlw   high extNoErosion   ; source is constant in program memory -- no erosion factor
    movwf   FSR0H               ;  ("high" directive will automatically set bit 7 for program space)
    movlw   low extNoErosion
    movwf   FSR0L

    goto    copyConstantToVarSSD

useExtErosionFactor:

    movlw   high ext17Erosion   ; source is constant in program memory -- with erosion factor
    movwf   FSR0H               ;  ("high" directive will automatically set bit 7 for program space)
    movlw   low ext17Erosion
    movwf   FSR0L

copyConstantToVarSSD:
    
    movlw   .12                 ; number of bytes
    goto    copyBytes

; end of setStepDistance
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; copyBytes
;
; Copies the number of bytes specified by W from FSR0 to FSR1.
;
; On entry:
;
; W contains the number of bytes to copy
; bank points to scratch1
; FSR0 points to MSB of source bytes
; FSR1 points to MSB of destination bytes
;

copyBytes:

    movwf   scratch1        ; use scratch variable as loop counter

cBLoop1:

    moviw   FSR0++          ; copy each byte
    movwi   FSR1++

    decfsz  scratch1,F
    goto    cBLoop1

    return

; end of copyBytes
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
; setLEDArrays
;
; Sets LED arrays to known on/off states.
;

setLEDArrays:

    banksel scratch2

    movlw   0x00            ; value for red LED array
    comf    WREG,W          ; invert the value -- a zero turns an LED on
    movwf   scratch2
    movlw   0x00            ; value for green LED array
    comf    WREG,W          ; invert the value -- a zero turns an LED on
    movwf   scratch3

    call    sendLEDArrayValues

    return

; end of setLEDArrays
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendLEDArrayValues
;
; Sends values to LED PIC to set on/off state of the LED arrays.
;
; Value for the red LED array should be in scratch2.
; Value for the red LED array should be in scratch3.
;

sendLEDArrayValues:

    banksel scratch0

    movlw   .3                      ; send command byte and two values
    movwf   scratch0
    movlw   LEDPIC_SET_LEDS         ; put command byte in scratch1
    movwf   scratch1

    movlw   high scratch1           ; point to first byte to be sent
    movwf   FSR0H
    movlw   low scratch1
    movwf   FSR0L

    call    sendBytesToLEDPICViaI2C

    return

; end of sendLEDArrayValues
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendLEDPICStart
;
; Sends start command to LED PIC to initiate displaying the Current and Voltage monitor voltages
; on the LED arrays.
;

sendLEDPICStart:

    banksel scratch0

    movlw   .1                      ; send command byte
    movwf   scratch0
    movlw   LEDPIC_START            ; put command byte in scratch1
    movwf   scratch1
    
    movlw   high scratch1           ; point to first byte to be sent
    movwf   FSR0H
    movlw   low scratch1
    movwf   FSR0L

    call    sendBytesToLEDPICViaI2C

    return

; end of sendLEDPICStart
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendBytesToLEDPICViaI2C
;
; Sends byte to the LED PIC via the I2C bus.
;
; The number of bytes to be written should be in scratch0.
; Indirect register FSR0 should point to first byte in RAM to be written.
;

sendBytesToLEDPICViaI2C:

    movlp   high clearSSP1IF        ; ready PCLATH for the calls below

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   LED_PIC_WRITE_ID        ; send proper ID to write to LED PIC
    call    sendI2CByte             ; send byte in W register on I2C bus after SSP1IF goes high

loopSBLP1:

    movlp   high sendI2CByte ; ready PCLATH for the call to sendI2CByte
    moviw   FSR0++                  ; load next byte to be sent
    call    sendI2CByte
    
    movlp   high loopSBLP1          ; set PCLATH for the goto
    banksel scratch0
	decfsz	scratch0,F              ; count down number of bytes transferred
	goto	loopSBLP1               ; not zero yet - transfer more bytes

    movlp   high waitForSSP1IFHighThenClearIt  ; ready PCLATH for the calls below
    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion
    call    generateI2CStop
    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished
    movlp   high sendBytesToLEDPICViaI2C    ; set PCLATH back to what it was on entry

    return

; end of sendBytesToLEDPICViaI2C
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; debugFunc1
;
; Performs functions for debug testing such as stuffing values into variables, etc.
;

debugFunc1:

    movlw   high depth10
    movwf   FSR0H
    movlw   low depth10
    movwf   FSR0L

    movlw   high target10
    movwf   FSR1H
    movlw   low target10
    movwf   FSR1L

    movlw   .11             ; 11 digits in the operands

    call    addBCDVars

    movlw   high depth10
    movwf   FSR0H
    movlw   low depth10
    movwf   FSR0L

    movlw   high target10
    movwf   FSR1H
    movlw   low target10
    movwf   FSR1L

    movlw   .11             ; 11 digits in the operands

    call    subtractBCDVars

    return

    ; set scratch variables to a testing value

    movlw   high depth10
    movwf   FSR0H
    movlw   low depth10
    movwf   FSR0L

    banksel cycleTestRetract0

    movlw   .11         ; number of variables in buffer to set
    movwf   cycleTestRetract0
    movlw   .0          ; value to increment and store in variables
    movwf   cycleTestRetract1

dF1Loop:

    movwi   FSR0++

    incf    cycleTestRetract1,F
    movf    cycleTestRetract1,W

    decfsz  cycleTestRetract0,F
    goto    dF1Loop

    return


    ; stuff a value into the PWM variables

    movlw   0xff
    movwf   pwmDutyCycleLoByte
    movwf   pwmPeriod
    movwf   pwmPolarity

    movlw   0xfd
    movwf   pwmCheckSum

    return

; end of debugFunc1
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendOutputStates
;
; Sends the appropriate output states via the serial port to a remote device such as the User
; Interface board.
;
; On Entry:
;
; On Exit:
;

sendOutputStates:

    call    setupOutputStatesPacket    

    banksel outputStates                ; start with all outputs off
    movlw   0xff
    movwf   outputStates

    btfsc   MODE_JOGUP_SEL_EPWR_P, AC_PWR_OK_SIGNAL ; signal input high to turn on LED
    bcf     outputStates, AC_OK_LED_FLAG

    btfsc   SHORT_DETECT_P, SHORT_DETECT        ; buzzer and Short LED on when short detected
    goto    noShortAlerts
 
    banksel POWER_ON_L                          ; no short if power supply is not turned on
    btfss   POWER_ON_L,POWER_ON                 ; this results in "no current" but is ignored
    goto    noShortAlerts
   
    banksel outputStates

    bcf     outputStates, SHORT_LED_FLAG        ; light the "short" LED
    
    btfsc   flags3,ALARM_ENABLED                ; activate audible alarm only if enabled
    bcf     outputStates, BUZZER_FLAG

noShortAlerts:    

    banksel outputStates    
    movf    outputStates,W
    
    movlp   high writeByteToSerialXmtBuf
    call    writeByteToSerialXmtBuf    
    
    movlp   high startSerialPortTransmit
    call    startSerialPortTransmit

    return

; end of sendOutputStates
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupOutputStatesPacket
;
; Prepares the serial transmit buffer with header, length byte of 3, and command SET_OUTPUTS_CMD.
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
    
setupOutputStatesPacket:

    movlw   SET_OUTPUTS_CMD           ; packet command byte
    
    movlp   high setupSerialXmtPkt
    call    setupSerialXmtPkt
    movlp   high setupOutputStatesPacket

    banksel flags

    return

; end of setupOutpuStatesPacket
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; handleSwitchStatesPacket
;
; Stores the switch states reported by the remote device via serial link, such as the User Interface
; board.
;
; On Entry:
;
;   FSR1 points to serialRcvBuf
; 

handleSwitchStatesPacket:

    moviw   1[FSR1]                     ; get the switch state value from the packet

    banksel switchStatesRemote
    movwf   switchStatesRemote          ; store the switch states

    return

; end of handleSwitchStatesPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; parseCommandFromSerialPacket
;
; Parses the command byte in a serial packet and performs the appropriate action.
;
; On Entry:
;
; On Exit:
;
; FSR1 points to serialRcvBuf
;

parseCommandFromSerialPacket:

    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H     ; point FSR0 at start of receive buffer
    movwf   FSR1H
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   FSR1L

; parse the command byte by comparing with each command

    movf    INDF1,W
    sublw   SWITCH_STATES_CMD
    btfsc   STATUS,Z
    goto    handleSwitchStatesPacket

;    movf    INDF1,W
;    sublw   ???        -- use this example for a future command
;    btfsc   STATUS,Z
;    goto    handleLCDInstructionPacket

    return

; end of parseCommandFromSerialPacket
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
    goto    parseCommandFromSerialPacket    ; checksum good so handle command

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

    addfsr  FSR0,.3                             ; skip 2 header bytes and 1 length byte
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
    addfsr  INDF0,1

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
; Step Ratio Constants:
;
; Specifies the blade travel distance per motor step for different types of cutting heads.
;
; These are unpacked BCD decimal values, each digit can be 0-9.
; The decimal point is two digits from the left for all depth related values.
;
; Standard Reach Tool Ratio
;
; One motor step is 0.000217391"
; typically, blade erosion is 17%
;
; For 1:1 (no erosion factor), use .000217391
; For 17% erosion factior use .000217391 * .83 = 0.000180435
;
; Extended Reach Tool Ratio
;
; One motor step is .000023774"
; typically, blade erosion is 17%
;
; For 1:1 (no erosion factor), use .000023774
; For 17% erosion factior use .000023774 * .83 = 0.000019732
;
; Unpacked BCD is used for inch/pulse depth related values as it allows for the fastest math during
; operation. The decimal point is two digits from the left for all depth related values.
;
; NOTE: program memory is only 14 bits wide. Using dw rather than db forces each value to fill
; an entire 14 bit word rather than being packed two values to a word.
;
; Last digit is sign: 0 for positive; 1 for negative
;
; 00.000217391 -> standard tool with no erosion factor ~ inches/motor pulse
stdNoErosion    dw  0,0,0,0,0,2,1,7,3,9,1,0     ; unpacked BCD ~ xx.xxxxxxxxx

; 00.000180435 -> standard tool with 17% erosion factor ~ inches/motor pulse
std17Erosion    dw  0,0,0,0,0,1,8,0,4,3,5,0     ; unpacked BCD ~ xx.xxxxxxxxx

; 00.000023774 -> extended tool with no erosion factor ~ inches/motor pulse
extNoErosion    dw  0,0,0,0,0,0,2,3,7,7,4,0     ; unpacked BCD ~ xx.xxxxxxxxx

; 00.000019732 -> extended tool with 17% erosion factor ~ inches/motor pulse
ext17Erosion    dw  0,0,0,0,0,0,1,9,7,3,2,0     ; unpacked BCD ~ xx.xxxxxxxxx

; end of Constants in Program Memory
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; Strings in Program Memory
;
    
string0	    dw	'O','P','T',' ','A','u','t','o','N','o','t','c','h','e','r',' ','7','.','7','i',0x00
string1	    dw	'C','H','O','O','S','E',' ','C','O','N','F','I','G','U','R','A','T','I','O','N',0x00
string2	    dw	'1',' ','-',' ','E','D','M',' ','N','o','t','c','h','C','u','t','t','e','r',0x00
string3	    dw	'2',' ','-',' ','E','D','M',' ','E','x','t','e','n','d',' ','R','e','a','c','h',0x00
string4	    dw	'O','P','T',' ','E','D','M',' ','N','o','t','c','h','C','u','t','t','e','r',0x00
string5	    dw	'1',' ','-',' ','S','e','t',' ','C','u','t',' ','D','e','p','t','h',0x00
string6	    dw	'1',' ','-',' ','D','e','p','t','h',' ','=',' ',0x00
string7	    dw	'2',' ','-',' ','C','u','t',' ','N','o','t','c','h',0x00
string8	    dw	'3',' ','-',' ','J','o','g',' ','E','l','e','c','t','r','o','d','e',0x00
string9	    dw	' ',' ',' ','S','e','t',' ','C','u','t',' ','D','e','p','t','h',0x00
string10    dw	'0','.','0','0','0',' ','i','n','c','h','e','s',0x00
string11    dw	'J','o','g',' ','M','o','d','e',0x00
string12    dw	'Z','e','r','o',' ','o','r',' ','E','x','i','t',0x00
string13    dw	'O','P','T',' ','E','D','M',' ','E','x','t','e','n','d',' ','R','e','a','c','h',0x00
string14    dw	'T','u','r','n',' ','o','n',' ','C','u','t',' ','V','o','l','t','a','g','e',0x00
string15    dw	'U','p',' ',' ',' ','S','p','e','e','d','>',0x00
string16    dw	'D','o','w','n',' ',' ','S','t','o','p','>',0x00
string17    dw	'N','o','t','c','h',' ','M','o','d','e',0x00
string18    dw	'W','a','l','l',' ','M','o','d','e',' ',0x00
string19    dw	'C','y','c','l','e',' ','T','e','s','t',0x00
string20    dw	'5',' ','-',' ','M','o','t','o','r',' ','D','i','r',' ',0x00
string21    dw	'4',' ','-',' ',0x00
string22    dw	'N','o','r','m','a','l',0x00
string23    dw	'R','e','v',0x00
string24    dw	'6',' ','-',' ','E','r','o','s','i','o','n',' ',0x00
string25    dw	'7',' ','-',' ','A','l','a','r','m',' ',0x00
string26    dw	'N','o','n','e',0x00
string27    dw	'1','7','%',0x00
string28    dw	'O','f','f',0x00
string29    dw	'O','n',0x00
    
; end of Strings in Program Memory
;--------------------------------------------------------------------------------------------------

    END
