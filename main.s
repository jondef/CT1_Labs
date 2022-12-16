; ------------------------------------------------------------------
; --  _____       ______  _____                                    -
; -- |_   _|     |  ____|/ ____|                                   -
; --   | |  _ __ | |__  | (___    Institute of Embedded Systems    -
; --   | | | '_ \|  __|  \___ \   Zurich University of             -
; --  _| |_| | | | |____ ____) |  Applied Sciences                 -
; -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland     -
; ------------------------------------------------------------------
; --
; -- main.s
; --
; -- CT1 P06 "ALU und Sprungbefehle" mit MUL
; --
; -- $Id: main.s 4857 2019-09-10 17:30:17Z akdi $
; ------------------------------------------------------------------
;Directives
        PRESERVE8
        THUMB

; ------------------------------------------------------------------
; -- Address Defines
; ------------------------------------------------------------------

ADDR_LED_15_0           EQU     0x60000100
ADDR_LED_31_16          EQU     0x60000102
ADDR_DIP_SWITCH_7_0     EQU     0x60000200
ADDR_DIP_SWITCH_15_8    EQU     0x60000201
ADDR_7_SEG_BIN_DS3_0    EQU     0x60000114
ADDR_BUTTONS            EQU     0x60000210
MASK_KEY_T0             EQU     0x00000001

ADDR_LCD_RED            EQU     0x60000340
ADDR_LCD_GREEN          EQU     0x60000342
ADDR_LCD_BLUE           EQU     0x60000344
LCD_BACKLIGHT_FULL      EQU     0xffff
LCD_BACKLIGHT_OFF      	EQU     0x0000
	
MASK_BIT_CLEAR			EQU		0xF0
NUMBER_16		EQU 	0x10
NUMBER_TEN		EQU 	0xA

; ------------------------------------------------------------------
; -- myCode
; ------------------------------------------------------------------
        AREA myCode, CODE, READONLY

        ENTRY

main    PROC
        export main
            
		; STUDENTS: To be programmed
		LDR 	R2, =MASK_BIT_CLEAR ; 0xF0
		
		; BCD Ones
		LDR 	R0, =ADDR_DIP_SWITCH_7_0
        LDRB 	R0, [R0]
		
		; BCD Tens
		LDR 	R1, =ADDR_DIP_SWITCH_15_8
        LDRB 	R1, [R1]
		
		;Clear the last 4 digits of the BCD tens so that it cant go over 9
		BICS 	R0, R0, R2
		BICS 	R1, R1, R2
		
		BL T0_pressed
		
		;  First we copy the BCD result
		MOV 	R3, R2
		
		; Then we shift 4 bits to the left
		LSLS 	R3, #4
		
		; Then add the entered tens
		ADDS 	R3, R1
		
		; Again we shift 4 bits (by value 16) to the left
		LSLS 	R3, #4 
		
		; Then add the entered ones
		ADDS 	R3, R0 

		; Store the sum of R3 od R4
		LDR 	R4, =ADDR_LED_15_0
       	STR 	R3, [R4]
		
		; display R3 on 7-segment display
		LDR 	R4, =ADDR_7_SEG_BIN_DS3_0
        STR 	R3, [R4]
		
		MOVS R7, R2
		
		;Overwrite R5 with 0
        MOVS R5, #0

		; Task 2 end
		; END: To be programmed
        B       main
        ENDP
            
;----------------------------------------------------
; Subroutines
;----------------------------------------------------
T0_pressed
        PUSH    {R0, R1, R2}
        ; load base address of keys
        LDR     R1, =ADDR_BUTTONS 
        
        ; Load key mask T0             
        LDR     R2, =MASK_KEY_T0     
        
        ; Load key values          
        LDRB    R0, [R1]  
        
        ; Check if key T0 is pressed                   
        TST     R0, R2                         
	
		;Remove element R0 and R1 and R2 from list
		POP     {R0, R1, R2}

        ; Reset lights
        PUSH    {R0, R1}
        LDR     R0, =ADDR_LCD_BLUE
        
        ; Store element R0 in R1
        LDR     R1, =LCD_BACKLIGHT_OFF
        STR     R1, [R0]
        
        ; Store element R0 in R1
        LDR     R0, =ADDR_LCD_RED
        STR     R1, [R0]
        
        ;Remove element R0 and R1 from list
        POP     {R0, R1}

        BEQ     multiplication
        B       shiftingAndAddition
        ALIGN

multiplication
        ; set LCD color
        PUSH    {R0, R1}
        LDR     R0, =ADDR_LCD_BLUE              ; load base address of pwm blue
        LDR     R1, =LCD_BACKLIGHT_FULL         ; backlight full blue
        STRH    R1, [R0]                        ; write pwm register
        POP     {R0, R1}

		; Use 10 for multiplication
		MOVS	R2, #10
	
		; Make the multiplication on bcd tens
	    MULS 	R2, R1, R2 
	
		; Now add the ones to result of multiplication
		ADDS 	R2, R0 		

        BX      LR
        ALIGN
		
shiftingAndAddition
        ; set LCD color
        PUSH    {R0, R1}
        LDR     R0, =ADDR_LCD_RED               
        LDR     R1, =LCD_BACKLIGHT_FULL         
        STRH    R1, [R0]                   
        POP     {R0, R1}
		
        PUSH    {R3} ; tmp
		; Leftshift by 2^3=8 to R3
        LSLS    R3, R1, #3
		; Leftshift by 2^1=2 to R2
        LSLS    R2, R1, #1  
		; Add the values (n * 8) + (n * 2)
        ADDS	R2, R3		
		ADDS	R2, R0		
        POP     {R3}

        BX      LR
        ALIGN

disco_loop
	LDR R7, =0x10
	CMP R1, R7 ;check if iterator is 16
	
	BEQ disco_loop_ended ;exit loop
	
	ADDS R1, R1, #1
	
	MOVS R3, R2
	; Duplication of the two bytes into the upper two bytes
	LSLS R3, R3, #16
	ORRS R2, R2, R3
	
	LDR R5, =0x01 ;speed of loop
        RORS R2, R2, R5
	LDR R4, =ADDR_LED_31_16
	STRH  R2, [R4]
	
	BL pause
	B disco_loop
disco_loop_ended

;----------------------------------------------------
; pause for disco_lights
pause           PROC
        PUSH    {R0, R1}
        LDR     R1, =1
        LDR     R0, =0x000FFFFF
        
loop        
        SUBS    R0, R0, R1
        BCS     loop
    
        POP     {R0, R1}
        BX      LR
        ALIGN
        ENDP

; ------------------------------------------------------------------
; End of code
; ------------------------------------------------------------------
        END
