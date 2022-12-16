;* ------------------------------------------------------------------
;* --  _____       ______  _____                                    -
;* -- |_   _|     |  ____|/ ____|                                   -
;* --   | |  _  | |  | (___    Institute of Embedded Systems    -
;* --   | | | '_ \|  __|  \___ \   Zurich University of             -
;* --  _| |_| | | | |____ ____) |  Applied Sciences                 -
;* -- |_____|_| |_|______|_____/   8401 Winterthur, Switzerland     -
;* ------------------------------------------------------------------
;* --
;* -- Project     : CT1 - Lab 7
;* -- Description : Control structures
;* -- 
;* -- $Id: main.s 3748 2016-10-31 13:26:44Z kesr $
;* ------------------------------------------------------------------


; -------------------------------------------------------------------
; -- Constants
; -------------------------------------------------------------------
    
                AREA myCode, CODE, READONLY
                    
                THUMB

ADDR_LED_15_0           EQU     0x60000100
ADDR_LED_31_16          EQU     0x60000102
ADDR_7_SEG_BIN_DS1_0    EQU     0x60000114
ADDR_DIP_SWITCH_15_0    EQU     0x60000200
ADDR_HEX_SWITCH         EQU     0x60000211

NR_CASES            EQU     0xB

jump_table      ; ordered table containing the labels of all cases
                ; STUDENTS: To be programmed 
		DCD case_dark
		DCD case_add
		DCD case_sub
		DCD case_mul
		DCD case_and
		DCD case_or
		DCD case_xor
		DCD case_not
		DCD case_nand
		DCD case_nor
		DCD case_xnor
		DCD case_default
		DCD case_default
		DCD case_default
		DCD case_default
		DCD case_default
		; END: To be programmed
        

; -------------------------------------------------------------------
; -- Main
; -------------------------------------------------------------------   
                        
main            PROC
                EXPORT main
                
read_dipsw      ; Read operands into R0 and R1 and display on LEDs
                ; STUDENTS: To be programmed
        
        LDR  R0,=ADDR_DIP_SWITCH_15_0
        LDR  R0,[R0]
        LDR  R3, =ADDR_LED_15_0
        
        STRH R0,[R3]
        MOVS R1,R0
        
        LDR R3,=0xFF00
        ANDS R0,R0,R3 ;get bits 8-15
        LSRS R0,R0,#8
        
        MOVS R3,#0xFF
        ANDS R1,R1,R3;get bits 0-7

read_hexsw      ; Read operation into R2 and display on 7seg.
                ; STUDENTS: To be programmed
        LDR R3,=ADDR_HEX_SWITCH
        LDRH R2,[R3]
        LDR R3,=ADDR_7_SEG_BIN_DS1_0
        MOVS R4,#0x0F ; remove all bytes but first
        ANDS R2,R2,R4
        STRH R2,[R3]


                ; END: To be programmed
                
case_switch     ; Implement switch statement as shown on lecture slide
                ; STUDENTS: To be programmed
        
        CMP R2,#NR_CASES
        BHS case_default
        
        LSLS R2, #2
        LDR R3, =jump_table
        LDR R3,[R3,R2]
        BX R3



                ; END: To be programmed


; Add the code for the individual cases below
; - operand 1 in R0
; - operand 2 in R1
; - result in R0

case_dark       
                LDR  R0, =0
                B    display_result  

case_add        
                ADDS R0, R0, R1
                B    display_result
      

; STUDENTS: To be programmed

case_sub
            SUBS R0, R0, R1
                B    display_result

case_mul
            MULS R0, R0, R1
                B    display_result

case_and
            ANDS R0, R0, R1
                B    display_result
        
case_or
            ORRS R0, R0, R1
                B    display_result    
        
case_xor
            EORS R0, R0, R1
                B    display_result    

case_not
            MVNS R0, R1
                B    display_result    
        
case_nand
        	ANDS R0, R0, R1
            MVNS R0, R1
                B    display_result
case_nor
            ORRS R0, R0, R1
            MVNS R0, R1
                B    display_result    
case_xnor
             EORS R0, R0, R1
            MVNS R0, R1
                B    display_result  

case_default
        LDR  R0, =1
                B    display_result 
; END: To be programmed


display_result  ; Display result on LEDs
                ; STUDENTS: To be programmed


                ; END: To be programmed

                B    read_dipsw
                
                ALIGN
                ENDP

; -------------------------------------------------------------------
; -- End of file
; -------------------------------------------------------------------                      
                END
