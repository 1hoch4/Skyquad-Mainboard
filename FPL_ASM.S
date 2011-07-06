/*****************************************************************************/
/* Description: Fixpoint function libary 									 */
/*****************************************************************************/

/*
-------------------------------------------------------------------------------
Copyright (c) 1hoch4UG (haftungsbeschränkt)
www.1hoch4.de
-------------------------------------------------------------------------------
GERMAN:
-------------------------------------------------------------------------------
Alle Rechte an dem gesamten Projekt und allen damit verbundenen Dateien und 
Informationen, verbleiben bei der 1hoch4 UG. Dies gilt insbesondere für die in
Form des Quellcodes veröffentlichten Softwareteile. 

Nutzung der Hardware:
Eine kommerzielle Anwendung (z.B. Luftbildfotografie) der Hardware steht dem 
Nutzer frei. Die 1hoch4 UG schließt jedoch jegliche Haftung für Schäden durch
eine kommerzielle Nutzung aus, da es sich um ein experimentelles Hobbyprojekt
im Betastatus handelt, dessen Hard- und Software sich in einer stetigen
Weiterentwicklung befindet und deshalb nicht explizit für einen professionellen
Einsatz freigegeben werden kann. Der nicht private Verkauf, die Weiter-
verarbeitung (z.B. Bestückung) oder die Zusammenstellung der angebotenen
Bausätze und/oder Platinen zu einem fertigen Produkt bedarf der Abstimmung mit
der 1hoch4 UG.


Nutzung der Software(quellen):
Grundsätzlich darf die Software nur auf den von der 1hoch4 UG zur Verfügung
gestellten Hardware eingesetzt werden. Jegliche Art der Nutzung des
veröffentlichten Sourcecodes, auch auszugsweise, ist nur für den privaten und 
nichtkommerziellen Gebrauch zulässig. Jegliche kommerzielle Nutzung oder
Portierung auf andere Hardware bedarf der schriftlichen Zustimmung der
1hoch4 UG. Eine private Verwendung (auch auszugsweise) des Quellcodes,
unabhängig davon ob verändert oder unverändert, hat zur Folge, dass die
Software weiterhin den hier beschriebenen Bedingungen/Lizenz unterliegt und
diese den verwendeten Softwareteilen beigefügt werden müssen. Weiterhin ist die
1hoch4 UG eindeutig als Quelle anzugegeben. Eine Veränderung und Verwendung der
Softwarequellen geschied auf eigene Gefahr. 

Die 1hoch4 UG übernimmt keinerlei Haftung für direkte oder indirekte
Personen-/Sachschäden. Bedingt durch den experimentellen Status der
1hoch4-Projekte wird keine Gewähr auf Fehlerfreiheit, Vollständigkeit oder
Funktion gegeben.
-------------------------------------------------------------------------------
ENGLISH:
-------------------------------------------------------------------------------
t.b.d.



-------------------------------------------------------------------------------
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE. 
-------------------------------------------------------------------------------
*/


; usage of register in GCC:
;
; r0, r18...r27 and r30...r31 free to use
; r02...r17 and r28...r29 save to stack
; clear r1 if necessary at the end of ASM routine, GCC expect r1=0

; *****************************************************************************
; *                                 Includes                                  *
; *****************************************************************************

#include "avr/io.h"

; *****************************************************************************
; *                            FPL_mulsu16x8_32_                              *
; *****************************************************************************
; *                                                                           *
; * multiplication of int16 and uint8				                          *
; *                                                                           *
; * C-header	   :                                                          *
; *                                                                           *
; * int32_t FPL_mulsu16x8_32_(int16_t a, uint8_t b)                           *
; *                                                                           *
; * c = a*b                                                                   *
; *                                                                           *
; * - argument a in (r25:r24)                           				      *
; * - argument b in r22                 				                      *
; * - result   c in (r25:r24:r23:r22) 					                      *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define c0 r22
#define c1 r23
#define c2 r24
#define c3 r25

#define al r18
#define ah r19
#define b  r20

#define zero r26

.global FPL_mulsu16x8_32_
.func FPL_mulsu16x8_32_

FPL_mulsu16x8_32_:			
		
		clr   zero
		movw  al,  r24     ; copy a to ah:al 
		mov   b,   r22     ; copy b to bh:bl 
		clr   c2
		clr   c3
        mul   al,  b       ; (signed)b * al ...
		movw  c0,  r0      ; => c1:c0                
        muls  ah,  b       ; (signed)ah * (signed)b ...
        sbc   c3,  zero
		add   c1,  r0
		adc   c2,  r1
        adc   c3,  zero												
		clr   r1           ; C-Compiler expect r1 == 0
        ret

#undef c0
#undef c1
#undef c2
#undef c3

#undef al
#undef ah
#undef b

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_macsu16x8_32_                              *
; *****************************************************************************
; *                                                                           *
; * calculate product of int16 and uint8 and add to int32                     *
; *                                                                           *
; * C-header	   :                                                          *
; *                                                                           *
; * void FPL_macsu16x8_32_(int16_t a, uint8_t b, int32_t *cPtr)               *
; *                                                                           *
; * *cPtr += a*b                                                              *
; *                                                                           *
; * - argument a in (r25:r24)                           				      *
; * - argument b in r22                 				                      *
; * - result   cPtr in (r21:r20)	 					                      *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define c0 r24
#define c1 r25
#define c2 r26
#define c3 r27

#define al r20
#define ah r21
#define b  r22

#define zero r23

.global FPL_macsu16x8_32_
.func FPL_macsu16x8_32_

FPL_macsu16x8_32_:

		clr   zero
		movw  r30, r20     ; z = cPtr
		movw  al,  r24
		ld    c0,  z+
		ld    c1,  z+
		ld    c2,  z+
		ld    c3,  z+
		mul   al,  b       ; (signed)b * al ...
		add   c0,  r0
		adc   c1,  r1
		adc   c2,  zero
		adc   c3,  zero
		muls  ah,  b       ; (signed)ah * (signed)b ...
		sbc   c3,  zero
		add   c1,  r0
		adc   c2,  r1
		adc   c3,  zero
		st    -z,  c3
		st    -z,  c2
		st    -z,  c1
		st    -z,  c0
		clr   r1           ; C-Compiler expect r1 == 0
		ret

#undef c0
#undef c1
#undef c2
#undef c3

#undef al
#undef ah
#undef b

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_muls16x16_32_                              *
; *****************************************************************************
; *                                                                           *
; * product of two int16					                                  *
; *                                                                           *
; * C-header	   :                                                          *
; *                                                                           *
; * int32_t FPL_muls16x16_32_(int16_t a, int16_t b)                           *
; *                                                                           *
; * c = a*b                                                                   *
; *                                                                           *
; * - argument a in (r25:r24)                                				  *
; * - argument b in (r23:r22)                                				  *
; * - result   c in(r25:r24:r23:r22)                   						  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define c0 r22
#define c1 r23
#define c2 r24
#define c3 r25

#define al r18
#define ah r19
#define bl r20
#define bh r21

#define zero r26

.global FPL_muls16x16_32_
.func FPL_muls16x16_32_

FPL_muls16x16_32_:

		clr   zero
		movw  al,  r24     ; copy a to ah:al
		movw  bl,  r22     ; copy b to bh:bl
		muls  ah,  bh      ; (signed)ah * (signed)bh ...
		movw  c2,  r0      ; => c3:c2
		mul   al,  bl      ; al * bl ...
		movw  c0,  r0      ; => c1:c0
		mulsu ah,  bl      ; (c3:c2:c1) += (signed)ah * bl
		sbc   c3,  zero     
		add   c1,  r0
		adc   c2,  r1
		adc   c3,  zero
		mulsu bh,  al      ; (c3:c2:c1) += (signed)bh * al
		sbc   c3,  zero
		add   c1,  r0
		adc   c2,  r1
		adc   c3,  zero												
		clr   r1           ; C-Compiler expect r1 == 0
		ret

#undef c0
#undef c1
#undef c2
#undef c3

#undef al
#undef ah
#undef bl
#undef bh

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_macs16x16_32_                              *
; *****************************************************************************
; *                                                                           *
; * multiplicate two int16 and add to int32				                      *
; *                                                                           *
; * C-header:                                                        		  *
; *                                                                           *
; * void FPL_macs16x16_32_(int16_t a, int16_t b, int32_t *cPtr)               *
; *                                                                           *
; * *cPtr += a*b                                                              *
; *                                                                           *
; * - argument a in (r25:r24)                                				  *
; * - argument b in (r23:r22)                                				  *
; * - ergebnis cPtr in (r21:r20)				                              *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define c0 r24
#define c1 r25
#define c2 r26
#define c3 r27

#define al r20
#define ah r21
#define bl r22
#define bh r23

#define zero r19

.global FPL_macs16x16_32_
.func FPL_macs16x16_32_

FPL_macs16x16_32_:

		clr   zero
		movw  r30, r20     ; z = cPtr
		movw  al,  r24     ; copy a to ah:al
		ld    c0,  z+
		ld    c1,  z+
		ld    c2,  z+
		ld    c3,  z+
		muls  ah,  bh      ; (signed)ah * (signed)bh ...
		add   c2,  r0
		adc   c3,  r1
		mul   al,  bl      ; al * bl ...
		add   c0,  r0
		adc   c1,  r1
		adc   c2,  zero
		adc   c3,  zero
		mulsu ah,  bl      ; (c3:c2:c1) += (signed)ah * bl
		sbc   c3,  zero     
		add   c1,  r0
		adc   c2,  r1
		adc   c3,  zero
		mulsu bh,  al      ; (c3:c2:c1) += (signed)bh * al
		sbc   c3,  zero
		add   c1,  r0
		adc   c2,  r1
		adc   c3,  zero
		st    -z,  c3
		st    -z,  c2
		st    -z,  c1
		st    -z,  c0											
		clr   r1           ; C-Compiler expect r1 == 0
		ret

#undef c0
#undef c1
#undef c2
#undef c3

#undef al
#undef ah
#undef bl
#undef bh

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_fmuls16x16_32_                             *
; *****************************************************************************
; *                                                                           *
; * multiplication of two 1.15-fix point numbers,  result format 1.31	      *
; *                                                                           *
; * C-header:                                                                 *
; *                                                                           *
; * int32_t FPL_fmuls16x16_32_(int16_t a, int16_t b)                          *
; *                                                                           *
; * c = (a*b)<<1                                                              *
; *                                                                           *
; * - argument a in (r25:r24)                                				  *
; * - argument b in (r23:r22)                                				  *
; * - result   c in(r25:r24:r23:r22)                     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define c0 r22
#define c1 r23
#define c2 r24
#define c3 r25

#define al r18
#define ah r19
#define bl r20
#define bh r21

#define zero r26

.global FPL_fmuls16x16_32_
.func FPL_fmuls16x16_32_

FPL_fmuls16x16_32_:

		clr    zero
		movw   al,  r24    ; copy a to ah:al
		movw   bl,  r22    ; copy b to bh:bl
		fmuls  ah,  bh     ; (signed)ah * (signed)bh ...
		movw   c2,  r0     ; => c3:c2
		fmul   al,  bl     ; al * bl ...
		adc    c2,  zero
		movw   c0,  r0     ; => c1:c0
		fmulsu ah,  bl     ; (c3:c2:c1) += (signed)ah * bl
		sbc    c3,  zero     
		add    c1,  r0
		adc    c2,  r1
		adc    c3,  zero
		fmulsu bh,  al     ; (c3:c2:c1) += (signed)bh * al
		sbc    c3,  zero
		add    c1,  r0
		adc    c2,  r1
		adc    c3,  zero												
		clr    r1          ; C-Compiler expect r1 == 0
		ret

#undef c0
#undef c1
#undef c2
#undef c3

#undef al
#undef ah
#undef bl
#undef bh

#undef zero

.endfunc


; *****************************************************************************
; *                             FPL_fmacs16x16_32_                            *
; *****************************************************************************
; *   																		  *
; * multiplicate two fix point numbers 1.15, format to 1.31 and add to a 1.31 *
; *                                                                           *
; * C-Funktionskopf:                                                          *
; *                                                                           *
; * void FPL_fmacs16x16_32_(int16_t a, int16_t b, int32_t *cPtr)              *
; *                                                                           *
; * *cPtr += (a*b)<<1                                                         *
; *                                                                           *
; * - argument a in (r25:r24)                                				  *
; * - argument b in (r23:r22)                                 				  *
; * - result cPtr in (r21:r20)					                              *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define c0 r24
#define c1 r25
#define c2 r26
#define c3 r27

#define al r20
#define ah r21
#define bl r22
#define bh r23

#define zero r19

.global FPL_fmacs16x16_32_
.func FPL_fmacs16x16_32_

FPL_fmacs16x16_32_:

		clr    zero
		movw   r30, r20    ; z = cPtr
		movw   al,  r24    ; copy a to ah:al
		ld     c0,  z+
		ld     c1,  z+
		ld     c2,  z+
		ld     c3,  z+
		fmuls  ah,  bh     ; (signed)ah * (signed)bh ...
		add    c2,  r0
		adc    c3,  r1
		fmul   al,  bl     ; al * bl ...
		adc    c2,  zero
		add    c0,  r0
		adc    c1,  r1
		adc    c2,  zero
		adc    c3,  zero
		fmulsu ah,  bl     ; (c3:c2:c1) += (signed)ah * bl
		sbc    c3,  zero     
		add    c1,  r0
		adc    c2,  r1
		adc    c3,  zero
		fmulsu bh,  al     ; (c3:c2:c1) += (signed)bh * al
		sbc    c3,  zero
		add    c1,  r0
		adc    c2,  r1
		adc    c3,  zero
		st     -z,  c3
		st     -z,  c2
		st     -z,  c1
		st     -z,  c0											
		clr    r1          ; C-Compiler expect r1 == 0
		ret

#undef c0
#undef c1
#undef c2
#undef c3

#undef al
#undef ah
#undef bl
#undef bh

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_muls32x16_64_                              *
; *****************************************************************************
; *                                                                           *
; * multiplication of int32 and int16, result: int64		                  *
; *                                                                           *
; * C-header:     		                                                      *
; *                                                                           *
; * int64_t FPL_muls32x16_64_(int32_t a, int16_t b)                           *
; *                                                                           *
; * c = a*b                                                                   *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20)                                 				  *
; * - result   c in(r25:r24:r23:r22:r21:r20:r19:r18)     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b1 r19
#define b0 r18

#define c5 r29
#define c4 r28
#define c3 r27
#define c2 r26
#define c1 r25
#define c0 r24

#define zero r17

.global FPL_muls32x16_64_
.func FPL_muls32x16_64_

FPL_muls32x16_64_:

		push  r28
		push  r29
		push  r17
		movw  b0, r20
		movw  a0, r22
		movw  a2, r24
		clr   zero                
		mul   a0, b0
		movw  c0, r0
		mul   a2, b0
		movw  c2, r0
		muls  a3, b1
		movw  c4, r0
		mul   a1, b0
		add   c1, r0
		adc   c2, r1
		adc   c3, zero
		adc   c4, zero
		adc   c5, zero
		mulsu a3, b0
		sbc   c5, zero
		add   c3, r0
		adc   c4, r1
		adc   c5, zero
		mulsu b1, a0
		sbc   c3, zero
		sbc   c4, zero
		sbc   c5, zero
		add   c1, r0
		adc   c2, r1
		adc   c3, zero
		adc   c4, zero
		adc   c5, zero
		mulsu b1, a1
		sbc   c4, zero
		sbc   c5, zero
		add   c2, r0
		adc   c3, r1
		adc   c4, zero
		adc   c5, zero
		mulsu b1, a2
		sbc   c5, zero
		add   c3, r0
		adc   c4, r1
		adc   c5, zero
		movw  r18, c0
		movw  r20, c2
		movw  r22, c4
		sbrc  r23, 7       ; execute sign extension for HiWord
		sec                ; set Carry only if result is negativ
		sbc   r24, r24
		sbc   r25, r25                
		clr   r1           ; C-Compiler expect r1 == 0
		pop   r17          
		pop   r29         
		pop   r28          
		ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b1
#undef b0

#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_macs32x16_64_                              *
; *****************************************************************************
; *                                                                           *
; * multiplication of int32 and int16 to a int64  					          *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * void FPL_macs32x16_64_(int32_t a, int16_t b, int64_t *cPtr)               *
; *                                                                           *
; * *cPtr += a*b                                                              *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20)                               				  *
; * - result cPtr in (r19:r18)					                              *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b1 r19
#define b0 r18

#define c5 r29
#define c4 r28
#define c3 r27
#define c2 r26
#define c1 r25
#define c0 r24

#define zero r17

.global FPL_macs32x16_64_
.func FPL_macs32x16_64_

FPL_macs32x16_64_:

		push  r28
		push  r29
		push  r17
		movw  r30, r18     ; cPtr => z
		movw  b0,  r20
		movw  a0,  r22
		movw  a2,  r24
		clr   zero
		ld    c0,  z+      ; load c (= *cPtr) from SRAM
		ld    c1,  z+
		ld    c2,  z+
		ld    c3,  z+
		ld    c4,  z+
		ld    c5,  z+
		mul   a0,  b0
		add   c0,  r0
		adc   c1,  r1
		adc   c2,  zero
		adc   c3,  zero
		adc   c4,  zero
		adc   c5,  zero                
		mul   a2,  b0
		add   c2,  r0
		adc   c3,  r1
		adc   c4,  zero
		adc   c5,  zero
		muls  a3,  b1
		add   c4,  r0
		adc   c5,  r1                
		mul   a1,  b0
		add   c1,  r0
		adc   c2,  r1
		adc   c3,  zero
		adc   c4,  zero
		adc   c5,  zero
		mulsu a3,  b0
		sbc   c5,  zero
		add   c3,  r0
		adc   c4,  r1
		adc   c5,  zero
		mulsu b1,  a0
		sbc   c3,  zero
		sbc   c4,  zero
		sbc   c5,  zero
		add   c1,  r0
		adc   c2,  r1
		adc   c3,  zero
		adc   c4,  zero
		adc   c5,  zero
		mulsu b1,  a1
		sbc   c4,  zero
		sbc   c5,  zero
		add   c2,  r0
		adc   c3,  r1
		adc   c4,  zero
		adc   c5,  zero
		mulsu b1,  a2
		sbc   c5,  zero
		add   c3,  r0
		adc   c4,  r1
		adc   c5,  zero
		sbrc  c5,  7       ; execute sign extension for HiWord
		sec                ; set Carry only if result is negativ
		sbc   r18, r18     ; r18 = 00h, if Carry = 0, else FFh
		std   z+1, r18     ; c7 = r18
		st    z,   r18     ; c6 = r18
		st    -z,  c5
		st    -z,  c4
		st    -z,  c3
		st    -z,  c2
		st    -z,  c1
		st    -z,  c0               
		clr   r1           ; C-Compiler expect r1 == 0
		pop   r17          
		pop   r29         
		pop   r28          
		ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b1
#undef b0

#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                             FPL_fmuls32x16_64_                            *
; *****************************************************************************
; *                                                                           *
; * multiplication of 16.16- and 1.15-. result format is 32.32                *
; *                                                                           *
; * C-header: 		                                                          *
; *                                                                           *
; * int64_t FPL_fmuls32x16_64_(int32_t a, int16_t b)                          *
; *                                                                           *
; * c = (a*b)<<1                                                              *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20)                                 				  *
; * - result   c in(r25:r24:r23:r22:r21:r20:r19:r18)     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b1 r19
#define b0 r18

#define c5 r29
#define c4 r28
#define c3 r27
#define c2 r26
#define c1 r25
#define c0 r24

#define zero r17

.global FPL_fmuls32x16_64_
.func FPL_fmuls32x16_64_

FPL_fmuls32x16_64_:

                push   r28
				push   r29
				push   r17
				movw   b0, r20
				movw   a0, r22
				movw   a2, r24
				clr    zero                
				fmuls  a3, b1
				movw   c4, r0
                fmul   a2, b0
				adc    c4, zero
                movw   c2, r0
				fmul   a0, b0
				adc    c2, zero
                movw   c0, r0
                fmul   a1, b0
				adc    c3, zero
				add    c1, r0
				adc    c2, r1
				adc    c3, zero
				adc    c4, zero
				adc    c5, zero
				fmulsu a3, b0
				sbc    c5, zero
				add    c3, r0
				adc    c4, r1
				adc    c5, zero
				fmulsu b1, a0
                sbc    c3, zero
				sbc    c4, zero
				sbc    c5, zero
				add    c1, r0
				adc    c2, r1
				adc    c3, zero
				adc    c4, zero
				adc    c5, zero
				fmulsu b1, a1
				sbc    c4, zero
				sbc    c5, zero
				add    c2, r0
				adc    c3, r1
				adc    c4, zero
				adc    c5, zero
				fmulsu b1, a2
				sbc    c5, zero
				add    c3, r0
				adc    c4, r1
				adc    c5, zero
                movw   r18, c0
				movw   r20, c2
				movw   r22, c4
                sbrc   r23, 7      ; execute sign extension for HiWord
				sec                ; set Carry only if result is negativ
				sbc    r24, r24
				sbc    r25, r25                
				clr    r1          ; C-Compiler expect r1 == 0
				pop    r17          
				pop    r29         
				pop    r28          
                ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b1
#undef b0

#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_fmacs32x16_64_                             *
; *****************************************************************************
; *                                                                           *
; * multiplication of 16.16 and 1.15 and add to 32.32; result format is 32.32 *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * void FPL_fmacs32x16_64_(int32_t a, int16_t b, int64_t *cPtr)              *
; *                                                                           *
; * *cPtr += (a*b)<<1                                                         *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                        				  *
; * - argument b in (r21:r20)                                				  *
; * - result cPtr in (r19:r18)                             					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b1 r19
#define b0 r18

#define c5 r29
#define c4 r28
#define c3 r27
#define c2 r26
#define c1 r25
#define c0 r24

#define zero r17

.global FPL_fmacs32x16_64_
.func FPL_fmacs32x16_64_

FPL_fmacs32x16_64_:

                push   r28
				push   r29
				push   r17
				movw   r30, r18    ; cPtr => z
				movw   b0,  r20
				movw   a0,  r22
				movw   a2,  r24
				clr    zero
                ld     c0,  z+     ; load c (= *cPtr) from SRAM
				ld     c1,  z+
				ld     c2,  z+
				ld     c3,  z+
				ld     c4,  z+
				ld     c5,  z+
				fmul   a0,  b0
				adc    c2,  zero
                add    c0,  r0
				adc    c1,  r1
				adc    c2,  zero
				adc    c3,  zero
				adc    c4,  zero
				adc    c5,  zero                
				fmul   a2,  b0
				adc    c4,  zero
                add    c2,  r0
				adc    c3,  r1
				adc    c4,  zero
				adc    c5,  zero
				fmuls  a3,  b1
                add    c4,  r0
				adc    c5,  r1                
				fmul   a1,  b0
				adc    c3,  zero
				add    c1,  r0
				adc    c2,  r1
				adc    c3,  zero
				adc    c4,  zero
				adc    c5,  zero
				fmulsu a3,  b0
				sbc    c5,  zero
				add    c3,  r0
				adc    c4,  r1
				adc    c5,  zero
				fmulsu b1,  a0
                sbc    c3,  zero
				sbc    c4,  zero
				sbc    c5,  zero
				add    c1,  r0
				adc    c2,  r1
				adc    c3,  zero
				adc    c4,  zero
				adc    c5,  zero
				fmulsu b1,  a1
				sbc    c4,  zero
				sbc    c5,  zero
				add    c2,  r0
				adc    c3,  r1
				adc    c4,  zero
				adc    c5,  zero
				fmulsu b1,  a2
				sbc    c5,  zero
				add    c3,  r0
				adc    c4,  r1
				adc    c5,  zero
                sbrc   c5,  7      ; execute sign extension for HiWord
				sec                ; set Carry only if result is negativ
				sbc    r18, r18    ; r18 = 00h, if Carry = 0, else FFh
                std    z+1, r18    ; c7 = r18
				st     z,   r18    ; c6 = r18
                st     -z,  c5
				st     -z,  c4
				st     -z,  c3
				st     -z,  c2
				st     -z,  c1
				st     -z,  c0               
				clr    r1          ; C-Compiler expect r1 == 0
				pop    r17          
				pop    r29         
				pop    r28          
                ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b1
#undef b0

#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_muls32x32_64_                              *
; *****************************************************************************
; *                                                                           *
; * multiplication of two int32, result: int64		                          *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * int64_t FPL_muls32x32_64_(int32_t a, int32_t b)                           *
; *                                                                           *
; * c = a*b                                                                   *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20:r19:r18)                         				  *
; * - result   c in(r25:r24:r23:r22:r21:r20:r19:r18)     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b3 r19
#define b2 r18
#define b1 r17
#define b0 r16

#define c7 r31
#define c6 r30
#define c5 r29
#define c4 r28
#define c3 r27
#define c2 r26
#define c1 r25
#define c0 r24

#define zero r15


.global FPL_muls32x32_64_
.func FPL_muls32x32_64_

FPL_muls32x32_64_:

                push  r29
				push  r28
				push  r17
				push  r16
				push  r15
                movw  r16, r18
				movw  r18, r20
				movw  r20, r22
				movw  r22, r24
				clr   zero
                mul   a0, b0
                movw  c0, r0
				mul   a2, b0
				movw  c2, r0
                mul   a2, b2
				movw  c4, r0
				muls  a3, b3
				movw  c6, r0
				mul   a1, b0
				add   c1, r0
				adc   c2, r1
				adc   c3, zero
				adc   c4, zero
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mulsu a3, b0
				sbc   c5, zero
				sbc   c6, zero
				sbc   c7, zero
				add   c3, r0
				adc   c4, r1
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mul   a0, b1
				add   c1, r0
				adc   c2, r1
				adc   c3, zero
				adc   c4, zero
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mul   a1, b1
				add   c2, r0
				adc   c3, r1
				adc   c4, zero
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mul   a2, b1
				add   c3, r0
				adc   c4, r1
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mulsu a3, b1
				sbc   c6, zero
				sbc   c7, zero
				add   c4, r0
				adc   c5, r1
				adc   c6, zero
				adc   c7, zero
				mul   a0, b2
				add   c2, r0
				adc   c3, r1
				adc   c4, zero
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mul   a1, b2
				add   c3, r0
				adc   c4, r1
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mulsu a3, b2
				sbc   c7, zero
				add   c5, r0
				adc   c6, r1
				adc   c7, zero
				mulsu b3, a0
				sbc   c5, zero
				sbc   c6, zero
				sbc   c7, zero
				add   c3, r0
				adc   c4, r1
				adc   c5, zero
				adc   c6, zero
				adc   c7, zero
				mulsu b3, a1
				sbc   c6, zero
				sbc   c7, zero
				add   c4, r0
				adc   c5, r1
				adc   c6, zero
				adc   c7, zero
				mulsu b3, a2
				sbc   c7, zero
				add   c5, r0
				adc   c6, r1
				adc   c7, zero
                movw  r18, c0
                movw  r20, c2
				movw  r22, c4
				movw  r24, c6
				clr   r1           ; C-Compiler expect r1 == 0
				pop   r15
				pop   r16
				pop   r17
				pop   r28
				pop   r29				
                ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b3
#undef b2
#undef b1
#undef b0

#undef c7
#undef c6
#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_fmuls32x32_64_                             *
; *****************************************************************************
; *                                                                           *
; * multiplication of 16.16 and 17.15, result format is 32.32                 *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * int64_t FPL_fmuls32x32_64_(int32_t a, int32_t b)                          *
; *                                                                           *
; * c = (a*b)<<1                                                              *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20:r19:r18)                        				  *
; * - result   c in(r25:r24:r23:r22:r21:r20:r19:r18)     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b3 r19
#define b2 r18
#define b1 r17
#define b0 r16

#define c7 r31
#define c6 r30
#define c5 r29
#define c4 r28
#define c3 r27
#define c2 r26
#define c1 r25
#define c0 r24

#define zero r15


.global FPL_fmuls32x32_64_
.func FPL_fmuls32x32_64_

FPL_fmuls32x32_64_:

                push   r29
				push   r28
				push   r17
				push   r16
				push   r15
                movw   r16, r18
				movw   r18, r20
				movw   r20, r22
				movw   r22, r24
				clr    zero
				fmuls  a3, b3
				movw   c6, r0
                fmul   a2, b2
				adc    c6, zero
				movw   c4, r0
				fmul   a2, b0
				adc    c4, zero
				movw   c2, r0
                fmul   a0, b0
				adc    c2, zero
                movw   c0, r0
				fmul   a1, b0
				adc    c3, zero
				add    c1, r0
				adc    c2, r1
				adc    c3, zero
				adc    c4, zero
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmulsu a3, b0
				sbc    c5, zero
				sbc    c6, zero
				sbc    c7, zero
				add    c3, r0
				adc    c4, r1
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmul   a0, b1
				adc    c3, zero
				add    c1, r0
				adc    c2, r1
				adc    c3, zero
				adc    c4, zero
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmul   a1, b1
				adc    c4, zero
				add    c2, r0
				adc    c3, r1
				adc    c4, zero
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmul   a2, b1
				adc    c5, zero
				add    c3, r0
				adc    c4, r1
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmulsu a3, b1
				sbc    c6, zero
				sbc    c7, zero
				add    c4, r0
				adc    c5, r1
				adc    c6, zero
				adc    c7, zero
				fmul   a0, b2
				adc    c4, zero
				add    c2, r0
				adc    c3, r1
				adc    c4, zero
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmul   a1, b2
				adc    c5, zero
				add    c3, r0
				adc    c4, r1
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmulsu a3, b2
				sbc    c7, zero
				add    c5, r0
				adc    c6, r1
				adc    c7, zero
				fmulsu b3, a0
				sbc    c5, zero
				sbc    c6, zero
				sbc    c7, zero
				add    c3, r0
				adc    c4, r1
				adc    c5, zero
				adc    c6, zero
				adc    c7, zero
				fmulsu b3, a1
				sbc    c6, zero
				sbc    c7, zero
				add    c4, r0
				adc    c5, r1
				adc    c6, zero
				adc    c7, zero
				fmulsu b3, a2
				sbc    c7, zero
				add    c5, r0
				adc    c6, r1
				adc    c7, zero
                movw   r18, c0
                movw   r20, c2
				movw   r22, c4
				movw   r24, c6
				clr    r1           ; C-Compiler expect r1 == 0
				pop    r15
				pop    r16
				pop    r17
				pop    r28
				pop    r29				
                ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b3
#undef b2
#undef b1
#undef b0

#undef c7
#undef c6
#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_macs32x32_64_                              *
; *****************************************************************************
; *                                                                           *
; * multiplication of two int32 and add to a int64		                      *
; *                                                                           *
; * C-header	:                                                     	      *
; *                                                                           *
; * void FPL_macs32x32_64_(int32_t a, int32_t b, int64_t *cPtr)               *
; *                                                                           *
; * *cPtr += a*b                                                              *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20:r19:r18)                        				  *
; * - result cPtr in (r17:r16)                             					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b3 r19
#define b2 r18
#define b1 r17
#define b0 r16

#define c7 r29
#define c6 r28
#define c5 r27
#define c4 r26
#define c3 r25
#define c2 r24
#define c1 r15
#define c0 r14

#define zero r13


.global FPL_macs32x32_64_
.func FPL_macs32x32_64_

FPL_macs32x32_64_:

                push  r29
				push  r28
				push  r15
				push  r14
				push  r13
				movw  r30, r16     ; z = cPtr                
                movw  b0,  r18
				movw  b2,  r20
				movw  a0,  r22
				movw  a2,  r24
				clr   zero
				ld    c0,  z+      ; c = *z = *cPtr
				ld    c1,  z+
				ld    c2,  z+
				ld    c3,  z+
				ld    c4,  z+
				ld    c5,  z+
				ld    c6,  z+
				ld    c7,  z+                
                mul   a0,  b0
                add   c0,  r0
				adc   c1,  r1
				adc   c2,  zero
				adc   c3,  zero
				adc   c4,  zero
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mul   a2,  b0
				add   c2,  r0
                adc   c3,  r1
				adc   c4,  zero
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
                mul   a2,  b2
				add   c4,  r0
				adc   c5,  r1
				adc   c6,  zero
				adc   c7,  zero
				muls  a3,  b3
				add   c6,  r0
				adc   c7,  r1
				mul   a1,  b0
				add   c1,  r0
				adc   c2,  r1
				adc   c3,  zero
				adc   c4,  zero
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mulsu a3,  b0
				sbc   c5,  zero
				sbc   c6,  zero
				sbc   c7,  zero
				add   c3,  r0
				adc   c4,  r1
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mul   a0,  b1
				add   c1,  r0
				adc   c2,  r1
				adc   c3,  zero
				adc   c4,  zero
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mul   a1,  b1
				add   c2,  r0
				adc   c3,  r1
				adc   c4,  zero
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mul   a2,  b1
				add   c3,  r0
				adc   c4,  r1
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mulsu a3,  b1
				sbc   c6,  zero
				sbc   c7,  zero
				add   c4,  r0
				adc   c5,  r1
				adc   c6,  zero
				adc   c7,  zero
				mul   a0,  b2
				add   c2,  r0
				adc   c3,  r1
				adc   c4,  zero
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mul   a1,  b2
				add   c3,  r0
				adc   c4,  r1
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mulsu a3,  b2
				sbc   c7,  zero
				add   c5,  r0
				adc   c6,  r1
				adc   c7,  zero
				mulsu b3,  a0
				sbc   c5,  zero
				sbc   c6,  zero
				sbc   c7,  zero
				add   c3,  r0
				adc   c4,  r1
				adc   c5,  zero
				adc   c6,  zero
				adc   c7,  zero
				mulsu b3,  a1
				sbc   c6,  zero
				sbc   c7,  zero
				add   c4,  r0
				adc   c5,  r1
				adc   c6,  zero
				adc   c7,  zero
				mulsu b3,  a2
				sbc   c7,  zero
				add   c5,  r0
				adc   c6,  r1
				adc   c7,  zero
                st    -z,  c7
				st    -z,  c6
				st    -z,  c5
				st    -z,  c4
				st    -z,  c3
				st    -z,  c2
				st    -z,  c1
				st    -z,  c0
				clr   r1           ; C-Compiler expect r1 == 0                							
				pop   r13
				pop   r14
				pop   r15
				pop   r28
				pop   r29
    			ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b3
#undef b2
#undef b1
#undef b0

#undef c7
#undef c6
#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                           FPL_fmacs32x32_64_                              *
; *****************************************************************************
; *                                                                           *
; * Bringt das Produkt einer 16.16- und einer 17.15-Festkommazahl auf das     *
; * Format 32.32 und addiert es zu einer 32.32-Festkommazahl hinzu            *
; *                                                                           *
; * C-Funktionskopf:                                                          *
; *                                                                           *
; * void FPL_fmacs32x32_64_(int32_t a, int32_t b, int64_t *cPtr)              *
; *                                                                           *
; * *cPtr += (a*b)<<1                                                         *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         *
; * - argument b in (r21:r20:r19:r18)                         *
; * - Argument cPtr wird uebergeben in (r17:r16)                              *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b3 r19
#define b2 r18
#define b1 r17
#define b0 r16

#define c7 r29
#define c6 r28
#define c5 r27
#define c4 r26
#define c3 r25
#define c2 r24
#define c1 r15
#define c0 r14

#define zero r13


.global FPL_fmacs32x32_64_
.func FPL_fmacs32x32_64_

FPL_fmacs32x32_64_:

                push   r29
				push   r28
				push   r15
				push   r14
				push   r13
				movw   r30, r16     ; z = cPtr                
                movw   b0,  r18
				movw   b2,  r20
				movw   a0,  r22
				movw   a2,  r24
				clr    zero
				ld     c0,  z+      ; c = *z = *cPtr
				ld     c1,  z+
				ld     c2,  z+
				ld     c3,  z+
				ld     c4,  z+
				ld     c5,  z+
				ld     c6,  z+
				ld     c7,  z+                
                fmul   a0,  b0
				adc    c2,  zero
                add    c0,  r0
				adc    c1,  r1
				adc    c2,  zero
				adc    c3,  zero
				adc    c4,  zero
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmul   a2,  b0
				adc    c4,  zero
				add    c2,  r0
                adc    c3,  r1
				adc    c4,  zero
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
                fmul   a2,  b2
				adc    c6,  zero
				add    c4,  r0
				adc    c5,  r1
				adc    c6,  zero
				adc    c7,  zero
				fmuls  a3,  b3
				add    c6,  r0
				adc    c7,  r1
				fmul   a1,  b0
				adc    c3,  zero
				add    c1,  r0
				adc    c2,  r1
				adc    c3,  zero
				adc    c4,  zero
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmulsu a3,  b0
				sbc    c5,  zero
				sbc    c6,  zero
				sbc    c7,  zero
				add    c3,  r0
				adc    c4,  r1
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmul   a0,  b1
				adc    c3,  zero
				add    c1,  r0
				adc    c2,  r1
				adc    c3,  zero
				adc    c4,  zero
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmul   a1,  b1
				adc    c4,  zero
				add    c2,  r0
				adc    c3,  r1
				adc    c4,  zero
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmul   a2,  b1
				adc    c5,  zero
				add    c3,  r0
				adc    c4,  r1
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmulsu a3,  b1
				sbc    c6,  zero
				sbc    c7,  zero
				add    c4,  r0
				adc    c5,  r1
				adc    c6,  zero
				adc    c7,  zero
				fmul   a0,  b2
				adc    c4,  zero
				add    c2,  r0
				adc    c3,  r1
				adc    c4,  zero
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmul   a1,  b2
				adc    c5,  zero
				add    c3,  r0
				adc    c4,  r1
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmulsu a3,  b2
				sbc    c7,  zero
				add    c5,  r0
				adc    c6,  r1
				adc    c7,  zero
				fmulsu b3,  a0
				sbc    c5,  zero
				sbc    c6,  zero
				sbc    c7,  zero
				add    c3,  r0
				adc    c4,  r1
				adc    c5,  zero
				adc    c6,  zero
				adc    c7,  zero
				fmulsu b3,  a1
				sbc    c6,  zero
				sbc    c7,  zero
				add    c4,  r0
				adc    c5,  r1
				adc    c6,  zero
				adc    c7,  zero
				fmulsu b3,  a2
				sbc    c7,  zero
				add    c5,  r0
				adc    c6,  r1
				adc    c7,  zero
                st     -z,  c7
				st     -z,  c6
				st     -z,  c5
				st     -z,  c4
				st     -z,  c3
				st     -z,  c2
				st     -z,  c1
				st     -z,  c0
				clr    r1           ; C-Compiler expect r1 == 0
				pop    r13
				pop    r14
				pop    r15
				pop    r28
				pop    r29
    			ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b3
#undef b2
#undef b1
#undef b0

#undef c7
#undef c6
#undef c5
#undef c4
#undef c3
#undef c2
#undef c1
#undef c0

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_muls32x32_32_                              *
; *****************************************************************************
; *                                                                           *
; * multiplication of two int32, result: int32                                *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * int32_t FPL_muls32x32_32_(int32_t a, int32_t b)                           *
; *                                                                           *
; * c = (a*b)>>16                                                             *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20:r19:r18)                         				  *
; * - result   c in(r25:r24:r23:r22)                     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define a3 r23
#define a2 r22
#define a1 r21
#define a0 r20

#define b3 r19
#define b2 r18
#define b1 r17
#define b0 r16

#define c5 r29
#define c4 r28
#define c3 r27
#define c2 r26
#define c1 r25

#define zero r30

.global FPL_muls32x32_32_
.func FPL_muls32x32_32_

FPL_muls32x32_32_:

                push  r29
				push  r28
				push  r17
				push  r16
				clr   zero
                movw  r16, r18
				movw  r18, r20
				movw  r20, r22
				movw  r22, r24
                mul   a0, b1
				mov   c1, r0
				mov   c2, r1
                mul   a1, b2
				mov   c3, r0
				mov   c4, r1
				clr   c5				
				mul   a0, b0
				add   c1, r1
				adc   c2, zero
                adc   c3, zero
				adc   c4, zero
				adc   c5, zero
                mul   a0, b2
				add   c2, r0
				adc   c3, r1
				adc   c4, zero
				adc   c5, zero
				mulsu b3, a0
				sbc   c5, zero
				add   c3, r0
				adc   c4, r1
                adc   c5, zero
				mul   a1, b0
				add   c1, r0
				adc   c2, r1
				adc   c3, zero
				adc   c4, zero
				adc   c5, zero
                mul   a1, b1
				add   c2, r0
				adc   c3, r1
				adc   c4, zero
				adc   c5, zero
                mulsu b3, a1			
				add   c4, r0
				adc   c5, r1
				mul   a2, b0
				add   c2, r0
				adc   c3, r1
				adc   c4, zero
				adc   c5, zero				
				mul   a2, b1
				add   c3, r0
				adc   c4, r1
				adc   c5, zero
				mul   a2, b2
				add   c4, r0
				adc   c5, r1
				mulsu b3, a2
				add   c5, r0				
				mulsu a3, b0
				sbc   c5, zero
				add   c3, r0
				adc   c4, r1
				adc   c5, zero				
				mulsu a3, b1
				add   c4, r0
				adc   c5, r1				
                mulsu a3, b2
				add   c5, r0 				          
				subi  c1, 0x80     ; cut the lower 16 Bit by rounding
			    sbci  c2, 0xFF
				sbci  c3, 0xFF							
				sbci  c4, 0xFF
				sbci  c5, 0xFF				
				movw  r22, c2
				movw  r24, c4
				clr   r1           ; C-Compiler expect r1 == 0
				pop   r16
				pop   r17
				pop   r28
				pop   r29				
                ret

#undef a3
#undef a2
#undef a1
#undef a0

#undef b3
#undef b2
#undef b1
#undef b0

#undef c5
#undef c4
#undef c3
#undef c2
#undef c1

#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_divs32_16_32_                              *
; *****************************************************************************
; *                                                                           *
; * int 32 division by int16, result: int32					                  *
; * result is rounded         					                              *
; *                                                                           *
; * C-Funktionskopf:                                                          *
; *                                                                           *
; * int32_t FPL_divs32_16_32_(int32_t a, int16_t b)                           *
; *                                                                           *
; * c = round(a / b)                                                          *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22)                         				  *
; * - argument b in (r21:r20)                                 				  *
; * - result   c in(r25:r24:r23:r22)                     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define rh   r27
#define rl   r26
#define q3   r25
#define q2   r24
#define q1   r23
#define q0   r22
#define dh   r21
#define dl   r20
#define zero r1


.global FPL_divs32_16_32_
.func FPL_divs32_16_32_

FPL_divs32_16_32_:

                mov  r18, r25      ; determin the sign of the quotient: a3 => r18
				eor  r18, r21      ; r18:= r18 xor b3 = a3 xor b3
                bst  r18, 7        ; SREG.T = r18.7 = sign of quotient
				tst  r25           ; a negativ ?
				brpl divs32_16_0   ; no, go on.
                com  r25           ; yes, sum is complement on two of Arg. a
				com  r24
				com  r23
				neg  r22
				sbci r23, 0xFF
				sbci r24, 0xFF
				sbci r25, 0xFF
divs32_16_0:    tst  r21           ; b negativ ?
                brpl divs32_16_1   ; no, go on.
                com  r21           ; yes, sum is complement on two of Arg. a
				neg  r20
				sbci r21, 0xFF
divs32_16_1:    movw r18, dl       ; divisor b = (dh:dl) to (r19:r18)
				lsr  r19           ; divisor in (r19:r18) cut in half
				ror  r18
                add  r22, r18      ; dividend += divisor/2
				adc  r23, r19
				adc  r24, zero
				adc  r25, zero
                clr  rl            ; delete the rest (rh:rl)
				clr  rh
                lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_0
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_0:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_1
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_1:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_2
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_2:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_3
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_3:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_4
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_4:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_5
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_5:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_6
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_6:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_7
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_7:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_8
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_8:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_9
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_9:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_A
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_A:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_B
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_B:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_C
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_C:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_D
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_D:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_E
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_E:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_F
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_F:  lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_10
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_10: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_11
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_11: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_12
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_12: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_13
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_13: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_14
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_14: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_15
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_15: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_16
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_16: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_17
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_17: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_18
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_18: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_19
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_19: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_1A
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_1A: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_1B
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_1B: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_1C
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_1C: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_1D
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_1D: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_1E
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_1E: lsl  q0
				rol  q1
				rol  q2
				rol  q3
				rol  rl
				rol  rh
				cp   rl, dl
				cpc  rh, dh
				brcs DontSub3216_1F
				sub  rl, dl
				sbc  rh, dh
				ori  q0, 1
DontSub3216_1F: brtc divs32_16_2   ; SREG.T contains the sign of the quotienten
                com  q3            ; SREG.T is set, therefore negate the result
				com  q2
				com  q1
				neg  q0
				sbci q1, 0xFF
				sbci q2, 0xFF
				sbci q3, 0xFF                
divs32_16_2:    ret

#undef rh
#undef rl
#undef q3
#undef q2
#undef q1
#undef q0
#undef dh
#undef dl
#undef zero

.endfunc


; *****************************************************************************
; *                            FPL_divs64_32_64_                              *
; *****************************************************************************
; *                                                                           *
; * int64 division by int32, result: int64					                  *
; * result is rounded					                                      *
; *                                                                           *
; * C-header: 		                                                          *
; *                                                                           *
; * int64_t FPL_divs64_32_64_(int64_t a, int32_t b)                           *
; *                                                                           *
; * c = round(a / b)                                                          *
; *                                                                           *
; * - argument a in (r25:r24:r23:r22:r21:r20:r19:r18)         				  *
; * - argument b in (r17:r16:r15:r14)                         				  *
; * - result   c in (r25:r24:r23:r22:r21:r20:r19:r18)     					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define rm3  r29
#define rm2  r28
#define rm1  r27
#define rm0  r26
#define q7   r25
#define q6   r24
#define q5   r23
#define q4   r22
#define q3   r21
#define q2   r20
#define q1   r19
#define q0   r18
#define d3   r17
#define d2   r16
#define d1   r15
#define d0   r14
#define zero r1
#define lc   r30

.global FPL_divs64_32_64_
.func FPL_divs64_32_64_

FPL_divs64_32_64_:

                push r29
				push r28
                mov  r26, r25      ; determin the sign of the quotient: a7 => r26
				eor  r26, r17      ; r26:= r26 xor b3 = a7 xor b3
                bst  r26, 7        ; SREG.T = r26.7 = sign of the quotient
				tst  r25           ; a negativ ?
				brpl divs64_32_0   ; no, go on
                com  r25           ; yes, sum is complement on two of Arg. a
				com  r24
				com  r23
				com  r22
				com  r21
				com  r20
				com  r19
				neg  r18
				sbci r19, 0xFF
				sbci r20, 0xFF
				sbci r21, 0xFF
				sbci r22, 0xFF
				sbci r23, 0xFF
				sbci r24, 0xFF
				sbci r25, 0xFF
divs64_32_0:    tst  r17           ; b negativ ?
                brpl divs64_32_1   ; no, go on.
                movw r26, r14
				movw r28, r16
				com  r29           ; yes, sum is complement on two of Arg. b
				com  r28
				com  r27
				neg  r26
				sbci r27, 0xFF
				sbci r28, 0xFF
				sbci r29, 0xFF
                movw r14, r26
				movw r16, r28
divs64_32_1:    movw r26, r14      ; copy b to in (r29:r28:r27:r26),
				movw r28, r16
				lsr  r29           ; ... cut on half ...
				ror  r28
				ror  r27
				ror  r26
                add  q0, r26       ; ... add to dividend
				adc  q1, r27
				adc  q2, r28
				adc  q3, r29
				adc  q4, zero
				adc  q5, zero
				adc  q6, zero
				adc  q7, zero
                clr  rm0           ; delete the Rest (rm3:rm2:rm1:rm0)
				clr  rm1
				movw rm2,rm0
				ldi  lc, 64        ; 64 iterations
LoopEntry6432:  lsl  q0            ; quotient and partial rest <<= 1
				rol  q1
				rol  q2
				rol  q3
				rol  q4
				rol  q5
				rol  q6
				rol  q7
				rol  rm0
				rol  rm1
				rol  rm2
				rol  rm3
				cp   rm0, d0       ; compare partial rest with divisor
				cpc  rm1, d1
				cpc  rm2, d2
				cpc  rm3, d3
				brcs DontSub6432   ; rest < divisor, do not subtract divisor
				sub  rm0, d0       ; rest >= divisor, subtract divisor
				sbc  rm1, d1
				sbc  rm2, d2
				sbc  rm3, d3
				ori  q0, 1         ; set quotient.Bit0
DontSub6432:    dec  lc            ; counter --
                brne LoopEntry6432 ; if necessary next iteration
                brtc divs64_32_2   ; SREG.T contains the sign of the quotienten
                com  q7            ; SREG.T is set, therefore result is ...
				com  q6            ; ... by complement on two ...
				com  q5            ; ... to negate
				com  q4
				com  q3
				com  q2
				com  q1
				neg  q0
				sbci q1, 0xFF
				sbci q2, 0xFF
				sbci q3, 0xFF
				sbci q4, 0xFF
				sbci q5, 0xFF
				sbci q6, 0xFF
				sbci q7, 0xFF
divs64_32_2:    pop  r28
				pop  r29
				ret

#undef rm3
#undef rm2
#undef rm1
#undef rm0
#undef q7
#undef q6
#undef q5
#undef q4
#undef q3
#undef q2
#undef q1
#undef q0
#undef d3
#undef d2
#undef d1
#undef d0
#undef zero
#undef lc

.endfunc


; *****************************************************************************
; *                            FPL_asr64_by_n_                                *
; *****************************************************************************
; *                                                                           *
; * artimetic shift with a int64 about n bit right 							  *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * void FPL_asr64_by_n_(int64_t *xPtr, uint8_t n)                            *
; *                                                                           *
; * *xPtr >>= n                                                               *
; *                                                                           *
; * - argument xPtr  in (r25:r24) 				                              *
; * - argument n  in r22                                       				  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define n    r27
#define w    r26
#define x7   r25
#define x6   r24
#define x5   r23
#define x4   r22
#define x3   r21
#define x2   r20
#define x1   r19
#define x0   r18

.global FPL_asr64_by_n_
.func FPL_asr64_by_n_

FPL_asr64_by_n_:

              movw r30,r24         ; z = (r25:r24)
              mov  n,  r22
              mov  w, n
			  ld   x0, z+
			  ld   x1, z+
			  ld   x2, z+
			  ld   x3, z+
			  ld   x4, z+
			  ld   x5, z+
			  ld   x6, z+
			  ld   x7, z+
			  andi w,  0x07        ; n mod 8 = numbers of shifts w
			  breq asr64_32        ; w = 0: no shifts, go on
			  sub  n,  w           ; n = n - (n mod 8)
			  cpi  w,  4           ; if w > 4 shifts: shift left faster
			  brlo asr64_sr        ; less than 4 shifts, --> shift right
              neg  w               ; w = -w ...
			  subi w, -8           ; ... +8 equals number of shift left
              subi n, -8           ; n += 8, additional 8 shifts right
asr64_sl:     lsl  x0              ; shift left by w positions
              rol  x1
			  rol  x2
			  rol  x3
			  rol  x4
			  rol  x5
			  rol  x6
			  rol  x7
			  dec  w
			  brne asr64_sl        ; continue with shifts by >= 8 positions 
              rjmp asr64_32
asr64_sr:     asr  x7              ; shift right by w positions
              ror  x6
			  ror  x5
			  ror  x4
			  ror  x3
			  ror  x2
			  ror  x1
			  ror  x0
			  dec  w
			  brne asr64_sr
asr64_32:     cpi  n,  32          ; n >= 32 ?
			  brlo asr64_16        ; no: check if n >= 16
              subi n,  32          ; yes: n -= 32, ...
			  movw x0, x4          ; HiWord to LoWord ...
			  movw x2, x6          ; copy, if necessary execute sign extension...
              sbrc x3, 7           ; durchfuehren: if LoWord negativ ...
			  sec                  ; set Carry , if positiv, do not set
			  sbc  x4, x4          ; if C = 0: result 00h, else FFh
			  sbc  x5, x5          ; => sign extension
			  sbc  x6, x6
			  sbc  x7, x7
asr64_16:     cpi  n,  16
              brlo asr64_8
              subi n,  16
			  movw x0, x2
			  movw x2, x4
			  movw x4, x6
			  sbrc x5, 7
			  sec
			  sbc  x6, x6
			  sbc  x7, x7			  
asr64_8:      cpi  n,  8
              brlo asr64_end
              subi n,  8
			  mov  x0, x1
			  mov  x1, x2
			  mov  x2, x3
			  mov  x3, x4
			  mov  x4, x5
			  mov  x5, x6
			  mov  x6, x7
              sbrc x6, 7
			  sec
			  sbc  x7, x7
asr64_end:    st   -z, x7
              st   -z, x6
			  st   -z, x5
			  st   -z, x4
			  st   -z, x3
			  st   -z, x2
			  st   -z, x1
			  st   -z, x0             
			  ret

#undef n
#undef w
#undef x7
#undef x6
#undef x5
#undef x4
#undef x3
#undef x2
#undef x1
#undef x0

.endfunc


; *****************************************************************************
; *                             FPL_sl64_by_n_                                *
; *****************************************************************************
; *                                                                           *
; * left shift about n bits with a int64					                  *
; *                                                                           *
; * C-header:                                                         		  *
; *                                                                           *
; * void FPL_sl64_by_n_(int64_t *xPtr, uint8_t n)                             *
; *                                                                           *
; * *xPtr <<= n                                                               *
; *                                                                           *
; * - argument xPtr  in (r25:r24)                              				  *
; * - argument n  in r22                                      				  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define n    r27
#define w    r26
#define x7   r25
#define x6   r24
#define x5   r23
#define x4   r22
#define x3   r21
#define x2   r20
#define x1   r19
#define x0   r18

.global FPL_sl64_by_n_
.func FPL_sl64_by_n_

FPL_sl64_by_n_:

              movw r30,r24         ; z = (r25:r24)
              mov  n,  r22
              mov  w, n
			  ld   x0, z+
			  ld   x1, z+
			  ld   x2, z+
			  ld   x3, z+
			  ld   x4, z+
			  ld   x5, z+
			  ld   x6, z+
			  ld   x7, z+
			  andi w, 0x07         ; n mod 8 = number of shifts w
			  breq sl64_32         ; w = 0: no single shifts, go on
			  sub  n, w            ; n = n - (n mod 8)
			  cpi  w, 4            ; if w > 4 shifts: shit to the right faster
			  brlo sl64_sl         ; less than 4 shifts, than shift to the left
              neg  w               ; w = -w ...
			  subi w, -8           ; ... +8 equals number of shift left
              subi n, -8           ; n += 8, additional 8 shifts right
sl64_sr:      asr  x7              ; shift right by w positions
              ror  x6
			  ror  x5
			  ror  x4
			  ror  x3
			  ror  x2
			  ror  x1
			  ror  x0
			  dec  w
			  brne sl64_sr         ; continue by shifts >= 8 Positionen 
              rjmp sl64_32
sl64_sl:      lsl  x0              ; shift left by w positions
              rol  x1
			  rol  x2
			  rol  x3
			  rol  x4
			  rol  x5
			  rol  x6
			  rol  x7
			  dec  w
			  brne sl64_sl
sl64_32:      cpi  n, 32           ; n >= 32 ?
			  brlo sl64_16         ; no: check if n >= 16
              subi n, 32           ; yes: n -= 32, ...
			  movw x4, x0          ; copy LoWord to HiWord ...
			  movw x6, x2          ; and set ...
              clr  x0              ; ... LoWord ...
			  clr  x1              ; ... to 0 ...
			  movw x2, x0          
sl64_16:      cpi  n, 16
              brlo sl64_8
              subi n, 16
			  movw x6, x4
			  movw x4, x2
			  movw x2, x0
			  clr  x0
			  clr  x1
sl64_8:       cpi  n, 8
              brlo sl64_end
              subi n, 8
			  mov  x7, x6
			  mov  x6, x5
			  mov  x5, x4
			  mov  x4, x3
			  mov  x3, x2
			  mov  x2, x1
			  mov  x1, x0
              clr  x0
sl64_end:     st   -z, x7
              st   -z, x6
			  st   -z, x5
			  st   -z, x4
			  st   -z, x3
			  st   -z, x2
			  st   -z, x1
			  st   -z, x0
              ret

#undef n
#undef w
#undef x7
#undef x6
#undef x5
#undef x4
#undef x3
#undef x2
#undef x1
#undef x0

.endfunc


; *****************************************************************************
; *                            FPL_add64_64_64_                               *
; *****************************************************************************
; *                                                                           *
; * addition of two int64, result: int64                                      *
; *                                                                           *
; * C-header:		                                                          *
; *                                                                           *
; * void FPL_add64_64_64_(int64_t *aPtr, int64_t b)                           *
; *                                                                           *
; * *aPtr := *aPtr + b                                                        *
; *                                                                           *
; * - argument aPtr in (r25:r24)                              				  *
; * - argument b    in (r23:r22:r21:r20:r19:r18:r17:r16)      				  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define b7   r23
#define b6   r22
#define b5   r21
#define b4   r20
#define b3   r19
#define b2   r18
#define b1   r17
#define b0   r16
#define w    r24

.global FPL_add64_64_64_
.func FPL_add64_64_64_

FPL_add64_64_64_:

              movw r30, r24        ; z = aPtr
			  ld   w,   z+         ; w = *aPtr++ = a0
			  add  b0,  w          ; b0 := b0 + a0
			  ld   w,   z+         ; w = *aPtr++ = a1
			  adc  b1,  w          ; b1 := b1 + a1 + Carry
			  ld   w,   z+
			  adc  b2,  w          ; ...
              ld   w,   z+
			  adc  b3,  w
			  ld   w,   z+
			  adc  b4,  w
			  ld   w,   z+
			  adc  b5,  w
			  ld   w,   z+
			  adc  b6,  w
			  ld   w,   z+
			  adc  b7,  w
			  st   -z,  b7         ; *(--aPtr) = b7
			  st   -z,  b6         ; *(--aPtr) = b6
			  st   -z,  b5         ; ...
			  st   -z,  b4
			  st   -z,  b3
			  st   -z,  b2
			  st   -z,  b1
			  st   -z,  b0
			  ret

#undef b7
#undef b6
#undef b5
#undef b4
#undef b3
#undef b2
#undef b1
#undef b0
#undef w

.endfunc


; *****************************************************************************
; *                            FPL_sub64_64_64_                               *
; *****************************************************************************
; *                                                                           *
; * subtract two int64, result: int64		                                  *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * void FPL_sub64_64_64_(int64_t *aPtr, int64_t b)                           *
; *                                                                           *
; * *aPtr := *aPtr - b                                                        *
; *                                                                           *
; * - argument aPtr in (r25:r24)                              				  *
; * - argument b in (r23:r22:r21:r20:r19:r18:r17:r16)        				  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define b7   r23
#define b6   r22
#define b5   r21
#define b4   r20
#define b3   r19
#define b2   r18
#define b1   r17
#define b0   r16
#define w    r24

.global FPL_sub64_64_64_
.func FPL_sub64_64_64_

FPL_sub64_64_64_:

			  movw r30, r24        ; z = aPtr
			  ld   w,   z          ; w = *z = *aPtr = a0
              sub  w,   b0         ; a0 := a0 - b0
			  st   z+,  w          ; *aPtr++ = a0
              ld   w,   z          ; w = *z = *aPtr = a1
			  sbc  w,   b1         ; a1 := a1 - b1 - Carry
			  st   z+,  w          ; *aPtr++ = a1
			  ld   w,   z          ; ...
			  sbc  w,   b2
              st   z+,  w
			  ld   w,   z
			  sbc  w,   b3
              st   z+,  w
			  ld   w,   z
			  sbc  w,   b4
              st   z+,  w
			  ld   w,   z
			  sbc  w,   b5
              st   z+,  w
			  ld   w,   z
			  sbc  w,   b6
			  st   z+,  w
			  ld   w,   z
			  sbc  w,   b7
			  st   z,   w
			  ret

#undef b7
#undef b6
#undef b5
#undef b4
#undef b3
#undef b2
#undef b1
#undef b0
#undef w

.endfunc


; *****************************************************************************
; *                               FPL_neg64_64_                               *
; *****************************************************************************
; *                                                                           *
; * negate a int64		                                                      *
; *                                                                           *
; * C-header:                                    	                          *
; *                                                                           *
; * void FPL_neg64_64_(int64_t *xPtr)                                         *
; *                                                                           *
; * *xPtr := -(*xPtr)                                                         *
; *                                                                           *
; * - argument xPtr in (r25:r24)				                              *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define x0 r18
#define x1 r19
#define x2 r20
#define x3 r21
#define x4 r22
#define x5 r23
#define x6 r24
#define x7 r25

.global FPL_neg64_64_
.func FPL_neg64_64_

FPL_neg64_64_:

			  movw r30, r24        ; z = xPtr
              ld   x0,  z+
              ld   x1,  z+
			  ld   x2,  z+
			  ld   x3,  z+
			  ld   x4,  z+
			  ld   x5,  z+
			  ld   x6,  z+
			  ld   x7,  z+
              com  x7
			  com  x6
			  com  x5
			  com  x4
			  com  x3
			  com  x2
			  com  x1
			  neg  x0
			  sbci x1, 0xFF
			  sbci x2, 0xFF
			  sbci x3, 0xFF
			  sbci x4, 0xFF
			  sbci x5, 0xFF
			  sbci x6, 0xFF
			  sbci x7, 0xFF
              st   -z, x7
			  st   -z, x6
			  st   -z, x5
			  st   -z, x4
			  st   -z, x3
			  st   -z, x2
			  st   -z, x1
			  st   -z, x0			                
			  ret

#undef x0
#undef x1
#undef x2
#undef x3
#undef x4
#undef x5
#undef x6
#undef x7

.endfunc


; *****************************************************************************
; *                              FPL_xtract64_32_                             *
; *****************************************************************************
; *                                                                           *
; * extract a int32 from a int64, result is rounded!		                  *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * int32_t FPL_xtract64_32_(int64_t *xPtr, uint8_t bytePos)                  *
; *                                                                           *
; * - argument xPtr in (r25:r24)				                              *
; * - argument bytePos in r22         				                          *
; * - result y in (r25:r24:r23:r22)          					              *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define y3   r25
#define y2   r24
#define y1   r23
#define y0   r22
#define w    r21
#define p    r20
#define zero r1


.global FPL_xtract64_32_
.func FPL_xtract64_32_

FPL_xtract64_32_:

              movw r30, r24        ; z = xPtr
              mov  p,   r22        ; p = bytePos
			  add  r30, p          ; z += p
			  adc  r31, zero	                 
			  ld   y0,  z          ; int32 ab Byte p ...			  
              ldd  y1,  z+1        ; ... in y3:y2:y1:y0			  
			  ldd  y2,  z+2        ; ... laden
			  ldd  y3,  z+3			  
              cpi  p,   0          ; total number of decimal?
			  breq _xtr32_end      ; no -> no rounding necessary		  
              ld   w,   -z         ; load total number of decimal			  
			  subi w,   0x80       ; add 0.5
			  sbci y0,  0xFF       
			  sbci y1,  0xFF       
			  sbci y2,  0xFF       
			  sbci y3,  0xFF			  
_xtr32_end:   ret

#undef y3
#undef y2
#undef y1
#undef y0
#undef w
#undef p
#undef zero

.endfunc


; *****************************************************************************
; *                              FPL_xtract64_16_                             *
; *****************************************************************************
; *                                                                           *
; * extract a int16 from a int64 result is rounded				              *
; *                                                                           *
; * C-header	   :                                                          *
; *                                                                           *
; * int16_t FPL_xtract64_16_(int64_t *xPtr, uint8_t bytePos)                  *
; *                                                                           *
; * - argument xPtr in (r25:r24)                             				  *
; * - argument bytePos in r22                                 				  *
; * - result y in (r25:r24)                             					  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define y1   r25
#define y0   r24
#define w    r23
#define p    r22
#define zero r1


.global FPL_xtract64_16_
.func FPL_xtract64_16_

FPL_xtract64_16_:

              movw r30, r24        ; z = xPtr
			  add  r30, p          ; z += p
			  adc  r31, zero	                 
			  ld   y0,  z          ; int16 at Byte p ...			  
              ldd  y1,  z+1        ; ... load to y1:y0
              cpi  p,   0          ; total number of decimal?
			  breq _xtr16_end      ; no -> no rounding necessary		  
              ld   w,   -z         ; load total number of decimal			  
			  subi w,   0x80       ; add 0.5
			  sbci y0,  0xFF       
			  sbci y1,  0xFF       
_xtr16_end:   ret

#undef y1
#undef y0
#undef w
#undef p
#undef zero

.endfunc


; *****************************************************************************
; *                             FPL_insert32_64_                              *
; *****************************************************************************
; *                                                                           *
; * copy a int32 in a int64. on position bytePos							  *
; *                                                                           *
; * C-header:                                                    		      *
; *                                                                           *
; * void FPL_insert32_64_(int64_t *aPtr, int32_t b, uint8_t bytePos)          *
; *                                                                           *
; * - argument aPtr in (r25:r24)                         				      *
; * - argument b in (r23:r22:r21:r20)                    				      *
; * - argument bytePos in r18                          						  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define b3   r23
#define b2   r22
#define b1   r21
#define b0   r20
#define p    r18
#define zero r1

.global FPL_insert32_64_
.func FPL_insert32_64_

FPL_insert32_64_:

              movw r30, r24        ; z = aPtr
			  add  r30, p          ; z += p
			  adc  r31, zero
			  st   z+,  b0         ; store int32 at byte p
              st   z+,  b1
			  st   z+,  b2
			  st   z+,  b3
              neg  p               ; p := -p ...
              subi p,   0xFC       ;      +4 = number of Bytes for sign extension
              breq ins32_1         ; no sign extension necessary
              sbrc b3,  7          ; next step, set Carry only if, ...
			  sec                  ; ... b3.7 = 1
              sbc  b3,  b3         ; sub with Carry equals 00h or FFh
ins32_0:      st   z+,  b3         ; sign extend next high byte ...
			  dec  p               ; ... as long as a byte exist
			  brne ins32_0              
ins32_1:      ret

#undef b3
#undef b2
#undef b1
#undef b0
#undef p
#undef zero

.endfunc


; *****************************************************************************
; *                             FPL_insert16_64_                              *
; *****************************************************************************
; *                                                                           *
; * copy a int16 into a int64 on position bytePos							  *
; *                                                                           *
; * C-header:                                                          		  *
; *                                                                           *
; * void FPL_insert16_64_(int64_t *aPtr, int16_t b, uint8_t bytePos)          *
; *                                                                           *
; * - argument aPtr in (r25:r24)  				                              *
; * - argument b in (r23:r22)                     				              *
; * - argument bytePos in r20                                				  *
; *                                                                           *
; *****************************************************************************

; register allocation:

#define b1   r23
#define b0   r22
#define p    r20
#define zero r1

.global FPL_insert16_64_
.func FPL_insert16_64_

FPL_insert16_64_:

              movw r30, r24        ; z = aPtr
			  add  r30, p          ; z += p
			  adc  r31, zero
			  st   z+,  b0         ; store int16 at byte p ...			  
              st   z+,  b1         ;  ... 			  
              neg  p               ; p := -p ...
              subi p,   0xFA       ;      +6 = number of bytes for sign extension
              breq ins16_1         ; no sign extension necessary
              sbrc b1,  7          ; next step, set Carry only if, ...
			  sec                  ; ... b1.7 = 1
              sbc  b1,  b1         ; sub with Carry equals 00h or FFh
ins16_0:      st   z+,  b1         ; sign extend next high byte ...
			  dec  p               ; ... as long as a byte exist
			  brne ins16_0              
ins16_1:      ret              
			  			  
#undef b1
#undef b0
#undef p
#undef zero

.endfunc


.end

