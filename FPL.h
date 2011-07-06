/*****************************************************************************/
/* Description: fix comma routines											 */
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
All rights to the entire project and all related files and information are reserved by 1hoch4 UG.
This includes, without limitation, software published as source code.

Use of hardware:
Users are permitted to utilise the hardware for commercial purposes (e.g. aerial photography).
However, 1hoch4 UG cannot be held responsible for any damage that arises from commercial use,
as the product is an experimental hobby project in the beta phase. The hardware and software
are therefore under continuous development and cannot be expressly authorised for professional uses.
The prior consent of 1hoch4 UG is required for any commercial sale, utilisation for other purposes
(including, without limitation, the population of unpopulated PCBs), or the combination of kits
and/or circuit boards to create a marketable product.

Use of software (source code):
The software may only be used on hardware supplied by 1hoch4 UG. Use of all or part of the
published source code is only permitted for private and non-commercial purposes. The written
consent of 1hoch4 UG is required for any commercial usage or porting to different hardware.
These terms and conditions/licence also apply to all private use of the source code (even in part),
whether modified or unmodified, and the licence must be supplied with the software. In addition,
the source must be clearly identified as 1hoch4. Users modify and use the source code at their own risk.

1hoch4 UG assumes no liability whatsoever for any direct or indirect damage to persons and property.
Because the 1hoch4 projects are experimental, we cannot guarantee that they are free of faults,
complete or that they function correctly.




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



#ifndef __FPL_H
#define __FPL_H


/*****************************************************************************/
/*                                 includes                                  */
/*****************************************************************************/

#include <avr/io.h>


/*****************************************************************************/
/*                                  makros                                   */
/*****************************************************************************/

/* modulus of int16, result even at 8000h correct				             */
#define FPL_ABS_16_(x)		 (((x) == 0x8000) ? 0x7FFF : \
                     		 ((x) < 0 ? -(x):(x))       )


/* modulus of int32, result even at 80000000h correct				         */
#define FPL_ABS_32_(x) 		 (((x) == 0x80000000) ? 0x7FFFFFFF : \
                     		 ((x) < 0 ? -(x):(x))               )


/* cut the lower 16 Bit by rounding			                                 */
#define FPL_ROUND_OFF_16_(x) 	 (((x) + 0x8000) >> 16)


/* cut the lower 8 Bit by rounding	 		                                 */
#define FPL_ROUND_OFF_8_(x) 	 (((x) + 0x80) >> 8)


/* clip the value to +/-L 			                                         */
#define FPL_CLIP_TO_(x,L)	 (((x) > (L))  ? (L)  : \
                           	 ((x) < -(L)) ? -(L) : (x))


/* 8x8 signed multiplication, result 16 bit signed                           */
#define FPL_muls8x8_16_(a,b)	 ((int8_t)(a) * (int8_t)(b))

/* 8x1.7 signed fix-point multiplication, result 8.8 signed                  */                        
#define FPL_fmuls8x8_16_(a,b)	 (((int8_t)(a) * (int8_t)(b)) << 1)


/*****************************************************************************/
/*                           funktion-prototypes                             */
/*****************************************************************************/

/* 16x8 Bit signed x unsigned multiplication, result 32 Bit signed           */
int32_t FPL_mulsu16x8_32_(int16_t a, uint8_t b);

/* 16x8 Bit signed x unsigned "MultiplyAndAccumulate", accum.var. 32 Bit sig.*/
void FPL_macsu16x8_32_(int16_t a, uint8_t b, int32_t *cPtr);

/* 16x16 Bit signed multiplication, result 32 Bit signed                     */
int32_t FPL_muls16x16_32_(int16_t a, int16_t b);

/* 16x16 Bit signed "MultiplyAndAccumulate", accumulatorvar. 32 Bit signed   */
void FPL_macs16x16_32_(int16_t a, int16_t b, int32_t *cPtr);

/* (1.15)x(1.15) signed fix-point multiplication, result 1.31 signed       	 */
int32_t FPL_fmuls16x16_32_(int16_t a, int16_t b);

/* (1.15)x(1.15) signed "MultiplyAndAccumulate", accumulatorvar. 1.31 signed */
void FPL_fmacs16x16_32_(int16_t a, int16_t b, int32_t *cPtr);

/* 32x16 Bit signed mult., result 48 Bit signed, sign extension auf int64_t	 */
int64_t FPL_muls32x16_64_(int32_t a, int16_t b);

/* 32x16 Bit signed "MultiplyAndAccumulate", accumulatorvar. 64 Bit signed   */
void FPL_macs32x16_64_(int32_t a, int16_t b, int64_t *cPtr);

/* (16.16)x(1.15) signed fix-point multiplication, result 32.32 signed     	 */
int64_t FPL_fmuls32x16_64_(int32_t a, int16_t b);

/* (16.16)x(1.15) signed "MultiplyAndAccumulate", accumulator 32.32 signed   */
void FPL_fmacs32x16_64_(int32_t a, int16_t b, int64_t *cPtr);

/* 32x32 Bit signed multiplication, result 64 Bit signed                   	 */
int64_t FPL_muls32x32_64_(int32_t a, int32_t b);

/* (16.16)x(17.15) signed fix-point multiplication, result 32.32 signed    	 */
int64_t FPL_fmuls32x32_64_(int32_t a, int32_t b);

/* 32x32 Bit signed "MultiplyAndAccumulate", accumulatorvar. 64 Bit signed   */
void FPL_macs32x32_64_(int32_t a, int32_t b, int64_t *cPtr);

/* (16.16)x(17.15) signed "MultiplyAndAccumulate", accumulator 32.32 signed  */
void FPL_fmacs32x32_64_(int32_t a, int32_t b, int64_t *cPtr);

/* 32x32 Bit signed multiplication, result 32 Bit signed (die mittleren      */
/* result:  16bit 	- 	32bit 	- 	16bit									 */
/*			cut		-	result	-	rounding								 */
int32_t FPL_muls32x32_32_(int32_t a, int32_t b);

/* 32/16 Bit signed Division, result 32 Bit signed                           */
int32_t FPL_divs32_16_32_(int32_t a, int16_t b);

/* 64/32 Bit signed division, result 64 Bit signed                           */
int64_t FPL_divs64_32_64_(int64_t a, int32_t b);

/* arithmetic right shift of 64 Bit signed-value by n bits         		 	 */
void FPL_asr64_by_n_(int64_t *xPtr, uint8_t n);

/* left shift of 64 Bit signed-value by n bits		                         */
void FPL_sl64_by_n_(int64_t *xPtr, uint8_t n);

/* add of two 64 bit values                                              	 */
void FPL_add64_64_64_(int64_t *aPtr, int64_t b);

/* subtract of two 64 bit values                                          	 */
void FPL_sub64_64_64_(int64_t *aPtr, int64_t b);

/* negate of a signed 64 bit value                                       	 */
void FPL_neg64_64_(int64_t *xPtr);

/* extraction of a 32 bit signed value from a int64 at "bytePos"			 */
/* the lower bits will be cut by rounding									 */
int32_t FPL_xtract64_32_(int64_t *xPtr, uint8_t bytePos);

/* extraction of a 16 bit signed value from a int64 at "bytePos"			 */
/* the lower bits will be cut by rounding									 */
int16_t FPL_xtract64_16_(int64_t *xPtr, uint8_t bytePos);

/* insert a 32 bit signed value in a int64 at "bytePos"						 */
/* the higher bits are sign extended if necessary							 */
void FPL_insert32_64_(int64_t *aPtr, int32_t b, uint8_t bytePos);

/* insert a 16 bit signed value in a int64 at "bytePos"						 */
/* the higher bits are sign extended if necessary							 */
void FPL_insert16_64_(int64_t *aPtr, int16_t b, uint8_t bytePos); 

/* sinus value with fix point format 1.15. input angle x in "binary rad"	 */
/* Pi => 2^15																 */
int16_t FPL_sin16_(int16_t x);

/* cosine value with fix point format 1.15. input angle x in "binary rad"	 */
/* Pi => 2^15																 */
int16_t FPL_cos16_(int16_t x);

/* tangent value with fix point format 16.16. input angle x in "binary rad"	 */
/* Pi => 2^15																 */
int32_t FPL_tan16_(int16_t x);

/* arc sine value in "binary rad" input value x = fix point 1.15			 */
/* Pi/2 => 2^14																 */
int16_t FPL_arcsin16_(int16_t x);

/* arc cosine value in "binary rad" input value x = fix point 1.15			 */
/* Pi/2 => 2^14																 */
int16_t FPL_arccos16_(int16_t x);

/* "atan2" function, input "binary rad"										 */
/* Pi => 2^15																 */
int16_t FPL_atan2_16_(int16_t x, int16_t y);

/* square root from uint32. requirement x <= 7FFFFFFFh						 */
uint16_t FPL_sqrt32_(uint32_t x);


/*****************************************************************************/

#endif /* __FPL_H */
