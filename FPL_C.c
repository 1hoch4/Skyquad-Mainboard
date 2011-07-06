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



/*****************************************************************************/
/*                                 includes                                  */
/*****************************************************************************/
#include "avr/io.h"
#include "avr/pgmspace.h"
#include "FPL.h"

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/

/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/
#define SINLUT(x) 			(pgm_read_word(&FPL_SinLUT[x]))
#define ARCSINLUT(x) 		(pgm_read_word(&FPL_ArcsinLUT[x])) 
#define ARCTANLUT(x)    	(pgm_read_word(&FPL_ArctanLUT[x]))

/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/

/*****************************************************************************/
/*                             local variablen                               */
/*****************************************************************************/
static const int16_t FPL_SinLUT[256] PROGMEM =
{
    0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,
    0x0648,0x0711,0x07D9,0x08A2,0x096B,0x0A33,0x0AFB,0x0BC4,
    0x0C8C,0x0D54,0x0E1C,0x0EE4,0x0FAB,0x1073,0x113A,0x1201,
    0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,
    0x18F9,0x19BE,0x1A83,0x1B47,0x1C0C,0x1CD0,0x1D93,0x1E57,
    0x1F1A,0x1FDD,0x209F,0x2162,0x2224,0x22E5,0x23A7,0x2467,
    0x2528,0x25E8,0x26A8,0x2768,0x2827,0x28E5,0x29A4,0x2A62,
    0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3042,
    0x30FC,0x31B5,0x326E,0x3327,0x33DF,0x3497,0x354E,0x3604,
    0x36BA,0x3770,0x3825,0x38D9,0x398D,0x3A40,0x3AF3,0x3BA5,
    0x3C57,0x3D08,0x3DB8,0x3E68,0x3F17,0x3FC6,0x4074,0x4121,
    0x41CE,0x427A,0x4326,0x43D1,0x447B,0x4524,0x45CD,0x4675,
    0x471D,0x47C4,0x486A,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9E,
    0x4C40,0x4CE1,0x4D81,0x4E21,0x4EC0,0x4F5E,0x4FFB,0x5098,
    0x5134,0x51CF,0x5269,0x5303,0x539B,0x5433,0x54CA,0x5560,
    0x55F6,0x568A,0x571E,0x57B1,0x5843,0x58D4,0x5964,0x59F4,
    0x5A82,0x5B10,0x5B9D,0x5C29,0x5CB4,0x5D3E,0x5DC8,0x5E50,
    0x5ED7,0x5F5E,0x5FE4,0x6068,0x60EC,0x616F,0x61F1,0x6272,
    0x62F2,0x6371,0x63EF,0x646C,0x64E9,0x6564,0x65DE,0x6657,
    0x66D0,0x6747,0x67BD,0x6832,0x68A7,0x691A,0x698C,0x69FD,
    0x6A6E,0x6ADD,0x6B4B,0x6BB8,0x6C24,0x6C8F,0x6CF9,0x6D62,
    0x6DCA,0x6E31,0x6E97,0x6EFB,0x6F5F,0x6FC2,0x7023,0x7083,
    0x70E3,0x7141,0x719E,0x71FA,0x7255,0x72AF,0x7308,0x735F,
    0x73B6,0x740B,0x7460,0x74B3,0x7505,0x7556,0x75A6,0x75F4,
    0x7642,0x768E,0x76D9,0x7723,0x776C,0x77B4,0x77FB,0x7840,
    0x7885,0x78C8,0x790A,0x794A,0x798A,0x79C9,0x7A06,0x7A42,
    0x7A7D,0x7AB7,0x7AEF,0x7B27,0x7B5D,0x7B92,0x7BC6,0x7BF9,
    0x7C2A,0x7C5A,0x7C89,0x7CB7,0x7CE4,0x7D0F,0x7D3A,0x7D63,
    0x7D8A,0x7DB1,0x7DD6,0x7DFB,0x7E1E,0x7E3F,0x7E60,0x7E7F,
    0x7E9D,0x7EBA,0x7ED6,0x7EF0,0x7F0A,0x7F22,0x7F38,0x7F4E,
    0x7F62,0x7F75,0x7F87,0x7F98,0x7FA7,0x7FB5,0x7FC2,0x7FCE,
    0x7FD9,0x7FE2,0x7FEA,0x7FF1,0x7FF6,0x7FFA,0x7FFE,0x7FFF
};

static const int16_t FPL_ArcsinLUT[256] PROGMEM =
{
    0x0000,0x0051,0x00A3,0x00F4,0x0146,0x0198,0x01E9,0x023B,
    0x028C,0x02DE,0x0330,0x0381,0x03D3,0x0425,0x0477,0x04C9,
    0x051B,0x056D,0x05C0,0x0612,0x0664,0x06B7,0x070A,0x075C,
    0x07AF,0x0802,0x0856,0x08A9,0x08FC,0x0950,0x09A4,0x09F7,
    0x0A4C,0x0AA0,0x0AF4,0x0B49,0x0B9E,0x0BF3,0x0C48,0x0C9D,
    0x0CF3,0x0D49,0x0D9F,0x0DF5,0x0E4C,0x0EA3,0x0EFA,0x0F52,
    0x0FA9,0x1001,0x105A,0x10B3,0x110C,0x1165,0x11BF,0x1219,
    0x1273,0x12CE,0x1329,0x1385,0x13E1,0x143D,0x149A,0x14F7,
    0x1555,0x15B4,0x1613,0x1672,0x16D2,0x1732,0x1793,0x17F5,
    0x1857,0x18BA,0x191E,0x1982,0x19E7,0x1A4C,0x1AB3,0x1B1A,
    0x1B82,0x1BEB,0x1C54,0x1CBF,0x1D2A,0x1D97,0x1E04,0x1E73,
    0x1EE3,0x1F53,0x1FC5,0x2039,0x20AD,0x2123,0x219A,0x2213,
    0x228E,0x230A,0x2387,0x2407,0x2489,0x250C,0x2592,0x261A,
    0x26A4,0x2732,0x27C1,0x2854,0x28EB,0x2984,0x2A21,0x2A49,
    0x2A72,0x2A9A,0x2AC3,0x2AEC,0x2B15,0x2B3F,0x2B69,0x2B93,
    0x2BBE,0x2BE9,0x2C14,0x2C3F,0x2C6B,0x2C97,0x2CC4,0x2CF1,
    0x2D1E,0x2D4C,0x2D7A,0x2DA9,0x2DD8,0x2E08,0x2E37,0x2E68,
    0x2E99,0x2ECA,0x2EFC,0x2F2F,0x2F62,0x2F95,0x2FCA,0x2FFF,
    0x3034,0x306A,0x30A1,0x30D9,0x3111,0x314B,0x3185,0x31C0,
    0x31FC,0x3239,0x3277,0x32B6,0x32F6,0x3337,0x337A,0x33BE,
    0x3403,0x344A,0x3493,0x34DE,0x352A,0x3579,0x35CA,0x361D,
    0x3673,0x36CD,0x3729,0x3741,0x3759,0x3771,0x378A,0x37A3,
    0x37BC,0x37D5,0x37EF,0x3809,0x3824,0x383E,0x3859,0x3875,
    0x3891,0x38AD,0x38CA,0x38E7,0x3905,0x3923,0x3941,0x3960,
    0x3980,0x39A1,0x39C2,0x39E3,0x3A06,0x3A29,0x3A4D,0x3A72,
    0x3A98,0x3ABF,0x3AE7,0x3B11,0x3B3C,0x3B68,0x3B96,0x3BC6,
    0x3BF9,0x3C2E,0x3C66,0x3CA1,0x3CB1,0x3CC1,0x3CD1,0x3CE1,
    0x3CF2,0x3D03,0x3D15,0x3D27,0x3D39,0x3D4C,0x3D60,0x3D74,
    0x3D89,0x3D9E,0x3DB4,0x3DCB,0x3DE3,0x3DFD,0x3E17,0x3E33,
    0x3E51,0x3E71,0x3E94,0x3E9D,0x3EA6,0x3EB0,0x3EBA,0x3EC4,
    0x3ECF,0x3EDA,0x3EE6,0x3EF2,0x3EFE,0x3F0C,0x3F1A,0x3F28,
    0x3F38,0x3F4A,0x3F5D,0x3F73,0x3F8D,0x3FAF,0x4000
};


static const int16_t FPL_ArctanLUT[256] PROGMEM =
{
    0x0000,0x0029,0x0051,0x007A,0x00A3,0x00CC,0x00F4,0x011D,
    0x0146,0x016F,0x0197,0x01C0,0x01E9,0x0211,0x023A,0x0262,
    0x028B,0x02B4,0x02DC,0x0305,0x032D,0x0356,0x037E,0x03A7,
    0x03CF,0x03F7,0x0420,0x0448,0x0470,0x0499,0x04C1,0x04E9,
    0x0511,0x0539,0x0561,0x0589,0x05B1,0x05D9,0x0601,0x0629,
    0x0651,0x0678,0x06A0,0x06C8,0x06EF,0x0717,0x073E,0x0766,
    0x078D,0x07B5,0x07DC,0x0803,0x082A,0x0851,0x0878,0x089F,
    0x08C6,0x08ED,0x0914,0x093B,0x0961,0x0988,0x09AE,0x09D5,
    0x09FB,0x0A22,0x0A48,0x0A6E,0x0A94,0x0ABA,0x0AE0,0x0B06,
    0x0B2C,0x0B51,0x0B77,0x0B9D,0x0BC2,0x0BE7,0x0C0D,0x0C32,
    0x0C57,0x0C7C,0x0CA1,0x0CC6,0x0CEB,0x0D10,0x0D34,0x0D59,
    0x0D7D,0x0DA2,0x0DC6,0x0DEA,0x0E0F,0x0E33,0x0E56,0x0E7A,
    0x0E9E,0x0EC2,0x0EE5,0x0F09,0x0F2C,0x0F50,0x0F73,0x0F96,
    0x0FB9,0x0FDC,0x0FFF,0x1021,0x1044,0x1067,0x1089,0x10AB,
    0x10CE,0x10F0,0x1112,0x1134,0x1156,0x1177,0x1199,0x11BB,
    0x11DC,0x11FD,0x121F,0x1240,0x1261,0x1282,0x12A3,0x12C3,
    0x12E4,0x1305,0x1325,0x1345,0x1366,0x1386,0x13A6,0x13C6,
    0x13E6,0x1405,0x1425,0x1444,0x1464,0x1483,0x14A2,0x14C1,
    0x14E0,0x14FF,0x151E,0x153D,0x155B,0x157A,0x1598,0x15B7,
    0x15D5,0x15F3,0x1611,0x162F,0x164C,0x166A,0x1688,0x16A5,
    0x16C2,0x16E0,0x16FD,0x171A,0x1737,0x1754,0x1770,0x178D,
    0x17AA,0x17C6,0x17E2,0x17FE,0x181B,0x1837,0x1853,0x186E,
    0x188A,0x18A6,0x18C1,0x18DD,0x18F8,0x1913,0x192E,0x1949,
    0x1964,0x197F,0x199A,0x19B4,0x19CF,0x19E9,0x1A04,0x1A1E,
    0x1A38,0x1A52,0x1A6C,0x1A86,0x1A9F,0x1AB9,0x1AD3,0x1AEC,
    0x1B05,0x1B1F,0x1B38,0x1B51,0x1B6A,0x1B83,0x1B9C,0x1BB4,
    0x1BCD,0x1BE5,0x1BFE,0x1C16,0x1C2E,0x1C46,0x1C5E,0x1C76,
    0x1C8E,0x1CA6,0x1CBE,0x1CD5,0x1CED,0x1D04,0x1D1B,0x1D33,
    0x1D4A,0x1D61,0x1D78,0x1D8E,0x1DA5,0x1DBC,0x1DD3,0x1DE9,
    0x1DFF,0x1E16,0x1E2C,0x1E42,0x1E58,0x1E6E,0x1E84,0x1E9A,
    0x1EB0,0x1EC5,0x1EDB,0x1EF0,0x1F06,0x1F1B,0x1F30,0x1F45,
    0x1F5A,0x1F6F,0x1F84,0x1F99,0x1FAE,0x1FC3,0x1FD7,0x1FEC
};

/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
int16_t FPL_sin16_(int16_t x);
int16_t FPL_cos16_(int16_t x);
int32_t FPL_tan16_(int16_t x);
int16_t FPL_arcsin16_(int16_t x);
int16_t FPL_arccos16_(int16_t x);
int16_t FPL_atan2_16_(int16_t x, int16_t y);
uint16_t FPL_sqrt32_(uint32_t x);

/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/
int16_t FPL_sin16_(int16_t x)
{
    uint8_t lutIndLo, lutIndHi, deltaX;
    uint16_t y;

	/* lookup table = sinus values of the first quadrant. The first two  	 */
	/* HSB = nuber of quadrant. The next 8 bit = table offset of the 		 */
	/* quadrant.	 														 */
	    
	lutIndLo = ((uint16_t)x << 2) >> 8;

    deltaX   = x & 0x3F;

    /* which quadrant ? quadrant 0,2: LUT-Index correct			             */
    /*                  quadrant 1,3: mirror LUT-index  	                 */

    if (x & 0x4000)
    {
        /* quadrant 1,3: mirror LUT-index  									 */
        lutIndHi = -lutIndLo;
        deltaX   = - deltaX + 0x40;

        lutIndLo = lutIndHi-1;
    }
    else
    {
        lutIndHi = lutIndLo+1;
    }

    if (lutIndHi == 0)
    {
        y = 0x7FFF;
    }
    else
    {
        y   = deltaX * (uint8_t)( SINLUT(lutIndHi) - SINLUT(lutIndLo) );


        y <<= 2;
        y  += 0x80;
        y >>= 8;

        y  += SINLUT(lutIndLo);
    }

    if (x & 0x8000)
    {
        y = -y;
    }

    return(y);
}


int16_t FPL_cos16_(int16_t x)
{
    return(FPL_sin16_(x + 0x4000));
}


int32_t FPL_tan16_(int16_t x)
{
    int16_t cosVal;
    int32_t sinVal, tanVal;

	sinVal = ((int32_t)FPL_sin16_(x))<<16;
	cosVal = FPL_cos16_(x);

    if (cosVal == 0)
	{
        cosVal = 1; 
	}

    tanVal = FPL_divs32_16_32_(sinVal, cosVal);


    return(tanVal);
}


int16_t FPL_arcsin16_(int16_t x)
{
    #define X_START_SEG_1       0x6E00
    #define TABIND_START_SEG_1  0x6E

    #define X_START_SEG_2       0x7D00
    #define TABIND_START_SEG_2  0xAA

    #define X_START_SEG_3       0x7F90
    #define TABIND_START_SEG_3  0xD3

    #define X_START_SEG_4       0x7FEC
    #define TABIND_START_SEG_4  0xEA

    uint8_t lutIndLo, deltaX, signX;
    uint16_t y;


    if (x < 0)
    {
        signX = 1;
        if (x == -32768)
        {
            return(-16384);
        }
        else
        {
            x = -x;
        }
    }
    else
    {
        signX = 0;
    }
	/* check in which segment is the result and then interpolate linear to   */
    /* x-offset of the table values									         */

    if (x < X_START_SEG_1)
    {
        /* table segment 0, dx = 256                                         */

        /* lower table index is equal to the highest 8 bit of x              */
        lutIndLo = x >> 8;

        /* x-offset relative to the table value [lutIndLo] is equal to		 */
        /* lowest 8 bit of x                                                 */
        deltaX = x & 0xFF;

        /* difference of the table values is still 8-bit	         		 */
        /* and deltaX  => unsigned 8x8 bit MUL is equals uint16      		 */
        y = deltaX * (uint8_t)( ARCSINLUT(lutIndLo+1) - ARCSINLUT(lutIndLo) );

        /* divide by 256,round the result and add to the lower table value   */
        y += 0x80;
        y >>= 8;
        y += ARCSINLUT(lutIndLo);
    }
    else
    {
        /* table segment 1 ... 4, 						                     */
        if (x < X_START_SEG_3)
        {
            if (x < X_START_SEG_2)
            {
                /* table segment 1, dx = 64                                  */

                x -= X_START_SEG_1; /* refer x to start of segment no 1      */

                deltaX = x & 0x3F;  /* x modulo 64 = x-offset                */

                x <<= 2; x >>= 8;   /* x div 64 = relativer Tab.index        */
                lutIndLo = x + TABIND_START_SEG_1; /* = abs. Tab.index       */

                /* difference of the table values is still 8-bit	         */
                /* and deltaX  => unsigned 8x8 bit MUL is equals uint16      */
                y = (uint8_t)( ARCSINLUT(lutIndLo+1) -
                               ARCSINLUT(lutIndLo  )   ) * deltaX;

                /* divide by 64,round the result and add to the lower table  */
                /* value			                                         */
                y <<= 2;
                y += 0x80;
                y >>= 8;
                y += ARCSINLUT(lutIndLo);
            }
            else
            {
                /* table segment 2, dx = 16                                  */

                x -= X_START_SEG_2; /* refer x to start of segment no 2      */

                deltaX = x & 0xF;   /* x modulo 16 = x-offset                */

                x >>= 4;            /* x div 16 = relative table.index       */
                lutIndLo = x + TABIND_START_SEG_2; /* = abs. table.index     */

                /* difference of the table values is still 8-bit	         */
                /* and deltaX  => unsigned 8x8 bit MUL is equals uint16      */
                y = (uint8_t)(ARCSINLUT(lutIndLo+1)-
                              ARCSINLUT(lutIndLo  ) ) * deltaX;

                /* divide by 16,round the result and add to the lower table  */
                /* value			                                         */
                y += 0x08;
                y >>= 4;
                y += ARCSINLUT(lutIndLo);
            }
        }
        else
        {
            if (x < X_START_SEG_4)
            {
                /* table segment   3, dx = 4                                 */

                x -= X_START_SEG_3; /* refer x to start of segment no 3 	 */

                deltaX = x & 0x3;   /* x modulo 4 = x-offset                 */

                x >>= 2;            /* x div 4 = relative table.index        */
                lutIndLo = x + TABIND_START_SEG_3; /* = abs. table.index     */

                /* difference of the table values is still 8-bit	         */
                /* and deltaX  => unsigned 8x8 bit MUL is equals uint16      */
                y = (uint8_t)(ARCSINLUT(lutIndLo+1)-
                              ARCSINLUT(lutIndLo  ) ) * deltaX;

                /* divide by 4, round the result and add to the lower table  */
                /* value			                                         */
                y += 0x02;
                y >>= 2;
                y += ARCSINLUT(lutIndLo);
            }
            else
            {
                /* segment no.4, dx = 1                              	     */

                x -= X_START_SEG_4; /* x auf Start von Segm. 4 beziehen      */

                lutIndLo = x + TABIND_START_SEG_4; /* = abs. Tab.index       */

                /* read direct the basic value                               */
                y = ARCSINLUT(lutIndLo);
            }
        }
    }

    if (signX == 1)
    {
        y = -y;
    }

    return(y);
}


int16_t FPL_arccos16_(int16_t x)
{
    return(0x4000 - FPL_arcsin16_(x));
}


int16_t FPL_atan2_16_(int16_t x, int16_t y)
{
    int16_t retAngle, xa, ya;
    uint8_t SecondOctand_b;
    uint8_t lutIxLo, lutIxHi, dx, dy;
    static uint32_t q_abs;

    xa = FPL_ABS_16_(x); ya = FPL_ABS_16_(y);

    if (ya <= xa)
    {
        q_abs = FPL_divs32_16_32_((int32_t)ya << 16, xa);

        SecondOctand_b = 0; /* store for octand-corretion		             */
    }
    else
    {
        q_abs = FPL_divs32_16_32_((int32_t)xa << 16, ya);

        SecondOctand_b = 1; /* store for octand-corretion		             */
    }

    /* compute from q_abs with lineare interpolation the angle and correct   */
    /* if necessary the octand and quadrant									 */

    if (q_abs == 0x10000)
    {
        retAngle = 0x2000; /* 45°, not part of the table                     */
    }
    else
    {
        lutIxLo = (q_abs >> 8); dx = q_abs & 0xFF;

        lutIxHi = lutIxLo + 1;

        if (lutIxHi == 0)
        {
            dy = 0x2000 - ARCTANLUT(0xFF);
        }
        else
        {
            dy = ARCTANLUT(lutIxHi) - ARCTANLUT(lutIxLo);
        }

        retAngle  = ARCTANLUT(lutIxLo);
		retAngle += ((dy * dx) + 0x80) >> 8;
    }

    if (SecondOctand_b)
    {
        retAngle  = 0x4000 - retAngle;
    }

    if (x < 0)
    {
        /* 2. or 3. quadrant                             */

        if (y < 0)
        {
            retAngle += 0x8000; /* 3. quadrant, add 180° */
        }
        else
        {
            retAngle = 0x8000 - retAngle; /* 2. quadrant */
        }
    }
    else
    {
        /* 1. or 4. quadrant                             */

        if (y < 0)
        {
            retAngle = -retAngle; /* 4. quadrant         */
        }
    }

    return(retAngle);
}


uint16_t FPL_sqrt32_(uint32_t x)
{

    int32_t e, t;
    uint32_t a;
    uint8_t n;

    e = -(int32_t)x;
    x = 0;

    a = 0x40000000;

    x = 0;

    for (n = 0; n < 16; n++)
    {
        t = e + x + a;

        x >>= 1;

        if (t <= 0)
        {
            e = t;
            x |= a;
        }

        a >>= 2;
    }

    return(x);
}
