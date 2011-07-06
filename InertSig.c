/*****************************************************************************/
/* Description: communication and calculation of the inertial sensors		 */
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



/*****************************************************************************/
/*                                 includes                                  */
/*****************************************************************************/
#include <avr/io.h>

#include "FPL.h"
#include "config.h"
#include "spi.h"
#include "eeprom.h"
#include "radio.h"
#include "communication.h"
#include "AttCtrl.h"
#include "InertSig.h"

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/


#define IS__RATE_GAINCORR   		    26179	/* see comment below	     */
#define IS__RATE2K_GAINCORR 			19560	/*    "       "      "       */
#define IS__RATE_MAX        		    31500	/* 180°/s                    */

#define IS__SIN_COS_45				    23170	/* 0.7071, 1.15 fixed        */

#define IS__ACC_MIN_ANGLE_CORRECTION    5800	/* 0.87 g                    */
#define IS__ACC_MAX_ANGLE_CORRECTION    7500	/* 1.13 g                    */
#define IS__ACC_GAIN_CORR			    20133	/* see comment below	     */

#define IS__PHI_Y_MAX_GYRO_COMP			14563 	/* 80°						 */

/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/

InertSigType_GyroData InertSig_Gyro;

InertSigType_AccData  InertSig_Acc;

uint16_t SensOffsCount_ui;

uint8_t SQ_is_static_uc = 0;

int16_t MaxAngle_si = 0;


/*****************************************************************************/
/*                             local variablen                               */
/*****************************************************************************/

static int32_t AccumRate_sl[3];
static int16_t AccumRate2k_si[3];
static int32_t AccumAcc_sl[3];

static int16_t SigVal16_si, SigVal8_si;
static int16_t *OffsCorrValPtr_si, *RbufWdPtr_si;
static int32_t AccSum_sl;

static int16_t SinPhiX_si;  /*  1.15                                         */
static int16_t CosPhiX_si;  /*  1.15                                         */
static int32_t TanPhiY_sl;  /* 16.16                                         */
static int32_t OmYsinPhiXplusOmZcosPhiX_sl; /* 17.15                         */
static int32_t Val32_sl;    /* format variable                               */
static int64_t Result64_sll;/* format variable                               */


/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void InertSig_CalibOffsets(void);
inline void InertSig_PollAndPreprocess(void);
inline void InertSig_CoordTransformXY(int16_t *xPtr_si, int16_t *yPtr_si);
inline void InertSig_CompensateGyroSignals(void);
inline void InertSig_CompensateAngles(void);
void InertSig_GetAndProcessSignals(void);


/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* InertSig_CalibOffsets													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* determination of the sensor offsets										 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void InertSig_CalibOffsets(void)
{
    uint8_t ChIx;
	int16_t *CalDataPtr;

	if (SensOffsCount_ui == 1)
	{
        /* Initialisation										             */

        /* disable of radio PPM signal evaluation		                     */		
		TIMSK1 &= ~(1<<ICIE1);  

		LED_YELLOW_ON;

        for (ChIx = 0; ChIx < 3; ChIx++)
		{
		    AccumRate_sl[ChIx]   = 0;
			AccumRate2k_si[ChIx] = 0;
            
			AccumAcc_sl[ChIx]    = 0;
        }
	}

    if (SensOffsCount_ui <= 256)
    {
        /* determination is running, accumulate the sum variabels	        */

		for (ChIx = 0; ChIx < 3; ChIx++)
		{
			AccumRate_sl[ChIx]   += InertSig_Gyro.RateRaw_si[ChIx];
			AccumRate2k_si[ChIx] += InertSig_Gyro.Rate2kRaw_sc[ChIx];
            
			AccumAcc_sl[ChIx]    += InertSig_Acc.AccRaw_si[ChIx];
		}

		SensOffsCount_ui++;
	}
	else
	{
		/* determination is finished average and rounding the result		 */
		/* write to EEPROM												     */
				
		CalDataPtr = (int16_t *)&Eeprom_CalDataSecOffsCorr;
		
		for (ChIx = 0; ChIx < 3; ChIx++)
		{
			CalDataPtr[ChIx]   = FPL_ROUND_OFF_8_(AccumRate_sl[ChIx]);

			CalDataPtr[ChIx+3] = FPL_ROUND_OFF_8_(AccumRate2k_si[ChIx]);

			/* calibration of the gyros and ACC sensors)					 */
			if(CalibSelect_uc == GyroAccCal) 
			{
				CalDataPtr[ChIx+6] = FPL_ROUND_OFF_8_(AccumAcc_sl[ChIx]);
				/* substract 1g to get only the offset of the sensor		 */
				/* (quantisation is 6667 LSB/g)								 */      
				Eeprom_CalDataSecOffsCorr.OffsAccZ_si -= 6667;
			}
		}

		

		

		/* if calibration of the gyros and ACC sensors)						 */
		if(CalibSelect_uc == GyroAccCal) 
		{
			Eeprom_CalDataSecOffsCorr.InertSigCalibrated = 1;
			CalibSelect_uc = 0;
			Buzzer(2,Short);
		}
		/* if only calibration of the gyros									 */
		else if(CalibSelect_uc == GyroCal) 
		{
			CalibSelect_uc = 0;
			Buzzer(1,Short);
		}

		/* write to EEPROM									                 */
		Eeprom_WriteToOffsCorrSection();
				
		/* counter re-initialise                                             */
		SensOffsCount_ui = 0;

 		/* activate radio check				         						 */
		TIMSK1 |= (1<<ICIE1);

		LED_YELLOW_OFF;
	}
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* InertSig_PollAndPreprocess												 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* read the sensors and preprocess the signals								 */
/* resolution:																 */
/*	HByte gyro: 4.3721°/sec/bit												 */
/*	normal gyro signal: 0.00571°/sec/bit									 */
/*	ACC-signal:  6666bit=1g													 */
/*	yaw rate 0.00571°/sec/bit -> 560°/sec = 98074 bit --> signed long		 */
/*	angle  1° = 58377/64 = 912 bit											 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
inline void InertSig_PollAndPreprocess(void)
{
	uint8_t byte0_uc, byte1_uc, byte2_uc, byte3_uc;
	uint8_t ChIx = 0;

	/* --------------------------------------------------------------------- */
	/* read and precalculate of ACC sensor signals (Offset- and Sensitivity) */
	/* --------------------------------------------------------------------- */

        OffsCorrValPtr_si = (int16_t *)&Eeprom_CalDataSecOffsCorr.OffsAccX_si;

#if (FS_ACC_BUILT == FS_LISACC)

		signed int byte1_si=0,byte2_si=0;
		signed long temp_sl=0;

		SPI_CS_LIS_LOW;
		
		/*  Read und Auto Increment											 */
		byte1_si = SPI_SENS_TransferByte((0x28 +(ChIx << 1)) | 0x80 | 0x40); 
		byte1_si = SPI_SENS_TransferByte(0xFF) ;
		byte1_si = byte1_si & 0x00FF;

		byte2_si = SPI_SENS_TransferByte(0xFF);
		byte2_si = ((byte2_si << 8) & 0x0F00);

		temp_sl = (byte2_si | byte1_si) << 4;
		/*  scale to 6666 = 1g												 */
		temp_sl =((temp_sl * 1024)/2517);

		InertSig_Acc.AccRaw_si[1] = -(signed int)temp_sl;

	//*********************************************************

		byte1_si = SPI_SENS_TransferByte(0xFF) ;
		byte1_si = byte1_si & 0x00FF;

		byte2_si = SPI_SENS_TransferByte(0xFF);
		byte2_si = ((byte2_si << 8) & 0x0F00);

		temp_sl = (byte2_si | byte1_si) << 4;
		/*  scale to 6666 = 1g												 */
		temp_sl =((temp_sl * 1024)/2517);

		InertSig_Acc.AccRaw_si[0] = (signed int)temp_sl;


	//*********************************************************

		byte1_si = SPI_SENS_TransferByte(0xFF) ;
		byte1_si = byte1_si & 0x00FF;

		byte2_si = SPI_SENS_TransferByte(0xFF);
		byte2_si = ((byte2_si << 8) & 0x0F00);

		temp_sl = (byte2_si | byte1_si) << 4;
		/*  scale to 6666 = 1g												 */
		temp_sl =((temp_sl * 1024)/2517);

		InertSig_Acc.AccRaw_si[2] = -(signed int)temp_sl;

		SPI_CS_LIS_HIGH;

		/* ----------------------- Offsetkorrektur ------------------------- */
	    /* no over- or underflow plausibility check because +/-5 g is 		 */
	    /* in practical use													 */
	        for (ChIx = 0; ChIx < 3; ChIx++)
	        {

		    SigVal16_si = InertSig_Acc.AccRaw_si[ChIx] - OffsCorrValPtr_si[ChIx];
		    InertSig_Acc.Acc_si[ChIx] = SigVal16_si;

            }

	/* --------------------------------------------------------------------- */
	/* broadcast to all sensore to store the act. value in the output buffer */
	/* --------------------------------------------------------------------- */

	SPI_SENS_CS_LOW;    
	SPI_SENS_TransferByte(0xE0);
	SPI_SENS_CS_HIGH;


#endif
//*********************************************************

#if (FS_ACC_BUILT == FS_BOSCHACC)

	/* --------------------------------------------------------------------- */
	/* broadcast to all sensore to store the act. value in the output buffer */
	/* --------------------------------------------------------------------- */

	SPI_SENS_CS_LOW;
	SPI_SENS_TransferByte(0xE0);
	SPI_SENS_CS_HIGH;

	for (ChIx = 0; ChIx < 3; ChIx++)
	{
    	SPI_SENS_CS_LOW;

		/* only the first three received bytes containing the signal values  */

		/* Bit 4 ... 0: RD_DATA-Instr. = 0x12, Bit 7 .. 5: (TYP Cadr1 Cadr0) */
		/*(TYP tells the type of sensor element (1:SMB, 0:SMG), (Cadr1,Cadr0)*/
		/* stands for the 2 Bit long Chip ID, which is configured by the     */
		/* two hardware inputs (ID1 , ID0) of the sensor elements.           */

		byte0_uc = SPI_SENS_TransferByte(0x92 | (ChIx << 5));
    	byte1_uc = SPI_SENS_TransferByte(0x00);
    	byte2_uc = SPI_SENS_TransferByte(0x00);
		SPI_SENS_TransferByte(0x00);
		SPI_SENS_TransferByte(0x02);

		SPI_SENS_CS_HIGH;

		SigVal16_si =
			((int16_t)byte0_uc << 14) | ((int16_t)byte1_uc << 6) |
			 (byte2_uc >> 2);

		InertSig_Acc.AccRaw_si[ChIx] = SigVal16_si;

		/* ----------------------- Offsetkorrektur ------------------------- */
	    /* no over- or underflow plausibility check because +/-5 g is 		 */
	    /* in practical use													 */

		SigVal16_si -= OffsCorrValPtr_si[ChIx];

		InertSig_Acc.Acc_si[ChIx] = SigVal16_si;
	}

#endif
	/* --------------------------------------------------------------------- */
	/* read gyro-signals and process data (offset- a. sensibility)			 */
	/* --------------------------------------------------------------------- */

    OffsCorrValPtr_si = (int16_t *)&Eeprom_CalDataSecOffsCorr.OffsGyroX_si;

	for (ChIx = 0; ChIx < 3; ChIx++)
	{
    	SPI_SENS_CS_LOW;		
		
		/* The first four received bytes containing the signal values        */
		byte0_uc = SPI_SENS_TransferByte(0x12 | (ChIx << 5));
    	byte1_uc = SPI_SENS_TransferByte(0x00);
    	byte2_uc = SPI_SENS_TransferByte(0x00);  	
		byte3_uc = SPI_SENS_TransferByte(0x00);    	
		SPI_SENS_TransferByte(0x02);		
		
		SPI_SENS_CS_HIGH;

		SigVal16_si =
			((int16_t)byte0_uc << 14) | ((int16_t)byte1_uc << 6) | 
			 (byte2_uc >> 2);

		InertSig_Gyro.RateRaw_si[ChIx] = SigVal16_si;

		/* - Offset- and sensitivity correction, switch between ranges -     */

	    if (FPL_ABS_16_(SigVal16_si) > IS__RATE_MAX)
		{
        	/* ------------ |Rate| > 180°/s, switch to Rate2k -------------  */

			/* Offset correction Rate2kHz                                    */
			SigVal8_si  = (int16_t)InertSig_Gyro.Rate2kRaw_sc[ChIx];
			SigVal8_si -= OffsCorrValPtr_si[ChIx+3];
        	
			if (SigVal8_si >  127) SigVal8_si =  127; /* Ggfs. clippen       */
			if (SigVal8_si < -128) SigVal8_si = -128; /* dito                */

			SigVal8_si <<= 8;

		    SigVal16_si = 
			    FPL_ROUND_OFF_16_
				(
					FPL_fmuls16x16_32_
				 	(
				 		SigVal8_si, IS__RATE2K_GAINCORR
				 	)
				);
		}
		else
		{
	    	/* ---------------- use of regular rate-signal ----------------- */

			/* Offset correction rate, Over-/Underflow not possible, because */
			/* raw signal doesn´t reach range limit                          */
	        SigVal16_si -= OffsCorrValPtr_si[ChIx]; 
                                
			/* sensitivity correction with factor IS__RATE_GAINCORR/64       */
			/* = 0.00312076 * 2^17 provides the correct 16bit long			 */
			/* "Binärdegree" (2^16 <=> 360°) of the accumulated angles.		 */
			/* That is done in dt = 3 ms.  								 */
			/* The additional factor of 64 contained in the correction factor*/
			/* leads to a higher resolution and corresponds to a decimal     */
			/* -part of the accumulated angle of 6 Bit.					     */
			/* The decimal-part will be reduced to 8bit during the euler     */
			/* compensation of the angle rates ("Achskopplung") to calculate */
			/* easier with the "16-Bit-Binaerdegree"					   - */
			
			SigVal16_si = 
		    	(FPL_muls16x16_32_
			 	 (
				   	SigVal16_si, IS__RATE_GAINCORR
			     ) 
				 + 0x10000) >> 16;
	    
			SigVal16_si >>= 1;
    	}

		InertSig_Gyro.Omg_si[ChIx] = SigVal16_si;
             
    	/* Rate2k is delayed by one task (= 3 ms)							 */
		/* (Output Low pass of the ASIC delayed due to the group runtime	 */
		/* of the Rate-signal compared to the Rate2kHz-signal)               */
    	InertSig_Gyro.Rate2kRaw_sc[ChIx] = (byte2_uc << 6) | (byte3_uc >> 2);
	}

	/* --------------------------------------------------------------------- */
	/* mainboard rotated? If yes, compute transformation of sensor signals   */
	/* in the xy-plain.												         */
	/* --------------------------------------------------------------------- */

	//YYY: That parameterswitch must not be variantcoded. It should be handeled 
	//exactly like the motor mixer parameter struct.
	if(Var[Variant_uc].ParaName.FS_MAINBOARD_ROTATED_ui == 1)
	{
		InertSig_CoordTransformXY
		(
			&InertSig_Gyro.Omg_si[0], &InertSig_Gyro.Omg_si[1]
		);
	
		InertSig_CoordTransformXY
		(
			&InertSig_Acc.Acc_si[0], &InertSig_Acc.Acc_si[1]
		);
	}

	/* --------------------------------------------------------------------- */
   	/*        Acc-filter											         */
	/* --------------------------------------------------------------------- */

	for (ChIx = 0; ChIx < 3; ChIx++)
	{
		SigVal16_si = InertSig_Acc.Acc_si[ChIx];

    	RbufWdPtr_si = &InertSig_Acc.Rbuf_si[ChIx][InertSig_Acc.RbufIx_uc];

		AccSum_sl = InertSig_Acc.RbufSum_sl[ChIx];

        /* substract old value									             */
		AccSum_sl -= *RbufWdPtr_si; 

        /* add actual value to raw value				                     */	
		AccSum_sl += SigVal16_si;  

		InertSig_Acc.RbufSum_sl[ChIx] = AccSum_sl;
    
    	*RbufWdPtr_si = SigVal16_si; 

		/* store new filter result											 */
        InertSig_Acc.AccFilt_si[ChIx] = ((AccSum_sl + 4) >> 3);
	}

	InertSig_Acc.RbufIx_uc++;
	InertSig_Acc.RbufIx_uc &= 0x07;
}


inline void InertSig_CoordTransformXY(int16_t *xPtr_si, int16_t *yPtr_si)
{
	int16_t xt_si, yt_si;

	/* we have to calculate:											     */
	/*																		 */
	/* x' = (y + x) * sin(45°)												 */
	/* y' = (y - x) * sin(45°)												 */

	/*************************************************************************/
	/*   	    			  x' = (y + x) * sin(45°)					     */
	/*************************************************************************/

	/* calc sum with maximum resolution and remove LSB by round of			 */
	/* to avoid 16-Bit-overflow => factor 2 must be taken into account later */
	Val32_sl   = (int32_t)(*xPtr_si) + *yPtr_si;
	Val32_sl  += 1;
	Val32_sl >>= 1;

	/* (16.0 * 1.15) << 1 => 16.16, cut LoByte => 24.8    			         */
	Val32_sl = FPL_fmuls16x16_32_((int16_t)Val32_sl, IS__SIN_COS_45) >> 8;

	/* multiply with factor 2 and limit to int24-range					     */
	Val32_sl = FPL_CLIP_TO_(Val32_sl << 1, 0x007FFFFF);

	xt_si = FPL_ROUND_OFF_8_(Val32_sl); /* remove lower 8 Bit by round of    */

	/*************************************************************************/
	/*   	    			  y' = (y - x) * sin(45°)					     */
	/*************************************************************************/

	/* calc sum with maximum resolution and remove LSB by round of			 */
	/* to avoid 16-Bit-overflow => factor 2 must be taken into account later */
	Val32_sl   = (int32_t)(*yPtr_si) - *xPtr_si;
	Val32_sl  += 1;
	Val32_sl >>= 1;

	/* (16.0 * 1.15) << 1 => 16.16, cut LoByte => 24.8       			     */
	Val32_sl = FPL_fmuls16x16_32_((int16_t)Val32_sl, IS__SIN_COS_45) >> 8;
	
	/* multiply with factor 2 and limit to int24-range					     */
	Val32_sl = FPL_CLIP_TO_(Val32_sl << 1, 0x007FFFFF);

	yt_si = FPL_ROUND_OFF_8_(Val32_sl); /* remove lower 8 Bit by round of    */

	/* return result of calculation (by reference)		                     */
	*xPtr_si = xt_si; *yPtr_si = yt_si;
}


inline void InertSig_CompensateGyroSignals(void)
{
	/*************************************************************************/
	/* we have to calculate:											     */
    /*                                                                       */
    /* OmXcomp = OmX + OmY*sin(PhiX)*tan(PhiY) + OmZ*cos(PhiX)*tan(PhiY)     */
    /*                                                                       */
	/* OmYcomp = OmY*cos(PhiX) - OmZ*sin(PhiX)                               */
    /*                                                                       */
    /* OmZcomp = OmY*sin(PhiX)/cos(PhiY) + OmZ*cos(PhiX)/cos(PhiY)           */
	/*																		 */
	/*************************************************************************/

	/*************************************************************************/
    /* pre calculate the following values, they are used several times	     */
	/*************************************************************************/

	SinPhiX_si = FPL_sin16_(IS__PHI_X_16);
	CosPhiX_si = FPL_cos16_(IS__PHI_X_16); 	
	TanPhiY_sl = FPL_tan16_(IS__PHI_Y_16); 

	/* ------------------ OmY*sin(PhiX) + OmZ*cos(PhiX) -------------------- */

    /*   OmY*sin(PhiX)  => 17.15                                             */
    OmYsinPhiXplusOmZcosPhiX_sl =
		FPL_muls16x16_32_(InertSig_Gyro.Omg_si[1], SinPhiX_si);

    /* + OmZ*cos(PhiX)  => 17.15                                             */
    FPL_macs16x16_32_(InertSig_Gyro.Omg_si[2], CosPhiX_si,
		&OmYsinPhiXplusOmZcosPhiX_sl);

    /*************************************************************************/
    /*      OmXcomp = OmX + tan(PhiY)*(OmY*sin(PhiX) + OmZ*cos(PhiX))        */
	/*************************************************************************/

    /* tan(PhiY)*(""), => (16.16 * 17.15) << 1 => 32.32                      */
	Result64_sll = FPL_muls32x32_64_(TanPhiY_sl, OmYsinPhiXplusOmZcosPhiX_sl);
	Result64_sll += Result64_sll;

    /* roud to 32.0, add OmX 					                             */
    Val32_sl = FPL_xtract64_32_(&Result64_sll, 4) + InertSig_Gyro.Omg_si[0];

    /* two additional decimal bits necessary => SHL by 2 Bit. reason:  		 */
    /* rates are available as 10.6-values, convert compensated value into	 */
	/* 24.8 format for later integration						             */
    InertSig_Gyro.OmgComp_sl[0] = FPL_CLIP_TO_(Val32_sl, 0x1FFFFFFF) << 2;

    /*************************************************************************/
    /*              OmYcomp = OmY*cos(PhiX) - OmZ*sin(PhiX)                  */
	/*************************************************************************/
    
	/*   OmY*cos(PhiX)    => 17.15                                           */
	Val32_sl = FPL_muls16x16_32_(InertSig_Gyro.Omg_si[1], CosPhiX_si);
    
	/* + OmZ*(-sin(PhiX)) => 17.15                                           */
	FPL_macs16x16_32_(InertSig_Gyro.Omg_si[2], -SinPhiX_si, &Val32_sl);

    /* shift result by 13 Bit to the right to ... 							 */
	Val32_sl = FPL_ROUND_OFF_8_(Val32_sl); Val32_sl <<= 3; 
	Val32_sl = FPL_ROUND_OFF_8_(Val32_sl);

	/* ... bring the comp. Rate to the 24.8 format for later integration	 */	
	InertSig_Gyro.OmgComp_sl[1] = Val32_sl;
	
    /*************************************************************************/
    /*         OmZcomp = (OmY*sin(PhiX) + OmZ*cos(PhiX))/cos(PhiY)           */
	/*************************************************************************/
	
    /* ("")/cos(PhiY) 17.15 / 1.15 => 32.0                                   */
    Val32_sl = 
		FPL_divs32_16_32_
			(OmYsinPhiXplusOmZcosPhiX_sl, FPL_cos16_(IS__PHI_Y_16));

    /* two additional decimal bits necessary => SHL by 2 Bit. reason:  		 */
    /* rates are available as 10.6-values, convert compensated value into	 */
	/* 24.8 format for later integration						             */

    InertSig_Gyro.OmgComp_sl[2] = FPL_CLIP_TO_(Val32_sl, 0x1FFFFFFF) << 2;			
}


inline void InertSig_CompensateAngles(void)
{
	int16_t AccGainCorr_si, AngleError_si;

	/* --------------------------------------------------------------------- */
    /* compute actual acc, sqrt(ax^2 + ay^2 + az^2) 						 */
	/* --------------------------------------------------------------------- */
	
	Val32_sl = 
		FPL_muls16x16_32_
		(
			InertSig_Acc.AccFilt_si[0], InertSig_Acc.AccFilt_si[0]
		);
		
	FPL_macs16x16_32_
	(
		InertSig_Acc.AccFilt_si[1], InertSig_Acc.AccFilt_si[1], &Val32_sl
	);

	FPL_macs16x16_32_
	(
		InertSig_Acc.AccFilt_si[2], InertSig_Acc.AccFilt_si[2], &Val32_sl
	);
    
	InertSig_Acc.AccResultant_ui = FPL_sqrt32_(Val32_sl);


	/* --------------------------------------------------------------------- */
	/* compute roll and pitch angle from acc sensors						 */
	/* --------------------------------------------------------------------- */
	/* modify ax-resolution to get 1g = 2^15. That is necessary for			 */
	/* arcsin()calculation. (Resolution of raw signal is 6667 bit/g)     	 */
     	   
	AccGainCorr_si = 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				InertSig_Acc.AccFilt_si[0] << 2, IS__ACC_GAIN_CORR
			)
		);

	AccGainCorr_si = FPL_CLIP_TO_(AccGainCorr_si, 0x1FFF) << 2;

	/* pitch angle calculated with ax = - arcsin(ax / g)                 	 */

	InertSig_Acc.PhiY_si = - FPL_arcsin16_(AccGainCorr_si);
		
	/* modify ay-resolution to get 1g = 2^15. That is necessary for		 	 */
	/* arcsin()calculation. (Resolution of raw signal is 6667 bit/g)     	 */
     	   
	AccGainCorr_si = 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				InertSig_Acc.AccFilt_si[1] << 2, IS__ACC_GAIN_CORR
			)
		);

	AccGainCorr_si = FPL_CLIP_TO_(AccGainCorr_si, 0x1FFF) << 2;

	/* roll angle calculated with ay = arcsin(ay / g) / cos(PhiY)		 	 */

	Val32_sl = (int32_t)FPL_arcsin16_(AccGainCorr_si) << 16;
		
	/* 16.16 / 1.15 => 31.1, 31.1 >> 1 => 32.0                           	 */
	Val32_sl = FPL_divs32_16_32_(Val32_sl, FPL_cos16_(IS__PHI_Y_16)) >> 1;

	InertSig_Acc.PhiX_si = FPL_CLIP_TO_(Val32_sl, 0x00007FFF);

	/* --------------------------------------------------------------------- */
	/* check if support of gyro angles by acc angles is possible             */
	/* --------------------------------------------------------------------- */
    if ( (InertSig_Acc.AccResultant_ui < IS__ACC_MIN_ANGLE_CORRECTION) || 
         (InertSig_Acc.AccResultant_ui > IS__ACC_MAX_ANGLE_CORRECTION) ||
	     (ABS(InertSig_Acc.PhiX_si) > 8192        		           	 ) || 
		 (ABS(InertSig_Acc.PhiY_si) > 8192							 ) ||
		 (Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 1			 ) ||
		 (InertSig_Acc.AccFilt_si[2]   < 0                           ) ||
		 (BUTTON_ON                                                  ))
    {
		/* if resulting acc <>1g or az < 0 --> no compensation			     */

		LED_GREEN_OFF; /* integral compensation inactive					 */
		SQ_is_static_uc = 0;                
    }
	else
	{
		LED_GREEN_ON;  /* integral compensation active		                 */
		SQ_is_static_uc = 1;


		/* ----------------------------------------------------------------- */
		/*     calculate error of gyro angels filter them (subtract a certain*/
		/*     part of it from the integrations values                       */
		/* ----------------------------------------------------------------- */
		/* The amount is w = 1/512. That is done by calculating the error    */
		/* not in full 24.8 (fixed-format) resolution, but in 16-Bit-Binaer- */
		/* degree. That results in 1/256. An additional ASHR by 1 Bit gives  */
		/* w = 1/512.														 */
		/* roll angle error = roll angle (integral) - roll angle (acc)  	 */
        /* Rollwinkel-Fehler = Rollwinkel(Integration) - Rollwinkel(ay)      */
	
		AngleError_si   = IS__PHI_X_16 - InertSig_Acc.PhiX_si;
		AngleError_si >>= 1;

		/* subtract a certain amount of the error						     */

		InertSig_Gyro.Phi_sl[0] -= AngleError_si;

		/* pitch angle error = pitch angle (integral) - pitch angle (acc)  	 */
	
		AngleError_si   = IS__PHI_Y_16 - InertSig_Acc.PhiY_si;
		AngleError_si >>= 1;

		/* subtract a certain amount of the error						     */

		InertSig_Gyro.Phi_sl[1] -= AngleError_si;
	}		
}


void InertSig_GetAndProcessSignals(void)
{
	uint8_t ChIx;

	/* --------------------------------------------------------------------- */
	/* Get the corrected gyro and acc sensor signals, compensate offsets	 */
	/* adjust resolution to the numerical integration and filter the signal. */
	/* --------------------------------------------------------------------- */

	InertSig_PollAndPreprocess();

	/* --------------------------------------------------------------------- */
	/* calc euler equations "achskopplung" to compensate rate signals 		 */
	/* and convert into 24.8 format										     */
	/* --------------------------------------------------------------------- */

	if (FPL_ABS_16_(IS__PHI_Y_16) < IS__PHI_Y_MAX_GYRO_COMP && !LoopActiv_uc 
		&& !RollActiv_uc && Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 0)
	{
		InertSig_CompensateGyroSignals();
	}
	else
	{
		for (ChIx = 0; ChIx < 3; ChIx++)
		{
			InertSig_Gyro.OmgComp_sl[ChIx] = 
				(int32_t)InertSig_Gyro.Omg_si[ChIx] << 2;
		}
	}

	/* --------------------------------------------------------------------- */
	/*  		    	integrate yaw rate --> angle						 */
	/* The integrationsvariables are build out of a 16-Bit-Binaerdegreepart  */
	/* and additional a 8-Bit-Decimal-part. The integration is done with the */
	/* higher range to gain resolution.										 */
	/* --------------------------------------------------------------------- */

	for (ChIx = 0; ChIx < 3; ChIx++)
	{
		InertSig_Gyro.Phi_sl[ChIx] += InertSig_Gyro.OmgComp_sl[ChIx];
	}

	InpSig.RollAngle_HH_tmp_sl += InertSig_Gyro.OmgComp_sl[0];
	InpSig.PitchAngle_HH_tmp_sl += InertSig_Gyro.OmgComp_sl[1];

	/* --------------------------------------------------------------------- */
    /* use acc sensors to support the gyros?								 */
	/* --------------------------------------------------------------------- */

	InertSig_CompensateAngles();

	/* --------------------------------------------------------------------- */
	/* The anglerate integration variables have the "24.8" resolution.		 */
	/* But for the attitude controller the 16-Bit_Binaerdegree is sufficient */
	/* => round of the lower 8 Bit.											 */
	/* --------------------------------------------------------------------- */

	InpSig.RollAngle_si  = FPL_ROUND_OFF_8_(InertSig_Gyro.Phi_sl[0]);
	InpSig.PitchAngle_si = FPL_ROUND_OFF_8_(InertSig_Gyro.Phi_sl[1]);
	InpSig.YawAngle_si   = FPL_ROUND_OFF_8_(InertSig_Gyro.Phi_sl[2]);

	if(Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 1)
	{
		InpSig.RollAngle_HH_sl = FPL_ROUND_OFF_8_(InpSig.RollAngle_HH_tmp_sl);
		InpSig.PitchAngle_HH_sl = FPL_ROUND_OFF_8_(InpSig.PitchAngle_HH_tmp_sl);
	}
	else
	{
		InpSig.RollAngle_HH_sl = 0;
		InpSig.PitchAngle_HH_sl = 0;
	}

	/* --------------------------------------------------------------------- */
	/* compute maximum of pitch and roll angles. That is the largest angle.  */
	/* --------------------------------------------------------------------- */
	
	MaxAngle_si = MAX(ABS(InpSig.RollAngle_si),ABS(InpSig.PitchAngle_si));

}
