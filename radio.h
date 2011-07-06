/*****************************************************************************/
/* Description: radio control, emergency landing strategy					 */
/* servo compensation output, estimate hover throttle						 */
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



#ifndef __RADIO_H
#define __RADIO_H

/*****************************************************************************/
/*                                 includes                                  */
/*****************************************************************************/


/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/
typedef struct
{
	unsigned char Throttle_uc;		/*Channel for Throttle					 */
	unsigned char Pitch_uc;			/*Channel for Pitch						 */
	unsigned char Roll_uc;			/*Channel for Roll						 */
	unsigned char Yaw_uc;			/*Channel for Yaw						 */
	unsigned char UserFunc1_uc;		/*Channel for User Function 1			 */
	unsigned char UserFunc2_uc;		/*Channel for User Function 2			 */
	unsigned char UserFunc3_uc;		/*Channel for User Function 3			 */
	unsigned char UserFunc4_uc;		/*Channel for User Function 4			 */
	unsigned char UserFunc5_uc;		/*Channel for User Function 5			 */
	unsigned char UserFunc6_uc;		/*Channel for User Function 6			 */
	unsigned char UserFunc7_uc;		/*Channel for User Function 7			 */
	unsigned char UserFunc8_uc;		/*Channel for User Function 8			 */
	uint32_t CRC;					/*CRC for EEPROM storing				 */
}RadioType_RadioMapping;

/* Servo Extension board													 */
typedef struct
{
	unsigned int NumberOfServos_ui;		/* How many Servos are connected	 */
	unsigned int UpdateRate_ui;			/* update rate for frame			 */
	unsigned int ServoOut_ui[10];		/* Value of Servo					 */
	unsigned int ServoOutMax_ui[10]; 	/* Maximum value for Servo			 */
	unsigned int ServoOutMin_ui[10]; 	/* Minimum value for Servo			 */
	unsigned int ServoOutOffset_si[10];	/* Offset for Servo value			 */
	unsigned int ServoOutGain_si[10];	/* gain for Servo value				 */
}ServoExtType_Data;

/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/

#define ServoCenter 432
#define ServoMax 692
#define ServoMin 172

/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
extern volatile signed int RadioChannelRaw_si[13];
extern volatile signed int RadioChannel_si[13];
extern volatile unsigned int Funke_Kanal_D_ui[13];
extern volatile unsigned char Signal_OK_uc;
extern volatile unsigned char NewRadioData_uc;
extern volatile unsigned int SensOffsCount_uc;
extern unsigned char MotorsOn_uc;
extern unsigned char EmergLand_uc;
extern unsigned char RadioQuali_uc;
extern unsigned int SQinAir_ui;
extern unsigned int EmergLandTime_ui;
extern unsigned int EstHovThr_ui;
extern unsigned int n_ui;
extern unsigned long Stick_Thr_Sum_ul;
extern unsigned char CalibSelect_uc;
extern volatile RadioType_RadioMapping RC_Map;
extern volatile ServoExtType_Data ServoExt;
extern volatile unsigned int ServoOut1_ui;
extern volatile unsigned int ServoOut2_ui;
/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
extern void Radio_Init(void);
extern void Radio_Signal_Check(void);
extern void Radio_Stick_Check(void);
extern void Radio_Channel_Calib(void);
extern void Radio_Normalisation(void);
extern void Servo_Output(void);
extern void Est_Hov_Throttle(void);
extern void Calc_Pitch_Basis_Throttle(void);
extern void Power_Output(void);

#endif //__FUNKE_H
