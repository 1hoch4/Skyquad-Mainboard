/*****************************************************************************/
/* Description: Compute of attitude control values (P-, D-portion)			 */
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



#ifndef __ATTCTRL_H
#define __ATTCTRL_H


/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/

typedef struct
{
	int16_t RollAngle_si;			/* Ctrl-Input	Rollangle             	 */
	int16_t PitchAngle_si;			/*		"		Pitchangle			  	 */
	int32_t RollAngle_HH_sl;		/* Ctrl-Input 	_sl Rollangle Head.Hold	 */
	int32_t PitchAngle_HH_sl;		/*		"		_sl Pitchangle Head.Hold */
	int32_t RollAngle_HH_tmp_sl;	/* Ctrl-Input 	_sl Rollangle Head.Hold	 */
	int32_t PitchAngle_HH_tmp_sl;	/*		"		_sl Pitchangle Head.Hold */
	int16_t YawAngle_si;			/*		"		Yawangle			 	 */ 
    int16_t SetPointRoll_si;		/* target value	Rollangle (Radio !)  	 */
    int16_t SetPointPitch_si;		/*	"		"	Pitchangle (Radio !) 	 */
    int16_t SetPointYaw_si;			/*	"		"	Yawangle			 	 */
	int16_t SetPointRollRate_si;	/*	"		"	Rollrate			 	 */
	int16_t SetPointPitchRate_si;	/*	"		"	Pitchrate			 	 */
	int16_t SetPointYawRate_si;		/*	"		"	Yawrate				 	 */
	int32_t SetPointRolltmp_sl;		/*	"		"	_sl temp Rollangle	 	 */
	int32_t SetPointPitchtmp_sl;	/*	"		"	_sl temp Pitchangle	 	 */
	int32_t SetPointRoll_HH_sl;		/*	"		"	_sl Head.Hold Rollangle  */
	int32_t SetPointPitch_HH_sl;	/*	"		"	_sl Head.Hold Pitchangle */
}
AttCtrlType_InpSig;

typedef struct
{
    uint8_t 	Amount_uc;					/* Count of used motors			 */
	int8_t 		FMT_sc[P_MaxMotorAmount];	/* factor between  0 and 10		 */
	int8_t 		FMP_sc[P_MaxMotorAmount];	/* factor between -8 and 8		 */
	int8_t 		FMR_sc[P_MaxMotorAmount];	/* factor between -8 and 8		 */
	int8_t 		FMY_sc[P_MaxMotorAmount];	/* factor between -8 and 8		 */
	uint32_t 	CRC_uc;						/* CRC32 for EEPROM storing		 */

}
AttCtrlType_MotorGain;

typedef struct
{
    int16_t Throttle_si;		/* Throttle control-output value			 */
    int16_t Roll_si;			/* Roll-control-output value				 */
    int16_t Pitch_si;			/* Pitch-control-output value				 */
    int16_t Yaw_si;				/* Yaw-control-output value					 */   
    int16_t Motor_si[8];		/* Target value for motor					 */
	int16_t Motor_K1_si[8];		/* Target value for motor from last cycle	 */
}
AttCtrlType_OutpSig;


typedef struct
{
    int16_t PCmpntRoll_si;		/* P-component, Rollangle-control			 */
    int16_t PCmpntRoll_K1_si;	/* P-component, Rollangle-control last task  */
	int16_t DCmpntRoll_si;		/* D-component,	Rollangle-control			 */
    int16_t PCmpntPitch_si;		/* P-component, Pitchangle-control			 */
    int16_t PCmpntPitch_K1_si;	/* P-component, Pitchangle-control last task */
	int16_t DCmpntPitch_si;		/* D-component,	Pitchangle-control			 */
    int16_t PCmpntYaw_si;		/* P-component, Yawangle-control			 */
    int16_t DCmpntYaw_si;		/* D-component,		"		"				 */
    int16_t ErrorRoll_si;		/* Control-error Rollangle					 */
    int16_t ErrorPitch_si;		/*	"		"	 Pitchangle					 */
    int16_t ErrorYaw_si;		/*	"		"	 Yawangle					 */
	int16_t ErrorRollRate_si;	/*  "		"	 Rollrate					 */
	int16_t ErrorPitchRate_si;	/*  "		"	 Pitchrate					 */
	int16_t ErrorYawRate_si;	/*  "		"	 Yawrate					 */
}
AttCtrlType_InternSig;


/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/


/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
extern AttCtrlType_InpSig	 InpSig;
extern volatile AttCtrlType_OutpSig	 OutpSig;
extern AttCtrlType_InternSig InternSig;
extern AttCtrlType_MotorGain MotorGain;
extern unsigned int Stick_Gas_Comp_ui;
extern int16_t CmdYaw_si;

/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void AttCtrl_CalcCtrlOutp(void);
void AttCtrl_ExecMotorMixer(void);


#endif
