/*****************************************************************************/
/* Description: Compute of attitude control values (P-, D-portion)           */
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

#include "config.h"
#include "FPL.h"
#include "radio.h"
#include "analog.h"
#include "InertSig.h"
#include "AttCtrl.h"
#include "communication.h"

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/


/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
AttCtrlType_InpSig	  InpSig;
volatile AttCtrlType_OutpSig	  OutpSig;
AttCtrlType_InternSig InternSig;
AttCtrlType_MotorGain MotorGain;
unsigned int Stick_Gas_Comp_ui = 0;
int16_t CmdYaw_si, CmdPitch_si, CmdRoll_si;

/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/
static unsigned long tmp32_sl;
static unsigned char Stick_Pitch_neutral_uc;
static unsigned char Stick_Roll_neutral_uc;
static unsigned char Stick_Pitch_not_neutral_uc;
static unsigned char Stick_Roll_not_neutral_uc;
signed long temp_throttle_sl = 0;
signed long temp_pitch_sl = 0;
signed long temp_roll_sl = 0;
signed long temp_yaw_sl = 0;
signed long temp_motor_sl = 0;


/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void AttCtrl_CalcCtrlOutput(void);
void AttCtrl_ExecMotorMixer(void);


/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* AttCtrl_CalcCtrlOutp														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* compute controler output values											 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void AttCtrl_CalcCtrlOutp(void)
{

	/*************************************************************************/
	/*     		     	Calculate ctrl-target angle values			   		 */
	/*************************************************************************/

	if(Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 1)
	{

		//Integrate "Target RollRate" to "Target RollAngle"
		InpSig.SetPointRolltmp_sl += (int32_t)InpSig.SetPointRollRate_si<<2;
		InpSig.SetPointRoll_HH_sl = FPL_ROUND_OFF_8_(InpSig.SetPointRolltmp_sl);
		
		//Integrate "Target PitchRate" to "Target PitchAngle"
		InpSig.SetPointPitchtmp_sl += (int32_t)InpSig.SetPointPitchRate_si<<2;
		InpSig.SetPointPitch_HH_sl = FPL_ROUND_OFF_8_(InpSig.SetPointPitchtmp_sl);
	}
	else
	{
		InpSig.SetPointRoll_si  = 
			FPL_fmuls8x8_16_(Stick_Roll_si, (uint8_t)Var[Variant_uc].ParaName.
				P_FACTOR_STICK_ROPI_CONTROLLER_ui);

		InpSig.SetPointPitch_si = 
			FPL_fmuls8x8_16_(Stick_Pitch_si, (uint8_t)Var[Variant_uc].ParaName.
				P_FACTOR_STICK_ROPI_CONTROLLER_ui);
	}

	/* Target Yaw angle is always zero!									     */
	InpSig.SetPointYaw_si   = 0; 

	/*************************************************************************/
	/*                   	Calculate angle deviation	                     */
	/*************************************************************************/
  
	//during HH-Mode
	if(Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 1)
	{
		int32_t temp32_sl = 0;
		#define ERROR_ANGLE_MAX 16384 //limit error of angle to 90°
		
		//Roll
		//---------------------------------------------------------------------
		//calculate error angle between target and current angle values and
		//bound to +/- ERROR_ANGLE_MAX
		temp32_sl = InpSig.SetPointRoll_HH_sl - InpSig.RollAngle_HH_sl;
		BOUND(temp32_sl,ERROR_ANGLE_MAX,-ERROR_ANGLE_MAX);
		InternSig.ErrorRoll_si = temp32_sl;


		//Pitch
		//---------------------------------------------------------------------
		//calculate error angle between target and current angle values and
		//bound to +/- ERROR_ANGLE_MAX
		temp32_sl = InpSig.SetPointPitch_HH_sl - InpSig.PitchAngle_HH_sl;
		BOUND(temp32_sl,ERROR_ANGLE_MAX,-ERROR_ANGLE_MAX);
		InternSig.ErrorPitch_si = temp32_sl;
	}

	//in normal ACC-Mode
	else
	{
		//Roll
		InternSig.ErrorRoll_si  = 
			FPL_CLIP_TO_
			(
				(int32_t)InpSig.SetPointRoll_si - 
				InpSig.RollAngle_si,
				0x7FFF
			);
		//Pitch
		InternSig.ErrorPitch_si  = 
			FPL_CLIP_TO_
			(
				(int32_t)InpSig.SetPointPitch_si - 
				InpSig.PitchAngle_si,
				0x7FFF
			);
	}

	//Yaw
	InternSig.ErrorYaw_si  = 
		FPL_CLIP_TO_
		(
			(int32_t)InpSig.SetPointYaw_si - 
			InpSig.YawAngle_si,
			0x7FFF
		);

	/*************************************************************************/
	/*                Calculate target rates and rate-errors                 */
	/*************************************************************************/

	//calulate the error between current and target roll-/pitch-RATE, which is
	//requested by the pilot.
	if(Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 1)
	{
		//Roll-Axle
		//calc target rate requested by pilot via radio
		InpSig.SetPointRollRate_si = (Stick_Roll_si * 
		(uint8_t)Var[Variant_uc].ParaName.P_FACTOR_STICK_ROPI_HEADINGHOLD_ui)>>1;

		//calc "error" between current and requested rate
		InternSig.ErrorRollRate_si = InertSig_Gyro.Omg_si[0] -
										InpSig.SetPointRollRate_si;


		//Pitch-Axle
		//calc target rate requested by pilot via radio
		InpSig.SetPointPitchRate_si = (Stick_Pitch_si * 
		(uint8_t)Var[Variant_uc].ParaName.P_FACTOR_STICK_ROPI_HEADINGHOLD_ui)>>1;

		//calc "error" between current and requested rate
		InternSig.ErrorPitchRate_si = InertSig_Gyro.Omg_si[1] -
										InpSig.SetPointPitchRate_si;
	}
	//in case of normal ACC-Mode the Targetrates are "0"
	else
	{  
		InternSig.ErrorRollRate_si = InertSig_Gyro.Omg_si[0];
		InternSig.ErrorPitchRate_si = InertSig_Gyro.Omg_si[1];
	}
	
	//Yaw-Axle
	//calc target rate requested by pilot via radio
	InpSig.SetPointYawRate_si = (- Stick_Yaw_si * 0)<<1;

	//calc "error" between current and requested rate
	InternSig.ErrorYawRate_si = InertSig_Gyro.Omg_si[2] -
									InpSig.SetPointYawRate_si;

	/*************************************************************************/
	/*                       Calculate P- and D-portion                      */
	/*************************************************************************/
	
	//#define PCmpnt_LIMIT 100
	//#define DCmpnt_LIMIT 75

	/* --------------------------- Rollaxle -------------------------------- */

	InternSig.PCmpntRoll_si	= 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				InternSig.ErrorRoll_si, (Poti.P_KP_ROLL_PITCH_ui << 3)
			)
		);
	//limit P-portion
	//BOUND(InternSig.PCmpntRoll_si,PCmpnt_LIMIT,-PCmpnt_LIMIT);


	InternSig.DCmpntRoll_si	= 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				- InternSig.ErrorRollRate_si, (Poti.P_KD_ROLL_PITCH_ui << 3)
			)
		);
	//limit D-portion
	//BOUND(InternSig.DCmpntRoll_si,DCmpnt_LIMIT,-DCmpnt_LIMIT);

	/* --------------------------- Pitchaxle ------------------------------- */

	InternSig.PCmpntPitch_si = 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				InternSig.ErrorPitch_si, (Poti.P_KP_ROLL_PITCH_ui << 3)
			)
		);
	//limit P-portion
	//BOUND(InternSig.PCmpntPitch_si,PCmpnt_LIMIT,-PCmpnt_LIMIT);


	InternSig.DCmpntPitch_si = 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				- InternSig.ErrorPitchRate_si, (Poti.P_KD_ROLL_PITCH_ui << 3)
			)
		);
	//limit D-portion
	//BOUND(InternSig.DCmpntPitch_si,DCmpnt_LIMIT,-DCmpnt_LIMIT);

	/* ----------------------------- Yawaxle ------------------------------- */

	InternSig.PCmpntYaw_si = 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				InternSig.ErrorYaw_si, (Poti.P_KP_YAW_ui << 2)
			)
		);

	InternSig.DCmpntYaw_si = 
		FPL_ROUND_OFF_16_
		(
			FPL_muls16x16_32_
			(
				- InternSig.ErrorYawRate_si, (Poti.P_KD_YAW_ui << 3)
			)
		);
	
	/* ---------------------- if pilot wants to yaw -------------------------*/
	if(ABS(Stick_Yaw_si + GPS_Yaw_si) > (uint8_t)Var[Variant_uc].ParaName.
			P_STICK_YAW_THRESHOLD_ui)
	{
		/* Set yawangle, yaw-portion and yaw-gain to zero					 */
		InertSig_Gyro.Phi_sl[2] = 0;
		InternSig.PCmpntYaw_si = 0;
	}

	/* ---------------- in case of "HeadingHold" modus ----------------------*/
	if (Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 1)
	{
		// pitch movement
		if(ABS(Stick_Pitch_si) < (uint8_t)Var[Variant_uc].ParaName.
			P_STICK_HEADINGHOLD_THRESHOLD_ui)
		{
			//JOE:Stick_Pitch_not_neutral_uc = 0;
			//clear angle just once
			if(Stick_Pitch_neutral_uc == 0)
			{	
				//copy P-Part for the next tasks before clearing it below
				InternSig.PCmpntPitch_K1_si += InternSig.PCmpntPitch_si;

				InpSig.PitchAngle_HH_sl = 0;
				InpSig.PitchAngle_HH_tmp_sl = 0;
				InternSig.PCmpntPitch_si = 0;
				InpSig.SetPointPitchtmp_sl = 0;
				InpSig.SetPointPitch_HH_sl = 0;
				
				Stick_Pitch_neutral_uc = 1;
			}
		}
		else
		{
			Stick_Pitch_neutral_uc = 0;
			
			if(Stick_Pitch_not_neutral_uc == 0)
			{
				InternSig.PCmpntPitch_K1_si += InternSig.PCmpntPitch_si;
			

				InpSig.PitchAngle_HH_sl = 0;
				InpSig.PitchAngle_HH_tmp_sl = 0;
				InternSig.PCmpntPitch_si = 0;
				InpSig.SetPointPitchtmp_sl = 0;
				InpSig.SetPointPitch_HH_sl = 0;

				Stick_Pitch_not_neutral_uc = 1;
			}
		}




		// roll movement
		if(ABS(Stick_Roll_si) < (uint8_t)Var[Variant_uc].ParaName.
			P_STICK_HEADINGHOLD_THRESHOLD_ui)
		{
			//JOE:Stick_Roll_not_neutral_uc = 0;
			//clear angle just once
			if(Stick_Roll_neutral_uc == 0)	
			{	
				//copy P-Part for the next tasks before clearing it below
				InternSig.PCmpntRoll_K1_si += InternSig.PCmpntRoll_si;
				
				InpSig.RollAngle_HH_sl = 0;
				InpSig.RollAngle_HH_tmp_sl = 0;
				InternSig.PCmpntRoll_si	= 0;
				InpSig.SetPointRolltmp_sl = 0;
				InpSig.SetPointRoll_HH_sl = 0;
				
				Stick_Roll_neutral_uc = 1;
			}
		}
		else
		{
			Stick_Roll_neutral_uc = 0;
			
			if(Stick_Roll_not_neutral_uc == 0)
			{
				InternSig.PCmpntRoll_K1_si += InternSig.PCmpntRoll_si;
				
			
				InpSig.RollAngle_HH_sl = 0;
				InpSig.RollAngle_HH_tmp_sl = 0;
				InternSig.PCmpntRoll_si	= 0;
				InpSig.SetPointRolltmp_sl = 0;
				InpSig.SetPointRoll_HH_sl = 0;
			
				Stick_Roll_not_neutral_uc = 1;
			}
		}
	}
	else
	{
		InternSig.PCmpntPitch_K1_si = 0;
		InternSig.PCmpntRoll_K1_si = 0;
	}
  

	/*************************************************************************/
	/*           Set P-portion to zero under certain conditions              */
	/*************************************************************************/

	/* ----------------------------- Rolle --------------------------------- */
		if (RollActiv_uc == 1)
		{
			InternSig.PCmpntRoll_si = 0;
			//YYY
	        /* Set yawangle to zero during loops to avoid turnback			 */
        	InertSig_Gyro.Phi_sl[2] = 0;
			/* ...for safety yaw-P-portion as well							 */
		    InternSig.PCmpntYaw_si = 0;
		}
	/* ---------------------------- Looping -------------------------------- */
 	    if (LoopActiv_uc == 1)
		{
	   	   	InternSig.PCmpntPitch_si = 0;
			/* Set yawangle to zero during loops to avoid turnback			 */
            //YYY
			InertSig_Gyro.Phi_sl[2] = 0;
		    /* ...for safety yaw-P-portion as well							 */
			InternSig.PCmpntYaw_si = 0;
      	}

	/*************************************************************************/
	/*             Compute control output: sum of P+D    	                 */
	/*************************************************************************/

	OutpSig.Roll_si = InternSig.PCmpntRoll_si + InternSig.DCmpntRoll_si
						+ InternSig.PCmpntRoll_K1_si;

	OutpSig.Pitch_si = InternSig.PCmpntPitch_si + InternSig.DCmpntPitch_si
						+ InternSig.PCmpntPitch_K1_si;

	OutpSig.Yaw_si = InternSig.PCmpntYaw_si + InternSig.DCmpntYaw_si;	

/*	//JJJ: should avoid/reduce a change of altitude during yawing
	if(OutpSig.Yaw_si > (Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui)/2)
	{
		OutpSig.Yaw_si = (Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui)/2;
	}

	if(OutpSig.Yaw_si < -(Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui)/2)
	{
		OutpSig.Yaw_si = -(Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui)/2;
	}

	if(OutpSig.Yaw_si > ((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui)/2)
	{
		OutpSig.Yaw_si = ((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui)/2;
	}

	if(OutpSig.Yaw_si < -((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui)/2)
	{
		OutpSig.Yaw_si = -((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui)/2;
	}
	//JJJ
*/

	/* Set to altitude control value										 */
	OutpSig.Throttle_si = AltControllerThrottle_sc;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* motor_mixer																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* compute motor values														 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void AttCtrl_ExecMotorMixer(void)
{
	uint8_t counter_uc=0;

	if (MotorsOn_uc == 1)
	{
		if ((Stick_Throttle_ui < (uint8_t)Var[Variant_uc].ParaName.
			P_MIN_THRO_CONTROLLER_ACTIVE_ui) && !RollActiv_uc && !LoopActiv_uc)
		{
			/*****************************************************************/
			/*       		       AttCtrl not active	                     */
			/*****************************************************************/

			LED_YELLOW_OFF;

			for(counter_uc=0;counter_uc<P_MaxMotorAmount;counter_uc++)
			{
				OutpSig.Motor_si[counter_uc] =(uint8_t)Var[Variant_uc].
													ParaName.P_MIN_THRO_ui;
			}
			//reset all HH-Modus relevant variables
			InpSig.SetPointRolltmp_sl = 0;
			InpSig.SetPointRoll_HH_sl = 0;
			InternSig.PCmpntRoll_si = 0;
			InternSig.PCmpntRoll_K1_si = 0;
			InpSig.RollAngle_HH_sl = 0;
			InpSig.RollAngle_HH_tmp_sl = 0;
			InternSig.ErrorRoll_si = 0;

			InpSig.SetPointPitchtmp_sl = 0;
			InpSig.SetPointPitch_HH_sl = 0;
			InternSig.PCmpntPitch_si = 0;
			InternSig.PCmpntPitch_K1_si = 0;
			InpSig.PitchAngle_HH_sl = 0;
			InpSig.PitchAngle_HH_tmp_sl = 0;
			InternSig.ErrorPitch_si = 0;
		}
		
		//Ctrl must be active
		else
		{
			/*****************************************************************/
			/*       				  AttCtrl active                         */
			/*****************************************************************/

			LED_YELLOW_ON;

			/*****************************************************************/
			/*  Precalculation of motor target values out of radio signals   */
			/*****************************************************************/

			/* Following calculation are used frequently					 */
			/* --> continuously recalculation necessary						 */
			
			//if GPS off, bad GPS RX, in HeadingHold modus or no EXO connected
			if(Gps_Mode_uc == Off || GPS.GPSFix_uc < 3 || Var[Variant_uc].
				ParaName.FS_HEADING_HOLD_ui == 1 || EXO_Connected_uc == 0)
			{
				//no GPS control is allowed
				GPS_Pitch_si = 0;
				GPS_Roll_si = 0;
				GPS_Yaw_si = 0;
			}

			/* Yawing only allowed if no loop active, due to massive error	 */
			/* occurancy due to missing Euler-equations						 */
			if(LoopActiv_uc == 0 && RollActiv_uc == 0)
			{
				CmdYaw_si =	((int16_t)(GPS_Yaw_si - Stick_Yaw_si)
							*(uint8_t)Var[Variant_uc].ParaName.
							P_FACTOR_STICK_YAW_DIRECT_ui)>>6;
			
				//JJJ: should avoid/reduce a change of altitude during yawing
				if(CmdYaw_si > (Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui))
				{
					CmdYaw_si = (Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui);
				}

				if(CmdYaw_si < -(Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui))
				{
					CmdYaw_si = -(Stick_Throttle_ui - (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui);
				}

				if(CmdYaw_si > ((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui))
				{
					CmdYaw_si = ((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui);
				}

				if(CmdYaw_si < -((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui))
				{
					CmdYaw_si = -((uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui - Stick_Throttle_ui);
				}
				//JJJ
			}
			else CmdYaw_si = 0;

			if(Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 0)
			{
				CmdPitch_si = ((int16_t)Stick_Pitch_si
							*(uint8_t)Var[Variant_uc].ParaName.
							P_FACTOR_STICK_ROPI_DIRECT_ui)>>6;
				CmdRoll_si = ((int16_t)Stick_Roll_si
							*(uint8_t)Var[Variant_uc].ParaName.
							P_FACTOR_STICK_ROPI_DIRECT_ui)>>6;
			}
			else
			{
				CmdPitch_si = 0;
				CmdRoll_si = 0;
			}

      		//limit Stick_Throttle to MAX_THRO_STICK
      		A_MIN_AB(Stick_Throttle_ui, 
					(uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_STICK_ui);


			/* Angle dependent throttle offset to inclination-compensation   */
			/* of ascending force 											 */
			/* Actual implementation: (1-cos) actualy correct is (1/cos)-1	 */
			/* For inclination angles up to approx. 30° the results are 	 */
			/* almost identical. Additional errors at (1/cos(90)) are avoided*/
			if(EXO_Connected_uc && AltControllerActive_uc == 1)
			{
				/* Result scaled with 2^15									 */
				tmp32_sl = FPL_muls16x16_32_(FPL_cos16_(MaxAngle_si), 
							Stick_Throttle_ui);
				/* Result scaled with 2^16									 */
				tmp32_sl <<= 1;
				Stick_Gas_Comp_ui = Stick_Throttle_ui 
									- ABS(FPL_ROUND_OFF_16_(tmp32_sl));
				A_MIN_AB(Stick_Gas_Comp_ui, 30);
			}
			else
			{
				Stick_Gas_Comp_ui = 0;
			}


			/*****************************************************************/
			/* Mapping of pilot, attitude, altitude and GPS commands to the	 */
			/* motor values													 */
			/*****************************************************************/

			//calc the pitch, roll and yaw sum once, because they are needed
			//several times
			temp_throttle_sl = OutpSig.Throttle_si + Stick_Gas_Comp_ui + Stick_Throttle_ui;
			temp_pitch_sl = OutpSig.Pitch_si + CmdPitch_si + (GPS_Pitch_si >> 1);
			temp_roll_sl = OutpSig.Roll_si + CmdRoll_si + (GPS_Roll_si >> 1);
			temp_yaw_sl = OutpSig.Yaw_si + CmdYaw_si;

			for(counter_uc=0;counter_uc<P_MaxMotorAmount;counter_uc++)
			{
					temp_motor_sl  = temp_throttle_sl * MotorGain.FMT_sc[counter_uc];
					temp_motor_sl += temp_pitch_sl * MotorGain.FMP_sc[counter_uc];
					temp_motor_sl += temp_roll_sl * MotorGain.FMR_sc[counter_uc];
					temp_motor_sl += temp_yaw_sl * MotorGain.FMY_sc[counter_uc];
				
				//divide by 32, due to MotorGain factors.
				temp_motor_sl = temp_motor_sl>>5;
				
				//"brake" motor in case of lower target rpm compared to the last cycle
				if(temp_motor_sl < OutpSig.Motor_K1_si[counter_uc])
				{
					OutpSig.Motor_si[counter_uc] = (temp_motor_sl * 6)>>3; //*0,75
				}
				else
				{
					OutpSig.Motor_si[counter_uc] = temp_motor_sl;	
				}				
	
				OutpSig.Motor_K1_si[counter_uc]	= temp_motor_sl;

				//limit the final motor values to MAX_THRO and MIN_THRO
				BOUND(OutpSig.Motor_si[counter_uc], 
					(uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui, 
					(uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui);
			}
		}
	}
}
