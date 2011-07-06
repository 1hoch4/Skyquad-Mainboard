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
#include <avr/interrupt.h>

#include "FPL.h"
#include "config.h"
#include "eeprom.h"
#include "InertSig.h"
#include "AttCtrl.h"
#include "communication.h"
#include "radio.h"

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           makros and #defines                             */
/*****************************************************************************/


/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
volatile signed int RadioChannelRaw_si[13];
volatile signed int RadioChannel_si[13];
volatile unsigned char Signal_OK_uc;
volatile unsigned char NewRadioData_uc = 0;
unsigned char MotorsOn_uc = 0;
signed int Min_Radio_Channel_si[13];
signed int Max_Radio_Channel_si[13];
unsigned char EmergLand_uc = 0;
unsigned char RadioQuali_uc = UC_MAX;
unsigned int SQinAir_ui = 0;
unsigned int EmergLandTime_ui = 0;
unsigned char LoopActiv_uc = 0;
unsigned char RollActiv_uc = 0;
unsigned int EstHovThr_ui = 0;	
unsigned int n_ui = 0;
unsigned long Stick_Thr_Sum_ul = 0;
volatile RadioType_RadioMapping RC_Map;
volatile unsigned int ServoOut1_ui=432;
volatile unsigned int ServoOut2_ui=432;
volatile ServoExtType_Data ServoExt;

/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/
unsigned char Stick_Config_Active_uc;
unsigned char CalibSelect_uc = 0;

/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void Radio_Init(void);
void Radio_Signal_Check(void);
void Radio_Stick_Check(void);
void Radio_Channel_Calib(void);
void Radio_Normalisation(void);
void Est_Hov_Throttle(void);
void Servo_Output(void);

/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Radio_Init																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* init of timer1 16bit, to evaluate the PPM-SumSignal of radio				 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void Radio_Init(void)
{
	//Timer 1:
	//Output Compare: Off
	//PWM Mode: Off
	//Noise Canceler: On
	//Input Capture: Rising Edge
	//Prescale: 64	
	
	TCCR1A=0x00;
	TCCR1B=(1<<CS11)|(1<<CS10)|(1<<ICES1)|(1<<ICNC1);
	TIMSK1 |= (1<<ICIE1);

	unsigned char index_uc;
	for (index_uc=0; index_uc<13; index_uc++)
	{
		Faktor_MaxFunke_Kanal_si[index_uc] = 127;
		Faktor_MinFunke_Kanal_si[index_uc] = -128;
	}


#if (FS_SERVOEXT == FS_SERVOEXT_ON)

	//Timer 0:
	//Output Compare: On (OCRA)
	//CTC Mode: ON
		
	//WGM0 2:0=2
	//Prescale: 64	
	//Output COM0A =2
	//Output COM0B =0;

	TCCR0A=(1<<WGM01);
	TCCR0B=(1<<CS01)|(1<<CS00);
	TIMSK0 |= (1<<OCIE0A);

	cbi(TCCR0A,COM0A0); //start with high level
 	sbi(TCCR0A,COM0A1);
	PORTB &=~0x08; //Reset low active

#endif

#if (FS_SERVOEXT == FS_SERVOEXT_OFF)

	//Timer 0:
	//Output Compare: On (OCRA)
	//CTC Mode: ON
		
	//WGM0 2:0=2
	//Prescale: 64	
	//Output COM0A =2
	//Output COM0B =0;

	TCCR0A=(1<<WGM01);
	TCCR0B=(1<<CS01)|(1<<CS00);
	TIMSK0 |= (1<<OCIE0A);

	cbi(TCCR0A,COM0A0); //start with high level
 	sbi(TCCR0A,COM0A1);

	cbi(TCCR0A,COM0B0); //start with high level
 	sbi(TCCR0A,COM0B1);

#endif
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* ISR(TIMER1_CAPT_vect														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* interrupt service routine for input capture event of timer1				 */
/* sum-signal of radio receiver will be analysed and splitted into different */
/* channels																	 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
ISR(TIMER1_CAPT_vect)
{
    //18.432.000 Hz / 64 / 1000 = 288 Ticks/ms ==> 1.5ms = 432 Ticks	
	//content of status register will be copied into temporary variable,
	//to recopy the original content after finishing that routine
	unsigned int Temp_StatusRegister_ui = SREG;

	//disable all interrupts
	cli();

	//initialisation
	signed int PPM_time_si;
	static signed int PPM_time_old_si=0;
	static unsigned char index_uc;

	PPM_time_si = ICR1 - PPM_time_old_si;
	PPM_time_old_si = ICR1;
	
	//shorter than 0.6ms (noise) or longer than 14,5ms --> loss of radio (noise)  
	if (PPM_time_si < 173 || PPM_time_si > 4200)
	{	
		/* RadioQuali_uc = 0.75*RadioQuali_uc */
		RadioQuali_uc -= (RadioQuali_uc >> 2);
	}

	//after longer high-level --> new data
	//shortest pause of PPM24: 5,7ms

	//longer than 5ms and shorter than 14,5ms
	else if (PPM_time_si > 1440 && PPM_time_si < 4200)
	{
		index_uc = 1;
		NewRadioData_uc = 1;
	}
	//high level between 0,7ms and 2,4ms (at 100% min. 1ms, max. 2ms)
	//max. 12 channel
	else if (PPM_time_si > 200 && PPM_time_si < 700 && index_uc < 13)
    {
		//plausible change of channels: increase quality value		
		if ((ABS((PPM_time_si - 432) - RadioChannelRaw_si[index_uc]) < 6) 
			&& RadioQuali_uc < 240) 
		{
			RadioQuali_uc += 10;
		}

		//assign counter to according radio channel and filter signal.
		//adjust the filter with parameter P_FunkeFilter
		#define RADIO_FILTER 3
		RadioChannelRaw_si[index_uc]
			=(signed int)((RadioChannelRaw_si[index_uc]*(RADIO_FILTER - 1))
												+PPM_time_si-432)/RADIO_FILTER;
		index_uc++;
	}
	
	if (RadioQuali_uc > 2) RadioQuali_uc -= 1;
	
	//use original state of status register
	SREG = Temp_StatusRegister_ui;		
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Radio_Signal_Check														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* 6ms Task																	 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void Radio_Signal_Check(void)
{
	unsigned char counter_uc=0;
	//reduce RadioQuali_uc every task
	if (RadioQuali_uc > 2) RadioQuali_uc -= 2;

	//set EmergLand
	if (RadioQuali_uc < 120) EmergLand_uc = 1;
	if (EmergLand_uc == 1)
	{
		//in case of loss of radio, do the following...:
		//##############################################################
		//if potis are used vor parameters, this potis have to be set to
		//a reasonable value to ensure a save flight and/or landing
		//##############################################################
		
		/* keep shortly the last valid throttle value						 */
		/* && for a certain duration  &&  SQ flies							 */
		if(EmergLandTime_ui > 83 
			&& EmergLandTime_ui < Var[Variant_uc].ParaName.P_EMER_THRO_DURATION_ui 
			&& SQinAir_ui != 0)
		{
			if(AltControllerActive_uc == 1)
			{
				/* Increase throttle a bit to give altitude-				 */
				/* controller enough margin									 */
				Stick_Throttle_ui = EstHovThr_ui + 15; 
			}
			else
			{
				Stick_Throttle_ui = EstHovThr_ui;
			}
		}

		/* maximum duration of emergency throttle expired or SQ NOT flying	 */
		/* -> turn off motors												 */
		if(EmergLandTime_ui >= Var[Variant_uc].ParaName.P_EMER_THRO_DURATION_ui
			|| SQinAir_ui == 0)
		{
			Stick_Throttle_ui = 0;

			for(counter_uc=0;counter_uc<P_MaxMotorAmount;counter_uc++)
			{
		    	OutpSig.Motor_si[counter_uc] = 0;
			}

			MotorsOn_uc = 0;
			SQinAir_ui = 0;
			LED_YELLOW_OFF;
		}

		Stick_Pitch_si = 0;
		Stick_Roll_si = 0;
		Stick_Yaw_si = 0;
		if(SensOffsCount_ui == 0 && BootDelay_uc >= 60) Buzzer(0,ContinousOn);
		
		//6ms task, therefore: 65535 = 6,5535 min.
		EmergLandTime_ui ++;
		LED_RED_ON;
	}
	
	if (RadioQuali_uc >= 150) 
	{
		EmergLandTime_ui = 0;
		//reset Bit
		EmergLand_uc = 0;
		LED_RED_OFF;
		Buzzer(0,ContinousOff);
	}

	/* check if roll is demanded. Just if allowed via function switch		 */
	#if ((FS_LOOPING == FS_LOOP_ROLL) || (FS_LOOPING == FS_LOOP_PITCHROLL))

		if ((Stick_Roll_si > (uint8_t)Var[Variant_uc].
			   ParaName.P_LOOP_ROLL_ON_THRESHOLD_ui
			 || Stick_Roll_si < -(uint8_t)Var[Variant_uc].ParaName.
			   P_LOOP_ROLL_ON_THRESHOLD_ui) && SQinAir_ui)
		{
			RollActiv_uc = 1;
		}

		if (Stick_Roll_si <= ((uint8_t)Var[Variant_uc].
			  ParaName.P_LOOP_ROLL_ON_THRESHOLD_ui 
			- (uint8_t)Var[Variant_uc].ParaName.P_LOOP_ROLL_OFF_HYST_ui)
			&& Stick_Roll_si >= (-(uint8_t)Var[Variant_uc].
			  ParaName.P_LOOP_ROLL_ON_THRESHOLD_ui + (uint8_t)Var[Variant_uc].
			  ParaName.P_LOOP_ROLL_OFF_HYST_ui))
		{
			RollActiv_uc = 0;
		}
	#endif

	/* check if loop is demanded. Just if allowed via function switch		 */
	#if ((FS_LOOPING == FS_LOOP_PITCH) || (FS_LOOPING == FS_LOOP_PITCHROLL))
		
		if ((Stick_Pitch_si > (uint8_t)Var[Variant_uc].
			   ParaName.P_LOOP_PITCH_ON_THRESHOLD_ui
			 || Stick_Pitch_si < -(uint8_t)Var[Variant_uc].ParaName.
			    P_LOOP_PITCH_ON_THRESHOLD_ui) && SQinAir_ui)
		{
			LoopActiv_uc = 1;
		}

		if (Stick_Pitch_si <= ((uint8_t)Var[Variant_uc].
			  ParaName.P_LOOP_PITCH_ON_THRESHOLD_ui 
			- (uint8_t)Var[Variant_uc].ParaName.P_LOOP_PITCH_OFF_HYST_ui) 
			&& Stick_Pitch_si >= (-(uint8_t)Var[Variant_uc].ParaName.
			  P_LOOP_PITCH_ON_THRESHOLD_ui + (uint8_t)Var[Variant_uc].ParaName.
			  P_LOOP_PITCH_OFF_HYST_ui))
		{
			LoopActiv_uc = 0;
		}
	#endif
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Radio_Stick_Check()														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* evaluate radio channels and set according bits							 */
/* motor on: MotorsOn_uc = 1												 */
/* motor off: MotorsOn_uc = 0												 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void Radio_Stick_Check(void)
{
	unsigned char counter_uc=0;
	if (!EmergLand_uc)
	{	
		/*********************************************************************/
		/* start and stop motors with throttle stick in minimum and the		 */
		/* yaw stick in min resp max; ONLY IF NOT IN PITCH MODUS		 */
		/*********************************************************************/

		/* Motors on														 */
		if(Stick_Throttle_ui < (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui
			&& Stick_Yaw_si < -110)
		{
			/* reset integrals at motor start								 */
			InertSig_Gyro.Phi_sl[0] = 0;
			InertSig_Gyro.Phi_sl[1] = 0;
			InertSig_Gyro.Phi_sl[2] = 0;
			
			if(Var[Variant_uc].ParaName.FS_HEADING_HOLD_ui == 1)
			{
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
			/* reset integrals at motor start								 */


			if (Eeprom_CalDataStatus.Byte_uc == EEPROM_ALL_SECTIONS_OK)
			{
                /* allow motor start only if EEPROM data check is ok		 */
				/* even the flag "MotorsOn_uc" will be set only in case of	 */
				/* valid EEPROM data										 */
				for(counter_uc=0;counter_uc<P_MaxMotorAmount;counter_uc++)
				{
			    	OutpSig.Motor_si[counter_uc] = (uint8_t)Var[Variant_uc]
														.ParaName.P_MIN_THRO_ui;
				}
 			    MotorsOn_uc = 1;
            }
			else
			{
				/* error: EEPROM - data damaged !							 */
                LED_RED_ON;
			}
		}

		/* Motors off														 */
		if(Stick_Throttle_ui < (uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui
			&& Stick_Yaw_si > 110)
		{
			for(counter_uc=0;counter_uc<P_MaxMotorAmount;counter_uc++)
			{
		    	OutpSig.Motor_si[counter_uc] = 0;
			}

			MotorsOn_uc = 0;
			SQinAir_ui = 0;
		}


		/*********************************************************************/
		/* increase counter "SQinAir_ui" as soon as the att_ctrl is active	 */
		/* if the motors are off the counter has to be reset.				 */
		/* due to 96ms task: 65535 = 109,225 min.							 */
		/*********************************************************************/
		if(Stick_Throttle_ui >= (uint8_t)Var[Variant_uc].ParaName
				.P_MIN_THRO_CONTROLLER_ACTIVE_ui
			&& MotorsOn_uc
			&& SQinAir_ui < 65535)
		{
			SQinAir_ui ++;
		}

		/*********************************************************************/
		/* offset calibration of the gyro + ACC or just gyro				 */
		/* at full throttle, yaw right and motors off -> gyro + ACC			 */
		/* at full throttle, yaw left and motors off -> just gyro			 */
		/* yellow LED on during ongoing OffsetCalib							 */
		/*********************************************************************/
		
		/* Gyro + Acc														 */
		if((Stick_Throttle_ui > 230 && Stick_Yaw_si < -110 && !MotorsOn_uc)
			|| Stick_Config_Active_uc)
		{
			if (!Stick_Config_Active_uc)							
			{														
				Stick_Config_Active_uc = 1;
				SensOffsCount_ui = 1;		
				CalibSelect_uc = GyroAccCal;
			}
			else if ((Stick_Yaw_si < 20 && Stick_Yaw_si > -20) 
					|| Stick_Throttle_ui < 100)
			{
				Stick_Config_Active_uc=0;
			}
		}
		/* Just Gyro														 */
		if((Stick_Throttle_ui > 230 && Stick_Yaw_si > 110 && !MotorsOn_uc)
			|| Stick_Config_Active_uc)
		{
			if (!Stick_Config_Active_uc)							
			{														
				Stick_Config_Active_uc = 1;	
				SensOffsCount_ui = 1;
				CalibSelect_uc = GyroCal;							
			}														
			else if ((Stick_Yaw_si < 20 && Stick_Yaw_si > -20)
					|| Stick_Throttle_ui < 100)
			{
				Stick_Config_Active_uc=0;
			}
		}
	}
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Radio_Channel_Calib()													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* 1.calibration of the radio channels with motors off and Dip				 */
/* switch 2 pressed.														 */
/* 2.all sticks and potis in middle position, then press button.			 */
/* 3.now move every stick and poti to min and max position.					 */
/* 4. DIP2 up again -> radio is calibrated and saved in the EEPROM.			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void Radio_Channel_Calib(void)
{
	unsigned char index_uc;

	/* DIP2 pressed?														 */
	if DIP2_ON
	{
		/* Button -> record zero values										 */
	    if BUTTON_ON
		{
			LED_YELLOW_ON;
			for (index_uc = 1; index_uc<13; index_uc++)
			{
				Faktor_MaxFunke_Kanal_si[index_uc] = 127;		
				Faktor_MinFunke_Kanal_si[index_uc] = -128;		
				OffsFunke_Kanal_si[index_uc] = 0;				
				Min_Radio_Channel_si[index_uc] = 0;				
				Max_Radio_Channel_si[index_uc] = 0;				
			}
		
			for (index_uc = 1; index_uc<5; index_uc++)
			{
				OffsFunke_Kanal_si[index_uc] = RadioChannelRaw_si[index_uc];
			}
			
			/* DIP2 pressed?												 */
			while DIP2_ON
			{
				for (index_uc = 1; index_uc<13; index_uc++)
				{
					if (RadioChannelRaw_si[index_uc]
						 - OffsFunke_Kanal_si[index_uc]
						< Min_Radio_Channel_si[index_uc])
					{
						Min_Radio_Channel_si[index_uc] = RadioChannelRaw_si[index_uc]
						 								 - OffsFunke_Kanal_si[index_uc];
					}

					if (RadioChannelRaw_si[index_uc]
						 - OffsFunke_Kanal_si[index_uc]
						> Max_Radio_Channel_si[index_uc])
					{
						Max_Radio_Channel_si[index_uc] = RadioChannelRaw_si[index_uc]
						 								 - OffsFunke_Kanal_si[index_uc];
					}
				}
			}

			for (index_uc = 1; index_uc<13; index_uc++)
			{
				if (Min_Radio_Channel_si[index_uc] == 0) 
				{
					/* avoid division by zero!!!							 */
					Min_Radio_Channel_si[index_uc] = 1;
				}
				
				if (Max_Radio_Channel_si[index_uc] == 0)
				{
					/* avoid division by zero!!!							 */
					Max_Radio_Channel_si[index_uc] = 1;
				}
			}

			for (index_uc = 1; index_uc<13; index_uc++)
			{
				Faktor_MinFunke_Kanal_si[index_uc] = 16384 / Min_Radio_Channel_si[index_uc];
				Faktor_MaxFunke_Kanal_si[index_uc] = 16384 / Max_Radio_Channel_si[index_uc];
			}

			Eeprom_CalDataSecRemoteCtrl.RadioCalibrated_uc = 1;
			Eeprom_WriteToRemoteCtrlSection();

			LED_YELLOW_OFF;
		}
	}
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Radio_Normalisation														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void Radio_Normalisation(void)
{
	unsigned char index_uc;
		
	for (index_uc = 1; index_uc<13; index_uc++)
	{
		if ((RadioChannelRaw_si[index_uc] - OffsFunke_Kanal_si[index_uc]) > 0)
		{
			RadioChannel_si[index_uc] = MIN((((RadioChannelRaw_si[index_uc]
											- OffsFunke_Kanal_si[index_uc])
											* Faktor_MaxFunke_Kanal_si[index_uc]) + 64) 
										   / 128, 127);
  	   	}
		else
		{
			RadioChannel_si[index_uc] = MAX((((RadioChannelRaw_si[index_uc]
											- OffsFunke_Kanal_si[index_uc])
											* (-Faktor_MinFunke_Kanal_si[index_uc])) - 64)
										   / 128, -128);
		}
	}
	/* Add Offset to Throttle Channel										 */
	Stick_Throttle_ui = Stick_Throttle_ui + 128;
    
    /* Add Offset to User-Function Channels									 */
    for (index_uc = 5; index_uc<13; index_uc++)
    {
    	RadioChannel_si[index_uc] += 128;
    }
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Est_Hov_Throttle															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* estimate the necessary hover throttle during the complete flight by		 */
/* calculating the mean of Stick_Gas_ui										 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void Est_Hov_Throttle(void)
{	
	if(n_ui < UI_MAX && !EmergLand_uc && SQ_is_static_uc == 1
		&& SQinAir_ui != 0 
		&& Stick_Throttle_ui >= (uint8_t)Var[Variant_uc].ParaName.
			P_MIN_THRO_CONTROLLER_ACTIVE_ui)
	{
		if(AltControllerActive_uc != 1)	
		{
			Stick_Thr_Sum_ul += Stick_Throttle_ui;
		}

		else 
		{
			/* Addition due to given possibility of negative				 */
			/* 'AltControllerThrottle_sc'values								 */
			Stick_Thr_Sum_ul += (Stick_Throttle_ui + AltControllerThrottle_sc);
		}

		n_ui++;
		EstHovThr_ui = Stick_Thr_Sum_ul / n_ui;
    	BOUND(EstHovThr_ui, (uint8_t)Var[Variant_uc].ParaName.P_MAX_THRO_ui,
				(uint8_t)Var[Variant_uc].ParaName.P_MIN_THRO_ui);
	}
	
	/* Recalculate estimation if motors are switched off					 */
	if(Stick_Throttle_ui < (uint8_t)Var[Variant_uc].ParaName.
		P_MIN_THRO_CONTROLLER_ACTIVE_ui)
	{
		n_ui = 0;
		Stick_Thr_Sum_ul = 0;
		//EstHovThr_ui = 0;
	}
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Servo_Output													 			 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* read servo signals 1 and 2, calculate timer values, which will be used in */
/* the timer0 function. Signal resolution: 0=left max 128=middel 255=right	 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void Servo_Output(void)
{
#if (FS_SERVOEXT	== FS_SERVOEXT_ON)

	/****************************/
	/*		172bit = 0.6ms		*/
	/*		432bit = 1.5ms		*/
	/*		692bit = 2.4ms		*/
	/****************************/	

	
	//if(gain==127) gain=1; // no div by zero
	//Servo_Signal= ((signed int)(Frank_16_mul_16((Value_si-128)*(127-Gain))/64)+432)+(offset-128);


	//Servo Signal = (((Value * 1.125)+Offset)+minValue);
	//Sample for Servo 1
	//ServoExt.ServoOut_ui[0]=((Value1+(Value1>>3)
				//+ServoExt.ServoOutOffset_si[0])+ServoExt.ServoOutMin_ui[0]);
	//Sample for Servo 2
	//ServoExt.ServoOut_ui[1]=((Value2+(Value2>>3)
				//+ServoExt.ServoOutOffset_si[1])+ServoExt.ServoOutMin_ui[1]);
	//Sample for Servo 3
	//ServoExt.ServoOut_ui[2]=((Value3+(Value3>>3)
				//+ServoExt.ServoOutOffset_si[2])+ServoExt.ServoOutMin_ui[2]);
	//Sample for Servo 4
	//ServoExt.ServoOut_ui[3]=((Value4+(Value4>>3)
				//+ServoExt.ServoOutOffset_si[3])+ServoExt.ServoOutMin_ui[3]);


#endif


#if (FS_SERVOEXT == FS_SERVOEXT_OFF)

//For Servo2
static signed int temp0_si=0;
//For Servo1
static signed int temp1_si=0;

if(Poti.P_CAMERACOMP_ON_OFF_ui>=128)
{

	if (Poti.P_CAMERAPITCHCOMP_ON_OFF_ui>=128)
	{
		//No div by zero
		if(Var[Variant_uc].ParaName.P_CAMERAPITCHCOMP_GAIN_ui==127) 
			Var[Variant_uc].ParaName.P_CAMERAPITCHCOMP_GAIN_ui=128;

		temp0_si = ((InpSig.PitchAngle_si/((signed int)Var[Variant_uc].
				ParaName.P_CAMERAPITCHCOMP_GAIN_ui-127))+
				((signed int)Poti.P_CAMERAPITCHCOMP_OFFSET_ui-127)); 

	}
	else
	{
		// no div by zero
		if(Var[Variant_uc].ParaName.P_SERVO1_GAIN_ui==127) Var[Variant_uc].
				ParaName.P_SERVO1_GAIN_ui=128;

		temp0_si= ((signed int)(FPL_muls16x16_32_(((signed int)Poti.
				P_SERVO1_VALUE_ui-128),(127-(signed int)Var[Variant_uc].
				ParaName.P_SERVO1_GAIN_ui))/64))+
				((signed int)Var[Variant_uc].ParaName.P_SERVO1_OFFSET_ui-128);
	}


	if (Poti.P_CAMERAROLLCOMP_ON_OFF_ui>=128)
	{
		//No div by zero
		if(Var[Variant_uc].ParaName.P_CAMERAROLLCOMP_GAIN_ui==127) 
			Var[Variant_uc].ParaName.P_CAMERAROLLCOMP_GAIN_ui=128;

		temp1_si = ((InpSig.RollAngle_si/((signed int)Var[Variant_uc].
				ParaName.P_CAMERAROLLCOMP_GAIN_ui-127))+
				((signed int)Poti.P_CAMERAROLLCOMP_OFFSET_ui-127)); 

	}
	else
	{
		// no div by zero
		if(Var[Variant_uc].ParaName.P_SERVO2_GAIN_ui==127) Var[Variant_uc].
				ParaName.P_SERVO2_GAIN_ui=128; 

		temp1_si= ((signed int)(FPL_muls16x16_32_(((signed int)Poti.
				P_SERVO2_VALUE_ui-128),(127-(signed int)Var[Variant_uc].
				ParaName.P_SERVO2_GAIN_ui))/64))+
				((signed int)Var[Variant_uc].ParaName.P_SERVO2_OFFSET_ui-128);
	}

}

BOUND(temp0_si,260,-260); //max 0.6ms-2.4ms
BOUND(temp1_si,260,-260); //max 0.6ms-2.4ms

ServoOut1_ui = (unsigned int)(temp0_si + ServoCenter);
ServoOut2_ui = (unsigned int)(temp1_si + ServoCenter);

#endif  /* Servo extension OFF	 */

}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Power_Output													 			 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Function to Switch the Power Outputs on and off 							 */
/* 																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void Power_Output(void)
{
	#if (FS_MAINBOARD_TYPE == FS_NEW_MAINBOARD)
  	if((Poti.P_POWER_VALUE1_ui) < (Var[Variant_uc].ParaName.
										P_POWER_THRESHOLD1_ui)) POWER1_OFF;
	else POWER1_ON;
	#endif

	if((Poti.P_POWER_VALUE2_ui) < (Var[Variant_uc].ParaName.
										P_POWER_THRESHOLD2_ui)) POWER2_OFF;
	else POWER2_ON;

	if((Poti.P_POWER_VALUE3_ui) < (Var[Variant_uc].ParaName.
										P_POWER_THRESHOLD3_ui)) POWER3_OFF;
	else POWER3_ON;
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Servo_Output_Interrupt										 			 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* generate Servo PWM														 */
/* this function works interrupt triggert.									 */
/* Due to the 8bit timer and the prescaler of 64: 18432000/64 = 288 counts/ms*/
/* therefore we have to count several times to get the middle position of a  */
/* servo (1,5ms). (1,5ms -> 288 counts/ms * 1,5 = 432 counts. 432counts/255= */
/* 1,694. That means: overflow and rest 432-255=177)						 */
/*																			 */
/* order: Servo1, high......low												 */
/*        Servo2, high......low												 */
/*		  then pause of 14 overflows...										 */
/*		  then start again from beginning.									 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
ISR(TIMER0_COMPA_vect)
{

#if (FS_SERVOEXT == FS_SERVOEXT_ON)

	static unsigned char CycleCount_uc=0,CycleCountK1_uc=10,ReloadValue_uc=0;
	static signed int Temp_si=0;				// temp value	
	static unsigned char TempCounter_uc=0;		// temporary counter for reset
	static signed int TotalFrameTime_si=0;		// sum of all single signals
    //5760 Update rate for hole frame
	// 288/ms -> 5760/288=20ms=50Hz
	//3744 -> 76,9Hz
	/****************************/
	/*		172bit = 0.6ms		*/
	/*		432bit = 1.5ms		*/
	/*		692bit = 2.4ms		*/
	/****************************/	

	
	cbi(TCCR0A,COM0A0);		// High level

	if(CycleCount_uc==ServoExt.NumberOfServos_ui)
	{
		ServoExt.ServoOut_ui[CycleCount_uc] = ServoExt.UpdateRate_ui - TotalFrameTime_si;
		TempCounter_uc++;
	}

	if (CycleCount_uc!=CycleCountK1_uc)
	{
		Temp_si = ServoExt.ServoOut_ui[CycleCount_uc];

			//No BOUND at last cycle
			if(CycleCount_uc < ServoExt.NumberOfServos_ui)
			{
				BOUND(Temp_si,ServoExt.ServoOutMax_ui[CycleCount_uc],ServoExt.ServoOutMin_ui[CycleCount_uc]); 
			}
		TotalFrameTime_si += Temp_si;
	}

	CycleCountK1_uc=CycleCount_uc;

	if((Temp_si)<=170) //minimum restvalue = 21 -> no "self-overun" from timer0 
	{
		ReloadValue_uc=Temp_si;
		sbi(TCCR0A,COM0A0);  // set low
		Temp_si=0;

		if(CycleCount_uc==ServoExt.NumberOfServos_ui)
		{
			CycleCount_uc=0;
			TotalFrameTime_si=0;
			PORTB &=~(1<<PB4); //Reset low active
			TempCounter_uc=0;
		}
		else
		{
			CycleCount_uc++;
		}
	}
	else
	{
		ReloadValue_uc=150; //shortest signal 0.6ms =~ 170bit
		Temp_si-=150;		

			//activate reset high after ~1.5ms after last cycle
			if(TempCounter_uc==4)
			{
				PORTB |= (1<<PB4);	//Reset high
			}
	}

	OCR0A=ReloadValue_uc;

#endif


#if (FS_SERVOEXT == FS_SERVOEXT_OFF)

	static unsigned char CycleCount_uc=0, ReloadValueA_uc=0, ReloadValueB_uc=0;
	static unsigned char NumberOfServoSeq_uc=0;
	static unsigned int TempA_ui=0, TempB_ui=0, TempC_ui=0;	
	
	#define TimeDiff12 864		//3ms timeshift between Servo 1 and 2
					
	//Sequence
	//Servo_ui[0]->TimeDiff12->Servo_ui[1]->UpdateRate_si

	/****************************/
	/*		172bit = 0.6ms		*/
	/*		432bit = 1.5ms		*/
	/*		692bit = 2.4ms		*/
	/****************************/	

	
	if (NumberOfServoSeq_uc==0 && CycleCount_uc==0)
	{
		TempA_ui = ServoOut1_ui;
		BOUND(TempA_ui,Var[Variant_uc].ParaName.P_SERVO1_MAX_ui,Var[Variant_uc].ParaName.P_SERVO1_MIN_ui)
		TempC_ui = TempA_ui; //save for 2. sequence!
	}

	if (NumberOfServoSeq_uc==1 && CycleCount_uc==0)
	{
		TempA_ui = TimeDiff12-TempC_ui;
	}

	if (NumberOfServoSeq_uc==3 && CycleCount_uc==0)
	{
		TempA_ui = Var[Variant_uc].ParaName.P_REFRESHRATESERVO_ui - TimeDiff12;
	}

//First Servo_ui[0]
	if((TempA_ui)<=170) //minimum restvalue = 21 -> no "self-overrun" from timer0 
	{
		ReloadValueA_uc=TempA_ui;
		TempA_ui=0;

		if(NumberOfServoSeq_uc==0)
		{
			NumberOfServoSeq_uc=1;
			cbi(TCCR0A,COM0A0);		// low level
		}
		else if(NumberOfServoSeq_uc==1)
		{
			NumberOfServoSeq_uc=3;

		}
		else if(NumberOfServoSeq_uc==3) 
		{
			NumberOfServoSeq_uc=0;
			sbi(TCCR0A,COM0A0);		// High level		
		}
		CycleCount_uc=0;
	}
	else
	{
		ReloadValueA_uc=150; //shortest signal 0.6ms =~ 170bit
		TempA_ui-=150;		
		CycleCount_uc++;
	}
// END Servo_ui[0]

//Servo[1]
if (NumberOfServoSeq_uc==3 && CycleCount_uc==0)
	{
		TempB_ui = ServoOut2_ui;
		BOUND(TempB_ui,Var[Variant_uc].ParaName.P_SERVO2_MAX_ui,Var[Variant_uc].ParaName.P_SERVO2_MIN_ui)
		ReloadValueB_uc=ReloadValueA_uc;
		sbi(TCCR0A,COM0B0);		// B high level
	}
else
	{
		if (TempB_ui<=150)
		{
			ReloadValueB_uc=TempB_ui;
			TempB_ui=0;
			cbi(TCCR0A,COM0B0);		// low level
		}
		else if(TempB_ui>150 && TempB_ui<170) //No "self-overrun" 
		{
			ReloadValueB_uc=TempB_ui;
			TempB_ui=0;
			cbi(TCCR0A,COM0B0);		// low level
				
			ReloadValueA_uc=TempB_ui; 
			TempA_ui-=(TempB_ui-150);		
		}
		else
		{
			ReloadValueB_uc=255;
			TempB_ui-=150;
		}
	}

	OCR0A=ReloadValueA_uc;
	OCR0B=ReloadValueB_uc;

#endif	/* Servo extension OFF												 */
}   //End of ISR
