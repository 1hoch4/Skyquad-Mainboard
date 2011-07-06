/*****************************************************************************/
/* Description: Main functions, task management, initialisation				 */
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <avr/wdt.h>
#include "FPL.h"

#include "config.h"
#include "analog.h"
#include "eeprom.h"
#include "radio.h"
#include "communication.h"
#include "InertSig.h"
#include "AttCtrl.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"


/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/

/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/

/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
volatile unsigned char Counter_1ms_uc = 0;
volatile unsigned char Counter_6ms_uc = 0;
volatile unsigned char Counter_12ms_uc = 4;
volatile unsigned char Counter_24ms_uc = 10;
volatile unsigned char Counter_96ms_uc = 70;

volatile unsigned char Variant_uc=0;

unsigned char BootDelay_uc =0;

ParaUnion_t Var[P_MaxVariant];
SW_VersionType EEPROM_Version;
SW_VersionType SW_Version;
PotiParaType Poti;


/*************************************/
/* 				GPS					 */
/*************************************/
GPS_Struct GPS;

signed long P_North_sl;
signed long D_North_sl;
signed long I_North_sl;
signed long P_East_sl;
signed long D_East_sl;
signed long I_East_sl;
signed int P_Yaw_si;
signed long Pos_Dev_North_sl;
signed long Pos_Dev_East_sl;
signed long Pos_Dev_Integ_North_sl;
signed long Pos_Dev_Integ_East_sl;
signed long Vel_Integ_North_sl;
signed long Vel_Integ_East_sl;
signed long Vel_Integ_North_no_sl;
signed long Vel_Integ_East_no_sl;
signed long Angle_Dev_Yaw_si;
signed long GPS_Home_North_sl;
signed long GPS_Home_East_sl;
unsigned int GPS_Start_Alti_ui;
signed long Tar_Pos_North_sl;
signed long Tar_Pos_East_sl;
signed int GPS_North_si;
signed int GPS_East_si;
signed int GPS_Pitch_si;
signed int GPS_Roll_si;
unsigned char targetreached_uc;
unsigned char waypoint_cnt_uc;
unsigned char nopilotctrl_cnt_uc;
unsigned char Gps_Mode_uc;
unsigned int MM3_angle_ui=0;
signed int GPS_Yaw_si = 0;
signed int heading2WP_si = 0;
signed int MM3_AngleError_si = 0;
unsigned int MM3_GyroCompass_ui = 0;
signed long Heading2Target_sl = 0;
signed int Heading_Error_si = 0;
unsigned int HomeDistance_ui = 0;
unsigned int Tar_Home_Distance_ui = 0;
signed int GPS_PI_North_WindComp_si = 0;
signed int GPS_PI_East_WindComp_si = 0;
signed int GPS_PI_North_si = 0;
signed int GPS_PI_East_si = 0;
signed int SideSlipAngle_si = 0;  //for Windcompensation
volatile unsigned int TargetDistance_ui = 0;

/*************************************/
/* 			Altitude Control		 */
/*************************************/
volatile unsigned char AltControllerActive_uc=0;	
volatile signed char AltControllerThrottle_sc=0;	
volatile signed int AltControllerControlP_si=0;
volatile signed int AltControllerControlD_si=0;
volatile signed int AltControllerDeviation_si=0;
volatile unsigned int PressureFF_ui=0;
volatile signed char AltControllerThrottleF_sc=0;
volatile signed int AltControllerControlAcc_si=0;
volatile unsigned int DAC_Value_ui =0;
volatile unsigned int Pressure_ui=0;
volatile unsigned int PressureF_ui=0;


/*************************************/
/* 			Buzzer from EXO			 */
/*************************************/

volatile unsigned char Buzzer_uc = 0; // 0 = OFF; 1 = ON;


/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/

/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void system_init(void);
void Parameter_init(void);
void MotorMixer_init(void);
void Poti_2_Param(void);
void timerinit(void);
void InitialCalibration(void);
void CatchInitFailure(void);
void SoftReset(void);
void tasktimer(void);


/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* system_init																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Initialise GPIO															 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void system_init(void)
{
	/*************************************/
	/* 				PORTS				 */
	/*************************************/
	//PORT A
	// PIN 0: ADC (Battery voltage)	INPUT
	// PIN 1: DIP 4					INPUT
	// PIN 2: DIP 3					INPUT
	// PIN 3: DIP 2					INPUT
	// PIN 4: DIP 1					INPUT
	// PIN 5: LED 5					OUTPUT
	// PIN 6: LED 4					OUTPUT
	// PIN 7: ADC (Current sensor)	INPUT
	DDRA = 0x60;
	PORTA = 0x7E;//Pullups der DIPs an
	
	//PORT B
	// PIN 0: PWR 1					OUTPUT
	// PIN 1: PWR 2					OUTPUT
	// PIN 2: PWR 3					OUTPUT
	// PIN 3: OC0A (Servo1)			OUTPUT
	// PIN 4: OC0B (Servo2)			OUTPUT
	// PIN 5: MOSI (SPI-Exo-Board)	OUTPUT
	// PIN 6: MISO (SPI-Exo-Board)	OUTPUT
	// PIN 7: SCK (SPI-Exo-Board)	OUTPUT
	DDRB = 0xFF;
	PORTB =0x07;
	
#if (FS_ACC_BUILT == FS_LISACC)
	//PORT C
	// PIN 0: SCL					OUTPUT
	// PIN 1: SDA					INPUT
	// PIN 2: 					    OUTPUT
	// PIN 3: 					    OUTPUT
	// PIN 4: 					    OUTPUT
	// PIN 5: CS_LIS				OUTPUT
	// PIN 6: LED 2					OUTPUT
	// PIN 7: LED 1					OUTPUT
	DDRC = 0xFD;
	PORTC = 0xF0;
#endif

#if (FS_ACC_BUILT == FS_BOSCHACC)
	//PORT C
	// PIN 0: SCL					OUTPUT
	// PIN 1: SDA					INPUT
	// PIN 2: TDI					INPUT
	// PIN 3: TDO					INPUT
	// PIN 4: TMS					INPUT
	// PIN 5: TCK					INPUT
	// PIN 6: LED 2					OUTPUT
	// PIN 7: LED 1					OUTPUT
	DDRC = 0xC1;
	PORTC = 0xC0;
#endif
	
	//PORT D
	// PIN 0: RX					INPUT
	// PIN 1: TX					OUTPUT
	// PIN 2: MISO (SPI-Sensors)	OUTPUT
	// PIN 3: MOSI (SPI-Sensors)	OUTPUT
	// PIN 4: SCK  (SPI-Sensors)	OUTPUT
	// PIN 5: CS   (SPI-Sensors)	OUTPUT
	// PIN 6: ICP  (Radio)			INPUT
	// PIN 7: CS   (SPI-EXO-Board)	OUTPUT
	DDRD = 0xBA;
	PORTD = 0x80;
	

	/*************************************/
	/* 		  External interrupts		 */
	/*************************************/

	EICRA=0x00;
	EIMSK=0x00;
	EIFR=0x00;
	PCICR=0x00;
	PCMSK0=0x00;


	/*************************************/
	/* 		  		Timer				 */
	/*************************************/

	// Timer/Counter 0 initialization (8bit) Servos
	// Timer disabled
	// Interrupts disabled
	TCCR0A=0x00;
	TCCR0B=0x00;
	TCNT0=0x00;
	OCR0A=0x00;
	OCR0B=0x00;
	TIMSK0=0x00;
	TIFR0=0x00;

	// Timer/Counter 1 initialization (16bit) Funke
	// Timer disabled
	// Interrupts disabled
	TCCR1A=0x00;
	TCCR1B=0x00;
	TCCR1C=0x00;
	TCNT1H=0x00;
	TCNT1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	TIMSK1=0x00;
	TIFR1=0x00;


	// Timer/Counter 2 initialization (8bit) Taskmanagement
	// Timer disabled
	// Interrupts disabled
	TCCR2A=0x00;
	TCCR2B=0x00;
	TCNT2=0x00;
	OCR2A=0x00;
	OCR2B=0x00;
	ASSR=0x00;
	TIMSK2=0x00;
	TIFR2=0x00;
	GTCCR=0x00;


	/*************************************/
	/* 		  		USART				 */
	/*************************************/

	// USART0 initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART0 Receiver: On
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	// USART0 Baud Rate: 115200 @ 18,432MHz
	UCSR0A=0x00;
	UCSR0B=0x18;
	UCSR0C=0x06;
	UBRR0H=0x00;
	UBRR0L=0x09;

	// USART1 initialization
	// Communication Parameters:
	// USART1 Receiver: Off
	// USART1 Transmitter: Off
	// USART1 Mode: Asynchronous
	// USART1 Baud Rate: 0
	UCSR1A=0x00;
	UCSR1B=0x00;
	UCSR1C=0x00;
	UBRR1H=0x00;
	UBRR1L=0x00;

	/*************************************/
	/* 		   Analog Comparator		 */
	/*************************************/

	// Analog Comparator initialization
	// Analog Comparator: Off
	ACSR=0x80;


	/*************************************/
	/* 		   		SPI					 */
	/*************************************/

	// SPI initialization
	// SPI Type: Master
	// SPI Clock Rate: 1000,000 kHz
	// SPI Clock Phase: Cycle Half
	// SPI Clock Polarity: Low
	// SPI Data Order: MSB First
	SPCR=0x51;
	SPSR=0x00;


	/*************************************/
	/*			  Variables				 */
	/*************************************/

	i2c_error_ui=100;

	SW_Version.Major_uc = SW_MAJOR;
	SW_Version.Minor_uc = SW_MINOR;
	EEPROM_Version.Major_uc = EE_MAJOR;
	EEPROM_Version.Minor_uc = EE_MINOR;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Parameter_init															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Global parameter default initialisation									 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void Parameter_init(void)
{
	signed char iVarCnt;
	unsigned char tempcounter_uc=0;

	for(iVarCnt=0; iVarCnt<P_MaxVariant; iVarCnt++)
	{
		Var[iVarCnt].ParaName.P_STICK_HEADINGHOLD_THRESHOLD_ui=3;
		Var[iVarCnt].ParaName.P_FACTOR_STICK_ROPI_CONTROLLER_ui=0;
		Var[iVarCnt].ParaName.P_FACTOR_STICK_ROPI_DIRECT_ui =112;
		Var[iVarCnt].ParaName.P_FACTOR_STICK_ROPI_HEADINGHOLD_ui =77;
		Var[iVarCnt].ParaName.P_FACTOR_STICK_YAW_DIRECT_ui =90;
		Var[iVarCnt].ParaName.P_STICK_YAW_THRESHOLD_ui =6; 
		Var[iVarCnt].ParaName.P_LOOP_PITCH_ON_THRESHOLD_ui =90;
		Var[iVarCnt].ParaName.P_LOOP_PITCH_OFF_HYST_ui =30;
		Var[iVarCnt].ParaName.P_LOOP_ROLL_ON_THRESHOLD_ui =90;
		Var[iVarCnt].ParaName.P_LOOP_ROLL_OFF_HYST_ui =30;
		Var[iVarCnt].ParaName.P_ACCU_EMPTY_MOTOR_REDUCTION_ui =0;
		Var[iVarCnt].ParaName.P_EMER_THRO_ui =90;
		Var[iVarCnt].ParaName.P_EMER_THRO_ACCU_EMPTY_ui =66;
		Var[iVarCnt].ParaName.P_EMER_THRO_DURATION_ui =2500;
		Var[iVarCnt].ParaName.P_YAW_LIMIT_ui =0;
		Var[iVarCnt].ParaName.P_MIN_THRO_ui =15;
		Var[iVarCnt].ParaName.P_MAX_THRO_ui =255;
		Var[iVarCnt].ParaName.P_MAX_THRO_STICK_ui =240;		
		Var[iVarCnt].ParaName.P_MIN_THRO_CONTROLLER_ACTIVE_ui =8;
		Var[iVarCnt].ParaName.P_ACCU_EMPTY_VOLTAGE_ui =320;
		Var[iVarCnt].ParaName.P_CURRENT_FILTER_ui =8;
		Var[iVarCnt].ParaName.P_VOLTAGE_FILTER_ui=3;
		Var[iVarCnt].ParaName.P_KP_ROLL_PITCH_ui=175;
		Var[iVarCnt].ParaName.P_KD_ROLL_PITCH_ui=100;
		Var[iVarCnt].ParaName.P_KP_YAW_ui=200;
		Var[iVarCnt].ParaName.P_KD_YAW_ui=138;
		Var[iVarCnt].ParaName.P_RADIO_FILTER_ui=3;
		Var[iVarCnt].ParaName.P_CAMERACOMP_ON_OFF_ui=0;
		Var[iVarCnt].ParaName.P_CAMERAPITCHCOMP_ON_OFF_ui=200;
		Var[iVarCnt].ParaName.P_CAMERAROLLCOMP_ON_OFF_ui=200;
		Var[iVarCnt].ParaName.P_CAMERAPITCHCOMP_GAIN_ui=187;
		Var[iVarCnt].ParaName.P_CAMERAPITCHCOMP_OFFSET_ui=127;
		Var[iVarCnt].ParaName.P_CAMERAROLLCOMP_GAIN_ui=187;
		Var[iVarCnt].ParaName.P_CAMERAROLLCOMP_OFFSET_ui=127;
		Var[iVarCnt].ParaName.P_SERVO1_VALUE_ui=127;
		Var[iVarCnt].ParaName.P_SERVO1_GAIN_ui=54;
		Var[iVarCnt].ParaName.P_SERVO1_OFFSET_ui=127;
		Var[iVarCnt].ParaName.P_SERVO1_MAX_ui=576;
		Var[iVarCnt].ParaName.P_SERVO1_MIN_ui=288;
		Var[iVarCnt].ParaName.P_SERVO2_VALUE_ui=127;
		Var[iVarCnt].ParaName.P_SERVO2_GAIN_ui=54;
		Var[iVarCnt].ParaName.P_SERVO2_OFFSET_ui=127;
		Var[iVarCnt].ParaName.P_SERVO2_MAX_ui=576;
		Var[iVarCnt].ParaName.P_SERVO2_MIN_ui=288;
		Var[iVarCnt].ParaName.P_REFRESHRATESERVO_ui=5760;
		Var[iVarCnt].ParaName.P_POWER_VALUE1_ui=0;
		Var[iVarCnt].ParaName.P_POWER_VALUE2_ui=0;
		Var[iVarCnt].ParaName.P_POWER_VALUE3_ui=0;
		Var[iVarCnt].ParaName.P_POWER_THRESHOLD1_ui=128;
		Var[iVarCnt].ParaName.P_POWER_THRESHOLD2_ui=128;
		Var[iVarCnt].ParaName.P_POWER_THRESHOLD3_ui=128;
		Var[iVarCnt].ParaName.FS_HEADING_HOLD_ui=0;
		Var[iVarCnt].ParaName.FS_MAINBOARD_ROTATED_ui=0;
	}

/* Init for the Servo extension	*/

		ServoExt.NumberOfServos_ui=4;		
		ServoExt.UpdateRate_ui=5760;			

		for(tempcounter_uc=0;tempcounter_uc<10;tempcounter_uc++)
		{
			ServoExt.ServoOut_ui[tempcounter_uc]=432;	
			ServoExt.ServoOutMax_ui[tempcounter_uc]=576; 
			ServoExt.ServoOutMin_ui[tempcounter_uc]=288; 
			ServoExt.ServoOutOffset_si[tempcounter_uc]=127;
			ServoExt.ServoOutGain_si[tempcounter_uc]=128;	
		}

}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* MotorMixer_init															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Factors of Motormixer default init for + Quad setting					 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void MotorMixer_init(void)
{
    MotorGain.Amount_uc = 4;
	//Motor_1
	MotorGain.FMT_sc[0] = 32;
	MotorGain.FMP_sc[0] = -32;
    MotorGain.FMR_sc[0] = 0;	
    MotorGain.FMY_sc[0] = -32;	
    //Motor_2
	MotorGain.FMT_sc[1] = 32;
	MotorGain.FMP_sc[1] = +32;	
    MotorGain.FMR_sc[1] = 0;	
	MotorGain.FMY_sc[1] = -32;	
    //Motor_3
	MotorGain.FMT_sc[2] = 32;
	MotorGain.FMP_sc[2] = 0;
    MotorGain.FMR_sc[2] = +32;
	MotorGain.FMY_sc[2] = +32;
	//Motor_4
	MotorGain.FMT_sc[3] = 32;
	MotorGain.FMP_sc[3] = 0;
    MotorGain.FMR_sc[3] = -32;		
    MotorGain.FMY_sc[3] = +32;
	//Motor_5
	MotorGain.FMT_sc[4] = 0;
	MotorGain.FMP_sc[4] = 0;
    MotorGain.FMR_sc[4] = 0;	
    MotorGain.FMY_sc[4] = 0;	
    //Motor_6
	MotorGain.FMT_sc[5] = 0;
	MotorGain.FMP_sc[5] = 0;	
    MotorGain.FMR_sc[5] = 0;	
	MotorGain.FMY_sc[5] = 0;	
    //Motor_7
	MotorGain.FMT_sc[6] = 0;
	MotorGain.FMP_sc[6] = 0;
    MotorGain.FMR_sc[6] = 0;
	MotorGain.FMY_sc[6] = 0;
	//Motor_8
	MotorGain.FMT_sc[7] = 0;
	MotorGain.FMP_sc[7] = 0;
    MotorGain.FMR_sc[7] = 0;		
    MotorGain.FMY_sc[7] = 0;			
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* RadioMapping_init														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Initialization of Radio-Mapping	values									 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void RadioMapping_init (void)
{
	RC_Map.Throttle_uc = 3;
	RC_Map.Pitch_uc = 1;
	RC_Map.Roll_uc = 2;
	RC_Map.Yaw_uc = 4;
	RC_Map.UserFunc1_uc = 5;
	RC_Map.UserFunc2_uc = 6;
	RC_Map.UserFunc3_uc = 7;
	RC_Map.UserFunc4_uc = 8;
	RC_Map.UserFunc5_uc = 9;
	RC_Map.UserFunc6_uc = 10;
	RC_Map.UserFunc7_uc = 11;
	RC_Map.UserFunc8_uc = 12;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Poti_2_Param																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Copy RadioPoti values to specified Parameters							 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void Poti_2_Param(void)
{
	/* Parameter value range must be same as poti range (0 - 255)			 */
	P2P(Poti.P_KP_ROLL_PITCH_ui,Var[Variant_uc].ParaName.P_KP_ROLL_PITCH_ui);
 	P2P(Poti.P_KD_ROLL_PITCH_ui,Var[Variant_uc].ParaName.P_KD_ROLL_PITCH_ui);
	P2P(Poti.P_KP_YAW_ui,Var[Variant_uc].ParaName.P_KP_YAW_ui);
	P2P(Poti.P_KD_YAW_ui,Var[Variant_uc].ParaName.P_KD_YAW_ui);
	P2P(Poti.P_CAMERACOMP_ON_OFF_ui,Var[Variant_uc].ParaName.P_CAMERACOMP_ON_OFF_ui);
	P2P(Poti.P_CAMERAPITCHCOMP_ON_OFF_ui,Var[Variant_uc].ParaName.P_CAMERAPITCHCOMP_ON_OFF_ui);
	P2P(Poti.P_CAMERAROLLCOMP_ON_OFF_ui,Var[Variant_uc].ParaName.P_CAMERAROLLCOMP_ON_OFF_ui);
	P2P(Poti.P_CAMERAPITCHCOMP_OFFSET_ui,Var[Variant_uc].ParaName.P_CAMERAPITCHCOMP_OFFSET_ui);
	P2P(Poti.P_CAMERAROLLCOMP_OFFSET_ui,Var[Variant_uc].ParaName.P_CAMERAROLLCOMP_OFFSET_ui);
	P2P(Poti.P_SERVO1_VALUE_ui,Var[Variant_uc].ParaName.P_SERVO1_VALUE_ui);
	P2P(Poti.P_SERVO2_VALUE_ui,Var[Variant_uc].ParaName.P_SERVO2_VALUE_ui);
	P2P(Poti.P_POWER_VALUE1_ui,Var[Variant_uc].ParaName.P_POWER_VALUE1_ui);
	P2P(Poti.P_POWER_VALUE2_ui,Var[Variant_uc].ParaName.P_POWER_VALUE2_ui);
	P2P(Poti.P_POWER_VALUE3_ui,Var[Variant_uc].ParaName.P_POWER_VALUE3_ui);
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* timerinit																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Initialisation of Timer2. Used for taskmanagement						 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void timerinit(void)
{
//Timer2:
//	18,432MHz
	TCCR2A = (1 << WGM21);	// CTC Mode
	TCCR2B = (1 << CS22) | (1 << CS20); // Teiler 128
	OCR2A  = 144;
	TIMSK2 |= ( 1 << OCIE2A); // Output Compare Match Interrunpt einschalten
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* InitialCalibration														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Calibration for inertial sensors and radio control at first startup or 	 */
/* if EEPROM Version has changed due to SW-update							 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void InitialCalibration(void)
{
	POWER1_OFF;
	BUZZER_OFF;
	LED_RED_ON;

	/* Enable interrupts													 */
	sei();
	
	
	/*************************************************************************/
	/*						Task-timing for 1st Startup						 */
	/*************************************************************************/

	/* Radio calibration													 */
	while (Eeprom_CalDataSecRemoteCtrl.RadioCalibrated_uc != 1)
	{
		
		if (Counter_1ms_uc >= 96)
		{
			Counter_1ms_uc = 0;
			
			LED_YELLOW_FLASH;
			Radio_Channel_Calib();
		}
	}
	/* Set TaskCounters back to initial values								 */
	Counter_1ms_uc = 0;
	Counter_6ms_uc = 0;
	Counter_96ms_uc = 70;
	

	/* Inertial sensors calibration											 */
	while (Eeprom_CalDataSecOffsCorr.InertSigCalibrated != 1)
	{
		if (Counter_1ms_uc >= 3)
		{
			Counter_1ms_uc = 0;
			Counter_6ms_uc += 3;
			Counter_96ms_uc += 3;

			// Read Sensor data
			InertSig_PollAndPreprocess();
			sendData();
		}


		if (Counter_6ms_uc >= 6)
		{
			Counter_6ms_uc = 0;

			if (NewRadioData_uc == TRUE)
			{
				Radio_Normalisation();
				NewRadioData_uc = FALSE;
			}
			
			
			if (SensOffsCount_ui)
			{
		  		InertSig_CalibOffsets();
			}
			Radio_Signal_Check();
		}

		if (Counter_96ms_uc >= 96)
		{
			Counter_96ms_uc = 0;
			
			LED_GREEN_FLASH;
			Radio_Stick_Check();
		}
	}
	/* Set TaskCounters back to initial values								 */
	Counter_1ms_uc = 0;
	Counter_6ms_uc = 0;
	Counter_96ms_uc = 70;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* CatchInitFailure															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Rewrites EEPROM with default values, if CRC at startup is incorrect		 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void CatchInitFailure(void)
{
	signed char iVarCnt;
	unsigned char i_uc;
	
	Parameter_init();
	MotorMixer_init();
	RadioMapping_init();
	
	/* Write default parameter-values to EEPROM								 */
	for(iVarCnt=0; iVarCnt<P_MaxVariant; iVarCnt++)
	{
		Eeprom_WriteToVariantSetting(iVarCnt);
	}
	
	/* Write MotorMixer defaults to EEPROM									 */
	Eeprom_WriteToMotorMixerSection();
	
	/* Write RadioMapping defaults to EEPROM								 */
	Eeprom_WriteToRadioMappingSection();

	/* Write system setting defaults										 */
	Variant_uc=0;								
	Eeprom_CalSystemSetting.VarianteEEPROM_uc=0;
	
	// EEPROM Version schreiben
	Eeprom_CalSystemSetting.EE_Version_Major_uc = EE_MAJOR;
	Eeprom_CalSystemSetting.EE_Version_Minor_uc = EE_MINOR;

	Eeprom_WriteToSystemSetting();


	/* Write inert. signals-offset defaults									 */
	Eeprom_CalDataSecOffsCorr.OffsGyroX_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsGyroY_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsGyroZ_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsHGyroX_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsHGyroY_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsHGyroZ_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsAccX_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsAccY_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.OffsAccZ_si = 0xAA;
	Eeprom_CalDataSecOffsCorr.InertSigCalibrated = 0;

	Eeprom_WriteToOffsCorrSection();


	/* Write radio calibration defaults										 */
	for (i_uc = 1; i_uc<13; i_uc++)
	{
		Faktor_MaxFunke_Kanal_si[i_uc] = -2;		
		Faktor_MinFunke_Kanal_si[i_uc] = -2;		
		OffsFunke_Kanal_si[i_uc] = 0;				
	}
	Eeprom_CalDataSecRemoteCtrl.RadioCalibrated_uc = 0;

	Eeprom_WriteToRemoteCtrlSection();


	/* If 2nd trial fails, EEPROM is damaged --> do nothing any more!		 */
	if (Eeprom_ReadAll() == RET_FAIL)
	{
		/*EEPROM defect, turn on LED and Buzzer								 */
		POWER1_ON;
		BUZZER_ON;
		LED_RED_ON;
		/* -> do nothing any more											 */
		while(1);
	}
	
	/*New written data could be read correctly --> 1st Startup				 */
	else
	{
		InitialCalibration();
	}

}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* ISR(TIMER2_COMPA_vect)													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Interrupt service routine each 1ms for taskmanagement					 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
ISR(TIMER2_COMPA_vect)
{
	Counter_1ms_uc++;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SoftReset																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Resets device if DIP1 and Button is pressed via Watchdog after 500ms		 */
/* for Bootloader support													 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void SoftReset(void)
{

	if DIP1_ON // DIP1 gesetzt?
	{
		/* Button -> Software Reset (Bootloader)							 */
	    if BUTTON_ON
		{
			/* WDT after 500ms												 */
			wdt_enable(WDTO_500MS);
			while(1);
		}
	}
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* tasktimer																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Taskmanagement, generates different tasks based upon 1ms (Timer2)		 */
/* 3	ms Task	(Maintask)													 */
/* 6	ms Task																 */
/* 12	ms Task																 */
/* 24	ms Task																 */
/* 96	ms Task																 */
/* In each iteration, only 3ms-Task and one other is executed to ensure		 */
/* processing does not last longer than 3ms at all!!!						 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void tasktimer(void)
{
	/*************************************/
	/*			  3ms Task				 */
	/*************************************/
	if (Counter_1ms_uc >= 3)
	{
		Counter_1ms_uc = 0;
		Counter_6ms_uc +=3;
		Counter_12ms_uc +=3;
		Counter_24ms_uc +=3;
		Counter_96ms_uc +=3;

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	/* Functions executed in 3ms Task										 */

		InertSig_GetAndProcessSignals();
		AttCtrl_CalcCtrlOutp();
		AttCtrl_ExecMotorMixer();
		i2c_motor_send();
		sendData();
		EXO_DataTransfer();
		Servo_Output();
	}
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*************************************/
	/*			  6ms Task				 */
	/*************************************/
	if (Counter_6ms_uc >= 6)
	{
		Counter_6ms_uc = 0;

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	/* Functions executed in 6ms Task										 */

		if (NewRadioData_uc == TRUE)
		{
			Radio_Normalisation();
			NewRadioData_uc = FALSE;
		}

		if (SensOffsCount_ui)
		{
	  		InertSig_CalibOffsets();
		}
		Radio_Signal_Check();
	}
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


	/*************************************/
	/*			  12ms Task				 */
	/*************************************/
	if (Counter_12ms_uc >= 12)
	{
		Counter_12ms_uc = 0;

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	/* Functions executed in 12ms Task										 */

	}
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*************************************/
	/*			  24ms Task				 */
	/*************************************/
	if (Counter_24ms_uc >= 24)
	{
		Counter_24ms_uc = 0;

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	/* Functions executed in 24ms Task										 */

		SoftReset();
	}
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*************************************/
	/*			  96ms Task				 */
	/*************************************/
	if (Counter_96ms_uc >= 96)
	{
		Counter_96ms_uc = 0;

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	/* Functions executed in 96ms Task										 */
		
		Radio_Stick_Check();
		Radio_Channel_Calib();
		read_adc();
		Est_Hov_Throttle();
		Poti_2_Param();
		Power_Output();
		Buzzer(0,0);
		//delay counter after booting to avoid emergency buzzer due to
		//necessary spektrum binding
		if(BootDelay_uc < 250) BootDelay_uc++;

	}
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* main																		 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Main loop																 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
int main(void)
{
	system_init();
	Parameter_init();
	MotorMixer_init();
	RadioMapping_init();
	uart_init();
	SPI_Init();
	i2c_init();
	timerinit();
	Radio_Init();
	ADC_Init();
#if (FS_ACC_BUILT == FS_LISACC)
	LISACC_Init();
#endif//(FS_ACC_BUILT == FS_LISACC)
	
	/* Read EEPROM and CRC chech											 */
	if (Eeprom_ReadAll() == RET_FAIL)
	{
		CatchInitFailure();
	}
	
	/* Set active variant (default = 0)										 */
	Variant_uc = Eeprom_CalSystemSetting.VarianteEEPROM_uc;
	
	
	/* EEPROM Version changed?												 */
	if (  (Eeprom_CalSystemSetting.EE_Version_Major_uc
			!= EEPROM_Version.Major_uc)
		||(Eeprom_CalSystemSetting.EE_Version_Minor_uc
			!= EEPROM_Version.Minor_uc))
	{
		CatchInitFailure();
	}
	
	/* Radio or InertialSensors calibrated?									 */
	if (!Eeprom_CalDataSecRemoteCtrl.RadioCalibrated_uc ||
		!Eeprom_CalDataSecOffsCorr.InertSigCalibrated)
	{
		InitialCalibration();
	}

	/* Enable interrupts													 */
	sei();

	
	//*************************************************************
	// check for number of battery cells after startup
	unsigned char icells_uc = 0;	

	for(icells_uc=0; icells_uc<100; icells_uc++)
	{
		read_adc();

		if(UBatF_ui>1270) Cells_uc = 4;
		else Cells_uc = 3;
	}

	//Tell user I am ready and show recognised number of cells after startup
	if(Cells_uc == 3)
	{
		Buzzer(3,Long);	//(3x long)
	}
	else if(Cells_uc == 4)
	{
		Buzzer(4,Long);	//(4x long)
	}
	//*************************************************************	

	while (1)
	{
		/* Execute waiting tasks											 */
		tasktimer();


		/* For debugging purposes Button could be used...					 */
		if BUTTON_ON
		{

		}
		
/*		 Buzzer from EXO activated?										 */
		if(Buzzer_uc == 1)
		{
			Buzzer(2,Short);
			Buzzer_uc = 0;
		}


	}
}
