/*****************************************************************************/
/* Description: Communication with external devices							 */
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
#include <stdlib.h>

#include "config.h"
#include "eeprom.h"
#include "i2c.h"
#include "uart.h"
#include "analog.h"
#include "InertSig.h"
#include "AttCtrl.h"
#include "radio.h"
#include "communication.h"

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/

/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/
#define SPI_EXO_MAX_ERROR 5


/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
volatile SPI_Union SPI_EXO_Data;
volatile UART_RX_TX_DataUnion UART_RX_TX_Data;
unsigned int SPI_EXO_All_errors_ui = 0;
unsigned char EXO_Connected_uc = 0;
unsigned char BuzzerStateInt_uc = Standby,BuzzerStateIntSave_uc = Standby;
unsigned char BuzzerLongState_uc=0,BuzzerShortState_uc=0,BuzzerAmountInt_uc=0;
unsigned char OffTimer_uc = 0,OnTimer_uc=0;
/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/
unsigned char SPI_EXO_error_uc = 0;
unsigned char SPI_EXO_ID_Counter = 1;


/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void sendData(void);
void EXO_DataTransfer(void);
void Buzzer(unsigned char newBuzzercase_uc,
			unsigned char Buzzerstate_uc);


/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* sendData																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Send data via USART according to recieve buffer 							 */
/* YYY kurze Erklärung über Aufbau und Funktion der Messtechnik einfügen	 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void sendData(void)
{
	static char sendID_Counter=0;

	/* Data recieved?														 */
	if(SendData_uc==1)
	{
		switch (Receive_Char_uc)
		{
			/* Determine transmission										 */

			case 0:
					SendData_uc=0;
			break;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Following two lines are jump markers for SQ-BaseStation tool!!!			 */
/* This information is 	necessary for each defined dataset					 */
/* 1st Line: Beginning of Dataset											 */
/* 2nd Line: Name of Dataset (between #-character)							 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//#Dataset_START#
//Datasetname:#GeneralData#
			/* Dataset 1 is requested, transmit one Block each 3ms Task		 */
			case 1:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(RadioQuali_uc);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#250#
					uart_write_int(UBat_ui);			//Unit:#V# Offset:#0# Factor:#0,01# Divisor:#1# GraphMIN:#0# GraphMAX:#15#
					uart_write_int(Current_si);			//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#30000#
					uart_write_int(Accu_Cap_ui);		//Unit:#mAh# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#5000#

					sendID_Counter++;
				break;

				case 1:
					uart_write_int(MotorsOn_uc);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-10# GraphMAX:#1,1#
					uart_write_int(SQinAir_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(EmergLand_uc);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(EmergLandTime_ui);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 2:
					uart_write_int(i2c_error_ui);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-10# GraphMAX:#200#
					uart_write_int(LoopActiv_uc);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(RollActiv_uc);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(EstHovThr_ui);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 3:
					uart_write_int(SQ_is_static_uc);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(EXO_Connected_uc);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Cells_uc);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#Compass#
			/* Dataset 2 is requested, transmit one Block each 3ms Task		 */
			case 2:
					uart_write_int(MM3_angle_ui);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#182,044# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(MM3_GyroCompass_ui);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#182,044# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(MM3_AngleError_si);	//Unit:#°# Offset:#0# //Factor:#1# Divisor:#182,044# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(MaxAngle_si);		//Unit:#°# Offset:#0# //Factor:#1# Divisor:#182,044# GraphMIN:#0# GraphMAX:#1000#

					SendData_uc=0;
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#Radio#
			/* Dataset 3 is requested, transmit one Block each 3ms Task		 */
			case 3:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(Stick_Throttle_ui);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Stick_Pitch_si);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Stick_Roll_si);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Stick_Yaw_si);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#

					sendID_Counter++;
				break;

				case 1:
					uart_write_int(Poti1_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Poti2_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Poti3_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Poti4_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#

					sendID_Counter++;
				break;

				case 2:
					uart_write_int(Poti5_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Poti6_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Poti7_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#
					uart_write_int(Poti8_ui);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-1000# GraphMAX:#300#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#MotorOutputs#
			/* Dataset 4 is requested, transmit one Block each 3ms Task		 */
			case 4:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(OutpSig.Motor_si[0]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Motor_si[1]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Motor_si[2]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Motor_si[3]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter++;
				break;

				case 1:
					uart_write_int(OutpSig.Motor_si[4]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Motor_si[5]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Motor_si[6]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Motor_si[7]);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#InertialSignals#
			/* Dataset 5 is requested, transmit one Block each 3ms Task		 */
			case 5:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(InertSig_Gyro.Omg_si[0]);	//Unit:#°/s# Offset:#0# Factor:#1# Divisor:#175# GraphMIN:#-100# GraphMAX:#100#
					uart_write_int(InertSig_Gyro.Omg_si[1]);	//Unit:#°/s# Offset:#0# Factor:#1# Divisor:#175# GraphMIN:#-100# GraphMAX:#100#
					uart_write_int(InertSig_Gyro.Omg_si[2]);	//Unit:#°/s# Offset:#0# Factor:#1# Divisor:#175# GraphMIN:#-100# GraphMAX:#100#

					sendID_Counter++;
				break;

				case 1:

					uart_write_int(InertSig_Gyro.Phi_sl[0]>>8);	//Unit:#°# Offset:#0# Factor:#1# Divisor:#182,044# GraphMIN:#-180# GraphMAX:#720#
					uart_write_int(InertSig_Gyro.Phi_sl[1]>>8);	//Unit:#°# Offset:#0# Factor:#1# Divisor:#182,044# GraphMIN:#-180# GraphMAX:#720#
					uart_write_int(InertSig_Gyro.Phi_sl[2]>>8);	//Unit:#°# Offset:#0# Factor:#1# Divisor:#182,044# GraphMIN:#-180# GraphMAX:#720#

					sendID_Counter++;
				break;

				case 2:
					uart_write_int(InertSig_Acc.PhiX_si);		//Unit:#°# Offset:#0# Factor:#1# Divisor:#182,044# GraphMIN:#-180# GraphMAX:#720#
					uart_write_int(InertSig_Acc.PhiY_si);		//Unit:#°# Offset:#0# Factor:#1# Divisor:#182,044# GraphMIN:#-180# GraphMAX:#720#

					sendID_Counter++;
				break;

				case 3:
					uart_write_int(InertSig_Acc.AccFilt_si[0]);	//Unit:#m/s²# Offset:#0# Factor:#9,81# Divisor:#6666# GraphMIN:#-15# GraphMAX:#100#
					uart_write_int(InertSig_Acc.AccFilt_si[1]);	//Unit:#m/s²# Offset:#0# Factor:#9,81# Divisor:#6666# GraphMIN:#-15# GraphMAX:#100#
					uart_write_int(InertSig_Acc.AccFilt_si[2]);	//Unit:#m/s²# Offset:#0# Factor:#9,81# Divisor:#6666# GraphMIN:#-15# GraphMAX:#100#
					uart_write_int(InertSig_Acc.AccResultant_ui);//Unit:#m/s²# Offset:#0# Factor:#9,81# Divisor:#6666# GraphMIN:#-15# GraphMAX:#100#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#

//#Dataset_START#
//Datasetname:#AttCtrl#
			/* Dataset 6 is requested, transmit one Block each 3ms Task		 */
			case 6:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(CmdRoll_si);					//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(CmdPitch_si);				//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(CmdYaw_si);					//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter++;
				break;

				case 1:
					uart_write_int(InternSig.ErrorRoll_si);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(InternSig.ErrorPitch_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(InternSig.ErrorYaw_si);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter++;
				break;

				case 2:
					uart_write_int(InternSig.PCmpntRoll_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(InternSig.PCmpntPitch_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(InternSig.PCmpntYaw_si);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter++;
				break;

				case 3:
					uart_write_int(InternSig.DCmpntRoll_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(InternSig.DCmpntPitch_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(InternSig.DCmpntYaw_si);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter++;
				break;

				case 4:
					uart_write_int(OutpSig.Roll_si);			//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Pitch_si);			//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(OutpSig.Yaw_si);				//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#AltController#
			/* Dataset 7 is requested, transmit one Block each 3ms Task		 */
			case 7:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(AltControllerActive_uc);		//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(DAC_Value_ui);				//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(Pressure_ui);				//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(PressureF_ui);				//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter++;
				break;

				case 1:
					uart_write_int(PressureFF_ui);				//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(AltControllerDeviation_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(AltControllerControlP_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(AltControllerControlD_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter++;
				break;

				case 2:
					uart_write_int(AltControllerControlAcc_si);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(AltControllerThrottle_sc);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(AltControllerThrottleF_sc);	//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#
					uart_write_int(Stick_Gas_Comp_ui);			//Unit:#-# Offset:#0# Factor:#1# Divisor:#1# GraphMIN:#-400# GraphMAX:#600#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#GPS#
			/* Dataset 8 is requested, transmit one Block each 3ms Task		 */
			case 8:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(GPS.GPSFix_uc);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS.noSV_uc);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Gps_Mode_uc);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(nopilotctrl_cnt_uc);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 1:
					uart_write_long(GPS.lati_sl);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_long(GPS.longi_sl);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 2:
					uart_write_int(GPS.GSpeed_sl);			//Unit:#cm/s# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS.velNorth_sl);		//Unit:#cm/s# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS.velEast_sl);			//Unit:#cm/s# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS.velDown_sl);			//Unit:#cm/s# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 3:
					uart_write_long(Pos_Dev_North_sl);		//Unit:#cm# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_long(Pos_Dev_East_sl);		//Unit:#cm# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 4:
					uart_write_long(Pos_Dev_Integ_North_sl);//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_long(Pos_Dev_Integ_East_sl);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 5:
					uart_write_int(Vel_Integ_North_sl);		//Unit:#cm# Offset:#0# //Factor:#0,1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Vel_Integ_East_sl);		//Unit:#cm# Offset:#0# //Factor:#0,1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Vel_Integ_North_no_sl);	//Unit:#cm# Offset:#0# //Factor:#0,1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Vel_Integ_East_no_sl);	//Unit:#cm# Offset:#0# //Factor:#0,1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 6:
					uart_write_int(P_North_sl);				//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(P_East_sl);				//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(I_North_sl);				//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(I_East_sl);				//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 7:
					uart_write_int(GPS_PI_North_si);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_PI_East_si);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_PI_North_WindComp_si);//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_PI_East_WindComp_si);//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 8:
					uart_write_int(D_North_sl);				//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(D_East_sl);				//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_North_si);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_East_si);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 9:
					uart_write_int(GPS_Pitch_si);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_Roll_si);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 10:
					uart_write_long(Heading2Target_sl);		//Unit:#°# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Angle_Dev_Yaw_si);		//Unit:#°# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_Yaw_si);				//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 11:
					uart_write_int(heading2WP_si);			//Unit:#°# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS.heading_sl);			//Unit:#°# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Heading_Error_si);		//Unit:#°# Offset:#0# //Factor:#0,0054945054# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(SideSlipAngle_si);		//Unit:#°# Offset:#0# //Factor:#0,0054945054# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#Waypoint_Flight#
			/* Dataset 9 is requested, transmit one Block each 3ms Task		 */
			case 9:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(RadioQuali_uc);			//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(UBat_ui);				//Unit:#V# Offset:#0# //Factor:#1# Divisor:#100# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Accu_Cap_ui);			//Unit:#mAh# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 1:
					uart_write_long(GPS.altitude_sl);		//Unit:#m# Offset:#0# //Factor:#1#Divisor:#100# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS_Start_Alti_ui);		//Unit:#m# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(GPS.GSpeed_sl);			//Unit:#km/h# Offset:#0# //Factor:#0,036# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 2:
					uart_write_long(GPS.lati_sl);			//Unit:#°# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_long(GPS.longi_sl);			//Unit:#°# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 3:
					uart_write_int(HomeDistance_ui);		//Unit:#m# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-100# GraphMAX:#1000#
					uart_write_int(TargetDistance_ui);		//Unit:#m# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Tar_Home_Distance_ui);	//Unit:#m# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#-100# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 4:
					uart_write_int(waypoint_cnt_uc);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(targetreached_uc);		//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#SensorOffsets#
			/* Dataset 10 is requested, transmit one Block each 3ms Task		 */
			case 10:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsGyroX_si);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsGyroY_si);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsGyroZ_si);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 1:
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsHGyroX_si);//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsHGyroY_si);//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsHGyroZ_si);//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 2:
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsAccX_si);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsAccY_si);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(Eeprom_CalDataSecOffsCorr.OffsAccZ_si);	//Unit:#-# Offset:#0# //Factor:#1# Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#


//#Dataset_START#
//Datasetname:#HeadingHold#
			/* Dataset 11 is requested, transmit one Block each 3ms Task		 */
			case 11:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(InternSig.PCmpntRoll_K1_si);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(InternSig.PCmpntPitch_K1_si);//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(InternSig.PCmpntRoll_si);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(InternSig.PCmpntPitch_si);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 1:
					uart_write_int(InpSig.SetPointRollRate_si);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(InpSig.SetPointPitchRate_si);//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(InternSig.ErrorRoll_si);		//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(InternSig.ErrorPitch_si);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 2:
					uart_write_long(InpSig.SetPointRoll_HH_sl);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_long(InpSig.SetPointPitch_HH_sl);//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter++;
				break;

				case 3:
					uart_write_long(InpSig.RollAngle_HH_sl);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_long(InpSig.PitchAngle_HH_sl);	//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#

//#Dataset_START#
//Datasetname:#GeneralTempDebug#
			/* Dataset 12 is requested, transmit one Block each 3ms Task		 */
			case 12:

			switch (sendID_Counter)
			{
				case 0:
					uart_write_int(0);				//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_int(0);				//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#-100# GraphMAX:#50#
					uart_write_int(0);				//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#-100# GraphMAX:#50#
					uart_write_int(0);				//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#-10# GraphMAX:#20#

					sendID_Counter++;
				break;

				case 1:
					uart_write_long(0);				//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#
					uart_write_long(0);				//Unit:#-# Offset:#0# //Factor:#1#Divisor:#1# GraphMIN:#0# GraphMAX:#1000#

					sendID_Counter=0;
					SendData_uc=0;
				break;
			}
			break;
//#Dataset_END#
			default:
			break;
		}
	}
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* EXO_DataTransfer														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Send and transmit all necessary  data with EXO							 */
/* Check if EXO is connected and if Rx-Data is valid, if not try to			 */
/* synchronise SPI connection												 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void EXO_DataTransfer(void)
{
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	/* Message body:														 */
	/* 32 byte reference data												 */
	/* 1  byte ID															 */
	/* 1  byte CRC															 */
	/* -----------------------												 */
	/* 34 byte total  centralised in UNION									 */
	/* Please see "SPI_MAIN_EXO.xls" for more information YYY				 */
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	/* Please document any changes in transfered							 */
	/* messages in "SPI_MAIN_EXO.xls"										 */
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	unsigned char Daten_OK_uc = 0;

	/*************************************************************************/
	/* 								Send Data								 */
	/*************************************************************************/
	if (SPI_EXO_error_uc >= SPI_EXO_MAX_ERROR)
	{
		EXO_Connected_uc = 0;

		/* Call Sync-function												 */
		if(SPI_EXO_Sync_Connection())
		{
			SPI_EXO_error_uc = 0;
		}
	}
	else
	{
		EXO_Connected_uc = 1;
		switch (SPI_EXO_ID_Counter)
		{
			case 1:
				/*************************************/
				/* 			Message ID 01			 */
				/*************************************/
				SPI_EXO_Data.Long_ul[0] = InertSig_Gyro.Omg_si[0];
				SPI_EXO_Data.Long_ul[1] = InertSig_Gyro.Omg_si[1];
				SPI_EXO_Data.Long_ul[2] = InertSig_Gyro.Omg_si[2];
				SPI_EXO_Data.Long_ul[3] = InertSig_Gyro.Phi_sl[0];
				SPI_EXO_Data.Long_ul[4] = InertSig_Gyro.Phi_sl[1];
				SPI_EXO_Data.Long_ul[5] = InertSig_Gyro.Phi_sl[2];
				SPI_EXO_Data.Int_ui[15] = InertSig_Acc.AccResultant_ui;
				// High Speed Message
				SPI_EXO_Data.Int_ui[16] = InertSig_Acc.AccFilt_si[0];
				SPI_EXO_Data.Int_ui[17] = InertSig_Acc.AccFilt_si[1];
				SPI_EXO_Data.Int_ui[18] = InertSig_Acc.AccFilt_si[2];
				/* Transfer Data, if CRC ok, release Data					 */
				if (SPI_EXO_TransferUnion(SPI_EXO_ID_Counter))
				{
					Daten_OK_uc = TRUE;
					SPI_EXO_error_uc = 0;
				}
				else
				{
					SPI_EXO_error_uc++;
					SPI_EXO_All_errors_ui++;
				}

				SPI_EXO_ID_Counter++;

			break;

			case 2:
				/*************************************/
				/* 			Message ID 02			 */
				/*************************************/
				SPI_EXO_Data.Int_ui[0] = Stick_Throttle_ui;
				SPI_EXO_Data.Int_ui[1] = Stick_Roll_si;
				SPI_EXO_Data.Int_ui[2] = Stick_Pitch_si;
				SPI_EXO_Data.Int_ui[3] = Stick_Yaw_si;
				SPI_EXO_Data.Int_ui[4] = Poti1_ui;
				SPI_EXO_Data.Int_ui[5] = Poti2_ui;
				SPI_EXO_Data.Int_ui[6] = Poti3_ui;
				SPI_EXO_Data.Int_ui[7] = Poti4_ui;
				SPI_EXO_Data.Int_ui[8] = Poti5_ui;
				SPI_EXO_Data.Int_ui[9] = Poti6_ui;
				SPI_EXO_Data.Int_ui[10] = Poti7_ui;
				SPI_EXO_Data.Int_ui[11] = Poti8_ui;
				SPI_EXO_Data.Int_ui[12] = UBat_ui;
				SPI_EXO_Data.Int_ui[13] = Current_si;
				SPI_EXO_Data.Int_ui[14] = Accu_Cap_ui;
				SPI_EXO_Data.Int_ui[15] = SQinAir_ui;
				// High Speed Message
				SPI_EXO_Data.Int_ui[16] = InertSig_Acc.AccFilt_si[0];
				SPI_EXO_Data.Int_ui[17] = InertSig_Acc.AccFilt_si[1];
				SPI_EXO_Data.Int_ui[18] = InertSig_Acc.AccFilt_si[2];

				/* Transfer Data, if CRC ok, release Data					 */
				if (SPI_EXO_TransferUnion(SPI_EXO_ID_Counter))
				{
					Daten_OK_uc = TRUE;
					SPI_EXO_error_uc = 0;
				}
				else
				{
					SPI_EXO_error_uc++;
					SPI_EXO_All_errors_ui++;
				}

				SPI_EXO_ID_Counter++;

			break;

			case 3:
				/*************************************/
				/* 			Message ID 03			 */
				/*************************************/
				// Byte 5 - 31 frei!!!
				SPI_EXO_Data.Int_ui[0] = EmergLandTime_ui;
				SPI_EXO_Data.Byte_uc[2] = MotorsOn_uc;
				SPI_EXO_Data.Byte_uc[3] = EmergLand_uc;
				SPI_EXO_Data.Byte_uc[4] = RadioQuali_uc;
				SPI_EXO_Data.Int_ui[3] = MaxAngle_si;
				// High Speed Message
				SPI_EXO_Data.Int_ui[16] = InertSig_Acc.AccFilt_si[0];
				SPI_EXO_Data.Int_ui[17] = InertSig_Acc.AccFilt_si[1];
				SPI_EXO_Data.Int_ui[18] = InertSig_Acc.AccFilt_si[2];
				/* Transfer Data, if CRC ok, release Data					 */
				if (SPI_EXO_TransferUnion(SPI_EXO_ID_Counter))
				{
					Daten_OK_uc = TRUE;
					SPI_EXO_error_uc = 0;
				}
				else
				{
					SPI_EXO_error_uc++;
					SPI_EXO_All_errors_ui++;
				}

				SPI_EXO_ID_Counter = 1;

			break;
		} // END switch (SPI_EXO_ID_Counter)

		/*********************************************************************/
		/* 							  Recieve Data							 */
		/*********************************************************************/

		/* If recieved Data is released, copy to variables					 */
		if (Daten_OK_uc)
		{
			switch (SPI_EXO_Data.Byte_uc[40])
			{
				case 1:
					/*************************************/
					/* 			Message ID 01			 */
					/*************************************/
					GPS_North_si = SPI_EXO_Data.Int_ui[2];
					GPS_East_si = SPI_EXO_Data.Int_ui[3];
					Pos_Dev_Integ_North_sl = SPI_EXO_Data.Long_ul[2];
					Pos_Dev_Integ_East_sl = SPI_EXO_Data.Long_ul[3];
					Pos_Dev_North_sl = SPI_EXO_Data.Long_ul[4];
					Pos_Dev_East_sl = SPI_EXO_Data.Long_ul[5];
					Vel_Integ_North_sl = SPI_EXO_Data.Long_ul[6];
					Vel_Integ_East_sl = SPI_EXO_Data.Long_ul[7];
					// High Speed Message
					AltControllerThrottle_sc = SPI_EXO_Data.Byte_uc[38];
					GPS_Pitch_si = SPI_EXO_Data.Int_ui[16];
					GPS_Roll_si = SPI_EXO_Data.Int_ui[17];
				break;

				case 2:
					/*************************************/
					/* 			Message ID 02			 */
					/*************************************/
					GPS_Home_North_sl = SPI_EXO_Data.Long_ul[0];
					GPS_Home_East_sl  = SPI_EXO_Data.Long_ul[1];
					Tar_Pos_North_sl = SPI_EXO_Data.Long_ul[2];
					Tar_Pos_East_sl = SPI_EXO_Data.Long_ul[3];
					P_North_sl = SPI_EXO_Data.Int_ui[8];
					D_North_sl = SPI_EXO_Data.Int_ui[9];
					I_North_sl = SPI_EXO_Data.Int_ui[10];
					P_East_sl = SPI_EXO_Data.Int_ui[11];
					D_East_sl = SPI_EXO_Data.Int_ui[12];
					I_East_sl = SPI_EXO_Data.Int_ui[13];
					SideSlipAngle_si = SPI_EXO_Data.Int_ui[14];
					Tar_Home_Distance_ui = SPI_EXO_Data.Int_ui[15];
					// High Speed Message
					AltControllerThrottle_sc = SPI_EXO_Data.Byte_uc[38];
					GPS_Pitch_si = SPI_EXO_Data.Int_ui[16];
					GPS_Roll_si = SPI_EXO_Data.Int_ui[17];
				break;

				case 3:
					/*************************************/
					/* 			Message ID 03			 */
					/*************************************/
					GPS_PI_North_si = SPI_EXO_Data.Int_ui[0];
					GPS_PI_East_si = SPI_EXO_Data.Int_ui[1];
					GPS_PI_North_WindComp_si = SPI_EXO_Data.Int_ui[2];
					GPS_PI_East_WindComp_si = SPI_EXO_Data.Int_ui[3];
					GPS_Start_Alti_ui = SPI_EXO_Data.Int_ui[4];
					Angle_Dev_Yaw_si = SPI_EXO_Data.Int_ui[5];
					P_Yaw_si = SPI_EXO_Data.Int_ui[6];
					targetreached_uc = SPI_EXO_Data.Byte_uc[14];
					waypoint_cnt_uc = SPI_EXO_Data.Byte_uc[15];
					nopilotctrl_cnt_uc = SPI_EXO_Data.Byte_uc[16];
					MM3_angle_ui = SPI_EXO_Data.Int_ui[9];
					GPS_Yaw_si = SPI_EXO_Data.Int_ui[10];
					Gps_Mode_uc = SPI_EXO_Data.Byte_uc[17];
					Vel_Integ_North_no_sl = SPI_EXO_Data.Long_ul[6];
					Vel_Integ_East_no_sl = SPI_EXO_Data.Long_ul[7];
					heading2WP_si = SPI_EXO_Data.Int_ui[11];
					// High Speed Message
					AltControllerThrottle_sc = SPI_EXO_Data.Byte_uc[38];
					GPS_Pitch_si = SPI_EXO_Data.Int_ui[16];
					GPS_Roll_si = SPI_EXO_Data.Int_ui[17];
					//LED_BLAU_FLASH;
				break;

				case 4:
					/*************************************/
					/* 			Message ID 04			 */
					/*************************************/
					MM3_AngleError_si = SPI_EXO_Data.Int_ui[2];
					MM3_GyroCompass_ui = SPI_EXO_Data.Int_ui[3];
					Heading2Target_sl = SPI_EXO_Data.Long_ul[0];
					GPS.altitude_sl = SPI_EXO_Data.Long_ul[2];
					GPS.velNorth_sl = SPI_EXO_Data.Long_ul[3];
					GPS.velEast_sl = SPI_EXO_Data.Long_ul[4];
					GPS.heading_sl = SPI_EXO_Data.Long_ul[5];
					GPS.GSpeed_sl = SPI_EXO_Data.Long_ul[6];
					GPS.longi_sl = SPI_EXO_Data.Long_ul[7];
					// High Speed Message
					AltControllerThrottle_sc = SPI_EXO_Data.Byte_uc[38];
					GPS_Pitch_si = SPI_EXO_Data.Int_ui[16];
					GPS_Roll_si = SPI_EXO_Data.Int_ui[17];
				break;

				case 5:
					/*************************************/
					/* 			Message ID 05			 */
					/*************************************/
					GPS.lati_sl = SPI_EXO_Data.Long_ul[0];
					GPS.GPSFix_uc = SPI_EXO_Data.Byte_uc[7];
					GPS.noSV_uc = SPI_EXO_Data.Byte_uc[6];
					Heading_Error_si = SPI_EXO_Data.Int_ui[2];
					Pressure_ui = SPI_EXO_Data.Int_ui[4];
					PressureF_ui = SPI_EXO_Data.Int_ui[5];
					DAC_Value_ui = SPI_EXO_Data.Int_ui[6];
					AltControllerControlP_si = SPI_EXO_Data.Int_ui[7];
					AltControllerControlD_si = SPI_EXO_Data.Int_ui[8];
					AltControllerDeviation_si = SPI_EXO_Data.Int_ui[9];
					AltControllerActive_uc = SPI_EXO_Data.Byte_uc[20];
					PressureFF_ui = SPI_EXO_Data.Int_ui[11];
					AltControllerThrottleF_sc = SPI_EXO_Data.Byte_uc[24];
					AltControllerControlAcc_si = SPI_EXO_Data.Int_ui[14];
					TargetDistance_ui = SPI_EXO_Data.Int_ui[15];
					HomeDistance_ui = SPI_EXO_Data.Int_ui[13];
					Buzzer_uc = SPI_EXO_Data.Byte_uc[25];
					// High Speed Message
					AltControllerThrottle_sc = SPI_EXO_Data.Byte_uc[38];
					GPS_Pitch_si = SPI_EXO_Data.Int_ui[16];
					GPS_Roll_si = SPI_EXO_Data.Int_ui[17];
				break;
			}
		} // END if (Daten_OK_uc)
	} // END if (SPI_EXO_error_uc >= SPI_EXO_MAX_ERROR)
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Buzzer																	 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Generation of Buzzer signals				(amount,type)			         */
/* 																			 */
/* 																			 */
/* Invoke from 96ms Task		 :	Buzzer(0,0,1);							 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Return: void																 */
/* Input:	unsigned char newBuzzercase_uc									 */
/*			unsigned char Buzzerstate_uc									 */
/*			Typen: ContinousOn ContinousOff Short Long						 */
/*****************************************************************************/
void Buzzer(unsigned char BuzzerAmount_uc,unsigned char BuzzerType_uc)
{
	static unsigned char BuzzerPower_uc = 0;
	if(BuzzerStateInt_uc == Standby || BuzzerStateInt_uc == Continous)
	{
		if(BuzzerType_uc == ContinousOn)
		{
			BuzzerStateInt_uc = Continous;
			BuzzerPower_uc = 1;
		}

		else if(BuzzerType_uc == ContinousOff)
		{
			BuzzerStateInt_uc = Standby;
			BuzzerPower_uc = 0;
		}
	}
	if(BuzzerStateInt_uc == Standby)
	{
		if(BuzzerType_uc == Short)
		{
			BuzzerStateInt_uc = Short;
			BuzzerAmountInt_uc = BuzzerAmount_uc;
		}


		else if(BuzzerType_uc == Long)
		{
			BuzzerStateInt_uc = Long;
			BuzzerAmountInt_uc = BuzzerAmount_uc;
		}

	}


	/*****************************************************************************/

	switch(BuzzerStateInt_uc)
	{
		case Standby:

			BuzzerPower_uc = 0;

		break;

		case Continous:

			BuzzerPower_uc = 1;

		break;

		case Short:

			if(BuzzerAmountInt_uc > 0)
			{
				if(BuzzerShortState_uc == Off)
				{
					OnTimer_uc++;
					BuzzerPower_uc = 1;
					if(OnTimer_uc >= TimeShort)
					{
						OnTimer_uc = 0;
						BuzzerAmountInt_uc--;
						BuzzerShortState_uc = On;
					}
							}
				else if(BuzzerShortState_uc == On)
				{
					OffTimer_uc++;
					BuzzerPower_uc = 0;
					if(OffTimer_uc >= TimeShort)
					{
						OffTimer_uc = 0;
						BuzzerShortState_uc = Off;
					}
				}
			}
			else if(BuzzerAmountInt_uc == 0)
			{
				BuzzerPower_uc = 0;
				BuzzerStateInt_uc = Standby;
			}

		break;

		case Long:

			if(BuzzerAmountInt_uc > 0)
			{
				if(BuzzerLongState_uc == Off)
				{
					OnTimer_uc++;
					BuzzerPower_uc = 1;
					if(OnTimer_uc >= TimeLong)
					{
						OnTimer_uc = 0;
						BuzzerAmountInt_uc--;
						BuzzerLongState_uc = On;
					}
							}
				else if(BuzzerLongState_uc == On)
				{
					OffTimer_uc++;
					BuzzerPower_uc = 0;
					if(OffTimer_uc >= TimeLong)
					{
						OffTimer_uc = 0;
						BuzzerLongState_uc = Off;
					}
				}
			}
			else if(BuzzerAmountInt_uc == 0)
			{
				BuzzerPower_uc = 0;
				BuzzerStateInt_uc = Standby;
			}

		break;

		default:

			BuzzerPower_uc = 0;
			BuzzerStateInt_uc = Standby;

		break;
		}

	/*****************************************************************************/

	/*Switching Buzzer, connected to POWER1 (BUZZER pin on Mainboard V3.8)*/
	if (BuzzerPower_uc > 0)
	{
		#if (FS_MAINBOARD_TYPE == FS_NEW_MAINBOARD)
		BUZZER_ON;
		#endif

		#if (FS_MAINBOARD_TYPE == FS_OLD_MAINBOARD)
		POWER1_ON;
		#endif
	}
	else
	{
		#if (FS_MAINBOARD_TYPE == FS_NEW_MAINBOARD)
		BUZZER_OFF;
		#endif

		#if (FS_MAINBOARD_TYPE == FS_OLD_MAINBOARD)
		POWER1_OFF;
		#endif

	}
}
