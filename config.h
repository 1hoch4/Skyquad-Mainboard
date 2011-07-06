/*****************************************************************************/
/*Description: definition of several global variables, parameters and defines*/
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



#ifndef __CONFIG_H
#define __CONFIG_H

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "spi.h"
#include "radio.h"

//*****************************************************************************
// mainboard SW version
//*****************************************************************************
#define SW_MAJOR	5
#define SW_MINOR	9

//*****************************************************************************
// mainboard EEPROM version
//*****************************************************************************
#define EE_MAJOR	1
#define EE_MINOR	5

//*****************************************************************************
// global defines
//*****************************************************************************
#define P_MaxVariant  3
#define P_MaxMotorAmount 8


/* ---------------------- general definitions -------------------------------*/

typedef uint8_t FctRetType;

/* --------------------------------------------------------------------------*/

typedef unsigned char *pchar_uc;

typedef union
{
	volatile unsigned long 	Long_ul[10];
	volatile unsigned int 	Int_ui[20];
	volatile unsigned char 	Byte_uc[42];
}SPI_Union;

typedef struct
{
	signed long 	northing_sl;	// in cm (+ = north)
	signed long		easting_sl;		// in cm (+ = east)
	signed long		altitude_sl;	// in cm
	signed long 	velNorth_sl;
	signed long		velEast_sl;
	signed long		velDown_sl;
	signed long		GSpeed_sl; 		//(151007Kr)
	signed long		longi_sl; 		//east-west direction (310708Kr)
	signed long		lati_sl; 		//north-south direction (310708Kr)
	unsigned char	newData_uc;		//status of data: 0 = invlid; 1 = valid
	unsigned char	GPSFix_uc;
	unsigned char 	noSV_uc;		//number of sats
	signed long		heading_sl;		//in degree
} GPS_Struct;

typedef union
{
	volatile unsigned int 	Int_ui[16];
	volatile unsigned char 	Byte_uc[32];
}UART_RX_TX_DataUnion;

typedef struct
{
	uint8_t Major_uc;
	uint8_t Minor_uc;
} SW_VersionType;


//*****************************************************************************
// parameter
//*****************************************************************************
//#PARA_START#
typedef struct
{
/*---------------------------------------------------------------------------*/
unsigned int P_STICK_HEADINGHOLD_THRESHOLD_ui;
/*DESCRIPTION:
#DEUTSCH:
Gibt den Nich-/Roll-Stickwert an, bei dem im HH-Modus die neue Solllage
eingelernt wird.
ENGLISH:
Stick pitch-/roll- threshold to lern new target angle in HH-modus.#
*/
//UNIT:			#-#
//DEFAULT:		#3#
//MIN:			#0#
//MAX:			#5#
/*---------------------------------------------------------------------------*/
unsigned int P_FACTOR_STICK_ROPI_CONTROLLER_ui;
/*DESCRIPTION:
#DEUTSCH:
Faktor zur Beeinflussung der Reglersollwerte für Roll/Nick über die
Fernsteuerung.
ENGLISH:
factor for modification of the controller target roll/pitch angles
via radio control.#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#20#
/*---------------------------------------------------------------------------*/
unsigned int P_FACTOR_STICK_ROPI_DIRECT_ui;
/*DESCRIPTION:
#DEUTSCH:
Faktor zur direkten Beeinflussung der Motordrehzahlen über die Fernsteuerung
mittels Roll/Nick.
ENGLISH:
factor for direct influence of the motor rpm via radio control roll/pitch.#
*/
//UNIT:			#-#
//DEFAULT:		#112#
//MIN:			#0#
//MAX:			#200#
/*---------------------------------------------------------------------------*/
unsigned int P_FACTOR_STICK_ROPI_HEADINGHOLD_ui;
/*DESCRIPTION:
#DEUTSCH:
Faktor zur Vorgabe der Solldrehrate im HeadingHold Modus über die Fernsteuerung
mittels Roll/Nick. Wird nur im HeadingHold Modus verwendet.
ENGLISH:
factor to set target rate in HeadingHold modus via radio control roll/pitch.
Only used during HeadingHold modus.#
*/
//UNIT:			#-#
//DEFAULT:		#77#
//MIN:			#20#
//MAX:			#200#
/*---------------------------------------------------------------------------*/
unsigned int P_FACTOR_STICK_YAW_DIRECT_ui;
/*DESCRIPTION:
#DEUTSCH:
Faktor zur direkten Beeinflussung der Motordrehzahlen über die Fernsteuerung
mittels Gier.
ENGLISH:
factor for direct indluence of the motor rpm via radio control yaw.#
*/
//UNIT:			#-#
//DEFAULT:	  	#90#
//MIN:			#25#
//MAX:			#125#
/*---------------------------------------------------------------------------*/
unsigned int P_STICK_YAW_THRESHOLD_ui;
/*DESCRIPTION:
#DEUTSCH:
Gibt den Gier-Stickwert an, bei dem der neue Sollwinkel gelernt wird.
ENGLISH:
Stick Yaw Threshold to lern new target angle.#
*/
//UNIT:			#-#
//DEFAULT:		#6#
//MIN:			#5#
//MAX:			#15#
/*---------------------------------------------------------------------------*/
unsigned int P_LOOP_PITCH_ON_THRESHOLD_ui;
/*DESCRIPTION:
#DEUTSCH:
Stickwert Nick, oberhalb dessen die P-Regelung ausgeschaltet wird.
ENGLISH:
Stick Pitch threshold, above the P-part of the controller is disabled.#
*/
//UNIT:			#-#
//DEFAULT:		#90#
//MIN:			#70#
//MAX:			#100#
/*---------------------------------------------------------------------------*/
unsigned int P_LOOP_PITCH_OFF_HYST_ui;
/*DESCRIPTION:
#DEUTSCH:
Stickwert Nick, unterhalb dessen die P-Regelung wieder eingeschaltet wird.
ENGLISH:
Stick Pitch value, below the P-part of the controller is enabled again.#
*/
//UNIT:			#-#
//DEFAULT:		#30#
//MIN:			#10#
//MAX:			#40#
/*---------------------------------------------------------------------------*/
unsigned int P_LOOP_ROLL_ON_THRESHOLD_ui;
/*DESCRIPTION:	#DEUTSCH:
Stickwert Roll, oberhalb dessen die P-Regelung ausgeschaltet wird.
ENGLISH:
Stick Roll threshold, above the P-part of the controller is disabled.#
*/
//UNIT:			#-#
//DEFAULT:		#90#
//MIN:			#70#
//MAX:			#100#
/*---------------------------------------------------------------------------*/
unsigned int P_LOOP_ROLL_OFF_HYST_ui;
/*DESCRIPTION:
#DEUTSCH:
Stickwert Roll, unterhalb dessen die P-Regelung wieder eingeschaltet wird.
ENGLISH:
Stick Roll value, below the P-part of the controller is enabled again.#
*/
//UNIT:			#-#
//DEFAULT:		#30#
//MIN:			#10#
//MAX:			#40#
/*---------------------------------------------------------------------------*/
unsigned int P_ACCU_EMPTY_MOTOR_REDUCTION_ui;
/*DESCRIPTION:
#DEUTSCH:
Akkuspannung ab der die Motoren auf P_EMER_THRO_ACCU_EMPTY_uc +
P_MIN_THRO_CONT_ACT_uc reduziert werden.
ENGLISH:
accu voltage to reduce the motor rmp to P_EMER_THRO_ACCU_EMPTY_uc +
P_MIN_THRO_CONT_ACT_uc.#
*/
//UNIT:			#V/100#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#100#
/*---------------------------------------------------------------------------*/
unsigned int P_EMER_THRO_ui;
/*DESCRIPTION:
#DEUTSCH:
!!!WIRD NICHT MEHR VERWENDET!!!
ENGLISH:
!!!NOT USED ANY MORE!!!#
*/
//UNIT:			#-#
//DEFAULT:		#90#
//MIN:			#50#
//MAX:			#175#
/*---------------------------------------------------------------------------*/
unsigned int P_EMER_THRO_ACCU_EMPTY_ui;
/*DESCRIPTION:
#DEUTSCH:
Notgas wenn Akku absolut leer ist.
ENGLISH:
emergency throttle in case of completely empty accu.#
*/
//UNIT:			#-#
//DEFAULT:		#66#
//MIN:			#10#
//MAX:			#120#
/*---------------------------------------------------------------------------*/
unsigned int P_EMER_THRO_DURATION_ui;
/*DESCRIPTION:
#DEUTSCH:
Notgasdauer für die das Notgas gehalten wird.
ENGLISH:
duration for emergency throttle.#
*/
//UNIT:			#6ms#
//DEFAULT:		#2500#
//MIN:			#500#
//MAX:			#50000#
/*---------------------------------------------------------------------------*/
unsigned int P_YAW_LIMIT_ui;
/*DESCRIPTION:
#DEUTSCH:
!!!WIRD NICHT MEHR VERWENDET!!!
ENGLISH:
!!!NOT USED ANY MORE!!!#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#50#
/*---------------------------------------------------------------------------*/
unsigned int P_MIN_THRO_ui;
/*DESCRIPTION:
#DEUTSCH:
Minimaler Gaswert der bei laufenden Motoren gesendet wird.
ENGLISH:
minimal throttle value which is send to the motors while running.#
*/
//UNIT:			#-#
//DEFAULT:		#15#
//MIN:			#1#
//MAX:			#50#
/*---------------------------------------------------------------------------*/
unsigned int P_MAX_THRO_ui;
/*DESCRIPTION:
#DEUTSCH:
Maximaler Gaswert der bei laufenden Motoren gesendet wird.
ENGLISH:
maximal throttle value which is send to the motors while running.#
*/
//UNIT:			#-#
//DEFAULT:		#255#
//MIN:			#200#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_MAX_THRO_STICK_ui;
/*DESCRIPTION:
#DEUTSCH:
Maximaler Gaswert auf den der Stick Gaswert begrenzt wird.
ENGLISH:
maximal throttle value to bound the stick throttle value.#
*/
//UNIT:			#-#
//DEFAULT:		#240#
//MIN:			#200#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_MIN_THRO_CONTROLLER_ACTIVE_ui;
/*DESCRIPTION:
#DEUTSCH:
Minimaler Gaswert bei dem die Regelung aktiviert wird.
ENGLISH:
minimal throttle value at which the controller is activated.#
*/
//UNIT:			#-#
//DEFAULT:		#8#
//MIN:			#5#
//MAX:			#50#
/*---------------------------------------------------------------------------*/
unsigned int P_ACCU_EMPTY_VOLTAGE_ui;
/*DESCRIPTION:
#DEUTSCH:
Akkuspannung pro Zelle bei der der Akku als leer betrachtet wird.
ENGLISH:
accu valtage per cell at which the accu is recognised as empty.#
*/
//UNIT:			#V/100#
//DEFAULT:	  	#320#
//MIN:			#300#
//MAX:			#400#
/*---------------------------------------------------------------------------*/
unsigned int P_CURRENT_FILTER_ui;
/*DESCRIPTION:
#DEUTSCH:
Filtertiefe des Stromsensors.
ENGLISH:
filter coefficient of current sensor.#
*/
//UNIT:			#-#
//DEFAULT:		#8#
//MIN:			#2#
//MAX:			#20#
/*---------------------------------------------------------------------------*/
unsigned int P_VOLTAGE_FILTER_ui;
/*DESCRIPTION:
#DEUTSCH:
Filtertiefe der Spannungsmessung.
ENGLISH:
filter coefficient of voltage measurement.#
*/
//UNIT:			#-#
//DEFAULT:		#3#
//MIN:			#2#
//MAX:			#20#
/*---------------------------------------------------------------------------*/
unsigned int P_KP_ROLL_PITCH_ui;
/*DESCRIPTION:
#DEUTSCH:
P-Verstärkung Roll / Nick des Lagereglers.
ENGLISH:
P-Gain Roll / Pitch of attitude controller.#
*/
//UNIT:			#-#
//DEFAULT:		#175#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_KD_ROLL_PITCH_ui;
/*DESCRIPTION:
#DEUTSCH:
D-Verstärkung Roll / Nick des Lagereglers.
ENGLISH:
D-Gain Roll / Pitch of attitude controller.#
*/
//UNIT:			#-#
//DEFAULT:	  	#100#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_KP_YAW_ui;
/*DESCRIPTION:
#DEUTSCH:
P-Verstärkung Gier des Lagereglers.
ENGLISH:
P-Gain Yaw of attitude controller.#
*/
//UNIT:			#-#
//DEFAULT:		#200#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_KD_YAW_ui;
/*DESCRIPTION:
#DEUTSCH:
D-Verstärkung Gier des Lagereglers.
ENGLISH:
D-Gain Yaw of attitude controller.#
*/
//UNIT:			#-#
//DEFAULT:		#138#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_RADIO_FILTER_ui;
/*DESCRIPTION:
#DEUTSCH:
Filtertiefe für Funke Eingangssignale.
ENGLISH:
filter coefficient of radio signals.#
*/
//UNIT:			#-#
//DEFAULT:		#3#
//MIN:			#2#
//MAX:			#10#
/*---------------------------------------------------------------------------*/
unsigned int P_CAMERACOMP_ON_OFF_ui;
/*DESCRIPTION:
#DEUTSCH:
Schalter zum globalen Ein- und Ausschalten der Roll und/oder Nickkompensation.
Kanal kann auf einen Potikanal gelegt werden. Wert <128 -> off, >128 -> on.
ENGLISH:
Switch to turn on/off roll and/or pitchcompensation. Channel can be used with
 a Poti. Value <128 -> off, >128 -> on.#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_CAMERAPITCHCOMP_ON_OFF_ui;
/*DESCRIPTION:
#DEUTSCH:
Schalter zum globalen Ein- und Ausschalten der Nickkompensation.
Kanal kann auf einen Potikanal gelegt werden. Wert <128 -> off, >128 -> on.
ENGLISH:
Switch to turn on/off roll pitchcompensation. Channel can be used with
 a Poti. Value <128 -> off, >128 -> on.#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_CAMERAROLLCOMP_ON_OFF_ui;
/*DESCRIPTION:
#DEUTSCH:
Schalter zum globalen Ein- und Ausschalten der Rollkompensation.
Kanal kann auf einen Potikanal gelegt werden. Wert <128 -> off, >128 -> on.
ENGLISH:
Switch to turn on/off rollcompensation. Channel can be used with
 a Poti. Value <128 -> off, >128 -> on.#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_CAMERAPITCHCOMP_GAIN_ui;
/*DESCRIPTION:
#DEUTSCH:
Verstärkung errechnet sich aus Parameter-127= 1/Verstärkung
bsp. Parameter=187 --> 187-127=60 --> 1/60 Winkelschritt
bsp. Parameter= 77 --> 77-127= -60 --> -1/60 Winkelschritt Drehrichtung geändert
--> Je größer der Winkelschritt desto größer die Verstärkung
ENGLISH:
Factor for ratio between SQ pitchangle -> position of Servo.
calc: (Parameter-127)=1/Gain
e.g. Parameter=187 --> 187-127=60 --> 1/60 anglestep cw
e.g. Parameter= 77 --> 77-127= -60 --> -1/60 anglestep ccw #
*/
//UNIT:			#-#
//DEFAULT:		#187#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_CAMERAPITCHCOMP_OFFSET_ui;
/*DESCRIPTION:
#DEUTSCH:
Offset für den Pitchkompensationsservo  bei Winkel=0°. Kann mit Poti belegt
werden
ENGLISH:
Offset for pitchservo at angle=0°. Can be used with a poti #
*/
//UNIT:			#-#
//DEFAULT:		#127#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_CAMERAROLLCOMP_GAIN_ui;
/*DESCRIPTION:
#DEUTSCH:
Übersetzungsfakor und Vorzeichen SQ Rollwinkel -> Stellweg vom Servo
Verstärkung errechnet sich aus Parameter-127= 1/Verstärkung
bsp. Parameter=187 --> 187-127=60 --> 1/60 Winkelschritt
bsp. Parameter= 77 --> 77-127= -60 --> -1/60 Winkelschritt Drehrichtung geändert
--> Je größer der Winkelschritt desto größer die Verstärkung
ENGLISH:
Factor for ratio between SQ rollangle -> position of Servo.
calc: (Parameter-127)=1/Gain
e.g. Parameter=187 --> 187-127=60 --> 1/60 anglestep cw
e.g. Parameter= 77 --> 77-127= -60 --> -1/60 anglestep ccw#
*/
//UNIT:			#-#
//DEFAULT:		#187#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_CAMERAROLLCOMP_OFFSET_ui;
/*DESCRIPTION:
#DEUTSCH:
Offset für den Rollkompensationsservo  bei Winkel=0°. Kann mit Poti belegt
werden
ENGLISH:
Offset for pitchservo at angle=0°. Can be used with a poti #
*/
//UNIT:			#-#
//DEFAULT:		#127#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO1_VALUE_ui;
/*DESCRIPTION:
#DEUTSCH:
Wert für den Servoausgang1. Alternativ auf ein Poti legen
ENGLISH:
Value for Servoconnector1. Also for use with Poti.#
*/
//UNIT:			#-#
//DEFAULT:		#127#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO1_GAIN_ui;
/*DESCRIPTION:
#DEUTSCH:
Verstärkungsfaktor für Servoausgang1. Berechnung: Gain = (127-Parameter)
Bsp. Parameter=54 --> Gain=73, Bsp2. Parameter=200 --> Gain=-73 negDrehrichtung
je näher der Wert bei 0 oder 255 desto größer die Verstärkung.
ENGLISH:
Gainfaktor for Servoconnector 1. Calc: Gain = (127-Parameter)
e.g. Parameter=54 --> Gain=73, e.g2. Parameter=200 --> Gain=-73 negDirection
if parameter is close to 0 or 255 then gain it high.#
*/
//UNIT:			#-#
//DEFAULT:		#54#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO1_OFFSET_ui;
/*DESCRIPTION:
#DEUTSCH:
Offset für Serovausgang 1
ENGLISH:
Offset for Servo 2#
*/
//UNIT:			#-#
//DEFAULT:		#127#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO1_MAX_ui;
/*DESCRIPTION:
#DEUTSCH:
Maximalausschlag für Servo 1. 692=2,4ms und  576=2,0ms
ENGLISH:
Maximum value for Servo 1 692 = 2,4ms and 576=2,0ms.#
*/
//UNIT:			#-#
//DEFAULT:		#576#
//MIN:			#432#
//MAX:			#692#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO1_MIN_ui;
/*DESCRIPTION:
#DEUTSCH:
Minimalausschlag für Servo 1. 172=0,6ms, 288= 1,0ms
ENGLISH:
Minimum value for Servo 1 172=0,6ms and 288=1,0ms#
*/
//UNIT:			#-#
//DEFAULT:		#288#
//MIN:			#172#
//MAX:			#432#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO2_VALUE_ui;
/*DESCRIPTION:
#DEUTSCH:
Wert für den Servoausgang2. Alternativ auf ein Poti legen
ENGLISH:
Value for Servoconnector2. Also for use with Poti#
*/
//UNIT:			#-#
//DEFAULT:		#127#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO2_GAIN_ui;
/*DESCRIPTION:
#DEUTSCH:
Verstärkungsfaktor für Servoausgang2. Berechnung: Gain = (127-Parameter)
Bsp. Parameter=54 --> Gain=73, Bsp2. Parameter=200 --> Gain=-73 negDrehrichtung
je näher der Wert bei 0 oder 255 desto größer die Verstärkung.
ENGLISH:
Gainfaktor for Servoconnector 2. Calc: Gain = (127-Parameter)
e.g. Parameter=54 --> Gain=73, e.g2. Parameter=200 --> Gain=-73 negDirection
if parameter is close to 0 or 255 then gain it high.#
*/
//UNIT:			#-#
//DEFAULT:		#54#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO2_OFFSET_ui;
/*DESCRIPTION:
#DEUTSCH:
Offset für Serovausgang 2
ENGLISH:
Offset for Servo 2.#
*/
//UNIT:			#-#
//DEFAULT:		#127#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO2_MAX_ui;
/*DESCRIPTION:
#DEUTSCH:
Maximalausschlag für Servo 2. 692=2,4ms und  576=2,0ms
ENGLISH:
Maximum value for Servo 2 692 = 2,4ms and 576=2,0ms.#
*/
//UNIT:			#-#
//DEFAULT:		#576#
//MIN:			#432#
//MAX:			#692#
/*---------------------------------------------------------------------------*/
unsigned int P_SERVO2_MIN_ui;
/*DESCRIPTION:
#DEUTSCH:
Minimalausschlag für Servo 2. 172=0,6ms, 288= 1,0ms
ENGLISH:
Minimum value for Servo 2 172=0,6ms and 288=1,0ms#
*/
//UNIT:			#-#
//DEFAULT:		#288#
//MIN:			#172#
//MAX:			#432#
/*---------------------------------------------------------------------------*/
unsigned int P_REFRESHRATESERVO_ui;
/*DESCRIPTION:
#DEUTSCH:
Update Rate für die Servoausgänge. 288/ms -> 5760/288=20ms=50Hz.3744 -> 76,9Hz
ENGLISH:
UpdateRate for Servoconnections 288/ms -> 5760/288=20ms=50Hz.3744 -> 76,9Hz#
*/
//UNIT:			#-#
//DEFAULT:		#5760#
//MIN:			#3744#
//MAX:			#5760#
/*---------------------------------------------------------------------------*/
unsigned int P_POWER_VALUE1_ui;
/*DESCRIPTION:
#DEUTSCH:
Wert für den Powerausgang1. Alternativ auf ein Poti legen
ENGLISH:
Value for Poweroutput1. Also for use with Poti#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_POWER_VALUE2_ui;
/*DESCRIPTION:
#DEUTSCH:
Wert für den Powerausgang2. Alternativ auf ein Poti legen
ENGLISH:
Value for Poweroutput2. Also for use with Poti#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/
unsigned int P_POWER_VALUE3_ui;
/*DESCRIPTION:
#DEUTSCH:
Wert für den Powerausgang3. Alternativ auf ein Poti legen
ENGLISH:
Value for Poweroutput3. Also for use with Poti#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/

unsigned int P_POWER_THRESHOLD1_ui;
/*DESCRIPTION:
#DEUTSCH:
Schwellwert zum Schalten der Powerausgang 1.
ENGLISH:
Threshold to Switch the Power Output 1.#
*/
//UNIT:			#-#
//DEFAULT:		#128#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/

unsigned int P_POWER_THRESHOLD2_ui;
/*DESCRIPTION:
#DEUTSCH:
Schwellwert zum Schalten der Powerausgang 2.
ENGLISH:
Threshold to Switch the Power Output 2.#
*/
//UNIT:			#-#
//DEFAULT:		#128#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/

unsigned int P_POWER_THRESHOLD3_ui;
/*DESCRIPTION:
#DEUTSCH:
Schwellwert zum Schalten der Powerausgang 3.
ENGLISH:
Threshold to Switch the Power Output 3.#
*/
//UNIT:			#-#
//DEFAULT:		#128#
//MIN:			#0#
//MAX:			#255#
/*---------------------------------------------------------------------------*/

unsigned int FS_HEADING_HOLD_ui;
/*DESCRIPTION:
#DEUTSCH:
Schalter fuer Heading Hold Modus (nick und roll). "=0": normaler ACC-Modus
"=1": HH-Modus
ENGLISH:
Switch for Heading Hold modus (pitch and roll). "=0": normal ACC-modus
"=1": HH-modus#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#1#
/*---------------------------------------------------------------------------*/

unsigned int FS_MAINBOARD_ROTATED_ui;
/*DESCRIPTION:
#DEUTSCH:
Schalter fuer Ausrichtung des Mainboards. "=0": normale Einbauposition.
"=1": um 45° gegen den Uhrzeigersinn gedrehter Einbau.
ENGLISH:
Switch for orientation of Mainboard. "=0": normal orientation.
"=1": orientation rotated by 45°, counterclockwise.#
*/
//UNIT:			#-#
//DEFAULT:		#0#
//MIN:			#0#
//MAX:			#1#
/*---------------------------------------------------------------------------*/
uint32_t CRC;
// CRC for EEPROM storing
/*---------------------------------------------------------------------------*/
}
ParaList;
//#PARA_END#

/*---------------------------------------------------------------------------*/
/* That are parameter which can be ajusted via poti values.					 */
/* if that list should be modified, the function "Poti_2_Param()"			 */
/* in "main.c" must be modified as well										 */
/* Parameter value range must be same as poti range (0 - 255)				 */

//#PARA2POTI_START#
typedef struct
{
	unsigned int P_KP_ROLL_PITCH_ui;
	unsigned int P_KD_ROLL_PITCH_ui;
	unsigned int P_KP_YAW_ui;
	unsigned int P_KD_YAW_ui;
	unsigned int P_CAMERACOMP_ON_OFF_ui;
	unsigned int P_CAMERAPITCHCOMP_ON_OFF_ui;
	unsigned int P_CAMERAROLLCOMP_ON_OFF_ui;
	unsigned int P_CAMERAPITCHCOMP_OFFSET_ui;
	unsigned int P_CAMERAROLLCOMP_OFFSET_ui;
	unsigned int P_SERVO1_VALUE_ui;
	unsigned int P_SERVO2_VALUE_ui;
	unsigned int P_POWER_VALUE1_ui;
	unsigned int P_POWER_VALUE2_ui;
	unsigned int P_POWER_VALUE3_ui;
}PotiParaType;
//#PARA2POTI_END#
/*---------------------------------------------------------------------------*/


typedef union
{
    ParaList    ParaName;
    uint16_t    ParaID[sizeof(ParaList) / sizeof(uint16_t)];
}
ParaUnion_t;
/*****************************************************************************/
/*                            global variables                               */
/*****************************************************************************/
extern ParaUnion_t Var[P_MaxVariant];

extern SW_VersionType EEPROM_Version;
extern SW_VersionType SW_Version;
extern PotiParaType Poti;

extern volatile unsigned char Variant_uc;
extern volatile unsigned char Counter_1ms_uc;
extern unsigned int Temp_StatusRegister_ui;
extern volatile unsigned char MaxVariant_uc;

extern unsigned char BootDelay_uc;

//Variables for EXO-Board data
extern GPS_Struct GPS;
extern signed long P_North_sl;
extern signed long D_North_sl;
extern signed long I_North_sl;
extern signed long P_East_sl;
extern signed long D_East_sl;
extern signed long I_East_sl;
extern signed int P_Yaw_si;
extern signed long Pos_Dev_North_sl;
extern signed long Pos_Dev_East_sl;
extern signed long Pos_Dev_Integ_North_sl;
extern signed long Pos_Dev_Integ_East_sl;
extern signed long Vel_Integ_North_sl;
extern signed long Vel_Integ_East_sl;
extern signed long Vel_Integ_North_no_sl;
extern signed long Vel_Integ_East_no_sl;
extern signed long Angle_Dev_Yaw_si;
extern signed long GPS_Home_North_sl;
extern signed long GPS_Home_East_sl;
extern unsigned int GPS_Start_Alti_ui;
extern signed long Tar_Pos_North_sl;
extern signed long Tar_Pos_East_sl;
extern signed int GPS_North_si;
extern signed int GPS_East_si;
extern signed int GPS_Pitch_si;
extern signed int GPS_Roll_si;
extern signed int GPS_Yaw_si;
extern unsigned char targetreached_uc;
extern unsigned char waypoint_cnt_uc;
extern unsigned char nopilotctrl_cnt_uc;
extern unsigned char Gps_Mode_uc;
extern volatile unsigned int TargetDistance_ui;
extern unsigned int MM3_angle_ui;
extern signed int heading2WP_si;
extern signed int MM3_AngleError_si;
extern unsigned int	MM3_GyroCompass_ui;
extern signed long Heading2Target_sl;
extern signed int Heading_Error_si;
extern unsigned int HomeDistance_ui;
extern unsigned int Tar_Home_Distance_ui;
extern signed int GPS_PI_North_WindComp_si;
extern signed int GPS_PI_East_WindComp_si;
extern signed int GPS_PI_North_si;
extern signed int GPS_PI_East_si;
extern signed int SideSlipAngle_si;  //for wind compensation


#define Off 0
#define Hold 1
#define Home 2
#define Waypoint 3
//****************************************
//AltController
//****************************************
extern volatile unsigned char AltControllerActive_uc;
extern volatile signed int AltControllerControlP_si;
extern volatile signed int AltControllerControlD_si;
extern volatile signed char AltControllerThrottle_sc;
extern volatile signed char AltControllerThrottleF_sc;
extern volatile signed int AltControllerDeviation_si;
extern volatile signed int AltControllerControlAcc_si;
extern volatile unsigned int DAC_Value_ui;
extern volatile unsigned int Pressure_ui;
extern volatile unsigned int PressureF_ui;
extern volatile unsigned int PressureFF_ui;

//*****************************************************************************
//buzzer and power outputs should be activated by the Exo-board
//*****************************************************************************
extern volatile unsigned char Buzzer_uc; // 0 = OFF; 1 = ON;


//*****************************************************************************
//switches
//*****************************************************************************

// Servo extension
//*--------------------------------------------------------------------------*/
#define FS_SERVOEXT_ON 1
// servo extension installed
#define FS_SERVOEXT_OFF 0
// servor extension not installed
#define FS_SERVOEXT			FS_SERVOEXT_OFF
//to enable the sum signal on the servo ports for the servo extension
//hardware.
//*--------------------------------------------------------------------------*/


//Loop
//*--------------------------------------------------------------------------*/
#define FS_LOOP_OFF 0
// no Loops
#define FS_LOOP_PITCH 1
// Loop (Pitchaxle) is allowed
#define FS_LOOP_ROLL 2
// Roll (Rollaxle) is allowed
#define FS_LOOP_PITCHROLL 3
//Loop (Pitchaxle) and Roll (Rollaxle) is allowed
#define FS_LOOPING				FS_LOOP_OFF
//Loop-function Pitch- and/or Rollaxle
//*--------------------------------------------------------------------------*/


// LIS or BOSCH ACC
//*--------------------------------------------------------------------------*/
#define FS_BOSCHACC 1
// Bosch ACC Sensor
#define FS_LISACC 0
// LIS ACC Sensor
#define FS_ACC_BUILT			FS_LISACC
//*--------------------------------------------------------------------------*/

// Old Style Mainboard?
//*--------------------------------------------------------------------------*/
#define FS_NEW_MAINBOARD 1
// Released Mainboard
#define FS_OLD_MAINBOARD 0
// Prototype Mainboards
#define FS_MAINBOARD_TYPE		FS_NEW_MAINBOARD
//*--------------------------------------------------------------------------*/




// radio.c
//*--------------------------------------------------------------------------*/
#define Stick_Throttle_ui RadioChannel_si[RC_Map.Throttle_uc]
// RadioChannel for Throttle
//*--------------------------------------------------------------------------*/
#define Stick_Roll_si RadioChannel_si[RC_Map.Roll_uc]
// RadioChannel for Roll
//*--------------------------------------------------------------------------*/
#define Stick_Pitch_si RadioChannel_si[RC_Map.Pitch_uc]
// RadioChannel for Pitch
//*--------------------------------------------------------------------------*/
#define Stick_Yaw_si RadioChannel_si[RC_Map.Yaw_uc]
// RadioChannel for Yaw
//*--------------------------------------------------------------------------*/
#define Poti1_ui RadioChannel_si[RC_Map.UserFunc1_uc]
// RadioChannel for Poti 1
//*--------------------------------------------------------------------------*/
#define Poti2_ui RadioChannel_si[RC_Map.UserFunc2_uc]
// RadioChannel for Poti 2
//*--------------------------------------------------------------------------*/
#define Poti3_ui RadioChannel_si[RC_Map.UserFunc3_uc]
// RadioChannel for Poti 3
//*--------------------------------------------------------------------------*/
#define Poti4_ui RadioChannel_si[RC_Map.UserFunc4_uc]
// RadioChannel for Poti 4
//*--------------------------------------------------------------------------*/
#define Poti5_ui RadioChannel_si[RC_Map.UserFunc5_uc]
// RadioChannel for Poti 5
//*--------------------------------------------------------------------------*/
#define Poti6_ui RadioChannel_si[RC_Map.UserFunc6_uc]
// RadioChannel for Poti 6
//*--------------------------------------------------------------------------*/
#define Poti7_ui RadioChannel_si[RC_Map.UserFunc7_uc]
// RadioChannel for Poti 7
//*--------------------------------------------------------------------------*/
#define Poti8_ui RadioChannel_si[RC_Map.UserFunc8_uc]
// RadioChannel for Poti 8
//*--------------------------------------------------------------------------*/



//*****************************************************************************
//*		macros																  *
//*****************************************************************************


// P2P(b,a)
// copy 'Poti' to 'Parameter'
// usage:  P2P(b,a);
#define P2P(b,a)		 { if(a > 65527) \
						   { \
						   	 if(a == 65528) b = Poti1_ui;\
							 else if(a == 65529) b = Poti2_ui;\
							 else if(a == 65530) b = Poti3_ui;\
							 else if(a == 65531) b = Poti4_ui;\
							 else if(a == 65532) b = Poti5_ui;\
							 else if(a == 65533) b = Poti6_ui;\
							 else if(a == 65534) b = Poti7_ui;\
						     else if(a == 65535) b = Poti8_ui;\
						   } \
						   else b = a;\
						 }

// A_MAX_BC(a,b,c)
// 'a' = maximum of 'b' and 'c'
// usage:  A_MAX_BC (a, b, c);
#define A_MAX_BC(a,b,c)  { (a) = (b); \
                           if ( (c) > (b) )\
                               {(a) = (c);}\
                         }

// A_MAX_AB(a,b)
// 'a' = maximum of 'a' and 'b
// usage:  A_MAX_AB (a, b);
#define A_MAX_AB(a,b)    { if ( (a) < (b) )\
                               {(a) = (b);}\
                         }


// A_MIN_BC(a,b,c)
// 'a' = minimum of 'b' and 'c'
// usage:  A_MIN_BC (a, b, c);
#define A_MIN_BC(a,b,c)  { (a) = (b); \
                           if ( (c) < (b) )\
                               {(a) = (c);}\
                         }


// A_MIN_AB(a,b)
// 'a' = minimum of 'a' and 'b'
// usage:  A_MIN_AB (a, b);
#define A_MIN_AB(a,b)    { if ( (a) > (b) )\
                               {(a) = (b);}\
                         }


// BOUND(x,max,min)
// bounds 'x' between 'max' (high border) and 'min' (low border)
// usage:  BOUND (x, max, min);
#define BOUND(x,max,min)  { if( (x) > (max) ) \
                            {(x)=(max);}      \
                            if( (x) < (min) ) \
                            {(x)=(min);}      \
                          }


// BOUND_X_MIN_MAX(x,min,max)
// bounds 'x' between 'min' (low border) and 'max' (high border)
// usage: BOUND_X_MIN_MAX(x,min,max);
#define BOUND_X_MIN_MAX(x,min,max) BOUND((x),(max),(min))

// ABS(a)
// absolute value of 'a'
// usage:  x = ABS (a);
#define ABS(a)        ( ((a) < (0)) ? -(a) : (a) )


// MIN(a,b)
// minimum of 'a' and 'b'
// usage:  x = MIN (a, b);
#define MIN(a, b)  ( ((a) < (b)) ? (a) : (b) )


// MAX(a,b)
// maximum of 'a' and 'b'
// usage:  x = MAX (a, b);
#define MAX(a, b)  ( ((a) > (b)) ? (a) : (b) )

#define TRUE  1
#define FALSE 0

#define RET_OK   0
#define RET_FAIL 1

#define BV(bit)			(1<<(bit))
#define cbi(reg,bit)	reg &= ~(BV(bit))
#define sbi(reg,bit)	reg |= (BV(bit))

// Helper to avoid over- or underflow
// 8 bit
#define SC_MAX	 127
#define SC_MIN	-128
#define UC_MAX	 255

// 16 bit
#define SI_MAX	 32767
#define SI_MIN	-32768
#define UI_MAX	 65535

// 32 bit
#define SL_MAX	 2147483647
#define SL_MIN	-2147483648
#define UL_MAX	 4294967296


//LED und Power Outputs

#define LED_GREEN_ON    PORTC &=~0x40	//GREEN LED 1
#define LED_GREEN_OFF   PORTC |= 0x40
#define LED_GREEN_FLASH	PORTC ^= 0x40

#define LED_YELLOW_ON    PORTC &=~0x80	//YELLOW LED 2
#define LED_YELLOW_OFF   PORTC |= 0x80
#define LED_YELLOW_FLASH PORTC ^= 0x80

#define LED_RED_ON    PORTA &=~0x40		//RED LED 3
#define LED_RED_OFF   PORTA |= 0x40
#define LED_RED_FLASH PORTA ^= 0x40

#define LED_BLUE_ON    PORTA &=~0x20	//BLUE LED 4
#define LED_BLUE_OFF   PORTA |= 0x20
#define LED_BLUE_FLASH PORTA ^= 0x20

#define POWER1_OFF   PORTB &=~0x01
#define POWER1_ON    PORTB |= 0x01
#define POWER1_FLASH PORTB ^= 0x01

#define POWER2_OFF   PORTB &=~0x02
#define POWER2_ON    PORTB |= 0x02
#define POWER2_FLASH PORTB ^= 0x02

#define POWER3_OFF   PORTB &=~0x04
#define POWER3_ON    PORTB |= 0x04
#define POWER3_FLASH PORTB ^= 0x04

#define BUZZER_OFF	 PORTC &=~0x10
#define BUZZER_ON    PORTC |= 0x10

#define DIP1_ON (!(PINA & (1<<PINA4)))
#define DIP2_ON (!(PINA & (1<<PINA3)))
#define DIP3_ON (!(PINA & (1<<PINA2)))
#define DIP4_ON (!(PINA & (1<<PINA1)))

#define BUTTON_ON (!(PINA & (1<<PINA1)))

//SPI

#define SPI_CS_SENS_LOW  PORTD &=~0x20
#define SPI_CS_SENS_HIGH PORTD |= 0x20

#define SPI_CS_EXO_LOW	 PORTD &=~0x80
#define SPI_CS_EXO_HIGH  PORTD |= 0x80

#if (FS_ACC_BUILT == FS_LISACC)

#define SPI_CS_LIS_LOW  PORTC &=~0x20
#define SPI_CS_LIS_HIGH PORTC |= 0x20

#endif //(FS_ACC_BUILT == FS_LISACC)

#endif //__CONFIG_H
