/*****************************************************************************/
/* Description: ADC init and communication									 */
/*****************************************************************************/

/*
-------------------------------------------------------------------------------
Copyright (c) 1hoch4UG (haftungsbeschr�nkt)
www.1hoch4.de
-------------------------------------------------------------------------------
GERMAN:
-------------------------------------------------------------------------------
Alle Rechte an dem gesamten Projekt und allen damit verbundenen Dateien und 
Informationen, verbleiben bei der 1hoch4 UG. Dies gilt insbesondere f�r die in
Form des Quellcodes ver�ffentlichten Softwareteile. 

Nutzung der Hardware:
Eine kommerzielle Anwendung (z.B. Luftbildfotografie) der Hardware steht dem 
Nutzer frei. Die 1hoch4 UG schlie�t jedoch jegliche Haftung f�r Sch�den durch
eine kommerzielle Nutzung aus, da es sich um ein experimentelles Hobbyprojekt
im Betastatus handelt, dessen Hard- und Software sich in einer stetigen
Weiterentwicklung befindet und deshalb nicht explizit f�r einen professionellen
Einsatz freigegeben werden kann. Der nicht private Verkauf, die Weiter-
verarbeitung (z.B. Best�ckung) oder die Zusammenstellung der angebotenen
Baus�tze und/oder Platinen zu einem fertigen Produkt bedarf der Abstimmung mit
der 1hoch4 UG.


Nutzung der Software(quellen):
Grunds�tzlich darf die Software nur auf den von der 1hoch4 UG zur Verf�gung
gestellten Hardware eingesetzt werden. Jegliche Art der Nutzung des
ver�ffentlichten Sourcecodes, auch auszugsweise, ist nur f�r den privaten und 
nichtkommerziellen Gebrauch zul�ssig. Jegliche kommerzielle Nutzung oder
Portierung auf andere Hardware bedarf der schriftlichen Zustimmung der
1hoch4 UG. Eine private Verwendung (auch auszugsweise) des Quellcodes,
unabh�ngig davon ob ver�ndert oder unver�ndert, hat zur Folge, dass die
Software weiterhin den hier beschriebenen Bedingungen/Lizenz unterliegt und
diese den verwendeten Softwareteilen beigef�gt werden m�ssen. Weiterhin ist die
1hoch4 UG eindeutig als Quelle anzugegeben. Eine Ver�nderung und Verwendung der
Softwarequellen geschied auf eigene Gefahr. 

Die 1hoch4 UG �bernimmt keinerlei Haftung f�r direkte oder indirekte
Personen-/Sachsch�den. Bedingt durch den experimentellen Status der
1hoch4-Projekte wird keine Gew�hr auf Fehlerfreiheit, Vollst�ndigkeit oder
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

#include <avr/io.h>

#include "config.h"
#include "analog.h"
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
unsigned int UBat_ui;
unsigned int UBatF_ui;
signed int IBat_si;
signed int IBatF_si;
signed int Current_si;
unsigned int Accu_Cap_ui;
unsigned char Cells_uc = 0;


/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/
unsigned long Accu_Cap_tmp_ul;


/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void ADC_Init(void);
unsigned int switch_adc(unsigned char adc_input);
void read_adc(void);


/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* ADC_Init																	 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* ADC initialisation														 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void ADC_Init(void)
{ 
  //Division Factor 128, Interrupt off, Free running
  ADCSRA = (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
  ADCSRB = 0x00;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* switch_adc																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Switch to the according AD input and result								 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
unsigned int switch_adc(unsigned char adc_input) // 0 und 7
{
	ADMUX = adc_input;
	// Delay needed for the stabilization of the ADC input voltage
	// Start the AD conversion

	ADCSRA |= 0x40;
	// Wait for the AD conversion to complete

	while ((ADCSRA & 0x10) == 0);
	ADCSRA |= 0x10;
	return ADC;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* read_adc																	 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Read ADC, calculate UBat_ui, IBat_si										 */
/* Battery voltage check with Buzzer										 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* 																			 */
/*****************************************************************************/
void read_adc(void)
{
	UBat_ui = (unsigned int)(switch_adc(0)*70)/36;
	IBat_si = (signed int)(switch_adc(7));

	// Filter of voltage value
	#define VOLTAGE_FILTER 3
	UBatF_ui=(UBatF_ui*(VOLTAGE_FILTER-1)+UBat_ui)/VOLTAGE_FILTER;

	// Filter of current value
	#define CURRENT_FILTER 8
	IBatF_si=(signed int)(IBatF_si*(CURRENT_FILTER-1)+IBat_si)/CURRENT_FILTER;
	

	/* Filter current signal												 */
	if(IBatF_si >= 125)
	{
	/* Calculation of IBatF_ui in mA (interpolation)						 */
		Current_si = (signed int)(IBatF_si * 81) - 9475;
	}
	else
	{
		//if current is under noise level of sensor, current is set to zero
		Current_si = 0; 
	}


	/* Integration of mA, but only if current positive!						 */
	if(Current_si > 0)
	{
		//corresponds to 1/37500 = 1/(10,416667 * 3600)
		Accu_Cap_tmp_ul += Current_si / 75;
		/* Normalisation of capacity 										 */
		Accu_Cap_ui = Accu_Cap_tmp_ul / 500;
	}

	/* Activation of Buzzer due to undervoltage								 */
	if(UBat_ui < (Cells_uc * Var[Variant_uc].ParaName.P_ACCU_EMPTY_VOLTAGE_ui))  
	{
		Buzzer(1,Long);
	}
}
