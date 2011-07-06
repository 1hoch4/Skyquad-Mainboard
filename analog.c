/*****************************************************************************/
/* Description: ADC init and communication									 */
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

	/* Filter of voltage value												 */ 
	UBatF_ui = (UBatF_ui 
			  * ((uint8_t)Var[Variant_uc].ParaName.P_VOLTAGE_FILTER_ui - 1)
			  + UBat_ui)
			  / (uint8_t)Var[Variant_uc].ParaName.P_VOLTAGE_FILTER_ui;

	/* Filter of current value												 */
	IBatF_si = (signed int)(IBatF_si 
			  * ((uint8_t)Var[Variant_uc].ParaName.P_CURRENT_FILTER_ui - 1) 
			  + IBat_si) 
			  / (uint8_t)Var[Variant_uc].ParaName.P_CURRENT_FILTER_ui;
	

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
		Buzzer(3,Short);
	}
}
