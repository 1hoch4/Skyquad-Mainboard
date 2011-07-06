/*****************************************************************************/
/* Description: i2c functions												 */
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
#include <inttypes.h>
#include "config.h"
#include "AttCtrl.h"
#include "i2c.h"


/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           makros and #defines                             */
/*****************************************************************************/


/*****************************************************************************/
/*                             global variables                             */
/*****************************************************************************/
volatile uint8_t motor = 0;
volatile uint8_t motor_rx[8];
volatile unsigned int i2c_error_ui = 0;
volatile unsigned int i2c_counter_ui = 0;
volatile unsigned int i2c_motor_counter_ui=0;
unsigned int while_wait_counter_ui = 0;
unsigned char exit_while_uc = 1;


/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/


/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_reset(void);
void i2c_write(unsigned char byte);
void i2c_motor_send(void);

/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* i2c_init																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* initialize TWI clock: 400 kHz clock										 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void i2c_init(void)
{
  	// no prescaler
  	TWSR = 0x00;
 	//~400kHz at 18432000Hz Cpu clock
	TWBR =	15;
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* i2c_start																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void i2c_start(void)
{
    TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT) | (1<<TWIE);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* i2c_stop																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void i2c_stop(void)
{
    TWCR = (1<<TWEN) | (1<<TWSTO) | (1<<TWINT);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* i2c_reset																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void i2c_reset(void)
{
	 i2c_stop();
	 TWCR = 0x80;
	 TWAMR = 0;
	 TWAR = 0;
	 TWDR = 0;
	 TWSR = 0;
	 TWBR = 0;
	 i2c_init();
	 i2c_start();
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* i2c_write																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void i2c_write(unsigned char byte)
{
    TWSR = 0x00;
    TWDR = byte;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* TWI_vect																	 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
SIGNAL(TWI_vect)
{
	/* state machine for sending motorvalues via i2c */
		switch(i2c_counter_ui)
			{

				case 0:
							i2c_write(0x52 + (i2c_motor_counter_ui<<1));
							i2c_counter_ui++;
							break;

				case 1:		
							i2c_write(OutpSig.Motor_si[i2c_motor_counter_ui]);
							i2c_counter_ui++;
							break;

				case 2:		
							i2c_stop();
							i2c_counter_ui=0;
							i2c_motor_counter_ui++;
							i2c_start();
							break;

				default:
							break;

			}
	/* All motor values sent? Then stop i2c communication */
	if(i2c_motor_counter_ui>=MotorGain.Amount_uc)
	{
		i2c_counter_ui=0;
		i2c_motor_counter_ui=0;
		i2c_stop();
	}
	/* Error counter */
	i2c_error_ui=100;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* i2c_motor_send   														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void i2c_motor_send(void)
{
	/* start the state machine in i2c interrupt routine */
	i2c_start();

	/* If error counter <= 20 then reset i2c */
	if(i2c_error_ui<=20)
	{
		i2c_reset();
	}
		else
	{
		i2c_error_ui--;
	}
}
