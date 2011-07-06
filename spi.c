/*****************************************************************************/
/* Description: Spi functions												 */
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



/*****************************************************************************/
/*                                 includes                                  */
/*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "spi.h"
#include "config.h"
#include "communication.h"

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           makros and #defines                             */
/*****************************************************************************/


/*****************************************************************************/
/*                             global variables                             */
/*****************************************************************************/



/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/

unsigned char SPI_SENS_RX_Data_uc;
unsigned char SPI_EXO_RX_Data_uc;

/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void SPI_Init(void);

#if (FS_ACC_BUILT == FS_LISACC)
	void LISACC_Init(void);
#endif//(FS_ACC_BUILT == FS_LISACC)

void SPI_SENS_SendByte(unsigned char data);
unsigned char SPI_SENS_TransferByte(unsigned char data);
unsigned int SPI_SENS_TransferWord(unsigned int data);
void SPI_EXO_SendByte(unsigned char data);
unsigned char SPI_EXO_TransferByte(unsigned char data);
unsigned int SPI_EXO_TransferWord(unsigned int data);
unsigned char SPI_EXO_TransferUnion(unsigned char SPI_ID_uc);
unsigned char SPI_EXO_Sync_Connection(void);


/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPIInit																 	 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void SPI_Init(void)
{

/*****************************************************************************/
// 	SPI on USART Sensoren
/*****************************************************************************/

	// setup SPI I/O pins
	sbi(PORTD, 4);	// set XCK hi
	sbi(DDRD, 4);	// set XCK as output
	cbi(DDRD, 2);	// set MISO as input
	sbi(DDRD, 3);	// set MOSI as output
	sbi(DDRD, 5);	// CS_Gyro+BoschAcc
	sbi(DDRC, 5);	// LISAcc

	// setup SPI interface (USART1 in MasterSPI mode):
	sbi(UCSR1C, UMSEL11);
	sbi(UCSR1C, UMSEL10);

	// no interrupts
	UCSR1A = 0x00;

	// enable Reciever and Tramsmitter
	UCSR1B = 0x18;

	//clock = f/8 --> 18,432MHz
	//entspricht 18,432/8=2,304MHz


	// set Baudraten Register MSB

	UBRR1H = 0x00;
	// set Baudraten Register LSB (115200 Baud 18,432MHz)

	UBRR1L = 0x08;



	// select clock phase positive-going in middle of data
	sbi(UCSR1C, UCPOL1);
	cbi(UCSR1C, 2);

#if (FS_ACC_BUILT == FS_LISACC)
	// SPI Mode 3 LIS
	sbi(UCSR1C, 1);
#endif

#if (FS_ACC_BUILT == FS_BOSCHACC)
	// SPI Mode 1 Bosch
	cbi(UCSR1C, 1);
#endif


	// clear status
	// inb(SPSR);
	// SPSR --> UDR1
	SPI_SENS_RX_Data_uc = UDR1;
	cbi(UCSR1A, UDRE1);

/*****************************************************************************/
//	SPI EXO Board
/*****************************************************************************/

	sbi(DDRB, 7);	// set XCK as output
	cbi(DDRB, 6);	// set MISO as input
	sbi(DDRB, 5);	// set MOSI as output
	sbi(DDRD, 7);	// SPI_CS_EXO as output
	sbi(PORTB, 7);	// set XCK hi

//	Master Mode
//	Clock f/4
//	MSB first
//	Clock Phase positiv
//	No Interrupt
//	SPI enabled
	SPCR = 0x59;
	SPSR = 0x00;

//	Read
	SPI_EXO_RX_Data_uc = SPDR;
	SPI_CS_EXO_HIGH;
}


#if (FS_ACC_BUILT == FS_LISACC)
/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* LISACC_Init																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void LISACC_Init(void)
{
	unsigned char t3_uc = 0;

	SPI_CS_LIS_LOW;

	SPI_SENS_TransferByte(0x20);
	// Power UP
	t3_uc = SPI_SENS_TransferByte(0xC7);

	SPI_CS_LIS_HIGH;


	SPI_CS_LIS_LOW;

	SPI_SENS_TransferByte(0x21);
	// 16 bit mode CTRLREG2 DAS = 1
	t3_uc = SPI_SENS_TransferByte(0x40);

	SPI_CS_LIS_HIGH;
}
#endif//(FS_ACC_BUILT == FS_LISACC)


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_SENS_SendByte														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void SPI_SENS_SendByte(unsigned char data)
{
	//Transfer Data
	SPDR = data;
	//Wait till transfer is finished
	while(!(UCSR1A & (1<<RXC1)));
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_SENS_TransferByte													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
unsigned char SPI_SENS_TransferByte(unsigned char data)
{
	//Transfer Data
	UDR1 = data;
	//Wait till transfer is finished
	while(!(UCSR1A & (1<<RXC1)));
	//Transfer received data
	SPI_SENS_RX_Data_uc = UDR1;
	return SPI_SENS_RX_Data_uc;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_SENS_TransferWord													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
unsigned int SPI_SENS_TransferWord(unsigned int data)
{
	unsigned int rxData = 0;

	//send MS byte of given data
	rxData = (SPI_SENS_TransferByte((data>>8) & 0x00FF))<<8;
	//send LS byte of given data
	rxData |= (SPI_SENS_TransferByte(data & 0x00FF));

	//return the received data
	return rxData;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_EXO_SendByte															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void SPI_EXO_SendByte(unsigned char data)
{
	//Transfer Data
	SPDR = data;
	//Wait till transfer is finished
	while (!(SPSR & (1<<SPIF)));
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_EXO_TransferByte														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
unsigned char SPI_EXO_TransferByte(unsigned char data)
{
	SPDR = data;
	//Wait till transfer is finished
	while(!(SPSR & (1<<SPIF)));
	//Wait till transfer is finished
	SPI_EXO_RX_Data_uc = SPDR;
	return SPI_EXO_RX_Data_uc;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_EXO_TransferWord														 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
unsigned int SPI_EXO_TransferWord(unsigned int data)
{
	unsigned int rxData = 0;

	//send MS byte of given data
	rxData = (SPI_SENS_TransferByte((data>>8) & 0x00FF))<<8;
	//send LS byte of given data
	rxData |= (SPI_SENS_TransferByte(data & 0x00FF));

	//return the received data
	return rxData;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_EXO_TransferUnion													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* send a union to the exo board,with checksum. 							 */
/* The received data will be checked										 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
unsigned char SPI_EXO_TransferUnion(unsigned char SPI_ID_uc)
{
	unsigned char index_uc = 0;
	unsigned char CHK_Sum = 0;
	unsigned char CHK_OK;
	// write ID
	SPI_EXO_Data.Byte_uc[40] = SPI_ID_uc;

	//calculate Checksum
	//CHK = Summe (Byte[0]:Byte[40])
	for (index_uc=0; index_uc<=40; index_uc++)
	{
		CHK_Sum += SPI_EXO_Data.Byte_uc[index_uc];
	}

	// write checksum
	SPI_EXO_Data.Byte_uc[41] = CHK_Sum;

	// reset checksum
	CHK_Sum = 0;

	// send + receive
	for (index_uc=0; index_uc<=41; index_uc++)
	{
		SPI_CS_EXO_LOW;

		SPI_EXO_Data.Byte_uc[index_uc] =
		SPI_EXO_TransferByte(SPI_EXO_Data.Byte_uc[index_uc]);

		_delay_us(5);
		SPI_CS_EXO_HIGH;
	}

	// calculate checksum of the received data
	for (index_uc=0; index_uc<=40; index_uc++)
	{
		CHK_Sum += SPI_EXO_Data.Byte_uc[index_uc];
	}

	// check checksum
	if (CHK_Sum == SPI_EXO_Data.Byte_uc[41])
	{
		CHK_OK = 1;
	}
	else
	{
		CHK_OK = 0;
	}

	return CHK_OK;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* SPI_EXO_Sync_Connection													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* Syncs the connection between mainboard and EXO							 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* return = 1 sync, return = 0 nosync										 */
/*****************************************************************************/
unsigned char SPI_EXO_Sync_Connection(void)
{
	unsigned char index_uc;

	//generate Init Message
	//Byte 1:...(copy from EXO Board)
	unsigned char INIT_Data_uc[5] = {0, 1, 2, 3, 6};

	//Send Waste byte to shift the transfer to one Byte
	SPI_CS_EXO_LOW;
	SPI_EXO_SendByte((unsigned char)255);
	_delay_us(30);
	SPI_CS_EXO_HIGH;

	//send Init Message
	for (index_uc=0; index_uc<5; index_uc++)
	{
		SPI_CS_EXO_LOW;
		INIT_Data_uc[index_uc] = SPI_EXO_TransferByte(INIT_Data_uc[index_uc]);
		_delay_us(30);
		SPI_CS_EXO_HIGH;

	}

	//check received data
	if (	INIT_Data_uc[0] == 0
		&&	INIT_Data_uc[1] == 1
		&&	INIT_Data_uc[2] == 2
		&&	INIT_Data_uc[3] == 3
		&&	INIT_Data_uc[4] == 6)
	{
		return 1;
	}
	else return 0;
}
