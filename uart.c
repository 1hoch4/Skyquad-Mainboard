/*****************************************************************************/
/* Description: serial communication										 */
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
#include <avr/wdt.h>

#include "config.h"
#include "uart.h"
#include "communication.h"
#include "eeprom.h"
#include "AttCtrl.h"


/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           makros and #defines                             */
/*****************************************************************************/


/*****************************************************************************/
/*                             global variables                             */
/*****************************************************************************/
unsigned char Receive_Char_uc = 0;
unsigned char SendData_uc = 0;
unsigned char EXOTransfer_uc = 0;
unsigned char EXOResponse_uc = 0;
unsigned char EXOTransferState_uc = 1;
unsigned char RxCounter_Tar_uc = 0;
unsigned char SizeReceived_uc = 0;

	//has to be initialized with 1 for dummy.

/*****************************************************************************/
/*                             local variables                               */
/*****************************************************************************/
volatile unsigned char RxData_uc = 0;
unsigned char TxQueue_uc[256];
volatile unsigned char TxQueueIxIn_uc = 0;
volatile unsigned char TxQueueIxOut_uc = 0;
volatile unsigned char TxInProgress_Flg_uc = 0;


/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void uart_init(void);
inline void uart_write_char(unsigned char Chr_uc);
void uart_write_str(pchar_uc Str);
void uart_write_int(signed int Data);
void uart_write_long(signed long Data);


/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/



/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* uart_init																 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void uart_init(void)
{
	//set baudrate register MSB
	UBRR0H = 0x00;
	//set baudrate register LSB (115200 Baud 18,432MHz)
	UBRR0L = 0x09;
	UCSR0A = 0x00;

	// TX + RX on, RX interrupt
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	//8-Bit character length
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* uart_write_char															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
inline void uart_write_char(unsigned char Chr_uc)
{
    TxQueue_uc[TxQueueIxIn_uc] = Chr_uc;

	TxQueueIxIn_uc++;

	if (!TxInProgress_Flg_uc)
	{
		//enable UDRE interrupt
	    UCSR0B |= (1<<UDRIE0);
		TxInProgress_Flg_uc = 1;
	}
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* uart_write_str															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void uart_write_str(pchar_uc Str)
{
  while (*Str)
  {
    uart_write_char(*Str++);
  }
  uart_write_char(CR);
  uart_write_char(LF);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* uart_write_int															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void uart_write_int(signed int Data)
{
  uart_write_char(Data);
  uart_write_char(Data>>8);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* uart_write_long															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
void uart_write_long(signed long Data)
{
  uart_write_char(Data);
  uart_write_char(Data>>8);
  uart_write_char(Data>>16);
  uart_write_char(Data>>24);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* interrupt USART0_RX_vect													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
SIGNAL(USART0_RX_vect)
{
	Receive_Char_uc = UDR0;	
	
	static unsigned char RxCounter_Cur_uc = 1;
	//index and value for checksum calculation.

	unsigned char Index_uc = 0,CHK_Sum = 0;

	if (RxData_uc == 0)
	{
		//measurement output
		if (Receive_Char_uc < 50)
		{
			//measurement input
			SendData_uc = 1;
		}

		//save data exchange
		else if (Receive_Char_uc >= 50 && Receive_Char_uc < 130)

		{
			//standard parameter receive
			if (Receive_Char_uc >= 50 && Receive_Char_uc < 60)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 5 more bytes
				RxCounter_Tar_uc = 6;
				//collect received bytes
				RxData_uc = 1;
			}

			//standard parameter send
			else if (Receive_Char_uc>=60 && Receive_Char_uc < 70)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 3 more bytes
				RxCounter_Tar_uc = 4;
				//collect received bytes
				RxData_uc = 2;
			}

			//write variant
			else if (Receive_Char_uc==70)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 2 more bytes
				RxCounter_Tar_uc = 3;
				//collect received bytes
				RxData_uc = 3;
			}

			//read variant
			else if (Receive_Char_uc==71)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 1 more bytes
				RxCounter_Tar_uc = 2;
				//collect received bytes
				RxData_uc = 4;
			}

			//save variant in EEPROM
			else if (Receive_Char_uc==72)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 2 more bytes
				RxCounter_Tar_uc = 3;
				//collect received bytes
				RxData_uc = 5;
			}

			//send SW and EEPROM version
			else if (Receive_Char_uc==73)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 1 more bytes
				RxCounter_Tar_uc = 2;
				//collect received bytes
				RxData_uc = 6;
			}
			
			//receive MotorGain values
			if (Receive_Char_uc == 74)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 6 more bytes
				RxCounter_Tar_uc = 7;
				//collect received bytes
				RxData_uc = 7;
			}

			//send MotorGain values
			else if (Receive_Char_uc == 75)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 2 more bytes
				RxCounter_Tar_uc = 3;
				//collect received bytes
				RxData_uc = 8;
			}

			//recieve Motor Amount
			else if (Receive_Char_uc == 76)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 2 more bytes
				RxCounter_Tar_uc = 3;
				//collect received bytes
				RxData_uc = 9;
			}

			//send Motor Amount
			else if (Receive_Char_uc == 77)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 1 more bytes
				RxCounter_Tar_uc = 2;
				//collect received bytes
				RxData_uc = 10;
			}
			
			// Recieve Radio Mapping Values
			else if (Receive_Char_uc == 78)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 13 more bytes
				RxCounter_Tar_uc = 14;
				//collect received bytes
				RxData_uc = 11;
			}

			// Send Radio Mapping Values
			else if (Receive_Char_uc == 79)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 1 more bytes
				RxCounter_Tar_uc = 2;
				//collect received bytes
				RxData_uc = 12;
			}
			
			// Send RadioChannelRaw
			else if (Receive_Char_uc == 90)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 1 more bytes
				RxCounter_Tar_uc = 2;
				//collect received bytes
				RxData_uc = 20;
			}
			// Tunnel EXODATA
			else if (Receive_Char_uc == 99)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=99;
				RxData_uc = 99;
				SizeReceived_uc = FALSE;
			}

 		}

		// Firmware and EEPROM functions
		else if (Receive_Char_uc >= 240)
		{
			//EEPROM init (0xFF)
			if (Receive_Char_uc == 240)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 3 more bytes
				RxCounter_Tar_uc = 4;
				//collect received bytes
				RxData_uc = 240;
			}

			// Reset devive for Firmware-Update
			else if (Receive_Char_uc == 250)
			{
				//write id
				UART_RX_TX_Data.Byte_uc[0]=Receive_Char_uc;
				//dummy-value for the union
				UART_RX_TX_Data.Byte_uc[1]=0;
				//receive 3 more bytes
				RxCounter_Tar_uc = 4;
				//collect received bytes
				RxData_uc = 250;
			}
		}
	}

	else if (RxData_uc > 0)
	{
		if(RxData_uc==99 && SizeReceived_uc == FALSE) // Get Size
		{
			RxCounter_Tar_uc = (Receive_Char_uc+1); //Nutzdaten + CRC+Dummy
			UART_RX_TX_Data.Byte_uc[RxCounter_Cur_uc]=Receive_Char_uc;
			SizeReceived_uc = TRUE;
		}
		else
		{
		//increment Counter
		RxCounter_Cur_uc++;
		UART_RX_TX_Data.Byte_uc[RxCounter_Cur_uc]=Receive_Char_uc;
		}
		//all Data received ?
		if(RxCounter_Tar_uc == RxCounter_Cur_uc)
		{
			//standard parameter receive
			if(RxData_uc==1)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=5; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				// is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[6])
				{
					//valid Variant?
					if((UART_RX_TX_Data.Byte_uc[0]-50)>=0
					&& (UART_RX_TX_Data.Byte_uc[0]-50)<P_MaxVariant)
						{
							Var[(UART_RX_TX_Data.Byte_uc[0]-50)].
								ParaID[UART_RX_TX_Data.Int_ui[1]] = 
									UART_RX_TX_Data.Int_ui[2];
						}
				}
			}


			/********************************************************************************************
			Frame Format SPI 40 byte Nutzdaten + 1 byte id + 1 byte crc
			Byte		1		2		3-31		32
			Value		ID		Size	Nutzdaten	CRC
			*//////////////////////////////////////////////////////////////////////////////////////////////

			//receive data for EXO
			if(RxData_uc==99)
			{
			
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=(RxCounter_Tar_uc-1); Index_uc++) // Nutzdaten + ID + Size + CRC
				{
					CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}

				EXOData.Byte_uc[0]=UART_RX_TX_Data.Byte_uc[2]; // Ohne ID und Size
				EXOData.Byte_uc[1]=0;

				for (Index_uc=0; Index_uc<=(RxCounter_Tar_uc); Index_uc++) // Nutzdaten umkopieren
				{
					EXOData.Byte_uc[Index_uc+2]=UART_RX_TX_Data.Byte_uc[Index_uc+3]; // Ohne ID und Size
				}
				// is checksum correct
				
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[RxCounter_Tar_uc])
				{
					EXOTransferState_uc = StartTransfer;
				}
			}

			//send standard parameter
			else if(RxData_uc==2)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=3; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				// is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[4])
				{
					//valid variant?
					if((UART_RX_TX_Data.Byte_uc[0]-60)>=0 && 
						(UART_RX_TX_Data.Byte_uc[0]-60)<P_MaxVariant)
						{
							//reset
							CHK_Sum=0;
							//Reset packet id
							uart_write_char(UART_RX_TX_Data.Byte_uc[0]);
							//reset ParaID
							uart_write_int(UART_RX_TX_Data.Int_ui[1]);
							//send parameter
							uart_write_int(Var[(UART_RX_TX_Data.
								Byte_uc[0]-60)].ParaID[UART_RX_TX_Data.Int_ui[1]]);

							CHK_Sum =  UART_RX_TX_Data.Byte_uc[0];
							CHK_Sum += UART_RX_TX_Data.Byte_uc[2];
							CHK_Sum += UART_RX_TX_Data.Byte_uc[3];
							CHK_Sum += Var[(UART_RX_TX_Data.Byte_uc[0]-60)].
								ParaID[UART_RX_TX_Data.Int_ui[1]];
							
							CHK_Sum += ((Var[(UART_RX_TX_Data.Byte_uc[0]-60)].
								ParaID[UART_RX_TX_Data.Int_ui[1]])>>8);
							
							uart_write_char(CHK_Sum);
						}
				}
			}

			//receive variant
			else if(RxData_uc==3)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=2; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				//is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[3])
				{
					//write variant
					Variant_uc=UART_RX_TX_Data.Byte_uc[2];
					Eeprom_CalSystemSetting.VarianteEEPROM_uc=UART_RX_TX_Data.
						Byte_uc[2];
				}
			}

			//send variant
			else if(RxData_uc==4)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=1; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				//is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[2])
				{
							//reset
							CHK_Sum=0;
							//reset PacketID
							uart_write_char(UART_RX_TX_Data.Byte_uc[0]);
							//send parameter
							uart_write_char(Eeprom_CalSystemSetting.VarianteEEPROM_uc);
							//calculate checksum
							CHK_Sum =  UART_RX_TX_Data.Byte_uc[0];
							CHK_Sum += Eeprom_CalSystemSetting.VarianteEEPROM_uc;
							//send checksum
							uart_write_char(CHK_Sum);

				}
			}
			//write variant
			else if(RxData_uc==5)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=2; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				// is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[3])
				{
					CHK_Sum=0;

					// Valid Variant, SQ on the ground
					if((UART_RX_TX_Data.Byte_uc[2])<P_MaxVariant)
					{
						if (SQinAir_ui==0)
						{
							// ACK
							uart_write_char(255);

							Eeprom_WriteToVariantSetting(UART_RX_TX_Data.
								Byte_uc[2]);

							Eeprom_WriteToSystemSetting();
							Buzzer(1,Short);
						}

						else if (SQinAir_ui>0)
						{
							// NACK
							uart_write_char(1);
						}
					}
				}
			}

			// send SW and EEPROM version
			else if(RxData_uc==6)
			{
				// reset packet ID
				uart_write_char(UART_RX_TX_Data.Byte_uc[0]);
				uart_write_char(SW_Version.Major_uc);
				uart_write_char(SW_Version.Minor_uc);
				uart_write_char(EEPROM_Version.Major_uc);
				uart_write_char(EEPROM_Version.Minor_uc);

				//calculate checksum of sent data
				CHK_Sum = UART_RX_TX_Data.Byte_uc[0];
				CHK_Sum += SW_Version.Major_uc;
				CHK_Sum += SW_Version.Minor_uc;
				CHK_Sum += EEPROM_Version.Major_uc;
				CHK_Sum += EEPROM_Version.Minor_uc,

				uart_write_char(CHK_Sum);
			}

			//receive MotorGain values
			else if(RxData_uc==7)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=6; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}

				// is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[7])
				{
				   
					if(SQinAir_ui == 0 && MotorsOn_uc == 0)
					{
						// save recieved value to RAM
						MotorGain.FMT_sc[UART_RX_TX_Data.Byte_uc[2]]
						 = (signed char)UART_RX_TX_Data.Byte_uc[3];

						MotorGain.FMP_sc[UART_RX_TX_Data.Byte_uc[2]]
						 = (signed char)UART_RX_TX_Data.Byte_uc[4];

						MotorGain.FMR_sc[UART_RX_TX_Data.Byte_uc[2]]
						 = (signed char)UART_RX_TX_Data.Byte_uc[5];

						MotorGain.FMY_sc[UART_RX_TX_Data.Byte_uc[2]]
						 = (signed char)UART_RX_TX_Data.Byte_uc[6];
					
						// ACK
						uart_write_char(255);
					
					}
					else
					{
						// NACK
						uart_write_char(1);
					}
				}
			}

			//send MotorGain values
			else if(RxData_uc==8)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=2; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				// is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[3])
				{
					//send back requested MotorGain Value
					uart_write_char(UART_RX_TX_Data.Byte_uc[0]);
					uart_write_char(UART_RX_TX_Data.Byte_uc[2]);
					uart_write_char(MotorGain.FMT_sc[UART_RX_TX_Data.Byte_uc[2]]);
					uart_write_char(MotorGain.FMP_sc[UART_RX_TX_Data.Byte_uc[2]]);
					uart_write_char(MotorGain.FMR_sc[UART_RX_TX_Data.Byte_uc[2]]);
					uart_write_char(MotorGain.FMY_sc[UART_RX_TX_Data.Byte_uc[2]]);

					//calculate checksum of sent data
					CHK_Sum = UART_RX_TX_Data.Byte_uc[0];
					CHK_Sum += UART_RX_TX_Data.Byte_uc[2];
					CHK_Sum += 
					 (unsigned char)MotorGain.FMT_sc[UART_RX_TX_Data.Byte_uc[2]];
					CHK_Sum +=
					 (unsigned char)MotorGain.FMP_sc[UART_RX_TX_Data.Byte_uc[2]];
					CHK_Sum +=
					 (unsigned char)MotorGain.FMR_sc[UART_RX_TX_Data.Byte_uc[2]];
					CHK_Sum +=
					 (unsigned char)MotorGain.FMY_sc[UART_RX_TX_Data.Byte_uc[2]];

					uart_write_char(CHK_Sum);
				}
			}

			//recieve Motor Amount
			else if(RxData_uc==9)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=2; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				// is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[3])
				{
					if (SQinAir_ui == 0 && MotorsOn_uc == 0)
					{
						// save recieved value to RAM
						MotorGain.Amount_uc = UART_RX_TX_Data.Byte_uc[2];
						
						// ACK
						uart_write_char(255);

						//save MotorGain values to EEPROM
						Eeprom_WriteToMotorMixerSection();
						Buzzer(1,Short);
					}

					else if (SQinAir_ui!=0)
					{
						// NACK
						uart_write_char(1);
					}
				}
			}

			//send Motor Amount
			else if(RxData_uc==10)
			{
				// is checksum correct
				if(UART_RX_TX_Data.Byte_uc[0] == UART_RX_TX_Data.Byte_uc[2])
				{
					uart_write_char(UART_RX_TX_Data.Byte_uc[0]);
					uart_write_char(MotorGain.Amount_uc);

					//calculate checksum of sent data
					CHK_Sum = UART_RX_TX_Data.Byte_uc[0];
					CHK_Sum += MotorGain.Amount_uc;

					uart_write_char(CHK_Sum);
				}
			}
			
			// Recieve Radio Mapping Values
			else if (RxData_uc==11)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=13; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				// is checksum correct
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[14])
				{
					if (SQinAir_ui == 0 && MotorsOn_uc == 0)
					{
						// save recieved value to RAM
						RC_Map.Throttle_uc = UART_RX_TX_Data.Byte_uc[2];
						RC_Map.Pitch_uc = UART_RX_TX_Data.Byte_uc[3];
						RC_Map.Roll_uc = UART_RX_TX_Data.Byte_uc[4];
						RC_Map.Yaw_uc = UART_RX_TX_Data.Byte_uc[5];
						RC_Map.UserFunc1_uc = UART_RX_TX_Data.Byte_uc[6];
						RC_Map.UserFunc2_uc = UART_RX_TX_Data.Byte_uc[7];
						RC_Map.UserFunc3_uc = UART_RX_TX_Data.Byte_uc[8];
						RC_Map.UserFunc4_uc = UART_RX_TX_Data.Byte_uc[9];
						RC_Map.UserFunc5_uc = UART_RX_TX_Data.Byte_uc[10];
						RC_Map.UserFunc6_uc = UART_RX_TX_Data.Byte_uc[11];
						RC_Map.UserFunc7_uc = UART_RX_TX_Data.Byte_uc[12];
						RC_Map.UserFunc8_uc = UART_RX_TX_Data.Byte_uc[13];

						// ACK
						uart_write_char(255);
						
						// save RadioMapping values to EEPROM
						Eeprom_WriteToRadioMappingSection();
						Buzzer(1,Short);
					}
					else 
					{
						// NACK
						uart_write_char(1);
					}

				}
			}

			// Send Radio Mapping Values
			else if (RxData_uc==12)
			{
				// is checksum correct
				if(UART_RX_TX_Data.Byte_uc[0] == UART_RX_TX_Data.Byte_uc[2])
				{
					// prepare data
					UART_RX_TX_Data.Byte_uc[1] = RC_Map.Throttle_uc;
					UART_RX_TX_Data.Byte_uc[2] = RC_Map.Pitch_uc;
					UART_RX_TX_Data.Byte_uc[3] = RC_Map.Roll_uc;
					UART_RX_TX_Data.Byte_uc[4] = RC_Map.Yaw_uc;
					UART_RX_TX_Data.Byte_uc[5] = RC_Map.UserFunc1_uc;
					UART_RX_TX_Data.Byte_uc[6] = RC_Map.UserFunc2_uc;
					UART_RX_TX_Data.Byte_uc[7] = RC_Map.UserFunc3_uc;
					UART_RX_TX_Data.Byte_uc[8] = RC_Map.UserFunc4_uc;
					UART_RX_TX_Data.Byte_uc[9] = RC_Map.UserFunc5_uc;
					UART_RX_TX_Data.Byte_uc[10] = RC_Map.UserFunc6_uc;
					UART_RX_TX_Data.Byte_uc[11] = RC_Map.UserFunc7_uc;
					UART_RX_TX_Data.Byte_uc[12] = RC_Map.UserFunc8_uc;
				}
				// calculate checksum
				for (Index_uc=0; Index_uc<=12; Index_uc++)
				{
					CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				UART_RX_TX_Data.Byte_uc[13] = CHK_Sum;

				for (Index_uc=0; Index_uc<=14; Index_uc++)
				{
					uart_write_char(UART_RX_TX_Data.Byte_uc[Index_uc]);
				}

			}			

			// send RadioChannelRaw
			else if(RxData_uc==20)
			{
				if (SQinAir_ui == 0	&& MotorsOn_uc == 0)
				{
					// is checksum correct
					if(UART_RX_TX_Data.Byte_uc[0] == UART_RX_TX_Data.Byte_uc[2])
					{
						for (Index_uc=1; Index_uc<=12; Index_uc++)
						{
							UART_RX_TX_Data.Int_ui[Index_uc] = 
									RadioChannelRaw_si[Index_uc];
						}
						for (Index_uc=0; Index_uc<=25; Index_uc++)
						{
							CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
							uart_write_char(UART_RX_TX_Data.Byte_uc[Index_uc]);
						}
						
						uart_write_char(CHK_Sum);
					}
				}
				else
				{
					uart_write_char(1); // NACK
				}
			}

			//EEPROM init (0xFF)
			else if(RxData_uc==240)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=3; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				
				// is checksum correct
				// and Byte[2] == 0xAA
				// and Byte[3] == 0xAA
				// and SQ is on Ground
				// and Motors are off
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[4]
					&& UART_RX_TX_Data.Byte_uc[2] == 0xAA
					&& UART_RX_TX_Data.Byte_uc[3] == 0xAA
				    && SQinAir_ui == 0
					&& MotorsOn_uc == 0)
					
				{
					unsigned int EEAddress_ui;

					unsigned int ThresLED_GR_ui = 0x199;
					unsigned int ThresLED_YE_ui = 0x332;
					unsigned int ThresLED_RE_ui = 0x4CB;
					unsigned int ThresLED_BL_ui = 0x664;

					LED_GREEN_OFF;
					LED_YELLOW_OFF;
					LED_RED_OFF;
					LED_BLUE_OFF;

					for(EEAddress_ui = 0;
						EEAddress_ui<=EE_LAST_ADDR;
						EEAddress_ui++)
					{
						/* Wait for completion of previous write			 */
						while(EECR & (1<<EEPE))
						/* Set up address and Data Registers				 */
						EEAR = EEAddress_ui;
						EEDR = 0xFF;
						/* Write logical one to EEMPE						 */
						EECR |= (1<<EEMPE);
						/* Start eeprom write by setting EEPE				 */
						EECR |= (1<<EEPE);
						
						/* giving user feedback								 */
						if (EEAddress_ui >= ThresLED_GR_ui)
								LED_GREEN_ON;
						if (EEAddress_ui >= ThresLED_YE_ui)
								LED_YELLOW_ON;
						if (EEAddress_ui >= ThresLED_RE_ui)
								LED_RED_ON;
						if (EEAddress_ui >= ThresLED_BL_ui)
								LED_BLUE_ON;
					}
					
					LED_GREEN_OFF;
					LED_YELLOW_OFF;
					LED_RED_OFF;
					LED_BLUE_OFF;

					/* Activate Watchdog to reset device					 */
					wdt_enable(WDTO_500MS);
					while(1);
				}
			}
			
			// Reset devive for Firmware-Update
			else if(RxData_uc==250)
			{
				//calculate checksum of the received data
				for (Index_uc=0; Index_uc<=3; Index_uc++)
				{
						CHK_Sum += UART_RX_TX_Data.Byte_uc[Index_uc];
				}
				// is checksum correct
				// and Byte[2] == 0xAA
				// and Byte[3] == 0xAA
				if(CHK_Sum == UART_RX_TX_Data.Byte_uc[4]
					&& UART_RX_TX_Data.Byte_uc[2] == 0xAA
					&& UART_RX_TX_Data.Byte_uc[3] == 0xAA)
					{
						// SQ is on Ground
						// and Motors are off
						// and DIP1 on
						if(SQinAir_ui == 0 && MotorsOn_uc == 0 && DIP1_ON)
						{
							/* Activate Watchdog to reset device					 */
							wdt_enable(WDTO_500MS);
							while(1);
						}
						else
						{
							uart_write_char(1); // NACK
						}
					}
			}
			RxData_uc=0;
			RxCounter_Cur_uc=1;
		}
	}
	return;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* interrupt USART0_TX_vect													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/*																			 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/*																			 */
/*****************************************************************************/
SIGNAL(USART0_UDRE_vect)
{
    if (TxQueueIxOut_uc != TxQueueIxIn_uc)
	{
        UDR0 = TxQueue_uc[TxQueueIxOut_uc];

	    TxQueueIxOut_uc++;
    }
	else
	{
	    TxInProgress_Flg_uc = 0;

		 // Disable UDRE interrupt
		UCSR0B &= ~(1<<UDRIE0);
    }
}
