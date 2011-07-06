/*****************************************************************************/
/* Description: Communication with external devices							 */
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



#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/
#define Standby 1
#define ContinousOn 2
#define ContinousOff 3
#define Short 4
#define Long 5
#define TimeLong 20
#define TimeShort 10
#define Continous 9
#define Off 0
#define On 1

/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
extern volatile SPI_Union SPI_EXO_Data;
extern volatile UART_RX_TX_DataUnion UART_RX_TX_Data;
extern unsigned char EXO_Connected_uc;
extern unsigned int SPI_EXO_All_errors_ui;
extern unsigned char OffTimer_uc,OnTimer_uc;

/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
extern void sendData(void);
extern void EXO_DataTransfer(void);
extern void Buzzer(unsigned char newBuzzercase_uc,
			unsigned char Buzzerstate_uc);

#endif
