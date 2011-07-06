/*****************************************************************************/
/* Description: communication and calculation of the inertial sensors		 */
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



#ifndef __INERT_SIG_H
#define __INERT_SIG_H

/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/
typedef struct
{
    int16_t RateRaw_si[3];	  /* raw value 16 Bit (175 LSB/°/s)              */
    int8_t  Rate2kRaw_sc[3];  /* rawvalue 8 Bit, fast signal (4.3°/s/LSB)    */
    int16_t Omg_si[3];		  /* Offset-/Gain-corr. values                   */
    int32_t OmgComp_sl[3];	  /* compensated yawrates --> "Achskopplung"     */     
    int32_t Phi_sl[3];		  /* angles								         */
}
InertSigType_GyroData;


typedef struct
{
    int16_t  AccRaw_si[3];	  /* raw value 16 Bit (6667 LSB/g)               */   
	int16_t  Acc_si[3];		  /* OOffset-/Gain-corr. values                  */
    int16_t  AccFilt_si[3];	  /* filtered values						     */

    int16_t  Rbuf_si[3][8];	  /* filter values   						     */	
    int32_t  RbufSum_sl[3];	  /* filter values   						     */
    uint8_t  RbufIx_uc;		  /* filter values   						     */

	uint16_t AccResultant_ui; /* actual common acc						     */

    int16_t  PhiX_si;		  /* accY roll angle				             */
    int16_t  PhiY_si;		  /* accX pitch angle				             */
}
InertSigType_AccData;

/*****************************************************************************/
/*                           macros and #defines                             */
/****************************************************************************/
#define IS__PHI_X_16			FPL_ROUND_OFF_8_(InertSig_Gyro.Phi_sl[0])
#define IS__PHI_Y_16			FPL_ROUND_OFF_8_(InertSig_Gyro.Phi_sl[1])
#define IS__PHI_Z_16			FPL_ROUND_OFF_8_(InertSig_Gyro.Phi_sl[2])
#define GyroAccCal	  1
#define GyroCal		  2


/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
extern InertSigType_GyroData InertSig_Gyro;
extern InertSigType_AccData  InertSig_Acc;
extern uint16_t SensOffsCount_ui;
extern uint8_t SQ_is_static_uc;
extern int16_t MaxAngle_si;

/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/
void InertSig_CalibOffsets(void);
void InertSig_GetAndProcessSignals(void);
inline void InertSig_PollAndPreprocess(void);

#endif
