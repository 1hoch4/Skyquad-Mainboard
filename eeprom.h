/*****************************************************************************/
/* Description: EEPROM handling												 */
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



#ifndef __EEPROM_H
#define __EEPROM_H


/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/
typedef enum
{
    BOTH_BLOCKS_OK,
	BLOCK_ONE_CORRUPTED,
	BLOCK_TWO_CORRUPTED,
	BOTH_BLOCKS_CORRUPTED
}
EepromType_RedundantBlockStatus;


/* --------------- data structure for offset data block  ------------------- */

typedef struct
{
    int16_t  OffsGyroX_si;
	int16_t  OffsGyroY_si;
	int16_t  OffsGyroZ_si;
	int16_t  OffsHGyroX_si;
	int16_t  OffsHGyroY_si;
	int16_t  OffsHGyroZ_si;
	int16_t  OffsAccX_si;
    int16_t  OffsAccY_si;
	int16_t  OffsAccZ_si;
	uint8_t  InertSigCalibrated;
    uint32_t CRC;
}
EepromType_CalDataSecOffsCorr;


/* ---------------- data structure for radio data block  ------------------- */

typedef struct
{
	int16_t  Faktor_MinFunke_Kanal_si[13];
    int16_t  Faktor_MaxFunke_Kanal_si[13];
    int16_t  OffsFunke_Kanal_si[13];
	uint8_t  RadioCalibrated_uc;
	uint32_t Crc;
}
EepromType_CalDataSecRemoteCtrl;

/* --------------- data structure for sstem data block  -------------------- */

typedef struct
{
	uint8_t VarianteEEPROM_uc;
	uint8_t EE_Version_Major_uc;
	uint8_t EE_Version_Minor_uc;
	uint32_t CRC;

}	
EepromType_CalSystemSetting;

/* ------------- data structure for valid flag data block  ----------------- */
typedef struct
{
    uint8_t OffsCorrDataValid_flg      : 1,
	        RemoteCtrlDataValid_flg    : 1,
			SystemSettingValid_flg	   : 1,
			VariantSetting_flg	 	   : 1,
			MotorMixerSettingValid_flg : 1,
			RadioMappingValid_flg	   : 1,
			rsvd                       : 2;
}
EepromType_CalDataStatusBits;

typedef union
{
    uint8_t                      Byte_uc;
	EepromType_CalDataStatusBits Bits;
}
EepromType_CalDataStatus;


/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/

/* ----------------- global status --> all data are valid ------------------ */

#define EEPROM_ALL_SECTIONS_OK		0x3F

/* ----------------------- EEPROM adres-definition ------------------------- */

#define EE_OFFSCORR_START_ADDR		0x010

#define EE_REMOTECTRL_START_ADDR	(EE_OFFSCORR_START_ADDR + \
	                                 sizeof(EepromType_CalDataSecOffsCorr))
	                                 
#define EE_RADIOMAPPING_START_ADDR	(EE_REMOTECTRL_START_ADDR + \
									 sizeof(EepromType_CalDataSecRemoteCtrl))

#define EE_SYSTEMSETT_START_ADDR	(EE_RADIOMAPPING_START_ADDR + \
	                                 sizeof(RadioType_RadioMapping))

#define EE_MOTORMIXER_START_ADDR	(EE_SYSTEMSETT_START_ADDR + \
	                                 sizeof(EepromType_CalSystemSetting))
	                                
#define EE_VARIANTSETT_START_ADDR	(EE_MOTORMIXER_START_ADDR + \
	                                 sizeof(AttCtrlType_MotorGain))

#define EE_MIRROR_ADDR_OFFS			0x400

#define EE_REPAIR_CNT_ADDR_BLK1		0x7F0
#define EE_REPAIR_CNT_ADDR_BLK2		(EE_REPAIR_CNT_ADDR_BLK1+2)

#define EE_LAST_ADDR				0x7FF

/* ---------- Makros fuer Kompatibilitaet mit bestehendem Code ------------- */

#define Faktor_MinFunke_Kanal_si  Eeprom_CalDataSecRemoteCtrl.Faktor_MinFunke_Kanal_si
#define Faktor_MaxFunke_Kanal_si  Eeprom_CalDataSecRemoteCtrl.Faktor_MaxFunke_Kanal_si
#define OffsFunke_Kanal_si        Eeprom_CalDataSecRemoteCtrl.OffsFunke_Kanal_si


/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/


extern EepromType_CalDataSecOffsCorr   Eeprom_CalDataSecOffsCorr;

extern EepromType_CalDataSecRemoteCtrl Eeprom_CalDataSecRemoteCtrl;

extern EepromType_CalDataStatus        Eeprom_CalDataStatus;

extern EepromType_CalSystemSetting	   Eeprom_CalSystemSetting;


/*****************************************************************************/
/*                           function prototypes                             */
/*****************************************************************************/

/* ------------ Generische Block-IO-Routinen inkl. CRC-Check --------------- */

FctRetType Eeprom_ReadAndVerifyBlockNormal
(
    uint16_t BlockStartAddr, uint8_t *RamDestPtr, uint16_t BlockSize
);


EepromType_RedundantBlockStatus Eeprom_ReadAndVerifyBlockRedundant
(
    uint16_t BlockStartAddr, uint16_t MirrAddrOffs,
	uint8_t *RamDestPtr, uint16_t BlockSize
);

void Eeprom_WriteBlockNormal
(
    uint16_t BlockStartAddr, uint8_t *RamSrcPtr, uint16_t BlockSize
);

EepromType_RedundantBlockStatus Eeprom_ReadAndVerifyBlockRedundant
(
    uint16_t BlockStartAddr, uint16_t MirrAddrOffs,
	uint8_t *RamDestPtr, uint16_t BlockSize
);

/* ------------------ Applikationsspezifische Routinen --------------------- */

void Eeprom_WriteToOffsCorrSection(void);

void Eeprom_WriteToRemoteCtrlSection(void);

void Eeprom_WriteToRadioMappingSection(void);

void Eeprom_WriteToSystemSetting(void);

void Eeprom_WriteToMotorMixerSection(void);

void Eeprom_WriteToVariantSetting(uint8_t VarianteSetting_uc);

FctRetType Eeprom_ReadAll(void);

/* ------------------------------------------------------------------------- */


#endif
