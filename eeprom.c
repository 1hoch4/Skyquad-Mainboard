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


#ifndef EEMEM 
#define EEMEM __attribute__ ((section (".eeprom")))      // EEProm definition 
#endif 


/*****************************************************************************/
/*                                 includes                                  */
/*****************************************************************************/
#include <avr/io.h>
#include <avr/eeprom.h>

#include "config.h"
#include "InertSig.h"
#include "AttCtrl.h"
#include "radio.h"
#include "CRC32.h"
#include "eeprom.h"


/*****************************************************************************/
/*                             type-declarations                             */
/*****************************************************************************/


/*****************************************************************************/
/*                           macros and #defines                             */
/*****************************************************************************/


/*****************************************************************************/
/*                             global variables                              */
/*****************************************************************************/
EepromType_CalDataSecOffsCorr   Eeprom_CalDataSecOffsCorr;
EepromType_CalDataSecRemoteCtrl Eeprom_CalDataSecRemoteCtrl;
EepromType_CalDataStatus        Eeprom_CalDataStatus;
EepromType_CalSystemSetting		Eeprom_CalSystemSetting;


/*****************************************************************************/
/*                             local variablen                               */
/*****************************************************************************/


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



/*****************************************************************************/
/*                             implementation                                */
/*****************************************************************************/


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_ReadAndVerifyBlockNormal											 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* read Datablock from EEPROM incl. CRC32 check. 							 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* uint16_t BlockStartAddr, uint8_t *RamDestPtr, uint16_t BlockSize			 */
/* Return																	 */
/* FctRetType RetStatus														 */
/*****************************************************************************/
FctRetType Eeprom_ReadAndVerifyBlockNormal
(
    uint16_t BlockStartAddr, uint8_t *RamDestPtr, uint16_t BlockSize
)
{
    FctRetType RetStatus;

    /* read block	                                                   ^     */
	eeprom_read_block(RamDestPtr, (uint8_t *)BlockStartAddr, BlockSize);

    /* execute CRC-Check 	                                                 */
	RetStatus = Crc32_VerifyBlock(RamDestPtr, BlockSize);

    /* tip from www.mikrocontroller.net-Forum to set EEAR to unused adress   */
	/* after EEPROM execution										         */
	EEAR = EE_LAST_ADDR;
	
	/* return CRC-Check-Status 		                                         */
    return(RetStatus);	
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_ReadAndVerifyBlockRedundant										 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* read Datablock from EEPROM incl. CRC32 check. 							 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* uint16_t BlockStartAddr, uint16_t MirrAddrOffs,							 */
/* uint8_t *RamDestPtr, uint16_t BlockSize									 */
/* Return:																	 */
/* EepromType_RedundantBlockStatus RetStatus								 */
/*****************************************************************************/
EepromType_RedundantBlockStatus Eeprom_ReadAndVerifyBlockRedundant
(
    uint16_t BlockStartAddr, uint16_t MirrAddrOffs,
	uint8_t *RamDestPtr, uint16_t BlockSize
)
{
	EepromType_RedundantBlockStatus RetStatus = BOTH_BLOCKS_OK;
	uint16_t BlkByteIx, RepairCnt;
	uint8_t *RdBackDataPtr;

    /* read original-Block 	                                                 */
    eeprom_read_block(RamDestPtr, (uint8_t *)BlockStartAddr, BlockSize);

    /* execute CRC-Check 						                             */
	if (Crc32_VerifyBlock(RamDestPtr, BlockSize) == RET_FAIL)
	{
        /* first block defect		                                         */
        RetStatus = BLOCK_ONE_CORRUPTED;
        
		/* read redundant block 			                                 */
		eeprom_read_block
		(
		    RamDestPtr, (uint8_t *)(BlockStartAddr + MirrAddrOffs), 
			BlockSize
        );

        /* check if consistent						                         */
        if (Crc32_VerifyBlock(RamDestPtr, BlockSize) == RET_OK)
        {
            /* redundant block is ok, try to repair the original block		 */
			eeprom_write_block
			(
			    RamDestPtr, (uint8_t *)BlockStartAddr, BlockSize
            );

			/* increment repair-Counter for block 1 			            */
			RepairCnt = 
			    eeprom_read_word((uint16_t *)EE_REPAIR_CNT_ADDR_BLK1);
					
			eeprom_write_word
			(
			    (uint16_t *)EE_REPAIR_CNT_ADDR_BLK1, RepairCnt + 1
            );                    			    
		}
        else
		{
            /* second block also defect! No correction possible			     */
            /* EEPROM-parameter not available	                             */
            RetStatus = BOTH_BLOCKS_CORRUPTED;
        }
	}
	else
	{
		/* First block ok, redundant block also? 							 */
        /* Read both blocks and compare bitwise.							 */
        RdBackDataPtr = (uint8_t *)(BlockStartAddr + MirrAddrOffs);

		for (BlkByteIx = 0; BlkByteIx < BlockSize; BlkByteIx++)
		{
            if (eeprom_read_byte(RdBackDataPtr++) != RamDestPtr[BlkByteIx])
			{
                RetStatus = BLOCK_TWO_CORRUPTED;
				
				break;
			}
		}

        if (RetStatus == BLOCK_TWO_CORRUPTED)
		{
            /* Redundant block not ok, try to repair with the original block.*/
			eeprom_write_block
			(
			    RamDestPtr, (uint8_t *)(BlockStartAddr + MirrAddrOffs),
				BlockSize
            );

			/* .and increment repair counter for block 2		             */
			RepairCnt = 
			    eeprom_read_word((uint16_t *)EE_REPAIR_CNT_ADDR_BLK2);
					
			eeprom_write_word
			(
			    (uint16_t *)EE_REPAIR_CNT_ADDR_BLK2, RepairCnt + 1
            );                    			    
		}
	}

    /* tip from www.mikrocontroller.net-Forum to set EEAR to unused adress   */
	/* after EEPROM execution										         */
	EEAR = EE_LAST_ADDR;

    return(RetStatus);
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteBlockNormal													 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write datablock to the EEPROM always incl. CRC32	at the end				 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* uint16_t BlockStartAddr, uint8_t *RamSrcPtr, uint16_t BlockSize			 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteBlockNormal
(
    uint16_t BlockStartAddr, uint8_t *RamSrcPtr, uint16_t BlockSize
)
{
    uint32_t *CrcPtr;

    /* Word-Pointer to adress, 4 Bytes below block end (= CRC-Adr.) 		 */
    CrcPtr = (uint32_t *)&RamSrcPtr[BlockSize - 4];
	
	/* compute CRC32 via Block-Payload and copy to the end of block  	     */
	*CrcPtr = 
	    Crc32_CalcBlockCrc
	    (
		    CRC32_STD_START_VALUE, RamSrcPtr, BlockSize - 4
        );

    /* write block	                                                         */
    eeprom_write_block
	(
	    RamSrcPtr, (uint8_t *)BlockStartAddr, BlockSize
    );

    /* tip from www.mikrocontroller.net-Forum to set EEAR to unused adress   */
	/* after EEPROM execution										         */
	EEAR = EE_LAST_ADDR;
}

/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteBlockRedundant												 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write datablock redundant to the EEPROM always incl. CRC32	at the end	 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* uint16_t BlockStartAddr, uint16_t MirrAddrOffs,							 */
/* uint8_t *RamSrcPtr, uint16_t BlockSize									 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteBlockRedundant
(
    uint16_t BlockStartAddr, uint16_t MirrAddrOffs,
	uint8_t *RamSrcPtr, uint16_t BlockSize
)
{
    uint32_t *CrcPtr;

    /* Word-Pointer to adress, 4 Bytes below block end (= CRC-Adr.) 		 */
    CrcPtr = (uint32_t *)&RamSrcPtr[BlockSize - 4];
	
	/* compute CRC32 via Block-Payload and copy to the end of block  	     */
	*CrcPtr = 
	    Crc32_CalcBlockCrc
	    (
		    CRC32_STD_START_VALUE, RamSrcPtr, BlockSize - 4
        );

    /* write original blcok		                                             */
    eeprom_write_block
	(
	    RamSrcPtr, (uint8_t *)BlockStartAddr, BlockSize
    );

    /* write redundant block shiftet about "MirrAddrOffs" 	                 */
	eeprom_write_block
	(
	    RamSrcPtr, (uint8_t *)(BlockStartAddr + MirrAddrOffs), BlockSize
    );

    /* tip from www.mikrocontroller.net-Forum to set EEAR to unused adress   */
	/* after EEPROM execution										         */
	EEAR = EE_LAST_ADDR;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteToOffsCorrSection											 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write offset datablock redundant to the EEPROM always incl. CRC32 at the  */
/* end.	After execution valid flag will be set								 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteToOffsCorrSection(void)
{
    /* Offsetkorrekturwerte redundant in's EEPROM schreiben (jeweils mit CRC)*/

    Eeprom_WriteBlockRedundant
	(
	    EE_OFFSCORR_START_ADDR, EE_MIRROR_ADDR_OFFS,
		(uint8_t *)&Eeprom_CalDataSecOffsCorr, 
		sizeof(Eeprom_CalDataSecOffsCorr)
    );

    Eeprom_CalDataStatus.Bits.OffsCorrDataValid_flg = TRUE;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteToRemoteCtrlSection											 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write radio datablock redundant to the EEPROM always incl. CRC32 at the   */
/* end.	After execution radio valid flag will be set.						 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteToRemoteCtrlSection(void)
{
    /* write radio datablock redundant to the EEPROM (incl CRC) 			 */

    Eeprom_WriteBlockRedundant
	(
	    EE_REMOTECTRL_START_ADDR, EE_MIRROR_ADDR_OFFS,
		(uint8_t *)&Eeprom_CalDataSecRemoteCtrl, 
		sizeof(Eeprom_CalDataSecRemoteCtrl)
    );

	Eeprom_CalDataStatus.Bits.RemoteCtrlDataValid_flg = TRUE;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteToRadioMappingSection										 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write radiomapping datablock redundant to the EEPROM always incl.		 */
/* CRC32 at the end. After execution radiomapping valid flag will be set	 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteToRadioMappingSection(void)
{
	/* write system datablock redundant to the EEPROM (incl. CRC) 		     */
	Eeprom_WriteBlockRedundant
	(
	    EE_RADIOMAPPING_START_ADDR, EE_MIRROR_ADDR_OFFS,
		(uint8_t *)&RC_Map, 
		sizeof(RC_Map)
    );

	Eeprom_CalDataStatus.Bits.RadioMappingValid_flg = TRUE;
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteToSystemSetting												 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write system datablock redundant to the EEPROM always incl. CRC32 at the  */
/* end.	After execution radio valid flag will be set.						 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteToSystemSetting(void)
{
    /* write system datablock redundant to the EEPROM (incl. CRC) 		     */

    Eeprom_WriteBlockRedundant
	(
	    EE_SYSTEMSETT_START_ADDR, EE_MIRROR_ADDR_OFFS,
		(uint8_t *)&Eeprom_CalSystemSetting, 
		sizeof(Eeprom_CalSystemSetting)
    );

	Eeprom_CalDataStatus.Bits.RemoteCtrlDataValid_flg = TRUE;


}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteToVariantSetting(uint8_t Variant_uc)							 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write variant datablock redundant to the EEPROM always incl. CRC32 at the */
/* end.	After execution radio valid flag will be set.						 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteToVariantSetting(uint8_t VarianteSetting_uc)
{

	uint32_t VARIANTSETT_START_ADDR_VAR;

	VARIANTSETT_START_ADDR_VAR = EE_VARIANTSETT_START_ADDR + 
				((uint16_t)VarianteSetting_uc * sizeof(Var[0]));

    /* write variant datablock redundant to the EEPROM always (incl. CRC)  	*/
    Eeprom_WriteBlockRedundant
	(
	    VARIANTSETT_START_ADDR_VAR, EE_MIRROR_ADDR_OFFS,
		(uint8_t *)&Var[VarianteSetting_uc], 
		sizeof(Var[VarianteSetting_uc])
    );

	Eeprom_CalDataStatus.Bits.RemoteCtrlDataValid_flg = TRUE;

}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_WriteToMotorMixerSection											 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* write MotorMixer datablock redundant to the EEPROM always incl. CRC32	 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
void Eeprom_WriteToMotorMixerSection(void)
{
	/* write variant datablock redundant to the EEPROM always (incl. CRC)    */
    Eeprom_WriteBlockRedundant
	(
	    EE_MOTORMIXER_START_ADDR, EE_MIRROR_ADDR_OFFS,
		(uint8_t *)&MotorGain, 
		sizeof(MotorGain)
    );
}


/*****************************************************************************/
/* FUNCTION_NAME:															 */
/* Eeprom_ReadAll															 */
/*---------------------------------------------------------------------------*/
/* FUNCTION_DESCRIPTION:													 */
/* after system start the complete EEPROM data will be read. Result will be	 */
/* writen to the global valid flags.										 */
/*****************************************************************************/
/* FUNCTION_PARAMETERS:														 */
/* Input																	 */
/* void 																	 */
/* Return:																	 */
/* void																		 */
/*****************************************************************************/
FctRetType Eeprom_ReadAll(void)
{
    EepromType_RedundantBlockStatus BlockStatus;
	FctRetType RetStatus;
	/*lokal counter															 */
	uint8_t temp,errorcounter_uc=0;		
	
	/* in the beginning all sektion are damaged.							 */
    Eeprom_CalDataStatus.Byte_uc = 0; 
	
	/* read offset data and check											 */
    BlockStatus = 
	    Eeprom_ReadAndVerifyBlockRedundant
        (
            EE_OFFSCORR_START_ADDR, EE_MIRROR_ADDR_OFFS,
	        (uint8_t *)&Eeprom_CalDataSecOffsCorr, 
			sizeof(Eeprom_CalDataSecOffsCorr)
        );

    if (BlockStatus != BOTH_BLOCKS_CORRUPTED)
	{
        /* minimum one block is valid --> data is available					 */
        Eeprom_CalDataStatus.Bits.OffsCorrDataValid_flg = TRUE;
	}

	/* read radio data and check											 */
    BlockStatus = 
	    Eeprom_ReadAndVerifyBlockRedundant
        (
            EE_REMOTECTRL_START_ADDR, EE_MIRROR_ADDR_OFFS,
	        (uint8_t *)&Eeprom_CalDataSecRemoteCtrl, 
			sizeof(Eeprom_CalDataSecRemoteCtrl)
        );

    if (BlockStatus != BOTH_BLOCKS_CORRUPTED)
	{
        /* minimum one block is valid --> data is available					 */
		Eeprom_CalDataStatus.Bits.RemoteCtrlDataValid_flg = TRUE;
	}
	
	/* read radiomapping data and check										 */
	BlockStatus = 
	    Eeprom_ReadAndVerifyBlockRedundant
        (
            EE_RADIOMAPPING_START_ADDR, EE_MIRROR_ADDR_OFFS,
	        (uint8_t *)&RC_Map, 
			sizeof(RC_Map)
        );

    if (BlockStatus != BOTH_BLOCKS_CORRUPTED)
	{
        /* minimum one block is valid --> data is available					 */    
        Eeprom_CalDataStatus.Bits.RadioMappingValid_flg = TRUE;
	}
	
	/* read system data and check											 */
    BlockStatus = 
	    Eeprom_ReadAndVerifyBlockRedundant
        (
            EE_SYSTEMSETT_START_ADDR, EE_MIRROR_ADDR_OFFS,
	        (uint8_t *)&Eeprom_CalSystemSetting, 
			sizeof(Eeprom_CalSystemSetting)
        );

    if (BlockStatus != BOTH_BLOCKS_CORRUPTED)
	{
        /* minimum one block is valid --> data is available					 */    
        Eeprom_CalDataStatus.Bits.SystemSettingValid_flg = TRUE;
	}
	
	/* read MotorMixer data and check										 */
	 BlockStatus = 
	    Eeprom_ReadAndVerifyBlockRedundant
        (
			EE_MOTORMIXER_START_ADDR, EE_MIRROR_ADDR_OFFS,
			(uint8_t *)&MotorGain,
			sizeof(MotorGain)
		);

	if (BlockStatus != BOTH_BLOCKS_CORRUPTED)
	{
        /* minimum one block is valid --> data is available					 */    
        Eeprom_CalDataStatus.Bits.MotorMixerSettingValid_flg = TRUE;
	}
	
	/* read Parameter data and check										 */
	for(temp=0;temp<P_MaxVariant;temp++)
	{
		/* compute adress for variant data									 */
		uint32_t VARIANTSETT_START_ADDR_VAR;
		VARIANTSETT_START_ADDR_VAR = EE_VARIANTSETT_START_ADDR + 
			((uint16_t)temp * sizeof(Var[0]));

		/* read variant data and check										 */         
    	BlockStatus = 
	    	Eeprom_ReadAndVerifyBlockRedundant
        	(

            	VARIANTSETT_START_ADDR_VAR, EE_MIRROR_ADDR_OFFS,
	        	(uint8_t *)&Var[temp], 
				sizeof(Var[temp])
        	);

	   
	    if (BlockStatus != BOTH_BLOCKS_CORRUPTED)
		{
		   errorcounter_uc++;
		}
		if(errorcounter_uc==P_MaxVariant)
		{
        /* minimum one block is valid --> data is available					 */      
	        Eeprom_CalDataStatus.Bits.VariantSetting_flg = TRUE;
		}
	}

	
    /* tip from www.mikrocontroller.net-Forum to set EEAR to unused adress   */
	/* after EEPROM execution										         */
	EEAR = EE_LAST_ADDR;
  
	if (Eeprom_CalDataStatus.Byte_uc != EEPROM_ALL_SECTIONS_OK)
	{
        RetStatus = RET_FAIL;
	}
	else
	{
        RetStatus = RET_OK;
    }

	return(RetStatus);
}

