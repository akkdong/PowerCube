/*
 * fat_app.c
 *
 *
 */

/* Private Includes */
#include <string.h>
#include <stdlib.h>
#include "fat_app.h"
#include "main.h"

/* Global Variables */
uint8_t versionStr[MAX_VERSIONSTR];
uint8_t ramBuffer[FAT_RAM_ALL_NBR*FAT_BLOCK_SIZE];									// Buffer to store some FATFS Sectors
uint8_t blkSector[FAT_BLOCK_SIZE];													// Blank sector to return an empty value for USB

FAT_BootSector_t *fatFS = (FAT_BootSector_t *)ramBuffer;							// Pointer to the Boot Sector into the USB RAM Buffer
FAT_BaseEntry_t *binFile;															// Pointer to the binary file
FAT_SystemInfo_t fatINFO;															// Type def to store some FAT Information, Settings and Status

/* Private Function Prototypes */
uint16_t FAT_GetRootDirSectors(FAT_BootSector_t* FATx);
uint16_t FAT_GetFirstDataSector(FAT_BootSector_t* FATx);
uint16_t FAT_GetFirstRootDirSecNum(FAT_BootSector_t* FATx);

/* Private Functions */
/**
 * @brief  Get the count of sectors occupied by the root directory
 * @param  FATx: Pointer to the FAT Boot Sector.
 * @retval RootDirSectors: Number of Root Directory Sectors
 */
uint16_t FAT_GetRootDirSectors(FAT_BootSector_t* FATx)
{
	/* Local Variables */
	uint16_t RootDirSectors		= 0,
			 RootEntCnt 		= (uint16_t)(FATx->RootEntCnt[1] << 8) + FATx->RootEntCnt[0],
			 BytsPerSec 		= (uint16_t)(FATx->BytsPerSec[1] << 8) + FATx->BytsPerSec[0];

	/* RootDirSectors Calculation */
	RootDirSectors = ((RootEntCnt * 32) + (BytsPerSec - 1)) / BytsPerSec;

	/* Return calculated value */
	return RootDirSectors;
}

/**
 * @brief  Get the first sector that contains file data
 * @param  FATx: Pointer to the FAT Boot Sector.
 * @retval FirstDataSector: Number of the first Sector that cointais file data
 */
uint16_t FAT_GetFirstDataSector(FAT_BootSector_t* FATx)
{
	/* Local Variables */
	uint8_t	 NumFATs 			= (uint8_t)FATx->NumFATs;
	uint16_t RootDirSectors		= 0,
			 FirstDataSector	= 0,
			 RootEntCnt 		= (uint16_t)(FATx->RootEntCnt[1] << 8) + FATx->RootEntCnt[0],
			 BytsPerSec 		= (uint16_t)(FATx->BytsPerSec[1] << 8) + FATx->BytsPerSec[0],
			 ResvdSecCnt 		= (uint16_t)(FATx->ResvdSecCnt[1] << 8) + FATx->ResvdSecCnt[0];
	uint64_t FATSz;


	/* Count of sectors occupied by the root directory */
	RootDirSectors = ((RootEntCnt * 32) + (BytsPerSec - 1)) / BytsPerSec;

	/* Get the First Data Sector */
	FATSz = (uint16_t)(FATx->FATSz16[1]<<8) + FATx->FATSz16[0];
	FirstDataSector = ResvdSecCnt + (NumFATs * FATSz) + RootDirSectors;

	/* Return calculated value */
	return FirstDataSector;
}

/**
 * @brief  Get the sector where the Root Directory is located
 * @param  FATx: Pointer to the FAT Boot Sector.
 * @retval FirstRootDirSecNum: Number of the first Sector that cointais the Root Directory
 */
uint16_t FAT_GetFirstRootDirSecNum(FAT_BootSector_t* FATx)
{
	/* Local Variables */
	uint8_t  NumFATs 			= (uint8_t)FATx->NumFATs;
	uint16_t FirstRootDirSecNum = 0,
			 ResvdSecCnt 		= (uint16_t)(FATx->ResvdSecCnt[1]<<8) + FATx->ResvdSecCnt[0],
			 FATSz16 			= (uint16_t)(FATx->FATSz16[1]<<8) + FATx->FATSz16[0];

	/* Calculate the First Root Directory Sector Number */
	FirstRootDirSecNum = ResvdSecCnt + (NumFATs * FATSz16);

	/* Return calculated value */
	return FirstRootDirSecNum;
}

/**
 *
 */

static uint8_t *GetFirstFAT(FAT_BootSector_t* FATx)
{
	uint16_t ResvdSecCnt = (uint16_t)(FATx->ResvdSecCnt[1]<<8) + FATx->ResvdSecCnt[0];

	return (uint8_t *)FATx + (ResvdSecCnt * FAT_BLOCK_SIZE);
}

/**
 *
 */

static uint8_t *GetSecondFAT(FAT_BootSector_t* FATx)
{
	uint16_t ResvdSecCnt = (uint16_t)(FATx->ResvdSecCnt[1]<<8) + FATx->ResvdSecCnt[0];
	uint16_t FATSz16 = (uint16_t)(FATx->FATSz16[1]<<8) + FATx->FATSz16[0];

	return (uint8_t *)FATx + ((ResvdSecCnt + FATSz16) * FAT_BLOCK_SIZE);
}

/**
 *
 */

static FAT_BaseEntry_t *GetRootDirEntry(FAT_BootSector_t* FATx)
{
	uint16_t FirstRootDirSecNum = FAT_GetFirstRootDirSecNum(FATx);

	return (FAT_BaseEntry_t *)((uint8_t *)FATx + (FirstRootDirSecNum * FAT_BLOCK_SIZE));
}


/* Public Functions */
/**
 * @brief  Initialize the FAT Sectors into RAM Buffer
 */

static FAT_BootSector_t hFAT =
{
	{0xEB, 0x3C, 0x90},				//jmpBoot
	"MSDOS5.0",						//OEMName
	{0x00, 0x08},					//BytsPerSec
	0x01,							//SecPerClus
	{0x01, 0x00},					//ResvdSecCnt
	0x02,							//NumFATs
	{0x40, 0x00},					//RootEntCnt	64(40h) --> 2048(1 sector) / 32 = 64 entries
	{0x04, 0x04},					//TotSec16		2M(DATA) + 4(REV,FAT1/2,ROOT) -> 1028 sectors
	0xF8,							//Media
	{0x01, 0x00},					//FATSz16
	{0x01, 0x00},					//SecPerTrk
	{0x01, 0x00},					//NumHeads
	{0x00, 0x00, 0x00, 0x00},		//HiddSec
	{0x00, 0x00, 0x00, 0x00},		//TotSec32
	0x80,							//DrvNum
	0x00,							//Reserved
	0x29,							//BootSig
	{0xDE, 0xAD, 0xBE, 0xEF},		//VolID
	"NO NAME    ",					//VolLab
	"FAT12   "						//FilSysType
};

static FAT_BaseEntry_t hVolume =
{
	"FWUP-CUBE  ",					//Volume Name - It can be changed, respecting the 11 bit length, fill with spaces until achieve 11 bytes
	0x08,							//Attribute
	0x00,							//Reserved
	0x00,							//CreationMS
	{0x00, 0x00},					//Creation Time
	{0x00, 0x00},					//Creation Date
	{0x00, 0x00},					//LastAccessDate
	{0x00, 0x00},					//First Cluster High
	{0x00, 0x00},					//Last Write Time
	{0x00, 0x00},					//Last Write Date
	{0x00, 0x00},					//First Cluster Low
	{0x00, 0x00, 0x00, 0x00}		//Size
};

static FAT_BaseEntry_t hVerFile =
{
	"VERSION TXT",					//Volume Name - It can be changed, respecting the 11 bit length, fill with spaces until achieve 11 bytes
	0x20,							//Attribute
	0x00,							//Reserved
	0x00,							//CreationMS
	{0x00, 0x00},					//Creation Time
	{0x00, 0x00},					//Creation Date
	{0x00, 0x00},					//LastAccessDate
	{0x00, 0x00},					//First Cluster High
	{0x00, 0x00},					//Last Write Time
	{0x00, 0x00},					//Last Write Date
	{0x02, 0x00},					//First Cluster Low		Sector: #2
	{0x20, 0x00, 0x00, 0x00}		//Size					Size: 32 bytes
};


void FAT_Init(void)
{
	// Clear buffer
	memset(ramBuffer, 0, sizeof(ramBuffer));
	memset(blkSector, 0, sizeof(blkSector));

	/* Initialize the Boot Sector and BIOS Parameter Block(BPB) */
	*fatFS = hFAT;

	/* Compute the File System Information and Settings */
	fatINFO.ReceivedFile 				= RESET;
	fatINFO.FileTransferCpt 			= RESET;
	fatINFO.RootDirSectors 				= FAT_GetRootDirSectors(fatFS);
	fatINFO.FirstDataSector 			= FAT_GetFirstDataSector(fatFS);
	fatINFO.FirstRootDirSecNum 			= FAT_GetFirstRootDirSecNum(fatFS);
	fatINFO.FirstDataPosition 			= 0;
	fatINFO.FileSize 					= 0;
	fatINFO.TransferredData 			= 0;

	// FAT allocation
	uint8_t *FAT1 = GetFirstFAT(fatFS);
	uint8_t *FAT2 = GetSecondFAT(fatFS);

	// Sector #0: FF0   : reserved
	//        #1: FFF   : reserved
	//        #2: FFF	: EOC of version file
	//        #3: FFF	: EOC of reserved(block sector) file
	//
	// F0 FF FF FF FF FF ...
	//
	FAT1[0] = FAT2[0] = 0xF0;
	FAT1[1] = FAT2[1] = 0xFF;
	FAT1[2] = FAT2[2] = 0xFF;
	FAT1[3] = FAT2[3] = 0xFF;
	FAT1[4] = FAT2[4] = 0xFF;
	FAT1[5] = FAT2[5] = 0xFF;

	/* Initialize the volume & version file */
	FAT_BaseEntry_t *ptrBaseEntry = GetRootDirEntry(fatFS); // (FAT_BaseEntry_t *)((uint8_t *)fatFS + ((fatINFO.FirstRootDirSecNum) * FAT_BLOCK_SIZE));
	ptrBaseEntry[0] = hVolume;	// Volume
	ptrBaseEntry[1] = hVerFile;	// Version file

	/* Fill version file */
	uint8_t *file = (uint8_t *)fatFS + (fatINFO.FirstDataSector * FAT_BLOCK_SIZE);

	memset(file, ' ', 32);
	memcpy(file, versionStr, strlen((const char *)versionStr));
}
