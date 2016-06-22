/*****************************************************************************
 * @Function : Function that must execute in RAM
 * @Author : Arthur, Welic
 * @version : 1.5.4.0
 * @Modify Date : 20131016
 * @Copyright GridComm-PLC (C) 2013
 *****************************************************************************/

#include "global.h"
#include "plcm_uart.h"
#include "platform.h"

/*
 * Flash operations:
 * NOTE: These need to be run in RAM.
 */
#define FLASH_LOCK()		FLASH->CR |= FLASH_CR_LOCK

//Disable and Enable Global interrupts are placed here as they are called by
//flash routines which cannot call functions stored in Flash.
static uint8_t _SemGlobal=0;


void DisableGlobalInterrupts(void)
{
  PLATFORM_DisableGlobalIRQ();
  _SemGlobal++;
}

void EnableGlobalInterrupts(void)
{
  if(_SemGlobal>0)
  {
    _SemGlobal--;
  }

  if(_SemGlobal==0)
  {
    PLATFORM_EnableGlobalIRQ();
  }
}

static FLASH_Status FLASH_WAIT(uint32_t Timeout)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Wait for a FLASH operation to complete or a TIMEOUT to occur */
  while(((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) && (Timeout != 0x00))
  {
    Timeout--;
  }

  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}

static void FLASH_UNLOCK(void)
{
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Unlocking the program memory access */
    FLASH->KEYR = FLASH_FKEY1;
    FLASH->KEYR = FLASH_FKEY2;
  }
}

void PLATFORM_EraseFlashPage(uint32_t startAddress)
{
  DisableGlobalInterrupts();
	FLASH_UNLOCK();

	if(FLASH_WAIT(FLASH_ER_PRG_TIMEOUT) == FLASH_COMPLETE)
	{
		/* If the previous operation is completed, proceed to erase the page */
		FLASH->CR |= FLASH_CR_PER;
		FLASH->AR  = startAddress;
		FLASH->CR |= FLASH_CR_STRT;

		/* Wait for last operation to be completed */
		FLASH_WAIT(FLASH_ER_PRG_TIMEOUT);

		/* Disable the PER Bit */
		FLASH->CR &= ~FLASH_CR_PER;
	}

	FLASH_LOCK();
  EnableGlobalInterrupts();
}

void PLATFORM_WriteFlash(uint32_t *address, uint8_t *pdata, int numBytes)
{
    int count;
    int numHalfWords;
    volatile uint16_t * addr = (uint16_t *)address;

    numHalfWords = (numBytes+1)/2;	 				/* Convert bytes to words */

  DisableGlobalInterrupts();
    FLASH_UNLOCK();
	FLASH->CR |= FLASH_CR_PG;
    for (count = 0; count < numHalfWords; count++)
    {
    	if(FLASH_WAIT(FLASH_ER_PRG_TIMEOUT) == FLASH_COMPLETE)
    	{
    		*addr++ = *((uint16_t *)pdata);
    	}
    	else
    	{
    		break;
    	}
    	pdata += 2;
    }
	FLASH_WAIT(FLASH_ER_PRG_TIMEOUT);
	/* Disable the PG Bit */
	FLASH->CR &= ~FLASH_CR_PG;
    FLASH_LOCK();
  EnableGlobalInterrupts();
}


void PLATFORM_CompleteFirmwareUpgrade(uint8_t numPages, bool newFwImage)
{
  uint8_t *tempchar;
  uint32_t bank;

//here is no return path, require reboot the system, as the ROM is overwritten.
  DisableGlobalInterrupts();

  tempchar=(uint8_t *)TEMP_UPGRADE_IMAGE_ADDR;

  if (newFwImage)
  {
    tempchar += UPG_IMG_HEADER_SIZE;
  }

  for(bank=0; bank<numPages; bank++)
  {

    PLATFORM_EraseFlashPage(IMAGE_START_ADDR+bank*FLASH_PAGE_SIZE);
    PLATFORM_WriteFlash((uint32_t *)(bank*FLASH_PAGE_SIZE), tempchar, FLASH_PAGE_SIZE);

    tempchar+=FLASH_PAGE_SIZE;
  }

  if (newFwImage)
  {
    //remove upgrade header
    PLATFORM_EraseFlashPage(TEMP_UPGRADE_IMAGE_ADDR);
  }

  // The system will reboot as the watchdog kick in
  REBOOT();
  //can't return from here as the flash is execute in place and all the code will be overwrite by the new data.
  //Defer rebooting until function returns
}

