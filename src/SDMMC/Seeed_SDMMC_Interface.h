#ifndef __SEEED_SDMMC_INTERFACE_H_
#define __SEEED_SDMMC_INTERFACE_H_
#ifdef SEEEDUINO_H7AI
#include <Arduino.h>
#include <Seeed_FS.h>
extern "C" { 
uint8_t MX_SDMMC1_SD_Init(void);
void DLYB_SDMMC1_Calibration(uint8_t phase);
DSTATUS SD_initialize(BYTE pdrv);
DSTATUS SD_status(BYTE pdrv);
DRESULT SD_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_ioctl(BYTE pdrv, BYTE cmd, void* buff);

extern SD_HandleTypeDef hsd1;
}
#endif
#endif
