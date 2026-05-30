#ifndef SENSOR_OTA_FLASH_H_
#define SENSOR_OTA_FLASH_H_

#include "Ifx_Types.h"
#include "IfxFlash.h"

#define SENSOR_OTA_FLASH_MODULE        0U
#define SENSOR_OTA_PFLASH_PAGE_LEN     32U
#define SENSOR_OTA_PFLASH_SECTOR_SIZE  0x4000UL

#define SENSOR_OTA_TO_FLASH_ADDR(addr) (((addr) & 0x0FFFFFFFU) | 0xA0000000U)

#define SENSOR_OTA_RELOCATION_START    0x70100000U

#define SENSOR_OTA_ERASESECTOR_LEN     256U
#define SENSOR_OTA_WAITUNBUSY_LEN      256U
#define SENSOR_OTA_ENTERPAGEMODE_LEN   256U
#define SENSOR_OTA_LOAD2X32_LEN        256U
#define SENSOR_OTA_WRITEPAGE_LEN       256U
#define SENSOR_OTA_ERASEWRAPPER_LEN    0x400U
#define SENSOR_OTA_WRITEWRAPPER_LEN    0x400U

#define SENSOR_OTA_ERASESECTOR_ADDR    (SENSOR_OTA_RELOCATION_START)
#define SENSOR_OTA_WAITUNBUSY_ADDR     (SENSOR_OTA_ERASESECTOR_ADDR   + SENSOR_OTA_ERASESECTOR_LEN)
#define SENSOR_OTA_ENTERPAGEMODE_ADDR  (SENSOR_OTA_WAITUNBUSY_ADDR    + SENSOR_OTA_WAITUNBUSY_LEN)
#define SENSOR_OTA_LOAD2X32_ADDR       (SENSOR_OTA_ENTERPAGEMODE_ADDR + SENSOR_OTA_ENTERPAGEMODE_LEN)
#define SENSOR_OTA_WRITEPAGE_ADDR      (SENSOR_OTA_LOAD2X32_ADDR      + SENSOR_OTA_LOAD2X32_LEN)
#define SENSOR_OTA_ERASEWRAPPER_ADDR   (SENSOR_OTA_WRITEPAGE_ADDR     + SENSOR_OTA_WRITEPAGE_LEN)
#define SENSOR_OTA_WRITEWRAPPER_ADDR   (SENSOR_OTA_ERASEWRAPPER_ADDR  + SENSOR_OTA_ERASEWRAPPER_LEN)

boolean SensorOtaFlash_Erase(uint32 addr, uint32 size, IfxFlash_FlashType flashType);

/*
 * CPU1 erase worker용 API.
 * 기존 SensorOtaFlash_Erase()와 달리 CPU1/CPU2를 halt하지 않는다.
 * 1단계에서는 선언/구현만 추가하고 아직 호출하지 않는다.
 */
boolean SensorOtaFlash_EraseNoCoreHalt(uint32 addr,
                                        uint32 size,
                                        IfxFlash_FlashType flashType);

boolean SensorOtaFlash_Write(uint32 addr, const uint8 *data, uint16 len, IfxFlash_FlashType flashType);

#endif /* SENSOR_OTA_FLASH_H_ */
