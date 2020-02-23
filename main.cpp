#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "Arduino.h"
#include "esp_spi_flash.h"
#include "TNSerialGlobal.h"
#include <esp_partition.h>
//#include "SPIFFS.h"
#define  DISK_SEC_LEN  0x1000
static uint32_t DFR_uDiskBaseAddress = 0x671000;
TaskHandle_t loopTaskHandle = NULL;
TaskHandle_t udiskTaskHandle = NULL;

#if CONFIG_AUTOSTART_ARDUINO

uint32_t UsbSerialbaud = 115200;
uint32_t NowSerialbaud = 115200;

static void SendFlashMYBUF(uint32_t Sec, uint32_t Offset, uint32_t TotLen){
    uint32_t flashAddr = 0x00;
    static uint8_t tempData[512]={0x00};
    flashAddr = (Sec * DISK_SEC_LEN + (TotLen - Offset));
    spi_flash_read((flashAddr+DFR_uDiskBaseAddress),tempData,512);
    uDiskSerial.write(tempData,32);
    delay(12);
    uDiskSerial.write(tempData+32,512-32);
}

void udiskTask(void *pvParameters){
    static uint8_t flashData[4096]={};
    uint8_t tempLen = 0;
    uint8_t CMD = 0;
    uint32_t SecNum = 0;
    uint32_t buflen = 0;
    uint8_t tempbuf[20];
    uint32_t totalLen = 0;
    
    uint32_t receviceDataLen = 0;
    uint32_t earseAddr = 0;
    uint32_t earseLen = 0;
    esp_partition_iterator_t iterator = NULL;
    const esp_partition_t *next_partition = NULL;
    iterator = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    next_partition = esp_partition_get(iterator);
    if (next_partition != NULL) {
        DFR_uDiskBaseAddress = next_partition->address;
    }
    uDiskSerial.begin(500000);
    Serial2.begin(500000);
    //delay(2000);
    pinMode(34,INPUT);
    
    for(;;){
    if(digitalRead(34) == 0)
    {
#if 1
        if(uDiskSerial.baudRate() != 500000){
            uDiskSerial.updateBaudRate(500000);
        }
first:
        if(uDiskSerial.available()){
            tempbuf[tempLen] = (uint8_t)uDiskSerial.read();
            tempLen++;
        }
        if(tempLen == 11){
            tempLen = 0;
            if(tempbuf[0] != 0x55 || tempbuf[1] != 0xBB){
                while(uDiskSerial.available()){
                    uDiskSerial.read();
                }
                goto first;
            }else{
            }
            CMD = tempbuf[2];
            if((CMD == 0x04) || (CMD == 0x03)){
            }else{
                buflen = ((tempbuf[3]<<24) | (tempbuf[4]<<16) | (tempbuf[5]<<8) | tempbuf[6]);
                SecNum = ((tempbuf[7]<<24) | (tempbuf[8]<<16) | (tempbuf[9]<<8) | tempbuf[10]);
                totalLen = buflen;
            }
            if(CMD == 0x00){
                SendFlashMYBUF(SecNum,buflen,totalLen);
            }else if(CMD == 0x01){//写flash
                if(0){
receviveWrite:
                    uint8_t flashdata[64]={0x00};
                    tempLen = 0;
                    while(1){
                        while(uDiskSerial.available()){
                            flashdata[tempLen] = uDiskSerial.read();
                            tempLen++;
                            if(tempLen == 0x40){
                                receviceDataLen += 0x40;
                                spi_flash_write((earseAddr+receviceDataLen-0x40),flashdata,tempLen);
                                break;
                                
                            }
                        }
                        if(receviceDataLen == earseLen){
                            tempLen = 0;
                            earseAddr = 0;
                            earseLen = 0;
                            receviceDataLen = 0;
                            goto first;
                        }
                        
                        if(tempLen == 0x40){
                            tempLen = 0;
                            goto receviveWrite;
                        }
                    }
                }
            }else if(CMD == 0x04){
                buflen -= 0x200;
                SendFlashMYBUF(SecNum,buflen,totalLen);
                //delay(1);
            }else if(CMD == 0x03){//重发上一次数据包
                SendFlashMYBUF(SecNum,buflen,totalLen);
            }
            else if(CMD == 0xFF){//擦除flash
                earseAddr = (DFR_uDiskBaseAddress+(SecNum * DISK_SEC_LEN));
                earseLen = buflen;
                if(earseAddr%0x1000 == 0){
                    if(buflen < 0x1000){
                        spi_flash_read(earseAddr,flashData,0x1000);
                        spi_flash_erase_range(earseAddr, 0x1000);
                        spi_flash_write((earseAddr+buflen),(flashData+buflen),(0x1000-buflen));
                    }else{
                        spi_flash_erase_range(earseAddr, buflen);
                    }
                }else{
                    {
                        uint32_t AddressSum = earseAddr+earseLen;//擦除到该地址的flash
                        uint32_t FirstSum = earseAddr%0x1000;//第一片flash需要补全的大小。
                        uint32_t StartAddress = earseAddr-(earseAddr%0x1000);//从这个flash地址开始最开始的操作。
                        uint32_t LastSum = AddressSum%0x1000;//最后一片flash应该擦除多长的flash
                        
                        spi_flash_read(StartAddress,flashData,0x1000);
                        spi_flash_erase_range(StartAddress, 0x1000);
                        spi_flash_write(StartAddress,flashData,FirstSum);
                        if(LastSum == 0){/*flash正好擦除*/
                            spi_flash_erase_range(StartAddress+0x1000, (AddressSum - StartAddress) );
                        }else{
                            spi_flash_read((AddressSum - LastSum),flashData,0x1000);
                            spi_flash_erase_range(StartAddress+0x1000, (AddressSum-StartAddress-0x1000-LastSum)+0x1000);
                            spi_flash_write(AddressSum,flashData+LastSum,(0x1000-LastSum));
                        }
                    }
                }
                goto receviveWrite;
            }else{
            }
        }else{
        }
    #endif
    }else{
        if(NowSerialbaud != UsbSerialbaud){
            uDiskSerial.updateBaudRate(UsbSerialbaud);
        }
        yield();
    }
    }
}


bool loopTaskWDTEnabled;

void loopTask(void *pvParameters)
{
    delay(1000);
    setup();
    pinMode(34,INPUT);
    for(;;) {
        if(loopTaskWDTEnabled){
            esp_task_wdt_reset();
        }
        if(digitalRead(34) == 1){
            loop();
            yield();
        }else{
            delay(10);
            yield();
        }
    }
}

extern "C" void app_main()
{
    loopTaskWDTEnabled = false;
    initArduino();
    xTaskCreateUniversal(udiskTask, "udiskTask", 2048, NULL, 1, &udiskTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreateUniversal(loopTask, "loopTask", 8192, NULL, 1, &loopTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    
}

#endif
