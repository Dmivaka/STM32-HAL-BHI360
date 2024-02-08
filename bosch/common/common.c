/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    common.c
 * @brief   Common source file for the BHy260 examples
 *
 */

#include "common.h"
#include "main.h"   
   
#include "bhy2_parse.h"

#ifdef UPLOAD_FIRMWARE_TO_FLASH
// no flash firmware for BHI360?
#else
#include "firmware/bhi360/BHI360.fw.h"
#endif

// pointer to log output function
extern custom_printf_prt my_printf;

void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  (void)callback_ref;
  uint8_t meta_event_type = callback_info->data_ptr[0];
  uint8_t byte1 = callback_info->data_ptr[1];
  uint8_t byte2 = callback_info->data_ptr[2];
  uint8_t *accuracy = (uint8_t*)callback_ref;
  char *event_text;
  
  if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
  {
      event_text = "[META EVENT]";
  }
  else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
  {
      event_text = "[META EVENT WAKE UP]";
  }
  else
  {
      return;
  }
  
  switch (meta_event_type)
  {
    case BHY2_META_EVENT_FLUSH_COMPLETE:
      my_printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
      break;
    case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
      my_printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
      break;
    case BHY2_META_EVENT_POWER_MODE_CHANGED:
      my_printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
      break;
    case BHY2_META_EVENT_ALGORITHM_EVENTS:
      my_printf("%s Algorithm event\r\n", event_text);
      break;
    case BHY2_META_EVENT_SENSOR_STATUS:
      my_printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
      if (accuracy)
      {
          *accuracy = byte2;
      }
      
      break;
    case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
      my_printf("%s BSX event (do steps main)\r\n", event_text);
      break;
    case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
      my_printf("%s BSX event (do steps calib)\r\n", event_text);
      break;
    case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
      my_printf("%s BSX event (get output signal)\r\n", event_text);
      break;
    case BHY2_META_EVENT_SENSOR_ERROR:
      my_printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
      break;
    case BHY2_META_EVENT_FIFO_OVERFLOW:
      my_printf("%s FIFO overflow\r\n", event_text);
      break;
    case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
      my_printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
      break;
    case BHY2_META_EVENT_FIFO_WATERMARK:
      my_printf("%s FIFO watermark reached\r\n", event_text);
      break;
    case BHY2_META_EVENT_INITIALIZED:
      my_printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
      break;
    case BHY2_META_TRANSFER_CAUSE:
      my_printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
      break;
    case BHY2_META_EVENT_SENSOR_FRAMEWORK:
      my_printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
      break;
    case BHY2_META_EVENT_RESET:
      my_printf("%s Reset event\r\n", event_text);
      break;
    case BHY2_META_EVENT_SPACER:
      break;
    default:
      my_printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
      break;
  }
}

void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
  if (rslt != BHY2_OK)
  {
    my_printf("%s\r\n", get_api_error(rslt));
    
    Error_Handler();
  }
}

void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev)
{
  uint8_t sensor_error;
  int8_t temp_rslt;
  int8_t rslt = BHY2_OK;
  
#ifdef UPLOAD_FIRMWARE_TO_FLASH
  if (boot_stat & BHY2_BST_FLASH_DETECTED)
  {
    uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
    uint32_t end_addr = start_addr + sizeof(bhy2_firmware_image);
    my_printf("Flash detected. Erasing flash to upload firmware\r\n");

    rslt = bhy2_erase_flash(start_addr, end_addr, dev);
    print_api_error(rslt, dev);
  }
  else
  {
    my_printf("Flash not detected\r\n");

    rslt = BHY2_E_IO;
    print_api_error(rslt, dev);
  }
  
  my_printf("Loading firmware into FLASH.\r\n");
  rslt = bhy2_upload_firmware_to_flash(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);
#else
  my_printf("Loading firmware into RAM.\r\n");
  rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);
#endif
  temp_rslt = bhy2_get_error_value(&sensor_error, dev);
  if (sensor_error)
  {
    my_printf("%s\r\n", get_sensor_error_text(sensor_error));
  }
  
  print_api_error(rslt, dev);
  print_api_error(temp_rslt, dev);
  
#ifdef UPLOAD_FIRMWARE_TO_FLASH
  my_printf("Booting from FLASH.\r\n");
  rslt = bhy2_boot_from_flash(dev);
#else
  my_printf("Booting from RAM.\r\n");
  rslt = bhy2_boot_from_ram(dev);
#endif
  
  temp_rslt = bhy2_get_error_value(&sensor_error, dev);
  if (sensor_error)
  {
    my_printf("%s\r\n", get_sensor_error_text(sensor_error));
  }
  
  print_api_error(rslt, dev);
  print_api_error(temp_rslt, dev);
}

uint8_t printInfo(struct bhy2_dev *bhy2)
{
  int8_t rslt;
  
  uint16_t kernel_version = 0, user_version = 0;
  uint16_t rom_version = 0;
  uint8_t product_id = 0;
  uint8_t host_status = 0, feat_status = 0;
  uint8_t boot_status = 0;
  uint8_t sensor_error;
  struct bhy2_sensor_info info;
  
  // Get product_id
  if( bhy2_get_product_id(&product_id, bhy2) != BHY2_OK )
  {
    my_printf("bhy2_get_product_id failed!");
    Error_Handler();
  }
  
  // Get Kernel version
  if( bhy2_get_kernel_version(&kernel_version, bhy2) != BHY2_OK )
  {
    my_printf("bhy2_get_kernel_version failed!");
    Error_Handler();
  }
  
  // Get User version
  if( bhy2_get_user_version(&user_version, bhy2) != BHY2_OK )
  {
    my_printf("bhy2_get_user_version failed!");
    Error_Handler();
  }
  
  // Get ROM version
  if( bhy2_get_rom_version(&rom_version, bhy2) != BHY2_OK )
  {
    my_printf("bhy2_get_rom_version failed!");
    Error_Handler();
  }  
  
  if( bhy2_get_host_status(&host_status, bhy2) != BHY2_OK )
  {
    my_printf("bhy2_get_host_status failed!");
    Error_Handler();
  }  
  
  if( bhy2_get_feature_status(&feat_status, bhy2) != BHY2_OK )
  {
    my_printf("bhy2_get_feature_status failed!");
    Error_Handler();
  }
  
  my_printf("Product ID     : %02x\r\n", product_id);
  my_printf("Kernel version : %04u\r\n", kernel_version);
  my_printf("User version   : %04u\r\n", user_version);
  my_printf("ROM version    : %04u\r\n", rom_version);
  my_printf("Power state    : %s\r\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
  my_printf("Host interface : %s\r\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
  my_printf("Feature status : 0x%02x\r\n", feat_status);
  
  // Read boot status
  if( bhy2_get_boot_status(&boot_status, bhy2) != BHY2_OK )
  {
    my_printf("bhy2_get_boot_status failed!");
    Error_Handler();
  }      
  
  my_printf("Boot Status : 0x%02x: \r\n", boot_status);
  
  if (boot_status & BHY2_BST_FLASH_DETECTED) {
    my_printf("\tFlash detected.\r\n");
  }
  
  if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
    my_printf("\tFlash verify done.\r\n");
  }
  
  if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
    my_printf("Flash verification failed.\r\n");
  }
  
  if (boot_status & BHY2_BST_NO_FLASH) {
    my_printf("\tNo flash installed.\r\n");
  }
  
  if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
    my_printf("\tHost interface ready.\r\n");
  }
  
  if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
    my_printf("\tFirmware verification done.\r\n");
  }
  
  if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
    my_printf("\tFirmware verification error.\r\n");
  }
  
  if (boot_status & BHY2_BST_HOST_FW_IDLE) {
    my_printf("\tFirmware halted.\r\n");
  }
  
  // Read error value
  if( bhy2_get_error_value(&sensor_error, bhy2) != BHY2_OK )
  {
    my_printf("\tFirmware halted.\r\n");
  }
  
  rslt = bhy2_get_error_value(&sensor_error, bhy2);
  if (sensor_error)
  {
    my_printf("%s\r\n", get_sensor_error_text(sensor_error));
  }
  
  if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK)
  {
    bhy2_update_virtual_sensor_list(bhy2);
    
    // Get present virtual sensor
    bhy2_get_virt_sensor_list(bhy2);
    
    my_printf("Virtual sensor list.\r\n");
    my_printf("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\r\n");
    my_printf("----------+--------------------------------------+-----+-----+-----------+-----------|\r\n");
    for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++)
    {
      if (bhy2_is_sensor_available(i, bhy2))
      {
        if (i < BHY2_SENSOR_ID_CUSTOM_START)
        {
            my_printf(" %8u | %36s ", i, get_sensor_name(i));
        }
        if( bhy2_get_sensor_info(i, &info, bhy2) != BHY2_OK )
        {
          my_printf("bhy2_get_sensor_info failed!");
          Error_Handler();
        }
        my_printf("| %3u | %3u | %9.4f | %9.4f |\r\n",
                      info.driver_id,
                      info.driver_version,
                      info.min_rate.f_val,
                      info.max_rate.f_val);
      }
    }
  }
  return 1;
}

char *get_api_error(int8_t error_code)
{
    char *ret = " ";

    switch (error_code)
    {
        case BHY2_OK:
            break;
        case BHY2_E_NULL_PTR:
            ret = "[API Error] Null pointer";
            break;
        case BHY2_E_INVALID_PARAM:
            ret = "[API Error] Invalid parameter";
            break;
        case BHY2_E_IO:
            ret = "[API Error] IO error";
            break;
        case BHY2_E_MAGIC:
            ret = "[API Error] Invalid firmware";
            break;
        case BHY2_E_TIMEOUT:
            ret = "[API Error] Timed out";
            break;
        case BHY2_E_BUFFER:
            ret = "[API Error] Invalid buffer";
            break;
        case BHY2_E_INVALID_FIFO_TYPE:
            ret = "[API Error] Invalid FIFO type";
            break;
        case BHY2_E_INVALID_EVENT_SIZE:
            ret = "[API Error] Invalid Event size";
            break;
        case BHY2_E_PARAM_NOT_SET:
            ret = "[API Error] Parameter not set";
            break;
        default:
            ret = "[API Error] Unknown API error code";
    }

    return ret;
}

char *get_sensor_error_text(uint8_t sensor_error)
{
    char *ret;

    switch (sensor_error)
    {
        case 0x00:
            break;
        case 0x10:
            ret = "[Sensor error] Bootloader reports: Firmware Expected Version Mismatch";
            break;
        case 0x11:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Header CRC";
            break;
        case 0x12:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: SHA Hash Mismatch";
            break;
        case 0x13:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Image CRC";
            break;
        case 0x14:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: ECDSA Signature Verification Failed";
            break;
        case 0x15:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Public Key CRC";
            break;
        case 0x16:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Signed Firmware Required";
            break;
        case 0x17:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: FW Header Missing";
            break;
        case 0x19:
            ret = "[Sensor error] Bootloader reports: Unexpected Watchdog Reset";
            break;
        case 0x1A:
            ret = "[Sensor error] ROM Version Mismatch";
            break;
        case 0x1B:
            ret = "[Sensor error] Bootloader reports: Fatal Firmware Error";
            break;
        case 0x1C:
            ret = "[Sensor error] Chained Firmware Error: Next Payload Not Found";
            break;
        case 0x1D:
            ret = "[Sensor error] Chained Firmware Error: Payload Not Valid";
            break;
        case 0x1E:
            ret = "[Sensor error] Chained Firmware Error: Payload Entries Invalid";
            break;
        case 0x1F:
            ret = "[Sensor error] Bootloader reports: Bootloader Error: OTP CRC Invalid";
            break;
        case 0x20:
            ret = "[Sensor error] Firmware Init Failed";
            break;
        case 0x21:
            ret = "[Sensor error] Sensor Init Failed: Unexpected Device ID";
            break;
        case 0x22:
            ret = "[Sensor error] Sensor Init Failed: No Response from Device";
            break;
        case 0x23:
            ret = "[Sensor error] Sensor Init Failed: Unknown";
            break;
        case 0x24:
            ret = "[Sensor error] Sensor Error: No Valid Data";
            break;
        case 0x25:
            ret = "[Sensor error] Slow Sample Rate";
            break;
        case 0x26:
            ret = "[Sensor error] Data Overflow (saturated sensor data)";
            break;
        case 0x27:
            ret = "[Sensor error] Stack Overflow";
            break;
        case 0x28:
            ret = "[Sensor error] Insufficient Free RAM";
            break;
        case 0x29:
            ret = "[Sensor error] Sensor Init Failed: Driver Parsing Error";
            break;
        case 0x2A:
            ret = "[Sensor error] Too Many RAM Banks Required";
            break;
        case 0x2B:
            ret = "[Sensor error] Invalid Event Specified";
            break;
        case 0x2C:
            ret = "[Sensor error] More than 32 On Change";
            break;
        case 0x2D:
            ret = "[Sensor error] Firmware Too Large";
            break;
        case 0x2F:
            ret = "[Sensor error] Invalid RAM Banks";
            break;
        case 0x30:
            ret = "[Sensor error] Math Error";
            break;
        case 0x40:
            ret = "[Sensor error] Memory Error";
            break;
        case 0x41:
            ret = "[Sensor error] SWI3 Error";
            break;
        case 0x42:
            ret = "[Sensor error] SWI4 Error";
            break;
        case 0x43:
            ret = "[Sensor error] Illegal Instruction Error";
            break;
        case 0x44:
            ret = "[Sensor error] Bootloader reports: Unhandled Interrupt Error / Exception / Postmortem Available";
            break;
        case 0x45:
            ret = "[Sensor error] Invalid Memory Access";
            break;
        case 0x50:
            ret = "[Sensor error] Algorithm Error: BSX Init";
            break;
        case 0x51:
            ret = "[Sensor error] Algorithm Error: BSX Do Step";
            break;
        case 0x52:
            ret = "[Sensor error] Algorithm Error: Update Sub";
            break;
        case 0x53:
            ret = "[Sensor error] Algorithm Error: Get Sub";
            break;
        case 0x54:
            ret = "[Sensor error] Algorithm Error: Get Phys";
            break;
        case 0x55:
            ret = "[Sensor error] Algorithm Error: Unsupported Phys Rate";
            break;
        case 0x56:
            ret = "[Sensor error] Algorithm Error: Cannot find BSX Driver";
            break;
        case 0x60:
            ret = "[Sensor error] Sensor Self-Test Failure";
            break;
        case 0x61:
            ret = "[Sensor error] Sensor Self-Test X Axis Failure";
            break;
        case 0x62:
            ret = "[Sensor error] Sensor Self-Test Y Axis Failure";
            break;
        case 0x64:
            ret = "[Sensor error] Sensor Self-Test Z Axis Failure";
            break;
        case 0x65:
            ret = "[Sensor error] FOC Failure";
            break;
        case 0x66:
            ret = "[Sensor error] Sensor Busy";
            break;
        case 0x6F:
            ret = "[Sensor error] Self-Test or FOC Test Unsupported";
            break;
        case 0x72:
            ret = "[Sensor error] No Host Interrupt Set";
            break;
        case 0x73:
            ret = "[Sensor error] Event ID Passed to Host Interface Has No Known Size";
            break;
        case 0x75:
            ret = "[Sensor error] Host Download Channel Underflow (Host Read Too Fast)";
            break;
        case 0x76:
            ret = "[Sensor error] Host Upload Channel Overflow (Host Wrote Too Fast)";
            break;
        case 0x77:
            ret = "[Sensor error] Host Download Channel Empty";
            break;
        case 0x78:
            ret = "[Sensor error] DMA Error";
            break;
        case 0x79:
            ret = "[Sensor error] Corrupted Input Block Chain";
            break;
        case 0x7A:
            ret = "[Sensor error] Corrupted Output Block Chain";
            break;
        case 0x7B:
            ret = "[Sensor error] Buffer Block Manager Error";
            break;
        case 0x7C:
            ret = "[Sensor error] Input Channel Not Word Aligned";
            break;
        case 0x7D:
            ret = "[Sensor error] Too Many Flush Events";
            break;
        case 0x7E:
            ret = "[Sensor error] Unknown Host Channel Error";
            break;
        case 0x81:
            ret = "[Sensor error] Decimation Too Large";
            break;
        case 0x90:
            ret = "[Sensor error] Master SPI/I2C Queue Overflow";
            break;
        case 0x91:
            ret = "[Sensor error] SPI/I2C Callback Error";
            break;
        case 0xA0:
            ret = "[Sensor error] Timer Scheduling Error";
            break;
        case 0xB0:
            ret = "[Sensor error] Invalid GPIO for Host IRQ";
            break;
        case 0xB1:
            ret = "[Sensor error] Error Sending Initialized Meta Events";
            break;
        case 0xC0:
            ret = "[Sensor error] Bootloader reports: Command Error";
            break;
        case 0xC1:
            ret = "[Sensor error] Bootloader reports: Command Too Long";
            break;
        case 0xC2:
            ret = "[Sensor error] Bootloader reports: Command Buffer Overflow";
            break;
        case 0xD0:
            ret = "[Sensor error] User Mode Error: Sys Call Invalid";
            break;
        case 0xD1:
            ret = "[Sensor error] User Mode Error: Trap Invalid";
            break;
        case 0xE1:
            ret = "[Sensor error] Firmware Upload Failed: Firmware header corrupt";
            break;
        case 0xE2:
            ret = "[Sensor error] Sensor Data Injection: Invalid input stream";
            break;
        default:
            ret = "[Sensor error] Unknown error code";
    }

    return ret;
}

char *get_sensor_name(uint8_t sensor_id)
{
    char *ret;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC_PASS:
            ret = "Accelerometer passthrough";
            break;
        case BHY2_SENSOR_ID_ACC_RAW:
            ret = "Accelerometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_ACC:
            ret = "Accelerometer corrected";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS:
            ret = "Accelerometer offset";
            break;
        case BHY2_SENSOR_ID_ACC_WU:
            ret = "Accelerometer corrected wake up";
            break;
        case BHY2_SENSOR_ID_ACC_RAW_WU:
            ret = "Accelerometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_PASS:
            ret = "Gyroscope passthrough";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW:
            ret = "Gyroscope uncalibrated";
            break;
        case BHY2_SENSOR_ID_GYRO:
            ret = "Gyroscope corrected";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS:
            ret = "Gyroscope offset";
            break;
        case BHY2_SENSOR_ID_GYRO_WU:
            ret = "Gyroscope wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
            ret = "Gyroscope uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_MAG_PASS:
            ret = "Magnetometer passthrough";
            break;
        case BHY2_SENSOR_ID_MAG_RAW:
            ret = "Magnetometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_MAG:
            ret = "Magnetometer corrected";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS:
            ret = "Magnetometer offset";
            break;
        case BHY2_SENSOR_ID_MAG_WU:
            ret = "Magnetometer wake up";
            break;
        case BHY2_SENSOR_ID_MAG_RAW_WU:
            ret = "Magnetometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GRA:
            ret = "Gravity vector";
            break;
        case BHY2_SENSOR_ID_GRA_WU:
            ret = "Gravity vector wake up";
            break;
        case BHY2_SENSOR_ID_LACC:
            ret = "Linear acceleration";
            break;
        case BHY2_SENSOR_ID_LACC_WU:
            ret = "Linear acceleration wake up";
            break;
        case BHY2_SENSOR_ID_RV:
            ret = "Rotation vector";
            break;
        case BHY2_SENSOR_ID_RV_WU:
            ret = "Rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GAMERV:
            ret = "Game rotation vector";
            break;
        case BHY2_SENSOR_ID_GAMERV_WU:
            ret = "Game rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GEORV:
            ret = "Geo-magnetic rotation vector";
            break;
        case BHY2_SENSOR_ID_GEORV_WU:
            ret = "Geo-magnetic rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_ORI:
            ret = "Orientation";
            break;
        case BHY2_SENSOR_ID_ORI_WU:
            ret = "Orientation wake up";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
            ret = "Accelerometer offset wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
            ret = "Gyroscope offset wake up";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
            ret = "Magnetometer offset wake up";
            break;
        case BHY2_SENSOR_ID_TEMP:
            ret = "Temperature";
            break;
        case BHY2_SENSOR_ID_BARO:
            ret = "Barometer";
            break;
        case BHY2_SENSOR_ID_HUM:
            ret = "Humidity";
            break;
        case BHY2_SENSOR_ID_GAS:
            ret = "Gas";
            break;
        case BHY2_SENSOR_ID_TEMP_WU:
            ret = "Temperature wake up";
            break;
        case BHY2_SENSOR_ID_BARO_WU:
            ret = "Barometer wake up";
            break;
        case BHY2_SENSOR_ID_HUM_WU:
            ret = "Humidity wake up";
            break;
        case BHY2_SENSOR_ID_GAS_WU:
            ret = "Gas wake up";
            break;
        case BHY2_SENSOR_ID_KLIO:
            ret = "Klio";
            break;
        case BHY2_SENSOR_ID_KLIO_LOG:
            ret = "Klio log";
            break;
        case BHY2_SENSOR_ID_SWIM:
            ret = "Swim recognition";
            break;
        case BHY2_SENSOR_ID_SI_ACCEL:
            ret = "SI Accel";
            break;
        case BHY2_SENSOR_ID_SI_GYROS:
            ret = "SI Gyro";
            break;
        case BHY2_SENSOR_ID_LIGHT:
            ret = "Light";
            break;
        case BHY2_SENSOR_ID_LIGHT_WU:
            ret = "Light wake up";
            break;
        case BHY2_SENSOR_ID_PROX:
            ret = "Proximity";
            break;
        case BHY2_SENSOR_ID_PROX_WU:
            ret = "Proximity wake up";
            break;
        case BHY2_SENSOR_ID_STC:
            ret = "Step counter";
            break;
        case BHY2_SENSOR_ID_STC_WU:
            ret = "Step counter wake up";
            break;
        case BHY2_SENSOR_ID_STC_LP:
            ret = "Low Power Step counter";
            break;
        case BHY2_SENSOR_ID_STC_LP_WU:
            ret = "Low Power Step counter wake up";
            break;
        case BHY2_SENSOR_ID_SIG:
            ret = "Significant motion";
            break;
        case BHY2_SENSOR_ID_STD:
            ret = "Step detector";
            break;
        case BHY2_SENSOR_ID_STD_WU:
            ret = "Step detector wake up";
            break;
        case BHY2_SENSOR_ID_TILT_DETECTOR:
            ret = "Tilt detector";
            break;
        case BHY2_SENSOR_ID_WAKE_GESTURE:
            ret = "Wake gesture";
            break;
        case BHY2_SENSOR_ID_GLANCE_GESTURE:
            ret = "Glance gesture";
            break;
        case BHY2_SENSOR_ID_PICKUP_GESTURE:
            ret = "Pickup gesture";
            break;
        case BHY2_SENSOR_ID_SIG_LP:
            ret = "Low Power Significant motion";
            break;
        case BHY2_SENSOR_ID_SIG_LP_WU:
            ret = "Low Power Significant motion wake up";
            break;
        case BHY2_SENSOR_ID_STD_LP:
            ret = "Low Power Step detector";
            break;
        case BHY2_SENSOR_ID_STD_LP_WU:
            ret = "Low Power Step detector wake up";
            break;
        case BHY2_SENSOR_ID_AR:
            ret = "Activity recognition";
            break;
        case BHY2_SENSOR_ID_EXCAMERA:
            ret = "External camera trigger";
            break;
        case BHY2_SENSOR_ID_GPS:
            ret = "GPS";
            break;
        case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
            ret = "Wrist tilt gesture";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
            ret = "Device orientation";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
            ret = "Device orientation wake up";
            break;
        case BHY2_SENSOR_ID_STATIONARY_DET:
            ret = "Stationary detect";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION_LP:
            ret = "Low Power Any motion";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
            ret = "Low Power Any motion wake up";
            break;
        case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
            ret = "Low Power No Motion wake up";
            break;
        case BHY2_SENSOR_ID_MOTION_DET:
            ret = "Motion detect";
            break;
        case BHI3_SENSOR_ID_AR_WEAR_WU:
            ret = "Activity recognition for Wearables";
            break;
        case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
            ret = "Low Power Wrist Wear wake up";
            break;
        case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            ret = "Low Power Wrist Gesture wake up";
            break;
        case BHI3_SENSOR_ID_MULTI_TAP:
            ret = "Multi Tap Detector";
            break;
        case BHY2_SENSOR_ID_AIR_QUALITY:
            ret = "Air Quality";
            break;
        case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
            ret = "Head Misalignment Calibrator";
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
            ret = "IMU Head Orientation Quaternion";
            break;
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
            ret = "NDOF Head Orientation Quaternion";
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
            ret = "IMU Head Orientation Euler";
            break;
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
            ret = "NDOF Head Orientation Euler";
            break;
        default:
            if ((sensor_id >= BHY2_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHY2_SENSOR_ID_CUSTOM_END))
            {
                ret = "Custom sensor ID ";
            }
            else
            {
                ret = "Undefined sensor ID ";
            }
    }

    return ret;
}

float get_sensor_default_scaling(uint8_t sensor_id)
{
    float scaling = -1.0f;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC_PASS:
        case BHY2_SENSOR_ID_ACC_RAW:
        case BHY2_SENSOR_ID_ACC:
        case BHY2_SENSOR_ID_ACC_BIAS:
        case BHY2_SENSOR_ID_ACC_WU:
        case BHY2_SENSOR_ID_ACC_RAW_WU:
        case BHY2_SENSOR_ID_GRA:
        case BHY2_SENSOR_ID_GRA_WU:
        case BHY2_SENSOR_ID_LACC:
        case BHY2_SENSOR_ID_LACC_WU:
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
            scaling = 1.0f / 4096.0f;
            break;
        case BHY2_SENSOR_ID_GYRO_PASS:
        case BHY2_SENSOR_ID_GYRO_RAW:
        case BHY2_SENSOR_ID_GYRO:
        case BHY2_SENSOR_ID_GYRO_BIAS:
        case BHY2_SENSOR_ID_GYRO_WU:
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
            scaling = 2000.0f / 32768.0f;
            break;
        case BHY2_SENSOR_ID_MAG_PASS:
        case BHY2_SENSOR_ID_MAG_RAW:
        case BHY2_SENSOR_ID_MAG:
        case BHY2_SENSOR_ID_MAG_BIAS:
        case BHY2_SENSOR_ID_MAG_WU:
        case BHY2_SENSOR_ID_MAG_RAW_WU:
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
            scaling = 2500.0f / 32768.0f;
            break;
        case BHY2_SENSOR_ID_RV:
        case BHY2_SENSOR_ID_RV_WU:
        case BHY2_SENSOR_ID_GAMERV:
        case BHY2_SENSOR_ID_GAMERV_WU:
        case BHY2_SENSOR_ID_GEORV:
        case BHY2_SENSOR_ID_GEORV_WU:
            scaling = 1.0f / 16384.0f;
            break;
        case BHY2_SENSOR_ID_ORI:
        case BHY2_SENSOR_ID_ORI_WU:
            scaling = 360.0f / 32768.0f;
            break;
        case BHY2_SENSOR_ID_TEMP:
        case BHY2_SENSOR_ID_TEMP_WU:
            scaling = 1.0f / 100.0f;
            break;
        case BHY2_SENSOR_ID_BARO:
        case BHY2_SENSOR_ID_BARO_WU:
            scaling = 100.0f / 128.0f;
            break;
        case BHY2_SENSOR_ID_HUM:
        case BHY2_SENSOR_ID_HUM_WU:
            scaling = 1.0f;
            break;
        case BHY2_SENSOR_ID_GAS:
        case BHY2_SENSOR_ID_GAS_WU:
            scaling = 1.0f;
            break;
        case BHY2_SENSOR_ID_LIGHT:
        case BHY2_SENSOR_ID_LIGHT_WU:
            scaling = 10000.0f / 216.0f;
            break;
        case BHY2_SENSOR_ID_PROX:
        case BHY2_SENSOR_ID_PROX_WU:
            scaling = 1.0f;
            break;
        case BHY2_SENSOR_ID_SI_ACCEL:
        case BHY2_SENSOR_ID_SI_GYROS:

            /* Scaling factor already applied in firmware*/
            break;
        case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
            scaling = 1.0f / 16384.0f; /*2^14 -> 16384*/
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
            scaling = 360.0f / 32768.0f; /*2^15 -> 32768*/
            break;
        default:
            scaling = -1.0f; /* Do not apply the scaling factor */
    }

    return scaling;
}

char *get_sensor_parse_format(uint8_t sensor_id)
{
    char *ret;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC_PASS:
        case BHY2_SENSOR_ID_ACC_RAW:
        case BHY2_SENSOR_ID_ACC:
        case BHY2_SENSOR_ID_ACC_BIAS:
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
        case BHY2_SENSOR_ID_ACC_WU:
        case BHY2_SENSOR_ID_ACC_RAW_WU:
        case BHY2_SENSOR_ID_GYRO_PASS:
        case BHY2_SENSOR_ID_GYRO_RAW:
        case BHY2_SENSOR_ID_GYRO:
        case BHY2_SENSOR_ID_GYRO_BIAS:
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
        case BHY2_SENSOR_ID_GYRO_WU:
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
        case BHY2_SENSOR_ID_MAG_PASS:
        case BHY2_SENSOR_ID_MAG_RAW:
        case BHY2_SENSOR_ID_MAG:
        case BHY2_SENSOR_ID_MAG_BIAS:
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
        case BHY2_SENSOR_ID_MAG_WU:
        case BHY2_SENSOR_ID_MAG_RAW_WU:
        case BHY2_SENSOR_ID_GRA:
        case BHY2_SENSOR_ID_GRA_WU:
        case BHY2_SENSOR_ID_LACC:
        case BHY2_SENSOR_ID_LACC_WU:
            ret = "s16,s16,s16";
            break;
        case BHY2_SENSOR_ID_RV:
        case BHY2_SENSOR_ID_RV_WU:
        case BHY2_SENSOR_ID_GAMERV:
        case BHY2_SENSOR_ID_GAMERV_WU:
        case BHY2_SENSOR_ID_GEORV:
        case BHY2_SENSOR_ID_GEORV_WU:
            ret = "s16,s16,s16,s16,u16";
            break;
        case BHY2_SENSOR_ID_ORI:
        case BHY2_SENSOR_ID_ORI_WU:
            ret = "s16,s16,s16";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
        case BHY2_SENSOR_ID_HUM:
        case BHY2_SENSOR_ID_HUM_WU:
        case BHY2_SENSOR_ID_PROX:
        case BHY2_SENSOR_ID_PROX_WU:
        case BHY2_SENSOR_ID_EXCAMERA:
        case BHI3_SENSOR_ID_MULTI_TAP:
            ret = "u8";
            break;
        case BHY2_SENSOR_ID_TEMP:
        case BHY2_SENSOR_ID_TEMP_WU:
            ret = "s16";
            break;
        case BHY2_SENSOR_ID_BARO:
        case BHY2_SENSOR_ID_BARO_WU:
            ret = "u24";
            break;
        case BHY2_SENSOR_ID_GAS:
        case BHY2_SENSOR_ID_GAS_WU:
        case BHY2_SENSOR_ID_STC:
        case BHY2_SENSOR_ID_STC_WU:
        case BHY2_SENSOR_ID_STC_LP:
        case BHY2_SENSOR_ID_STC_LP_WU:
            ret = "u32";
            break;
        case BHY2_SENSOR_ID_KLIO:
            ret = "u8,u8,u8,u8,u8,u8,f";
            break;
        case BHY2_SENSOR_ID_SWIM:
            ret = "u16,u16,u16,u16,u16,u16,u16";
            break;
        case BHY2_SENSOR_ID_SI_ACCEL:
        case BHY2_SENSOR_ID_SI_GYROS:
            ret = "f,f,f";
            break;
        case BHY2_SENSOR_ID_LIGHT:
        case BHY2_SENSOR_ID_LIGHT_WU:
            ret = "s16";
            break;
        case BHY2_SENSOR_ID_SIG:
        case BHY2_SENSOR_ID_STD:
        case BHY2_SENSOR_ID_STD_WU:
        case BHY2_SENSOR_ID_TILT_DETECTOR:
        case BHY2_SENSOR_ID_WAKE_GESTURE:
        case BHY2_SENSOR_ID_GLANCE_GESTURE:
        case BHY2_SENSOR_ID_PICKUP_GESTURE:
        case BHY2_SENSOR_ID_SIG_LP:
        case BHY2_SENSOR_ID_SIG_LP_WU:
        case BHY2_SENSOR_ID_STD_LP:
        case BHY2_SENSOR_ID_STD_LP_WU:
        case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
        case BHY2_SENSOR_ID_STATIONARY_DET:
        case BHY2_SENSOR_ID_ANY_MOTION_LP:
        case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
        case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
        case BHY2_SENSOR_ID_MOTION_DET:
        case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
            ret = "";
            break;
        case BHY2_SENSOR_ID_AR:
        case BHI3_SENSOR_ID_AR_WEAR_WU:
            ret = "u16";
            break;
        case BHY2_SENSOR_ID_GPS:
            ret = "st";
            break;
        case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            ret = "u8";
            break;
        case BHY2_SENSOR_ID_AIR_QUALITY:
            ret = "f32,f32,f32,f32,f32,f32,f32,u8";
            break;
        case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
            ret = "s16,s16,s16,s16,u8";
            break;

        case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
            ret = "s16,s16,s16,u8";
            break;
        default:
            ret = "";
    }

    return ret;
}

char *get_sensor_axis_names(uint8_t sensor_id)
{
    char *ret;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC_PASS:
        case BHY2_SENSOR_ID_ACC_RAW:
        case BHY2_SENSOR_ID_ACC:
        case BHY2_SENSOR_ID_ACC_BIAS:
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
        case BHY2_SENSOR_ID_ACC_WU:
        case BHY2_SENSOR_ID_ACC_RAW_WU:
        case BHY2_SENSOR_ID_GYRO_PASS:
        case BHY2_SENSOR_ID_GYRO_RAW:
        case BHY2_SENSOR_ID_GYRO:
        case BHY2_SENSOR_ID_GYRO_BIAS:
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
        case BHY2_SENSOR_ID_GYRO_WU:
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
        case BHY2_SENSOR_ID_MAG_PASS:
        case BHY2_SENSOR_ID_MAG_RAW:
        case BHY2_SENSOR_ID_MAG:
        case BHY2_SENSOR_ID_MAG_BIAS:
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
        case BHY2_SENSOR_ID_MAG_WU:
        case BHY2_SENSOR_ID_MAG_RAW_WU:
        case BHY2_SENSOR_ID_GRA:
        case BHY2_SENSOR_ID_GRA_WU:
        case BHY2_SENSOR_ID_LACC:
        case BHY2_SENSOR_ID_LACC_WU:
        case BHY2_SENSOR_ID_SI_ACCEL:
        case BHY2_SENSOR_ID_SI_GYROS:
            ret = "x,y,z";
            break;
        case BHY2_SENSOR_ID_RV:
        case BHY2_SENSOR_ID_RV_WU:
        case BHY2_SENSOR_ID_GAMERV:
        case BHY2_SENSOR_ID_GAMERV_WU:
        case BHY2_SENSOR_ID_GEORV:
        case BHY2_SENSOR_ID_GEORV_WU:
            ret = "x,y,z,w,ar";
            break;
        case BHY2_SENSOR_ID_ORI:
        case BHY2_SENSOR_ID_ORI_WU:
            ret = "h,p,r";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
            ret = "o";
            break;
        case BHY2_SENSOR_ID_TEMP:
        case BHY2_SENSOR_ID_TEMP_WU:
            ret = "t";
            break;
        case BHY2_SENSOR_ID_BARO:
        case BHY2_SENSOR_ID_BARO_WU:
            ret = "p";
            break;
        case BHY2_SENSOR_ID_HUM:
        case BHY2_SENSOR_ID_HUM_WU:
            ret = "h";
            break;
        case BHY2_SENSOR_ID_GAS:
        case BHY2_SENSOR_ID_GAS_WU:
            ret = "g";
            break;
        case BHY2_SENSOR_ID_KLIO:
            ret = "lin,lid,lpr,lcr,rin,rid,rc";
            break;
        case BHY2_SENSOR_ID_SWIM:
            ret = "d,lc,f,br,bu,ba,sc";
            break;
        case BHY2_SENSOR_ID_LIGHT:
        case BHY2_SENSOR_ID_LIGHT_WU:
            ret = "l";
            break;
        case BHY2_SENSOR_ID_PROX:
        case BHY2_SENSOR_ID_PROX_WU:
            ret = "p";
            break;
        case BHY2_SENSOR_ID_STC:
        case BHY2_SENSOR_ID_STC_WU:
        case BHY2_SENSOR_ID_STC_LP:
        case BHY2_SENSOR_ID_STC_LP_WU:
        case BHY2_SENSOR_ID_EXCAMERA:
            ret = "c";
            break;
        case BHY2_SENSOR_ID_SIG:
        case BHY2_SENSOR_ID_STD:
        case BHY2_SENSOR_ID_STD_WU:
        case BHY2_SENSOR_ID_TILT_DETECTOR:
        case BHY2_SENSOR_ID_WAKE_GESTURE:
        case BHY2_SENSOR_ID_GLANCE_GESTURE:
        case BHY2_SENSOR_ID_PICKUP_GESTURE:
        case BHY2_SENSOR_ID_SIG_LP:
        case BHY2_SENSOR_ID_SIG_LP_WU:
        case BHY2_SENSOR_ID_STD_LP:
        case BHY2_SENSOR_ID_STD_LP_WU:
        case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
        case BHY2_SENSOR_ID_STATIONARY_DET:
        case BHY2_SENSOR_ID_ANY_MOTION_LP:
        case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
        case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
        case BHY2_SENSOR_ID_MOTION_DET:
        case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
            ret = "e";
            break;
        case BHY2_SENSOR_ID_AR:
        case BHI3_SENSOR_ID_AR_WEAR_WU:
            ret = "a";
            break;
        case BHY2_SENSOR_ID_GPS:
            ret = "g";
            break;
        case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            ret = "wrist_gesture";
            break;
        case BHI3_SENSOR_ID_MULTI_TAP:
            ret = "taps";
            break;
        case BHY2_SENSOR_ID_AIR_QUALITY:
            ret = "t,h,g,i,si,c,v,a";
            break;
        case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
            ret = "x,y,z,w,a";
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
            ret = "h,p,r,a";
            break;
        default:
            ret = "";
    }

    return ret;
}

char *get_klio_error(bhy2_klio_driver_error_state_t error)
{
    char *ret = "";

    switch (error)
    {
        case KLIO_DRIVER_ERROR_NONE:
            break;
        case KLIO_DRIVER_ERROR_INVALID_PARAMETER:
            ret = "[Klio error] Invalid parameter";
            break;
        case KLIO_DRIVER_ERROR_PARAMETER_OUT_OF_RANGE:
            ret = "[Klio error] Parameter out of range";
            break;
        case KLIO_DRIVER_ERROR_INVALID_PATTERN_OPERATION:
            ret = "[Klio error] Invalid pattern operation";
            break;
        case KLIO_DRIVER_ERROR_NOT_IMPLEMENTED:
            ret = "[Klio error] Not implemented";
            break;
        case KLIO_DRIVER_ERROR_BUFSIZE:
            ret = "[Klio error] Buffer size";
            break;
        case KLIO_DRIVER_ERROR_INTERNAL:
            ret = "[Klio error] Internal";
            break;
        case KLIO_DRIVER_ERROR_UNDEFINED:
            ret = "[Klio error] Undefined";
            break;
        case KLIO_DRIVER_ERROR_OPERATION_PENDING:
            ret = "[Klio error] Operation pending";
            break;
        default:
            ret = "[Klio error] Unknown error code";
    }

    return ret;
}