/*
MIT License

Copyright (c) 2024 VoltBro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "bosch_imu.h"
#include "common.h"
#include "main.h"

#include "bhy2_parse.h"

enum bhy2_intf intf;

#define WORK_BUFFER_SIZE  2048

// pointer to log output function
extern custom_printf_prt my_printf;

void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_accell(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_gyro(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

uint8_t product_id = 0;
uint16_t version = 0;
int8_t rslt;
struct bhy2_dev bhy2;
uint8_t work_buffer[WORK_BUFFER_SIZE];
uint8_t hintr_ctrl, hif_ctrl, boot_status;
uint8_t accuracy; // Accuracy is reported as a meta event. It is being printed alongside the data

void bosch_imu_setup(void)
{
  intf = BHY2_I2C_INTERFACE;

  rslt = bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2);
  print_api_error(rslt, &bhy2);
  
  rslt = bhy2_soft_reset(&bhy2);
  print_api_error(rslt, &bhy2);
  
  rslt = bhy2_get_product_id(&product_id, &bhy2);
  print_api_error(rslt, &bhy2);
  
  // Check for a valid product ID 
  if (product_id != BHY2_PRODUCT_ID)
  {
    my_printf("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
  }
  else
  {
    my_printf("BHIx60/BHAx60 found. Product ID read %X\r\n", product_id);
  }
  
  // Check the interrupt pin and FIFO configurations. Disable status and debug
  hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
  
  rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
  print_api_error(rslt, &bhy2);
  rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &bhy2);
  print_api_error(rslt, &bhy2);
  
  // Configure the host interface 
  hif_ctrl = 0;
  rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
  print_api_error(rslt, &bhy2);
  
  // Check if the sensor is ready to load firmware 
  rslt = bhy2_get_boot_status(&boot_status, &bhy2);
  print_api_error(rslt, &bhy2);  
  
  if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
  {
    upload_firmware(boot_status, &bhy2);
    
    rslt = bhy2_get_kernel_version(&version, &bhy2);
    print_api_error(rslt, &bhy2);
    if ((rslt == BHY2_OK) && (version != 0))
    {
      my_printf("Boot successful. Kernel version %u.\r\n", version);
    }
    
    rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy, &bhy2);
    print_api_error(rslt, &bhy2);
    rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&accuracy, &bhy2);
    print_api_error(rslt, &bhy2);
    
    rslt = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ORI, parse_euler, (void*)&accuracy, &bhy2);
    print_api_error(rslt, &bhy2);
    
    rslt = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, parse_accell, (void*)&accuracy, &bhy2);
    print_api_error(rslt, &bhy2);    
    
    rslt = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO, parse_gyro, (void*)&accuracy, &bhy2);
    print_api_error(rslt, &bhy2);       
    
    rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
    print_api_error(rslt, &bhy2);
  }
  else
  {
    my_printf("Host interface not ready. Exiting\r\n");

    Error_Handler();
  }  
  
  rslt = bhy2_update_virtual_sensor_list(&bhy2);
  print_api_error(rslt, &bhy2);  
  
  printInfo(&bhy2);
  
  float sample_rate = 100.0; // Read out data measured at 100Hz
  uint32_t report_latency_ms = 0; // Report immediately
  
  rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ORI, sample_rate, report_latency_ms, &bhy2);
  print_api_error(rslt, &bhy2);
  my_printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHY2_SENSOR_ID_ORI), sample_rate);
  
  rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, sample_rate, report_latency_ms, &bhy2);
  print_api_error(rslt, &bhy2);
  my_printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHY2_SENSOR_ID_ACC), sample_rate);  
  
  rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, sample_rate, report_latency_ms, &bhy2);
  print_api_error(rslt, &bhy2);
  my_printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHY2_SENSOR_ID_GYRO), sample_rate); 
}

void bosch_imu_run(void)
{
  rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
  print_api_error(rslt, &bhy2);
}

extern uint64_t last_result;
extern uint8_t sensor_reports;

float Heading = 0.0f;
float Pitch = 0.0f;
float Roll = 0.0f;
void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  (void)callback_ref;

  struct bhy2_data_orientation data;
  uint32_t s, ns;
  uint8_t *accuracy = (uint8_t*)callback_ref;
  if (callback_info->data_size != 7) // Check for a valid payload size. Includes sensor ID
  {
    return;
  }
  
  bhy2_parse_orientation(callback_info->data_ptr, &data);
  
  uint64_t timestamp = *callback_info->time_stamp; // Store the last timestamp
  
  timestamp = timestamp * 15625; // Timestamp is now in nanoseconds
  s = (uint32_t)(timestamp / UINT64_C(1000000000));
  ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));
  
  if (accuracy)
  {
    Heading = data.heading * 360.0f / 32768.0f;
    Pitch = data.pitch * 360.0f / 32768.0f;
    Roll = data.roll * 360.0f / 32768.0f;
  }
  else
  {
    Heading = data.heading * 360.0f / 32768.0f;
    Pitch = data.pitch * 360.0f / 32768.0f;
    Roll = data.roll * 360.0f / 32768.0f;
  }

  sensor_reports |= 1UL << 0; // register euler sensor callback

  last_result = micros();
}

float acc_x = 0.0f;
float acc_y = 0.0f;
float acc_z = 0.0f;
void parse_accell(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  (void)callback_ref;

  struct bhy2_data_xyz data;
  uint32_t s, ns;
  uint8_t *accuracy = (uint8_t*)callback_ref;
  if (callback_info->data_size != 7) // Check for a valid payload size. Includes sensor ID
  {
    return;
  }
  
  bhy2_parse_xyz(callback_info->data_ptr, &data);
  
  uint64_t timestamp = *callback_info->time_stamp; // Store the last timestamp
  
  timestamp = timestamp * 15625; // Timestamp is now in nanoseconds
  s = (uint32_t)(timestamp / UINT64_C(1000000000));
  ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));
  
  if (accuracy)
  {
    acc_x = data.x * 1.0f / 4096.0f;
    acc_y = data.y * 1.0f / 4096.0f;
    acc_z = data.z * 1.0f / 4096.0f;
  }
  else
  {
    acc_x = data.x * 1.0f / 4096.0f;
    acc_y = data.y * 1.0f / 4096.0f;
    acc_z = data.z * 1.0f / 4096.0f;
  }

  sensor_reports |= 1UL << 1; // register accelerometer sensor callback
  
  last_result = micros();
}

float gyr_x = 0.0f;
float gyr_y = 0.0f;
float gyr_z = 0.0f;
void parse_gyro(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  (void)callback_ref;

  struct bhy2_data_xyz data;
  uint32_t s, ns;
  uint8_t *accuracy = (uint8_t*)callback_ref;
  if (callback_info->data_size != 7) // Check for a valid payload size. Includes sensor ID
  {
    return;
  }
  
  bhy2_parse_xyz(callback_info->data_ptr, &data);
  
  uint64_t timestamp = *callback_info->time_stamp; // Store the last timestamp
  
  timestamp = timestamp * 15625; // Timestamp is now in nanoseconds
  s = (uint32_t)(timestamp / UINT64_C(1000000000));
  ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));
  
  if (accuracy)
  {
    gyr_x = data.x * 2000.0f / 32768.0f;
    gyr_y = data.y * 2000.0f / 32768.0f;
    gyr_z = data.z * 2000.0f / 32768.0f;
  }
  else
  {
    gyr_x = data.x * 2000.0f / 32768.0f;
    gyr_y = data.y * 2000.0f / 32768.0f;
    gyr_z = data.z * 2000.0f / 32768.0f;
  }

  sensor_reports |= 1UL << 2; // register gyro sensor callback
  
  last_result = micros();
}