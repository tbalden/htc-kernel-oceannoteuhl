/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */



#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include "../stmvl53l0-i2c.h"
#include "../stmvl53l0-cci.h"

#include "vl53l0_platform.h"
#include "vl53l0_i2c_platform.h"
#include "vl53l0_def.h"

#include "vl53l0_platform_log.h"

#ifdef VL53L0_LOG_ENABLE
#define trace_print(level, ...) \
	trace_print_module_function(TRACE_MODULE_PLATFORM, level,\
	TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) \
	trace_print_module_function(TRACE_MODULE_NONE, \
	TRACE_LEVEL_NONE, TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif

#define I2C_BUFFER_CONFIG 1

#if I2C_BUFFER_CONFIG == 0
    
	uint8_t i2c_global_buffer[VL53L0_MAX_I2C_XFER_SIZE];

    #define DECL_I2C_BUFFER
    #define VL53L0_GetLocalBuffer(Dev, n_byte)  i2c_global_buffer

#elif I2C_BUFFER_CONFIG == 1
    
	uint8_t LocBuffer[VL53L0_MAX_I2C_XFER_SIZE];
    #define VL53L0_GetLocalBuffer(Dev, n_byte)  LocBuffer
#elif I2C_BUFFER_CONFIG == 2
    #define DECL_I2C_BUFFER
#else
#error "invalid I2C_BUFFER_CONFIG "
#endif


#define VL53L0_I2C_USER_VAR         
#define VL53L0_GetI2CAccess(Dev)    
#define VL53L0_DoneI2CAcces(Dev)    


char  debug_string[VL53L0_MAX_STRING_LENGTH_PLT];


#define MIN_COMMS_VERSION_MAJOR     1
#define MIN_COMMS_VERSION_MINOR     8
#define MIN_COMMS_VERSION_BUILD     1
#define MIN_COMMS_VERSION_REVISION  0

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

bool_t _check_min_version(void)
{
	bool_t min_version_comms_dll = false;

	min_version_comms_dll = true;

	return min_version_comms_dll;
}

int32_t VL53L0_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz)
{
	int32_t status   = STATUS_OK;

	return status;
}

int32_t VL53L0_comms_close(void)
{
	int32_t status = STATUS_OK;


	return status;
}

int32_t VL53L0_set_page(VL53L0_DEV dev, uint8_t page_data)
{
	int32_t status = STATUS_OK;
	uint16_t page_index = 0xFF;
	uint8_t *buffer;

	buffer =  VL53L0_GetLocalBuffer(dev, 3);
	buffer[0] = page_index >> 8;
	buffer[1] = page_index & 0xff;
	buffer[2] = page_data;

	status = VL53L0_I2CWrite(dev, buffer, (uint8_t) 3);
	return status;
}

int32_t VL53L0_write_multi(VL53L0_DEV dev, uint8_t index, uint8_t *pdata,
			int32_t count)
{
	int32_t status = STATUS_OK;
	uint8_t *buffer;

#ifdef VL53L0_LOG_ENABLE
	int32_t i = 0;
	char value_as_str[VL53L0_MAX_STRING_LENGTH_PLT];
	char *pvalue_as_str;

	pvalue_as_str =  value_as_str;

	for (i = 0 ; i < count ; i++) {
		snprintf(pvalue_as_str, sizeof(pvalue_as_str),
			"%02X", *(pdata + i));

		pvalue_as_str += 2;
	}
	trace_i2c("Write reg : 0x%04X, Val : 0x%s\n", index, value_as_str);
#endif
	if ((count + 1) > VL53L0_MAX_I2C_XFER_SIZE)
		return STATUS_FAIL;
	buffer =  VL53L0_GetLocalBuffer(dev, (count+1));
	buffer[0] = index;
	memcpy(&buffer[1], pdata, count);
	status = VL53L0_I2CWrite(dev, buffer, (count+1));

	return status;
}

int32_t VL53L0_read_multi(VL53L0_DEV dev, uint8_t index, uint8_t *pdata,
			int32_t count)
{
	int32_t status = STATUS_OK;
	uint8_t *buffer;

#ifdef VL53L0_LOG_ENABLE
	int32_t      i = 0;
	char   value_as_str[VL53L0_MAX_STRING_LENGTH_PLT];
	char *pvalue_as_str;
#endif

	if ((count + 1) > VL53L0_MAX_I2C_XFER_SIZE)
		return STATUS_FAIL;

	buffer =  VL53L0_GetLocalBuffer(dev, 1);
	buffer[0] = index;
	status = VL53L0_I2CWrite(dev, (uint8_t *)buffer, (uint8_t)1);
	if (!status) {
		pdata[0] = index;
		status = VL53L0_I2CRead(dev, pdata, count);
	}

#ifdef VL53L0_LOG_ENABLE
	pvalue_as_str =  value_as_str;

	for (i = 0 ; i < count ; i++) {
		snprintf(pvalue_as_str, sizeof(value_as_str),
			"%02X", *(pdata+i));
		pvalue_as_str += 2;
	}

	trace_i2c("Read  reg : 0x%04X, Val : 0x%s\n", index, value_as_str);
#endif

	return status;
}


int32_t VL53L0_write_byte(VL53L0_DEV dev, uint8_t index, uint8_t data)
{
	int32_t status = STATUS_OK;
	const int32_t cbyte_count = 1;

	status = VL53L0_write_multi(dev, index, &data, cbyte_count);

	return status;

}


int32_t VL53L0_write_word(VL53L0_DEV dev, uint8_t index, uint16_t data)
{
	int32_t status = STATUS_OK;

	uint8_t  buffer[BYTES_PER_WORD];

	
	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data &  0x00FF);

	status = VL53L0_write_multi(dev, index, buffer, BYTES_PER_WORD);

	return status;

}


int32_t VL53L0_write_dword(VL53L0_DEV dev, uint8_t index, uint32_t data)
{
	int32_t status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_DWORD];

	
	buffer[0] = (uint8_t) (data >> 24);
	buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (data &  0x000000FF);

	status = VL53L0_write_multi(dev, index, buffer, BYTES_PER_DWORD);

	return status;

}


int32_t VL53L0_read_byte(VL53L0_DEV dev, uint8_t index, uint8_t *pdata)
{
	int32_t status = STATUS_OK;
	int32_t cbyte_count = 1;

	status = VL53L0_read_multi(dev, index, pdata, cbyte_count);

	return status;

}


int32_t VL53L0_read_word(VL53L0_DEV dev, uint8_t index, uint16_t *pdata)
{
	int32_t  status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_WORD];

	status = VL53L0_read_multi(dev, index, buffer, BYTES_PER_WORD);
	*pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

	return status;

}

int32_t VL53L0_read_dword(VL53L0_DEV dev, uint8_t index, uint32_t *pdata)
{
	int32_t status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_DWORD];

	status = VL53L0_read_multi(dev, index, buffer, BYTES_PER_DWORD);
	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) +
			((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

	return status;

}

int32_t VL53L0_platform_wait_us(int32_t wait_us)
{
	int32_t status = STATUS_OK;

	msleep((wait_us/1000));

#ifdef VL53L0_LOG_ENABLE
	trace_i2c("Wait us : %6d\n", wait_us);
#endif

	return status;

}


int32_t VL53L0_wait_ms(int32_t wait_ms)
{
	int32_t status = STATUS_OK;

	msleep(wait_ms);

#ifdef VL53L0_LOG_ENABLE
	trace_i2c("Wait ms : %6d\n", wait_ms);
#endif

	return status;

}


int32_t VL53L0_set_gpio(uint8_t level)
{
	int32_t status = STATUS_OK;
#ifdef VL53L0_LOG_ENABLE
	trace_i2c("// Set GPIO = %d;\n", level);
#endif

	return status;

}


int32_t VL53L0_get_gpio(uint8_t *plevel)
{
	int32_t status = STATUS_OK;
#ifdef VL53L0_LOG_ENABLE
	trace_i2c("// Get GPIO = %d;\n", *plevel);
#endif
	return status;
}


int32_t VL53L0_release_gpio(void)
{
	int32_t status = STATUS_OK;
#ifdef VL53L0_LOG_ENABLE
	trace_i2c("// Releasing force on GPIO\n");
#endif
	return status;

}

int32_t VL53L0_cycle_power(void)
{
	int32_t status = STATUS_OK;
#ifdef VL53L0_LOG_ENABLE
	trace_i2c("// cycle sensor power\n");
#endif

	return status;
}


int32_t VL53L0_get_timer_frequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 0;
	return STATUS_FAIL;
}


int32_t VL53L0_get_timer_value(int32_t *ptimer_count)
{
	*ptimer_count = 0;
	return STATUS_FAIL;
}
