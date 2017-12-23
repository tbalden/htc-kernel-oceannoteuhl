/*
 *  vl53l0_i2c_platform.h - Linux kernel modules for STM VL53L0 FlightSense TOF
 *							sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */



#ifndef _VL53L0_I2C_PLATFORM_H_
#define _VL53L0_I2C_PLATFORM_H_

#include "vl53l0_def.h"


/** Maximum buffer size to be used in i2c */
#define VL53L0_MAX_I2C_XFER_SIZE   64



#ifndef bool_t
typedef unsigned char bool_t;
#endif


#define	   I2C                0x01
#define	   SPI                0x00

#define    COMMS_BUFFER_SIZE    64

#define    BYTES_PER_WORD        2
#define    BYTES_PER_DWORD       4

#define    VL53L0_MAX_STRING_LENGTH_PLT       256


int32_t VL53L0_comms_initialise(uint8_t  comms_type,
			uint16_t comms_speed_khz);


int32_t VL53L0_comms_close(void);


int32_t VL53L0_cycle_power(void);

int32_t VL53L0_set_page(VL53L0_DEV dev, uint8_t page_data);

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * Wrapper for SystemVerilog Write Multi task
 *
 * @code
 *
 * Example:
 *
 * uint8_t *spad_enables;
 *
 * int status = VL53L0_write_multi(RET_SPAD_EN_0, spad_enables, 36);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to uint8_t buffer containing the data to be written
 * @param  count - number of bytes in the supplied byte buffer
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0_write_multi(VL53L0_DEV dev, uint8_t index, uint8_t  *pdata,
		int32_t count);



int32_t VL53L0_read_multi(VL53L0_DEV dev,  uint8_t index, uint8_t  *pdata,
		int32_t count);



int32_t VL53L0_write_byte(VL53L0_DEV dev,  uint8_t index, uint8_t   data);


/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the
 * MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint16_t nvm_ctrl_pulse_width = 0x0004;
 *
 * int status = VL53L0_write_word(NVM_CTRL__PULSE_WIDTH_MSB,
 * nvm_ctrl_pulse_width);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uin16_t data value write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0_write_word(VL53L0_DEV dev,  uint8_t index, uint16_t  data);


/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the
 * MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint32_t nvm_data = 0x0004;
 *
 * int status = VL53L0_write_dword(NVM_CTRL__DATAIN_MMM, nvm_data);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uint32_t data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0_write_dword(VL53L0_DEV dev, uint8_t index, uint32_t  data);




int32_t VL53L0_read_byte(VL53L0_DEV dev,  uint8_t index, uint8_t  *pdata);



int32_t VL53L0_read_word(VL53L0_DEV dev,  uint8_t index, uint16_t *pdata);



int32_t VL53L0_read_dword(VL53L0_DEV dev, uint8_t index, uint32_t *pdata);



int32_t VL53L0_platform_wait_us(int32_t wait_us);



int32_t VL53L0_wait_ms(int32_t wait_ms);



int32_t VL53L0_set_gpio(uint8_t  level);



int32_t VL53L0_get_gpio(uint8_t *plevel);


int32_t VL53L0_release_gpio(void);



int32_t VL53L0_get_timer_frequency(int32_t *ptimer_freq_hz);


int32_t VL53L0_get_timer_value(int32_t *ptimer_count);
int VL53L0_I2CWrite(VL53L0_DEV dev, uint8_t *buff, uint8_t len);
int VL53L0_I2CRead(VL53L0_DEV dev, uint8_t *buff, uint8_t len);

#endif 

