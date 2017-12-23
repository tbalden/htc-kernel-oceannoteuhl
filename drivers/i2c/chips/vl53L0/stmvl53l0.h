/*
 *  stmvl53l0.h - Linux kernel modules for STM VL53L0 FlightSense TOF sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division
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
#ifndef STMVL53L0_H
#define STMVL53L0_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>

#define STMVL53L0_DRV_NAME	"stmvl53l0"
#define STMVL53L0_SLAVE_ADDR	(0x52>>1)

#define DRIVER_VERSION		"1.0.5.1"
#define CALIBRATED          0x67
#define XTALK_CALIBRATED    0x01
#define OFFSET_CALIBRATED   0x10
#define REF_SPAD_CALIBRATED 0x20

#define I2C_M_WR			0x00

#define DEBUG
#define vl53l0_dbgmsg(str, args...)	\
	pr_err("[LASER] %s: " str, __func__, ##args)
#define vl53l0_errmsg(str, args...) \
	pr_err("[LASER] %s: " str, __func__, ##args)

#define timing_dbgmsg(str, args...)	\
	pr_err("[LASER][T_DEBUG] %s: " str, __func__, ##args)

#define E(x...) pr_err("[LASER] " x)
#define D(x...) pr_debug("[LASER] " x)
#define I(x...) pr_info("[LASER] " x)
#define W(x...) pr_warn("[LASER] " x)

#define VL53L0_VDD_MIN      2600000
#define VL53L0_VDD_MAX      3000000
#define HTC
typedef enum {
	NORMAL_MODE = 0,
	OFFSETCALIB_MODE = 1,
	XTALKCALIB_MODE = 2,
} init_mode_e;

typedef enum {
	OFFSET_PAR = 0,
	XTALKRATE_PAR = 1,
	XTALKENABLE_PAR = 2,
	GPIOFUNC_PAR = 3,
	LOWTHRESH_PAR = 4,
	HIGHTHRESH_PAR = 5,
	DEVICEMODE_PAR = 6,
	INTERMEASUREMENT_PAR = 7,
	REFERENCESPADS_PAR = 8,
	REFCALIBRATION_PAR = 9,
} parameter_name_e;

enum {
	CCI_BUS = 0,
	I2C_BUS = 1,
};

struct stmvl53l0_register {
	uint32_t is_read; 
	uint32_t reg_index;
	uint32_t reg_bytes;
	uint32_t reg_data;
	int32_t status;
};

struct stmvl53l0_parameter {
	uint32_t is_read; 
	parameter_name_e name;
	int32_t value;
	int32_t value2;
	int32_t status;
};

struct stmvl53l0_custom_use_case {
    FixPoint1616_t  signalRateLimit;
    FixPoint1616_t  sigmaLimit;
    uint32_t        preRangePulsePeriod;
    uint32_t        finalRangePulsePeriod;
    uint32_t        timingBudget;
};

struct stmvl53l0_data {

	VL53L0_DevData_t Data;	
	uint8_t   I2cDevAddr;	
	uint8_t   comms_type;	
	uint16_t  comms_speed_khz;	
	uint8_t   bus_type;		

	void *client_object; 

	struct mutex update_lock;
	struct delayed_work	dwork;		
	struct input_dev *input_dev_ps;
	struct kobject *range_kobj;

	const char *dev_name;
	

	
	struct miscdevice miscdev;

	int irq;
	unsigned int reset;

	
	unsigned int enable_ps_sensor;

	
	
	
	unsigned int ps_data;			

	
	unsigned int offsetCalDistance;
	unsigned int xtalkCalDistance;

    
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    int32_t OffsetMicroMeter;
    FixPoint1616_t XTalkCompensationRateMegaCps;
    uint32_t  setCalibratedValue;

#ifdef HTC
    
    int offset_kvalue;
    FixPoint1616_t xtalk_kvalue;
#endif

    
    FixPoint1616_t signalRateLimit;
    FixPoint1616_t sigmaLimit;
    uint32_t        preRangePulsePeriod;
    uint32_t        finalRangePulsePeriod;

	
	VL53L0_RangingMeasurementData_t rangeData;

	
	VL53L0_DeviceModes 			deviceMode;
	uint32_t					interMeasurems;
	VL53L0_GpioFunctionality gpio_function;
	VL53L0_InterruptPolarity gpio_polarity;
	FixPoint1616_t low_threshold;
	FixPoint1616_t high_threshold;

	
	uint8_t delay_ms;

	
	uint32_t 	   timingBudget;
	
	uint32_t       noInterruptCount;
    
    uint8_t         useCase;
    
    uint8_t         updateUseCase;

	
	struct task_struct *poll_thread;
	
	wait_queue_head_t poll_thread_wq;

	
	uint32_t		interruptStatus;
	struct mutex work_mutex;

    struct timer_list timer;
    uint32_t flushCount;

	
	unsigned int enableDebug;
	unsigned int enableTimingDebug;
	uint8_t interrupt_received;

#ifdef HTC
    
    u32 pwdn_gpio;
    u32 power_gpio;
    u32 laser_irq_gpio;
    struct regulator *camio_1v8;
    struct regulator *power_2v8;
    struct pinctrl *pinctrl;
    struct pinctrl_state *gpio_state_init;
    struct device *sensor_dev;
    struct class *laser_class;
    struct device *laser_dev;
    bool laser_power;
    FixPoint1616_t cali_distance;
    u8 cali_status;
#endif
};

struct stmvl53l0_module_fn_t {
	int (*init)(void);
	void (*deinit)(void *);
	int (*power_up)(void *, unsigned int *);
	int (*power_down)(void *);
};



int stmvl53l0_setup(struct stmvl53l0_data *data);
void stmvl53l0_cleanup(struct stmvl53l0_data *data);

#endif 
