/*******************************************************************************
Copyright © 2016, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/



#ifndef _VL53L010_API_H_
#define _VL53L010_API_H_

#include "vl53l010_device.h"
#include "vl53l010_strings.h"
#include "vl53l0_def.h"
#include "vl53l0_platform.h"


#ifdef __cplusplus
extern "C" {
#endif


#ifdef _MSC_VER
#   ifdef VL53L0_API_EXPORTS
#       define VL53L010_API  __declspec(dllexport)
#   else
#       define VL53L010_API
#   endif
#else
#   define VL53L010_API
#endif




VL53L010_API VL53L0_Error VL53L010_GetVersion(VL53L0_Version_t *pVersion);

VL53L010_API VL53L0_Error VL53L010_GetPalSpecVersion(
			VL53L0_Version_t *pPalSpecVersion);


VL53L010_API VL53L0_Error VL53L010_GetDeviceInfo(VL53L0_DEV Dev,
			VL53L0_DeviceInfo_t *pVL53L0_DeviceInfo);


VL53L010_API VL53L0_Error VL53L010_GetDeviceErrorStatus(VL53L0_DEV Dev,
			VL53L010_DeviceError *pDeviceErrorStatus);

VL53L010_API VL53L0_Error VL53L010_GetDeviceErrorString(
			VL53L010_DeviceError ErrorCode, char *pDeviceErrorString);


VL53L010_API VL53L0_Error VL53L010_GetPalErrorString(VL53L0_Error PalErrorCode,
			char *pPalErrorString);


VL53L010_API VL53L0_Error VL53L010_GetPalState(VL53L0_DEV Dev,
			VL53L0_State *pPalState);


VL53L010_API VL53L0_Error VL53L010_SetPowerMode(VL53L0_DEV Dev,
			VL53L0_PowerModes PowerMode);

VL53L010_API VL53L0_Error VL53L010_GetPowerMode(VL53L0_DEV Dev,
			VL53L0_PowerModes *pPowerMode);


VL53L010_API VL53L0_Error VL53L010_SetOffsetCalibrationDataMicroMeter(
			VL53L0_DEV Dev,
			int32_t OffsetCalibrationDataMicroMeter);

VL53L010_API VL53L0_Error VL53L010_GetOffsetCalibrationDataMicroMeter(
			VL53L0_DEV Dev,
			int32_t *pOffsetCalibrationDataMicroMeter);

VL53L010_API VL53L0_Error VL53L010_SetGroupParamHold(VL53L0_DEV Dev,
			uint8_t GroupParamHold);

VL53L010_API VL53L0_Error VL53L010_GetUpperLimitMilliMeter(VL53L0_DEV Dev,
			uint16_t *pUpperLimitMilliMeter);




VL53L010_API VL53L0_Error VL53L010_SetDeviceAddress(VL53L0_DEV Dev,
			uint8_t DeviceAddress);

VL53L010_API VL53L0_Error VL53L010_DataInit(VL53L0_DEV Dev);

VL53L010_API VL53L0_Error VL53L010_StaticInit(VL53L0_DEV Dev);

VL53L010_API VL53L0_Error VL53L010_WaitDeviceBooted(VL53L0_DEV Dev);

VL53L010_API VL53L0_Error VL53L010_ResetDevice(VL53L0_DEV Dev);




VL53L010_API VL53L0_Error VL53L010_SetDeviceParameters(VL53L0_DEV Dev,
			const VL53L0_DeviceParameters_t *pDeviceParameters);

VL53L010_API VL53L0_Error VL53L010_GetDeviceParameters(VL53L0_DEV Dev,
			VL53L0_DeviceParameters_t *pDeviceParameters);

VL53L010_API VL53L0_Error VL53L010_SetDeviceMode(VL53L0_DEV Dev,
			VL53L0_DeviceModes DeviceMode);

VL53L010_API VL53L0_Error VL53L010_GetDeviceMode(VL53L0_DEV Dev,
			VL53L0_DeviceModes *pDeviceMode);

VL53L010_API VL53L0_Error VL53L010_SetHistogramMode(VL53L0_DEV Dev,
			VL53L0_HistogramModes HistogramMode);

VL53L010_API VL53L0_Error VL53L010_GetHistogramMode(VL53L0_DEV Dev,
			VL53L0_HistogramModes *pHistogramMode);

VL53L010_API VL53L0_Error VL53L010_SetMeasurementTimingBudgetMicroSeconds(
			VL53L0_DEV Dev,
			uint32_t MeasurementTimingBudgetMicroSeconds);

VL53L010_API VL53L0_Error VL53L010_GetMeasurementTimingBudgetMicroSeconds(
			VL53L0_DEV Dev,
			uint32_t *pMeasurementTimingBudgetMicroSeconds);

VL53L010_API VL53L0_Error VL53L010_SetInterMeasurementPeriodMilliSeconds(
			VL53L0_DEV Dev,
			uint32_t InterMeasurementPeriodMilliSeconds);

VL53L010_API VL53L0_Error VL53L010_GetInterMeasurementPeriodMilliSeconds(
			VL53L0_DEV Dev,
			uint32_t *pInterMeasurementPeriodMilliSeconds);

VL53L010_API VL53L0_Error VL53L010_SetXTalkCompensationEnable(
			VL53L0_DEV Dev, uint8_t XTalkCompensationEnable);

VL53L010_API VL53L0_Error VL53L010_GetXTalkCompensationEnable(
			VL53L0_DEV Dev, uint8_t *pXTalkCompensationEnable);

VL53L010_API VL53L0_Error VL53L010_SetXTalkCompensationRateMegaCps(
			VL53L0_DEV Dev,
			FixPoint1616_t XTalkCompensationRateMegaCps);

VL53L010_API VL53L0_Error VL53L010_GetXTalkCompensationRateMegaCps(
			VL53L0_DEV Dev,
			FixPoint1616_t *pXTalkCompensationRateMegaCps);


VL53L010_API VL53L0_Error VL53L010_GetNumberOfLimitCheck(
			uint16_t *pNumberOfLimitCheck);

VL53L010_API VL53L0_Error VL53L010_GetLimitCheckInfo(VL53L0_DEV Dev,
			uint16_t LimitCheckId, char *pLimitCheckString);


VL53L010_API VL53L0_Error VL53L010_SetLimitCheckEnable(VL53L0_DEV Dev,
			uint16_t LimitCheckId, uint8_t LimitCheckEnable);


VL53L010_API VL53L0_Error VL53L010_GetLimitCheckEnable(VL53L0_DEV Dev,
			uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);

VL53L010_API VL53L0_Error VL53L010_SetLimitCheckValue(VL53L0_DEV Dev,
			uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue);

VL53L010_API VL53L0_Error VL53L010_GetLimitCheckValue(VL53L0_DEV Dev,
			uint16_t LimitCheckId,
			FixPoint1616_t *pLimitCheckValue);


VL53L010_API VL53L0_Error VL53L010_GetLimitCheckCurrent(VL53L0_DEV Dev,
		uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);

VL53L010_API VL53L0_Error VL53L010_SetWrapAroundCheckEnable(VL53L0_DEV Dev,
			uint8_t WrapAroundCheckEnable);

VL53L010_API VL53L0_Error VL53L010_GetWrapAroundCheckEnable(VL53L0_DEV Dev,
			uint8_t *pWrapAroundCheckEnable);




VL53L010_API VL53L0_Error VL53L010_PerformSingleMeasurement(VL53L0_DEV Dev);

VL53L010_API VL53L0_Error VL53L010_PerformRefCalibration(VL53L0_DEV Dev);

VL53L010_API VL53L0_Error VL53L010_PerformXTalkCalibration(VL53L0_DEV Dev,
			FixPoint1616_t XTalkCalDistance,
			FixPoint1616_t *pXTalkCompensationRateMegaCps);

VL53L010_API VL53L0_Error VL53L010_PerformOffsetCalibration(VL53L0_DEV Dev,
            FixPoint1616_t CalDistanceMilliMeter, int32_t* pOffsetMicroMeter);

VL53L010_API VL53L0_Error VL53L010_StartMeasurement(VL53L0_DEV Dev);

VL53L010_API VL53L0_Error VL53L010_StopMeasurement(VL53L0_DEV Dev);

VL53L010_API VL53L0_Error VL53L010_GetMeasurementDataReady(VL53L0_DEV Dev,
			uint8_t *pMeasurementDataReady);

VL53L010_API VL53L0_Error VL53L010_WaitDeviceReadyForNewMeasurement(VL53L0_DEV Dev,
			uint32_t MaxLoop);


VL53L010_API VL53L0_Error VL53L010_GetRangingMeasurementData(VL53L0_DEV Dev,
		VL53L0_RangingMeasurementData_t *pRangingMeasurementData);

VL53L010_API VL53L0_Error VL53L010_GetHistogramMeasurementData(VL53L0_DEV Dev,
		VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData);

VL53L010_API VL53L0_Error VL53L010_PerformSingleRangingMeasurement(VL53L0_DEV Dev,
		VL53L0_RangingMeasurementData_t *pRangingMeasurementData);

VL53L010_API VL53L0_Error VL53L010_PerformSingleHistogramMeasurement(
		VL53L0_DEV Dev,
		VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData);



VL53L010_API VL53L0_Error VL53L010_SetNumberOfROIZones(VL53L0_DEV Dev,
			uint8_t NumberOfROIZones);

VL53L010_API VL53L0_Error VL53L010_GetNumberOfROIZones(VL53L0_DEV Dev,
			uint8_t *pNumberOfROIZones);

VL53L010_API VL53L0_Error VL53L010_GetMaxNumberOfROIZones(VL53L0_DEV Dev,
			uint8_t *pMaxNumberOfROIZones);





VL53L010_API VL53L0_Error VL53L010_SetGpioConfig(VL53L0_DEV Dev, uint8_t Pin,
				VL53L0_DeviceModes DeviceMode,
				VL53L0_GpioFunctionality Functionality,
				VL53L0_InterruptPolarity Polarity);


VL53L010_API VL53L0_Error VL53L010_GetGpioConfig(VL53L0_DEV Dev, uint8_t Pin,
			VL53L0_DeviceModes *pDeviceMode,
			VL53L0_GpioFunctionality *pFunctionality,
			VL53L0_InterruptPolarity *pPolarity);

VL53L010_API VL53L0_Error VL53L010_SetInterruptThresholds(VL53L0_DEV Dev,
			VL53L0_DeviceModes DeviceMode,
			FixPoint1616_t ThresholdLow,
			FixPoint1616_t ThresholdHigh);

VL53L010_API VL53L0_Error VL53L010_GetInterruptThresholds(VL53L0_DEV Dev,
			VL53L0_DeviceModes DeviceMode,
			FixPoint1616_t *pThresholdLow,
			FixPoint1616_t *pThresholdHigh);

VL53L010_API VL53L0_Error VL53L010_ClearInterruptMask(VL53L0_DEV Dev,
			uint32_t InterruptMask);

VL53L010_API VL53L0_Error VL53L010_GetInterruptMaskStatus(VL53L0_DEV Dev,
			uint32_t *pInterruptMaskStatus);


VL53L010_API VL53L0_Error VL53L010_EnableInterruptMask(VL53L0_DEV Dev,
			uint32_t InterruptMask);







VL53L010_API VL53L0_Error VL53L010_SetSpadAmbientDamperThreshold(VL53L0_DEV Dev,
					uint16_t SpadAmbientDamperThreshold);

VL53L010_API VL53L0_Error VL53L010_GetSpadAmbientDamperThreshold(VL53L0_DEV Dev,
					uint16_t *pSpadAmbientDamperThreshold);


VL53L010_API VL53L0_Error VL53L010_SetSpadAmbientDamperFactor(VL53L0_DEV Dev,
					uint16_t SpadAmbientDamperFactor);

VL53L010_API VL53L0_Error VL53L010_GetSpadAmbientDamperFactor(VL53L0_DEV Dev,
				   uint16_t *pSpadAmbientDamperFactor);






#ifdef __cplusplus
}
#endif

#endif 
