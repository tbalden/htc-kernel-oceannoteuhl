/*******************************************************************************
  Copyright ?2016, STMicroelectronics International N.V.
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
 ******************************************************************************/

#include "vl53l0_api.h"
#include "vl53l0_api_core.h"
#include "vl53l0_api_calibration.h"


#ifndef __KERNEL__
#include <stdlib.h>
#endif
#define LOG_FUNCTION_START(fmt, ...) \
    _LOG_FUNCTION_START(TRACE_MODULE_API, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
    _LOG_FUNCTION_END(TRACE_MODULE_API, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
    _LOG_FUNCTION_END_FMT(TRACE_MODULE_API, status, fmt, ##__VA_ARGS__)

VL53L0_Error VL53L0_reverse_bytes(uint8_t *data, uint32_t size)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t tempData;
    uint32_t mirrorIndex;
    uint32_t middle = size/2;
    uint32_t index;

    for (index = 0; index < middle; index++) {
        mirrorIndex      = size - index - 1;
        tempData         = data[index];
        data[index]      = data[mirrorIndex];
        data[mirrorIndex] = tempData;
    }
    return Status;
}

VL53L0_Error VL53L0_measurement_poll_for_completion(VL53L0_DEV Dev)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t NewDataReady = 0;
    uint32_t LoopNb;

    LOG_FUNCTION_START("");

    LoopNb = 0;

    do {
        Status = VL53L0_GetMeasurementDataReady(Dev, &NewDataReady);
        if (Status != 0)
            break; 

        if (NewDataReady == 1)
            break; 

        LoopNb++;
        if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP) {
            Status = VL53L0_ERROR_TIME_OUT;
            break;
        }

        VL53L0_PollingDelay(Dev);
    } while (1);

    LOG_FUNCTION_END(Status);

    return Status;
}


uint8_t VL53L0_decode_vcsel_period(uint8_t vcsel_period_reg)
{

    uint8_t vcsel_period_pclks = 0;

    vcsel_period_pclks = (vcsel_period_reg + 1) << 1;

    return vcsel_period_pclks;
}

uint8_t VL53L0_encode_vcsel_period(uint8_t vcsel_period_pclks)
{

    uint8_t vcsel_period_reg = 0;

    vcsel_period_reg = (vcsel_period_pclks >> 1) - 1;

    return vcsel_period_reg;
}


uint32_t VL53L0_isqrt(uint32_t num)
{

    uint32_t  res = 0;
    uint32_t  bit = 1 << 30;

    
    while (bit > num)
        bit >>= 2;


    while (bit != 0) {
        if (num >= res + bit) {
            num -= res + bit;
            res = (res >> 1) + bit;
        } else
            res >>= 1;

        bit >>= 2;
    }

    return res;
}


uint32_t VL53L0_quadrature_sum(uint32_t a, uint32_t b)
{
    uint32_t  res = 0;

    if (a > 65535 || b > 65535)
        res = 65535;
    else
        res = VL53L0_isqrt(a * a + b * b);

    return res;
}


VL53L0_Error VL53L0_device_read_strobe(VL53L0_DEV Dev)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t strobe;
    uint32_t LoopNb;
    LOG_FUNCTION_START("");

    Status |= VL53L0_WrByte(Dev, 0x83, 0x00);

    if (Status == VL53L0_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0_RdByte(Dev, 0x83, &strobe);
            if ((strobe != 0x00) || Status != VL53L0_ERROR_NONE)
                break;

            LoopNb = LoopNb + 1;
        } while (LoopNb < VL53L0_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP)
            Status = VL53L0_ERROR_TIME_OUT;

    }

    Status |= VL53L0_WrByte(Dev, 0x83, 0x01);

    LOG_FUNCTION_END(Status);
    return Status;

}

VL53L0_Error VL53L0_get_info_from_device(VL53L0_DEV Dev, uint8_t option)
{

    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t byte;
    uint32_t TmpDWord;
    uint8_t ModuleId;
    uint8_t Revision;
    uint8_t ReferenceSpadCount = 0;
    uint8_t ReferenceSpadType = 0;
    uint32_t PartUIDUpper = 0;
    uint32_t PartUIDLower = 0;
    uint32_t OffsetFixed1104_mm = 0;
    int16_t OffsetMicroMeters = 0;
    uint32_t DistMeasTgtFixed1104_mm = 400 << 4;
    uint32_t DistMeasFixed1104_400_mm = 0;
    uint32_t SignalRateMeasFixed1104_400_mm = 0;
    char ProductId[19];
    char *ProductId_tmp;
    uint8_t ReadDataFromDeviceDone;
    FixPoint1616_t SignalRateMeasFixed400mmFix = 0;
    uint8_t NvmRefGoodSpadMap[VL53L0_REF_SPAD_BUFFER_SIZE];
    int i;


    LOG_FUNCTION_START("");

    ReadDataFromDeviceDone = VL53L0_GETDEVICESPECIFICPARAMETER(Dev,
            ReadDataFromDeviceDone);

    if (ReadDataFromDeviceDone != 7) {

        Status |= VL53L0_WrByte(Dev, 0x80, 0x01);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0_WrByte(Dev, 0x00, 0x00);

        Status |= VL53L0_WrByte(Dev, 0xFF, 0x06);
        Status |= VL53L0_RdByte(Dev, 0x83, &byte);
        Status |= VL53L0_WrByte(Dev, 0x83, byte|4);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x07);
        Status |= VL53L0_WrByte(Dev, 0x81, 0x01);

        Status |= VL53L0_PollingDelay(Dev);

        Status |= VL53L0_WrByte(Dev, 0x80, 0x01);

        if (((option & 1) == 1) &&
                ((ReadDataFromDeviceDone & 1) == 0)) {
            Status |= VL53L0_WrByte(Dev, 0x94, 0x6b);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            ReferenceSpadCount = (uint8_t)((TmpDWord >> 8) & 0x07f);
            ReferenceSpadType  = (uint8_t)((TmpDWord >> 15) & 0x01);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x24);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);


            NvmRefGoodSpadMap[0] = (uint8_t)((TmpDWord >> 24)
                    & 0xff);
            NvmRefGoodSpadMap[1] = (uint8_t)((TmpDWord >> 16)
                    & 0xff);
            NvmRefGoodSpadMap[2] = (uint8_t)((TmpDWord >> 8)
                    & 0xff);
            NvmRefGoodSpadMap[3] = (uint8_t)(TmpDWord & 0xff);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x25);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            NvmRefGoodSpadMap[4] = (uint8_t)((TmpDWord >> 24)
                    & 0xff);
            NvmRefGoodSpadMap[5] = (uint8_t)((TmpDWord >> 16)
                    & 0xff);
        }

        if (((option & 2) == 2) &&
                ((ReadDataFromDeviceDone & 2) == 0)) {

            Status |= VL53L0_WrByte(Dev, 0x94, 0x02);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdByte(Dev, 0x90, &ModuleId);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x7B);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdByte(Dev, 0x90, &Revision);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x77);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[0] = (char)((TmpDWord >> 25) & 0x07f);
            ProductId[1] = (char)((TmpDWord >> 18) & 0x07f);
            ProductId[2] = (char)((TmpDWord >> 11) & 0x07f);
            ProductId[3] = (char)((TmpDWord >> 4) & 0x07f);

            byte = (uint8_t)((TmpDWord & 0x00f) << 3);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x78);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[4] = (char)(byte +
                    ((TmpDWord >> 29) & 0x07f));
            ProductId[5] = (char)((TmpDWord >> 22) & 0x07f);
            ProductId[6] = (char)((TmpDWord >> 15) & 0x07f);
            ProductId[7] = (char)((TmpDWord >> 8) & 0x07f);
            ProductId[8] = (char)((TmpDWord >> 1) & 0x07f);

            byte = (uint8_t)((TmpDWord & 0x001) << 6);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x79);

            Status |= VL53L0_device_read_strobe(Dev);

            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[9] = (char)(byte +
                    ((TmpDWord >> 26) & 0x07f));
            ProductId[10] = (char)((TmpDWord >> 19) & 0x07f);
            ProductId[11] = (char)((TmpDWord >> 12) & 0x07f);
            ProductId[12] = (char)((TmpDWord >> 5) & 0x07f);

            byte = (uint8_t)((TmpDWord & 0x01f) << 2);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x7A);

            Status |= VL53L0_device_read_strobe(Dev);

            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[13] = (char)(byte +
                    ((TmpDWord >> 30) & 0x07f));
            ProductId[14] = (char)((TmpDWord >> 23) & 0x07f);
            ProductId[15] = (char)((TmpDWord >> 16) & 0x07f);
            ProductId[16] = (char)((TmpDWord >> 9) & 0x07f);
            ProductId[17] = (char)((TmpDWord >> 2) & 0x07f);
            ProductId[18] = '\0';

        }

        if (((option & 4) == 4) &&
                ((ReadDataFromDeviceDone & 4) == 0)) {

            Status |= VL53L0_WrByte(Dev, 0x94, 0x7B);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &PartUIDUpper);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x7C);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &PartUIDLower);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x73);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            SignalRateMeasFixed1104_400_mm = (TmpDWord &
                    0x0000000ff) << 8;

            Status |= VL53L0_WrByte(Dev, 0x94, 0x74);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            SignalRateMeasFixed1104_400_mm |= ((TmpDWord &
                        0xff000000) >> 24);

            Status |= VL53L0_WrByte(Dev, 0x94, 0x75);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            DistMeasFixed1104_400_mm = (TmpDWord & 0x0000000ff)
                << 8;

            Status |= VL53L0_WrByte(Dev, 0x94, 0x76);
            Status |= VL53L0_device_read_strobe(Dev);
            Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

            DistMeasFixed1104_400_mm |= ((TmpDWord & 0xff000000)
                    >> 24);
        }

        Status |= VL53L0_WrByte(Dev, 0x81, 0x00);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x06);
        Status |= VL53L0_RdByte(Dev, 0x83, &byte);
        Status |= VL53L0_WrByte(Dev, 0x83, byte&0xfb);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0_WrByte(Dev, 0x00, 0x01);

        Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
        Status |= VL53L0_WrByte(Dev, 0x80, 0x00);
    }

    if ((Status == VL53L0_ERROR_NONE) &&
            (ReadDataFromDeviceDone != 7)) {
        
        if (((option & 1) == 1) &&
                ((ReadDataFromDeviceDone & 1) == 0)) {
            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    ReferenceSpadCount, ReferenceSpadCount);

            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    ReferenceSpadType, ReferenceSpadType);

            for (i = 0; i < VL53L0_REF_SPAD_BUFFER_SIZE; i++) {
                Dev->Data.SpadData.RefGoodSpadMap[i] =
                    NvmRefGoodSpadMap[i];
            }
        }

        if (((option & 2) == 2) &&
                ((ReadDataFromDeviceDone & 2) == 0)) {
            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    ModuleId, ModuleId);

            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    Revision, Revision);

            ProductId_tmp = VL53L0_GETDEVICESPECIFICPARAMETER(Dev,
                    ProductId);
            VL53L0_COPYSTRING(ProductId_tmp, ProductId);

        }

        if (((option & 4) == 4) &&
                ((ReadDataFromDeviceDone & 4) == 0)) {
            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    PartUIDUpper, PartUIDUpper);

            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    PartUIDLower, PartUIDLower);

            SignalRateMeasFixed400mmFix =
                VL53L0_FIXPOINT97TOFIXPOINT1616(
                        SignalRateMeasFixed1104_400_mm);

            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    SignalRateMeasFixed400mm,
                    SignalRateMeasFixed400mmFix);

            OffsetMicroMeters = 0;
            if (DistMeasFixed1104_400_mm != 0) {
                OffsetFixed1104_mm =
                    DistMeasFixed1104_400_mm -
                    DistMeasTgtFixed1104_mm;
                OffsetMicroMeters = (OffsetFixed1104_mm
                        * 1000) >> 4;
                OffsetMicroMeters *= -1;
            }

            PALDevDataSet(Dev,
                    Part2PartOffsetAdjustmentNVMMicroMeter,
                    OffsetMicroMeters);
        }
        byte = (uint8_t)(ReadDataFromDeviceDone|option);
        VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone,
                byte);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}


uint32_t VL53L0_calc_macro_period_ps(VL53L0_DEV Dev, uint8_t vcsel_period_pclks)
{
    uint64_t PLL_period_ps;
    uint32_t macro_period_vclks;
    uint32_t macro_period_ps;

    LOG_FUNCTION_START("");

    PLL_period_ps = 1655;

    macro_period_vclks = 2304;
    macro_period_ps = (uint32_t)(macro_period_vclks
            * vcsel_period_pclks * PLL_period_ps);

    LOG_FUNCTION_END("");
    return macro_period_ps;
}

uint16_t VL53L0_encode_timeout(uint32_t timeout_macro_clks)
{

    uint16_t encoded_timeout = 0;
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_macro_clks > 0) {
        ls_byte = timeout_macro_clks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte = ls_byte >> 1;
            ms_byte++;
        }

        encoded_timeout = (ms_byte << 8)
            + (uint16_t) (ls_byte & 0x000000FF);
    }

    return encoded_timeout;

}

uint32_t VL53L0_decode_timeout(uint16_t encoded_timeout)
{

    uint32_t timeout_macro_clks = 0;

    timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF)
            << (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

    return timeout_macro_clks;
}


uint32_t VL53L0_calc_timeout_mclks(VL53L0_DEV Dev,
        uint32_t timeout_period_us,
        uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t timeout_period_mclks = 0;

    macro_period_ps = VL53L0_calc_macro_period_ps(Dev, vcsel_period_pclks);
    macro_period_ns = (macro_period_ps + 500) / 1000;

    timeout_period_mclks =
        (uint32_t) (((timeout_period_us * 1000)
                    + (macro_period_ns / 2)) / macro_period_ns);

    return timeout_period_mclks;
}

uint32_t VL53L0_calc_timeout_us(VL53L0_DEV Dev,
        uint16_t timeout_period_mclks,
        uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t actual_timeout_period_us = 0;

    macro_period_ps = VL53L0_calc_macro_period_ps(Dev, vcsel_period_pclks);
    macro_period_ns = (macro_period_ps + 500) / 1000;

    actual_timeout_period_us =
        ((timeout_period_mclks * macro_period_ns)
         + (macro_period_ns / 2)) / 1000;

    return actual_timeout_period_us;
}


VL53L0_Error get_sequence_step_timeout(VL53L0_DEV Dev,
        VL53L0_SequenceStepId SequenceStepId,
        uint32_t *pTimeOutMicroSecs)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t CurrentVCSELPulsePeriodPClk;
    uint8_t EncodedTimeOutByte = 0;
    uint32_t TimeoutMicroSeconds = 0;
    uint16_t PreRangeEncodedTimeOut = 0;
    uint16_t MsrcTimeOutMClks;
    uint16_t PreRangeTimeOutMClks;
    uint16_t FinalRangeTimeOutMClks = 0;
    uint16_t FinalRangeEncodedTimeOut;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;

    if ((SequenceStepId == VL53L0_SEQUENCESTEP_TCC)  ||
            (SequenceStepId == VL53L0_SEQUENCESTEP_DSS)  ||
            (SequenceStepId == VL53L0_SEQUENCESTEP_MSRC)) {

        Status = VL53L0_GetVcselPulsePeriod(Dev,
                VL53L0_VCSEL_PERIOD_PRE_RANGE,
                &CurrentVCSELPulsePeriodPClk);
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_RdByte(Dev,
                    VL53L0_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                    &EncodedTimeOutByte);
        }
        MsrcTimeOutMClks = VL53L0_decode_timeout(EncodedTimeOutByte);

        TimeoutMicroSeconds = VL53L0_calc_timeout_us(Dev,
                MsrcTimeOutMClks,
                CurrentVCSELPulsePeriodPClk);
    } else if (SequenceStepId == VL53L0_SEQUENCESTEP_PRE_RANGE) {
        
        Status = VL53L0_GetVcselPulsePeriod(Dev,
                VL53L0_VCSEL_PERIOD_PRE_RANGE,
                &CurrentVCSELPulsePeriodPClk);

        
        if (Status == VL53L0_ERROR_NONE) {

            
            Status = VL53L0_GetVcselPulsePeriod(Dev,
                    VL53L0_VCSEL_PERIOD_PRE_RANGE,
                    &CurrentVCSELPulsePeriodPClk);

            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_RdWord(Dev,
                        VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                        &PreRangeEncodedTimeOut);
            }

            PreRangeTimeOutMClks = VL53L0_decode_timeout(
                    PreRangeEncodedTimeOut);

            TimeoutMicroSeconds = VL53L0_calc_timeout_us(Dev,
                    PreRangeTimeOutMClks,
                    CurrentVCSELPulsePeriodPClk);
        }
    } else if (SequenceStepId == VL53L0_SEQUENCESTEP_FINAL_RANGE) {

        VL53L0_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);
        PreRangeTimeOutMClks = 0;

        if (SchedulerSequenceSteps.PreRangeOn) {
            
            Status = VL53L0_GetVcselPulsePeriod(Dev,
                    VL53L0_VCSEL_PERIOD_PRE_RANGE,
                    &CurrentVCSELPulsePeriodPClk);

            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_RdWord(Dev,
                        VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                        &PreRangeEncodedTimeOut);
                PreRangeTimeOutMClks = VL53L0_decode_timeout(
                        PreRangeEncodedTimeOut);
            }
        }

        if (Status == VL53L0_ERROR_NONE) {
            
            Status = VL53L0_GetVcselPulsePeriod(Dev,
                    VL53L0_VCSEL_PERIOD_FINAL_RANGE,
                    &CurrentVCSELPulsePeriodPClk);
        }

        
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_RdWord(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    &FinalRangeEncodedTimeOut);
            FinalRangeTimeOutMClks = VL53L0_decode_timeout(
                    FinalRangeEncodedTimeOut);
        }

        FinalRangeTimeOutMClks -= PreRangeTimeOutMClks;
        TimeoutMicroSeconds = VL53L0_calc_timeout_us(Dev,
                FinalRangeTimeOutMClks,
                CurrentVCSELPulsePeriodPClk);
    }

    *pTimeOutMicroSecs = TimeoutMicroSeconds;

    return Status;
}


VL53L0_Error set_sequence_step_timeout(VL53L0_DEV Dev,
        VL53L0_SequenceStepId SequenceStepId,
        uint32_t TimeOutMicroSecs)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t CurrentVCSELPulsePeriodPClk;
    uint8_t MsrcEncodedTimeOut;
    uint16_t PreRangeEncodedTimeOut;
    uint16_t PreRangeTimeOutMClks;
    uint16_t MsrcRangeTimeOutMClks;
    uint16_t FinalRangeTimeOutMClks;
    uint16_t FinalRangeEncodedTimeOut;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;

    if ((SequenceStepId == VL53L0_SEQUENCESTEP_TCC)  ||
            (SequenceStepId == VL53L0_SEQUENCESTEP_DSS)  ||
            (SequenceStepId == VL53L0_SEQUENCESTEP_MSRC)) {

        Status = VL53L0_GetVcselPulsePeriod(Dev,
                VL53L0_VCSEL_PERIOD_PRE_RANGE,
                &CurrentVCSELPulsePeriodPClk);

        if (Status == VL53L0_ERROR_NONE) {
            MsrcRangeTimeOutMClks = VL53L0_calc_timeout_mclks(Dev,
                    TimeOutMicroSecs,
                    (uint8_t)CurrentVCSELPulsePeriodPClk);

            if (MsrcRangeTimeOutMClks > 256)
                MsrcEncodedTimeOut = 255;
            else
                MsrcEncodedTimeOut =
                    (uint8_t)MsrcRangeTimeOutMClks - 1;

            VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                    LastEncodedTimeout,
                    MsrcEncodedTimeOut);
        }

        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                    MsrcEncodedTimeOut);
        }
    } else {

        if (SequenceStepId == VL53L0_SEQUENCESTEP_PRE_RANGE) {

            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_GetVcselPulsePeriod(Dev,
                        VL53L0_VCSEL_PERIOD_PRE_RANGE,
                        &CurrentVCSELPulsePeriodPClk);
                PreRangeTimeOutMClks =
                    VL53L0_calc_timeout_mclks(Dev,
                            TimeOutMicroSecs,
                            (uint8_t)CurrentVCSELPulsePeriodPClk);
                PreRangeEncodedTimeOut = VL53L0_encode_timeout(
                        PreRangeTimeOutMClks);

                VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
                        LastEncodedTimeout,
                        PreRangeEncodedTimeOut);
            }

            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_WrWord(Dev,
                        VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                        PreRangeEncodedTimeOut);
            }

            if (Status == VL53L0_ERROR_NONE) {
                VL53L0_SETDEVICESPECIFICPARAMETER(
                        Dev,
                        PreRangeTimeoutMicroSecs,
                        TimeOutMicroSecs);
            }
        } else if (SequenceStepId == VL53L0_SEQUENCESTEP_FINAL_RANGE) {


            VL53L0_GetSequenceStepEnables(Dev,
                    &SchedulerSequenceSteps);
            PreRangeTimeOutMClks = 0;
            if (SchedulerSequenceSteps.PreRangeOn) {

                
                Status = VL53L0_GetVcselPulsePeriod(Dev,
                        VL53L0_VCSEL_PERIOD_PRE_RANGE,
                        &CurrentVCSELPulsePeriodPClk);

                if (Status == VL53L0_ERROR_NONE) {
                    Status = VL53L0_RdWord(Dev, 0x51,
                            &PreRangeEncodedTimeOut);
                    PreRangeTimeOutMClks =
                        VL53L0_decode_timeout(
                                PreRangeEncodedTimeOut);
                }
            }

            if (Status == VL53L0_ERROR_NONE) {

                Status = VL53L0_GetVcselPulsePeriod(Dev,
                        VL53L0_VCSEL_PERIOD_FINAL_RANGE,
                        &CurrentVCSELPulsePeriodPClk);
            }
            if (Status == VL53L0_ERROR_NONE) {

                FinalRangeTimeOutMClks =
                    VL53L0_calc_timeout_mclks(Dev,
                            TimeOutMicroSecs,
                            (uint8_t) CurrentVCSELPulsePeriodPClk);

                FinalRangeTimeOutMClks += PreRangeTimeOutMClks;

                FinalRangeEncodedTimeOut =
                    VL53L0_encode_timeout(FinalRangeTimeOutMClks);

                if (Status == VL53L0_ERROR_NONE) {
                    Status = VL53L0_WrWord(Dev, 0x71,
                            FinalRangeEncodedTimeOut);
                }

                if (Status == VL53L0_ERROR_NONE) {
                    VL53L0_SETDEVICESPECIFICPARAMETER(
                            Dev,
                            FinalRangeTimeoutMicroSecs,
                            TimeOutMicroSecs);
                }
            }
        } else
            Status = VL53L0_ERROR_INVALID_PARAMS;

    }
    return Status;
}

VL53L0_Error VL53L0_set_vcsel_pulse_period(VL53L0_DEV Dev,
        VL53L0_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t vcsel_period_reg;
    uint8_t MinPreVcselPeriodPCLK = 12;
    uint8_t MaxPreVcselPeriodPCLK = 18;
    uint8_t MinFinalVcselPeriodPCLK = 8;
    uint8_t MaxFinalVcselPeriodPCLK = 14;
    uint32_t MeasurementTimingBudgetMicroSeconds;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t PreRangeTimeoutMicroSeconds;
    uint32_t MsrcTimeoutMicroSeconds;
    uint8_t PhaseCalInt = 0;

    

    if ((VCSELPulsePeriodPCLK % 2) != 0) {
        
        Status = VL53L0_ERROR_INVALID_PARAMS;
    } else if (VcselPeriodType == VL53L0_VCSEL_PERIOD_PRE_RANGE &&
            (VCSELPulsePeriodPCLK < MinPreVcselPeriodPCLK ||
             VCSELPulsePeriodPCLK > MaxPreVcselPeriodPCLK)) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    } else if (VcselPeriodType == VL53L0_VCSEL_PERIOD_FINAL_RANGE &&
            (VCSELPulsePeriodPCLK < MinFinalVcselPeriodPCLK ||
             VCSELPulsePeriodPCLK > MaxFinalVcselPeriodPCLK)) {

        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    

    if (Status != VL53L0_ERROR_NONE)
        return Status;


    if (VcselPeriodType == VL53L0_VCSEL_PERIOD_PRE_RANGE) {

        
        if (VCSELPulsePeriodPCLK == 12) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x18);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);
        } else if (VCSELPulsePeriodPCLK == 14) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x30);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);
        } else if (VCSELPulsePeriodPCLK == 16) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x40);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);
        } else if (VCSELPulsePeriodPCLK == 18) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x50);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);
        }
    } else if (VcselPeriodType == VL53L0_VCSEL_PERIOD_FINAL_RANGE) {

        if (VCSELPulsePeriodPCLK == 8) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x10);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);

            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);

            Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_LIM,
                    0x30);
            Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
        } else if (VCSELPulsePeriodPCLK == 10) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x28);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);

            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);

            Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_LIM,
                    0x20);
            Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
        } else if (VCSELPulsePeriodPCLK == 12) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x38);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);

            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);

            Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_LIM,
                    0x20);
            Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
        } else if (VCSELPulsePeriodPCLK == 14) {

            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                    0x048);
            Status = VL53L0_WrByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                    0x08);

            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);

            Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0_WrByte(Dev,
                    VL53L0_REG_ALGO_PHASECAL_LIM,
                    0x20);
            Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
        }
    }


    

    if (Status == VL53L0_ERROR_NONE) {
        vcsel_period_reg = VL53L0_encode_vcsel_period((uint8_t)
                VCSELPulsePeriodPCLK);

        /* When the VCSEL period for the pre or final range is changed,
         * the corresponding timeout must be read from the device using
         * the current VCSEL period, then the new VCSEL period can be
         * applied. The timeout then must be written back to the device
         * using the new VCSEL period.
         *
         * For the MSRC timeout, the same applies - this timeout being
         * dependant on the pre-range vcsel period.
         */
        switch (VcselPeriodType) {
            case VL53L0_VCSEL_PERIOD_PRE_RANGE:
                Status = get_sequence_step_timeout(Dev,
                        VL53L0_SEQUENCESTEP_PRE_RANGE,
                        &PreRangeTimeoutMicroSeconds);

                if (Status == VL53L0_ERROR_NONE)
                    Status = get_sequence_step_timeout(Dev,
                            VL53L0_SEQUENCESTEP_MSRC,
                            &MsrcTimeoutMicroSeconds);

                if (Status == VL53L0_ERROR_NONE)
                    Status = VL53L0_WrByte(Dev,
                            VL53L0_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
                            vcsel_period_reg);


                if (Status == VL53L0_ERROR_NONE)
                    Status = set_sequence_step_timeout(Dev,
                            VL53L0_SEQUENCESTEP_PRE_RANGE,
                            PreRangeTimeoutMicroSeconds);


                if (Status == VL53L0_ERROR_NONE)
                    Status = set_sequence_step_timeout(Dev,
                            VL53L0_SEQUENCESTEP_MSRC,
                            MsrcTimeoutMicroSeconds);

                VL53L0_SETDEVICESPECIFICPARAMETER(
                        Dev,
                        PreRangeVcselPulsePeriod,
                        VCSELPulsePeriodPCLK);
                break;
            case VL53L0_VCSEL_PERIOD_FINAL_RANGE:
                Status = get_sequence_step_timeout(Dev,
                        VL53L0_SEQUENCESTEP_FINAL_RANGE,
                        &FinalRangeTimeoutMicroSeconds);

                if (Status == VL53L0_ERROR_NONE)
                    Status = VL53L0_WrByte(Dev,
                            VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                            vcsel_period_reg);


                if (Status == VL53L0_ERROR_NONE)
                    Status = set_sequence_step_timeout(Dev,
                            VL53L0_SEQUENCESTEP_FINAL_RANGE,
                            FinalRangeTimeoutMicroSeconds);

                VL53L0_SETDEVICESPECIFICPARAMETER(
                        Dev,
                        FinalRangeVcselPulsePeriod,
                        VCSELPulsePeriodPCLK);
                break;
            default:
                Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_GETPARAMETERFIELD(Dev,
                MeasurementTimingBudgetMicroSeconds,
                MeasurementTimingBudgetMicroSeconds);

        Status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(Dev,
                MeasurementTimingBudgetMicroSeconds);
    }

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_perform_phase_calibration(
                Dev, &PhaseCalInt, 0, 1);

    return Status;
}

VL53L0_Error VL53L0_get_vcsel_pulse_period(VL53L0_DEV Dev,
        VL53L0_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t vcsel_period_reg;

    switch (VcselPeriodType) {
        case VL53L0_VCSEL_PERIOD_PRE_RANGE:
            Status = VL53L0_RdByte(Dev,
                    VL53L0_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
                    &vcsel_period_reg);
            break;
        case VL53L0_VCSEL_PERIOD_FINAL_RANGE:
            Status = VL53L0_RdByte(Dev,
                    VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                    &vcsel_period_reg);
            break;
        default:
            Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    if (Status == VL53L0_ERROR_NONE)
        *pVCSELPulsePeriodPCLK =
            VL53L0_decode_vcsel_period(vcsel_period_reg);

    return Status;
}



VL53L0_Error VL53L0_set_measurement_timing_budget_micro_seconds(VL53L0_DEV Dev,
        uint32_t MeasurementTimingBudgetMicroSeconds)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint32_t FinalRangeTimingBudgetMicroSeconds;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;
    uint32_t MsrcDccTccTimeoutMicroSeconds  = 2000;
    uint32_t StartOverheadMicroSeconds      = 1320;
    uint32_t EndOverheadMicroSeconds        = 960;
    uint32_t MsrcOverheadMicroSeconds       = 660;
    uint32_t TccOverheadMicroSeconds        = 590;
    uint32_t DssOverheadMicroSeconds        = 690;
    uint32_t PreRangeOverheadMicroSeconds   = 660;
    uint32_t FinalRangeOverheadMicroSeconds = 550;
    uint32_t PreRangeTimeoutMicroSeconds    = 0;
    uint32_t cMinTimingBudgetMicroSeconds   = 20000;
    uint32_t SubTimeout = 0;

    LOG_FUNCTION_START("");

    if (MeasurementTimingBudgetMicroSeconds
            < cMinTimingBudgetMicroSeconds) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
        return Status;
    }

    FinalRangeTimingBudgetMicroSeconds =
        MeasurementTimingBudgetMicroSeconds -
        (StartOverheadMicroSeconds + EndOverheadMicroSeconds);

    Status = VL53L0_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);

    if (Status == VL53L0_ERROR_NONE &&
            (SchedulerSequenceSteps.TccOn  ||
             SchedulerSequenceSteps.MsrcOn ||
             SchedulerSequenceSteps.DssOn)) {

        
        Status = get_sequence_step_timeout(Dev,
                VL53L0_SEQUENCESTEP_MSRC,
                &MsrcDccTccTimeoutMicroSeconds);


        if (Status != VL53L0_ERROR_NONE)
            return Status;

        
        if (SchedulerSequenceSteps.TccOn) {

            SubTimeout = MsrcDccTccTimeoutMicroSeconds
                + TccOverheadMicroSeconds;

            if (SubTimeout <
                    FinalRangeTimingBudgetMicroSeconds) {
                FinalRangeTimingBudgetMicroSeconds -=
                    SubTimeout;
            } else {
                
                Status = VL53L0_ERROR_INVALID_PARAMS;
            }
        }

        if (Status != VL53L0_ERROR_NONE) {
            LOG_FUNCTION_END(Status);
            return Status;
        }

        
        if (SchedulerSequenceSteps.DssOn) {

            SubTimeout = 2 * (MsrcDccTccTimeoutMicroSeconds +
                    DssOverheadMicroSeconds);

            if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                FinalRangeTimingBudgetMicroSeconds
                    -= SubTimeout;
            } else {
                
                Status = VL53L0_ERROR_INVALID_PARAMS;
            }
        } else if (SchedulerSequenceSteps.MsrcOn) {
            
            SubTimeout = MsrcDccTccTimeoutMicroSeconds +
                MsrcOverheadMicroSeconds;

            if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                FinalRangeTimingBudgetMicroSeconds
                    -= SubTimeout;
            } else {
                
                Status = VL53L0_ERROR_INVALID_PARAMS;
            }
        }

    }

    if (Status != VL53L0_ERROR_NONE) {
        LOG_FUNCTION_END(Status);
        return Status;
    }

    if (SchedulerSequenceSteps.PreRangeOn) {

        

        Status = get_sequence_step_timeout(Dev,
                VL53L0_SEQUENCESTEP_PRE_RANGE,
                &PreRangeTimeoutMicroSeconds);

        SubTimeout = PreRangeTimeoutMicroSeconds +
            PreRangeOverheadMicroSeconds;

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
            FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
            
            Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }


    if (Status == VL53L0_ERROR_NONE &&
            SchedulerSequenceSteps.FinalRangeOn) {

        FinalRangeTimingBudgetMicroSeconds -=
            FinalRangeOverheadMicroSeconds;

        Status = set_sequence_step_timeout(Dev,
                VL53L0_SEQUENCESTEP_FINAL_RANGE,
                FinalRangeTimingBudgetMicroSeconds);

        VL53L0_SETPARAMETERFIELD(Dev,
                MeasurementTimingBudgetMicroSeconds,
                MeasurementTimingBudgetMicroSeconds);
    }

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0_Error VL53L0_get_measurement_timing_budget_micro_seconds(VL53L0_DEV Dev,
        uint32_t *pMeasurementTimingBudgetMicroSeconds)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t MsrcDccTccTimeoutMicroSeconds  = 2000;
    uint32_t StartOverheadMicroSeconds      = 1910;
    uint32_t EndOverheadMicroSeconds        = 960;
    uint32_t MsrcOverheadMicroSeconds       = 660;
    uint32_t TccOverheadMicroSeconds        = 590;
    uint32_t DssOverheadMicroSeconds        = 690;
    uint32_t PreRangeOverheadMicroSeconds   = 660;
    uint32_t FinalRangeOverheadMicroSeconds = 550;
    uint32_t PreRangeTimeoutMicroSeconds    = 0;

    LOG_FUNCTION_START("");

    
    *pMeasurementTimingBudgetMicroSeconds
        = StartOverheadMicroSeconds + EndOverheadMicroSeconds;

    Status = VL53L0_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);

    if (Status != VL53L0_ERROR_NONE) {
        LOG_FUNCTION_END(Status);
        return Status;
    }


    if (SchedulerSequenceSteps.TccOn  ||
            SchedulerSequenceSteps.MsrcOn ||
            SchedulerSequenceSteps.DssOn) {

        Status = get_sequence_step_timeout(Dev,
                VL53L0_SEQUENCESTEP_MSRC,
                &MsrcDccTccTimeoutMicroSeconds);

        if (Status == VL53L0_ERROR_NONE) {
            if (SchedulerSequenceSteps.TccOn) {
                *pMeasurementTimingBudgetMicroSeconds +=
                    MsrcDccTccTimeoutMicroSeconds +
                    TccOverheadMicroSeconds;
            }

            if (SchedulerSequenceSteps.DssOn) {
                *pMeasurementTimingBudgetMicroSeconds +=
                    2 * (MsrcDccTccTimeoutMicroSeconds +
                            DssOverheadMicroSeconds);
            } else if (SchedulerSequenceSteps.MsrcOn) {
                *pMeasurementTimingBudgetMicroSeconds +=
                    MsrcDccTccTimeoutMicroSeconds +
                    MsrcOverheadMicroSeconds;
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        if (SchedulerSequenceSteps.PreRangeOn) {
            Status = get_sequence_step_timeout(Dev,
                    VL53L0_SEQUENCESTEP_PRE_RANGE,
                    &PreRangeTimeoutMicroSeconds);
            *pMeasurementTimingBudgetMicroSeconds +=
                PreRangeTimeoutMicroSeconds +
                PreRangeOverheadMicroSeconds;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        if (SchedulerSequenceSteps.FinalRangeOn) {
            Status = get_sequence_step_timeout(Dev,
                    VL53L0_SEQUENCESTEP_FINAL_RANGE,
                    &FinalRangeTimeoutMicroSeconds);
            *pMeasurementTimingBudgetMicroSeconds +=
                (FinalRangeTimeoutMicroSeconds +
                 FinalRangeOverheadMicroSeconds);
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETPARAMETERFIELD(Dev,
                MeasurementTimingBudgetMicroSeconds,
                *pMeasurementTimingBudgetMicroSeconds);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}



VL53L0_Error VL53L0_load_tuning_settings(VL53L0_DEV Dev,
        uint8_t *pTuningSettingBuffer)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int i;
    int Index;
    uint8_t msb;
    uint8_t lsb;
    uint8_t SelectParam;
    uint8_t NumberOfWrites;
    uint8_t Address;
    uint8_t localBuffer[4]; 
    uint16_t Temp16;

    LOG_FUNCTION_START("");

    Index = 0;

    while ((*(pTuningSettingBuffer + Index) != 0) &&
            (Status == VL53L0_ERROR_NONE)) {
        NumberOfWrites = *(pTuningSettingBuffer + Index);
        Index++;
        if (NumberOfWrites == 0xFF) {
            
            SelectParam = *(pTuningSettingBuffer + Index);
            Index++;
            switch (SelectParam) {
                case 0: 
                    msb = *(pTuningSettingBuffer + Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer + Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstRefArray, Temp16);
                    break;
                case 1: 
                    msb = *(pTuningSettingBuffer + Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer + Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstEffPulseWidth,
                            Temp16);
                    break;
                case 2: 
                    msb = *(pTuningSettingBuffer + Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer + Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstEffAmbWidth, Temp16);
                    break;
                case 3: 
                    msb = *(pTuningSettingBuffer + Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer + Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, targetRefRate, Temp16);
                    break;
                default: 
                    Status = VL53L0_ERROR_INVALID_PARAMS;
            }

        } else if (NumberOfWrites <= 4) {
            Address = *(pTuningSettingBuffer + Index);
            Index++;

            for (i = 0; i < NumberOfWrites; i++) {
                localBuffer[i] = *(pTuningSettingBuffer +
                        Index);
                Index++;
            }

            Status = VL53L0_WriteMulti(Dev, Address, localBuffer,
                    NumberOfWrites);

        } else {
            Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0_Error VL53L0_get_total_xtalk_rate(VL53L0_DEV Dev,
        VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
        FixPoint1616_t *ptotal_xtalk_rate_mcps)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    uint8_t xtalkCompEnable;
    FixPoint1616_t totalXtalkMegaCps;
    FixPoint1616_t xtalkPerSpadMegaCps;

    *ptotal_xtalk_rate_mcps = 0;

    Status = VL53L0_GetXTalkCompensationEnable(Dev, &xtalkCompEnable);
    if (Status == VL53L0_ERROR_NONE) {

        if (xtalkCompEnable) {

            VL53L0_GETPARAMETERFIELD(
                    Dev,
                    XTalkCompensationRateMegaCps,
                    xtalkPerSpadMegaCps);

            
            totalXtalkMegaCps =
                pRangingMeasurementData->EffectiveSpadRtnCount *
                xtalkPerSpadMegaCps;

            
            *ptotal_xtalk_rate_mcps =
                (totalXtalkMegaCps + 0x80) >> 8;
        }
    }

    return Status;
}

VL53L0_Error VL53L0_get_total_signal_rate(VL53L0_DEV Dev,
        VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
        FixPoint1616_t *ptotal_signal_rate_mcps)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    FixPoint1616_t totalXtalkMegaCps;

    LOG_FUNCTION_START("");

    *ptotal_signal_rate_mcps =
        pRangingMeasurementData->SignalRateRtnMegaCps;

    Status = VL53L0_get_total_xtalk_rate(
            Dev, pRangingMeasurementData, &totalXtalkMegaCps);

    if (Status == VL53L0_ERROR_NONE)
        *ptotal_signal_rate_mcps += totalXtalkMegaCps;

    return Status;
}

VL53L0_Error VL53L0_calc_dmax(
        VL53L0_DEV Dev,
        FixPoint1616_t totalSignalRate_mcps,
        FixPoint1616_t totalCorrSignalRate_mcps,
        FixPoint1616_t pwMult,
        uint32_t sigmaEstimateP1,
        FixPoint1616_t sigmaEstimateP2,
        uint32_t peakVcselDuration_us,
        uint32_t *pdmax_mm)
{
    const uint32_t cSigmaLimit		= 18;
    const FixPoint1616_t cSignalLimit	= 0x4000; 
    const FixPoint1616_t cSigmaEstRef	= 0x00000042; 
    const uint32_t cAmbEffWidthSigmaEst_ns = 6;
    const uint32_t cAmbEffWidthDMax_ns	   = 7;
    uint32_t dmaxCalRange_mm;
    FixPoint1616_t dmaxCalSignalRateRtn_mcps;
    FixPoint1616_t minSignalNeeded;
    FixPoint1616_t minSignalNeeded_p1;
    FixPoint1616_t minSignalNeeded_p2;
    FixPoint1616_t minSignalNeeded_p3;
    FixPoint1616_t minSignalNeeded_p4;
    FixPoint1616_t sigmaLimitTmp;
    FixPoint1616_t sigmaEstSqTmp;
    FixPoint1616_t signalLimitTmp;
    FixPoint1616_t SignalAt0mm;
    FixPoint1616_t dmaxDark;
    FixPoint1616_t dmaxAmbient;
    FixPoint1616_t dmaxDarkTmp;
    FixPoint1616_t sigmaEstP2Tmp;
    uint32_t signalRateTemp_mcps;

    VL53L0_Error Status = VL53L0_ERROR_NONE;

    LOG_FUNCTION_START("");

    dmaxCalRange_mm =
        PALDevDataGet(Dev, DmaxCalRangeMilliMeter);

    dmaxCalSignalRateRtn_mcps =
        PALDevDataGet(Dev, DmaxCalSignalRateRtnMegaCps);

    
    SignalAt0mm = dmaxCalRange_mm * dmaxCalSignalRateRtn_mcps;

    
    SignalAt0mm = (SignalAt0mm + 0x80) >> 8;
    SignalAt0mm *= dmaxCalRange_mm;

    minSignalNeeded_p1 = 0;
    if (totalCorrSignalRate_mcps > 0) {

        signalRateTemp_mcps = totalSignalRate_mcps << 10;

        
        minSignalNeeded_p1 = signalRateTemp_mcps +
            (totalCorrSignalRate_mcps/2);

        
        minSignalNeeded_p1 /= totalCorrSignalRate_mcps;

        minSignalNeeded_p1 *= 3;

        
        minSignalNeeded_p1 *= minSignalNeeded_p1;

        
        minSignalNeeded_p1 = (minSignalNeeded_p1 + 0x8000) >> 16;
    }

    minSignalNeeded_p2 = pwMult * sigmaEstimateP1;

    
    minSignalNeeded_p2 = (minSignalNeeded_p2 + 0x8000) >> 16;

    
    minSignalNeeded_p2 *= minSignalNeeded_p2;

    sigmaEstP2Tmp = (sigmaEstimateP2 + 0x8000) >> 16;
    sigmaEstP2Tmp = (sigmaEstP2Tmp + cAmbEffWidthSigmaEst_ns/2)/
        cAmbEffWidthSigmaEst_ns;
    sigmaEstP2Tmp *= cAmbEffWidthDMax_ns;

    if (sigmaEstP2Tmp > 0xffff) {
        minSignalNeeded_p3 = 0xfff00000;
    } else {

        sigmaEstimateP2 = (sigmaEstimateP2 + cAmbEffWidthSigmaEst_ns/2)/
            cAmbEffWidthSigmaEst_ns;
        sigmaEstimateP2 *= cAmbEffWidthDMax_ns;

        
        minSignalNeeded_p3 = (sigmaEstimateP2 + 0x8000) >> 16;

        minSignalNeeded_p3 *= minSignalNeeded_p3;

    }

    
    sigmaLimitTmp = ((cSigmaLimit << 14) + 500) / 1000;

    
    sigmaLimitTmp *= sigmaLimitTmp;

    
    sigmaEstSqTmp = cSigmaEstRef * cSigmaEstRef;

    
    sigmaEstSqTmp = (sigmaEstSqTmp + 0x08) >> 4;

    
    sigmaLimitTmp -=  sigmaEstSqTmp;

    
    minSignalNeeded_p4 = 4 * 12 * sigmaLimitTmp;

    
    minSignalNeeded_p4 = (minSignalNeeded_p4 + 0x2000) >> 14;

    
    minSignalNeeded = (minSignalNeeded_p2 + minSignalNeeded_p3);

    
    minSignalNeeded += (peakVcselDuration_us/2);
    minSignalNeeded /= peakVcselDuration_us;

    
    minSignalNeeded <<= 14;

    
    minSignalNeeded += (minSignalNeeded_p4/2);
    minSignalNeeded /= minSignalNeeded_p4;

    
    minSignalNeeded *= minSignalNeeded_p1;

    minSignalNeeded = (minSignalNeeded + 500) / 1000;
    minSignalNeeded <<= 4;

    minSignalNeeded = (minSignalNeeded + 500) / 1000;

    
    signalLimitTmp = (cSignalLimit + 0x80) >> 8;

    
    if (signalLimitTmp != 0)
        dmaxDarkTmp = (SignalAt0mm + (signalLimitTmp / 2))
            / signalLimitTmp;
    else
        dmaxDarkTmp = 0;

    dmaxDark = VL53L0_isqrt(dmaxDarkTmp);

    
    if (minSignalNeeded != 0)
        dmaxAmbient = (SignalAt0mm + minSignalNeeded/2)
            / minSignalNeeded;
    else
        dmaxAmbient = 0;

    dmaxAmbient = VL53L0_isqrt(dmaxAmbient);

    *pdmax_mm = dmaxDark;
    if (dmaxDark > dmaxAmbient)
        *pdmax_mm = dmaxAmbient;

    LOG_FUNCTION_END(Status);

    return Status;
}


VL53L0_Error VL53L0_calc_sigma_estimate(VL53L0_DEV Dev,
        VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
        FixPoint1616_t *pSigmaEstimate,
        uint32_t *pDmax_mm)
{
    
    const uint32_t cPulseEffectiveWidth_centi_ns   = 800;
    
    const uint32_t cAmbientEffectiveWidth_centi_ns = 600;
    const FixPoint1616_t cSigmaEstRef              = 0x00000042; 
    const uint32_t cVcselPulseWidth_ps             = 4700; 
    const FixPoint1616_t cSigmaEstMax              = 0x028F87AE;
    const FixPoint1616_t cSigmaEstRtnMax           = 0xF000;
    const FixPoint1616_t cAmbToSignalRatioMax      = 0xF0000000/
        cAmbientEffectiveWidth_centi_ns;
    
    const FixPoint1616_t cTOF_per_mm_ps            = 0x0006999A;
    const uint32_t c16BitRoundingParam             = 0x00008000;
    const FixPoint1616_t cMaxXTalk_kcps            = 0x00320000;
    const uint32_t cPllPeriod_ps                   = 1655;

    uint32_t vcselTotalEventsRtn;
    uint32_t finalRangeTimeoutMicroSecs;
    uint32_t preRangeTimeoutMicroSecs;
    FixPoint1616_t sigmaEstimateP1;
    FixPoint1616_t sigmaEstimateP2;
    FixPoint1616_t sigmaEstimateP3;
    FixPoint1616_t deltaT_ps;
    FixPoint1616_t pwMult;
    FixPoint1616_t sigmaEstRtn;
    FixPoint1616_t sigmaEstimate;
    FixPoint1616_t xTalkCorrection;
    FixPoint1616_t ambientRate_kcps;
    FixPoint1616_t peakSignalRate_kcps;
    FixPoint1616_t xTalkCompRate_mcps;
    uint32_t xTalkCompRate_kcps;
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    FixPoint1616_t diff1_mcps;
    FixPoint1616_t diff2_mcps;
    FixPoint1616_t sqr1;
    FixPoint1616_t sqr2;
    FixPoint1616_t sqrSum;
    FixPoint1616_t sqrtResult_centi_ns;
    FixPoint1616_t sqrtResult;
    FixPoint1616_t totalSignalRate_mcps;
    FixPoint1616_t correctedSignalRate_mcps;
    uint32_t vcselWidth;
    uint32_t finalRangeMacroPCLKS;
    uint32_t preRangeMacroPCLKS;
    uint32_t peakVcselDuration_us;
    uint8_t finalRangeVcselPCLKS;
    uint8_t preRangeVcselPCLKS;

    LOG_FUNCTION_START("");

    VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            xTalkCompRate_mcps);


    ambientRate_kcps =
        (pRangingMeasurementData->AmbientRateRtnMegaCps * 1000) >> 16;

    correctedSignalRate_mcps =
        pRangingMeasurementData->SignalRateRtnMegaCps;


    Status = VL53L0_get_total_signal_rate(
            Dev, pRangingMeasurementData, &totalSignalRate_mcps);
    Status = VL53L0_get_total_xtalk_rate(
            Dev, pRangingMeasurementData, &xTalkCompRate_mcps);


    peakSignalRate_kcps = (totalSignalRate_mcps * 1000);
    peakSignalRate_kcps = (peakSignalRate_kcps + 0x8000) >> 16;

    xTalkCompRate_kcps = xTalkCompRate_mcps * 1000;

    if (xTalkCompRate_kcps > cMaxXTalk_kcps)
        xTalkCompRate_kcps = cMaxXTalk_kcps;

    if (Status == VL53L0_ERROR_NONE) {

        
        finalRangeTimeoutMicroSecs = VL53L0_GETDEVICESPECIFICPARAMETER(
                Dev, FinalRangeTimeoutMicroSecs);

        finalRangeVcselPCLKS = VL53L0_GETDEVICESPECIFICPARAMETER(
                Dev, FinalRangeVcselPulsePeriod);

        finalRangeMacroPCLKS = VL53L0_calc_timeout_mclks(
                Dev, finalRangeTimeoutMicroSecs, finalRangeVcselPCLKS);

        
        preRangeTimeoutMicroSecs = VL53L0_GETDEVICESPECIFICPARAMETER(
                Dev, PreRangeTimeoutMicroSecs);

        preRangeVcselPCLKS = VL53L0_GETDEVICESPECIFICPARAMETER(
                Dev, PreRangeVcselPulsePeriod);

        preRangeMacroPCLKS = VL53L0_calc_timeout_mclks(
                Dev, preRangeTimeoutMicroSecs, preRangeVcselPCLKS);

        vcselWidth = 3;
        if (finalRangeVcselPCLKS == 8)
            vcselWidth = 2;


        peakVcselDuration_us = vcselWidth * 2048 *
            (preRangeMacroPCLKS + finalRangeMacroPCLKS);
        peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;
        peakVcselDuration_us *= cPllPeriod_ps;
        peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;

        
        totalSignalRate_mcps = (totalSignalRate_mcps + 0x80) >> 8;

        
        vcselTotalEventsRtn = totalSignalRate_mcps *
            peakVcselDuration_us;

        
        vcselTotalEventsRtn = (vcselTotalEventsRtn + 0x80) >> 8;

        
        totalSignalRate_mcps <<= 8;
    }

    if (Status != VL53L0_ERROR_NONE) {
        LOG_FUNCTION_END(Status);
        return Status;
    }

    if (peakSignalRate_kcps == 0) {
        *pSigmaEstimate = cSigmaEstMax;
        PALDevDataSet(Dev, SigmaEstimate, cSigmaEstMax);
        *pDmax_mm = 0;
    } else {
        if (vcselTotalEventsRtn < 1)
            vcselTotalEventsRtn = 1;


        sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

        
        sigmaEstimateP2 = (ambientRate_kcps << 16)/peakSignalRate_kcps;
        if (sigmaEstimateP2 > cAmbToSignalRatioMax) {
            sigmaEstimateP2 = cAmbToSignalRatioMax;
        }
        sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;

        sigmaEstimateP3 = 2 * VL53L0_isqrt(vcselTotalEventsRtn * 12);

        
        deltaT_ps = pRangingMeasurementData->RangeMilliMeter *
            cTOF_per_mm_ps;

        diff1_mcps = (((peakSignalRate_kcps << 16) -
                    xTalkCompRate_kcps) + 500)/1000;

        
        diff2_mcps = (((peakSignalRate_kcps << 16) +
                    xTalkCompRate_kcps) + 500)/1000;

        diff1_mcps <<= 8;

        
        xTalkCorrection	 = abs(diff1_mcps/diff2_mcps);

        
        xTalkCorrection <<= 8;

        
        pwMult = deltaT_ps/cVcselPulseWidth_ps; 

        pwMult *= ((1 << 16) - xTalkCorrection);

        
        pwMult =  (pwMult + c16BitRoundingParam) >> 16;

        
        pwMult += (1 << 16);

        pwMult >>= 1;
        
        pwMult = pwMult * pwMult;

        
        pwMult >>= 14;

        
        sqr1 = pwMult * sigmaEstimateP1;

        
        sqr1 = (sqr1 + 0x8000) >> 16;

        
        sqr1 *= sqr1;

        sqr2 = sigmaEstimateP2;

        
        sqr2 = (sqr2 + 0x8000) >> 16;

        
        sqr2 *= sqr2;

        
        sqrSum = sqr1 + sqr2;

        
        sqrtResult_centi_ns = VL53L0_isqrt(sqrSum);

        
        sqrtResult_centi_ns <<= 16;

        sigmaEstRtn = (((sqrtResult_centi_ns+50)/100) /
                sigmaEstimateP3);
        sigmaEstRtn      *= VL53L0_SPEED_OF_LIGHT_IN_AIR;

        
        sigmaEstRtn      += 5000;
        sigmaEstRtn      /= 10000;

        if (sigmaEstRtn > cSigmaEstRtnMax) {
            sigmaEstRtn = cSigmaEstRtnMax;
        }

        
        sqr1 = sigmaEstRtn * sigmaEstRtn;
        
        sqr2 = cSigmaEstRef * cSigmaEstRef;

        
        sqrtResult = VL53L0_isqrt((sqr1 + sqr2));

        sigmaEstimate	 = 1000 * sqrtResult;

        if ((peakSignalRate_kcps < 1) || (vcselTotalEventsRtn < 1) ||
                (sigmaEstimate > cSigmaEstMax)) {
            sigmaEstimate = cSigmaEstMax;
        }

        *pSigmaEstimate = (uint32_t)(sigmaEstimate);
        PALDevDataSet(Dev, SigmaEstimate, *pSigmaEstimate);
        Status = VL53L0_calc_dmax(
                Dev,
                totalSignalRate_mcps,
                correctedSignalRate_mcps,
                pwMult,
                sigmaEstimateP1,
                sigmaEstimateP2,
                peakVcselDuration_us,
                pDmax_mm);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0_Error VL53L0_get_pal_range_status(VL53L0_DEV Dev,
        uint8_t DeviceRangeStatus,
        FixPoint1616_t SignalRate,
        uint16_t EffectiveSpadRtnCount,
        VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
        uint8_t *pPalRangeStatus)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t NoneFlag;
    uint8_t SigmaLimitflag = 0;
    uint8_t SignalRefClipflag = 0;
    uint8_t RangeIgnoreThresholdflag = 0;
    uint8_t SigmaLimitCheckEnable = 0;
    uint8_t SignalRateFinalRangeLimitCheckEnable = 0;
    uint8_t SignalRefClipLimitCheckEnable = 0;
    uint8_t RangeIgnoreThresholdLimitCheckEnable = 0;
    FixPoint1616_t SigmaEstimate;
    FixPoint1616_t SigmaLimitValue;
    FixPoint1616_t SignalRefClipValue;
    FixPoint1616_t RangeIgnoreThresholdValue;
    FixPoint1616_t SignalRatePerSpad;
    uint8_t DeviceRangeStatusInternal = 0;
    uint16_t tmpWord = 0;
    uint8_t Temp8;
    uint32_t Dmax_mm = 0;
    FixPoint1616_t LastSignalRefMcps;

    LOG_FUNCTION_START("");



    DeviceRangeStatusInternal = ((DeviceRangeStatus & 0x78) >> 3);

    if (DeviceRangeStatusInternal == 0 ||
            DeviceRangeStatusInternal == 5 ||
            DeviceRangeStatusInternal == 7 ||
            DeviceRangeStatusInternal == 12 ||
            DeviceRangeStatusInternal == 13 ||
            DeviceRangeStatusInternal == 14 ||
            DeviceRangeStatusInternal == 15
       ) {
        NoneFlag = 1;
    } else {
        NoneFlag = 0;
    }

    
    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_RdWord(Dev,
                VL53L0_REG_RESULT_PEAK_SIGNAL_RATE_REF,
                &tmpWord);

    LastSignalRefMcps = VL53L0_FIXPOINT97TOFIXPOINT1616(tmpWord);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x00);

    PALDevDataSet(Dev, LastSignalRefMcps, LastSignalRefMcps);

    if (Status == VL53L0_ERROR_NONE)
        Status =  VL53L0_GetLimitCheckEnable(Dev,
                VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
                &SigmaLimitCheckEnable);

    if ((SigmaLimitCheckEnable != 0) && (Status == VL53L0_ERROR_NONE)) {
        Status = VL53L0_calc_sigma_estimate(
                Dev,
                pRangingMeasurementData,
                &SigmaEstimate,
                &Dmax_mm);
        if (Status == VL53L0_ERROR_NONE)
            pRangingMeasurementData->RangeDMaxMilliMeter = Dmax_mm;

        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_GetLimitCheckValue(Dev,
                    VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
                    &SigmaLimitValue);

            if ((SigmaLimitValue > 0) &&
                    (SigmaEstimate > SigmaLimitValue))
                
                SigmaLimitflag = 1;
        }
    }

    if (Status == VL53L0_ERROR_NONE)
        Status =  VL53L0_GetLimitCheckEnable(Dev,
                VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
                &SignalRefClipLimitCheckEnable);

    if ((SignalRefClipLimitCheckEnable != 0) &&
            (Status == VL53L0_ERROR_NONE)) {

        Status = VL53L0_GetLimitCheckValue(Dev,
                VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
                &SignalRefClipValue);

        if ((SignalRefClipValue > 0) &&
                (LastSignalRefMcps > SignalRefClipValue)) {
            
            SignalRefClipflag = 1;
        }
    }

    if (Status == VL53L0_ERROR_NONE)
                                    Status =  VL53L0_GetLimitCheckEnable(Dev,
                                            VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                            &RangeIgnoreThresholdLimitCheckEnable);

    if ((RangeIgnoreThresholdLimitCheckEnable != 0) &&
            (Status == VL53L0_ERROR_NONE)) {

        
        if (EffectiveSpadRtnCount == 0) {
            SignalRatePerSpad = 0;
        } else {
            SignalRatePerSpad = (FixPoint1616_t)((256 * SignalRate)
                    / EffectiveSpadRtnCount);
        }

        Status = VL53L0_GetLimitCheckValue(Dev,
                VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                &RangeIgnoreThresholdValue);

        if ((RangeIgnoreThresholdValue > 0) &&
                (SignalRatePerSpad < RangeIgnoreThresholdValue)) {
            
            RangeIgnoreThresholdflag = 1;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        if (NoneFlag == 1) {
            *pPalRangeStatus = 255;  
        } else if (DeviceRangeStatusInternal == 1 ||
                DeviceRangeStatusInternal == 2 ||
                DeviceRangeStatusInternal == 3) {
            *pPalRangeStatus = 5; 
        } else if (DeviceRangeStatusInternal == 6 ||
                DeviceRangeStatusInternal == 9) {
            *pPalRangeStatus = 4;  
        } else if (DeviceRangeStatusInternal == 8 ||
                DeviceRangeStatusInternal == 10 ||
                SignalRefClipflag == 1) {
            *pPalRangeStatus = 3;  
        } else if (DeviceRangeStatusInternal == 4 ||
                RangeIgnoreThresholdflag == 1) {
            *pPalRangeStatus = 2;  
        } else if (SigmaLimitflag == 1) {
            *pPalRangeStatus = 1;  
        } else {
            *pPalRangeStatus = 0; 
        }
    }

    
    if (*pPalRangeStatus == 0)
        pRangingMeasurementData->RangeDMaxMilliMeter = 0;

    

    Status =  VL53L0_GetLimitCheckEnable(Dev,
            VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            &SignalRateFinalRangeLimitCheckEnable);

    if (Status == VL53L0_ERROR_NONE) {
        if ((SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1))
            Temp8 = 1;
        else
            Temp8 = 0;
        VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, Temp8);

        if ((DeviceRangeStatusInternal == 4) ||
                (SignalRateFinalRangeLimitCheckEnable == 0))
            Temp8 = 1;
        else
            Temp8 = 0;
        VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                Temp8);

        if ((SignalRefClipLimitCheckEnable == 0) ||
                (SignalRefClipflag == 1))
            Temp8 = 1;
        else
            Temp8 = 0;

        VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, Temp8);

        if ((RangeIgnoreThresholdLimitCheckEnable == 0) ||
                (RangeIgnoreThresholdflag == 1))
            Temp8 = 1;
        else
            Temp8 = 0;

        VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                Temp8);
    }

    LOG_FUNCTION_END(Status);
    return Status;

}
