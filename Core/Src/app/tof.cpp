#include "main.h"
#include "tof.h"
#include <stdexcept>

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#define WAIT(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#else
#define WAIT(ms) HAL_Delay(ms)
#endif

#define I2C_TIME_OUT_BASE 10
#define I2C_TIME_OUT_BYTE 1

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */
    0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center, use SetROI() */
    0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

static const uint8_t status_rtn[24] = {255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
                                       255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
                                       255, 255, 11, 12};

VL53L1X_Sensor::VL53L1X_Sensor(I2C_HandleTypeDef *i2c_bus,
                               uint8_t i2c_addr) : dev_addr(i2c_addr), useMutex(false)
{
    this->dev_i2c_bus = i2c_bus;
    this->_pGetMutexFunction = nullptr;
    this->_pReleaseMutexFunction = nullptr;
};

VL53L1X_Sensor::VL53L1X_Sensor(I2C_HandleTypeDef *i2c_bus,
                               FunctionType pGetMutexFunction,
                               FunctionType pReleaseMutexFunction,
                               uint8_t i2c_addr) : dev_addr(i2c_addr), useMutex(true)
{
    this->dev_i2c_bus = i2c_bus;
    this->_pGetMutexFunction = pGetMutexFunction;
    this->_pReleaseMutexFunction = pReleaseMutexFunction;
};

VL53L1X_Sensor::~VL53L1X_Sensor() {
    // Destructor
};

uint8_t VL53L1X_Sensor::getModelId()
{
    uint8_t tmp;
    this->_readByte(VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
    return tmp;
}

uint8_t VL53L1X_Sensor::getModelType()
{
    uint8_t tmp;
    this->_readByte(0x0110, &tmp);
    return tmp;
}

uint8_t VL53L1X_Sensor::getMaskRevision()
{
    uint8_t tmp;
    this->_readByte(0x0111, &tmp);
    return tmp;
}

void VL53L1X_Sensor::setI2CAddress(uint8_t new_address)
{
    this->_writeByte(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address);
    return;
}

void VL53L1X_Sensor::sensorInit()
{
    uint8_t addr = 0x00;
    for (addr = 0x2D; addr <= 0x87; addr++)
    {
        this->_writeByte(addr, VL51L1X_DEFAULT_CONFIGURATION[addr - 0x2D]);
    }
    this->startRanging();
    while (this->checkForDataReady())
    {
        WAIT(5);
    }
    this->clearInterrupt();
    this->stopRanging();
    this->_writeByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); // two bounds VHV
    this->_writeByte(0x0B, 0);                                            // start VHV from the previous temperature
    return;
}

void VL53L1X_Sensor::clearInterrupt()
{
    this->_writeByte(SYSTEM__INTERRUPT_CLEAR, 0x01);
    return;
}

void VL53L1X_Sensor::setInterruptPolarity(uint8_t newPolarity)
{
    uint8_t tmp;
    this->_readByte(GPIO_HV_MUX__CTRL, &tmp);
    tmp = tmp & 0xEF;
    this->_writeByte(GPIO_HV_MUX__CTRL, tmp | (!(newPolarity & 1)) << 4);
    return;
}

uint8_t VL53L1X_Sensor::getInterruptPolarity()
{
    uint8_t tmp;
    this->_readByte(GPIO_HV_MUX__CTRL, &tmp);
    tmp = tmp & 0x10;
    return !(tmp >> 4);
}

void VL53L1X_Sensor::startRanging()
{
    this->_writeByte(SYSTEM__MODE_START, 0x40); // Enable VL53L1X
    return;
}

void VL53L1X_Sensor::stopRanging()
{
    this->_writeByte(SYSTEM__MODE_START, 0x00); // Disable VL53L1X
    return;
}

uint8_t VL53L1X_Sensor::checkForDataReady()
{
    uint8_t temp;
    uint8_t intPol;
    intPol = this->getInterruptPolarity();
    this->_readByte(GPIO__TIO_HV_STATUS, &temp);
    if ((temp & 1) == intPol)
        return 1;
    else
        return 0;
}

void VL53L1X_Sensor::setTimingBudgetInMs(uint16_t timingBudget)
{
    uint16_t distanceMode = this->getDistanceMode();
    if (distanceMode == 0)
        throw std::runtime_error("setTimingBudgetInMs() failed: distance mode not set");
    else if (distanceMode == 1) // Short DistanceMode
    {
        switch (timingBudget)
        {
        case 15:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
            break;
        case 20:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            break;
        case 33:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            break;
        case 50:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01AE);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
            break;
        case 100:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
            break;
        case 200:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
            break;
        case 500:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
            break;
        default:
            throw std::invalid_argument("setTimingBudgetInMs() failed: invalid timing budget");
        }
    }
    else
    {
        switch (timingBudget)
        {
        case 20:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
            break;
        case 33:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            break;
        case 50:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
            break;
        case 100:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
            break;
        case 200:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
            break;
        case 500:
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
            this->_writeWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
            break;
        default:
            throw std::invalid_argument("setTimingBudgetInMs() failed: invalid timing budget");
        }
    }
    return;
}

uint16_t VL53L1X_Sensor::getTimingBudgetInMs()
{
    uint16_t temp;
    uint16_t timingBudget;
    this->_readWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &temp);
    switch (temp)
    {
    case 0x001D:
        timingBudget = 15;
        break;
    case 0x0051:
    case 0x001E:
        timingBudget = 20;
        break;
    case 0x00D6:
    case 0x0060:
        timingBudget = 33;
        break;
    case 0x01AE:
    case 0x00AD:
        timingBudget = 50;
        break;
    case 0x02E1:
    case 0x01CC:
        timingBudget = 100;
        break;
    case 0x03E1:
    case 0x02D9:
        timingBudget = 200;
        break;
    case 0x0591:
    case 0x048F:
        timingBudget = 500;
        break;
    default:
        throw std::runtime_error("getTimingBudgetInMs() failed: invalid timing budget");
        timingBudget = 0;
    }
    return timingBudget;
}

void VL53L1X_Sensor::setDistanceMode(uint16_t DM)
{
    uint16_t timingBudget;
    try
    {
        timingBudget = this->getTimingBudgetInMs();
    }
    catch (const std::runtime_error &e)
    {
        throw e;
    }

    switch (DM)
    {
    case 1:
        this->_writeByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
        this->_writeByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
        this->_writeByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
        this->_writeByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
        this->_writeWord(SD_CONFIG__WOI_SD0, 0x0705);
        this->_writeWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
        break;
    case 2:
        this->_writeByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
        this->_writeByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
        this->_writeByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
        this->_writeByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
        this->_writeWord(SD_CONFIG__WOI_SD0, 0x0F0D);
        this->_writeWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
        break;
    default:
        throw std::invalid_argument("setDistanceMode() failed: invalid distance mode");
    }
    this->setTimingBudgetInMs(timingBudget);
    return;
}

uint16_t VL53L1X_Sensor::getDistanceMode()
{
    uint8_t tmp;
    this->_readByte(PHASECAL_CONFIG__TIMEOUT_MACROP, &tmp);
    if (tmp == 0x14)
        return 1;
    else if (tmp == 0x0A)
        return 2;
    else
        return 0;
}

void VL53L1X_Sensor::setInterMeasurementInMs(uint32_t interMeasurement)
{
    uint16_t clockPll;
    this->_readWord(VL53L1_RESULT__OSC_CALIBRATE_VAL, &clockPll);
    clockPll = clockPll & 0x3FF;
    this->_writeDWord(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, (uint32_t)((float)interMeasurement * clockPll * 1.075));
    return;
}

uint16_t VL53L1X_Sensor::getInterMeasurementInMs()
{
    uint16_t clockPll;
    uint32_t tmp;
    this->_readDWord(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
    this->_readWord(VL53L1_RESULT__OSC_CALIBRATE_VAL, &clockPll);
    clockPll = clockPll & 0x3FF;

    return (uint16_t)((float)tmp / clockPll / 1.065);
}

bool VL53L1X_Sensor::getBootState()
{
    uint8_t tmp;
    this->_readByte(VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
    return tmp;
}

uint16_t VL53L1X_Sensor::getDistance()
{
    uint16_t distance;
    this->_readWord(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &distance);
    return distance;
}

uint16_t VL53L1X_Sensor::getSignalPerSpad()
{
    uint16_t spadNb = 1, signal;
    this->_readWord(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
    this->_readWord(VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &spadNb);
    return (uint16_t)(200.0 * signal / spadNb);
}

uint16_t VL53L1X_Sensor::getAmbientPerSpad()
{
    uint16_t AmbientRate, spadNb = 1;
    this->_readWord(RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
    this->_readWord(VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &spadNb);
    return (uint16_t)(200.0 * AmbientRate / spadNb);
}

uint16_t VL53L1X_Sensor::getSignalRate()
{
    uint16_t signal;
    this->_readWord(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
    return signal * 8;
}

uint16_t VL53L1X_Sensor::getSpadNb()
{
    uint16_t spadNb;
    this->_readWord(VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &spadNb);
    return spadNb >> 8;
}

uint16_t VL53L1X_Sensor::getAmbientRate()
{
    uint16_t AmbientRate;
    this->_readWord(RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
    return AmbientRate * 8;
}

uint8_t VL53L1X_Sensor::getRangeStatus()
{
    uint8_t tmp;
    this->_readByte(VL53L1_RESULT__RANGE_STATUS, &tmp);
    tmp = tmp & 0x1F;
    if (tmp < 24)
        return status_rtn[tmp];
    else
        return 255;
}

VL53L1X_Result_t VL53L1X_Sensor::getResult()
{
    VL53L1X_Result_t result;
    uint8_t tmp[17];
    uint8_t rangeStatus = 255;
    this->_readMulti(VL53L1_RESULT__RANGE_STATUS, tmp, 17);
    rangeStatus = tmp[0] & 0x1F;
    if (rangeStatus < 24)
        rangeStatus = status_rtn[rangeStatus];
    result.Status = rangeStatus;
    result.Ambient = ((uint16_t)tmp[7] << 8 | tmp[8]) * 8;
    result.NumSPADs = tmp[3];
    result.SigPerSPAD = (tmp[15] << 8 | tmp[16]) * 8;
    result.Distance = (tmp[13] << 8 | tmp[14]);
    return result;
}

void VL53L1X_Sensor::setOffset(int16_t offset)
{
    int16_t tmp;
    tmp = offset * 4;
    this->_writeWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)tmp);
    this->_writeWord(MM_CONFIG__INNER_OFFSET_MM, 0x0000);
    this->_writeWord(MM_CONFIG__OUTER_OFFSET_MM, 0x0000);
    return;
}

int16_t VL53L1X_Sensor::getOffset()
{
    uint16_t tmp;
    this->_readWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, &tmp);
    tmp = tmp << 3;
    tmp = tmp >> 5;

    int16_t offset = (int16_t)tmp;
    if (offset > 1024)
        offset = offset - 2048;
    return offset;
}

void VL53L1X_Sensor::setXtalk(uint16_t xtalk)
{
    this->_writeWord(ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000);
    this->_writeWord(ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000);
    this->_writeWord(ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, (xtalk << 9) / 1000); // * << 9 (7.9 format) and /1000 to convert cps to kpcs

    return;
}

uint16_t VL53L1X_Sensor::getXtalk()
{
    uint16_t xtalk;
    this->_readWord(ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &xtalk);
    xtalk = (xtalk * 1000) >> 9; // * 1000 to convert kcps to cps and >> 9 (7.9 format)
    return xtalk;
}

void VL53L1X_Sensor::setDistanceThreshold(uint16_t threshLow, uint16_t threshHigh,
                                          uint8_t window, uint8_t intOnNoTarget)
{
    uint8_t tmp = 0;

    this->_readByte(SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
    tmp = tmp & (~0x6F);
    tmp = tmp | window;
    if (intOnNoTarget == 0)
        this->_writeByte(SYSTEM__INTERRUPT_CONFIG_GPIO, tmp);
    else
        this->_writeByte(SYSTEM__INTERRUPT_CONFIG_GPIO, tmp | 0x40);
    this->_writeWord(SYSTEM__THRESH_HIGH, threshHigh);
    this->_writeWord(SYSTEM__THRESH_LOW, threshLow);
    return;
}

uint16_t VL53L1X_Sensor::getDistanceThresholdWindow()
{
    uint8_t tmp;
    this->_readByte(SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
    tmp = tmp & 0x07;
    return tmp;
}

uint16_t VL53L1X_Sensor::getDistanceThresholdLow()
{
    uint16_t tmp;
    this->_readWord(SYSTEM__THRESH_LOW, &tmp);
    return tmp;
}

uint16_t VL53L1X_Sensor::getDistanceThresholdHigh()
{
    uint16_t tmp;
    this->_readWord(SYSTEM__THRESH_HIGH, &tmp);
    return tmp;
}

void VL53L1X_Sensor::setROICenter(uint8_t center)
{
    this->_writeByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, center);
    return;
}

uint8_t VL53L1X_Sensor::getROICenter()
{
    uint8_t tmp;
    this->_readByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, &tmp);
    return tmp;
}

void VL53L1X_Sensor::setROI(uint16_t x, uint16_t y)
{
    uint8_t opticalCenter;
    this->_readByte(VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &opticalCenter);

    if (x > 16)
        x = 16;

    if (y > 16)
        y = 16;

    if (x > 10 || y > 10)
        opticalCenter = 199;

    this->_writeByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, opticalCenter);
    this->_writeByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (y - 1) << 4 | (x - 1));
    return;
}

void VL53L1X_Sensor::getROI_XY(uint16_t *pX, uint16_t *pY)
{
    uint8_t tmp;
    this->_readByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
    *pX = ((uint16_t)tmp & 0x0F) + 1;
    *pY = (((uint16_t)tmp & 0xF0) >> 4) + 1;
    return;
}

void VL53L1X_Sensor::setSignalThreshold(uint16_t signal)
{
    this->_writeWord(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal >> 3);
    return;
}

uint16_t VL53L1X_Sensor::getSignalThreshold()
{
    uint16_t tmp;
    this->_readWord(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
    return tmp << 3;
}

void VL53L1X_Sensor::setSigmaThreshold(uint16_t sigma)
{
    if (sigma > (0xFFFF >> 2))
        throw std::invalid_argument("setSigmaThreshold() failed: invalid sigma threshold");
    /* 16 bits register 14.2 format */
    this->_writeWord(RANGE_CONFIG__SIGMA_THRESH, sigma << 2);
    return;
}

uint16_t VL53L1X_Sensor::getSigmaThreshold()
{
    uint16_t tmp;
    this->_readWord(RANGE_CONFIG__SIGMA_THRESH, &tmp);
    return tmp >> 2;
}

void VL53L1X_Sensor::startTemperatureUpdate()
{
    uint8_t tmp = 0;
    this->_writeByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81); // full VHV
    this->_writeByte(0x0B, 0x92);
    this->startRanging();
    while (this->checkForDataReady())
    {
        WAIT(5);
    }
    tmp = 0;
    this->clearInterrupt();
    this->stopRanging();
    this->_writeByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); // two bounds VHV
    this->_writeByte(0x0B, 0);                                            // start VHV from the previous temperature
    return;
}

void VL53L1X_Sensor::_i2cWrite(uint8_t *pData, uint32_t count)
{
    HAL_StatusTypeDef ret;
    uint32_t i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
    ret = HAL_I2C_Master_Transmit(this->dev_i2c_bus, this->dev_addr | 0, pData, count, i2c_time_out);
    if (ret != HAL_OK)
    {
        if (ret == HAL_TIMEOUT)
            throw std::runtime_error("I2C write timeout");
        else
            throw std::runtime_error("I2C write error");
    }
    return;
}

void VL53L1X_Sensor::_i2cRead(uint8_t *pData, uint32_t count)
{
    HAL_StatusTypeDef ret;
    uint32_t i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
    ret = HAL_I2C_Master_Receive(this->dev_i2c_bus, this->dev_addr | 1, pData, count, i2c_time_out);
    if (ret != HAL_OK)
    {
        if (ret == HAL_TIMEOUT)
            throw std::runtime_error("I2C read timeout");
        else
            throw std::runtime_error("I2C read error");
    }
    return;
}

void VL53L1X_Sensor::_getMutex()
{
    if (this->useMutex && this->_pGetMutexFunction)
        this->_pGetMutexFunction();
    return;
}

void VL53L1X_Sensor::_releaseMutex()
{
    if (this->useMutex && this->_pReleaseMutexFunction)
        this->_pReleaseMutexFunction();
    return;
}

void VL53L1X_Sensor::_readByte(uint16_t index, uint8_t *pData)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 2);
    this->_i2cRead(pData, 1);
    this->_releaseMutex();
    return;
}

void VL53L1X_Sensor::_readWord(uint16_t index, uint16_t *pData)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 2);
    this->_i2cRead(this->_I2cBuffer, 2);
    *pData = ((uint16_t)this->_I2cBuffer[0] << 8) + (uint16_t)this->_I2cBuffer[1];
    this->_releaseMutex();
    return;
}

void VL53L1X_Sensor::_readDWord(uint16_t index, uint32_t *pData)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 2);
    this->_i2cRead(this->_I2cBuffer, 4);
    this->_releaseMutex();
    *pData = ((uint32_t)this->_I2cBuffer[0] << 24) + ((uint32_t)this->_I2cBuffer[1] << 16) + ((uint32_t)this->_I2cBuffer[2] << 8) + (uint32_t)this->_I2cBuffer[3];
    return;
}

void VL53L1X_Sensor::_readMulti(uint16_t index, uint8_t *pData, uint32_t count)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 2);
    this->_i2cRead(pData, count);
    this->_releaseMutex();
    return;
}

void VL53L1X_Sensor::_writeByte(uint16_t index, uint8_t Data)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_I2cBuffer[2] = Data;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 3);
    this->_releaseMutex();
    return;
}

void VL53L1X_Sensor::_writeWord(uint16_t index, uint16_t Data)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_I2cBuffer[2] = Data >> 8;
    this->_I2cBuffer[3] = Data & 0x00FF;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 4);
    this->_releaseMutex();
    return;
}

void VL53L1X_Sensor::_writeDWord(uint16_t index, uint32_t Data)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_I2cBuffer[2] = (Data >> 24) & 0xFF;
    this->_I2cBuffer[3] = (Data >> 16) & 0xFF;
    this->_I2cBuffer[4] = (Data >> 8) & 0xFF;
    this->_I2cBuffer[5] = Data & 0xFF;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 6);
    this->_releaseMutex();
    return;
}

void VL53L1X_Sensor::_writeMulti(uint16_t index, uint8_t *pData, uint32_t count)
{
    this->_I2cBuffer[0] = index >> 8;
    this->_I2cBuffer[1] = index & 0xFF;
    this->_getMutex();
    this->_i2cWrite(this->_I2cBuffer, 2);
    this->_i2cWrite(pData, count);
    this->_releaseMutex();
    return;
}
