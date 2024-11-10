#ifndef __TOF_H_
#define __TOF_H_

#include "stm32f4xx_hal.h"

#define SOFT_RESET 0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS 0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x001E
#define MM_CONFIG__INNER_OFFSET_MM 0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 0x0022
#define GPIO_HV_MUX__CTRL 0x0030
#define GPIO__TIO_HV_STATUS 0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP 0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI 0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A 0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B 0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI 0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO 0x0062
#define RANGE_CONFIG__SIGMA_THRESH 0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS 0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH 0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD 0x006C
#define SYSTEM__THRESH_HIGH 0x0072
#define SYSTEM__THRESH_LOW 0x0074
#define SD_CONFIG__WOI_SD0 0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0 0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE 0x0080
#define SYSTEM__SEQUENCE_CONFIG 0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 0x0082
#define SYSTEM__INTERRUPT_CLEAR 0x0086
#define SYSTEM__MODE_START 0x0087
#define VL53L1_RESULT__RANGE_STATUS 0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD 0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL 0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS 0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID 0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD 0x013E

typedef struct
{
    uint8_t Status;      /*!< ResultStatus */
    uint16_t Distance;   /*!< ResultDistance */
    uint16_t Ambient;    /*!< ResultAmbient */
    uint16_t SigPerSPAD; /*!< ResultSignalPerSPAD */
    uint16_t NumSPADs;   /*!< ResultNumSPADs */
} VL53L1X_Result_t;

class VL53L1X_Sensor
{
    using FunctionType = void (*)();

private:
    uint8_t _I2cBuffer[256];

    I2C_HandleTypeDef *dev_i2c_bus;
    const uint8_t dev_addr; // device i2c address, 7-bit LEFT aligned
                            // that is, 0x52 for write and 0x53 for read
    const bool useMutex;

    void _i2cWrite(uint8_t *pData, uint32_t count);
    void _i2cRead(uint8_t *pData, uint32_t count);

    void _readByte(uint16_t index, uint8_t *pData);
    void _readWord(uint16_t index, uint16_t *pData);
    void _readDWord(uint16_t index, uint32_t *pData);
    void _readMulti(uint16_t index, uint8_t *pData, uint32_t count);

    void _writeByte(uint16_t index, uint8_t Data);
    void _writeWord(uint16_t index, uint16_t Data);
    void _writeDWord(uint16_t index, uint32_t Data);
    void _writeMulti(uint16_t index, uint8_t *pData, uint32_t count);

    FunctionType _pGetMutexFunction, _pReleaseMutexFunction;

    void _getMutex();
    void _releaseMutex();

public:
    /**
     * @brief Construct a new vl53l1x sensor object, polling mode
     * @param i2c_bus
     * @param i2c_addr device i2c address, 7-bit LEFT aligned, default to 0x52
     *
     */
    VL53L1X_Sensor(I2C_HandleTypeDef *i2c_bus, uint8_t i2c_addr = 0x52);
    VL53L1X_Sensor(I2C_HandleTypeDef *i2c_bus, FunctionType pGetMutexFunction, FunctionType pReleaseMutexFunction, uint8_t i2c_addr = 0x52);

    ~VL53L1X_Sensor();

    uint8_t getModelId();
    uint8_t getModelType();
    uint8_t getMaskRevision();

    void setI2CAddress(uint8_t new_address);

    void sensorInit();
    void clearInterrupt();

    void setInterruptPolarity(uint8_t newPolarity);
    uint8_t getInterruptPolarity();

    void startRanging();
    void stopRanging();

    uint8_t checkForDataReady();

    void setTimingBudgetInMs(uint16_t timingBudget);
    uint16_t getTimingBudgetInMs();

    void setDistanceMode(uint16_t distanceMode);
    uint16_t getDistanceMode();

    void setInterMeasurementInMs(uint32_t interMeasurement);
    uint16_t getInterMeasurementInMs();

    bool getBootState();

    uint16_t getDistance();
    uint16_t getSignalPerSpad();
    uint16_t getAmbientPerSpad();
    uint16_t getSignalRate();
    uint16_t getSpadNb();
    uint16_t getAmbientRate();
    uint8_t getRangeStatus();

    VL53L1X_Result_t getResult();

    void setOffset(int16_t offset);
    int16_t getOffset();

    void setXtalk(uint16_t xtalk);
    uint16_t getXtalk();

    void setDistanceThreshold(uint16_t threshLow, uint16_t threshHigh, uint8_t window, uint8_t intOnNoTarget);
    uint16_t getDistanceThresholdWindow();
    uint16_t getDistanceThresholdLow();
    uint16_t getDistanceThresholdHigh();

    void setROICenter(uint8_t center);
    uint8_t getROICenter();

    void setROI(uint16_t x, uint16_t y);
    void getROI_XY(uint16_t *pX, uint16_t *pY);

    void setSignalThreshold(uint16_t signal);
    uint16_t getSignalThreshold();

    void setSigmaThreshold(uint16_t sigma);
    uint16_t getSigmaThreshold();

    void startTemperatureUpdate();
};
#endif
