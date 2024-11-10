#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "tof.h"

VL53L1X_Sensor *TofSensor1;

/**
 * @brief The application entry point.
 *
 */
extern "C" void app_main_init()
{
    HAL_Delay(5000);
    printf("Hello, World!\n");
    TofSensor1 = new VL53L1X_Sensor(&hi2c1);

    printf("Model ID: %X\n", TofSensor1->getModelId());
    printf("Model Type: %X\n", TofSensor1->getModelType());
    printf("Mask Revision: %X\n", TofSensor1->getMaskRevision());

    printf("Waiting for boot...\n");
    while (!TofSensor1->getBootState())
    {
        HAL_Delay(100);
    }
    printf("Chip booted\n");

    /* This function must to be called to initialize the sensor with the default setting  */
    TofSensor1->sensorInit();
    printf("Sensor initialized\n");
    /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    TofSensor1->setDistanceMode(2);           /* 1=short, 2=long */
    TofSensor1->setTimingBudgetInMs(100);     /* in ms possible values [20, 50, 100, 200, 500] */
    TofSensor1->setInterMeasurementInMs(100); /* in ms, IM must be > = TB */
    // TofSensor1->setOffset(20);                /* offset compensation in mm */
    // TofSensor1->setROI(16, 16);               /* minimum ROI 4,4 */
    // TofSensor1->calibrateOffset(140);         /* may take few second to perform the offset cal*/
    // TofSensor1->calibrateXtalk(1000);         /* may take few second to perform the xtalk cal */
    printf("VL53L1X Ultra Lite Driver Example running ...\n");
    TofSensor1->startRanging(); /* This function has to be called to enable the ranging */

    return;
}

/**
 * @brief application main loop.
 *
 */
extern "C" void app_main_loop()
{

     while (TofSensor1->checkForDataReady() == 0)
    {
      HAL_Delay(2);
    }
    uint16_t rangeStatus, distance, signalRate, ambientRate, spadNum;
    rangeStatus = TofSensor1->getRangeStatus();
    distance = TofSensor1->getDistance();
    signalRate = TofSensor1->getSignalPerSpad();
    ambientRate = TofSensor1->getAmbientPerSpad();
    spadNum = TofSensor1->getSpadNb();
    TofSensor1->clearInterrupt(); /* clear interrupt has to be called to enable next interrupt*/

    printf("\t%d\t|\t%d\t|\t%d\t|\t%d\t|\t%d\n", rangeStatus, distance, signalRate, ambientRate, spadNum);
}