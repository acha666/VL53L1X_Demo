/**
  ******************************************************************************
  * @file          : app_tof.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.4.2 instances.
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>

#include "53l1a2_ranging_sensor.h"
#include "custom.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 16 ms < TimingBudget < 500 ms */
#define POLLING_PERIOD (250U) /* refresh rate for polling mode (ms, shall be consistent with TimingBudget value) */

/* Private variables ---------------------------------------------------------*/
static RANGING_SENSOR_Capabilities_t Cap;
static RANGING_SENSOR_ProfileConfig_t Profile;
static RANGING_SENSOR_Result_t Result;
static int32_t status = 0;
static volatile uint8_t PushButtonDetected = 0;
volatile uint8_t ToF_EventDetected = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_53L1A2_SimpleRanging_Init(void);
static void MX_53L1A2_SimpleRanging_Process(void);
static void print_result(RANGING_SENSOR_Result_t *Result);
static int32_t decimal_part(float_t x);

void MX_TOF_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN TOF_Init_PreTreatment */

  /* USER CODE END TOF_Init_PreTreatment */

  /* Initialize the peripherals and the TOF components */

  MX_53L1A2_SimpleRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */

  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_TOF_Process(void)
{
  /* USER CODE BEGIN TOF_Process_PreTreatment */

  /* USER CODE END TOF_Process_PreTreatment */

  MX_53L1A2_SimpleRanging_Process();

  /* USER CODE BEGIN TOF_Process_PostTreatment */

  /* USER CODE END TOF_Process_PostTreatment */
}

static void MX_53L1A2_SimpleRanging_Init(void)
{
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* reset XSHUT (XSDN) pin */
  VL53L1A2_RANGING_SENSOR_SetPowerMode(VL53L1A2_DEV_CENTER, RANGING_SENSOR_POWERMODE_OFF);
  HAL_Delay(2);
  VL53L1A2_RANGING_SENSOR_SetPowerMode(VL53L1A2_DEV_CENTER, RANGING_SENSOR_POWERMODE_ON);
  HAL_Delay(2);

  printf("53L1A2 Simple Ranging demo application\n");
  status = VL53L1A2_RANGING_SENSOR_Init(VL53L1A2_DEV_CENTER);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L1A2_RANGING_SENSOR_Init failed\n");
    while (1);
  }
}

static void MX_53L1A2_SimpleRanging_Process(void)
{
  uint32_t Id;

  VL53L1A2_RANGING_SENSOR_ReadID(VL53L1A2_DEV_CENTER, &Id);
  VL53L1A2_RANGING_SENSOR_GetCapabilities(VL53L1A2_DEV_CENTER, &Cap);

  Profile.RangingProfile = RS_MULTI_TARGET_MEDIUM_RANGE;
  Profile.TimingBudget = TIMING_BUDGET;
  Profile.Frequency = 0; /* Induces intermeasurement period, set to ZERO for normal ranging */
  Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  VL53L1A2_RANGING_SENSOR_ConfigProfile(VL53L1A2_DEV_CENTER, &Profile);

  status = VL53L1A2_RANGING_SENSOR_Start(VL53L1A2_DEV_CENTER, RS_MODE_BLOCKING_CONTINUOUS);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L1A2_RANGING_SENSOR_Start failed\n");
    while (1);
  }

  while (1)
  {
    /* polling mode */
    status = VL53L1A2_RANGING_SENSOR_GetDistance(VL53L1A2_DEV_CENTER, &Result);

    if (status == BSP_ERROR_NONE)
    {
      print_result(&Result);
    }

    HAL_Delay(POLLING_PERIOD);
  }
}

static void print_result(RANGING_SENSOR_Result_t *Result)
{
  uint8_t i;
  uint8_t j;

  for (i = 0; i < RANGING_SENSOR_MAX_NB_ZONES; i++)
  {
    printf("\nTargets = %lu", (unsigned long)Result->ZoneResult[i].NumberOfTargets);

    for (j = 0; j < Result->ZoneResult[i].NumberOfTargets; j++)
    {
      printf("\n |---> ");

      printf("Status = %ld, Distance = %5ld mm ",
             (long)Result->ZoneResult[i].Status[j],
             (long)Result->ZoneResult[i].Distance[j]);

      if (Profile.EnableAmbient)
        printf(", Ambient = %ld.%02ld kcps/spad",
               (long)Result->ZoneResult[i].Ambient[j],
               (long)decimal_part(Result->ZoneResult[i].Ambient[j]));

      if (Profile.EnableSignal)
        printf(", Signal = %ld.%02ld kcps/spad",
               (long)Result->ZoneResult[i].Signal[j],
               (long)decimal_part(Result->ZoneResult[i].Signal[j]));
    }
  }
  printf("\n");
}

static int32_t decimal_part(float_t x)
{
  int32_t int_part = (int32_t) x;
  return (int32_t)((x - int_part) * 100);
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  PushButtonDetected = 1;
}

#ifdef __cplusplus
}
#endif
