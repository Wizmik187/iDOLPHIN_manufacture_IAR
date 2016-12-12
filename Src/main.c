/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include  <stdarg.h>
#include  <stdio.h>
#include  <stdbool.h>
#include  <math.h>
#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>
#include  <app_cfg.h>
#include  <bsp.h>

#include "CDCE925.h"
#include "Adafruit_TSL2591.h"
#include "Adafruit_TCS34725.h"
#include "stm32469i_discovery.h"
#include "stm32469i_discovery_ts.h"
#include "stm32469i_discovery_lcd.h"
#include "stm32469i_discovery_sdram.h"
#include "stm32469i_discovery_qspi.h"


#define APP_DELAY_MS(a)   OSTimeDly(a, OS_OPT_TIME_DLY, &err)
#define LOOP_DELAY_MS   30
#define LOOP_DELAY_CNT  200

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart3;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

tsl2591 htsl2591_eval;
tcs34725 htcs34725_eval;
uint32_t Lux;
uint16_t r, g, b, c, colorTemp;

/* Task specific stacks */
static CPU_STK AppTaskStart_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK App_Process1Stk[APP_CFG_TASK_STK_SIZE];
static CPU_STK App_Process2Stk[APP_CFG_TASK_STK_SIZE];
static CPU_STK App_Process3Stk[APP_CFG_TASK_STK_SIZE];
static CPU_STK App_Process4Stk[APP_CFG_TASK_STK_SIZE];
static CPU_STK App_Process5Stk[APP_CFG_TASK_STK_SIZE];
static CPU_STK App_Process6Stk[APP_CFG_TASK_STK_SIZE];

/* Task control blocks */
static  OS_TCB AppTaskStart_TCB;
static  OS_TCB Process1TCB;
static  OS_TCB Process2TCB;
static  OS_TCB Process3TCB;
static  OS_TCB Process4TCB;
static  OS_TCB Process5TCB;
static  OS_TCB Process6TCB;


OS_SEM  Sem1, Sem2, Sem3, Sem4, Sem5;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_QUADSPI_Init(void);

/* USER CODE BEGIN PFP */

static void tsl2591_Init(void);
static void tcs34725_Init(void);

/* Private function prototypes -----------------------------------------------*/
static void AppTaskStart(void *p_arg);
static void AppTaskCreate(void);
static void AppSemCreate(void);


void SendSignal(void)
{
    static CPU_INT08U SigNum = 0;
    static OS_TICK debounce_time = 0;

    OS_ERR err;
    
    /* Prevent debounce effect for user key */
    if( OSTimeGet(&err) - debounce_time > 50 )
    {
        debounce_time = OSTimeGet(&err);
    }
    else
    {
        return;
    }

    switch(SigNum++ % 3)
    {
      case 0:
          OSSemPost(&Sem1, OS_OPT_POST_1 + OS_OPT_POST_NO_SCHED, &err);
      break;
      case 1:
          OSSemPost(&Sem2, OS_OPT_POST_1 + OS_OPT_POST_NO_SCHED, &err);
      break;
      case 2:
          OSSemPost(&Sem3, OS_OPT_POST_1 + OS_OPT_POST_NO_SCHED, &err);
      break;
    }
    
    
}




/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Prevent unused argument(s) compilation warning */
    UNUSED(GPIO_Pin);
    
    if (GPIO_Pin == USER_KEY_Pin)
    {
        SendSignal();

    }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    OS_ERR     err;
    
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();

  /* USER CODE BEGIN 2 */
    BSP_IntDisAll();
    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    App_OS_SetAllHooks();
    
    OSTaskCreate((OS_TCB     *)&AppTaskStart_TCB,               /* Create the start task                                */
                 (CPU_CHAR   *)"Start",
                 (OS_TASK_PTR )AppTaskStart,
                 (void       *)0,
                 (OS_PRIO     )APP_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStart_Stk[0],
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK     )0,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR ),
                 (OS_ERR     *)&err);
    

    OSStart(&err);    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DMA2D init function */
static void MX_DMA2D_Init(void)
{

  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 800;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 114;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AH;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 0;
  hltdc.Init.VerticalSync = 0;
  hltdc.Init.AccumulatedHBP = 1;
  hltdc.Init.AccumulatedVBP = 1;
  hltdc.Init.AccumulatedActiveW = 801;
  hltdc.Init.AccumulatedActiveH = 481;
  hltdc.Init.TotalWidth = 802;
  hltdc.Init.TotalHeigh = 482;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }

  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* QUADSPI init function */
static void MX_QUADSPI_Init(void)
{

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RED_LED_Pin|ORANGE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_LED_Pin ORANGE_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|ORANGE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_LED_Pin */
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_KEY_Pin */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


static  void  AppTaskStart (void *p_arg)
{
    CPU_INT32U   freq;
    OS_ERR       err;

   (void)p_arg;

    BSP_LED_Init(LED1);                                                      /* Initialize BSP functions                          */
    BSP_LED_Init(LED2);
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    
    BSP_SDRAM_Init();
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, (uint32_t) 0xC0000000);
    //BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_FOREGROUND, (uint32_t) 0xC0400000);
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER_BACKGROUND);
    BSP_LCD_DisplayOn();
    tcs34725_Init();
    
    
    CPU_Init();                                                       /* Initialize the uC/CPU services                    */
    freq = BSP_CPU_ClkFreq();                                         /* Determine SysTick reference freq.                 */                                                                        
    OS_CPU_SysTickInit(freq);                                         /* Init uC/OS periodic time src (SysTick).           */

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                                     /* Compute CPU capacity with no task running         */
#endif

#ifdef  CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

    AppSemCreate();                                                 /* Create Application Kernel objects                 */

    AppTaskCreate();                                                  /* Create application tasks                          */
    
    BSP_LED_Off(LED1);
    BSP_LED_Off(LED2);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4);

    OSTaskDel((OS_TCB *)0, &err);
}

/*
*********************************************************************************************************
*                                      App_SemCreate()
*
* Description : Create the application Mailboxes
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void AppSemCreate (void)
{
  OS_ERR err = OS_ERR_NONE;

  OSSemCreate(&Sem1, "Sem1", (OS_SEM_CTR)0, &err);
  OSSemCreate(&Sem2, "Sem2", (OS_SEM_CTR)0, &err);
  OSSemCreate(&Sem3, "Sem3", (OS_SEM_CTR)0, &err);
  OSSemCreate(&Sem4, "Sem4", (OS_SEM_CTR)0, &err);
  OSSemCreate(&Sem5, "Sem5", (OS_SEM_CTR)0, &err);

  /* Error check */
  if(OS_ERR_NONE != err)
  {
    /* An error has been ocurred during message queue initialization */
    while(1U);
  }
}
static  void  Process1 (void)
{
    OS_ERR err = OS_ERR_NONE;
    CPU_TS ts;
    CPU_INT08U cnt;

    while(1){
        OSSemPend(&Sem1, 0, OS_OPT_PEND_BLOCKING, &ts, &err);
        printf("Task1 started\r\n");
        cnt = LOOP_DELAY_CNT;
        while(cnt--)
        {
            BSP_LED_Toggle(LED1);
            APP_DELAY_MS(30);
        }
        printf("Task1 finished\r\n");
    }
}

static  void  Process2 (void)
{
    OS_ERR err = OS_ERR_NONE;
    CPU_TS ts;
    CPU_INT08U cnt;

    while(1){
		OSSemPend(&Sem2, 0, OS_OPT_PEND_BLOCKING, &ts, &err);
		printf("Task2 started\r\n");
		cnt = LOOP_DELAY_CNT;
		while(cnt--)
		{
		    BSP_LED_Toggle(LED2);
			APP_DELAY_MS(30);
		}
		printf("Task2 finished\r\n");
    }
}

static  void  Process3 (void)
{
    OS_ERR err = OS_ERR_NONE;
    CPU_TS ts;
    CPU_INT08U cnt;

    while(1){
        Touchscreen_demo1();
    }
}

static  void  Process4 (void)
{
    OS_ERR err = OS_ERR_NONE;
    CPU_TS ts;


    while(1){
        Touchscreen_Calibration();
        OSSemPend(&Sem4, 0, OS_OPT_PEND_BLOCKING, &ts, &err);
    }
}

static  void  Process5 (void)
{
    OS_ERR err = OS_ERR_NONE;
    CPU_TS ts;
    //CPU_INT08U str[40];
    
    while(1)
    {
        tcs34725_getRawData(&r, &g, &b, &c);
        printf("%6d,%6d,%6d,%6d\r\n", r,g,b,c);
        colorTemp = tcs34725_calculateColorTemperature(r, g, b);
        Lux = (uint32_t) tcs34725_calculateLux(r, g, b);
        OSSemPost(&Sem5, OS_OPT_POST_1 + OS_OPT_POST_NO_SCHED, &err);     
    }
}
static  void  Process6 (void)
{
    OS_ERR err = OS_ERR_NONE;
    CPU_TS ts;
    CPU_INT08U str[40];
    
    BSP_LCD_Clear(LCD_COLOR_DARKGRAY);
    BSP_LCD_SetFont(&Font28);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(0, 0, 130, 38);
    BSP_LCD_FillRect(0, 152, 130, 38);
    BSP_LCD_FillRect(0, 360, 272, 24);
    BSP_LCD_FillRect(0, 432, 68, 24);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAtLine(0, "Lux");
    BSP_LCD_DisplayStringAtLine(4, "Color");
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAtLine(15, "Integration Time");
    BSP_LCD_DisplayStringAtLine(18, "Gain");
    
    while(1)
    {
        OSSemPend(&Sem5, 0, OS_OPT_PEND_BLOCKING, &ts, &err);
        BSP_LCD_SetFont(&Font48);
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        BSP_LCD_SetBackColor(LCD_COLOR_DARKGRAY);
        sprintf(str, "          %6d", Lux);
        BSP_LCD_DisplayStringAtLine(1, str);
        sprintf(str, "          %6d", colorTemp);
        BSP_LCD_DisplayStringAtLine(4, str);
    
        switch (htcs34725_eval._integration)
        {

        case TCS34725_INTEGRATIONTIME_50MS:
        sprintf(str, "50 ms ");
        break;
        case TCS34725_INTEGRATIONTIME_101MS:
        sprintf(str, "101 ms");
        break;
        case TCS34725_INTEGRATIONTIME_154MS:
        sprintf(str, "154 ms");
        break;
        case TCS34725_INTEGRATIONTIME_700MS:
        sprintf(str, "700 ms");
        break;
        }
        
        BSP_LCD_SetFont(&Font24);
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        BSP_LCD_SetBackColor(LCD_COLOR_DARKGRAY);
        BSP_LCD_DisplayStringAtLine(16, str);

        switch (htcs34725_eval._gain)
        {
        case TCS34725_GAIN_1X:
        sprintf(str, "1X ");
        break;
        case TCS34725_GAIN_4X:
        sprintf(str, "4X ");
        break;
        case TCS34725_GAIN_16X:
        sprintf(str, "16X");
        break;
        case TCS34725_GAIN_60X:
        sprintf(str, "60X");
        break;
        }
        
        BSP_LCD_DisplayStringAtLine(19, str);
        
    }
}


/*
*********************************************************************************************************
*                                      AppTaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
  OS_ERR err = OS_ERR_NONE;


  /* Create the process1 task                         */
  OSTaskCreate((void           *)&Process1TCB,  /* pointer to TCB */
               (CPU_CHAR       *)"Process1", /* task name, text */
               (void (*)(void *)) Process1, /* function pointer to task */
               (void           *) 0U, /* pointer to optional data */
               (CPU_INT08U      ) APP_CFG_PROCESS1_TASK_PRIO, /* priority */
               (CPU_STK        *) App_Process1Stk, /* stack base ptr */
               (CPU_STK_SIZE    )(APP_CFG_TASK_STK_SIZE / 10U), /* stack limit */
               (CPU_STK_SIZE    ) APP_CFG_TASK_STK_SIZE, /* stack size */
               (OS_MSG_QTY      ) 0U, /* q size */
               (OS_TICK         ) 0U, /* round robin time tick */
               (void           *) 0U, /* p ext */
               (OS_OPT          )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), /* options */
               (OS_ERR         *)&err);

  /* Create the process2 task                         */
  OSTaskCreate((void           *)&Process2TCB,  /* pointer to TCB */
               (CPU_CHAR       *)"Process2", /* task name, text */
               (void (*)(void *)) Process2, /* function pointer to task */
               (void           *) 0U, /* pointer to optional data */
               (CPU_INT08U      ) APP_CFG_PROCESS2_TASK_PRIO, /* priority */
               (CPU_STK        *) App_Process2Stk, /* stack base ptr */
               (CPU_STK_SIZE    )(APP_CFG_TASK_STK_SIZE / 10U), /* stack limit */
               (CPU_STK_SIZE    ) APP_CFG_TASK_STK_SIZE, /* stack size */
               (OS_MSG_QTY      ) 0U, /* q size */
               (OS_TICK         ) 0U, /* round robin time tick */
               (void           *) 0U, /* p ext */
               (OS_OPT          )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), /* options */
               (OS_ERR         *)&err);

  /* Create the process3 task                         */
  OSTaskCreate((void           *)&Process3TCB,  /* pointer to TCB */
               (CPU_CHAR       *)"Process3", /* task name, text */
               (void (*)(void *)) Process3, /* function pointer to task */
               (void           *) 0U, /* pointer to optional data */
               (CPU_INT08U      ) APP_CFG_PROCESS3_TASK_PRIO, /* priority */
               (CPU_STK        *) App_Process3Stk, /* stack base ptr */
               (CPU_STK_SIZE    )(APP_CFG_TASK_STK_SIZE / 10U), /* stack limit */
               (CPU_STK_SIZE    ) APP_CFG_TASK_STK_SIZE, /* stack size */
               (OS_MSG_QTY      ) 0U, /* q size */
               (OS_TICK         ) 0U, /* round robin time tick */
               (void           *) 0U, /* p ext */
               (OS_OPT          )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), /* options */
               (OS_ERR         *)&err);

  /* Create the process4 task                         */
  OSTaskCreate((void           *)&Process4TCB,  /* pointer to TCB */
               (CPU_CHAR       *)"Process4", /* task name, text */
               (void (*)(void *)) Process4, /* function pointer to task */
               (void           *) 0U, /* pointer to optional data */
               (CPU_INT08U      ) APP_CFG_PROCESS4_TASK_PRIO, /* priority */
               (CPU_STK        *) App_Process4Stk, /* stack base ptr */
               (CPU_STK_SIZE    )(APP_CFG_TASK_STK_SIZE / 10U), /* stack limit */
               (CPU_STK_SIZE    ) APP_CFG_TASK_STK_SIZE, /* stack size */
               (OS_MSG_QTY      ) 0U, /* q size */
               (OS_TICK         ) 0U, /* round robin time tick */
               (void           *) 0U, /* p ext */
               (OS_OPT          )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), /* options */
               (OS_ERR         *)&err);
  
    OSTaskCreate((void           *)&Process5TCB,  /* pointer to TCB */
               (CPU_CHAR       *)"Process5", /* task name, text */
               (void (*)(void *)) Process5, /* function pointer to task */
               (void           *) 0U, /* pointer to optional data */
               (CPU_INT08U      ) APP_CFG_PROCESS5_TASK_PRIO, /* priority */
               (CPU_STK        *) App_Process5Stk, /* stack base ptr */
               (CPU_STK_SIZE    )(APP_CFG_TASK_STK_SIZE / 10U), /* stack limit */
               (CPU_STK_SIZE    ) APP_CFG_TASK_STK_SIZE, /* stack size */
               (OS_MSG_QTY      ) 0U, /* q size */
               (OS_TICK         ) 0U, /* round robin time tick */
               (void           *) 0U, /* p ext */
               (OS_OPT          )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), /* options */
               (OS_ERR         *)&err);     
    
        OSTaskCreate((void           *)&Process6TCB,  /* pointer to TCB */
               (CPU_CHAR       *)"Process6", /* task name, text */
               (void (*)(void *)) Process6, /* function pointer to task */
               (void           *) 0U, /* pointer to optional data */
               (CPU_INT08U      ) APP_CFG_PROCESS6_TASK_PRIO, /* priority */
               (CPU_STK        *) App_Process6Stk, /* stack base ptr */
               (CPU_STK_SIZE    )(APP_CFG_TASK_STK_SIZE / 10U), /* stack limit */
               (CPU_STK_SIZE    ) APP_CFG_TASK_STK_SIZE, /* stack size */
               (OS_MSG_QTY      ) 0U, /* q size */
               (OS_TICK         ) 0U, /* round robin time tick */
               (void           *) 0U, /* p ext */
               (OS_OPT          )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), /* options */
               (OS_ERR         *)&err);   
        
   /* Error check */
  if(OS_ERR_NONE != err)
  {
    /* An error has been ocurred */
    while(1U) ;
  }
}

static void tsl2591_Init(void)
{
	htsl2591_eval._initialized = false;
	htsl2591_eval._integration = TSL2591_INTEGRATIONTIME_100MS;
	htsl2591_eval._gain        = TSL2591_GAIN_MED;
}

static void tcs34725_Init(void)
{
	htcs34725_eval._initialized = false;
	htcs34725_eval._integration = TCS34725_INTEGRATIONTIME_101MS;
	htcs34725_eval._gain = TCS34725_GAIN_1X;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
