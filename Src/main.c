/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention1
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "sai.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    BUFFER_OFFSET_NONE = 0,
    BUFFER_OFFSET_HALF = 1,
    BUFFER_OFFSET_FULL = 2,
} BUFFER_StateTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RECORD_BUFFER_SIZE  4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
SAI_HandleTypeDef haudio_out_sai, haudio_in_sai;
static AUDIO_DrvTypeDef  *audio_drv;
volatile uint32_t  audio_rec_buffer_state;
volatile uint32_t  audio_tx_buffer_state = 0;



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t RecordBuffer[RECORD_BUFFER_SIZE];
int16_t PlaybackBuffer[RECORD_BUFFER_SIZE];
uint8_t choice = 1;
uint8_t komunikt[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t BSP_AUDIO_Init(uint32_t AudioFreq);
uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
static uint8_t _BSP_AUDIO_OUT_Play(uint16_t* pBuffer, uint32_t Size);
void AUDIO_LOOPBACK(void);
static void SAI_AUDIO_IN_MspInit(SAI_HandleTypeDef *hsai, void *Params);
static void SAIx_In_Init(uint32_t AudioFreq);
static void SAIx_In_DeInit(void);
static void CopyBuffer(int16_t *pbuffer1, int16_t *pbuffer2, uint16_t BufferSize);
void get_max_val(int16_t *buf, uint32_t size, int16_t amp[]);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void recive_audio_task(void)
{
	vTaskDelay(1000/portTICK_RATE_MS);
    HAL_StatusTypeDef res = HAL_SAI_Receive_DMA(&haudio_in_sai, (uint8_t*)RecordBuffer, RECORD_BUFFER_SIZE);
    if (HAL_OK == res)
    {
   	 printf("SAI receive begin OK\r\n");
    } else {
        printf("SAI receive error: %d\r\n", res);
    }

    printf("Copying Record buffer to Playback buffer\r\n");

    for(;;);


}


void transmit_audio_task(void)
{
         int16_t amp[4];

         /* Play the recorded buffer */
         if (_BSP_AUDIO_OUT_Play((uint16_t *) &PlaybackBuffer[0], RECORD_BUFFER_SIZE) == AUDIO_OK)
         {
        	 printf("Audio output OK\r\n");
         } else {
        	 printf("Audio output error\r\n");
     	 }
         printf("\r\n");

         audio_rec_buffer_state = BUFFER_OFFSET_NONE;
         while (1)
         {
             /* 1st or 2nd half of the record buffer ready for being copied
             to the Playback buffer */
             if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
             {
                 /* Copy half of the record buffer to the playback buffer */
                 if (audio_rec_buffer_state == BUFFER_OFFSET_HALF)
                 {
                     get_max_val(RecordBuffer, RECORD_BUFFER_SIZE / 2, amp);
                     CopyBuffer(&PlaybackBuffer[0], &RecordBuffer[0], RECORD_BUFFER_SIZE / 2);
                 } else {
                	 /* if(audio_rec_buffer_state == BUFFER_OFFSET_FULL)*/
                     CopyBuffer(&PlaybackBuffer[RECORD_BUFFER_SIZE / 2],
                    		      &RecordBuffer[RECORD_BUFFER_SIZE / 2],
								                RECORD_BUFFER_SIZE / 2);
                 }
                 /* Wait for next data */
                 audio_rec_buffer_state = BUFFER_OFFSET_NONE;
             }
             if (audio_tx_buffer_state)
             {
                 audio_tx_buffer_state = 0;
             }
         } // end while(1)
 } // end AUDIO_LOOPBACK





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_USART1_UART_Init();
  MX_FREERTOS_Init();
  osKernelStart();
  /* USER CODE BEGIN 2 */


  BSP_AUDIO_IN_Init(SAI_AUDIO_FREQUENCY_44K, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_BOTH, 100, SAI_AUDIO_FREQUENCY_44K);
  AUDIO_LOOPBACK();
  xTaskCreate( recive_audio_task, "Recive_audio_task", 100, NULL, 1, NULL );
  xTaskCreate( transmit_audio_task, "transmit_audio_task", 100, NULL, 1, NULL );
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */

  /* Start scheduler */


  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static uint8_t BSP_AUDIO_Init(uint32_t AudioFreq)
     {
         uint8_t ret = AUDIO_ERROR;

         /* Disable SAI */
         SAIx_In_DeInit();

         /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */
         BSP_AUDIO_OUT_ClockConfig(&haudio_in_sai, AudioFreq, NULL);
         haudio_out_sai.Instance = AUDIO_OUT_SAIx;
         haudio_in_sai.Instance = AUDIO_IN_SAIx;
         if (HAL_SAI_GetState(&haudio_in_sai) == HAL_SAI_STATE_RESET)
         {
             BSP_AUDIO_OUT_MspInit(&haudio_out_sai, NULL);
             SAI_AUDIO_IN_MspInit(&haudio_in_sai, NULL);
         }


         SAIx_In_Init(AudioFreq); // inclu déja le code de SAIx_Out_Init()


         if ((wm8994_drv.ReadID(AUDIO_I2C_ADDRESS)) == WM8994_ID)
         {
             /* Reset the Codec Registers */
             wm8994_drv.Reset(AUDIO_I2C_ADDRESS);
             /* Initialize the audio driver structure */
             audio_drv = &wm8994_drv;
             ret = AUDIO_OK;
         } else {
             ret = AUDIO_ERROR;
         }

         if (ret == AUDIO_OK)
         {
             /* Initialize the codec internal registers */
             audio_drv->Init(AUDIO_I2C_ADDRESS, INPUT_DEVICE_ANALOG_MIC | OUTPUT_DEVICE_HEADPHONE , 100, AudioFreq);
         }

         /* Return AUDIO_OK when all operations are correctly done */
         return ret;
     }


     static uint8_t _BSP_AUDIO_OUT_Play(uint16_t* pBuffer, uint32_t Size)
     {
         /* Call the audio Codec Play function */
         if (audio_drv->Play(AUDIO_I2C_ADDRESS, (uint16_t *)pBuffer, Size) != 0)
         {
             return AUDIO_ERROR;
         }
         else
         {
             /* Update the Media layer and enable it for play */
             //if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*) pBuffer, DMA_MAX(Size / AUDIODATA_SIZE)) !=  HAL_OK)
             if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*) pBuffer, Size) !=  HAL_OK)
                 return AUDIO_ERROR;
             return AUDIO_OK;
         }
     }





void AUDIO_LOOPBACK(void)
{
         int16_t amp[4];

         /* Initialize Audio Recorder with 4 channels to be used */
         if (BSP_AUDIO_Init(SAI_AUDIO_FREQUENCY_44K) == AUDIO_OK)
         {
        	 printf("Audio I/O initialization OK\r\n");
         } else {
        	 printf("Audio I/O initialization failed.\r\n");
         }

         /* Start Recording */
         /*HAL_StatusTypeDef res = HAL_SAI_Receive_DMA(&haudio_in_sai, (uint8_t*)RecordBuffer, RECORD_BUFFER_SIZE);
         if (HAL_OK == res)
         {
        	 printf("SAI receive begin OK\r\n");
         } else {
             printf("SAI receive error: %d\r\n", res);
         }

         printf("Copying Record buffer to Playback buffer\r\n");
			*/
         /* Play the recorded buffer */
         	// end while(1)
 } // end AUDIO_LOOPBACK

static void CopyBuffer(int16_t *pbuffer1, int16_t *pbuffer2, uint16_t BufferSize)
{
    uint32_t i = 0;
    for (i = 0; i < BufferSize; i++)
    {
        pbuffer1[i] = pbuffer2[i];
    }
}

void get_max_val(int16_t *buf, uint32_t size, int16_t amp[])
{
    int16_t maxval[4] = { -32768, -32768, -32768, -32768};
    uint32_t idx;
    for (idx = 0 ; idx < size ; idx += 4) {
        if (buf[idx] > maxval[0])
            maxval[0] = buf[idx];
        if (buf[idx + 1] > maxval[1])
            maxval[1] = buf[idx + 1];
        if (buf[idx + 2] > maxval[2])
            maxval[2] = buf[idx + 2];
        if (buf[idx + 3] > maxval[3])
            maxval[3] = buf[idx + 3];
    }
    memcpy(amp, maxval, sizeof(maxval));
}

static void SAIx_In_Init(uint32_t AudioFreq)
     {
         /* Initialize SAI1 block A in MASTER TX */
         /* Initialize the haudio_out_sai Instance parameter */
         haudio_out_sai.Instance = AUDIO_OUT_SAIx;

         /* Disable SAI peripheral to allow access to SAI internal registers */
         __HAL_SAI_DISABLE(&haudio_out_sai);

         /* Configure SAI_Block_x */
         haudio_out_sai.Init.MonoStereoMode = SAI_STEREOMODE;
         haudio_out_sai.Init.AudioFrequency = AudioFreq;
         haudio_out_sai.Init.AudioMode      = SAI_MODEMASTER_TX;
         haudio_out_sai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
         haudio_out_sai.Init.Protocol       = SAI_FREE_PROTOCOL;
         haudio_out_sai.Init.DataSize       = SAI_DATASIZE_16;
         haudio_out_sai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
         haudio_out_sai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
         haudio_out_sai.Init.Synchro        = SAI_ASYNCHRONOUS;
         haudio_out_sai.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
         haudio_out_sai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
         haudio_out_sai.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
         haudio_out_sai.Init.CompandingMode = SAI_NOCOMPANDING;
         haudio_out_sai.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
         haudio_out_sai.Init.Mckdiv         = 0;

         /* Configure SAI_Block_x Frame */
         haudio_out_sai.FrameInit.FrameLength       = 64;
         haudio_out_sai.FrameInit.ActiveFrameLength = 32;
         haudio_out_sai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
         haudio_out_sai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
         haudio_out_sai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

         /* Configure SAI Block_x Slot */
         haudio_out_sai.SlotInit.FirstBitOffset = 0;
         haudio_out_sai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
         haudio_out_sai.SlotInit.SlotNumber     = 4;
         haudio_out_sai.SlotInit.SlotActive     = CODEC_AUDIOFRAME_SLOT_0123;

         HAL_SAI_Init(&haudio_out_sai);



         /* Initialize SAI1 block B in SLAVE RX synchronous from SAI1 block A */
         /* Initialize the haudio_in_sai Instance parameter */
         haudio_in_sai.Instance = AUDIO_IN_SAIx;

         /* Disable SAI peripheral to allow access to SAI internal registers */
         __HAL_SAI_DISABLE(&haudio_in_sai);

         /* Configure SAI_Block_x */
         haudio_in_sai.Init.MonoStereoMode = SAI_STEREOMODE;
         haudio_in_sai.Init.AudioFrequency = AudioFreq;
         haudio_in_sai.Init.AudioMode      = SAI_MODESLAVE_RX;
         haudio_in_sai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
         haudio_in_sai.Init.Protocol       = SAI_FREE_PROTOCOL;
         haudio_in_sai.Init.DataSize       = SAI_DATASIZE_16;
         haudio_in_sai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
         haudio_in_sai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
         haudio_in_sai.Init.Synchro        = SAI_SYNCHRONOUS;
         haudio_in_sai.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
         haudio_in_sai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
         haudio_in_sai.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
         haudio_in_sai.Init.CompandingMode = SAI_NOCOMPANDING;
         haudio_in_sai.Init.TriState       = SAI_OUTPUT_RELEASED;
         haudio_in_sai.Init.Mckdiv         = 0;

         /* Configure SAI_Block_x Frame */
         haudio_in_sai.FrameInit.FrameLength       = 64;
         haudio_in_sai.FrameInit.ActiveFrameLength = 32;
         haudio_in_sai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
         haudio_in_sai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
         haudio_in_sai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

         /* Configure SAI Block_x Slot */
         haudio_in_sai.SlotInit.FirstBitOffset = 0;
         haudio_in_sai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
         haudio_in_sai.SlotInit.SlotNumber     = 4;
         haudio_in_sai.SlotInit.SlotActive     = CODEC_AUDIOFRAME_SLOT_0123;

         HAL_SAI_Init(&haudio_in_sai);

         /* Enable SAI peripheral */
         __HAL_SAI_ENABLE(&haudio_in_sai);

         /* Enable SAI peripheral to generate MCLK */
         __HAL_SAI_ENABLE(&haudio_out_sai);
     }



static void SAIx_In_DeInit(void)
{
    /* Initialize the haudio_in_sai Instance parameter */
    haudio_in_sai.Instance = AUDIO_IN_SAIx;
    haudio_out_sai.Instance = AUDIO_OUT_SAIx;
    /* Disable SAI peripheral */
    __HAL_SAI_DISABLE(&haudio_in_sai);

    HAL_SAI_DeInit(&haudio_in_sai);
    HAL_SAI_DeInit(&haudio_out_sai);
}

static void SAI_AUDIO_IN_MspInit(SAI_HandleTypeDef *hsai, void *Params)
   {
       static DMA_HandleTypeDef hdma_sai_rx;
       GPIO_InitTypeDef  gpio_init_structure;

       /* Enable SAI clock */
       AUDIO_IN_SAIx_CLK_ENABLE();

       /* Enable SD GPIO clock */
       AUDIO_IN_SAIx_SD_ENABLE();
       /* CODEC_SAI pin configuration: SD pin */
       gpio_init_structure.Pin = AUDIO_IN_SAIx_SD_PIN;
       gpio_init_structure.Mode = GPIO_MODE_AF_PP;
       gpio_init_structure.Pull = GPIO_NOPULL;
       gpio_init_structure.Speed = GPIO_SPEED_FAST;
       gpio_init_structure.Alternate = AUDIO_IN_SAIx_AF;
       HAL_GPIO_Init(AUDIO_IN_SAIx_SD_GPIO_PORT, &gpio_init_structure);

       /* Enable Audio INT GPIO clock */
       AUDIO_IN_INT_GPIO_ENABLE();
       /* Audio INT pin configuration: input */
       gpio_init_structure.Pin = AUDIO_IN_INT_GPIO_PIN;
       gpio_init_structure.Mode = GPIO_MODE_INPUT;
       gpio_init_structure.Pull = GPIO_NOPULL;
       gpio_init_structure.Speed = GPIO_SPEED_FAST;
       HAL_GPIO_Init(AUDIO_IN_INT_GPIO_PORT, &gpio_init_structure);

       /* Enable the DMA clock */
       AUDIO_IN_SAIx_DMAx_CLK_ENABLE();

       if (hsai->Instance == AUDIO_IN_SAIx)
       {
           /* Configure the hdma_sai_rx handle parameters */
           hdma_sai_rx.Init.Channel             = AUDIO_IN_SAIx_DMAx_CHANNEL;
           hdma_sai_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
           hdma_sai_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
           hdma_sai_rx.Init.MemInc              = DMA_MINC_ENABLE;
           hdma_sai_rx.Init.PeriphDataAlignment = AUDIO_IN_SAIx_DMAx_PERIPH_DATA_SIZE;
           hdma_sai_rx.Init.MemDataAlignment    = AUDIO_IN_SAIx_DMAx_MEM_DATA_SIZE;
           hdma_sai_rx.Init.Mode                = DMA_CIRCULAR;
           hdma_sai_rx.Init.Priority            = DMA_PRIORITY_HIGH;
           hdma_sai_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
           hdma_sai_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
           hdma_sai_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
           hdma_sai_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;

           hdma_sai_rx.Instance = AUDIO_IN_SAIx_DMAx_STREAM;

           /* Associate the DMA handle */
           __HAL_LINKDMA(hsai, hdmarx, hdma_sai_rx);

           /* Deinitialize the Stream for new transfer */
           HAL_DMA_DeInit(&hdma_sai_rx);

           /* Configure the DMA Stream */
           HAL_DMA_Init(&hdma_sai_rx);
       }

       /* SAI DMA IRQ Channel configuration */
       HAL_NVIC_SetPriority(AUDIO_IN_SAIx_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
       HAL_NVIC_EnableIRQ(AUDIO_IN_SAIx_DMAx_IRQ);

       /* Audio INT IRQ Channel configuration */
       HAL_NVIC_SetPriority(AUDIO_IN_INT_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
       HAL_NVIC_EnableIRQ(AUDIO_IN_INT_IRQ);
   }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{


}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		if(choice == 'a')
		{
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
			sprintf(komunikt,"DIODA ON");
		}
		else
		{
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
			sprintf(komunikt,"DIODA OFF");
		}
		HAL_UART_Transmit_IT(&huart1, komunikt, 10);
		HAL_UART_Receive_IT(&huart1, &choice, 1);
	}

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
