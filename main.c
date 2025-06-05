/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx.h"
#include "MY_CS43L22.h"
#include "stdio.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
I2S_HandleTypeDef                 hAudioOutI2s;
const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t I2SPLLN[8] = {256, 429, 213, 429, 426, 271, 258, 344};
const uint32_t I2SPLLR[8] = {5, 4, 4, 4, 4, 6, 3, 1};
bool dma_ready = true;
//AUDIO_DrvTypeDef           *pAudioDrv;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t CS43L22_ReadID(void);
#define AUDIO_RESET_PIN GPIO_PIN_4
#define AUDIO_RESET_GPIO GPIOD
#define AUDIO_I2C_ADDRESS 0x94



// CS43L22 register addresses
#define CS43L22_REG_ID          0x01
#define CS43L22_REG_POWER_CTL1  0x02
#define CS43L22_REG_POWER_CTL2  0x04
#define CS43L22_REG_INTERFACE   0x06
#define CS43L22_REG_MISC        0x0E
#define CS43L22_REG_MASTER_A    0x20
#define CS43L22_REG_MASTER_B    0x21
#define CS43L22_REG_HEADPHONE_A 0x22
#define CS43L22_REG_HEADPHONE_B 0x23


#define SIN_FREQ        1000
#define SAMPLING_RATE   48000
#define BUFFER_LENGTH   (SAMPLING_RATE / SIN_FREQ)
#define PI 3.14159f
#define F_SAMPLE	96000.0f
#define F_OUT		2000.0f
#define PI 			3.14159f
#define MAX_SAMPLE_N 512
#define ADC_BUFFER_SIZE 128
#define NUM_BUFFERS 2
#define AUDIO_SAMPLE_RATE 48000
#define AUDIO_FREQUENCY     48000U
#define SAMPLES_PER_MS     48
#define AUDIO_BUFFER_SIZE  128
#define ADC_SAMPLE_TIME    ADC_SAMPLETIME_15CYCLES

#define PI 3.14159f
#define SAMPLE_RATE 48000
#define TONE_FREQUENCY 440.0f
#define BUFFER_SIZE 1024
#define SINE_FREQUENCY 1000.0f
#define AMPLITUDE 32767.0f

#define ADC_RING_BUFFER_SIZE 1024
#define SAMPLE_WINDOW 48000

#define ADC_BUFFER_SIZE 1024
uint16_t adc_dma_buffer[ADC_BUFFER_SIZE];
volatile uint8_t adc_buffer_half = 0;
volatile uint8_t adc_buffer_full = 0;

float sine_phase = 0.0f;
float sine_phase_increment = 2.0f * PI * SINE_FREQUENCY / SAMPLE_RATE;
volatile bool codec_initialized = false;
uint16_t dataI2Sy[NUM_BUFFERS][ADC_BUFFER_SIZE];
volatile uint8_t current_buffer = 0;
int16_t buffer_audio[2 * BUFFER_LENGTH];
uint16_t adc_buffer[ADC_BUFFER_SIZE];
    float processed_sample;
uint16_t dataI2S[ADC_BUFFER_SIZE * 2];
float mySinVal;
float sample_dt;
uint16_t sample_N;
uint16_t i_t;
volatile uint32_t adc_overflow_count = 0;
volatile uint32_t i2s_overflow_count = 0;
volatile bool adc_buffer_ready = false;
ALIGN_32BYTES(uint16_t adc_buffer[AUDIO_BUFFER_SIZE]);
ALIGN_32BYTES(uint16_t dataI2S[AUDIO_BUFFER_SIZE * 2]);
volatile uint32_t buffer_state = 0;

int16_t audioBuf[2][AUDIO_BUFFER_SIZE * 2];
volatile uint8_t currentBuf = 0;
volatile uint8_t current_buf = 0;
volatile uint8_t bufReady = 1;
volatile uint8_t buf_ready = 1;
volatile uint32_t errorCount = 0;
#define SMOOTH_SIZE 4
static inline int16_t convertADCToAudio(uint16_t adcValue);

uint16_t fastReadADC_SPI(uint8_t channel);

int16_t audioBuffer1[AUDIO_BUFFER_SIZE * 2];
int16_t audioBuffer2[AUDIO_BUFFER_SIZE * 2];

int16_t audioBuffer[2][BUFFER_SIZE * 2];
volatile uint8_t currentBuffer = 0;

#define SPI_CLOCK_CYCLES 10

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_buffer_half = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_buffer_full = 1;
    }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s == &hi2s3) {
        for(int i = 0; i < BUFFER_SIZE/2; i++) {
            int16_t audio_sample = ((int16_t)adc_dma_buffer[i] - 2048) * 16;

            audioBuffer[currentBuffer][i*2] = audio_sample;
            audioBuffer[currentBuffer][i*2 + 1] = audio_sample;
        }
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s == &hi2s3) {
        for(int i = BUFFER_SIZE/2; i < BUFFER_SIZE; i++) {
            int16_t audio_sample = ((int16_t)adc_dma_buffer[i] - 2048) * 16;

            audioBuffer[currentBuffer][i*2] = audio_sample;
            audioBuffer[currentBuffer][i*2 + 1] = audio_sample;
        }
        currentBuffer ^= 1;
    }
}

void initADC_Guitar(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;

    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;

    hadc1.Init.Resolution = ADC_RESOLUTION_12B;

    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;

    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;

    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;

    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    HAL_ADC_Init(&hadc1);

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;

    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    ADC->CCR |= ADC_CCR_DMA_1;
}

void processGuitarSamples(void)
{
    static float dc_bias = 2048.0f;
    static float envelope = 0.0f;
    const float dc_alpha = 0.0001f;
    const float env_alpha = 0.1f;

    if (adc_buffer_half)
    {
        adc_buffer_half = 0;
        for(int i = 0; i < ADC_BUFFER_SIZE/2; i++)
        {
            float sample = (float)adc_dma_buffer[i];

            // Update DC bias (very slow to avoid affecting guitar signal)
            dc_bias = (dc_bias * (1.0f - dc_alpha)) + (sample * dc_alpha);

            // Remove DC bias
            float centered = sample - dc_bias;

            float abs_sample = fabsf(centered);
            envelope = (envelope * (1.0f - env_alpha)) + (abs_sample * env_alpha);

            if(envelope < 10.0f) centered = 0.0f;

            // Convert to 16-bit audio with gain
            int16_t audio_sample = (int16_t)(centered * 16.0f);

            // Fill audio buffer
            audioBuf[currentBuf][i*2] = audio_sample;
            audioBuf[currentBuf][i*2 + 1] = audio_sample;
        }
    }

    if (adc_buffer_full)
    {
        adc_buffer_full = 0;
        for(int i = ADC_BUFFER_SIZE/2; i < ADC_BUFFER_SIZE; i++)
        {
            float sample = (float)adc_dma_buffer[i];
            dc_bias = (dc_bias * (1.0f - dc_alpha)) + (sample * dc_alpha);
            float centered = sample - dc_bias;

            float abs_sample = fabsf(centered);
            envelope = (envelope * (1.0f - env_alpha)) + (abs_sample * env_alpha);

            if(envelope < 10.0f) centered = 0.0f;

            int16_t audio_sample = (int16_t)(centered * 16.0f);
            audioBuf[currentBuf][i*2] = audio_sample;
            audioBuf[currentBuf][i*2 + 1] = audio_sample;
        }
    }
}

void monitorSignalLevels(void) {
    static uint32_t lastUpdate = 0;
    static float peak_level = 0.0f;

    if (HAL_GetTick() - lastUpdate >= 500) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Peak:%4d", (int)peak_level);
        SSD1306_Clear(&hi2c2);
        SSD1306_WriteString(&hi2c2, 0, 0, buf);
        SSD1306_UpdateScreen(&hi2c2);
        HAL_Delay(10);

        peak_level = 0.0f;
        lastUpdate = HAL_GetTick();
    }

    for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
        float level = fabsf((float)adc_dma_buffer[i] - 2048.0f);
        if(level > peak_level) peak_level = level;
    }
}

void startAudioCapture(void) {
    HAL_TIM_Base_Start(&htim2);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUFFER_SIZE);

    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)audioBuffer[0], BUFFER_SIZE * 2);
}


void initAudioDMA(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    uint32_t i2sclk = HAL_RCC_GetPCLK1Freq() * 2;
    hi2s3.Init.AudioFreq = 48000;

    prepareNextAudioBuffer(audioBuffer[0], BUFFER_SIZE);
    prepareNextAudioBuffer(audioBuffer[1], BUFFER_SIZE);

    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)audioBuffer[0], BUFFER_SIZE * 2);
}

#define SPI_TIMEOUT_US 10



void initAudio(void) {
	    uint8_t cmd;
	    HAL_Delay(10);

	    cmd = 0x99;
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
	    HAL_Delay(10);

	    // Power management and misc settings (Register 0x47)
	    cmd = 0x80;
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x47, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
	    HAL_Delay(10);

	    // Beware bit 7 in register 0x32 sequence
	    cmd = 0x80;
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x32, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
	    HAL_Delay(10);

	    cmd = 0x00;  // Clear bit 7
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x32, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
	    HAL_Delay(10);

	    // Final power up (Register 0x00)
	    cmd = 0x00;
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
	    HAL_Delay(10);

	    // Additional configuration for audio output
	    // Interface Control (Register 0x06)
	    cmd = 0x07;  // I2S, 16-bit
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x06, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);

	    // Headphone volume control (Registers 0x22, 0x23)
	    cmd = 0x00;  // 0dB, no attenuation
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x22, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x23, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);

	    // Power management (Register 0x02)
	    cmd = 0x9E;  // Power up all components
	    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x02, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);

	    // Verify configuration
	    uint8_t readBack;
	    HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x02, I2C_MEMADD_SIZE_8BIT, &readBack, 1, 100);

	    // Debug display
	    char buf[32];
	    SSD1306_Clear(&hi2c2);
	    snprintf(buf, sizeof(buf), "Init: 0x%02X", readBack);
	    SSD1306_WriteString(&hi2c2, 0, 0, buf);
	    SSD1306_UpdateScreen(&hi2c2);
	    HAL_Delay(1000);
}

void initCS43L22_DiscoveryBoard() {
    uint8_t cmd;
    char debug[32];

    SSD1306_Clear(&hi2c2);
    SSD1306_WriteString(&hi2c2, 0, 0, "Init CS43L22...");
    SSD1306_UpdateScreen(&hi2c2);

    // 1. Initial hardware reset
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(5);

    // 2. Required initialization sequence
    cmd = 0x99;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_Delay(1);

    cmd = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x47, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_Delay(1);

    cmd = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x32, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_Delay(1);

    cmd = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x32, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_Delay(1);

    cmd = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_Delay(1);

    // 3. Configure clocking
    cmd = 0x81;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x05, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // Read back and display clock status
    uint8_t clock_reg;
    HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x05, I2C_MEMADD_SIZE_8BIT, &clock_reg, 1, 10);
    snprintf(debug, sizeof(debug), "CLK: 0x%02X", clock_reg);
    SSD1306_WriteString(&hi2c2, 0, 8, debug);
    SSD1306_UpdateScreen(&hi2c2);

    // 4. Configure I2S interface
    cmd = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x06, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // 5. Configure power control 2
    cmd = 0xAF;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x04, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // 6. Configure zero-crossing
    cmd = 0x06;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x0E, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // 7. Set initial volume
    cmd = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x20, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x21, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // 8. Set headphone volume
    cmd = 0x60;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x22, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x23, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // 9. Power up DAC
    cmd = 0x9E;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x02, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_Delay(1);

    // 10. Unmute and set volume
    cmd = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x1A, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x1B, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // Read and display status register (0x2E)
    uint8_t status;
    HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x2E, I2C_MEMADD_SIZE_8BIT, &status, 1, 10);

    // Break down status bits
    snprintf(debug, sizeof(debug), "Status: 0x%02X", status);
    SSD1306_WriteString(&hi2c2, 0, 16, debug);

    // Decode specific status bits
    snprintf(debug, sizeof(debug), "CLK:%d OVF:%d",
        (status & 0x40) >> 6,    // SPCLKERR bit
        (status & 0x3C) >> 2     // Various overflow bits
    );
    SSD1306_WriteString(&hi2c2, 0, 24, debug);
    SSD1306_UpdateScreen(&hi2c2);

    cmd = 0xA0;  // Around -30dB
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x20, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x21, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // Set headphone volume lower
    cmd = 0xA0;  // Around -30dB
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x22, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x23, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // Make sure PCM channel is unmuted and at reasonable volume
    cmd = 0x00;  // Unmuted, 0dB
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x1A, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x1B, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    cmd = 0x60;  // Default gain, no inversions
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x0D, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    // Playback Control 2 (Register 0x0F)
    cmd = 0x00;  // No mutes, normal stereo operation
    HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x0F, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

    uint8_t reg05, reg06;
    HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x05, I2C_MEMADD_SIZE_8BIT, &reg05, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x06, I2C_MEMADD_SIZE_8BIT, &reg06, 1, 10);

    // Explicitly set volume to maximum
   cmd = 0x00;  // 0dB, maximum volume
   HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x20, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
   HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x21, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

   // Unmute both channels
   cmd = 0x00;
   HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x1A, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
   HAL_I2C_Mem_Write(&hi2c1, 0x94, 0x1B, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);

   // Additional debug
   uint8_t volume_a, volume_b, mute_a, mute_b;
   HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x20, I2C_MEMADD_SIZE_8BIT, &volume_a, 1, 10);
   HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x21, I2C_MEMADD_SIZE_8BIT, &volume_b, 1, 10);
   HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x1A, I2C_MEMADD_SIZE_8BIT, &mute_a, 1, 10);
   HAL_I2C_Mem_Read(&hi2c1, 0x94, 0x1B, I2C_MEMADD_SIZE_8BIT, &mute_b, 1, 10);


   SSD1306_Clear(&hi2c2);
   snprintf(debug, sizeof(debug), "Vol A:0x%02X B:0x%02X", volume_a, volume_b);
   SSD1306_WriteString(&hi2c2, 0, 0, debug);
   snprintf(debug, sizeof(debug), "Mute A:0x%02X B:0x%02X", mute_a, mute_b);
   SSD1306_WriteString(&hi2c2, 0, 8, debug);
   SSD1306_UpdateScreen(&hi2c2);
   HAL_Delay(2000);

    char buf[32];
    SSD1306_Clear(&hi2c2);
    snprintf(buf, sizeof(buf), "Reg05: 0x%02X", reg05);
    SSD1306_WriteString(&hi2c2, 0, 0, buf);
    snprintf(buf, sizeof(buf), "Reg06: 0x%02X", reg06);
    SSD1306_WriteString(&hi2c2, 0, 8, buf);
    SSD1306_UpdateScreen(&hi2c2);

    HAL_Delay(10);

    codec_initialized = true;
}

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize OLED
     SSD1306_Init(&hi2c2);
     SSD1306_Clear(&hi2c2);
     SSD1306_WriteString(&hi2c2, 0, 0, "Starting...");
     SSD1306_UpdateScreen(&hi2c2);

     // Initialize I2S
     if(HAL_I2S_Init(&hi2s3) != HAL_OK) {
         SSD1306_WriteString(&hi2c2, 0, 8, "I2S Init Err");
         SSD1306_UpdateScreen(&hi2c2);
         while(1);
     }
     initADC_Guitar();
     initCS43L22_DiscoveryBoard();
     HAL_TIM_Base_Start(&htim2);
     HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUFFER_SIZE);
     HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)audioBuffer[0], BUFFER_SIZE * 2);
     // Initialize audio codec

     CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
     DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
     DWT->CYCCNT = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  processGuitarSamples();  // Process ADC samples
	  monitorSignalLevels();   // Optional: monitor signal levels on OLED
	  HAL_Delay(1);           // Small delay to prevent overload
	      //}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  //}
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  char spi_status[32];
  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
      snprintf(spi_status, sizeof(spi_status), "SPI OK");
  } else {
      snprintf(spi_status, sizeof(spi_status), "SPI ERR");
  }
  SSD1306_WriteString(&hi2c2, 0, 0, spi_status);
  SSD1306_UpdateScreen(&hi2c2);
  HAL_Delay(2000);  // Show for 2 seconds
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1749;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD15 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
