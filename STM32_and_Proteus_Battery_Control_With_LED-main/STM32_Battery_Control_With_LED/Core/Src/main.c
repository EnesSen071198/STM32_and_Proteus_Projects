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
// Gerekli ek kütüphaneler buraya eklenebilir
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Özel tip tanımlamaları buraya eklenebilir
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MAX_VALUE       4095.0f  // 12-bit ADC için maksimum değer (STM32F103)
#define VREF_VOLTAGE        3.3f     // STM32'nin dahili referans voltajı (genellikle 3.3V)

// Pil voltajı toleransları (gerçek pil voltajları için, 6V için voltaj bölücü sonrası okunan değeri temsil eder)
#define V_3V_MIN            2.8f     // 3V pil için alt tolerans sınırı
#define V_3V_MAX            3.2f     // 3V pil için üst tolerans sınırı
#define V_1V_MIN            0.8f     // 1V pil için alt tolerans sınırı
#define V_1V_MAX            1.2f     // 1V pil için üst tolerans sınırı
#define V_6V_MIN            2.8f     // 6V pil için alt tolerans sınırı (voltaj bölücü sonrası ~3V'a karşılık)
#define V_6V_MAX            3.2f     // 6V pil için üst tolerans sınırı (voltaj bölücü sonrası ~3V'a karşılık)

// LED pin tanımlamaları (Sizin tercihlerinize göre: Yeşil: PA3, Mavi: PA4, Kırmızı: PA5)
#define GREEN_LED_PIN       GPIO_PIN_3  // PA3 - 3V Pil için
#define BLUE_LED_PIN        GPIO_PIN_4  // PA4 - 1V Pil için
#define RED_LED_PIN         GPIO_PIN_5  // PA5 - 6V Pil için

#define LED_GPIO_PORT       GPIOA       // Tüm LED'ler GPIOA portunda

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Özel makrolar buraya eklenebilir
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1; // ADC1 için HAL tanıtıcısı

/* USER CODE BEGIN PV */
// Özel değişkenler buraya eklenebilir
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // Sistem saati yapılandırma fonksiyonu
static void MX_GPIO_Init(void); // GPIO pinlerini başlatma fonksiyonu
static void MX_ADC1_Init(void); // ADC1 birimini başlatma fonksiyonu
/* USER CODE BEGIN PFP */
// Özel fonksiyon prototipleri buraya eklenebilir
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Kullanıcı tanımlı özel kod blokları buraya eklenebilir
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // main fonksiyonunun başlangıcı için kullanıcı kodu
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // HAL kütüphanesini başlat

  /* USER CODE BEGIN Init */
  // Çevre birimi başlatmadan önce kullanıcı kodu
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // Sistem saatini yapılandır

  /* USER CODE BEGIN SysInit */
  // Sistem başlatıldıktan sonra kullanıcı kodu
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); // GPIO pinlerini başlat
  MX_ADC1_Init(); // ADC1 birimini başlat
  /* USER CODE BEGIN 2 */
  // Çevre birimleri başlatıldıktan sonra kullanıcı kodu
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t adc_raw_value_ch0; // PA0'dan okunan ham ADC değeri (3V pil)
  uint32_t adc_raw_value_ch1; // PA1'den okunan ham ADC değeri (1V pil)
  uint32_t adc_raw_value_ch2; // PA2'den okunan ham ADC değeri (6V pil - voltaj bölücü sonrası)
  float voltage_3v_battery;   // 3V pilin okunacak voltaj değeri
  float voltage_1v_battery;   // 1V pilin okunacak voltaj değeri
  float voltage_6v_battery;   // 6V pilin okunacak voltaj değeri (voltaj bölücü sonrası)

  while (1)
  {
    // ADC dönüşümlerini başlat (Scan Mode etkin olduğu için 3 kanalı sırayla okuyacak)
    HAL_ADC_Start(&hadc1);

    // İlk kanalın (PA0 - 3V pil) dönüşümünün tamamlanmasını bekle
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_raw_value_ch0 = HAL_ADC_GetValue(&hadc1); // PA0 değerini al

    // İkinci kanalın (PA1 - 1V pil) dönüşümünün tamamlanmasını bekle
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_raw_value_ch1 = HAL_ADC_GetValue(&hadc1); // PA1 değerini al

    // Üçüncü kanalın (PA2 - 6V pil) dönüşümünün tamamlanmasını bekle
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_raw_value_ch2 = HAL_ADC_GetValue(&hadc1); // PA2 değerini al

    // ADC'yi durdur
    HAL_ADC_Stop(&hadc1);

    // Ham ADC değerlerini gerçek voltaja dönüştür
    voltage_3v_battery = (adc_raw_value_ch0 / ADC_MAX_VALUE) * VREF_VOLTAGE;
    voltage_1v_battery = (adc_raw_value_ch1 / ADC_MAX_VALUE) * VREF_VOLTAGE;
    voltage_6v_battery = (adc_raw_value_ch2 / ADC_MAX_VALUE) * VREF_VOLTAGE; // Okunan değer voltaj bölücü sonrası değerdir

    // --- LED Kontrol Mantığı ---
    // Her döngü başında tüm LED'leri söndür ki sadece ilgili olan yansın
    HAL_GPIO_WritePin(LED_GPIO_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_PORT, BLUE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_PORT, RED_LED_PIN, GPIO_PIN_RESET);

    // Eğer 3V pil voltajı belirlenen aralıktaysa Yeşil LED'i yak (PA3)
    if (voltage_3v_battery >= V_3V_MIN && voltage_3v_battery <= V_3V_MAX)
    {
      HAL_GPIO_WritePin(LED_GPIO_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
    }
    // Eğer 1V pil voltajı belirlenen aralıktaysa Mavi LED'i yak (PA4)
    else if (voltage_1v_battery >= V_1V_MIN && voltage_1v_battery <= V_1V_MAX)
    {
    	HAL_GPIO_WritePin(LED_GPIO_PORT, BLUE_LED_PIN, GPIO_PIN_SET);
    }
    // Eğer 6V pil voltajı belirlenen aralıktaysa Kırmızı LED'i yak (PA5 - voltaj bölücü sonrası ~3V'a karşılık)
    else if (voltage_6v_battery >= V_6V_MIN && voltage_6v_battery <= V_6V_MAX)
    {
    	HAL_GPIO_WritePin(LED_GPIO_PORT, RED_LED_PIN, GPIO_PIN_SET);
    }
    // Hiçbir koşul sağlanmazsa zaten tüm LED'ler başta söndürülmüştü

    HAL_Delay(500); // 500 milisaniye bekle
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  // Sonsuz döngüden sonraki kullanıcı kodu (buraya genellikle ulaşılmaz)
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  // HSI (Dahili Yüksek Hızlı Osilatör) kullanılıyor.
  // Eğer harici kristal (HSE) kullanmak isterseniz bu kısmı CubeIDE'den ayarlayarak değiştirmelisiniz.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // PLL kullanılmıyor, HSI doğrudan SYSCLK'e bağlanıyor
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  // Sistem saati HSI'dan geliyor (8MHz varsayılan)
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // Sistem saati kaynağı HSI
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  // ADC saati yapılandırması (APB2 saatinden bölünerek)
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2; // APB2 saatinin 2'ye bölünmesi
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  // ADC1 başlatma öncesi kullanıcı kodu
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  // ADC1 başlatma sonrası kullanıcı kodu
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  * ADC'nin genel ayarları
  */
  hadc1.Instance = ADC1; // Kullanılacak ADC birimi (ADC1)
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; // Tarama modu AÇIK
  hadc1.Init.ContinuousConvMode = DISABLE;   // Sürekli dönüşüm modu KAPALI (manuel başlatacağız)
  hadc1.Init.DiscontinuousConvMode = DISABLE; // Kesintili dönüşüm modu KAPALI
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Yazılım tetiklemeli başlatma
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; // Veri hizalaması (sağa hizalı)
  hadc1.Init.NbrOfConversion = 3; // Toplam 3 kanal okunacak (PA0, PA1, PA2)
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - ADC_CHANNEL_0 (PA0)
  * PA0 pinini ADC Kanal 0 olarak yapılandır (Rank 1)
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1; // Düzenli dönüşüm sırasındaki ilk kanal
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; // Örnekleme süresi
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - ADC_CHANNEL_1 (PA1)
  * PA1 pinini ADC Kanal 1 olarak yapılandır (Rank 2)
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2; // Düzenli dönüşüm sırasındaki ikinci kanal
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; // Örnekleme süresi
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - ADC_CHANNEL_2 (PA2)
  * PA2 pinini ADC Kanal 2 olarak yapılandır (Rank 3)
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3; // Düzenli dönüşüm sırasındaki üçüncü kanal
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; // Örnekleme süresi
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

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
  // GPIO başlatma öncesi kullanıcı kodu
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE(); // GPIOD saati etkinleştirildi (eğer kullanılıyorsa)
  __HAL_RCC_GPIOA_CLK_ENABLE(); // GPIOA saati etkinleştirildi

  /*Configure GPIO pin Output Level */
  // Başlangıçta tüm LED pinlerini KAPALI yap
  HAL_GPIO_WritePin(GPIOA, GREEN_LED_PIN|BLUE_LED_PIN|RED_LED_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 */
  // LED pinlerini çıkış olarak yapılandır
  GPIO_InitStruct.Pin = GREEN_LED_PIN|BLUE_LED_PIN|RED_LED_PIN; // PA3, PA4, PA5
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull çıkış modu
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Pull-up/down direnci yok
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Düşük çıkış hızı
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // GPIO başlatma sonrası kullanıcı kodu
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Özel fonksiyonlar buraya eklenebilir
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq(); // Tüm kesmeleri devre dışı bırak
  while (1)
  {
    // Hata durumunda burada takılı kalır, debug için kullanılabilir
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
