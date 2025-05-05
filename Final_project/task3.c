//sampling

while (1)
  {
    /* USER CODE END WHILE */
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uint8_t adc_val = HAL_ADC_GetValue(&hadc1);  // cast to 8-bit

	  HAL_UART_Transmit(&huart2, &adc_val, 1, HAL_MAX_DELAY);  // send 1 byte

	  HAL_Delay(1);  // ~1kHz sampling rate
    /* USER CODE BEGIN 3 */
  }

//Processing
  UART_HandleTypeDef huart1;  // USART1: receiving from Sampling STM
  UART_HandleTypeDef huart2;  // USART2: sending to PC
  
  uint8_t rx_data = 0;
  uint8_t sample = 0;
  uint8_t filtered = 0;
  
  void SystemClock_Config(void);
  void MX_USART1_UART_Init(void);
  void MX_USART2_UART_Init(void);
  
  int main(void)
  {
      HAL_Init();
      SystemClock_Config();
      MX_USART1_UART_Init();  // USART1: input
      MX_USART2_UART_Init();  // USART2: output
  
      while (1)
      {
          // Step 1: Receive from Sampling STM (USART1)
          if (HAL_UART_Receive(&huart1, &rx_data, 1, HAL_MAX_DELAY) == HAL_OK)
          {
              // Step 2: Apply moving average filter
              filtered = (sample + rx_data) / 2;
  
              // Step 3: Send filtered data to PC (USART2)
              HAL_UART_Transmit(&huart2, &filtered, 1, HAL_MAX_DELAY);
  
              // Step 4: Update last sample
              sample = rx_data;
          }
      }
  }
  
  // USART1 for receiving
  void MX_USART1_UART_Init(void)
  {
      huart1.Instance = USART1;
      huart1.Init.BaudRate = 115200;
      huart1.Init.WordLength = UART_WORDLENGTH_8B;
      huart1.Init.StopBits = UART_STOPBITS_1;
      huart1.Init.Parity = UART_PARITY_NONE;
      huart1.Init.Mode = UART_MODE_RX;  // Only receive
      huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart1.Init.OverSampling = UART_OVERSAMPLING_16;
      HAL_UART_Init(&huart1);
  }
  
  // USART2 for sending
  void MX_USART2_UART_Init(void)
  {
      huart2.Instance = USART2;
      huart2.Init.BaudRate = 115200;
      huart2.Init.WordLength = UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
      huart2.Init.Parity = UART_PARITY_NONE;
      huart2.Init.Mode = UART_MODE_TX;  // Only transmit
      huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart2.Init.OverSampling = UART_OVERSAMPLING_16;
      HAL_UART_Init(&huart2);
  }
  
