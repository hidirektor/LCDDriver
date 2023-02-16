/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "lcd_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// LCD kontrol pins
#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_GPIO_Port GPIOA
#define LCD_RW_Pin GPIO_PIN_1
#define LCD_RW_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_2
#define LCD_EN_GPIO_Port GPIOA

// LCD data pins
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOA

#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDRESS 0x40
#define LCD_SET_DDRAM_ADDRESS 0x80

#define LCD_4BIT_MODE 0x00
#define LCD_8BIT_MODE 0x10
#define LCD_1LINE_MODE 0x00
#define LCD_2LINE_MODE 0x08
#define LCD_5X8_DOTS_MODE 0x00
#define LCD_5X10_DOTS_MODE 0x04
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void LCD_delay(uint16_t ms);
void LCD_sendCommand(uint8_t command);
void LCD_sendData(uint8_t data);
void LCD_init(void);
void LCD_setCursor(uint8_t row, uint8_t col);
void LCD_print(char* str);
void LCD_clear(void);
void LCD_home(void);
void LCD_displayOn(void);
void LCD_displayOff(void);
void LCD_cursorOn(void);
void LCD_cursorOff(void);
void LCD_blinkOn(void);
void LCD_blinkOff(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  // LCD tanımlama
  LCD_sendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE_MODE | LCD_5X8_DOTS_MODE);
  HAL_Delay(5);
  LCD_sendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE_MODE | LCD_5X8_DOTS_MODE);
  HAL_Delay(1);
  LCD_sendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE_MODE | LCD_5X8_DOTS_MODE);
  HAL_Delay(1);
  LCD_sendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE_MODE | LCD_5X8_DOTS_MODE);
  HAL_Delay(1);
  LCD_sendCommand(LCD_DISPLAY_CONTROL | 0x04);  // lcd kapatma
  HAL_Delay(1);
  LCD_sendCommand(LCD_CLEAR_DISPLAY);           // lcd temizleme
  HAL_Delay(5);
  LCD_sendCommand(LCD_ENTRY_MODE_SET | 0x06);   // girdi modu
  HAL_Delay(1);
  LCD_sendCommand(LCD_DISPLAY_CONTROL | 0x0C);  // lcd aktif etme
  HAL_Delay(1);

      // "Onder Lift Celik"
      char* line1 = "Onder Lift Celik";
      char* line2 = "Onder Lift Celik";
      LCD_sendCommand(LCD_SET_DDRAM_ADDRESS | 0x00);  // DDRAM adres => 0
      for (int i = 0; i < 16; i++) {
          if (line1[i] != '\0') {
              LCD_sendData(line1[i]);
          } else {
              break;
          }
      }
      LCD_sendCommand(LCD_SET_DDRAM_ADDRESS | 0x40);  // DDRAM adres => 40h
      for (int i = 0; i < 16; i++) {
          if (line2[i] != '\0') {
              LCD_sendData(line2[i]);
          } else {
              break;
          }
      }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

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

//predefined user code for custom lcd library:
void LCD_delay(uint16_t ms) {
    HAL_Delay(ms);
}

void LCD_sendCommand(uint8_t command) {
    // ilk yarı komutu gönderme
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, ((command >> 4) & 0x01));
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((command >> 5) & 0x01));
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((command >> 6) & 0x01));
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((command >> 7) & 0x01));
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);

    // kalan yarı komutu gönderme
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (command & 0x01));
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((command >> 1) & 0x01));
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((command >> 2) & 0x01));
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((command >> 3) & 0x01));
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);

    // tamamlama süresi
    LCD_delay(1);
}

void LCD_sendData(uint8_t data) {
    // ilk yarı datayı gönderme
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, ((data >> 4) & 0x01));
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((data >> 5) & 0x01));
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((data >> 6) & 0x01));
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((data >> 7) & 0x01));
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
    // kalan yarı datayı gönderme
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (data & 0x01));
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((data >> 1) & 0x01));
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((data >> 2) & 0x01));
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((data >> 3) & 0x01));
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);

    // tamamlama süresi
    LCD_delay(1);
}

void LCD_init() {
    // ilk gücü bekleme
    LCD_delay(100);
    // LCD tanımlama
    LCD_sendCommand(0x33);
    LCD_sendCommand(0x32);
    LCD_sendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE_MODE | LCD_5X8_DOTS_MODE);
    LCD_sendCommand(LCD_DISPLAY_CONTROL | 0x04);
    LCD_sendCommand(LCD_ENTRY_MODE_SET | 0x02);
    LCD_sendCommand(LCD_CLEAR_DISPLAY);

    // tanımlama tamamlama süresi
    LCD_delay(5);
}

void LCD_setCursor(uint8_t row, uint8_t col) {
    uint8_t address;
    if (row == 0) {
        address = 0x00;
    } else {
        address = 0x40;
    }

    address |= col;
    LCD_sendCommand(LCD_SET_DDRAM_ADDRESS | address);
}

void LCD_print(char* str) {
    while (*str) {
        LCD_sendData(*str++);
    }
}

void LCD_clear() {
    LCD_sendCommand(LCD_CLEAR_DISPLAY);
}

void LCD_home() {
    LCD_sendCommand(LCD_RETURN_HOME);
}

void LCD_displayOn() {
    LCD_sendCommand(LCD_DISPLAY_CONTROL | 0x04);
}

void LCD_displayOff() {
    LCD_sendCommand(LCD_DISPLAY_CONTROL);
}

void LCD_cursorOn() {
    LCD_sendCommand(LCD_DISPLAY_CONTROL | 0x02);
}

void LCD_cursorOff() {
    LCD_sendCommand(LCD_DISPLAY_CONTROL);
}

void LCD_blinkOn() {
    LCD_sendCommand(LCD_DISPLAY_CONTROL | 0x01);
}

void LCD_blinkOff() {
    LCD_sendCommand(LCD_DISPLAY_CONTROL);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
