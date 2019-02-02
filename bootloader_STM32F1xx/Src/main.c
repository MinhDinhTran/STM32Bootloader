/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "usbd_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define C_UART &huart2
#define BL_RX_LEN 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t bl_version = 10;
uint8_t bl_rx_buffer[BL_RX_LEN];
uint8_t supported_commands[] = {
    BL_GET_VER,
    BL_GET_HELP,
    BL_GET_CID,
    BL_GET_RDP_STATUS,
    BL_GO_TO_ADDR,
    BL_FLASH_ERASE,
    BL_MEM_WRITE,
    BL_READ_SECTOR_P_STATUS};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void printmsg(char *format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* prints formatted string to console over UART */
void printmsg(char *format, ...)
{
#ifdef BL_DEBUG_MSG_EN
  char str[80];

  /*Extract the the argument list using VA apis */
  va_list args;
  va_start(args, format);
  vsprintf(str, format, args);
  // HAL_UART_Transmit(C_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  if (((USBD_CDC_HandleTypeDef *)(hUsbDeviceFS.pClassData))->TxState == 0)
  {
    /* Transmiting data to USB over CDC interface */
    /* (Browse through usbd_cdc_if.c file for the correct function to use */
    /* uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) */
    CDC_Transmit_FS((uint8_t *)str, strlen(str));
  }
  va_end(args);
#endif
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
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Lets check whether button is pressed or not, if not pressed jump to user application */
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    {
      printmsg("BL_DEBUG_MSG:Button is pressed .. going to BL mode\n\r");
      HAL_Delay(1000);

      //we should continue in bootloader mode
      bootloader_uart_read_data();
    }
    else
    {
      printmsg("BL_DEBUG_MSG:Button is not pressed .. executing user app\n\r");
      // printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n\r", bl_version, bl_version);
      HAL_Delay(1000);
      // jump to user application
      // bootloader_jump_to_user_app();
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void bootloader_uart_read_data(void)
{
  uint8_t rcv_len = 0;

  while (1)
  {
    memset(bl_rx_buffer, 0, 200);
    //here we will read and decode the commands coming from host
    //first read only one byte from the host , which is the "length" field of the command packet
    HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);
    rcv_len = bl_rx_buffer[0];
    HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);
    switch (bl_rx_buffer[1])
    {
    case BL_GET_VER:
      bootloader_handle_getver_cmd(bl_rx_buffer);
      break;
    case BL_GET_HELP:
      bootloader_handle_gethelp_cmd(bl_rx_buffer);
      break;
    case BL_GET_CID:
      // bootloader_handle_getcid_cmd(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n\r");
      break;
    case BL_GET_RDP_STATUS:
      // bootloader_handle_getrdp_cmd(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n\r");
      break;
    case BL_GO_TO_ADDR:
      // bootloader_handle_go_cmd(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_go_cmd\n\r");
      break;
    case BL_FLASH_ERASE:
      // bootloader_handle_flash_erase_cmd(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\n\r");
      break;
    case BL_MEM_WRITE:
      // bootloader_handle_mem_write_cmd(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n\r");
      break;
    case BL_EN_RW_PROTECT:
      // bootloader_handle_en_rw_protect(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_en_rw_protect\n\r");
      break;
    case BL_MEM_READ:
      // bootloader_handle_mem_read(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_mem_read\n\r");
      break;
    case BL_READ_SECTOR_P_STATUS:
      // bootloader_handle_read_sector_protection_status(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\n\r");
      break;
    case BL_OTP_READ:
      // bootloader_handle_read_otp(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_read_otp\n\r");
      break;
    case BL_DIS_R_W_PROTECT:
      // bootloader_handle_dis_rw_protect(bl_rx_buffer);
      printmsg("BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\n\r");
      break;
    default:
      printmsg("BL_DEBUG_MSG:Invalid command code received from host \n\r");
      break;
    }
  }
}
/*Helper function to handle BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
  uint8_t bl_version;

  // 1) verify the checksum
  printmsg("BL_DEBUG_MSG:bootloader_handle_getver_cmd\n\r");

  //Total length of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;

  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

  if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
  {
    printmsg("BL_DEBUG_MSG:checksum success !!\n\r");
    // checksum is correct..
    bootloader_send_ack(bl_rx_buffer[0], 1);
    bl_version = get_bootloader_version();
    printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n\r", bl_version, bl_version);
    bootloader_uart_write_data(&bl_version, 1);
  }
  else
  {
    printmsg("BL_DEBUG_MSG:checksum fail !!\n\r");
    //checksum is wrong send nack
    bootloader_send_nack();
  }
}

/*Helper function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command codes
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
  printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n\r");

  //Total length of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;

  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

  if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
  {
    printmsg("BL_DEBUG_MSG:checksum success !!\n\r");
    bootloader_send_ack(pBuffer[0], sizeof(supported_commands));
    bootloader_uart_write_data(supported_commands, sizeof(supported_commands));
  }
  else
  {
    printmsg("BL_DEBUG_MSG:checksum fail !!\n\r");
    bootloader_send_nack();
  }
}

/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
  //here we send 2 byte.. first byte is ack and the second byte is len value
  uint8_t ack_buf[2];
  ack_buf[0] = BL_ACK;
  ack_buf[1] = follow_len;
  HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}

/*This function sends NACK */
void bootloader_send_nack(void)
{
  uint8_t nack = BL_NACK;
  HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}

//This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
  uint32_t uwCRCValue = 0xff;

  for (uint32_t i = 0; i < len; i++)
  {
    uint32_t i_data = pData[i];
    uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
  }

  /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

  if (uwCRCValue == crc_host)
  {
    return VERIFY_CRC_SUCCESS;
  }

  return VERIFY_CRC_FAIL;
}

/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len)
{
  /*you can replace the below ST's USART driver API call with your MCUs driver API call */
  HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}
//Just returns the macro value .
uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
