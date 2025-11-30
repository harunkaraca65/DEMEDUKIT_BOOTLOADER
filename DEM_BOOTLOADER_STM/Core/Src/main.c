/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 Custom Bootloader Main Program Body
  * @project        : OTA Update System with ESP32 and OLED
  * @author         : hrnkrc
  * @date           : November 2025
  * @description    : This bootloader listens for update commands from ESP32 via UART.
  * It handles Flash Erase, Write, and Verification (CRC32).
  * It also updates an SSD1306 OLED display with progress status.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306_ll.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- PROTOCOL DEFINITIONS --- */
#define PROTOCOL_CMD_INIT       0xAB  // Magic byte to enter bootloader mode
#define PROTOCOL_ACK            0xC1  // Acknowledge (OK)
#define PROTOCOL_NACK           0x7F  // Not Acknowledge (Error)

/* --- BOOTLOADER COMMANDS --- */
#define CMD_ERASE               0x50  // Erase Application Area
#define CMD_WRITE               0x51  // Write Data Chunk
#define CMD_JUMP_APP            0x52  // Jump to User Application
#define CMD_VERIFY              0x53  // Verify Integrity (CRC32)

/* --- CONFIGURATION --- */
#define BOOTLOADER_TIMEOUT_MS   3000
#define APP_START_ADDRESS       0x08003000 // Flash address where App begins
#define FLASH_PAGE_SIZE         0x800      // 2KB for STM32G0

/* --- FLASH UNLOCK KEYS (From Datasheet) --- */
#define FLASH_KEY1              0x45670123U
#define FLASH_KEY2              0xCDEF89ABU
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
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* --- BOOTLOADER FUNCTIONS --- */
void Bootloader_JumpToApp(void);
void Bootloader_ListeningMode(void);
void Bootloader_ProcessCommands(void);

/* --- DRIVER FUNCTIONS --- */
void Flash_Unlock(void);
void Flash_Lock(void);
void Flash_EraseAppArea(void);
uint8_t Flash_WriteData(uint32_t start_address, uint8_t *data, uint16_t length);
uint32_t Calculate_CRC32(uint32_t start_address, uint32_t length);

/* --- UART HELPER FUNCTIONS --- */
uint8_t UART_ReceiveByte(void);
void UART_SendByte(uint8_t data);
void UART_ReceiveArray(uint8_t *buffer, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* SysTick_IRQn interrupt configuration */
	NVIC_SetPriority(SysTick_IRQn, 3);

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	/* 1. Initialize OLED Display */
	  SSD1306_Init();
	  SSD1306_DrawProgressBar(0); // Draw empty bar
	  SSD1306_UpdateScreen();

	  /* 2. Enable UART for Communication */
	  LL_USART_Enable(USART2);

	  /* 3. Enter Listening Mode (Wait for ESP32) */
	  Bootloader_ListeningMode();

	  /* 4. If no update, jump to Application */
	  Bootloader_JumpToApp();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* Error State: Blink Red LED if Jump fails */
		    LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_3);
		    LL_mDelay(100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
	}

	/* HSI configuration and activation */
	LL_RCC_HSI_Enable();
	while (LL_RCC_HSI_IsReady() != 1) {
	}

	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8,
	LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_Enable();
	LL_RCC_PLL_EnableDomain_SYS();
	while (LL_RCC_PLL_IsReady() != 1) {
	}

	/* Set AHB prescaler*/
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

	/* Sysclk activation on the main PLL */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
	}

	/* Set APB1 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

	LL_Init1msTick(64000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(64000000);
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	LL_I2C_InitTypeDef I2C_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	/**I2C1 GPIO Configuration
	 PB6   ------> I2C1_SCL
	 PB7   ------> I2C1_SDA
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */

	/** I2C Initialization
	 */
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_InitStruct.Timing = 0x00C12166;
	I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_InitStruct.DigitalFilter = 0;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C1, &I2C_InitStruct);
	LL_I2C_EnableAutoEndMode(I2C1);
	LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_DisableOwnAddress2(I2C1);
	LL_I2C_DisableGeneralCall(I2C1);
	LL_I2C_EnableClockStretching(I2C1);
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**USART1 GPIO Configuration
	 PA9   ------> USART1_TX
	 PA10   ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);
	LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_DisableFIFO(USART1);
	LL_USART_ConfigAsyncMode(USART1);

	/* USER CODE BEGIN WKUPType USART1 */

	/* USER CODE END WKUPType USART1 */

	LL_USART_Enable(USART1);

	/* Polling USART1 initialisation */
	while ((!(LL_USART_IsActiveFlag_TEACK(USART1)))
			|| (!(LL_USART_IsActiveFlag_REACK(USART1)))) {
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**USART2 GPIO Configuration
	 PA2   ------> USART2_TX
	 PA3   ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART2 interrupt Init */
	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(USART2);

	/* USER CODE BEGIN WKUPType USART2 */

	/* USER CODE END WKUPType USART2 */

	LL_USART_Enable(USART2);

	/* Polling USART2 initialisation */
	while ((!(LL_USART_IsActiveFlag_TEACK(USART2)))
			|| (!(LL_USART_IsActiveFlag_REACK(USART2)))) {
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOD);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_1);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_2);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_3);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Jumps to the User Application located at APP_START_ADDRESS.
  * It resets the stack pointer and vector table offset before jumping.
  */
typedef void (*pFunction)(void);

void Bootloader_JumpToApp(void)
{
    uint32_t app_address = APP_START_ADDRESS;

    /* Check if valid stack pointer exists at the application address */
    if (((*(__IO uint32_t*)app_address) & 0x2FFE0000) == 0x20000000)
    {
        /* 1. De-initialize Peripherals */
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;
        LL_USART_Disable(USART1);
        LL_USART_Disable(USART2);

        /* 2. Prepare Jump Address */
        uint32_t jump_address = *(__IO uint32_t*) (app_address + 4);
        pFunction Jump_To_Application = (pFunction) jump_address;

        /* 3. Set Main Stack Pointer (MSP) */
        __set_MSP(*(__IO uint32_t*) app_address);

        /* 4. Jump! */
        Jump_To_Application();
    }
}

/**
  * @brief  Listens for the magic byte (0xAB) from ESP32 for 3 seconds.
  * If received, enters Command Processing mode.
  */
void Bootloader_ListeningMode(void)
{
    uint32_t timeout_counter = BOOTLOADER_TIMEOUT_MS / 10;

    while (timeout_counter > 0)
    {
        /* Visual Feedback: Toggle Red LED */
        if (timeout_counter % 50 == 0) LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_3);

        /* Check UART RX */
        if (LL_USART_IsActiveFlag_RXNE(USART2))
        {
            if (LL_USART_ReceiveData8(USART2) == PROTOCOL_CMD_INIT)
            {
                /* Handshake Successful */
                LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_3); // Solid Red LED
                UART_SendByte(PROTOCOL_ACK);                // Send ACK
                Bootloader_ProcessCommands();               // Enter Command Loop
                return;
            }
        }
        LL_mDelay(10);
        timeout_counter--;
    }

    /* Timeout: Turn off LED and exit */
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_3);
}

/**
  * @brief  Main loop to process commands from ESP32 (Erase, Write, Jump).
  */
void Bootloader_ProcessCommands(void)
{
    uint8_t cmd;
    uint8_t len;
    uint8_t rx_buffer[128];
    uint32_t current_write_addr = APP_START_ADDRESS;
    uint32_t written_bytes = 0;

    Flash_Unlock();

    while (1)
    {
        cmd = UART_ReceiveByte();

        /* --- COMMAND: ERASE --- */
        if (cmd == CMD_ERASE)
        {
            SSD1306_Clear();
            SSD1306_WriteString(35, 5, "ERASING...");
            SSD1306_DrawProgressBar(0);
            SSD1306_UpdateScreen();

            Flash_EraseAppArea();

            current_write_addr = APP_START_ADDRESS;
            written_bytes = 0;

            UART_SendByte(PROTOCOL_ACK);
        }
        /* --- COMMAND: WRITE --- */
        else if (cmd == CMD_WRITE)
        {
            len = UART_ReceiveByte();
            UART_ReceiveArray(rx_buffer, len);

            uint8_t result = Flash_WriteData(current_write_addr, rx_buffer, len);

            if (result == 1) // Success
            {
                current_write_addr += len;
                written_bytes += len;

                /* Update OLED every 1KB to save time */
                if (written_bytes % 1024 < 64)
                {
                    uint8_t percent = (written_bytes * 100) / 20000; // Est. 20KB
                    if (percent > 95) percent = 95;

                    SSD1306_Clear();
                    SSD1306_WriteString(25, 5, "LOADING...");
                    SSD1306_DrawProgressBar(percent);
                    SSD1306_UpdateScreen();
                }
                UART_SendByte(PROTOCOL_ACK);
            }
            else // Failure
            {
                UART_SendByte(PROTOCOL_NACK);
                UART_SendByte(result); // Send Error Code
            }
        }
        /* --- COMMAND: VERIFY (CRC32) --- */
        else if (cmd == CMD_VERIFY)
        {
            uint8_t verify_buffer[8];
            UART_ReceiveArray(verify_buffer, 8);

            /* Parse Received Data (Little Endian) */
            uint32_t received_crc = verify_buffer[0] | (verify_buffer[1] << 8) |
                                    (verify_buffer[2] << 16) | (verify_buffer[3] << 24);

            uint32_t total_len = verify_buffer[4] | (verify_buffer[5] << 8) |
                                 (verify_buffer[6] << 16) | (verify_buffer[7] << 24);

            /* Update Screen */
            SSD1306_Clear();
            SSD1306_WriteString(25, 5, "VERIFYING");
            SSD1306_DrawProgressBar(50);
            SSD1306_UpdateScreen();

            /* Calculate Local CRC */
            uint32_t calculated_crc = Calculate_CRC32(APP_START_ADDRESS, total_len);

            if (calculated_crc == received_crc)
            {
                /* SUCCESS */
                SSD1306_Clear();
                SSD1306_WriteString(35, 5, "SECURE!");
                SSD1306_DrawProgressBar(100);
                SSD1306_UpdateScreen();
                UART_SendByte(PROTOCOL_ACK);
            }
            else
            {
                /* FAILURE */
                SSD1306_Clear();
                SSD1306_WriteString(35, 5, "CRC ERROR!");
                SSD1306_UpdateScreen();
                UART_SendByte(PROTOCOL_NACK);
            }
        }
        /* --- COMMAND: JUMP --- */
        else if (cmd == CMD_JUMP_APP)
        {
            SSD1306_Clear();
            SSD1306_WriteString(30, 5, "UPDATED");
            SSD1306_DrawProgressBar(100);
            SSD1306_UpdateScreen();

            Flash_Lock();
            UART_SendByte(PROTOCOL_ACK);
            return; // Exit loop to jump
        }
    }
}

/* ========================================================================== */
/* FLASH DRIVER FUNCTIONS                                                     */
/* ========================================================================== */

void Flash_Unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

void Flash_Lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

void Flash_EraseAppArea(void) {
    uint32_t page_index;
    __disable_irq();

    /* Clear Flash Flags */
    FLASH->SR = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
                FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR |
                FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_EOP;

    /* Erase Pages 6 to 31 (Application Area) */
    for (page_index = 6; page_index < 32; page_index++) {
        while (FLASH->SR & FLASH_SR_BSY1);
        FLASH->CR |= FLASH_CR_PER;
        FLASH->CR &= ~FLASH_CR_PNB;
        FLASH->CR |= (page_index << FLASH_CR_PNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        while (FLASH->SR & FLASH_SR_BSY1);
        FLASH->CR &= ~FLASH_CR_PER;
        FLASH->CR &= ~FLASH_CR_PNB;
    }
    __enable_irq();
}

uint8_t Flash_WriteData(uint32_t start_address, uint8_t *data, uint16_t length) {
    uint32_t i;
    uint32_t data_word1, data_word2;

    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }

    __disable_irq();

    /* Clear Flags */
    FLASH->SR = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
                FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR |
                FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_EOP;

    /* Enable Programming Mode */
    FLASH->CR |= FLASH_CR_PG;

    for (i = 0; i < length; i += 8) {
        /* Align Data (Byte to Word) */
        data_word1 = (uint32_t)data[i] | ((uint32_t)data[i+1] << 8) |
                     ((uint32_t)data[i+2] << 16) | ((uint32_t)data[i+3] << 24);
        data_word2 = (uint32_t)data[i+4] | ((uint32_t)data[i+5] << 8) |
                     ((uint32_t)data[i+6] << 16) | ((uint32_t)data[i+7] << 24);

        /* Write Double Word */
        *(__IO uint32_t*)(start_address + i) = data_word1;
        *(__IO uint32_t*)(start_address + i + 4) = data_word2;

        while (FLASH->SR & FLASH_SR_BSY1);

        /* Error Check */
        if (FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
                         FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR)) {
            uint8_t error = (uint8_t)(FLASH->SR);
            FLASH->CR &= ~FLASH_CR_PG;
            __enable_irq();
            return error;
        }

        if (FLASH->SR & FLASH_SR_EOP) {
            FLASH->SR = FLASH_SR_EOP;
        }
    }

    FLASH->CR &= ~FLASH_CR_PG;
    __enable_irq();
    return 1;
}

uint32_t Calculate_CRC32(uint32_t start_address, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    uint8_t *pData = (uint8_t*) start_address;
    uint32_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= pData[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

/* ========================================================================== */
/* UART HELPER FUNCTIONS                                                      */
/* ========================================================================== */

uint8_t UART_ReceiveByte(void) {
    while (!LL_USART_IsActiveFlag_RXNE(USART2));
    return LL_USART_ReceiveData8(USART2);
}

void UART_ReceiveArray(uint8_t *buffer, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) buffer[i] = UART_ReceiveByte();
}

void UART_SendByte(uint8_t data) {
    LL_USART_TransmitData8(USART2, data);
    while (!LL_USART_IsActiveFlag_TC(USART2));
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
