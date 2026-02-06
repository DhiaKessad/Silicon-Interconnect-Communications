#include "main.h"

/* --- Private defines --- */
#define LCD_ADDR (0x27 << 1)
#define RS_BIT   (1 << 0)
#define EN_BIT   (1 << 2)
#define BL_BIT   (1 << 3)

/* --- Register Macros for Bit-Banging --- */
#define SCL_HIGH() SCL_GPIO_Port->BSRR = SCL_Pin
#define SCL_LOW()  SCL_GPIO_Port->BSRR = (uint32_t)SCL_Pin << 16
#define SDA_HIGH() SDA_GPIO_Port->BSRR = SDA_Pin
#define SDA_LOW()  SDA_GPIO_Port->BSRR = (uint32_t)SDA_Pin << 16
#define SDA_READ() (SDA_GPIO_Port->IDR & SDA_Pin)

/* --- Function Prototypes --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);


/* --- I2C Functions --- */
void I2C_Delay(void) {
    for(volatile int i = 0; i < 50; i++);
}

void I2C_Start(void) {
    SDA_HIGH(); SCL_HIGH(); I2C_Delay();
    SDA_LOW();  I2C_Delay();
    SCL_LOW();  I2C_Delay();
}

void I2C_Stop(void) {
    SDA_LOW();  I2C_Delay();
    SCL_HIGH(); I2C_Delay();
    SDA_HIGH(); I2C_Delay();
}

void I2C_Write(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & (0x80 >> i)) SDA_HIGH(); else SDA_LOW();
        I2C_Delay(); SCL_HIGH(); I2C_Delay(); SCL_LOW();
    }
    // ACK Pulse
    SDA_HIGH(); SCL_HIGH(); I2C_Delay(); SCL_LOW();
}

/* --- LCD PCF8574 Logic --- */
void LCD_InternalWrite(uint8_t val) {
    I2C_Start();
    I2C_Write(LCD_ADDR);
    I2C_Write(val | BL_BIT);
    I2C_Stop();
}

void LCD_Send(uint8_t val, uint8_t rs) {
    uint8_t high = (val & 0xF0) | rs;
    uint8_t low = ((val << 4) & 0xF0) | rs;

    // Send High Nibble
    LCD_InternalWrite(high | EN_BIT);
    I2C_Delay();
    LCD_InternalWrite(high);

    // Send Low Nibble
    LCD_InternalWrite(low | EN_BIT);
    I2C_Delay();
    LCD_InternalWrite(low);
}

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_Send(0x33, 0);
    HAL_Delay(5);
    LCD_Send(0x32, 0);
    LCD_Send(0x28, 0); // 4-bit mode, 2 lines, 5x8 dots
    LCD_Send(0x0C, 0); // Display ON
    LCD_Send(0x01, 0); // Clear Display
    HAL_Delay(2);
}

/* --- MAIN FUNCTION --- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Initialize the LCD
    LCD_Init();

    // Print to the LCD
    char* msg = "Hello VLSI";
    while(*msg) {
        LCD_Send(*msg++, RS_BIT);
    }

    while (1)
    {
        // Program finished
    }
}

/* --- Hardware Configurations --- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, SCL_Pin|SDA_Pin, GPIO_PIN_SET); // Release bus initially

  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
