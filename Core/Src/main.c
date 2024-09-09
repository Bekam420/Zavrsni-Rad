/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "liquidcrystal_i2c.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/

// Definiranje portova redova (ROWS)
#define ROWS_GPIO_PORT_A GPIOA  // GPIO port A za redove
#define ROWS_GPIO_PORT_B GPIOB  // GPIO port B za redove
#define ROWS_GPIO_PORT_C GPIOC  // GPIO port C za redove

// Definiranje portova stupaca (COLS)
#define COLS_GPIO_PORT_A GPIOA  // GPIO port A za stupce
#define COLS_GPIO_PORT_B GPIOB  // GPIO port B za stupce

// Definiranje pinova redova (ROWS)
#define ROW1_PIN GPIO_PIN_6  // Red 1, povezan na Port A, Pin 6
#define ROW2_PIN GPIO_PIN_7  // Red 2, povezan na Port A, Pin 7
#define ROW3_PIN GPIO_PIN_6  // Red 3, povezan na Port B, Pin 6
#define ROW4_PIN GPIO_PIN_7  // Red 4, povezan na Port C, Pin 7

// Definiranje pinova stupaca (COLS)
#define COL1_PIN GPIO_PIN_8   // Stupac 1, povezan na Port A, Pin 8
#define COL2_PIN GPIO_PIN_5   // Stupac 2, povezan na Port B, Pin 5
#define COL3_PIN GPIO_PIN_4   // Stupac 3, povezan na Port B, Pin 4
#define COL4_PIN GPIO_PIN_10  // Stupac 4, povezan na Port B, Pin 10

// Definiranje pina za LED
#define LED_PIN GPIO_PIN_5 // Pin A5

// Definiranje pina za relejni modul
#define GPIO_PIN_RELAY GPIO_PIN_0 // Pin 0
#define GPIO_PORT_RELAY GPIOC     // Port C


#define MASTER_PASSWORD "314159" // Master lozinka (prvih 6 znamenaka π)
#define PASSWORD_LENGTH 4        // Definiranje duljine lozinke

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static char Keypad_Scan(void);

void TIM6_DAC_IRQHandler(void);
void TIM7_DAC_IRQHandler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

char Keypad_Scan(void)
{
    char key = '\0';

    // Podesi svaki stupac na nisku razinu, jedan po jedan, i provjeri dali su redovi na visokoj razini (pritisnutu tipku).
    for (int col = 0; col < 4; col++)
    {
        // Postavi sve kolone na visoko stanje osim trenutne
        HAL_GPIO_WritePin(COLS_GPIO_PORT_A, COL1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL2_PIN | COL3_PIN | COL4_PIN, GPIO_PIN_SET);

        // Postavi trenutnu kolonu na nisko stanje
        switch (col)
        {
            case 0:
                HAL_GPIO_WritePin(COLS_GPIO_PORT_A, COL1_PIN, GPIO_PIN_RESET);
                break;
            case 1:
                HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL2_PIN, GPIO_PIN_RESET);
                break;
            case 2:
                HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL3_PIN, GPIO_PIN_RESET);
                break;
            case 3:
                HAL_GPIO_WritePin(COLS_GPIO_PORT_B, COL4_PIN, GPIO_PIN_RESET);
                break;
        }

        // Provjeri redove za visoku razinu (pritisnuta tipka)
        if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_A, ROW1_PIN) == GPIO_PIN_RESET)
        {
            key = (col == 0) ? '1' : (col == 1) ? '2' : (col == 2) ? '3' : 'A';
            break;
        }
        else if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_A, ROW2_PIN) == GPIO_PIN_RESET)
        {
            key = (col == 0) ? '4' : (col == 1) ? '5' : (col == 2) ? '6' : 'B';
            break;
        }
        else if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_B, ROW3_PIN) == GPIO_PIN_RESET)
        {
            key = (col == 0) ? '7' : (col == 1) ? '8' : (col == 2) ? '9' : 'C';
            break;
        }
        else if (HAL_GPIO_ReadPin(ROWS_GPIO_PORT_C, ROW4_PIN) == GPIO_PIN_RESET)
        {
            key = (col == 0) ? '*' : (col == 1) ? '0' : (col == 2) ? '#' : 'D';
            break;
        }
    }

    return key;
}

/**
  * @brief  Timer period elapsed callback.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *               the configuration information for the TIM module.
  * @retval None
  */

#define DEBOUNCE_DELAY 50 // Podešavanje maksimalnog debounce kašnjenja po potrebi
volatile uint8_t debounce_flag = 0;               // Zastavica za debouncing
volatile char debounce_key = '\0';                // 0 ako lozinka nije postavljena, 1 ako je postavljena

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t last_debounce_time = 0;
    static char last_key = '\0';                // Praćenje posljednje pritisnute tipke

    // Proverava da li je tajmer koji je izazvao prekid TIM6
    if (htim->Instance == TIM6)                 // Obrada TIM6 prekida
    {
        uint32_t current_time = HAL_GetTick();  // Dobijanje trenutnog vremena u milisekundama

        // Proverava da li je prošlo dovoljno vremena za debouncing
        if (current_time - last_debounce_time >= DEBOUNCE_DELAY)
        {
            char key = Keypad_Scan();           // Skener za prepoznavanje pritisnute tipke


            if (key != '\0')
            {

                if (key != last_key)            // Provjera je li tipka različita od posljednje registrirane
                {
                    debounce_key = key;         // Sprema pritisnutu tipku za dalju obradu
                    debounce_flag = 1;          // Postavljanje "zastavice" za valjani pritisak tipke
                }
                last_key = key;                 // Ažurira poslednju pritisnutu tipku
            }
            else
            {
                last_key = '\0';                // Ako nema pritisnute tipke, ponovo pokreće pritisnutu tipku
            }

            last_debounce_time = current_time;  // Ažurira vreme poslednjeg debouncinga
        }
    }
}


void TIM7_Delay(uint16_t delay)
{
    __HAL_TIM_SET_COUNTER(&htim7, 0);              // Ponovno pokretanje brojača
    HAL_TIM_Base_Start(&htim7);                    // Pokretanje tajmera
    while(__HAL_TIM_GET_COUNTER(&htim7) < delay);  // Čekanje da prođe vrijeme kašnjenja
    HAL_TIM_Base_Stop(&htim7);                     // Zaustavljanje tajmera
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None

/*
  * @brief  Main program.
  * @retval int
  */

static uint8_t indeks = 0;
char stored_password[PASSWORD_LENGTH + 1] = {0};  // Pohranjena lozinka
char entered_password[PASSWORD_LENGTH + 1] = {0}; // Unesena lozinka
uint8_t password_set = 0;

uint8_t master_mode = 0;                          // Zastavica koja označava unos master lozinke
char master_input[7] = {0};                       // Dužina 6 + null terminator

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();


     HD44780_Init(2);              // Inicijalizacija LCD ekrana
     HD44780_Clear();              // Čišćenje LCD ekrana
     HD44780_SetCursor(0, 0);      // Postavljanje kursora na početak
     HD44780_PrintStr("Upi");      // Ispis na LCD
     HD44780_PrintSpecialChar(0);  // Ispis š na LCD
     HD44780_PrintStr("i 1. ");
     HD44780_PrintSpecialChar(0);
     HD44780_PrintStr("ifru:");
     HD44780_SetCursor(0, 1);      // Postavljanje kursora u sljedeći red

    HAL_TIM_Base_Start_IT(&htim6); // Pokretanje TIM6 u interrupt modu
    HAL_TIM_Base_Start_IT(&htim7); // Pokretanje TIM7 u interrupt modu

    uint8_t mode = 0;              // 0: Podešavanje lozinke, 1: Verifikacija lozinke
    uint8_t delay_done = 0;        // Zastavica za provjeru da li je kašnjenje završeno nakon unosa lozinke

    while (1)
    {
        // Provjera da li je debounce zastavica postavljena (određuje da li je pritisak tipke detektiran)
        if (debounce_flag)
        {
            debounce_flag = 0;  // Resetiranje debounce zastavice nakon obrade

            // Provjerava ako je pritisnuta '#' tipka
            if (debounce_key == '#')
            {
                if (master_mode)  // Ako je u master modu
                {
                    // Provjera da li uneseni master password odgovara glavnoj lozinki
                    if (strcmp(master_input, MASTER_PASSWORD) == 0)
                    {
                        memset(stored_password, 0, sizeof(stored_password));  // Resetiranje pohranjene lozinke
                        password_set = 0;  // Postavljanje zastavice za resetiranje lozinke

                        // Čišćenje LCD ekrana i ispis poruke
                        HD44780_Clear();                            // Čišćenje LCD ekrana
                        HD44780_SetCursor(0, 0);                    // Postavljanje kursora na početak
                        HD44780_PrintSpecialChar(0);                // Specijalni znak š
                        HD44780_PrintStr("ifra restartana");        // Ispis "ifra restartana"
                        TIM7_Delay(30000);                          // Kašnjenje od 3 sekunde
                        HD44780_Clear();                            // Čišćenje LCD ekrana
                        HD44780_SetCursor(0, 0);                    // Postavljanje kursora na početak
                        HD44780_PrintStr("Upi");                    // Ispis "upi"
                        HD44780_PrintSpecialChar(0);                // Specijalni znak š
                        HD44780_PrintStr("i 1. ");                  // ispis "i 1."
                        HD44780_PrintSpecialChar(0);                // Specijalni znak š
                        HD44780_PrintStr("ifru:");                  // ispis "ifru"
                        HD44780_SetCursor(0, 1);                            // Postavljanje kursora na drugu liniju
                        mode = 0;                                           // Prelazak u režim podešavanja lozinke
                        indeks = 0;                                         // Resetiranje indeksa za unos lozinke
                    }
                    master_mode = 0;  // Izlazak iz master moda
                }
                else  // Ako nije u master modu
                {
                    if (indeks == PASSWORD_LENGTH)  // Provjera da li je unesena cijela lozinka
                    {
                        entered_password[indeks] = '\0';  // Dodaje se null bit
                        HD44780_Clear();                  // Čišćenje LCD ekrana

                        if (mode == 0)  // Ako je u režimu podešavanja lozinke
                        {
                            if (password_set == 0)  // Ako lozinka još nije postavljena
                            {
                                strcpy(stored_password, entered_password);  // Spremanje unesene lozinke
                                password_set = 1;                           // Postavljanje "zastavice" za spremanje lozinke
                                HD44780_Clear();                            // Čišćenje LCD ekrana
                                HD44780_PrintSpecialChar(0);                // Specijalni znak š
                                HD44780_PrintStr("ifra zapam");             // ispis "ifra zapam"
                                HD44780_PrintSpecialChar(1);                // Specijalni znak ć
                                HD44780_PrintStr("ena");                    // ispis "ena"
                                TIM7_Delay(10000);                          // Kašnjenje od 1 sekunde
                            }
                            else  // Ako je lozinka već postavljena
                            {
                                strcpy(stored_password, entered_password);  // Ažuriranje pohranjene lozinke
                                HD44780_Clear();                            // Čišćenje LCD ekrana
                                HD44780_PrintSpecialChar(0);                // Specijalni znak š
                                HD44780_PrintStr("ifra promenjena");        // ispis "ifra promjenjena"
                                TIM7_Delay(10000);                          // Kašnjenje od 1 sekunde
                            }

                            // Automatsko otključavanje nakon spremanja
                            HD44780_SetCursor(0, 0);                                             // Postavljanje kursora na početak
                            HD44780_PrintStr("Pristup odobren");                                 // ispis "pristup odobren"
                            HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_SET);                     // Aktiviranje LED-a
                            HAL_GPIO_WritePin(GPIO_PORT_RELAY, GPIO_PIN_RELAY, GPIO_PIN_SET);    // Aktiviranje releja
                            TIM7_Delay(20000);                                                   // Kašnjenje od 2 sekunde
                            HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_RESET);                   // Deaktiviranje LED-a
                            HAL_GPIO_WritePin(GPIO_PORT_RELAY, GPIO_PIN_RELAY, GPIO_PIN_RESET);  // Deaktiviranje releja

                            // Automatski poziv za novu radnju
                            HD44780_Clear();                            // Čišćenje LCD ekrana
                            HD44780_SetCursor(0, 0);                    // Postavljanje kursora na početak
                            HD44780_PrintStr("Unesi ");                 // ispis "unesi "
                            HD44780_PrintSpecialChar(0);                // Specijalni znak
                            HD44780_PrintStr("ifru:");                  // ispis "ifru"
                            HD44780_SetCursor(0, 1);                    // Postavljanje kursora na drugu liniju
                            memset(entered_password, 0, sizeof(entered_password));  // Resetiranje unesene lozinke
                            indeks = 0;                                // Resetiranje indeksa za unos lozinke
                            mode = 1;                                  // Prelazak u režim verifikacije
                        }
                        else if (mode == 1)
                        {
                            if (strcmp(stored_password, entered_password) == 0)  // Provjera lozinke
                            {
                                HD44780_PrintStr("Pristup odobren");                                 // ispis "pristup odobren"
                                HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_SET);                     // Aktiviranje LED-a
                                HAL_GPIO_WritePin(GPIO_PORT_RELAY, GPIO_PIN_RELAY, GPIO_PIN_SET);    // Aktiviranje releja
                                TIM7_Delay(30000);                                                   // Kašnjenje od 3 sekunde
                                HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_RESET);                   // Deaktiviranje LED-a
                                HAL_GPIO_WritePin(GPIO_PORT_RELAY, GPIO_PIN_RELAY, GPIO_PIN_RESET);  // Deaktiviranje releja
                            }
                            else
                            {
                                HD44780_PrintStr("Pristup zabranjen");
                            }


                            TIM7_Delay(20000);                          // Kašnjenje od 2 sekunde
                            HD44780_Clear();                            // Čišćenje LCD ekrana
                            HD44780_SetCursor(0, 0);                    // Postavljanje kursora na početak
                            HD44780_PrintStr("Unesi ");                 // ispis "Unesi"
                            HD44780_PrintSpecialChar(0);                // Specijalni znak
                            HD44780_PrintStr("ifru:");                  // ispis "ifru"
                            HD44780_SetCursor(0, 1);                    // Postavljanje kursora na drugu liniju
                            memset(entered_password, 0, sizeof(entered_password));  // Resetiranje unesene lozinke
                            indeks = 0;                                 // Resetiranje indeksa za unos lozinke
                        }
                    }
                }
            }
            // Provjerava ako je pritisnuta 'C' tipka
            else if (debounce_key == 'C')
            {
                master_mode = 1;                            // Ulazak u master mode
                HD44780_Clear();                            // Čišćenje LCD ekrana
                HD44780_SetCursor(0, 0);                    // Postavljanje kursora na početak
                HD44780_PrintStr("Master ");                // Ispis teksta "Master "
                HD44780_PrintSpecialChar(0);                // Ispis specijalnog znaka š
                HD44780_PrintStr("ifra:");                  // Ispis teksta "ifra:"
                HD44780_SetCursor(0, 1);                    // Postavljanje kursora na drugu liniju
                memset(master_input, 0, sizeof(master_input));  // Resetiranje unosa glavne lozinke
                indeks = 0;                                // Resetiranje indeksa za unos lozinke
            }
            // Ako je u master modu i pritisnuti znak je broj od 0 do 9
            else if (master_mode && indeks < 6 && debounce_key >= '0' && debounce_key <= '9')
            {
                master_input[indeks++] = debounce_key;      // Dodavanje broja u master lozinku

                HD44780_PrintChar(debounce_key);            // Prikaz unesenog broja
            }
            // Ako nije u master modu i pritisnuti znak je broj od 0 do 9
            else if (!master_mode && indeks < PASSWORD_LENGTH && debounce_key >= '0' && debounce_key <= '9')
            {
                entered_password[indeks++] = debounce_key;  // Dodavanje broja u unesenu lozinku

                HD44780_PrintChar(debounce_key);            // Prikaz unesenog broja
            }
        }
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  // Enable TIM6 global interrupt
   HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE END TIM6_Init 2 */

}

static void MX_TIM7_Init(void)
{
    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 8400-1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 40000-1;
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim7);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim7, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
}

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
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

