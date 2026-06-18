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
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

////////////////////////////////////////////
uint32_t UNIC_ID_SENSOR; //uniq ID of sensor
uint8_t code;   //code of sensor

//For Firmware
uint16_t Ver_Number = 0x0011; // Ver. of Firmware
uint32_t Ver_Data = 0x070213; // Data create of Firmware

//////////////////////////////////////////
//Adress Var&Const in flash of memory
//
#define ID_VALUE_ADDRESS  								((uint32_t)0x08007C00) // 00 command
#define Ver_Number_ADDRESS  							((uint32_t)0x08007C04) //F0 command
#define Ver_Data_ADDRESS   								((uint32_t)0x08007C08) //F0 command
#define UNIQ_ID_SENSOR_ADDRESS   					((uint32_t)0x08007C0C)
#define CODE_SENSOR_ADDRESS   						((uint32_t)0x08007C10)

#define TEMPERATURE_ADC_ADDRESS  			((uint32_t)0x08007D00)

#define LimitWeight_plus_ADDRESS  				((uint32_t)0x08007E08)

#define StatByte_ADDRESS   								((uint32_t)0x08007E20)

#define T        700
#define LIMIT_1  2200
#define LIMIT_2  2790  

#define N_GAS_FILTER  10
///////////////////////////////////////////////////////
#define FLASH_PAGE				 			 ((uint8_t)0x1F) //31 page(number of pages of Flash memory)
#define SPEED_PAGE				 			 ((uint8_t)0x1E) //30 page(number of pages of Flash memory)
///////////////////////////////////////////////////////
uint32_t st_address = FLASH_BASE + FLASH_PAGE * 1024;  
uint32_t speed_address = FLASH_BASE + SPEED_PAGE * 1024;

uint8_t SpeedMode[6] = {120, 60, 24, 12, 6, 3};
uint8_t SpeedIndex;
uint8_t tempIndx;
////////////////////////////////////////////
uint8_t start = 0;
///////////////////////////
//for CAN
uint8_t Address_sensor; // embedded Address of sensor
uint8_t StatByte = 0x01; // Mode of sensor: 7 bit "0" - mode of calibration, "1" - mode of work

uint8_t  DATA[8];
uint8_t  can_err;
#define CAN_IDE_32   0x0004

int16_t LimitWeight_plus;

/////////////////////////////////////////
//for ADC across SPI

uint8_t rx_Buffer2[4]={0};
uint32_t rx_Buffer ={0};

//intern adc
uint16_t ADC_Data;

//state machine
uint8_t read_ADC = 0;
uint8_t StM_Ads= 1; // state of ADC of ready
uint8_t CntBit;
uint8_t CntClk;
uint32_t DataAds;

// for float filter of weight and temperature
#define N_filter1 (16)
uint32_t Average_Weight;
uint32_t Average_Weight_for_11;// for command 01 of calibration mode 
uint8_t AA1;
uint8_t BB1;
float U_adc_weight;
#define N_filter2 (8)
#define VREFINT_ADDRESS_CAL   			((uint32_t)0x1FFFF7BA)
uint16_t Average_Temp_ADC;

//DMA
uint32_t DMA_buf[3];

/////////////////////////////////////////////////////////

//can
static CanTxMsgTypeDef canTxMessage;//Buf for transmit 
static CanRxMsgTypeDef canRxMessage; //Buf for receive

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//Calculation of temperature for TC1047
// Temperature_Unembbed
int8_t Temperature_External(uint16_t Temperature_ADC2)	
	{
	uint32_t V_ref_emb = 3300000; //reference voltage of embedded sensor of temperature (variable) mkV 
	uint16_t max_ADC_emb = 4095; // (2^12) 4096-1 (2^12) 4096-1 (12 is bit capacity of ADC)
	uint32_t V_in; 
		
	V_in = Temperature_ADC2*((*(__IO uint16_t*)VREFINT_ADDRESS_CAL)*(V_ref_emb*1024/max_ADC_emb)/DMA_buf[2]); // Voltage of sensor in mkV  //	DMA_buf[1] = VREFINT_ADC
	
	return (V_in/(10000*1024))-50; //1000*10
	}
	
uint32_t Temperature_Voltage(uint16_t Temperature_ADC2)	
	{
	uint32_t V_ref_emb = 3300000; //reference voltage of embedded sensor of temperature (variable) mkV 
	uint16_t max_ADC_emb = 4095; // (2^12) 4096-1 (2^12) 4096-1 (12 is bit capacity of ADC)
	uint32_t V_in; 
		
	V_in = Temperature_ADC2*((*(__IO uint16_t*)VREFINT_ADDRESS_CAL)*(V_ref_emb*1024/max_ADC_emb)/DMA_buf[2]);
	
	return V_in / (1000 * 1024);
	}
	
int16_t Weight(uint32_t Weight_standart_ADC2, uint16_t Weight_standart2, uint32_t Weight_zero_ADC2, uint32_t Now_Weight_ADC, int8_t T_now) 
{
    uint64_t scale_coef = ((uint64_t)(Weight_standart_ADC2 - Weight_zero_ADC2) * 256) / Weight_standart2;
    if (scale_coef == 0) return 0;
    int32_t delta;
    int16_t result;
    
    if (Now_Weight_ADC >= Weight_zero_ADC2) {
        delta = (Now_Weight_ADC - Weight_zero_ADC2) * 256;
        result = delta / scale_coef;
        StatByte &= ~0x80;   
    } else {
        delta = (Weight_zero_ADC2 - Now_Weight_ADC) * 256;
        result = delta / scale_coef ;
        result = -result;
        StatByte |= 0x80;
    }
    
    return result;
}
//static inline int32_t sdiv(int32_t num, int32_t den) { return (den != 0) ? (num / den) : 0; }

//Function for work Flash of memory

//ready Flash of memory
uint8_t flash_ready(void) {
return !(FLASH->SR & FLASH_SR_BSY);
}

/* Errase of page */
void flash_erase_page(uint32_t address)
{
   FLASH->CR|= FLASH_CR_PER;
   FLASH->AR = address;
   FLASH->CR|= FLASH_CR_STRT;
   while(!flash_ready());
   FLASH->CR&= ~FLASH_CR_PER;
}

/* Unblocking */
void flash_unlock(void) {
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

/* Blocking of Flash */
void flash_lock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}

/* Write data in Flash */
void flash_write(uint32_t address,uint32_t data) 
{
	FLASH->CR |= FLASH_CR_PG;
	while(!flash_ready());
	*(__IO uint16_t*)address = (uint16_t)data;
	while(!flash_ready());
	address+=2;
	data>>=16;
	*(__IO uint16_t*)address = (uint16_t)data;
	while(!flash_ready());
	FLASH->CR &= ~(FLASH_CR_PG);
}

/* Write data in Flash */
void flash_write_16(uint32_t address,uint16_t data) 
{
	FLASH->CR |= FLASH_CR_PG;
	while(!flash_ready());
	*(__IO uint16_t*)address = (uint16_t)data;
	while(!flash_ready());
	FLASH->CR &= ~(FLASH_CR_PG);
}

/* Read data from Flash */
uint32_t flash_read(uint32_t address) {
return (*(__IO uint32_t*) address);
}

/* All read of data from Flash of memory */
void READ_BEGIN_VAR_FLASH(void)
{
	Address_sensor = flash_read(ID_VALUE_ADDRESS);
	LimitWeight_plus = flash_read(LimitWeight_plus_ADDRESS);
	
	StatByte = flash_read(StatByte_ADDRESS) | 0x01;
	UNIC_ID_SENSOR = flash_read(UNIQ_ID_SENSOR_ADDRESS);
	code = flash_read(CODE_SENSOR_ADDRESS);
}


/////////////////// float_filter for calc of weight
uint32_t float_filter(uint32_t a)
{
	static int m[N_filter1];
	static int n;
	m[n]=a;
	n=(n+1)%N_filter1;
	a=0;
	for(int j=0;j<N_filter1;j++)
		{
			a=a+m[j];
		}
	return a/N_filter1;
}

/////////////////// float_filter for calc of temp
uint16_t float_filter2(uint32_t a)
{
	static int m[N_filter2];
	static int n;
	static int start = 0;
	
	if(start == 0)
	{
		for(int i=0;i<N_filter2;i++)
		{
			m[i] = a;
		}
		start++;
	}
	m[n]=a;
	n=(n+1)%N_filter2;
	a=0;
	for(int j=0;j<N_filter2;j++)
		{
			a=a+m[j];
		}
	return a/N_filter2;
}

////////////////////////////////////////////////////////////////////////////
//for temperature compensation

///////////////////////////////////////////
//Commands of CAN

uint32_t ADC_ToMilliVolts(uint16_t raw)
{
    uint32_t V_ref_emb = 3300; 
    uint16_t max_ADC_emb = 4095; 
    uint32_t V_in; 
        
    V_in = raw * ((*(__IO uint16_t*)VREFINT_ADDRESS_CAL) * (V_ref_emb * 1024 / max_ADC_emb) / DMA_buf[2]);
    return V_in; // ? ???
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	can_err=HAL_CAN_GetError(hcan);
}


//This function has hardware TIMER
	uint8_t ReadADS1232(void){   //

    switch(StM_Ads){
        default:
            StM_Ads = 1;

				// "Waiting for readiness measurements from ADC"
        case 1: 
						if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)  //miso spi
						{
                StM_Ads++;                                                       
                CntBit = 0;                                                      
                DataAds = 0;                                                     
                CntClk = 0;                                                      
            }
            break;
        	
				//"Setting CLK output to high"		
        case 2:     
            CntClk++;
            if(CntClk >= 10) //If the duration of the CLK pulse has passed
						{
                CntClk = 0;
								HAL_GPIO_WritePin (GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //clk spi
                StM_Ads++;
            }
            break;
									
				//"Setting CLK output to low"	
        case 3:
            CntClk++;
            if(CntClk >= 10) //If the duration of the CLK pulse has passed
						{
                CntClk = 0;
								HAL_GPIO_WritePin (GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); //clk spi
                StM_Ads++;
            }
            break;
						
        //Read of data
        case 4:
            StM_Ads++;
						if(CntBit < 24)
					  {
							if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) //miso spi
							{
									DataAds |= 1; //write 1
							}
					  }
            break;
          			
				//Shift and write data in DataAds
        case 5:
            CntBit++;
				    if(CntBit>= 25 ) //0..24>=25 bit
							{ 
								//calibration ads1232 =26 bit
								if(TIM16->SR&TIM_SR_CC1IF) //Update interrupt pending. This bit is set by hardware. For 24mhz - 100 sec
								{
									TIM16->SR&=~TIM_SR_CC1IF; //It is cleared by software.
									StM_Ads = 2;
								}
								else //
								{
									StM_Ads = 6;
									CntBit = 0;
								}
							}
            else if(CntBit == 24) //24 bit
							{ 
								StM_Ads = 2;
							}
            else if(CntBit < 24 ) //<24 bit
							{ 
                DataAds <<= 1;
								StM_Ads = 2; 
							}
            break;

        case 6: 
						StM_Ads = 1;					
						Average_Weight = float_filter(DataAds); //function float_filter
						Average_Weight_for_11=DataAds;
						Average_Weight += 0x800000;
						rx_Buffer2[0] = Average_Weight>>24; //MSB
						rx_Buffer2[1] = Average_Weight>>16;
						rx_Buffer2[2] = Average_Weight>>8;
						rx_Buffer2[3] = Average_Weight; //LSB	
				
						return 0xFF; //ready
	}
		    return 0x00; //busy
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) 
{
	Average_Temp_ADC = float_filter2(DMA_buf[0]); //function float_filter
}

void Gas_Voltage(void)
{
    static uint32_t sample_tick  = 0;
    static uint32_t update_tick  = 0;
    static uint32_t last_send_tick = 0;

    static uint32_t gas_buf[N_GAS_FILTER] = {0};
    static uint8_t  gas_idx = 0;
    static uint8_t  gas_filled = 0;
    static uint32_t gas_avg = 0;
    if ((HAL_GetTick() - sample_tick) >= 100)
    {
        sample_tick = HAL_GetTick();

        gas_buf[gas_idx] = DMA_buf[1];
        gas_idx = (gas_idx + 1) % N_GAS_FILTER;
        if (gas_idx == 0) gas_filled = 1;
    }

    if ((HAL_GetTick() - update_tick) >= 1000)
    {
        update_tick = HAL_GetTick();

        if (gas_filled)
        {
            uint32_t sum = 0;
            for (uint8_t i = 0; i < N_GAS_FILTER; i++)
                sum += gas_buf[i];
            gas_avg = sum / N_GAS_FILTER;
        }
        else
        {
            gas_avg = DMA_buf[1];
        }
    }

    int8_t  t_now   = Temperature_External((uint16_t)DMA_buf[0]);
    int32_t temp_mv = (int32_t)Temperature_Voltage((uint16_t)DMA_buf[0]);

    uint32_t gas_mv = 8 * gas_avg / 10;

    int32_t gas_limit1;
    int32_t gas_limit2;

    if (temp_mv >= 100 && temp_mv < 1100)
    {
        gas_limit1 = LIMIT_1 + 12 * (temp_mv - T) / 10;
        gas_limit2 = LIMIT_2 + 41 * (temp_mv - T) / 40;
    }
    else if (temp_mv >= 1100)
    {
        gas_limit1 = 2680 - 11 * (temp_mv - 1100) / 15;
        gas_limit2 = 3200 -  5 * (temp_mv - 1100) / 6;
    }

    if ((int32_t)gas_mv > gas_limit2)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else if ((int32_t)gas_mv > gas_limit1)
    {
        if ((HAL_GetTick() % 1000) < 500)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    if ((HAL_GetTick() - last_send_tick) >= 1000)
    {
        last_send_tick = HAL_GetTick();

        hcan.pTxMsg->ExtId   = 0x00;
        hcan.pTxMsg->IDE     = CAN_ID_EXT;
        hcan.pTxMsg->RTR     = CAN_RTR_DATA;
        hcan.pTxMsg->DLC     = 8;
        hcan.pTxMsg->Data[0] = Address_sensor;
        hcan.pTxMsg->Data[1] = 0x1A;
        hcan.pTxMsg->Data[2] = StatByte;
        hcan.pTxMsg->Data[3] = (uint8_t)t_now;
        hcan.pTxMsg->Data[4] = (uint8_t)(gas_mv >> 8);
        hcan.pTxMsg->Data[5] = (uint8_t)gas_mv;
        hcan.pTxMsg->Data[6] = 0;
        hcan.pTxMsg->Data[7] = 0;
        HAL_CAN_Transmit(&hcan, 10);
    }
}

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

	SpeedIndex = flash_read(speed_address);
	if (SpeedIndex == 0xFF){
		SpeedIndex = 3;
	}

	READ_BEGIN_VAR_FLASH();
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
  MX_CAN_Init();
  MX_IWDG_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
		
	//init of signals
	HAL_GPIO_WritePin (GPIOA,GPIO_PIN_3,GPIO_PIN_SET);	//PDWN
	
	//init const data from flash of memory
	
	//Init Watchdog
	//HAL_IWDG_Init(&hiwdg);//1
	
	hcan.pTxMsg = &canTxMessage;//urok begin
	hcan.pRxMsg = &canRxMessage;
	
	/* CAN filter init */
	CAN_FilterConfTypeDef canFilterConfig;
	canFilterConfig.FilterNumber = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	
	canFilterConfig.FilterIdHigh = (uint16_t)(0x00FEEA00>>13); // //FilterIdHigh 
	canFilterConfig.FilterIdLow = (uint16_t)((0x00FEEA00<<3)|(CAN_IDE_32)); // FilterIdLow 
	canFilterConfig.FilterMaskIdHigh = (uint16_t)(0x1FFFFFFF>>13); // // FilterMaskIdHigh 
	canFilterConfig.FilterMaskIdLow = (uint16_t)((0x1FFFFFFF<<3)|CAN_IDE_32); // FilterMaskIdLow 

	canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 1;
	HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0); 

//START TIM,ADC&DMA
	HAL_ADC_Start_DMA(&hadc, &DMA_buf[0], 3);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim3);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t last_send_tick = 0;																																																																
  while (1)
  {

	HAL_IWDG_Refresh(&hiwdg);//40000/(32*1250) = 1 sec 
	read_ADC = ReadADS1232(); //read data from adc without interrupts
	Gas_Voltage();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ENABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_3TQ;
  hcan.Init.BS1 = CAN_BS1_13TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = ENABLE;
  hcan.Init.RFLM = ENABLE;
  hcan.Init.TXFP = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 1250;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 50000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins : PA3 PA5 PA7 PA8 
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin   = GPIO_PIN_0;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

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
  while(1)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation t                                                                                                                                                                                       o report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
