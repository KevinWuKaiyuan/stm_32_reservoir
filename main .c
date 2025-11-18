/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
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
#include<string.h>
#include<stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t byte;
uint8_t byte_number;
uint8_t byte_number2;
uint8_t time = 24;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t pipeline[5] = {0, 4, 4, 4, 4};
uint8_t PWM[5] = {4, 4, 4, 4, 4};
uint8_t PWM_sequence[5] = {0, 4, 4, 4, 4};
uint8_t pipeline_choice = 4;
uint8_t StartHr[5] = {24, 24, 24, 24, 24};
uint8_t EndHr[5] = {24, 24, 24, 24, 24};

// Sensor variables
uint16_t time_diff = 0;
uint16_t distance = 0;
uint8_t depth = 0;
uint8_t MAX_distance = 35;

uint8_t current_pipe;

// Motor variables
uint8_t motor_flag = 0;

uint16_t Motor_DCVAL = 0;

//Motor RPM calculation variables
uint16_t rpm_tick_count = 0;
uint16_t rpm = 0;

// Wall Clock Variables
volatile uint8_t clock_hours = 0;
volatile uint8_t clock_mins = 0;
volatile uint8_t clock_secs = 0;
uint8_t flag = 0;

//timer board Variables
uint8_t B_display = 0;
uint8_t A_display = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void ADC_Select_CH(int CH);
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rcv_intpt_flag = 0;
volatile uint8_t hcsr04_Rx_flag = 0;
volatile uint8_t first_edge = 0;
volatile uint16_t time_edge1 = 0;
volatile uint16_t time_edge2 = 0;

//display of timer board
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B)
{
uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits
int Abit0 = (DIGITA_VAL ) & 1; // extract Abit0 of the 4-bit value
int Abit1 = (DIGITA_VAL >> 1) & 1; // extract Abit1 of the 4-bit value
int Abit2 = (DIGITA_VAL >> 2) & 1; // extract Abit2 of the 4-bit value
int Abit3 = (DIGITA_VAL >> 3) & 1; // extract Abit3 of the 4-bit value

uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 bits
int Bbit0 = (DIGITB_VAL ) & 1; // extract Bbit0 of the 4-bit value
int Bbit1 = (DIGITB_VAL >> 1) & 1; // extract Bbit1 of the 4-bit value
int Bbit2 = (DIGITB_VAL >> 2) & 1; // extract Bbit2 of the 4-bit value
int Bbit3 = (DIGITB_VAL >> 3) & 1; // extract Bbit3 of the 4-bit value

if (Abit0 == (0))
{
HAL_GPIO_WritePin(GPIOA, DIGIT_A0_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOA, DIGIT_A0_Pin, GPIO_PIN_SET);

}
if (Abit1 == (0))
{
HAL_GPIO_WritePin(GPIOA, DIGIT_A1_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOA, DIGIT_A1_Pin, GPIO_PIN_SET);

}
if (Abit2 == (0))
{
HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);

}
if (Abit3 == (0))
{
HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);

}


if (Bbit0 == (0))
{
HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET);

}
if (Bbit1 == (0))
{
HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET);

}
if (Bbit2 == (0))
{
HAL_GPIO_WritePin(GPIOB, DIGIT_B2_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOB, DIGIT_B2_Pin, GPIO_PIN_SET);

}
if (Bbit3 == (0))
{
HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);

}
}

void ADC_Select_CH(int CH)
{
ADC_ChannelConfTypeDef sConfig = {0};
switch(CH)
{
case 0:
sConfig.Channel = ADC_CHANNEL_0;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 1:
sConfig.Channel = ADC_CHANNEL_1;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 2:
sConfig.Channel = ADC_CHANNEL_2;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 3:
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 4:
sConfig.Channel = ADC_CHANNEL_4;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 5:
sConfig.Channel = ADC_CHANNEL_5;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 6:
sConfig.Channel = ADC_CHANNEL_6;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 7:
sConfig.Channel = ADC_CHANNEL_7;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 8:
sConfig.Channel = ADC_CHANNEL_8;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 9:
sConfig.Channel = ADC_CHANNEL_9;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 10:
sConfig.Channel = ADC_CHANNEL_10;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 11:
sConfig.Channel = ADC_CHANNEL_11;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 12:
sConfig.Channel = ADC_CHANNEL_12;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 13:
sConfig.Channel = ADC_CHANNEL_13;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 14:
sConfig.Channel = ADC_CHANNEL_14;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 15:
sConfig.Channel = ADC_CHANNEL_15;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
}
}
/* USER CODE END 0 */

/**
* @brief The application entry point.
* @retval int
*/
int main(void)
{

/* USER CODE BEGIN 1 */
uint8_t txd_msg_buffer[64]={0}; //define a message variable
void HCSR04_TRIG_PULSE(void)
{
HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
for(int j = 0; j!= 15; j=j+1){};
HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
}
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
MX_USART2_UART_Init();
MX_USART6_UART_Init();
MX_TIM3_Init();
MX_TIM5_Init();
MX_TIM4_Init();
MX_ADC1_Init();
MX_TIM2_Init();
/* USER CODE BEGIN 2 */

// Motor Variables
HAL_TIM_Base_Init(&htim3);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
HAL_TIM_Base_Start_IT(&htim5);

HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);

HAL_TIM_Base_Start(&htim2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */

TIM2->CCR1 = 500;
TIM3->CCR1 = 0;
TIM3->CCR3 = 0;

while (1)
{
// Start with pipeline and PWM variables at an invalid choice
for(uint8_t i = 1; i < 4; i++) {
pipeline[i] = 4;
PWM[i] = 4;
PWM_sequence[i] = 4;
}
pipeline_choice = 4;

// Start with motor off
Motor_DCVAL = 0;
TIM3->CCR1 = Motor_DCVAL;

// Start setup mode

sprintf((char*)txd_msg_buffer,"\r\n\n\n SETUP MODE");
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

sprintf((char*)txd_msg_buffer,"\r\n\n Enter SETUP Parameter");
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

// Keep asking for pipeline and PWM information until
while( (PWM[0] == 4) || (PWM[1] == 4) || (PWM[2] == 4) || (PWM[3] == 4) ) {

// Get pipeline number and wait for interrupt to occur
sprintf((char*)txd_msg_buffer,"\r\n\n PIPELINE (options: 0 to 3):");
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart6, &byte, 1);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
while(rcv_intpt_flag == (00)){}

// Adjust the received number for ASCII representation and store the chosen pipeline number
byte_number = byte - 48;
pipeline_choice = byte_number;

// Get PWM number and wait for interrupt to occur
sprintf((char*)txd_msg_buffer,"\r\n Pump PWM (options: 0 to 3):");
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart6, &byte, 1);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
while(rcv_intpt_flag == (00)){}

// Adjust the received number for ASCII representation
byte_number = byte - 48;

// Check that the pipeline choice is valid
if((pipeline_choice == 0) || (pipeline_choice == 1) || (pipeline_choice == 2) || (pipeline_choice == 3)) {
// Always put the inlet pipe at the beginning
if(pipeline_choice == 0) {
pipeline[0] = 0;
PWM[PWM_sequence[pipeline_choice]] = 0;
PWM[0] = byte_number;
// Check if the chosen pipeline has already been written to
} else if(PWM_sequence[pipeline_choice] != 4) {
// If so, overwrite it
pipeline[PWM_sequence[pipeline_choice]] = pipeline_choice;
PWM[PWM_sequence[pipeline_choice]] = byte_number;
} else {
// If not, fill the next available slot
for(uint8_t i = 1; i < 4; i++) {
if(pipeline[i] == 4) {
PWM_sequence[pipeline_choice] = i;
pipeline[i] = pipeline_choice;
PWM[i] = byte_number;
break;
}
}
}
}
}

// Get time info
for(uint8_t i = 0; i <= 3; i++) {
time = 24;

while((time < 0) || (time > 23)) {
// Get FIRST HOUR and wait for interrupt to occur
// Then adjust the received number for ASCII representation
sprintf((char*)txd_msg_buffer, "\r\n\n Pipeline %d Pump FIRST HOUR (options: 00 to 23): ", PWM_sequence[i]);
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart6, &byte, 1);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
while(rcv_intpt_flag == (00)){}
byte_number = byte - 48;

rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart6, &byte, 1);
while(rcv_intpt_flag == (00)){}
byte_number2 = byte - 48;

// Store the entered time to compare and set values
time = byte_number*10 + byte_number2;

StartHr[PWM_sequence[i]] = time;
}

time = 24;

while((time < 0) || (time > 23)) {
// Get LAST HOUR and wait for interrupt to occur
// Then adjust the received number for ASCII representation
sprintf((char*)txd_msg_buffer, "\r\n Pipeline %d Pump LAST HOUR (options: 00 to 23): ", PWM_sequence[i]);
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart6, &byte, 1);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
while(rcv_intpt_flag == (00)){}
byte_number = byte - 48;

rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart6, &byte, 1);
while(rcv_intpt_flag == (00)){}
byte_number2 = byte - 48;

// Store the entered time to compare and set values
time = byte_number*10 + byte_number2;

EndHr[PWM_sequence[i]] = time;
}

time = 24;
}

sprintf((char*)txd_msg_buffer,"\r\n\n Printing SETUP Parameters");
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

sprintf((char*)txd_msg_buffer, "\r\n\n\n Current Wall Clock Hour 0");
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

// Pipeline 0
sprintf((char*)txd_msg_buffer, "\r\n\n Pipeline: %d", pipeline[0]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump PWM: %d", PWM[0]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump First Hour: %d",StartHr[0]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump Last Hour: %d",EndHr[0]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

// Pipeline 1
sprintf((char*)txd_msg_buffer, "\r\n\n Pipeline: %d", pipeline[1]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump PWM: %d", PWM[1]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump First Hour: %d",StartHr[1]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump Last Hour: %d",EndHr[1]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

// Pipeline 2
sprintf((char*)txd_msg_buffer, "\r\n\n Pipeline: %d", pipeline[2]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump PWM: %d", PWM[2]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump First Hour: %d",StartHr[2]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump Last Hour: %d",EndHr[2]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

// Pipeline 3
sprintf((char*)txd_msg_buffer, "\r\n\n Pipeline: %d", pipeline[3]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump PWM: %d", PWM[3]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump First Hour: %d",StartHr[3]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " Pump Last Hour: %d",EndHr[3]);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

// Indicate that setup is done and to press the button to continue
sprintf((char*)txd_msg_buffer, "\r\n\n\n SETUP is done. Press the Blue Button for RUN MODE");
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

//set LD2 to flashing and wait for RUN MDOE
HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
while (HAL_GPIO_ReadPin(GPIOC, B1_Pin) == GPIO_PIN_SET){
HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
HAL_Delay(250);

}

// Turn the LED steady on
HAL_GPIO_WritePin(GPIOA,LD2_Pin, GPIO_PIN_SET);
HAL_Delay(1000);

// Set Clock Time to 00:00:00
clock_hours = 0;
clock_mins = 0;
clock_secs = 0;
flag = 0;

// Display header for data each hour
sprintf((char*)txd_msg_buffer, "\r\n\n\n HOUR : PIPE : PWM : RPM : DEPTH");
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

// Get Ultrasonic sensor data
hcsr04_Rx_flag = 0; // reset the interrupt flag for the HCSR04
first_edge = 0; // set it back to false
time_edge1 = 0;
time_edge2 = 0;
time_diff = 0;
distance = 0;

HCSR04_TRIG_PULSE();
HAL_Delay(500);

while (hcsr04_Rx_flag == 0) {}

// enter your difference in time between the edges here
if(time_edge2 < time_edge1) {
time_diff = (TIM4->ARR - time_edge1) + time_edge2 +1;
} else {
time_diff = time_edge2 - time_edge1;
}

// enter your distance calculation here that uses HCSR04 datasheet information
distance = time_diff / 58;
if(distance > MAX_distance) {distance = MAX_distance;}
depth = (100*(MAX_distance - distance) / MAX_distance);

//display depth
A_display = depth/10;
B_display = depth%10;
DIGITS_Display(A_display, B_display);

//reset the counter
TIM5->CNT = 0;

// RUN MODE
while(clock_hours < 24 && depth > 0) {

//reset motor TICK count
rpm_tick_count = 0;

current_pipe = 4;
// Turn motor on
motor_flag = 0;

// Check which pipeline is running
for(uint8_t i = 0; i < 4; i++) {
if((clock_hours >= StartHr[i]) && (clock_hours <= EndHr[i])) {
current_pipe = i;
}
}

// Change LED and servo motor to indicate current pipeline
if(pipeline[current_pipe] == 0) {
HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);//RED is on
HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);//BLUE is on

TIM2->CCR1 = 1000;
HAL_Delay(1);
} else if(pipeline[current_pipe] == 1) {
HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET); //RED is on
HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);

TIM2->CCR1 = 1500;
HAL_Delay(1);
} else if(pipeline[current_pipe] == 2) {
HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);//GREEN is on
HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);

TIM2->CCR1 = 2000;
HAL_Delay(1);
} else if(pipeline[current_pipe] == 3) {
HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);//BLUE is on

TIM2->CCR1 = 2500;
HAL_Delay(1);
} else {
HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);

TIM2->CCR1 = 500;
HAL_Delay(1);
}

while (flag == 0){
// Find the correct speed for the motor
if(PWM[current_pipe] == 0 ) {
if(depth < 90){
ADC_Select_CH(9);
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1, 1000);
uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
HAL_ADC_Stop(&hadc1);
Motor_DCVAL = ADC_CH9*2000/255;
} else {
Motor_DCVAL = 0;
}
} else if(PWM[current_pipe] == 1) {
Motor_DCVAL = 1400;
} else if(PWM[current_pipe] == 2) {
Motor_DCVAL = 1700;
} else if(PWM[current_pipe] == 3) {
Motor_DCVAL = 1980;
} else {
Motor_DCVAL = 0;
}
// Set the motor to the correct speed with direction depending on the pipe number
if(pipeline[current_pipe] == 0) {
TIM3->CCR3 = 0;
TIM3->CCR1 = Motor_DCVAL;
} else {
TIM3->CCR1 = 0;
TIM3->CCR3 = Motor_DCVAL;
}
HAL_Delay(1);
}
flag = 0;
rpm = (rpm_tick_count/120);

// Get Ultrasonic sensor data
hcsr04_Rx_flag = 0; // reset the interrupt flag for the HCSR04
first_edge = 0; // set it back to false
time_edge1 = 0;
time_edge2 = 0;
time_diff = 0;
distance = 0;

HCSR04_TRIG_PULSE();
HAL_Delay(500);

while (hcsr04_Rx_flag == 0) {}

// enter your difference in time between the edges here
if(time_edge2 < time_edge1) {
time_diff = (TIM4->ARR - time_edge1) + time_edge2 +1;
} else {
time_diff = time_edge2 - time_edge1;
}

// enter your distance calculation here that uses HCSR04 datasheet information
distance = time_diff / 58;
if(distance > MAX_distance) {distance = MAX_distance;}
depth = (100*(MAX_distance - distance) / MAX_distance);

A_display = depth/10;
B_display = depth%10;
DIGITS_Display(A_display, B_display);

//print out the data
sprintf((char*)txd_msg_buffer, "\n\r %d : ", clock_hours - 1);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
if(pipeline[current_pipe] == 4) {
sprintf((char*)txd_msg_buffer, " : " );
} else {
sprintf((char*)txd_msg_buffer, " %d : ", pipeline[current_pipe]);
}
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
if(PWM[current_pipe] == 4) {
sprintf((char*)txd_msg_buffer, " : " );
} else {
sprintf((char*)txd_msg_buffer, " %d : ", PWM[current_pipe]);
}
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " %d : ", rpm);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
sprintf((char*)txd_msg_buffer, " %d", depth);
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
}

//Turn off all LEDs when it reaches 24 hours
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);

// Special event when the reservoir is empty
if(depth <= 0) {
// Turn off motor
TIM3->CCR3 = 0;
TIM3->CCR1 = 0;

// Print message to UART
sprintf((char*)txd_msg_buffer, "\n\n\r ### RESEVOIR IS EMPTY ###");
HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);

// Flash LED white
while(1) {
HAL_GPIO_TogglePin(GPIOC, GREEN_Pin);
HAL_GPIO_TogglePin(GPIOC, RED_Pin);
HAL_GPIO_TogglePin(GPIOC, BLUE_Pin);
HAL_Delay(500);
}
}

while(1){}

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

/** Configure the main internal regulator output voltage
*/
__HAL_RCC_PWR_CLK_ENABLE();
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
hadc1.Init.Resolution = ADC_RESOLUTION_8B;
hadc1.Init.ScanConvMode = ENABLE;
hadc1.Init.ContinuousConvMode = DISABLE;
hadc1.Init.DiscontinuousConvMode = DISABLE;
hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
hadc1.Init.NbrOfConversion = 1;
hadc1.Init.DMAContinuousRequests = DISABLE;
hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
if (HAL_ADC_Init(&hadc1) != HAL_OK)
{
Error_Handler();
}

/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
*/
sConfig.Channel = ADC_CHANNEL_9;
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
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE BEGIN TIM2_Init 1 */

/* USER CODE END TIM2_Init 1 */
htim2.Instance = TIM2;
htim2.Init.Prescaler = 16-1;
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = 20000-1;
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 500-1;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM2_Init 2 */

/* USER CODE END TIM2_Init 2 */
HAL_TIM_MspPostInit(&htim2);

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
TIM_OC_InitTypeDef sConfigOC = {0};
TIM_IC_InitTypeDef sConfigIC = {0};

/* USER CODE BEGIN TIM3_Init 1 */

/* USER CODE END TIM3_Init 1 */
htim3.Instance = TIM3;
htim3.Init.Prescaler = 16-1;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 2000-1;
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 1200-1;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
{
Error_Handler();
}
sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
sConfigIC.ICFilter = 0;
if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
{
Error_Handler();
}
sConfigOC.Pulse = 0;
if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM3_Init 2 */

/* USER CODE END TIM3_Init 2 */
HAL_TIM_MspPostInit(&htim3);

}

/**
* @brief TIM4 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM4_Init(void)
{

/* USER CODE BEGIN TIM4_Init 0 */

/* USER CODE END TIM4_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_IC_InitTypeDef sConfigIC = {0};

/* USER CODE BEGIN TIM4_Init 1 */

/* USER CODE END TIM4_Init 1 */
htim4.Instance = TIM4;
htim4.Init.Prescaler = 16-1;
htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
htim4.Init.Period = 65535;
htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
sConfigIC.ICFilter = 0;
if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM4_Init 2 */

/* USER CODE END TIM4_Init 2 */

}

/**
* @brief TIM5 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM5_Init(void)
{

/* USER CODE BEGIN TIM5_Init 0 */

/* USER CODE END TIM5_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};

/* USER CODE BEGIN TIM5_Init 1 */

/* USER CODE END TIM5_Init 1 */
htim5.Instance = TIM5;
htim5.Init.Prescaler = 53.3333-1;
htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
htim5.Init.Period = 3600000-1;
htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM5_Init 2 */

/* USER CODE END TIM5_Init 2 */

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
* @brief USART6 Initialization Function
* @param None
* @retval None
*/
static void MX_USART6_UART_Init(void)
{

/* USER CODE BEGIN USART6_Init 0 */

/* USER CODE END USART6_Init 0 */

/* USER CODE BEGIN USART6_Init 1 */

/* USER CODE END USART6_Init 1 */
huart6.Instance = USART6;
huart6.Init.BaudRate = 9600;
huart6.Init.WordLength = UART_WORDLENGTH_8B;
huart6.Init.StopBits = UART_STOPBITS_1;
huart6.Init.Parity = UART_PARITY_NONE;
huart6.Init.Mode = UART_MODE_TX_RX;
huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart6.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart6) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN USART6_Init 2 */

/* USER CODE END USART6_Init 2 */

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
HAL_GPIO_WritePin(GPIOC, RED_Pin|BLUE_Pin|GREEN_Pin|DIGIT_B0_Pin
|DIGIT_B1_Pin|DIGIT_B3_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIGIT_A1_Pin|DIGIT_A0_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin|DIGIT_A3_Pin|HCSR04_TRIG_Pin|DIGIT_B2_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin : B1_Pin */
GPIO_InitStruct.Pin = B1_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pins : RED_Pin BLUE_Pin GREEN_Pin DIGIT_B0_Pin
DIGIT_B1_Pin DIGIT_B3_Pin */
GPIO_InitStruct.Pin = RED_Pin|BLUE_Pin|GREEN_Pin|DIGIT_B0_Pin
|DIGIT_B1_Pin|DIGIT_B3_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/*Configure GPIO pins : LD2_Pin DIGIT_A1_Pin DIGIT_A0_Pin */
GPIO_InitStruct.Pin = LD2_Pin|DIGIT_A1_Pin|DIGIT_A0_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/*Configure GPIO pin : RPM_Tick_Pin */
GPIO_InitStruct.Pin = RPM_Tick_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(RPM_Tick_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pins : DIGIT_A2_Pin DIGIT_A3_Pin HCSR04_TRIG_Pin DIGIT_B2_Pin */
GPIO_InitStruct.Pin = DIGIT_A2_Pin|DIGIT_A3_Pin|HCSR04_TRIG_Pin|DIGIT_B2_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// UART Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if (huart->Instance == USART6)
{
HAL_UART_Transmit(&huart6, &byte, 1, 100);
rcv_intpt_flag = 1;
}
}

//ultrasonic sensor call_back routine
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM4)
{
if (htim->Channel == 2) // if the interrupt source is channel 2
{
if (first_edge==0) // if the first value is not captured
{
time_edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
first_edge = 1; // set the first captured as true
}
else // if the first is already captured
{
time_edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the second value
__HAL_TIM_SET_COUNTER(htim,0); // reset the counter
hcsr04_Rx_flag = 1; // set the interrupt flag for result done
}
}
}
}

// Motor RPM callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if(GPIO_Pin == RPM_Tick_Pin)
{
rpm_tick_count += 1;
}
}

// Wall Clock Interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
if((htim->Instance == TIM5)) {
flag = 1;
clock_hours ++;
/*flag = 0; // screen updates occur hourly on the half-hour
clock_secs ++;
if(clock_secs == 60) {
clock_mins ++;
clock_secs = 0;
if(clock_mins == 60) {
clock_hours ++;
clock_mins = 0;
flag = 1; // screen updates occur hourly on the hour

}
}*/
}
}
/* USER CODE END 4 */

/**
* @brief This function is executed in case of error occurrence.
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
#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
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