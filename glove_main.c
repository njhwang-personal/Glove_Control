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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

////// Define for Bno085 //////
#define RAD_TO_DEG (57.2957795131f)
#define SCALE_Q14 (1.0f/16384.0f)
#define BNO_I2C_ADDRESS (0x4A << 1)
#define BNO_I2C_HANDLE &hi2c1
#define BNO_MSG_LENGTH 21
#define BNO_READ_PERIOD 20 //ms


ADC_HandleTypeDef hadc;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart4;
SPI_HandleTypeDef hspi1;


uint8_t START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ[/* BNO_MSG_LENGTH */] =
{ 0x15, 0x00, 0x02, 0x00, 0xFD, 0x28, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t FishedOutMessage[BNO_MSG_LENGTH];
uint8_t BnoRxBuff[BNO_MSG_LENGTH];
static int16_t Data1, Data2, Data3, Data4;



typedef struct __attribute__((packed)){
    uint8_t  magic;      //0xAA
    uint8_t  version;
    uint16_t seq;

    int16_t  yaw;
    int16_t  pitch;
    int16_t  roll;

    uint8_t  flex[5];
    uint8_t  contacts;

    uint16_t batt_mv;    //battery (unused)

    uint8_t  reserved[12];  //future use

    uint16_t checksum;   //16-bit sum of first 30 bytes
} GlovePacket;

static uint16_t g_seq = 0; //global sequence counter





float Y_callibrate, P_callibrate, R_callibrate;
int iteration_enregistrement = 0;
static int calibration = 15;
uint32_t BnoSoftTimer;

#define DELTA_1I_Y (2)
#define DELTA_2I_Y (10)
#define DELTA_1D_Y (2)
#define DELTA_2D_Y (10)

#define DELTA_1I_P (2)
#define DELTA_2I_P (10)
#define DELTA_1D_P (2)
#define DELTA_2D_P (10)

#define DELTA_1I_R (2)
#define DELTA_2I_R (10)
#define DELTA_1D_R (2)
#define DELTA_2D_R (10)


int number_mesures = 5;


////// Define for Flex and FSR //////

#define FLEX_SENSOR_COUNT 4


static const uint32_t flex_channel_masks[FLEX_SENSOR_COUNT] = {
    ADC_CHANNEL_1,   //Flex 0
    ADC_CHANNEL_2,   //Flex 1
    ADC_CHANNEL_3,   //Flex 2
    ADC_CHANNEL_4    //Flex 3
};


#define FSR_GPIO_PORT GPIOA
#define FSR_GPIO_PIN GPIO_PIN_5

#define FSR_SAMPLE_PERIOD_MS 5 //~200 Hz
#define FLEX_SAMPLE_PERIOD_MS 10 //~100 Hz


//FSR thresholds
#define FSR_DELTA_PRESSED 2000
#define FSR_DELTA_RELEASED 500

//Flex thresholds
#define DELTA_FLEX_LOW 50
#define DELTA_FLEX_MID_1 75
#define DELTA_FLEX_MID_2 100
#define DELTA_FLEX_HIGH 130


static uint16_t flex_cal_adc[FLEX_SENSOR_COUNT];
static uint8_t  flex_last_state[FLEX_SENSOR_COUNT];
static uint32_t flex_last_ts_ms = 0;




///////////////////////////////  RADIO  ////////////////////////////////
#define NRF_CE_PORT   GPIOB
#define NRF_CE_PIN    GPIO_PIN_0
#define NRF_CSN_PORT  GPIOB
#define NRF_CSN_PIN   GPIO_PIN_1


#define NRF_CMD_R_REGISTER     0x00
#define NRF_CMD_W_REGISTER     0x20
#define NRF_CMD_R_RX_PAYLOAD   0x61
#define NRF_CMD_W_TX_PAYLOAD   0xA0
#define NRF_CMD_FLUSH_TX       0xE1
#define NRF_CMD_FLUSH_RX       0xE2
#define NRF_CMD_NOP            0xFF


#define NRF_REG_CONFIG         0x00
#define NRF_REG_EN_AA          0x01
#define NRF_REG_EN_RXADDR      0x02
#define NRF_REG_SETUP_AW       0x03
#define NRF_REG_SETUP_RETR     0x04
#define NRF_REG_RF_CH          0x05
#define NRF_REG_RF_SETUP       0x06
#define NRF_REG_STATUS         0x07
#define NRF_REG_RX_ADDR_P0     0x0A
#define NRF_REG_TX_ADDR        0x10
#define NRF_REG_RX_PW_P0       0x11
#define NRF_REG_FIFO_STATUS    0x17
#define NRF_REG_DYNPD          0x1C



#define NRF_MASK_RX_DR         (1 << 6)
#define NRF_MASK_TX_DS         (1 << 5)
#define NRF_MASK_MAX_RT        (1 << 4)
#define NRF_EN_CRC             (1 << 3)
#define NRF_CRCO               (1 << 2)
#define NRF_PWR_UP             (1 << 1)
#define NRF_PRIM_RX            (1 << 0)


#define NRF_RX_DR              (1 << 6)
#define NRF_TX_DS              (1 << 5)
#define NRF_MAX_RT             (1 << 4)


static inline void nrf_csn_low(void){ HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET); }
static inline void nrf_csn_high(void){ HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET); }
static inline void nrf_ce_low(void){ HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET); }
static inline void nrf_ce_high(void){ HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET); }

int Bno_read(float *yaw, float *pitch, float *roll);
void Bno_init(void);

void Flex_callibration(void);
void Flex_Task(uint8_t out_states[FLEX_SENSOR_COUNT]);

//void FSR_callibration(void);
uint8_t FSR_Task(void);



static uint8_t nrf_spi_xfer(uint8_t byte)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx, 1, HAL_MAX_DELAY);
    return rx;
}



static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t val;
    nrf_csn_low();
    nrf_spi_xfer(NRF_CMD_R_REGISTER | (reg & 0x1F));
    val = nrf_spi_xfer(0xFF);
    nrf_csn_high();
    return val;
}


static void nrf_write_reg(uint8_t reg, uint8_t val)
{
    nrf_csn_low();
    nrf_spi_xfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
    nrf_spi_xfer(val);
    nrf_csn_high();
}



static void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    nrf_csn_low();
    nrf_spi_xfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++)
        nrf_spi_xfer(buf[i]);
    nrf_csn_high();
}


static void nrf_flush_tx(void)
{
    nrf_csn_low();
    nrf_spi_xfer(NRF_CMD_FLUSH_TX);
    nrf_csn_high();
}



static void nrf_flush_rx(void)
{
    nrf_csn_low();
    nrf_spi_xfer(NRF_CMD_FLUSH_RX);
    nrf_csn_high();
}



static void nrf_clear_irq(void)
{
    nrf_write_reg(NRF_REG_STATUS, NRF_RX_DR | NRF_TX_DS | NRF_MAX_RT);
}




void nrf_init_tx(void)
{
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

    nrf_ce_low();
    nrf_csn_high();
    HAL_Delay(5);

    nrf_write_reg(NRF_REG_EN_AA, 0x00);

    nrf_write_reg(NRF_REG_EN_RXADDR, 0x01);

    nrf_write_reg(NRF_REG_SETUP_AW, 0x03);

    nrf_write_reg(NRF_REG_SETUP_RETR, 0x00);

    nrf_write_reg(NRF_REG_RF_CH, 76);

    nrf_write_reg(NRF_REG_RF_SETUP, 0x06);

    nrf_write_buf(NRF_REG_TX_ADDR, addr, 5);
    nrf_write_buf(NRF_REG_RX_ADDR_P0, addr, 5);

    nrf_write_reg(NRF_REG_RX_PW_P0, 32);

    nrf_write_reg(NRF_REG_DYNPD, 0x00);

    nrf_flush_tx();
    nrf_flush_rx();
    nrf_clear_irq();


    uint8_t config = NRF_EN_CRC | NRF_PWR_UP; //0x0A
    nrf_write_reg(NRF_REG_CONFIG, config);

    HAL_Delay(5);

}



bool nrf_send_payload(const uint8_t *data, uint8_t len)

{
    if (len == 0 || len > 32)
        return false;

    uint8_t cfg = nrf_read_reg(NRF_REG_CONFIG);
    cfg &= ~NRF_PRIM_RX;
    cfg |= NRF_PWR_UP;
    nrf_write_reg(NRF_REG_CONFIG, cfg);
    HAL_Delay(5);

    nrf_flush_tx();
    nrf_clear_irq();

    //Write payload
    nrf_csn_low();
    nrf_spi_xfer(NRF_CMD_W_TX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++)
        nrf_spi_xfer(data[i]);
    nrf_csn_high();

    nrf_ce_high();

    for (volatile int i = 0; i < 1000; i++) __NOP();
    nrf_ce_low();

    uint32_t start = HAL_GetTick();
    while (1)
    {
        uint8_t status = nrf_read_reg(NRF_REG_STATUS);
        if (status & NRF_TX_DS)
        {
            nrf_clear_irq();
            return true;
        }
        if (status & NRF_MAX_RT)
        {
            nrf_clear_irq();
            nrf_flush_tx();
            return false;    //failed
        }
        if (HAL_GetTick() - start > 100)
        {
            return false;
        }
    }
}
/////////////////////////////////  END RADIO SECTION  /////////////////////////////////


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/



static int build_glove_packet_from_sensors(GlovePacket *pkt)
{

    static float last_yaw   = 0.0f;
    static float last_pitch = 0.0f;
    static float last_roll  = 0.0f;

    float yaw, pitch, roll;

    if (Bno_read(&yaw, &pitch, &roll)){
        last_yaw   = yaw;
        last_pitch = pitch;
        last_roll  = roll;
    }

    uint8_t fsr  = FSR_Task();
    uint8_t flex_states[FLEX_SENSOR_COUNT];

    Flex_Task(flex_states);
    memset(pkt, 0, sizeof(*pkt));

    pkt->magic   = 0xAA;
    pkt->version = 1;
    pkt->seq     = g_seq++;

    pkt->yaw   = (int16_t)(last_yaw   * 100.0f);
    pkt->pitch = (int16_t)(last_pitch * 100.0f);
    pkt->roll  = (int16_t)(last_roll  * 100.0f);


    pkt->flex[0] = flex_states[0];
    pkt->flex[1] = flex_states[1];
    pkt->flex[2] = flex_states[2];
    pkt->flex[3] = flex_states[3];
    pkt->flex[4] = 0;


    pkt->contacts = 0;
    if (fsr){
        pkt->contacts |= (1 << 0);
    }

    pkt->batt_mv = 0;

    const uint8_t *bytes = (const uint8_t *)pkt;
    uint16_t sum = 0;
    for(int i = 0; i < 30; i++){
        sum += bytes[i];
    }
    pkt->checksum = sum;

    return 1;
}









/* USER CODE BEGIN PV */

int __io_putchar(int ch){
	uint8_t c = (uint8_t)ch ;
	if (c == '\n') {
	uint8_t cr = '\r';
	HAL_UART_Transmit(&huart4, &cr, 1, HAL_MAX_DELAY);
	}
	HAL_UART_Transmit(&huart4, &c, 1, HAL_MAX_DELAY);
	return ch;
}



static void ADC_SelectSingleChannel(ADC_HandleTypeDef *phadc, uint32_t ch_mask)
{

    HAL_ADC_Stop(phadc);

    phadc->Instance->CHSELR = 0;
    phadc->Instance->CHSELR = ch_mask;
}



float my_fmodf(float x, float y){
    if (y == 0.0f) {

        return 0.0f / 0.0f;
    }

    int quotient = (int)(x / y);
    float result = x - (float)quotient * y;


    if ((result > 0.0f && y < 0.0f) || (result < 0.0f && y > 0.0f)){
        result += y;
    }

    return result;
}

static float mod360(float x){
    float r = my_fmodf(x, 360.0);
    return (r < 0.0) ? r + 360.0 : r;
}


static float angle_diff_signed(float a_before, float a_after){
    return mod360((a_after - a_before) + 540.0) - 180.0;
}



/////////////////// Functions for BNO ////////////////////////
void bno_callibration(float y_c, float p_c, float r_c){
	Y_callibrate = y_c;
	P_callibrate = p_c ;
	R_callibrate = r_c;
	printf("c\r\n");
	//printf("Yc %ld\r\n", (long)lroundf(y_c));
	//printf("Pc %ld\r\n", (long)lroundf(p_c));
	//printf("Rc %ld\r\n", (long)lroundf(r_c));
}


int Bno_read(float *yaw, float *pitch, float *roll)
{
    float Qi, Qj, Qk, Qr;

    if (HAL_GetTick() - BnoSoftTimer < BNO_READ_PERIOD){
        return 0;
    }
    BnoSoftTimer = HAL_GetTick();

    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(
        BNO_I2C_HANDLE,
        BNO_I2C_ADDRESS,
        BnoRxBuff,
        BNO_MSG_LENGTH,
        10
    );

    if (st != HAL_OK) {
         printf("I2C RX err=0x%lX\r\n", HAL_I2C_GetError(BNO_I2C_HANDLE));
        return 0;
    }

     printf("BNO RAW: ");
     for (int i = 0; i < BNO_MSG_LENGTH; i++){
         printf("%02X ", BnoRxBuff[i]);
     }
     printf("\r\n");

    int rep_idx = -1;
    for (int i = 0; i < BNO_MSG_LENGTH; i++){
        if (BnoRxBuff[i] == 0x28) {
            rep_idx = i;
            break;
        }
    }

    if (rep_idx < 0){
        return 0;
    }

    if (rep_idx + 11 >= BNO_MSG_LENGTH){
        return 0;
    }

    int16_t d1 = (int16_t)(((uint16_t)BnoRxBuff[rep_idx + 5] << 8) | BnoRxBuff[rep_idx + 4]);
    int16_t d2 = (int16_t)(((uint16_t)BnoRxBuff[rep_idx + 7] << 8) | BnoRxBuff[rep_idx + 6]);
    int16_t d3 = (int16_t)(((uint16_t)BnoRxBuff[rep_idx + 9] << 8) | BnoRxBuff[rep_idx + 8]);
    int16_t d4 = (int16_t)(((uint16_t)BnoRxBuff[rep_idx + 11] << 8) | BnoRxBuff[rep_idx + 10]);

    Qi = d2 * SCALE_Q14;
    Qj = d3 * SCALE_Q14;
    Qk = d4 * SCALE_Q14;
    Qr = d1 * SCALE_Q14;

    float qi2 = Qi * Qi;
    float qj2 = Qj * Qj;
    float qk2 = Qk * Qk;
    float qr2 = Qr * Qr;
    float denom = qi2 + qj2 + qk2 + qr2;
    if (denom == 0.0f){
        return 0;
    }

    *yaw   = atan2f(2.0f * (Qi * Qj + Qk * Qr),
                    (qi2 - qj2 - qk2 + qr2))*RAD_TO_DEG;
    *pitch = asinf(-2.0f * (Qi * Qk - Qj * Qr) / denom) * RAD_TO_DEG;
    *roll  = atan2f(2.0f * (Qj * Qk + Qi * Qr),
                    (-qi2 - qj2 + qk2 + qr2))*RAD_TO_DEG;

    iteration_enregistrement++;
    if (iteration_enregistrement == calibration){
        bno_callibration(*yaw, *pitch, *roll);
    }

    return 1;
}

void Bno_init(void){
	HAL_I2C_Master_Transmit(BNO_I2C_HANDLE, BNO_I2C_ADDRESS,
	START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ,
	sizeof(START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ), 10);
	HAL_Delay(100);
	BnoSoftTimer = HAL_GetTick();
}



/////////////////////////////// Function for Flex and FSR ////////////////////////////////////////

static uint16_t ADC_ReadChannel(uint32_t channel_mask)
{
    ADC_SelectSingleChannel(&hadc, channel_mask);

    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
    (void)HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
    (void)HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);

    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);

    return val;

}



static uint16_t ADC_ReadAvg(uint32_t channel_mask, uint8_t n)
{
    uint32_t acc = 0;
    for (uint8_t i = 0; i < n; i++) {
        acc += ADC_ReadChannel(channel_mask);
    }
    return (uint16_t)(acc / n);
}

void Flex_callibration(void){
	const uint8_t samples = 8;

    for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
        uint16_t adc = ADC_ReadAvg(flex_channel_masks[i], samples);
        flex_cal_adc[i]   = adc;
        flex_last_state[i] = 0;
    	printf("Flex %d cal_adc = %u \r\n", i, adc);
    }

}


void Flex_Task(uint8_t out_states[FLEX_SENSOR_COUNT])
{
    const uint8_t samples = 8;
    uint32_t now = HAL_GetTick();

    if ((now - flex_last_ts_ms) < FLEX_SAMPLE_PERIOD_MS){

        for(int i = 0; i < FLEX_SENSOR_COUNT; i++){
            out_states[i] = flex_last_state[i];
        }
        return;
    }
    flex_last_ts_ms = now;

    for(int i = 0; i < FLEX_SENSOR_COUNT; i++){
        uint16_t adc = ADC_ReadAvg(flex_channel_masks[i], samples);
        uint16_t cal = flex_cal_adc[i];
        uint16_t diff = (adc > cal) ? (adc - cal) : (cal - adc);

        uint8_t state;
        if (diff >= DELTA_FLEX_HIGH){
            state = 4;
        } 
        else if (diff >= DELTA_FLEX_MID_2){
            state = 3;
        } 
        else if (diff >= DELTA_FLEX_MID_1){
            state = 2;
        } 
        else if (diff >= DELTA_FLEX_LOW){
            state = 1;
        } 
        else{
            state = 0;
        }

        if (state != flex_last_state[i]) {
            flex_last_state[i] = state;
            // printf("Flex %d state = %u (ADC=%u, cal=%u)\r\n", i, state, adc, cal);
        }

        out_states[i] = state;
        //printf("Flex%d: adc = %u cal=%u diff=%u state=%u\r\n", i, adc, cal, diff, state);
    }
}


uint8_t FSR_Task(void)	//changed from adc to gpio
{
// static uint32_t last_ts_ms = 0;
// static uint8_t is_pressed = 0;
// const uint8_t samples = 8;
//
// uint32_t now = HAL_GetTick();
// if ((now - last_ts_ms) < FSR_SAMPLE_PERIOD_MS){
//    return is_pressed;
// }
//
// last_ts_ms = now;
//
// uint16_t adc = ADC_ReadAvg(FSR_ADC_CHANNEL, samples);
//
// //printf("adc fsr %u\r\n", adc);
//
// uint16_t diff = (adc > fsr_cal_adc) ? (adc - fsr_cal_adc) : (fsr_cal_adc - adc);
// if (!is_pressed) {
//    if (diff >= FSR_DELTA_PRESSED) {
//    is_pressed = 1;
//    //printf("Pressed\r\n");
//    }
// } else {
//    if (diff <= FSR_DELTA_RELEASED) {
//        is_pressed = 0;
//        //printf("Released\r\n");
//    }
// }
//return is_pressed;

	GPIO_PinState s = HAL_GPIO_ReadPin(FSR_GPIO_PORT, FSR_GPIO_PIN);
	return (s == GPIO_PIN_RESET) ? 1 : 0;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief The application entry point.
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
    MX_I2C1_Init();
    MX_USART4_UART_Init();
    MX_ADC_Init();
    MX_SPI1_Init();
    nrf_init_tx();

    /* USER CODE BEGIN 2 */
    Bno_init();

 // bno_callibration(&Y_callibrate, &P_callibrate, &R_callibrate);
	printf("D\n");
	uint8_t fsr = 0;
	Flex_callibration();
	//FSR_callibration();

    GlovePacket pkt;
 /* USER CODE END 2 */

 /* USER CODE BEGIN WHILE */
    printf("entering loop \n");
    /* USER CODE BEGIN 2 */
    printf("Scanning I2C bus...\r\n");
    HAL_StatusTypeDef result;
    for(int i = 1; i < 128; i++) {

    	result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 2);
    	if(result == HAL_OK){
    		printf("Device found at 0x%02X\r\n", i);
    		HAL_Delay(1);
    	}
    }

    /* USER CODE END 2 */
    //Bno_init();
 while (1)

 {
 /* USER CODE END WHILE */


 /* USER CODE BEGIN 3 */

    // New: build and send RF packet when IMU has fresh data
	 printf("looking for packet \n");
        if(build_glove_packet_from_sensors(&pkt)){
            int ok = nrf_send_payload((uint8_t *)&pkt, sizeof(pkt));

            char dbg[64];
            snprintf(dbg, sizeof(dbg), "TX ok=%d seq=%u yaw=%d pitch=%d roll=%d flex= [%u, %u, %u, %u], fsr=%u\r\n",ok, pkt.seq, (int)pkt.yaw, (int)pkt.pitch, (int)pkt.roll, pkt.flex[0],
            		pkt.flex[1], pkt.flex[2], pkt.flex[3], (pkt.contacts & 0x01) ? 1 : 0);
            HAL_UART_Transmit(&huart4, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
        }
      HAL_Delay(20);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00000608;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
