#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>



/* Memory sections addresses */
#define MLX90632_ADDR_RAM   0x4000 /**< Start address of ram */
#define MLX90632_ADDR_EEPROM    0x2480 /**< Start address of user eeprom */

/* EEPROM addresses - used at startup */
#define MLX90632_EE_CTRL    0x24d4 /**< Control register initial value */
#define MLX90632_EE_CONTROL MLX90632_EE_CTRL /**< More human readable for Control register */

#define MLX90632_SENSOR_REG_CTRL   0x3001U
#define MLX90632_SENSOR_REG_STATUS 0x3FFFU

#define MLX90632_EE_I2C_ADDRESS 0x24d5 /**< I2C address register initial value */
#define MLX90632_EE_VERSION 0x240b /**< EEPROM version reg - assumed 0x101 */

#define MLX90632_EE_P_R     0x240c /**< Calibration constant ambient reference register 32bit */
#define MLX90632_EE_P_G     0x240e /**< Calibration constant ambient gain register 32bit */
#define MLX90632_EE_P_T     0x2410 /**< Calibration constant ambient tc2 register 32bit */
#define MLX90632_EE_P_O     0x2412 /**< Calibration constant ambient offset register 32bit */
#define MLX90632_EE_Aa      0x2414 /**< Aa calibration const register 32bit */
#define MLX90632_EE_Ab      0x2416 /**< Ab calibration const register 32bit */
#define MLX90632_EE_Ba      0x2418 /**< Ba calibration const register 32bit */
#define MLX90632_EE_Bb      0x241a /**< Bb calibration const register 32bit */
#define MLX90632_EE_Ca      0x241c /**< Ca calibration const register 32bit */
#define MLX90632_EE_Cb      0x241e /**< Cb calibration const register 32bit */
#define MLX90632_EE_Da      0x2420 /**< Da calibration const register 32bit */
#define MLX90632_EE_Db      0x2422 /**< Db calibration const register 32bit */
#define MLX90632_EE_Ea      0x2424 /**< Ea calibration constant register 32bit */
#define MLX90632_EE_Eb      0x2426 /**< Eb calibration constant register 32bit */
#define MLX90632_EE_Fa      0x2428 /**< Fa calibration constant register 32bit */
#define MLX90632_EE_Fb      0x242a /**< Fb calibration constant register 32bit */
#define MLX90632_EE_Ga      0x242c /**< Ga calibration constant register 32bit */
#define MLX90632_EE_Gb      0x242e /**< Ambient Beta calibration constant 16bit */
#define MLX90632_EE_Ka      0x242f /**< IR Beta calibration constant 16bit */
#define MLX90632_EE_Kb      0x2430 /**< IR Beta calibration constant 16bit */
#define MLX90632_EE_Ha      0x2481 /**< Ha customer calibration value register 16bit */
#define MLX90632_EE_Hb      0x2482 /**< Hb customer calibration value register 16bit */

#define MLX90632_EE_MEDICAL_MEAS1      0x24E1 /**< Medical measurement 1 16bit */
#define MLX90632_EE_MEDICAL_MEAS2      0x24E2 /**< Medical measurement 2 16bit */
#define MLX90632_EE_EXTENDED_MEAS1     0x24F1 /**< Extended measurement 1 16bit */
#define MLX90632_EE_EXTENDED_MEAS2     0x24F2 /**< Extended measurement 2 16bit */
#define MLX90632_EE_EXTENDED_MEAS3     0x24F3 /**< Extended measurement 3 16bit */


//-------------Data for Reset of sensors----------------------------------------------------//

#define MSB_RESET_DATA 0x3005U
#define LSB_RESET_DATA 0x0006U

//---------------------------------------------------------------------------------------------//

//----Data for 50mili seconds refresh rate for the sensors according to the datasheet----//

#define MLX90632_EE_EXTENDED_MEAS1_DATA     0x8700 /**< Extended data 3 16bit */
#define MLX90632_EE_EXTENDED_MEAS2_DATA     0x8712 /**< Extended data 3 16bit */
#define MLX90632_EE_EXTENDED_MEAS3_DATA     0x870C /**< Extended data 3 16bit */

//---------------------------------------------------------------------------------------------//

#define REFRESH_UNLOCK_MSB 0x3005U
#define REFRESH_UNLOCK_LSB 0x554CU
#define REFRESH_RATE_DELAY_MS 10U

//---------------------------------------------------------------------------------------------//

//----Adreses of memory----//

#define RAM_52 0x4033U

//---------------------------------------------------------------------------------------------//

#define I2C_TIMEOUT_MS 5U

//----------------------POW Constants--------------------------------------//

#define CONST_2POW_P19 524288.0F

#define CONST_2POW_N8 0.00390625F
#define CONST_2POW_N10 0.0009765625F
#define CONST_2POW_N16 0.00001525878F
#define CONST_2POW_N20 9.53674316e-7F
#define CONST_2POW_N36 1.4551915e-11F
#define CONST_2POW_N44 5.6843419e-14F
#define CONST_2POW_N46 1.4210855e-14F
//---------------------------------------------------------------------------------------------//


typedef enum 
{
    eHALT_MODE = 0U,
    eSLEEPING_STEP_MODE,
    eSTEP_MODE,
    eCONTINIOUS_MODE
} t_SensorsMode;

typedef enum 
{
    eMEDICAL = 0x00U,
    eEXTENDED = 0x11U
} t_SensorsMeasSelect;


typedef union __attribute__((packed)) Senor32Data
{
  int32_t       iLongWord;
  uint16_t      uiWordArray[2U];
  uint8_t       uiByteArray[4U];
} t_Senor32Data;

typedef union __attribute__((packed)) Senor16Data
{
  int16_t       iWordArray;
  uint8_t       uiByteArray[2U];
} t_Senor16Data;


typedef struct __attribute__((packed)) RegContorl
{
  uint16_t uiReserved1           :1;
  uint16_t uiMode                :2;
  uint16_t uiSoc                 :1;
  uint16_t uiMeas_Select         :5;
  uint16_t uiReserved2           :2;  
  uint16_t uiSob                 :1;
  uint16_t uiReserved3           :4;    
} t_RegContorl;

typedef union __attribute__((packed)) RegCtrUnified
{
  t_RegContorl  RegContorl;
  int16_t       iWordArray;
} t_RegCtrUnion;


typedef struct __attribute__((packed)) 
{
  uint16_t uiNewData              :1;
  uint16_t uiReserved1            :1;
  uint16_t uiCycle_Pos            :5;
  uint16_t uiReserved2            :1;
  uint16_t uiBrownOut             :1;  
  uint16_t uiEEprom_busy          :1;
  uint16_t uiDevice_busy          :1;
  uint16_t uiReserved3            :5;
} t_RegStatus;

typedef union __attribute__((packed))
{
  uint16_t  uiRegStatus;
  t_RegStatus RegStatus;   
} t_RegStatusUnified;



typedef struct __attribute__((packed)) RefreshRate
{
  uint16_t uiReserved1            :8;
  uint16_t uiRefresh_Rate         :3;
  uint16_t uiReserved2            :5;
} t_RefreshRate;


typedef struct Sensor
{
  t_RegCtrUnion  RegCtr;
  t_RegStatusUnified   RegStatusUnified;
  t_RefreshRate Meas_1;
  t_RefreshRate Meas_2;
  
  uint16_t       I2C_Adress;
  
  I2C_HandleTypeDef* Channel;
  
  float     fEE_P_R;     
  float     fEE_P_G;     
  float     fEE_P_T;     
  float     fEE_P_O;     
  float     fEE_Aa;      
  float     fEE_Ab;      
  float     fEE_Ba;      
  float     fEE_Bb;      
  float     fEE_Ca;      
  float     fEE_Cb;      
  float     fEE_Da;      
  float     fEE_Db;     
  float     fEE_Ea;      
  float     fEE_Eb;      
  float     fEE_Fa;      
  float     fEE_Fb;      
  float     fEE_Ga;      
  float     fEE_Gb;      
  float     fEE_Ka;
  float     fEE_Kb;
  float     fEE_Ha;      
  float     fEE_Hb;
  
  float fObject_Temp;
  float fAmbient_Temp;

} t_Sensor;



void Senors_I2C_Init();

void Calculate_Temps(uint8_t uiSensorIndex);

void IsDataReady(uint8_t uiSensorIndex);
