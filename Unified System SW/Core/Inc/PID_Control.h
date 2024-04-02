

//---------------PID parameters------------//

#define K_D 1.0F
#define K_P 1.0F
#define K_I 1.0F
#define K_PWM 65536.0F/38.0F

//-----------------------------------------//

#define MIN_DC 100U

#define IDEAL_TEMP 38.0F

#define PID_PERIOD 50000U  // 50,000 is 0.5 second.


void Calculate_PID (uint8_t uiSensorIndex);