//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   Space Elevator
//Version number:  Ver.1.0
//Date:            2020.03.06
//------------------------------------------------------------------//

//This program supports the following boards:
//* M5Stack(Grey version)

#define M5STACK_MPU6886 

//Include
//------------------------------------------------------------------//
#include <M5Stack.h>
#include <Servo.h>
#include "driver/pcnt.h"


//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 1                    // Timer Interrupt Period

#define ESC_LDEC_CHANNEL 3                  // 50Hz LDEC Timer

#define PULSE_INPUT_PIN 35                  // Rotaly Encoder Phase A
#define PULSE_CTRL_PIN  36                  // Rotaly Encoder Phase B
#define PCNT_H_LIM_VAL  10000               // Counter Limit H
#define PCNT_L_LIM_VAL -10000               // Counter Limit L 

//Global
//------------------------------------------------------------------//

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer10;
int iTimer50;
int bufferIndex;
int writeBank;
int bufferRecords;
int RecordType;
int rp;
int buffer;

// Encoder1
int16_t delta_count = 0;                    // Delta Counter
long    total_count = 0;                    // Total Counter

int16_t delta_count_buff;
long    total_count_buff;

// ESC
static const int escPin = 26;
Servo esc;
int power = 0;
int power_buff;

// Main
unsigned char pattern = 0;
unsigned char pattern_buff;
unsigned int  time_buff;
unsigned int  micros_time;


char motor_output;
float climber_altitude;
float climber_velocity;
float slip_rate=4.3;
float battery_voltage=26.0;

// MPU
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float temp = 0.0F;

float pitch_buff;
float roll_buff;
float yaw_buff;


//Prototype
//------------------------------------------------------------------//
uint8_t getBatteryGauge(void);
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);
void initEncoder(void);
void buttonAction(void);
void lcdDisplay(void);
void velocityControl(int object_count);

//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin();
  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 

  initEncoder();

  M5.Lcd.setTextSize(2);

  // Initialize IIC
  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU
  M5.IMU.Init();
  Serial.begin(115200);

  esc.attach(escPin, ESC_LDEC_CHANNEL, 0, 100, 1100, 1940);
  esc.write(0);

}

//Main
//------------------------------------------------------------------//
void loop() {

  timerInterrupt();

  switch (pattern) {
  case 0:    
    esc.write(0);
    //lcdDisplay();
    buttonAction();
    break;  

  case 11:
    lcdDisplay();
    esc.write(power);
    if( total_count >= 300000 || total_count <= -300000 ) {
      power = 0;
      total_count = 0; 
      time_buff = millis();   
      pattern = 13;
      break;
    } 
    if( power >= 100 ) {
      pattern = 12;
      break;
    }    
    break;

  case 12:
    lcdDisplay();  
    esc.write(power);
    if( total_count >= 300000 || total_count <= -300000 ) {
      power = 0;
      total_count = 0; 
      time_buff = millis();   
      pattern = 14;
      break;
    }
    break;

  case 13:
    lcdDisplay();
    power = 0;
    esc.write(power);
    if( millis() - time_buff >= 5000 ) {
      pattern = 0;
      break;
    }
    break;

case 14:
    lcdDisplay();
    power = 0;
    esc.write(power);                          
    if( millis() - time_buff >= 5000 ) {
      pattern = 0;
      break;
    }
    break;




  }
   
}

// Timer Interrupt
//------------------------------------------------------------------//
void timerInterrupt(void) {
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    
    pcnt_get_counter_value(PCNT_UNIT_0, &delta_count);
    pcnt_counter_clear(PCNT_UNIT_0);  
    total_count += delta_count;

    //Serial.printf("%3ld\n",millis());
    //10ms timerInterrupt
    switch(iTimer10){
    case 1:
      break;
    case 2:
      M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
      break;
    case 3:
       M5.IMU.getAccelData(&accX,&accY,&accZ);
      break;
    case 4:
      M5.IMU.getTempData(&temp);
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:
     iTimer10 = 0;
      break;
    }

    iTimer50++;
    //50ms timerInterrupt
    switch (iTimer50) {
    case 10:
      if(pattern == 11 && (power < 100)) power++;
      break;
    case 20:
     Serial.printf("%3ld,",millis()/1000);
     Serial.printf("%3d,",pattern);
     Serial.printf("%3d,",motor_output);
     Serial.printf("%3.1f,",climber_altitude);
     Serial.printf("%2.2f,",climber_velocity);
     Serial.printf("%2.2f,",slip_rate);
     Serial.printf("%2.2f,",battery_voltage);
     Serial.printf("%5d,",micros_time);
     Serial.printf("");
     Serial.printf("\n");
      break;
    case 30:
     break;
    case 50:
      iTimer50 = 0;
      break;
    }
  }
}

//------------------------------------------------------------------//

/*void velocityControl(int object_count) {
  int i, j;
  err = delta_count - object_count;
  i = err * kp;
  j = (err_buff - err)*kd;
  power = (i - j)/3;
  
  err_buff = err;
  if(power>100) power = 100;
  if(power<-100) power = -100;
}*/


// Initialize Encoder
//------------------------------------------------------------------//
void initEncoder(void) {
  pcnt_config_t pcnt_config_1A;
    pcnt_config_1A.pulse_gpio_num = PULSE_INPUT_PIN;
    pcnt_config_1A.ctrl_gpio_num = PULSE_CTRL_PIN;
    pcnt_config_1A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_1A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_1A.channel = PCNT_CHANNEL_0;
    pcnt_config_1A.unit = PCNT_UNIT_0;
    pcnt_config_1A.pos_mode = PCNT_COUNT_INC;
    pcnt_config_1A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_1A.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_1A.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_1B;
    pcnt_config_1B.pulse_gpio_num = PULSE_CTRL_PIN;
    pcnt_config_1B.ctrl_gpio_num = PULSE_INPUT_PIN;
    pcnt_config_1B.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_1B.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_1B.channel = PCNT_CHANNEL_1;
    pcnt_config_1B.unit = PCNT_UNIT_0;
    pcnt_config_1B.pos_mode = PCNT_COUNT_INC;
    pcnt_config_1B.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_1B.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_1B.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_unit_config(&pcnt_config_1A);            // Initialize Unit 1A
  pcnt_unit_config(&pcnt_config_1B);            // Initialize Unit 1B
  pcnt_counter_pause(PCNT_UNIT_0);              // Stop Counter
  pcnt_counter_clear(PCNT_UNIT_0);              // clear Counter
  pcnt_counter_resume(PCNT_UNIT_0);             // Start Count
}

// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {

  // Clear Display
 //M5.Lcd.setTextColor(BLACK);
 //M5.Lcd.setCursor(10, 10);
 //M5.Lcd.printf("Pattern: %3d", pattern_buff);  
 //M5.Lcd.setCursor(10, 40);
 //M5.Lcd.printf("Delta Counter: %6d", delta_count_buff);  
 //M5.Lcd.setCursor(10, 70);
 //M5.Lcd.printf("Total Counter: %3ld", total_count_buff); 
 //M5.Lcd.setCursor(10, 100);
 //M5.Lcd.printf("Motor Power: %3d", power_buff); 
 //M5.Lcd.setCursor(10, 130);
 //M5.Lcd.printf("PITCH: %3f", pitch_buff);
 //M5.Lcd.setCursor(10, 160);
 //M5.Lcd.printf("ROLL: %3f", roll_buff);
 //M5.Lcd.setCursor(10, 190);
  /*M5.Lcd.printf("ERR: %3d", err_buff2);*/


  // Refresh Display
  M5.Lcd.setTextColor(CYAN,BLACK);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("Pattern: %3d", pattern);  
  M5.Lcd.setCursor(10, 40);
  M5.Lcd.printf("Counter value: %6d", delta_count);
  M5.Lcd.setCursor(10, 70);
  M5.Lcd.printf("Total Counter: %3ld", total_count); 
  M5.Lcd.setCursor(10, 100);
  M5.Lcd.printf("Motor Power: %3d", power); 
  //M5.Lcd.setCursor(10, 130);
  //M5.Lcd.printf("PITCH: %3f", pitch);
  //M5.Lcd.setCursor(10, 160);
  //M5.Lcd.printf("ROLL: %3f", roll);
  //M5.Lcd.setCursor(10, 190);
  //M5.Lcd.printf("ERR: %3d", err);



}

// Button Action
//------------------------------------------------------------------//
void buttonAction(void){
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if( pattern == 0 ) {
      pattern = 11;
    }
  } else if (M5.BtnB.wasPressed()) {
  } else if (M5.BtnC.wasPressed()) {
  }
}

// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Battery Gauge
//------------------------------------------------------------------//
uint8_t getBatteryGauge() {
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  Wire.endTransmission(false);
  if(Wire.requestFrom(0x75, 1)) {
    return Wire.read();
  }
  return 0xff;
}