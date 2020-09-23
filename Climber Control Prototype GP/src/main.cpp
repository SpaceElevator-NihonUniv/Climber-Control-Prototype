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
#define TIMER_INTERRUPT 1                   // Timer Interrupt Period

#define ESC_LDEC_CHANNEL 3                  // 50Hz LDEC Timer

#define PULSE_INPUT_PIN_1 25                // Rotaly Encoder Phase A
#define PULSE_CTRL_PIN_1  26                // REncoder Phase B
#define PULSE_INPUT_PIN_2 13                // Rotaly Encoder Phase A
#define PULSE_CTRL_PIN_2  0                 // Rotaly Encoder Phase B
#define PCNT_H_LIM_VAL  10000               // Counter Limit H
#define PCNT_L_LIM_VAL -10000               // Counter Limit L 

#define BRAKE_THRESHOLD_VELOCITY 1          // 1m/s

#define DRIVER_ROLLER_PERIMETER 283         //mm
#define IDLER_ROLLER_PERIMETER 94.25        //mm

#define DRIVER_ROLLER_PPR 2000
#define IDLER_ROLLER_PPR 400

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
int16_t delta_count_1 = 0;                    // Delta Counter
long    total_count_1 = 0;                    // Total Counter
float   velocity_1;
float   travel_1;

int16_t delta_count_1_buff;
long    total_count_1_buff;


int16_t delta_count_2 = 0;                    // Delta Counter
long    total_count_2 = 0;                    // Total Counter
float   velocity_2;
float   travel_2;

int16_t delta_count_2_buff;
long    total_count_2_buff;
unsigned int braking_distance;

// ESC
static const int escPin = 2;
Servo esc;
int power = 0;
int power_buff;

// Main
unsigned char pattern = 0;
unsigned char pattern_buff;
unsigned int  time_buff;
unsigned int  micros_time;
bool lcd_flag = false;
bool sd_insert = false;
int lcd_pattern = 10;
uint16_t lcd_back = 0;

char motor_output;
float climber_altitude;
float climber_velocity;
float slip_rate=4.3;
float battery_voltage=26.0;

//Timer Interrupt
char  xbee_re_buffer[16];
unsigned  int xbee_index;

unsigned char se_pattern;
unsigned char re_pattern;
float re_val;
int sleep_flag;





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

// Paramenters
unsigned int target_travel = 15;
unsigned int target_velocity = 9;



//Prototype
//------------------------------------------------------------------//
uint8_t getBatteryGauge(void);
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);
void initEncoder(void);
void buttonAction(void);
void lcdDisplay(void);
void velocityControl(int object_count);
void xbee_re(void);
void xbee_se(void);

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

  sd_insert = SD.begin(TFCARD_CS_PIN, SPI,  24000000);
  M5.Lcd.drawJpgFile(SD, "/icon/icons8-sd.jpg", 280,  0);
  M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-100.jpg", 240,  0);
  M5.Lcd.drawJpgFile(SD, "/icon/icons8-wi-fi-1.jpg", 200,  0);
  M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-100.jpg", 160,  0);

  


}

//Main
//------------------------------------------------------------------//
void loop() {

  timerInterrupt();

  switch (pattern) {
  case 0: 
    power = 0;   
    esc.write( power);
    lcdDisplay();
    buttonAction();
    break;  

  case 11:
    esc.write(power);
    if( velocity_1 >= target_velocity || velocity_2 >= target_velocity ) {
      time_buff = millis();   
      pattern = 21;
      braking_distance = (target_velocity / 9.8)*target_velocity/2;
      break;
    }   
    break;

  case 21: 
    esc.write(power);
<<<<<<< HEAD
    if(  travel_1 >= target_travel - braking_distance || travel_2 >= target_travel - braking_distance ) {
=======
    if( travel_1 >= target_travel || travel_2 >= target_travel ) {
>>>>>>> 3970e077e1d244a05a5bc03e9abd73443caf6e22
      time_buff = millis();   
      pattern = 31;
      break;
    }
    break;

  case 31:
    esc.write(power);
    if( power <= 0 ) {
      pattern = 32;
      break;
    }
    break;

  case 32:
    power = 0;   
    esc.write(power);                          
    if( velocity_1 < BRAKE_THRESHOLD_VELOCITY || velocity_2 < BRAKE_THRESHOLD_VELOCITY ) {
      pattern = 41;
      break;
    }
    break;

  case 41:
    pattern = 0;
    break;
  




  }
  //xbee_re();
  //xbee_se();
   
}

// Timer Interrupt
//------------------------------------------------------------------//
void timerInterrupt(void) {
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    
    pcnt_get_counter_value(PCNT_UNIT_0, &delta_count_1);
    pcnt_counter_clear(PCNT_UNIT_0);  
    total_count_1 += delta_count_1;
    velocity_1 = delta_count_1 * DRIVER_ROLLER_PERIMETER / DRIVER_ROLLER_PPR / 1000;
    travel_1 = total_count_1 * DRIVER_ROLLER_PERIMETER / DRIVER_ROLLER_PPR / 1000;

    pcnt_get_counter_value(PCNT_UNIT_1, &delta_count_2);
    pcnt_counter_clear(PCNT_UNIT_1);  
    total_count_2 += delta_count_2;
    velocity_2 = delta_count_2 * IDLER_ROLLER_PERIMETER / IDLER_ROLLER_PPR / 1000;
    travel_2 = total_count_2 * IDLER_ROLLER_PERIMETER / IDLER_ROLLER_PPR / 1000;

    lcd_back += 1;
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("%d\n", lcd_back); 
    M5.Lcd.printf("%d\n", millis() - lcd_back); 

    //Serial.printf("%3ld\n",millis());
    //10ms timerInterrupt
    switch(iTimer10){
    case 1:
      break;
    case 2:
      //M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
      break;
    case 3:
      //M5.IMU.getAccelData(&accX,&accY,&accZ);
      break;
    case 4:
      //M5.IMU.getTempData(&temp);
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
      if(se_pattern ==  101 ){
        Serial.printf("%3.2f, ",(float)millis()/1000);
        Serial.printf("%3d, "  ,pattern);
        Serial.printf("%3d, "  ,motor_output);
        Serial.printf("%3.1f, ",climber_altitude);
        Serial.printf("%2.2f, ",climber_velocity);
        Serial.printf("%2.2f, ",slip_rate);
        Serial.printf("%2.2f, ",battery_voltage);
        Serial.printf("%5d, "  ,micros_time);
        Serial.printf("%5f, "  ,pitch);
        Serial.printf("\n");
      }
      break;
    case 30:
     break;
    case 50:
      
      //M5.Lcd.setCursor(0, 0);
      //M5.Lcd.printf("%d", lcd_back); 
      if( lcd_pattern > 10 && lcd_pattern < 20 && lcd_back > 2000 ) {
        M5.lcd.clear();
        lcd_pattern = 10;
      }
      else if( lcd_pattern > 110 && lcd_pattern < 120 && lcd_back > 2000){
        M5.lcd.clear();
        lcd_pattern = 110;
      }
      lcd_flag = true;
      iTimer50 = 0;
      break;
    }
  }
}

//XBee Re
//------------------------------------------------------------------//
void xbee_re(void){

  while(Serial.available()){
    xbee_re_buffer[xbee_index]=Serial.read();
    Serial.write(xbee_re_buffer[xbee_index]);

    if(xbee_re_buffer[xbee_index]==0x08){
      xbee_re_buffer[xbee_index-1]==NULL;
      xbee_index--;
      Serial.printf(" ");
      Serial.write(0x08);
    }else if(xbee_re_buffer[xbee_index]==0x0D){
      Serial.read();
      Serial.printf("\n\n");
      if(se_pattern == 0){
        re_pattern = atoi(xbee_re_buffer);
      }/*else if(se_pattern == 2){
        re_val = atof(xbee_re_buffer);
      }*/
      xbee_index = 0;

      switch(re_pattern){
      case 0:
        se_pattern = 1;
        break;

      case 11:
        se_pattern = 11;
        re_pattern = 21;
        break;
      case 21:
        if(re_val == 1){
          pattern = 101;
        } else {
          re_pattern = 1;
        }
        re_pattern = 0;
        break;

      case 31:
        se_pattern = 31;
        re_pattern = 41;
        break;
      case 41:
        motor_output = re_val;
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 32:
        se_pattern = 32;
        re_pattern = 42;
        break;
      case 42:
        climber_altitude = re_val;
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 33:
        se_pattern = 33;
        re_pattern = 43;
        break;
      case 43:
        climber_velocity = re_val;
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 34:
        se_pattern = 34;
        re_pattern = 44;
        break;
      case 44:
        slip_rate = re_val;
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 35:
        se_pattern = 35;
        re_pattern = 45;
        break;
      case 45:
        battery_voltage = re_val;
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 36:
        se_pattern = 36;
        re_pattern = 46;
        break;
      case 46:
        micros_time = re_val;
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 37:
        se_pattern = 37;
        re_pattern = 47;
        break;
      case 47:
        pitch = re_val;
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 201:
        if(re_val == 1){
          pattern = 201;
        }else{
          se_pattern = 1;
        }
        re_pattern = 0;
        break;

      case 211:
        if(re_val == 1){
          pattern = 211;
        }else{
          se_pattern = 1;
        }
        re_pattern = 0;
        break;
      }
      
      } else if( xbee_re_buffer[xbee_index] ==  'T' ||  xbee_re_buffer[xbee_index] == 't'){
        re_pattern  = 0;
        se_pattern  = 101;
        Serial.printf("\n");
      } else if( xbee_re_buffer[xbee_index] ==  'U' ||  xbee_re_buffer[xbee_index] == 'u'){
        re_pattern  = 201;
        se_pattern  = 201;
        Serial.printf("\n");
      } else if( xbee_re_buffer[xbee_index] ==  'D' ||  xbee_re_buffer[xbee_index] == 'd'){
        re_pattern  = 211;
        se_pattern  = 211;
        Serial.printf("\n");
      } else if( xbee_re_buffer[xbee_index] ==  ' '){
  
        lcd_pattern = 0;
        pattern = 2;
        Serial.printf("\n\n");
        Serial.printf(" Emargency Stop Enable \n ");
        Serial.printf(" return Case 0 \n ");
        re_pattern = 0;
        se_pattern = 1;
        Serial.printf("\n");
      }else{
          xbee_index++;
      }
    
    }
}

//XBee_SE
//------------------------------------------------------------------//
void xbee_se(void){

  switch (se_pattern){
    //Waiting Command
    case 0:
      break;

    case 1:
    Serial.printf("\n\n\n\n\n\n");
    Serial.printf("climber Controller (M5Stack version) "
                    "Test program Ver1.20\n");
    Serial.printf("\n");
    Serial.printf(" 11 : Start Seqwnse\n");
    Serial.printf("\n");
    Serial.printf(" 31 :  Motor Output     [%3d]\n " ,motor_output);
    Serial.printf(" 32 :  Climber Altitude [%3.1f]\n ",climber_altitude);
    Serial.printf(" 33 :  Climber Velocity [%2.2f]\n ",climber_velocity);
    Serial.printf(" 34 :  Slip  Rate       [%2.2f]\n ",slip_rate);
    Serial.printf(" 35 :  Battery Voltage  [%2.2f]\n ",battery_voltage);
    Serial.printf(" 36 :  Micros  Time     [%5d]\n"  ,micros_time);
    Serial.printf(" 37 :  Pitch            [%5f]\n "  ,pitch);
    Serial.printf("\n");
    Serial.printf(" T : Terementry\n");
    Serial.printf(" U : Manual  Climb\n");
    Serial.printf(" D : Manual  Desvent\n");

    Serial.printf("\n");
    Serial.printf(" Please  enter 11 35 ");

    se_pattern = 0;
    break;

  //Waiting value
  case 2 :
    break;

  case 11:
    Serial.printf("\n Check Current Paramenters\n");
    Serial.printf("\n");
    Serial.printf(" Motor Output     [%3d]\n   " ,motor_output);
    Serial.printf(" Climber Altitude [%3.1f]\n ",climber_altitude);
    Serial.printf(" Climber Velocity [%2.2f]\n ",climber_velocity);
    Serial.printf(" Slip  Rate       [%2.2f]\n ",slip_rate);
    Serial.printf(" Battery Voltage  [%2.2f]\n ",battery_voltage);
    Serial.printf(" Micros  Time     [%5d]\n   "  ,micros_time);
    Serial.printf(" Pitch            [%5f]\n   "  ,pitch);
    Serial.printf("\n");
    Serial.printf(" Confilm to Climb? ->  ");
    se_pattern  = 2;
    break;

  case 31:
     Serial.printf(" Motor Output [%3d]\n " ,motor_output);
     Serial.printf(" Please enter 0 to 4000 -> ");
     se_pattern = 2;
     break;

  case 32:
      Serial.printf(" Climber Altitude [%3.1f]\n ",climber_altitude);
      Serial.printf(" Please enter 0.0 to 1000 -> ");
      se_pattern = 2;
      break;

  case 33:
      Serial.printf(" Climber Velocity [%2.2f]\n ",climber_velocity);
      Serial.printf(" Please enter 0.00 to 50 -> ");
      se_pattern = 2;
      break;
    
  case 34:
      Serial.printf(" Slip  Rate [%2.2f]\n ",slip_rate);
      Serial.printf(" Please enter 0.00 to 50 -> ");
      se_pattern = 2;
      break;

  case 35:
      Serial.printf(" Battery Voltage [%2.2f]\n ",battery_voltage);
      Serial.printf(" Please enter 0.00 to 50 -> ");
      se_pattern = 2;
      break;

  //Telementry Mode
  case 101:
    break;

  //Manual Control Mode
  case 201:
    Serial.printf("\n Check Current Paramenters\n");
    Serial.printf("\n");
    Serial.printf(" Motor Output     [%3d]\n " ,motor_output);
    Serial.printf(" Climber Altitude [%3.1f]\n ",climber_altitude);
    Serial.printf(" Climber Velocity [%2.2f]\n ",climber_velocity);
    Serial.printf(" Slip  Rate       [%2.2f]\n ",slip_rate);
    Serial.printf(" Battery Voltage  [%2.2f]\n ",battery_voltage);
    Serial.printf(" Micros  Time     [%5d]\n"  ,micros_time);
    Serial.printf(" Pitch            [%5f]\n "  ,pitch);
    Serial.printf("\n");
    Serial.printf(" Confilm to Climb? ->  ");
    se_pattern  = 2;
    break;

  case 211:
    Serial.printf("\n Check Current Paramenters\n");
    Serial.printf("\n");
    Serial.printf(" Motor Output     [%3d]\n " ,motor_output);
    Serial.printf(" Climber Altitude [%3.1f]\n ",climber_altitude);
    Serial.printf(" Climber Velocity [%2.2f]\n ",climber_velocity);
    Serial.printf(" Slip  Rate       [%2.2f]\n ",slip_rate);
    Serial.printf(" Battery Voltage  [%2.2f]\n ",battery_voltage);
    Serial.printf(" Micros  Time     [%5d]\n"  ,micros_time);
    Serial.printf(" Pitch            [%5f]\n "  ,pitch);
    Serial.printf("\n");
    Serial.printf(" Confilm to Climb? ->  ");
    se_pattern  = 2;
    break;
}
  {
  //case /* constant-expression */:
    /* code */
    //break;
  
  //default:
   // break;
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
    pcnt_config_1A.pulse_gpio_num = PULSE_INPUT_PIN_1;
    pcnt_config_1A.ctrl_gpio_num = PULSE_CTRL_PIN_1;
    pcnt_config_1A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_1A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_1A.channel = PCNT_CHANNEL_0;
    pcnt_config_1A.unit = PCNT_UNIT_0;
    pcnt_config_1A.pos_mode = PCNT_COUNT_INC;
    pcnt_config_1A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_1A.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_1A.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_1B;
    pcnt_config_1B.pulse_gpio_num = PULSE_CTRL_PIN_1;
    pcnt_config_1B.ctrl_gpio_num = PULSE_INPUT_PIN_1;
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

  pcnt_config_t pcnt_config_2A;
    pcnt_config_2A.pulse_gpio_num = PULSE_INPUT_PIN_2;
    pcnt_config_2A.ctrl_gpio_num = PULSE_CTRL_PIN_2;
    pcnt_config_2A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_2A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_2A.channel = PCNT_CHANNEL_0;
    pcnt_config_2A.unit = PCNT_UNIT_1;
    pcnt_config_2A.pos_mode = PCNT_COUNT_INC;
    pcnt_config_2A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_2A.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_2A.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_config_t pcnt_config_2B;
    pcnt_config_2B.pulse_gpio_num = PULSE_CTRL_PIN_2;
    pcnt_config_2B.ctrl_gpio_num = PULSE_INPUT_PIN_2;
    pcnt_config_2B.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_2B.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_2B.channel = PCNT_CHANNEL_1;
    pcnt_config_2B.unit = PCNT_UNIT_1;
    pcnt_config_2B.pos_mode = PCNT_COUNT_INC;
    pcnt_config_2B.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_2B.counter_h_lim = PCNT_H_LIM_VAL;
    pcnt_config_2B.counter_l_lim = PCNT_L_LIM_VAL;

  pcnt_unit_config(&pcnt_config_2A);            // Initialize Unit 1A
  pcnt_unit_config(&pcnt_config_2B);            // Initialize Unit 1B
  pcnt_counter_pause(PCNT_UNIT_1);              // Stop Counter
  pcnt_counter_clear(PCNT_UNIT_1);              // clear Counter
  pcnt_counter_resume(PCNT_UNIT_1);             // Start Count
}

// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {
  if(lcd_flag) {
    switch (lcd_pattern){
    //Waiting Command
  
    case 10:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawJpgFile(SD, "/icon/status.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/control.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Params");  

     //M5.Lcd.drawJpgFile(SD, "/icon/icons8-sd.jpg", 280,  0);
     //M5.Lcd.drawJpgFile(battery-revel, "/icon/icons8-battery-revel-0.jpg", 250,  0);
     //M5.Lcd.drawJpgFile(battery-revel, "/icon/icons8-battery-revel-25.jpg", 250,  0);
     //M5.Lcd.drawJpgFile(battery-revel, "/icon/icons8-battery-revel-50.jpg", 250,  0);
     //M5.Lcd.drawJpgFile(battery-revel, "/icon/icons8-battery-revel-75.jpg", 250,  0);
     //M5.Lcd.drawJpgFile(battery-revel, "/icon/icons8-battery-revel-100.jpg", 250,  0);
     //M5.Lcd.drawJpgFile(wi-fi, "/icon/icons8-sd.jpg", 280,  0);
      break;

    case 11:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, ORANGE);
      M5.Lcd.fillRect(0, 29, 106, 2, ORANGE);
      M5.Lcd.fillRect(0, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(105, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(0, 238, 106, 2, ORANGE);
      M5.Lcd.drawJpgFile(SD, "/icon/status.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/control.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Params");
      break;

    case 12:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, ORANGE);
      M5.Lcd.fillRect(105, 29, 108, 2, ORANGE);
      M5.Lcd.fillRect(105, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(213, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(105, 238, 108, 2, ORANGE);
      M5.Lcd.drawJpgFile(SD, "/icon/status.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/control.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Params");
      break;

    case 13:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, ORANGE);
      M5.Lcd.fillRect(213, 29, 106, 2, ORANGE);
      M5.Lcd.fillRect(213, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(318, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(213, 238, 106, 2, ORANGE);
      M5.Lcd.drawJpgFile(SD, "/icon/status.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/control.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Params");
      break;

//Status    
    case 110:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawJpgFile(SD, "/icon/brake.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/sensor.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Sensor");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Codec");
      break;

    case 111:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, ORANGE);
      M5.Lcd.fillRect(0, 29, 106, 2, ORANGE);
      M5.Lcd.fillRect(0, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(105, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(0, 238, 106, 2, ORANGE);
      M5.Lcd.drawJpgFile(SD, "/icon/brake.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/sensor.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Sensor");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Codec");
      break;

    case 112:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, ORANGE);
      M5.Lcd.fillRect(105, 29, 108, 2, ORANGE);
      M5.Lcd.fillRect(105, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(213, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(105, 238, 108, 2, ORANGE);
      M5.Lcd.drawJpgFile(SD, "/icon/brake.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/sensor.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Sensor");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Codec");
      break;

    case 113:
      M5.Lcd.drawFastVLine(106, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastVLine(214, 30, 210, TFT_WHITE);
      M5.Lcd.drawFastHLine(0, 30, 320, TFT_WHITE);
      M5.Lcd.drawRect(0, 30, 106, 210, TFT_WHITE);
      M5.Lcd.drawRect(106, 30, 108, 210, TFT_WHITE);
      M5.Lcd.drawRect(214, 30, 106, 210, ORANGE);
      M5.Lcd.fillRect(213, 29, 106, 2, ORANGE);
      M5.Lcd.fillRect(213, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(318, 29, 2, 210, ORANGE);
      M5.Lcd.fillRect(213, 238, 106, 2, ORANGE);
      M5.Lcd.drawJpgFile(SD, "/icon/brake.jpg", 14, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/sensor.jpg", 120, 78);
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 238, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Sensor");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Codec");
      break;

//Status_Brake
    case 1110:
      M5.Lcd.drawFastVLine(160, 30, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawFastHLine(0, 30, 320, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(0, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(160, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(33, 120);
      M5.Lcd.printf("Temper");  
      M5.Lcd.setCursor(216, 120);
      M5.Lcd.printf("Incli");
      break;

    

//Status_Sensor
    case 1120:
      M5.Lcd.drawFastVLine(160, 30, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawFastHLine(0, 30, 320, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(0, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(160, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(45, 120);
      M5.Lcd.printf("Touch"); 
      M5.Lcd.setCursor(194, 120);
      M5.Lcd.printf("Marker");
      break;

//Status_Codec
    case 1130:
      M5.Lcd.drawFastVLine(160, 30, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawFastHLine(0, 30, 320, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(0, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(160, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(20, 120);
      M5.Lcd.printf("%d", total_count_1);  
      M5.Lcd.setCursor(195, 120);
      M5.Lcd.printf("%d", total_count_2);
      break;
  

//Control
    case 120:
      break;


    
//Params
    case 130:
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(20, 90);
      M5.Lcd.printf("altitude: %3.1f", climber_altitude);
      M5.Lcd.setCursor(20, 140);
      M5.Lcd.printf("velocity: %2.2f", climber_velocity); 
      break;



   // case 21:
      // Refresh Display
      //M5.Lcd.setTextColor(CYAN,BLACK);
      //M5.Lcd.setCursor(10, 10);
      //M5.Lcd.printf("Pattern: %3d", pattern);  
      //M5.Lcd.setCursor(10, 40);
      //M5.Lcd.printf("Counter value: %6d", delta_count);
      //M5.Lcd.setCursor(10, 70);
      //M5.Lcd.printf("Total Counter: %3ld", total_count); 
      //M5.Lcd.setCursor(10, 100);
      //M5.Lcd.printf("Motor Power: %3d", power); 
      //M5.Lcd.setCursor(10, 130);
      //M5.Lcd.printf("PITCH: %3f", pitch);
      //M5.Lcd.setCursor(10, 160);
      //M5.Lcd.printf("ROLL: %3f", roll);
      //break;
    }
    lcd_flag = false;
  }
}

// Button Action
//------------------------------------------------------------------//
void buttonAction(void){
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if( pattern == 0 ) {
      M5.lcd.clear();
      if( lcd_pattern >= 110 && lcd_pattern < 1000 ){
        lcd_pattern = 10;
      }
      if( lcd_pattern >= 1110 && lcd_pattern < 2000 ){
        lcd_pattern = 110;
      }
        
    }
  } else if (M5.BtnB.wasPressed()) {
    M5.lcd.clear();
    switch (lcd_pattern){
      case 11:
        lcd_pattern = 110;
        break;

      case 111:
        lcd_pattern = 1110;
        break;

      case 112:
        lcd_pattern = 1120;
        break;

      case 113:
        lcd_pattern = 1130;
        break;

      case 13:
        lcd_pattern = 130;
        break;

    }
  } else if (M5.BtnC.wasPressed()) {
    M5.lcd.clear();
    if( lcd_pattern >= 10  && lcd_pattern < 20 ){
      lcd_back = 0;
      lcd_pattern++;
      if( lcd_pattern > 13 ) lcd_pattern = 11; 
    }
    if( lcd_pattern >= 110  &&  lcd_pattern < 120){
      lcd_back = 0;
      lcd_pattern++;
      if( lcd_pattern > 113 ) lcd_pattern = 111;
    }
    if( lcd_pattern >= 1110  &&  lcd_pattern < 1120){
      lcd_back = 0;
      if( lcd_pattern > 1112 ) lcd_pattern = 1111;
    }


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