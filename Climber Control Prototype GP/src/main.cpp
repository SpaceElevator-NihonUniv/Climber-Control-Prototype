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
#include <CircularBuffer.h>
#include <Servo.h>
#include "driver/pcnt.h"
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoOTA.h>


//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 1                   // Timer Interrupt Period

#define ESC_LDEC_CHANNEL 3                  // 50Hz LDEC Timer

#define PULSE_INPUT_PIN_1 26                // Rotaly Encoder Phase A
#define PULSE_CTRL_PIN_1  25                // REncoder Phase B
#define PULSE_INPUT_PIN_2 0                // Rotaly Encoder Phase A
#define PULSE_CTRL_PIN_2  13                 // Rotaly Encoder Phase B
#define PCNT_H_LIM_VAL  10000               // Counter Limit H
#define PCNT_L_LIM_VAL -10000               // Counter Limit L 

#define BRAKE_THRESHOLD_VELOCITY 3.6          // 1km/h

#define DRIVER_ROLLER_PERIMETER 283.00         //mm
#define IDLER_ROLLER_PERIMETER 94.25        //mm

#define DRIVER_ROLLER_COEFFICIENT 1.0633
#define IDLER_ROLLER_COEFFICIENT 0.9788

#define DRIVER_ROLLER_PPR 2000
#define IDLER_ROLLER_PPR 400

#define AVERATING_BUFFER_SIZE 10            // Velocity Averating Buffer Size

//Global
//------------------------------------------------------------------//
CircularBuffer<int, AVERATING_BUFFER_SIZE> delta1_buffer;
CircularBuffer<int, AVERATING_BUFFER_SIZE> delta2_buffer;
CircularBuffer<int, 20> encoder1_status_buffer;
CircularBuffer<int, 20> encoder2_status_buffer;
CircularBuffer<int, 10> touch_up_buffer;
CircularBuffer<int, 10> touch_down_buffer;

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
uint16_t touch_up_status_counter;
uint16_t touch_down_status_counter;

// Encoder1
int16_t delta_count_1 = 0;                    // Delta Counter
long    total_count_1 = 0;                    // Total Counter
float   velocity_1;
float   travel_1;
int     delta_avg_1 = 0;

int16_t delta_count_1_buff;
long    total_count_1_buff;


int16_t delta_count_2 = 0;                    // Delta Counter
long    total_count_2 = 0;                    // Total Counter
float   velocity_2;
float   travel_2;
int     delta_avg_2 = 0;

int16_t delta_count_2_buff;
long    total_count_2_buff;
unsigned int braking_distance;

float travel_1_buff;
float travel_2_buff;
uint16_t encoder1_status_counter;
uint16_t encoder2_status_counter;

// ESC
static const int escPin = 2;
Servo esc;
float power = 0;
float power_increment = 1;

// Main
uint16_t pattern = 0;
unsigned char pattern_buff;
unsigned int  time_buff;
unsigned int  micros_time;
bool lcd_flag = false;
bool sd_insert = false;
int lcd_pattern = 10;
uint16_t lcd_back = 0;
uint8_t battery_persent;
long current_time = 0;
long current_time_buff = 0;
int descend_velocity_err;
int descend_torque_err;
float velocity_Kp = 0.0666;
unsigned int torque_Kp = 850;
float descend_torque;
float descend_angle;
unsigned int descend_velocity_threshold;
unsigned char brake_pattern = 11;

//Touch Sensor
static const int touch_upPin = 12;
static const int touch_downPin = 5;
bool touch_value_1 = false;
bool touch_value_2 = false;

//Timer Interrupt
char  xbee_re_buffer[16];
unsigned  int xbee_index;

unsigned char se_pattern = 1;
unsigned char re_pattern;
float re_val;
int sleep_flag;
int emergency_stopPin = 35;
volatile bool emergency_value = false;
unsigned int emergency_flag_buff;
bool touch_up_toggle_flag = false;
bool touch_down_toggle_flag = false;
bool servo_tx_flag = false;
bool torque_tx_flag = false;
bool servo_rx_flag = false;

// WiFi credentials.
// Set password to "" for open networks.
char ssid2[] = "Buffalo-G-9030";
char pass2[] = "kdtedkktjkcpc";
char ssid1[] = "Owl";
char pass1[] = "1234567890";
char ssid3[] = "Buffalo-G-0CBA";
char pass3[] = "hh4aexcxesasx";

bool second_flag = false;
bool third_flag = false;
bool wifi_flag = true;
unsigned char wifi_cnt = 0;

bool OTA_flag = false;

// Time
char ntpServer[] = "ntp.nict.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;

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

//RS405CB
unsigned char sendbuf[32];
unsigned char readbuf[32];
const int txden = 15;
int servo_angle_buff = 0;
float servo_angle = 0.0F;
int servo_torque_buff = 0;
float servo_torque = 0.0F;
int servo_temp = 0;
int servo_voltage_buff = 0;
float servo_voltage = 0.00F;
unsigned int servo_angle_open;
bool move_flag = true;

// Paramenters
unsigned int target_travel = 0;
unsigned int velocity_threshold = 0;
char motor_output;
unsigned int climb_height;
unsigned int descend_height;
unsigned int climb_velocity;
int descend_velocity;
unsigned int climber_accel;
int starting_count;
int stop_wait;

//RSSI
const int rssiPin = 34;
unsigned int rssi_width;
int rssi_value;
bool commu_loss_flag = false;
unsigned char commu_loss_cnt = 0;



//Prototype
//------------------------------------------------------------------//
int8_t getBatteryGauge(void);
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);
void initEncoder(void);
void buttonAction(void);
void lcdDisplay(void);
void velocityControl(int object_count);
void xbee_re(void);
void xbee_se(void);
void eeprom_write(void);
void eeprom_read(void);
void move(int sPos, int sTime);
void torque(int sMode);
void getServoStatus(void);
void readEncoder(void);
void touch_up(void);
void touch_down(void);
void emergency_stop(void);
void getTimeFromNTP(void);
void getTime(void);
void descendVelocityControl(int velocity);


//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin(1, 1, 1, 1);
  
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);  
  M5.Lcd.printf("Connecting Fact");  
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid1, pass1);  
  while (WiFi.status() != WL_CONNECTED) {
    wifi_cnt++;
    M5.Lcd.printf(".");  
    if( wifi_cnt > 8 ) {
      wifi_cnt = 0;
      second_flag = true;
      while(WiFi.status() == WL_CONNECTED ){
        WiFi.disconnect();
        delay(10);
      }
      break;
    }
    delay(500);  
  }
  if(second_flag) {
    M5.Lcd.setCursor(0, 30);  
    M5.Lcd.printf("Connecting X1Ext ");  
    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid2, pass2);  
    while (WiFi.status() != WL_CONNECTED) {
      wifi_cnt++;
      M5.Lcd.printf(".");  
      if( wifi_cnt > 8 ) {
        wifi_cnt = 0;
        third_flag = true;
        while(WiFi.status() == WL_CONNECTED ){
          WiFi.disconnect();
          delay(10);
        }
        break;
      }
      delay(500);  
    }
  }
  if(third_flag) {
    M5.Lcd.setCursor(0, 60);  
    M5.Lcd.printf("Connecting Phone ");  
    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid3, pass3);  
    while (WiFi.status() != WL_CONNECTED) {
      wifi_cnt++;
      M5.Lcd.printf(".");  
      if( wifi_cnt > 8 ) {
        wifi_cnt = 0;
        wifi_flag = false;
        break;
      }
      delay(500);  
    }
  }
  M5.Lcd.clear();  
  M5.Lcd.setCursor(0, 0);  
  M5.Lcd.printf("Hold BtnA to switch OTA");
  delay(1000);
  if(!M5.BtnA.read()) {
    if( wifi_flag ) {
      // timeSet
      getTimeFromNTP();
      getTime();
      while(WiFi.status() == WL_CONNECTED ){
        WiFi.disconnect();
        delay(2000);
      }
      //fname_buff  = "/log/Climb_"
      //            +(String)(timeinfo.tm_year + 1900)
      //            +"_"+(String)(timeinfo.tm_mon + 1)
      //            +"_"+(String)timeinfo.tm_mday
      //            +"_"+(String)timeinfo.tm_hour
      //            +"_"+(String)timeinfo.tm_min
      //            +".csv";
      //fname = fname_buff.c_str();
    } else {
      //fname_buff  = "/log/Climb_log.csv";
      //fname = fname_buff.c_str();
    }
    WiFi.mode(WIFI_OFF);
  } else {
    OTA_flag = true;
    ArduinoOTA
    .setHostname("M5StackClimberControllerGP")
    .onStart([]() {})
    .onEnd([]() {})
    .onProgress([](unsigned int progress, unsigned int total) {})
    .onError([](ota_error_t error) {}); 
    ArduinoOTA.begin();
    lcd_pattern = 51;
  }
  M5.Lcd.clear();

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 


  initEncoder();

  M5.Lcd.setTextSize(2);

  pinMode(rssiPin, INPUT);  
  pinMode(touch_upPin, INPUT);
  pinMode(touch_downPin, INPUT);
  pinMode(emergency_stopPin, INPUT);


  // Initialize GPIO Interrupt
  attachInterrupt(emergency_stopPin, emergency_stop, FALLING);

  // Initialize IIC
  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU
  //M5.IMU.Init();

  //Serial.begin(115200);
  // Servo Torque
  pinMode(txden, OUTPUT);  
  digitalWrite(txden, LOW);
  delay(1000);

  Serial2.begin(115200);
  EEPROM.begin(128);
  delay(10);
  eeprom_read();

  


  esc.attach(escPin, ESC_LDEC_CHANNEL, 0, 100, 900, 1940);
  esc.write(0);

  sd_insert = SD.begin(TFCARD_CS_PIN, SPI,  24000000);

  M5.Lcd.drawJpgFile(SD, "/icon/icons8-sd.jpg", 280,  0);
  //M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-100.jpg", 240,  0);
  if( wifi_flag ){
    M5.Lcd.drawJpgFile(SD, "/icon/icons8-wi-fi-1.jpg", 200,  0);
  } else {
    M5.Lcd.drawJpgFile(SD, "/icon/icons8-wi-fi-0.jpg", 200,  0);
  }
  //M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-100.jpg", 160,  0);

  
}

//Main
//------------------------------------------------------------------//
void loop() {

  if(OTA_flag) ArduinoOTA.handle();

  timerInterrupt();  
  xbee_re();
  xbee_se();

  current_time = millis() - current_time_buff;

  switch (pattern) {
  case 0: 
    power = 0;   
    esc.write(power);
    lcdDisplay();
    buttonAction();
    break; 

  case 11:
    if( current_time > 0 ) {
      power_increment = (float)100 / (10 / climber_accel *1000 / 50);
      using index_t = decltype(encoder1_status_buffer)::index_t;
      for (index_t i = 0; i < encoder1_status_buffer.size(); i++) {
        encoder1_status_buffer.push(1);
        }
      using index_t = decltype(encoder2_status_buffer)::index_t;
      for (index_t i = 0; i < encoder2_status_buffer.size(); i++) {
        encoder2_status_buffer.push(1);
      }
      pattern = 21;
      break;
    }
    break;

  case 21:
    esc.write(power);
    if( velocity_threshold >= 2 ) {
      time_buff = millis();   
      pattern = 31;
      break;
    }
    braking_distance = (velocity_1/3.6/ 9.8)*velocity_1/2/3.6;
    target_travel = climb_height - braking_distance;
    if( travel_1 >= target_travel || travel_2 >= target_travel){
      pattern = 41;
      break;
    }
    break;


  case 31:
    esc.write(power);
    if( travel_1 >= target_travel || travel_2 >= target_travel ) {
      pattern = 41;
      break;
    }
    break;

  case 41:
    power = 0;   
    esc.write(power);                          
    if( velocity_1 < BRAKE_THRESHOLD_VELOCITY || velocity_2 < BRAKE_THRESHOLD_VELOCITY ) {
      time_buff = millis();
      pattern = 51;
      break;
    }
    break;

  case 51:
    if( millis() - time_buff > stop_wait *1000 ) {
      pattern = 61;
    }
    break;

  case 61:
    power = 0;
    esc.write(power);
    descendVelocityControl(descend_velocity);
    break;

  case 81:
  /*
    if( travel_1 <= descend_height || travel_2 <= descend_height ){
      torque(0);
      pattern = 0;
      break;

    
  
    /*descend_velocity_err = descend_velocity - velocity_1;
    descend_torque = descend_velocity_err * velocity_Kp + 0.1;
    descend_torque_err = descend_torque - servo_torque;
    descend_angle = descend_torque_err * torque_Kp ;
    if( move_flag ) {
      torque(1);
      move(descend_angle, 0);
      move_flag = false;
    }*/  
    

  // Emergency stopPin
  case 900:
    lcdDisplay();
    power = 0;
    esc.write(power);
    break;

  // Emergency touch_upPin
  case 901:
    power = 0;
    esc.write(power);
    break;

  // Emergency touch_downPin
  case 902:
    power = 0;
    esc.write(power);
    break;

  // Emergency Encoder
  case 903:
    power = 0;
    esc.write(power);
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

    readEncoder();
    battery_persent = getBatteryGauge();

    emergency_value = !digitalRead(emergency_stopPin);
    if ( pattern == 900 && !emergency_value ) pattern = 0;
    emergency_flag_buff++;

    touch_value_1 = digitalRead(touch_upPin);
    touch_value_2 = digitalRead(touch_downPin);

    if( touch_value_1 ) {
      touch_up_buffer.push(1);
    } else {
      touch_up_buffer.push(0);
    }
    if( touch_value_2 ) {
      touch_down_buffer.push(1);
    } else {
      touch_down_buffer.push(0);
    }

    touch_up_status_counter = 0;
    using index_t = decltype(touch_up_buffer)::index_t;
    for (index_t i = 0; i < touch_up_buffer.size(); i++) {
      touch_up_status_counter += touch_up_buffer[i];
    }
    if( touch_up_status_counter > 5 && !touch_up_toggle_flag ) {
      touch_up_toggle_flag = true;
      se_pattern = 1;
      torque(0);
      pattern = 901;
    } else if(touch_up_status_counter == 0){
      touch_up_toggle_flag = false;
    }
    touch_down_status_counter = 0;
    using index_t = decltype(touch_down_buffer)::index_t;
    for (index_t i = 0; i < touch_down_buffer.size(); i++) {
      touch_down_status_counter += touch_down_buffer[i];
    }
    if( touch_down_status_counter > 5 && !touch_down_toggle_flag ) {
      touch_down_toggle_flag = true;
      se_pattern = 1;
      torque(0);
      pattern = 902;
    } else if(touch_down_status_counter == 0){
      touch_down_toggle_flag = false;
    }

    iTimer10++;
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
      if(pattern == 21) {
        if(power < 100) {
         power += power_increment;
        } else {
         power = 100;
        }
        if( velocity_1 >= climb_velocity || velocity_2 >= climb_velocity ){
         velocity_threshold++;
        }
      }  
      
     break;

    case 20:
      if(se_pattern ==  101 ){
        Serial2.printf("%3.2f, ",(float)current_time/1000);
        Serial2.printf("%3d, "  ,pattern);
        Serial2.printf("%5.1f, "  ,power);
        Serial2.printf("%5.2f, ",travel_1);
        Serial2.printf("%5.2f, ",velocity_1);
        Serial2.printf("%5.2f, ",travel_2);
        Serial2.printf("%5.2f, ",velocity_2);
        Serial2.printf("%5d, "  ,touch_up_status_counter);
        Serial2.printf("%5.1f, "  ,servo_angle);
        Serial2.printf("%5.3f, "  ,servo_torque);
        Serial2.printf("%5d, "  ,servo_temp);
        Serial2.printf("%5.2f, "  ,servo_voltage);
        Serial2.printf("\n");
      }
      break;

    case 30:
      rssi_width = pulseIn(rssiPin, HIGH, 100);
      if( rssi_width == 0 ) {
        if( digitalRead(rssiPin) ) {
          rssi_value = -57;
        } else {
          rssi_value = 0;
        }
      } else {
        rssi_value = rssi_width * 1.3814 - 117;
      }      
      if( rssi_value < -90 ) {
        commu_loss_cnt++;
        if(commu_loss_cnt>=5) {
          commu_loss_flag = true;
        } 
      } else {
        commu_loss_flag = false;
        commu_loss_cnt = 0;
      }
     break;
    
    case 40:   
      if( pattern != 61 ) getServoStatus();       
      break;

    case 50:
      move_flag = true;
      if( travel_1 == travel_1_buff ){
        encoder1_status_buffer.push(0);
      } else {
        encoder1_status_buffer.push(1);
      }
      if( travel_2 == travel_2_buff ){
        encoder2_status_buffer.push(0);
      } else {
        encoder2_status_buffer.push(1);
      }
      encoder1_status_counter = 0;
      using index_t = decltype(encoder1_status_buffer)::index_t;
      for (index_t i = 0; i < encoder1_status_buffer.size(); i++) {
        encoder1_status_counter += encoder1_status_buffer[i];
      }
      if( encoder1_status_counter <= 5 && pattern >= 21 && pattern <= 41 ) {
        se_pattern = 1;
        torque(0);
        pattern = 903;
      }
      encoder2_status_counter = 0;
      using index_t = decltype(encoder2_status_buffer)::index_t;
      for (index_t i = 0; i < encoder2_status_buffer.size(); i++) {
        encoder2_status_counter += encoder2_status_buffer[i];
      }
      if( encoder2_status_counter <= 5 && pattern >= 21 && pattern <= 41 ) {
        se_pattern = 1;
        torque(0);
        pattern = 903;
      }
      travel_1_buff = travel_1;
      travel_2_buff = travel_2;
      if( lcd_pattern > 10 && lcd_pattern < 20 && lcd_back > 2000 ) {
        M5.Lcd.fillRect(0, 29, 320, 211, BLACK);
        lcd_pattern = 10;
      } else if( lcd_pattern > 110 && lcd_pattern < 120 && lcd_back > 2000){
        M5.Lcd.fillRect(0, 29, 320, 211, BLACK);
        lcd_pattern = 110;
      }
      lcd_flag = true;
      iTimer50 = 0;
      break;
    }
  }
}

//XBee Receive
//------------------------------------------------------------------//
void xbee_re(void){

  while(Serial2.available()){
    xbee_re_buffer[xbee_index]=Serial2.read();
    Serial2.write(xbee_re_buffer[xbee_index]);

    if(xbee_re_buffer[xbee_index]==0x08){
      xbee_re_buffer[xbee_index-1]==NULL;
      xbee_index--;
      Serial2.printf(" ");
      Serial2.write(0x08);
    }else if(xbee_re_buffer[xbee_index]==0x0D){
      Serial2.read();
      Serial2.printf("\n\n");
      if(se_pattern == 0){
        re_pattern = atoi(xbee_re_buffer);
      }else if(se_pattern == 2){
        re_val = atof(xbee_re_buffer);
      }
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
          current_time_buff = millis() + starting_count*1000;
          pattern = 11;
          se_pattern = 101;
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
        climb_height = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 32:
        se_pattern = 32;
        re_pattern = 42;
        break;
      case 42:
        climb_velocity = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 33:
        se_pattern = 33;
        re_pattern = 43;
        break;
      case 43:
        descend_velocity = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 34:
        se_pattern = 34;
        re_pattern = 44;
        break;
      case 44:
        climber_accel = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 35:
        se_pattern = 35;
        re_pattern = 45;
        break;
      case 45:
        starting_count = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 36:
        se_pattern = 36;
        re_pattern = 46;
        break;
      case 46:
        stop_wait = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 37:
        se_pattern = 37;
        re_pattern = 47;
        break;
      case 47:
        servo_angle_open = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 38:
        se_pattern = 38;
        re_pattern = 48;
        break;
      case 48:
        descend_velocity = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;

      case 39:
        se_pattern = 39;
        re_pattern = 49;
        break;
      case 49:
        descend_height = re_val;
        eeprom_write();
        se_pattern = 1;
        re_pattern = 0;
        break;


      case 221:
        if(re_val == 1){
          torque(1);
          delay(10);
          move(servo_angle_open,0);

        }else{
          se_pattern = 1;
        }
        re_pattern = 0;
        break;
      }
      
      } else if( xbee_re_buffer[xbee_index] ==  'T' ||  xbee_re_buffer[xbee_index] == 't'){
        re_pattern  = 0;
        se_pattern  = 101;
        Serial2.printf("\n");
      } else if( xbee_re_buffer[xbee_index] ==  'B' ||  xbee_re_buffer[xbee_index] == 'b'){
        torque(0);
        delay(10);
        move(-40, 0);
        Serial2.printf("\n\n");
        Serial2.printf(" Brake On \n ");
        re_pattern  = 0;
        se_pattern  = 1;
        Serial2.printf("\n");
      } else if( xbee_re_buffer[xbee_index] ==  'O' ||  xbee_re_buffer[xbee_index] == 'o'){
        re_pattern  = 221;
        se_pattern  = 221;
        Serial2.printf("\n");
      } else if( xbee_re_buffer[xbee_index] ==  'A' ||  xbee_re_buffer[xbee_index] == 'a'){
        torque(1);
        delay(10);
        move(servo_angle_open, 0);
        pattern = 61;
        se_pattern = 101;
        Serial2.printf("\n");
      } else if( xbee_re_buffer[xbee_index] ==  ' '){
        lcd_pattern = 0;
        Serial2.printf("\n\n");
        Serial2.printf(" Emergency Stop Enable Spacekey\n ");
        Serial2.printf(" return Case 0 \n ");
        pattern = 0;
        re_pattern = 0;
        Serial2.printf("\n");
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
    Serial2.printf("\n\n\n\n\n\n");
    Serial2.printf("climber Controller (M5Stack version) "
                    "Test program Ver1.20\n");
    Serial2.printf("\n");
    Serial2.printf(" 11 : Start Seqwnse\n");
    Serial2.printf(" B  : brake On\n");
    Serial2.printf(" O  : brake Off\n");
    Serial2.printf("\n");
    Serial2.printf(" 31 :  Climb Height         [%4d]\n",climb_height);
    Serial2.printf(" 32 :  Climb Velocity       [%4d]\n",climb_velocity);
    Serial2.printf(" 33 :  Descend Velocity     [%4d]\n",descend_velocity);
    Serial2.printf(" 34 :  Climber Accel        [%4d]\n",climber_accel);
    Serial2.printf(" 35 :  Starting Count       [%4d]\n",starting_count);
    Serial2.printf(" 36 :  Stop Wait            [%4d]\n",stop_wait);
    Serial2.printf(" 37 :  Servo Angle Open     [%4d]\n",servo_angle_open);
    Serial2.printf(" 38 :  Descend Velocity     [%4d]\n",descend_velocity);
    Serial2.printf(" 39 :  Descend Height       [%4d]\n",descend_height);
    Serial2.printf(" T : Terementry\n");
    Serial2.printf(" U : Manual  Climb\n");
    Serial2.printf(" D : Manual  Desvent\n");

    Serial2.printf("\n");
    Serial2.printf(" Please  enter 11 36 ");

    se_pattern = 0;
    break;

  //Waiting value
  case 2 :
    break;

  case 11:
    Serial2.printf("\n Check Current Paramenters\n");
    Serial2.printf("\n");
    Serial2.printf(" 31 :  Climb Height         [%4d]\n",climb_height);
    Serial2.printf(" 32 :  Climb Velocity       [%4d]\n",climb_velocity);
    Serial2.printf(" 33 :  Descend Velocity      [%4d]\n",descend_velocity);
    Serial2.printf(" 34 :  Climber Accel        [%4d]\n",climber_accel);
    Serial2.printf(" 35 :  Starting Count       [%4d]\n",starting_count);
    Serial2.printf(" 36 :  Stop Wait            [%4d]\n",stop_wait);
    Serial2.printf(" 37 :  Servo Angle Open     [%4d]\n",servo_angle_open);
    Serial2.printf(" 38 :  Descend Velocity     [%4d]\n",descend_velocity);
    Serial2.printf(" 39 :  Descend Height       [%4d]\n",descend_height);
    Serial2.printf("\n");
    Serial2.printf(" Confilm to Climb? ->  ");
    se_pattern = 2;
    break;

  case 31:
     Serial2.printf(" Climb Height [%3d]\n " ,climb_height);
     Serial2.printf(" Please enter 0 to 1000 -> ");
     se_pattern = 2;
     break;

  case 32:
      Serial2.printf(" Climb Velocity [%4d]\n ",climb_velocity);
      Serial2.printf(" Please enter 0 to 50 -> ");
      se_pattern = 2;
      break;

  case 33:
      Serial2.printf(" Descend Velocity [%4d]\n ",descend_velocity);
      Serial2.printf(" Please enter 0 to 50 -> ");
      se_pattern = 2;
      break;
    
  case 34:
      Serial2.printf(" Climber Accel [%4d]\n ",climber_accel);
      Serial2.printf(" Please enter 0 to 50 -> ");
      se_pattern = 2;
      break;

  case 35:
      Serial2.printf(" Starting Count [%2d]\n ",starting_count);
      Serial2.printf(" Please enter 0 to 60 -> ");
      se_pattern = 2;
      break;

  case 36:
      Serial2.printf(" Stop Wait [%2d]\n ",stop_wait);
      Serial2.printf(" Please enter 0 to 60 -> ");
      se_pattern = 2;
      break;

  case 37:
      Serial2.printf(" Servo Angle Open    [%4d]\n",servo_angle_open);
      Serial2.printf(" Please enter 0 to 800 -> ");
      se_pattern = 2;

  case 38:
      Serial2.printf(" Descend Velocity    [%4d]\n",descend_velocity);
      Serial2.printf(" Please enter 0 to 100 -> ");
      se_pattern = 2;

  case 39:
      Serial2.printf(" Descend Height    [%4d]\n",descend_height);
      Serial2.printf(" Please enter 0 to 1000 -> ");
      se_pattern = 2;

  //Telementry Mode
  case 101:
    break;

  case 221:
    Serial2.printf(" Confilm to Brake Off? ->  ");
    se_pattern = 2;
    break;
  }
}

//emergency_Stop
//------------------------------------------------------------------//
void emergency_stop(void){
  if( emergency_flag_buff > 500 ){
    Serial2.printf(" detection emergency \n");
    emergency_flag_buff = 0;
    torque(0);
    pattern = 900;
  }
}

// RS405CB move
//------------------------------------------------------------------//
void move(int sPos, int sTime) {
  unsigned char sum;
  while(servo_rx_flag);
  servo_tx_flag = true;

  sendbuf[0] = (unsigned char) 0xFA;  // Hdr1
  sendbuf[1] = (unsigned char) 0xAF;  // Hdr2
  sendbuf[2] = (unsigned char) 1;     // ID
  sendbuf[3] = (unsigned char) 0x00;  // Flag
  sendbuf[4] = (unsigned char) 0x1E;  // Addr(0x1E=30)
  sendbuf[5] = (unsigned char) 0x04;  // Length(4byte)
  sendbuf[6] = (unsigned char) 0x01;  // Number
  sendbuf[7] = (unsigned char) (sPos & 0x00FF); // Position
  sendbuf[8] = (unsigned char) ((sPos & 0xFF00) >> 8); // Position
  sendbuf[9] = (unsigned char) (sTime & 0x00FF); // Time
  sendbuf[10] = (unsigned char) ((sTime & 0xFF00) >> 8); // Time

  // Caluculate check SUM
  sum = sendbuf[2];
  for (int i = 3; i < 11; i++) {
    sum = (unsigned char)(sum ^ sendbuf[i]);
  }
  sendbuf[11] = sum;

  // Transmit
  digitalWrite(txden, HIGH);
  Serial.write(sendbuf, 12); 
  delayMicroseconds(1200);
  digitalWrite(txden,LOW);
  
  servo_tx_flag = false;
}

// RS405CB torque
//------------------------------------------------------------------//
void torque(int sMode) {
  unsigned char sum;
  while(servo_rx_flag);
  torque_tx_flag = true;

  sendbuf[0] = (unsigned char) (0xFA);  // Hdr1
  sendbuf[1] = (unsigned char) (0xAF);  // Hdr2
  sendbuf[2] = (unsigned char) (1);   // ID
  sendbuf[3] = (unsigned char) (0x00);  // Flag
  sendbuf[4] = (unsigned char) (0x24);  // Addr(0x24=36)
  sendbuf[5] = (unsigned char) (0x01);  // Length(1byte)
  sendbuf[6] = (unsigned char) (0x01);  // Number
  sendbuf[7] = (unsigned char)((sMode&0x00FF)); // ON/OFFフラグ

  // Caluculate check SUM
  sum = sendbuf[2];
  for (int i = 3; i < 8; i++) {
    sum = (unsigned char) (sum ^ sendbuf[i]);
  }
  sendbuf[8] = sum;

  // Transmit
  digitalWrite(txden, HIGH);
  Serial.write(sendbuf, 9);
  delayMicroseconds(1000);
  digitalWrite(txden, LOW);

  torque_tx_flag = false;
}

// RS405CB Status
//------------------------------------------------------------------//
void getServoStatus(void) {
  unsigned char sum;
  if(!servo_tx_flag) {
    servo_rx_flag = true;

    sendbuf[0] = (unsigned char) 0xFA;  //
    sendbuf[1] = (unsigned char) 0xAF;  //
    sendbuf[2] = (unsigned char) 1;     //
    sendbuf[3] = (unsigned char) 0x09;  //
    sendbuf[4] = (unsigned char) 0x00;  //
    sendbuf[5] = (unsigned char) 0x00;  //
    sendbuf[6] = (unsigned char) 0x01;  //

    sum = sendbuf[2];
    for (int i = 3; i < 7; i++) {
      sum = (unsigned char) (sum ^ sendbuf[i]);
    }
    sendbuf[7] = sum;

    digitalWrite(txden, HIGH);
    Serial.write(sendbuf, 8);
    delayMicroseconds(750);
    digitalWrite(txden, LOW);
    for(int i=0;i<26;i++) {
      readbuf[i] = Serial.read();
    }
    servo_angle_buff = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
    servo_torque_buff = ((readbuf[14] << 8) & 0x0000FF00) | (readbuf[13] & 0x000000FF);
    servo_temp = ((readbuf[16] << 8) & 0x0000FF00) | (readbuf[15] & 0x000000FF );
    servo_voltage_buff = ((readbuf[18] << 8) & 0x0000FF00) | (readbuf[17] & 0x000000FF);


    if(servo_angle_buff < 32768) {
      servo_angle = servo_angle_buff / 10;
    } else {
      servo_angle = (servo_angle_buff - 65535) / 10;
    }
    servo_torque = (float)servo_torque_buff / 1000;
    servo_voltage = (float) servo_voltage_buff / 100;

    servo_rx_flag = false;

  }
  
}

// RS405CB Brake Speed Control
//------------------------------------------------------------------//
void descendVelocityControl(int velocity) { 

  switch (brake_pattern) {
  case 11:
    if(velocity_1*-1 >= velocity || velocity_2*-1 >= velocity ){
      time_buff = millis();
      brake_pattern = 12;
    }
    break;

  case 12:
    if( millis() - time_buff > 10 ) {
      time_buff = millis();
      torque(0);
      brake_pattern = 13;
      break;
    }
    break;
    
  case 13:
    if( millis() - time_buff > 10 ) {
      time_buff = millis();
      brake_pattern = 14;
      break;
    }
    break;

  case 14:
    if( millis() - time_buff > 10 ) {
      getServoStatus();
      brake_pattern = 21;
      break;
    }
    break;

  case 21:
    if( velocity_1*-1 < 1 || velocity_2*-1 < 1 ){
      torque(1);
      time_buff = millis();
      brake_pattern = 22;
      break;
    }
    break;

  case 22:
    if( millis() - time_buff > 10 ) {
      move(servo_angle_open, 50);
      time_buff = millis();
      brake_pattern = 23;
      break;
    }
    break;

  case 23:
    if(millis() - time_buff > 700) {
      brake_pattern = 24;
      break;
    }
    break;

  case 24:
    if(millis() - time_buff > 10) {
      getServoStatus();
      brake_pattern = 11;
      break;
    }
    break;
  }
}

// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {
  if(lcd_flag) {

   M5.Lcd.setCursor(5, 7);
    if( emergency_value ){
      M5.Lcd.setTextColor(RED,BLACK);
      M5.Lcd.printf("EMG ENA");
    } else {
      M5.Lcd.setTextColor(BLUE,BLACK);
      M5.Lcd.printf("EMG DIS");
    }
      M5.Lcd.setTextColor(CYAN,BLACK);


    if( battery_persent == 100) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-100.jpg", 240,  0);
    } else if( battery_persent == 75) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-75.jpg", 240,  0);
    } else if( battery_persent == 50) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-50.jpg", 240,  0);
    } else if( battery_persent == 25) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-25.jpg", 240,  0);
    } else {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-0.jpg", 240,  0);
    }
    if( rssi_value > -60) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-100.jpg", 160,  0);
    } else if( battery_persent == 75) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-75.jpg", 160,  0);
    } else if( battery_persent == 50) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-50.jpg", 160,  0);
    } else if( battery_persent == 25) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-25.jpg", 160,  0);
    } else {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-signal-0.jpg", 160,  0);
    }


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
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 234, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(234, 170);
      M5.Lcd.printf("Params");  
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
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 234, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(234, 170);
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
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 234, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(234, 170);
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
      M5.Lcd.drawJpgFile(SD, "/icon/params.jpg", 234, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(14, 170);
      M5.Lcd.printf("Status"); 
      M5.Lcd.setCursor(120, 170);
      M5.Lcd.printf("Control");  
      M5.Lcd.setCursor(234, 170);
      M5.Lcd.printf("Params");
      break;

    case 51:
      M5.Lcd.setCursor(60, 135);
      M5.Lcd.printf("OTA Mode active");
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
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 228, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(24, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(125, 170);
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
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 228, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(24, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(125, 170);
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
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 228, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(24, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(125, 170);
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
      M5.Lcd.drawJpgFile(SD, "/icon/codec.jpg", 228, 78);
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(24, 170);
      M5.Lcd.printf("Brake"); 
      M5.Lcd.setCursor(125, 170);
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
      M5.Lcd.setCursor(30, 50);
      M5.Lcd.printf("angle");
      M5.Lcd.setCursor(50, 90);
      M5.Lcd.printf("%5.1f", servo_angle); 
      M5.Lcd.setCursor(30, 155);
      M5.Lcd.printf("torque");
      M5.Lcd.setCursor(50, 190);
      M5.Lcd.printf("%5.3f", servo_torque);
      M5.Lcd.setCursor(190, 50);
      M5.Lcd.printf("temper");
      M5.Lcd.setCursor(210, 90);
      M5.Lcd.printf("%5d", servo_temp); 
      M5.Lcd.setCursor(190, 155);
      M5.Lcd.printf("voltage");
      M5.Lcd.setCursor(220, 190);
      M5.Lcd.printf("%5.1f", servo_voltage);
     
      break;

    

//Status_Sensor
    case 1120:
      M5.Lcd.drawFastVLine(106, 30, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawFastVLine(214, 30, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawFastHLine(0, 30, 320, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(106, 30, 108, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(214, 30, 106, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(0, 30, 106, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(24, 50);
      M5.Lcd.printf("Touch"); 
      M5.Lcd.setCursor(24, 90);
      M5.Lcd.printf("Up"); 
      M5.Lcd.setCursor(40, 130);
      M5.Lcd.printf("%d", touch_value_1);
      M5.Lcd.setCursor(24, 170);
      M5.Lcd.printf("Down"); 
      M5.Lcd.setCursor(40, 200);
      M5.Lcd.printf("%d", touch_value_2);
      M5.Lcd.setCursor(125, 170);
      M5.Lcd.printf("Marker");  
      M5.Lcd.setCursor(238, 170);
      M5.Lcd.printf("Press");
      break;



//Status_Codec
    case 1130:
      M5.Lcd.drawFastVLine(160, 30, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawFastHLine(0, 30, 320, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(0, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.drawRect(160, 30, 160, 210, M5.Lcd.color565(50,50,50));
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(30, 50);
      M5.Lcd.printf("travel_1");
      M5.Lcd.setCursor(50, 90);
      M5.Lcd.printf("%5.2f", travel_1); 
      M5.Lcd.setCursor(30, 155);
      M5.Lcd.printf("velocity_1");
      M5.Lcd.setCursor(60, 190);
      M5.Lcd.printf("%4.2f", velocity_1);
      M5.Lcd.setCursor(190, 50);
      M5.Lcd.printf("travel_2");
      M5.Lcd.setCursor(210, 90);
      M5.Lcd.printf("%5.2f", travel_2); 
      M5.Lcd.setCursor(190, 155);
      M5.Lcd.printf("velocity_2");
      M5.Lcd.setCursor(220, 190);
      M5.Lcd.printf("%4.2f", velocity_2);
      break;
  

//Control
    case 120:
      break;


    
//Params
    case 130:
      M5.Lcd.setTextColor(CYAN,BLACK);
      M5.Lcd.setCursor(20, 50);
      M5.Lcd.printf("climb_height: %4d", climb_height);
      M5.Lcd.setCursor(20, 80);
      M5.Lcd.printf("climb_velocity: %4d", climb_velocity); 
      M5.Lcd.setCursor(20, 110);
      M5.Lcd.printf("descend_velocity: %4d", descend_velocity); 
      M5.Lcd.setCursor(20, 140);
      M5.Lcd.printf("climber_accel: %4d", climber_accel); 
      M5.Lcd.setCursor(20, 170);
      M5.Lcd.printf("starting_count: %4d", starting_count); 
      M5.Lcd.setCursor(20, 200);
      M5.Lcd.printf("stop_wait: %4d", stop_wait); 
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
      M5.Lcd.fillRect(0, 29, 320, 211, BLACK);
      if( lcd_pattern >= 110 && lcd_pattern < 1000 ){
        lcd_pattern = 10;
      }
      if( lcd_pattern == 1110 || lcd_pattern == 1120 || lcd_pattern == 1130 ){
        lcd_pattern = 110;
      }
      
    }
  } else if (M5.BtnB.wasPressed()) {
    M5.Lcd.fillRect(0, 29, 320, 211, BLACK);
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

      case 1121:
        lcd_pattern = 11120;
        break;

      case 113:
        lcd_pattern = 1130;
        break;

      case 13:
        lcd_pattern = 130;
        break;

    }
  } else if (M5.BtnC.wasPressed()) {
    M5.Lcd.fillRect(0, 29, 320, 211, BLACK);
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
  }
}

// EEPROM Write
//------------------------------------------------------------------//
void eeprom_write(void){
    EEPROM.write(0,  (climb_height & 0xFF));
    EEPROM.write(1,  (climb_height>>8 & 0xFF));
    EEPROM.write(2,  (climb_height>>16 & 0xFF));
    EEPROM.write(3,  (climb_height>>24 & 0xFF));
    EEPROM.write(4,  (climb_velocity & 0xFF));
    EEPROM.write(5,  (climb_velocity>>8 & 0xFF));
    EEPROM.write(6,  (climb_velocity>>16 & 0xFF));
    EEPROM.write(7,  (climb_velocity>>24 & 0xFF));
    EEPROM.write(8,  (descend_velocity & 0xFF));
    EEPROM.write(9,  (descend_velocity>>8 & 0xFF));
    EEPROM.write(10, (descend_velocity>>16 & 0xFF));
    EEPROM.write(11, (descend_velocity>>24 & 0xFF));
    EEPROM.write(12, (climber_accel & 0xFF));
    EEPROM.write(13, (climber_accel>>8 & 0xFF));
    EEPROM.write(14, (climber_accel>>16 & 0xFF));
    EEPROM.write(15, (climber_accel>>24 & 0xFF));
    EEPROM.write(16, starting_count);
    EEPROM.write(17, stop_wait);
    EEPROM.write(18, (servo_angle_open & 0xFF));
    EEPROM.write(19, (servo_angle_open>>8 & 0xFF));
    EEPROM.write(20, (servo_angle_open>>16 & 0xFF));
    EEPROM.write(21, (servo_angle_open>>24 & 0xFF));
    delay(10);
    EEPROM.commit();
    delay(10);
}

//EEPROM read
//------------------------------------------------------------------//
void eeprom_read(void){
    climb_height = EEPROM.read(0) + (EEPROM.read(1)<<8) + (EEPROM.read(2)<<16) + (EEPROM.read(3)<<24);
    climb_velocity = EEPROM.read(4) + (EEPROM.read(5)<<8) + (EEPROM.read(6)<<16) + (EEPROM.read(7)<<24);
    descend_velocity = EEPROM.read(8) + (EEPROM.read(9)<<8) + (EEPROM.read(10)<<16) + (EEPROM.read(11)<<24);
    climber_accel = EEPROM.read(12) + (EEPROM.read(13)<<8) + (EEPROM.read(14)<<16) + (EEPROM.read(15)<<24);
    starting_count = EEPROM.read(16);
    stop_wait = EEPROM.read(17);
    servo_angle_open = EEPROM.read(18) + (EEPROM.read(19)<<8) + (EEPROM.read(20)<<16) + (EEPROM.read(21)<<24);
    delay(10);
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
int8_t getBatteryGauge() {
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  Wire.endTransmission(false);
  if(Wire.requestFrom(0x75, 1)) {
    switch (Wire.read() & 0xF0) {
    case 0xE0: return 25;
    case 0xC0: return 50;
    case 0x80: return 75;
    case 0x00: return 100;
    default: return 0;
    }
  }
  return -1;
}

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
  
// Read Encoder
//------------------------------------------------------------------//
void readEncoder(void) {
  pcnt_get_counter_value(PCNT_UNIT_0, &delta_count_1);
  pcnt_counter_clear(PCNT_UNIT_0);  
  total_count_1 += delta_count_1;
  delta1_buffer.push(delta_count_1);
  delta_avg_1 = 0;
  using index_t = decltype(delta1_buffer)::index_t;
	for (index_t i = 0; i < delta1_buffer.size(); i++) {
		delta_avg_1 += delta1_buffer[i];
	}
  velocity_1 = (float)delta_avg_1 / AVERATING_BUFFER_SIZE * DRIVER_ROLLER_PERIMETER / DRIVER_ROLLER_PPR *DRIVER_ROLLER_COEFFICIENT* 36 / 10;
  travel_1 = (float)total_count_1 * DRIVER_ROLLER_PERIMETER / DRIVER_ROLLER_PPR *DRIVER_ROLLER_COEFFICIENT/ 1000;

  pcnt_get_counter_value(PCNT_UNIT_1, &delta_count_2);
  pcnt_counter_clear(PCNT_UNIT_1);  
  total_count_2 += delta_count_2;
  delta2_buffer.push(delta_count_2);
  delta_avg_2 = 0;
  using index_t = decltype(delta2_buffer)::index_t;
	for (index_t i = 0; i < delta2_buffer.size(); i++) {
		delta_avg_2 += delta2_buffer[i];
	}
  velocity_2 = (float)delta_avg_2 / AVERATING_BUFFER_SIZE * IDLER_ROLLER_PERIMETER / IDLER_ROLLER_PPR * IDLER_ROLLER_COEFFICIENT * 36 / 10;
  travel_2 = (float)total_count_2 * IDLER_ROLLER_PERIMETER / IDLER_ROLLER_PPR*IDLER_ROLLER_COEFFICIENT / 1000;
}

//Get Time From NTP
//------------------------------------------------------------------//
void getTimeFromNTP(void){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
  }
}

//Get Convert Time
//------------------------------------------------------------------//
void getTime(void){
  getLocalTime(&timeinfo);
  dateStr = (String)(timeinfo.tm_year + 1900)
          + "/" + (String)(timeinfo.tm_mon + 1)
          + "/" + (String)timeinfo.tm_mday;
  timeStr = (String)timeinfo.tm_hour
          + ":" + (String)timeinfo.tm_min
          + ":" + (String)timeinfo.tm_sec;
}
