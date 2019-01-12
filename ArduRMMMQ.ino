// ====================================================
// ArduRMMMQ Copyright(C) 2019 Furkan Türkal
// This program comes with ABSOLUTELY NO WARRANTY; This is free software,
// and you are welcome to redistribute it under certain conditions; See
// file LICENSE, which is part of this source code package, for details.
// ====================================================//===============

//===============
//INCLUDES
//===============

#include "Arduino_FreeRTOS.h"
#include "LedControl.h"
#include <frt.h>

#include <Wire.h>
#include <LiquidTWI2.h>
#include <SoftwareSerial.h>

//===============
//DEFINES
//===============

#define PIN_BUZZER A1

#define PIN_MQ2_A A0
#define PIN_MQ2_D 5

#define PIN_MQ4_A A4
#define PIN_MQ4_D 4

#define PIN_MQ7_A A2
#define PIN_MQ7_D 3

#define PIN_MQ8_A A3
#define PIN_MQ8_D 2

#define PIN_LED_STATUS_TASK_MQ2 10
#define PIN_LED_STATUS_TASK_MQ4 11
#define PIN_LED_STATUS_TASK_MQ7 12
#define PIN_LED_STATUS_TASK_MQ8 13

#define RANGE_MQ_MAX 1000
#define RANGE_MQ_MIN 0
#define RANGE_MQ_NO_ECHO -1

#define TASK_PRIORITY_DEFAULT 10
#define TASK_PRIORITY_MQ2 2
#define TASK_PRIORITY_MQ4 3
#define TASK_PRIORITY_MQ7 4
#define TASK_PRIORITY_MQ8 5

#define GAS_MQ2LPG    0   
#define GAS_MQ2CO     1   
#define GAS_MQ2SMOKE  2

#define GAS_MQ4LPG    3   
#define GAS_MQ4CH4    4

#define GAS_MQ7CO     5   
#define GAS_MQ7H2     6

#define GAS_MQ8H2     7

#define CALIBARAION_SAMPLE_TIMES    5
#define CALIBRATION_SAMPLE_INTERVAL 500

#define READ_SAMPLE_INTERVAL  50
#define READ_SAMPLE_TIMES     5

#define RL_KOHM_VALUE 5
#define RO_CLEAN_AIR_FACTOR       9.21

//===============
//GLOBAL VARS
//===============

const TickType_t xDelayMS1000 = pdMS_TO_TICKS(1000UL);
const TickType_t xDelayMS3000 = pdMS_TO_TICKS(3000UL);
const TickType_t xDelayMS100 = pdMS_TO_TICKS(100UL);

static const char *pcTextForTaskLCD         = "vTaskLCD Ran!";
static const char *pcTextForTaskLED         = "vTaskLED Ran!";
static const char *pcTextForTaskMQ2         = "vTaskMQ2 Ran!";
static const char *pcTextForTaskMQ4         = "vTaskMQ4 Ran!";
static const char *pcTextForTaskMQ7         = "vTaskMQ7 Ran!";
static const char *pcTextForTaskMQ8         = "vTaskMQ8 Ran!";

static const char *pcMQ2Prefix = "[sensor:MQ2:";
static const char *pcMQ4Prefix = "[sensor:MQ4:";
static const char *pcMQ7Prefix = "[sensor:MQ7:";
static const char *pcMQ8Prefix = "[sensor:MQ8:";
static const char *pcMQPostfix = "]";

//MQ2 - OK
const float MQ2LPGCurve[3]    = {2.3, 0.21, -0.47};
const float MQ2COCurve[3]     = {2.3, 0.72, -0.34};
const float MQ2SmokeCurve[3]  = {2.3, 0.53, -0.44};

//MQ4
const float MQ4LPGCurve[3]    = {2.3, 0.21, -0.47};
const float MQ4CH4Curve[3]    = {2.3, 0.72, -0.34};

//MQ7
const float MQ7COCurve[3]     = {2.3, 0.72, -0.34};
const float MQ7H2Curve[3]     = {2.3, 0.72, -0.34};

//MQ8 - OK
const float MQ8H2Curve[3]     = {2.3, 0.93, -1.44};

LedControl lc = LedControl(10, 12, 11, 1);
LiquidTWI2 lcd(0x20);

float Ro2 = 10;
float Ro4 = 10;
float Ro7 = 10;
float Ro8 = 10;

void vTaskLCD(void *pvParamethers);
void vTaskMQ2(void *pvParamethers);
void vTaskMQ4(void *pvParamethers);
void vTaskMQ7(void *pvParamethers);
void vTaskMQ8(void *pvParamethers);

TaskHandle_t xTaskMQ2Handle = NULL;
BaseType_t xTaskMQ2;

TaskHandle_t xTaskMQ4Handle = NULL;
BaseType_t xTaskMQ4;

TaskHandle_t xTaskMQ7Handle = NULL;
BaseType_t xTaskMQ7;

TaskHandle_t xTaskMQ8Handle = NULL;
BaseType_t xTaskMQ8;

volatile bool wasCalibrated = false;
volatile bool isCalibrating = false;

byte countSuccessTasks = 0;

volatile uint16_t usPPM_MQ2_LPG   = 0;
volatile uint16_t usPPM_MQ2_CO    = 0;
volatile uint16_t usPPM_MQ2_Smoke = 0;

volatile uint16_t usPPM_MQ4_LPG   = 0;
volatile uint16_t usPPM_MQ4_CH4   = 0;

volatile uint16_t usPPM_MQ7_CO    = 0;
volatile uint16_t usPPM_MQ7_H2    = 0;

volatile uint16_t usPPM_MQ8_H2    = 0;

void setup() {
  Serial.begin(9600);

  //Wait the Serial if pin D9 is pressing
  pinMode(9, INPUT);
  if(digitalRead(9) == HIGH) {
    while (!Serial);
  }
  
  Serial.println("[status:setup:starting]");

  pinMode(PIN_BUZZER, OUTPUT);
  
  //Set MQ sensor pins
  pinMode(PIN_MQ2_A, INPUT);
  pinMode(PIN_MQ2_D, INPUT);
  pinMode(PIN_MQ4_A, INPUT);
  pinMode(PIN_MQ4_D, INPUT);
  pinMode(PIN_MQ7_A, INPUT);
  pinMode(PIN_MQ7_D, INPUT);
  pinMode(PIN_MQ8_A, INPUT);
  pinMode(PIN_MQ8_D, INPUT);

  //Set LED pins
  pinMode(PIN_LED_STATUS_TASK_MQ2, OUTPUT);
  pinMode(PIN_LED_STATUS_TASK_MQ4, OUTPUT);
  pinMode(PIN_LED_STATUS_TASK_MQ7, OUTPUT);
  pinMode(PIN_LED_STATUS_TASK_MQ8, OUTPUT);

  //7-Segment module
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  //LCD module
  lcd.setMCPType(LTI_TYPE_MCP23008);
  lcd.begin(16, 2);

  Wire.begin(8);
  Wire.onReceive(onWireReceive);

  xTaskCreate(vTaskLCD, (const portCHAR *) "vTaskLCD", 100, (void *)pcTextForTaskLCD, 4, NULL);
  
  xTaskMQ2 = xTaskCreate(vTaskMQ2, (const portCHAR *) "vTaskMQ2", 100, NULL, 2, &xTaskMQ2Handle);
  if(xTaskMQ2 == pdPASS){countSuccessTasks++;}
  
  xTaskMQ4 = xTaskCreate(vTaskMQ4, (const portCHAR *) "vTaskMQ4", 100, NULL, 2, &xTaskMQ4Handle);
  if(xTaskMQ4 == pdPASS){countSuccessTasks++;}
  
  xTaskMQ7 = xTaskCreate(vTaskMQ7, (const portCHAR *) "vTaskMQ7", 100, NULL, 2, &xTaskMQ7Handle);
  if(xTaskMQ7 == pdPASS){countSuccessTasks++;}
  
  xTaskMQ8 = xTaskCreate(vTaskMQ8, (const portCHAR *) "vTaskMQ8", 100, NULL, 2, &xTaskMQ8Handle);
  if(xTaskMQ8 == pdPASS){countSuccessTasks++;}
  
  //Booting animation
  for(int i = 0; i < 4 * 3; i++){
    lc.setDigit(0, i % 4, 1, false);
    delay(100);
    lc.clearDisplay(0);
  }
  
  digitalWrite(PIN_LED_STATUS_TASK_MQ2, LOW);
  digitalWrite(PIN_LED_STATUS_TASK_MQ4, LOW);
  digitalWrite(PIN_LED_STATUS_TASK_MQ7, LOW);
  digitalWrite(PIN_LED_STATUS_TASK_MQ8, LOW);

  doCalibration();

  if(countSuccessTasks > 0) {
      
    tone(PIN_BUZZER, 1000);
    delay(1000);
    noTone(PIN_BUZZER);
  
    Serial.println("[status:setup:started]");
    vTaskStartScheduler();
  } else {
    Serial.println("[status:setup:failed]");
  }
}

void onWireReceive(int bytes) {
  char c = Wire.read();
  int data = Wire.read();
  if (c == 'a') {
    Serial.print("[sensor:MQ135:");
  } else if (c == 'b') {
    Serial.print("[sensor:MQ9:");
  } else if (c == 'c') {
    Serial.print("[sensor:MQ5:");
  }
  Serial.print(data);
  Serial.println("]");
}

void loop() {
}

float MQResistanceCalculation(int raw_adc) {
  return (((float)RL_KOHM_VALUE * (1023-raw_adc) / raw_adc));
}

float MQCalibration(int mq_pin) {
  float val = 0;

  for (int i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  
  val = val / CALIBARAION_SAMPLE_TIMES;
  val = val / RO_CLEAN_AIR_FACTOR;
  return val;
}

float MQRead(int mq_pin) {
  float rs=0;
 
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs / READ_SAMPLE_TIMES;
 
  return rs;  
}

long MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if ( gas_id == GAS_MQ2LPG ) {
     return MQGetPercentage(rs_ro_ratio, MQ2LPGCurve);
  } else if ( gas_id == GAS_MQ2CO ) {
     return MQGetPercentage(rs_ro_ratio, MQ2COCurve);
  } else if ( gas_id == GAS_MQ2SMOKE ) {
     return MQGetPercentage(rs_ro_ratio, MQ2SmokeCurve);
     
  } else if ( gas_id == GAS_MQ4LPG ) {
     return MQGetPercentage(rs_ro_ratio, MQ4LPGCurve);
  } else if ( gas_id == GAS_MQ4CH4 ) {
     return MQGetPercentage(rs_ro_ratio, MQ4CH4Curve);
     
  } else if ( gas_id == GAS_MQ7CO ) {
     return MQGetPercentage(rs_ro_ratio, MQ7COCurve);
  } else if ( gas_id == GAS_MQ7H2 ) {
     return MQGetPercentage(rs_ro_ratio, MQ7H2Curve);

  } else if ( gas_id == GAS_MQ8H2) {
     return MQGetPercentage(rs_ro_ratio, MQ8H2Curve);
  }
  return 0;
}
 
long MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10, (((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}

void doCalibration(){ 
  isCalibrating = true;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating MQ2");
  Ro2 = MQCalibration(PIN_MQ2_A);

  delay(1000);
  lcd.clear();
  lcd.print("Calibrating MQ4");
  Ro4 = MQCalibration(PIN_MQ4_A);
  
  delay(1000);
  lcd.clear();
  lcd.print("Calibrating MQ7");
  Ro7 = MQCalibration(PIN_MQ7_A);
  
  delay(1000);
  lcd.clear();
  lcd.print("Calibrating MQ8");
  Ro8 = MQCalibration(PIN_MQ8_A);

  isCalibrating = false;
  
  delay(1000);
  lcd.clear();
  lcd.print("Calibrated!");
  lcd.setCursor(0, 1);

  delay(1500);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("R2:");
  lcd.print(Ro2);
  lcd.setCursor(9, 0);
  lcd.print("R4:");
  lcd.print(Ro4);
  lcd.setCursor(0, 1);
  lcd.print("R7:");
  lcd.print(Ro7);
  lcd.setCursor(9, 1);
  lcd.print("R8:");
  lcd.print(Ro8);
  
  delay(3000);
  
  wasCalibrated = true;
}

void vTaskLCD(void *pvParameters){
  (void) pvParameters;

  for(;;){
    if(isCalibrating) {
      vTaskDelay(xDelayMS1000);
      continue;
    } else if (wasCalibrated) {
      lcd.clear();   
      lcd.setCursor(0, 0);
      lcd.print("LPG:");
      lcd.print(usPPM_MQ2_LPG / 2 + usPPM_MQ4_LPG / 2);

      lcd.setCursor(6, 0);
      lcd.print("SMK:");
      lcd.print(usPPM_MQ2_Smoke);

      lcd.setCursor(0, 1);
      lcd.print("CH4:");
      lcd.print(usPPM_MQ4_CH4);
      
      lcd.setCursor(6, 1);
      lcd.print("H2:");
      lcd.print(usPPM_MQ7_H2 / 2 + usPPM_MQ8_H2 / 2);
   
      lcd.setCursor(11, 1);
      lcd.print("CO:");
      lcd.print(usPPM_MQ2_CO / 2 + usPPM_MQ7_CO / 2);
    }
    vTaskDelay(xDelayMS1000);
  }
  
  vTaskDelete(NULL);
}

unsigned long timer = 0;
int ledValue = 125;

void vTaskMQ2(void *pvParameters){
  digitalWrite(PIN_LED_STATUS_TASK_MQ2, HIGH);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  Serial.begin(9600);
  for(;;){
    if(isCalibrating) {
      vTaskDelayUntil(&xLastWakeTime, xDelayMS1000);
      continue;
    } else if (wasCalibrated) {
      taskENTER_CRITICAL();
      usPPM_MQ2_LPG    = MQGetGasPercentage(MQRead(PIN_MQ2_A)  / Ro2, GAS_MQ2LPG);
      usPPM_MQ2_CO     = MQGetGasPercentage(MQRead(PIN_MQ2_A)  / Ro2, GAS_MQ2CO);
      usPPM_MQ2_Smoke  = MQGetGasPercentage(MQRead(PIN_MQ2_A)  / Ro2, GAS_MQ2SMOKE); 
      Serial.print(pcMQ2Prefix);
      Serial.print(usPPM_MQ2_CO + usPPM_MQ2_LPG + usPPM_MQ2_Smoke);
      Serial.println(pcMQPostfix);
      taskEXIT_CRITICAL();
    }
    vTaskDelayUntil(&xLastWakeTime, xDelayMS3000);
  }

  digitalWrite(PIN_LED_STATUS_TASK_MQ2, LOW);
  vTaskDelete(NULL);
}

void vTaskMQ4(void *pvParameters){
  vTaskDelay(xDelayMS1000);
  
  digitalWrite(PIN_LED_STATUS_TASK_MQ4, HIGH);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  Serial.begin(9600);
  for(;;){
    if(isCalibrating) {
      vTaskDelayUntil(&xLastWakeTime, xDelayMS1000);
      continue;
    } else if (wasCalibrated) {  
      taskENTER_CRITICAL();
      usPPM_MQ4_LPG  = MQGetGasPercentage(MQRead(PIN_MQ4_A)  / Ro4, GAS_MQ4LPG);
      usPPM_MQ4_CH4  = MQGetGasPercentage(MQRead(PIN_MQ4_A)  / Ro4, GAS_MQ4CH4);
      Serial.print(pcMQ4Prefix);
      Serial.print(usPPM_MQ4_LPG + usPPM_MQ4_CH4);
      Serial.println(pcMQPostfix);
      taskEXIT_CRITICAL();
    }

    vTaskDelayUntil(&xLastWakeTime, xDelayMS3000);
  }
  
  digitalWrite(PIN_LED_STATUS_TASK_MQ4, LOW);
  vTaskDelete(NULL);
}

void vTaskMQ7(void *pvParameters){
  vTaskDelay(xDelayMS1000);
  vTaskDelay(xDelayMS1000);
  
  digitalWrite(PIN_LED_STATUS_TASK_MQ7, HIGH);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  Serial.begin(9600);
  for(;;){
    if(isCalibrating) {
      vTaskDelayUntil(&xLastWakeTime, xDelayMS1000);
      continue;
    } else if (wasCalibrated) {
      taskENTER_CRITICAL();
      usPPM_MQ7_CO  = MQGetGasPercentage(MQRead(PIN_MQ7_A)  / Ro7, GAS_MQ7CO);
      usPPM_MQ7_H2  = MQGetGasPercentage(MQRead(PIN_MQ7_A)  / Ro7, GAS_MQ7H2);
      Serial.print(pcMQ7Prefix);
      Serial.print(usPPM_MQ7_CO + usPPM_MQ7_H2);
      Serial.println(pcMQPostfix);
      taskEXIT_CRITICAL();
    }

    vTaskDelayUntil(&xLastWakeTime, xDelayMS3000);
  }
  
  digitalWrite(PIN_LED_STATUS_TASK_MQ7, LOW);
  vTaskDelete(NULL);
}

void vTaskMQ8(void *pvParameters){
  vTaskDelay(xDelayMS1000);
  vTaskDelay(xDelayMS1000);
  vTaskDelay(xDelayMS1000);
  
  digitalWrite(PIN_LED_STATUS_TASK_MQ8, HIGH);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  Serial.begin(9600);
  for(;;){
    if(isCalibrating) {
      vTaskDelay(xDelayMS1000);
      continue;
    } else if (wasCalibrated) {
      taskENTER_CRITICAL();
      usPPM_MQ8_H2  = MQGetGasPercentage(MQRead(PIN_MQ8_A)  / Ro8, GAS_MQ8H2);
      Serial.print(pcMQ8Prefix);
      Serial.print(usPPM_MQ8_H2);
      Serial.println(pcMQPostfix);
      taskEXIT_CRITICAL();
    }
    vTaskDelayUntil(&xLastWakeTime, xDelayMS3000);
  }

  digitalWrite(PIN_LED_STATUS_TASK_MQ8, LOW);
  vTaskDelete(NULL);
}