#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>
#include <ZMPT101B.h>

#define SENSITIVITY 500.0f
#define RFM95_CS 53
#define RFM95_RST 4
#define RFM95_INT 2
#define RF95_FREQ 868.0

#define CLIENT_ADDRESS 1  // mao ni siya ang aqms central system
#define SERVER_ADDRESS 2  // mao ni sya ang node sensor ( ESP32 )

// Water Flow Meter
//const int Flow = 3;
byte sensorInterrupt = 1;  // 0 = digital pin 2
byte flow_sensorpin = 3;
float calibrationFactor = 4.5;
volatile byte pulseCount;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long flowmeter_startmillis;
const unsigned long flow_period = 1000;  //blink period
unsigned long currentMillis;
float flow_data;
//***************************************************
// water level
unsigned long waterlevel1_startmillis;
unsigned long waterlevel2_startmillis;
const unsigned long waterlevel1_period = 1000;  //blink period
const unsigned long waterlevel2_period = 1300;  //blink period
int waterLevel;
int waterLevel2;
// **************************************************
unsigned long phmeter_startmillis;
const unsigned long phmeter_period = 1600;
float pHValue;
float Voltage; //*******
int sensorValue;
double m = 16.67;
double n= 24.51;
//***************************************************
unsigned long lora_startmillis;
const unsigned long lora_period = 2000;
//**************************************************
unsigned long voltage_sensormillis;
const unsigned long voltagesensing_period = 2300;
float voltage;
//**************************************************
#define Trigger 7 // ultrasonic 1
#define Echo 8    //ultrasonic 1
#define RedLed 11
#define GreenLed 12
#define redLedP 5
#define greenLedP 6
#define ultraTrig 34 // ultrasonic 2
#define ultraEcho 35 // ultrasonic 2
#define pHPin A0
#define pumpPin 9
#define voltage_sensor A2
#define outlet_V1 A3
#define outlet_V2 A4
#define inverter_V1 A5
#define inverter_V2 A6
/*
  const int Trigger = 7;

  const int Echo = 8;
  const int RedLed = 11;
  const int GreenLed = 12;
  const int redLedP = 5;
  const int greenLedP = 6;
  const int ultraTrig = 34;
  const int ultraEcho = 35;
  const int pHPin = A0;
  const int pumpPin = 9;
*/

//volatile unsigned int pulseCount = 0;
//unsigned long flowRate = 0;
//unsigned long pHValue = 0;
//unsigned int flowMilliLitres = 0;
//unsigned long totalMilliLitres = 0;
//unsigned long previousMillis = 0;
//unsigned int interval = 1000;
LiquidCrystal_I2C lcd(0x27, 20, 4);
NewPing sonar(Trigger, Echo, 200);
NewPing sonar2(ultraTrig, ultraEcho, 200);

RH_RF95 driver(RFM95_CS, RFM95_INT);
RHDatagram manager(driver, SERVER_ADDRESS);

#define MAX_LEN 20
#define MIN_LEN 10
char delimiter = 'Z';
char C_DATA[MIN_LEN];
char D_DATA[MIN_LEN];
char E_DATA[MIN_LEN];
char DATA_TS[MAX_LEN] = {};
int num = 100;

ZMPT101B voltageSensor(voltage_sensor, 50.0);
void pulseCounter();

void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(flow_sensorpin, INPUT_PULLUP);
  pinMode(RedLed, OUTPUT);
  pinMode(GreenLed, OUTPUT);
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(redLedP, OUTPUT);
  pinMode(greenLedP, OUTPUT);
  pinMode(ultraTrig, OUTPUT);
  pinMode(ultraEcho, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(voltage_sensor, INPUT);
  pinMode(outlet_V1, OUTPUT);
  pinMode(outlet_V2, OUTPUT);
  pinMode(inverter_V1, OUTPUT);
  pinMode(inverter_V2, OUTPUT);

  Serial.begin(9600); //*****
pinMode(pHValue, INPUT); //******

  digitalWrite(pumpPin, HIGH);

  digitalWrite(outlet_V1, LOW);
  digitalWrite(outlet_V2, LOW);
  digitalWrite(inverter_V1, HIGH);
  digitalWrite(inverter_V2, HIGH);

  // water flow meter
  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  //  oldTime           = 0;
  flow_data = 0.0;
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

  waterLevel = 0;
  waterLevel2 = 0;

  pHValue = 0;
  sensorValue = 0;
  voltage = 0.0;
  lcd.setCursor(0, 0);
  lcd.print("Flow: ");

  lcd.setCursor(0, 1);
  lcd.print("TANK 1: ");

  lcd.setCursor(0, 2);
  lcd.print("pH Value: ");

  lcd.setCursor(0, 3);
  lcd.print("TANK 2: ");

  Serial.begin(115200);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!manager.init()) {
    //Serial.println("LoRa radio init failed");
  }
  //Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  while (!driver.setFrequency(RF95_FREQ)) {
    //Serial.println("setFrequency failed");
    //while (1)
    // ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  voltageSensor.setSensitivity(SENSITIVITY);

  flowmeter_startmillis = millis();
  waterlevel1_startmillis = millis();
  waterlevel2_startmillis = millis();
  phmeter_startmillis = millis();
  lora_startmillis = millis();
  voltage_sensormillis = millis();
}


void loop() {
  currentMillis = millis();
  if ((currentMillis - flowmeter_startmillis) >= flow_period)   // Only process counters once per second
  {
    detachInterrupt(sensorInterrupt);
    flowRate = ((1000.0 / (currentMillis - flowmeter_startmillis)) * pulseCount) / calibrationFactor;

    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    unsigned int frac;
    //Serial.print("Flow rate: ");
    //Serial.print(int(flowRate));  // Print the integer part of the variable
    //Serial.print(".");             // Print the decimal point
    frac = (flowRate - int(flowRate)) * 10;

    //Serial.print(frac, DEC) ;      // Print the fractional part of the variable
    //Serial.print("L/min");
    //Serial.print("  Current Liquid Flowing: ");             // Output separator
    //Serial.print(flowMilliLitres);
    //Serial.print("mL/Sec");

    //Serial.print("  Output Liquid Quantity: ");             // Output separator
    //Serial.print(totalMilliLitres);
    //Serial.println("mL");

    lcd.setCursor(0, 0);
    lcd.print("FlowRate: ");
    lcd.print(int(flowRate));
    lcd.print(" ml/m ");

    pulseCount = 0;

    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    flowmeter_startmillis = currentMillis;
  }

  if (currentMillis - waterlevel1_startmillis >= waterlevel1_period)  //test whether the period has elapsed
  {
    waterLevel = sonar.ping_cm();
    //Serial.println(waterLevel);
    if (waterLevel >= 20)  /// low water level okay
    {
      digitalWrite(RedLed, HIGH);
      digitalWrite(GreenLed, LOW);
      digitalWrite(pumpPin, LOW);  // water pump on
      //delay(5000);
      lcd.setCursor(0, 1);
      lcd.print("TANK 1: LOW!");
    } else {
      digitalWrite(RedLed, LOW);
      digitalWrite(GreenLed, HIGH);
      digitalWrite(pumpPin, HIGH);  // water pump off
      lcd.setCursor(0, 1);
      lcd.print("TANK 1: HIGH");
    }
    waterlevel1_startmillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }

  if (currentMillis - waterlevel2_startmillis >= waterlevel2_period)  //test whether the period has elapsed
  {
    waterLevel2 = sonar2.ping_cm();
    if (waterLevel2 >= 10)  // low level okay
    {
      digitalWrite(redLedP, HIGH);
      digitalWrite(greenLedP, LOW);
      lcd.setCursor(0, 3);
      lcd.print("TANK 2: LOW!");
    } else {
      digitalWrite(redLedP, LOW);
      digitalWrite(greenLedP, HIGH);
      lcd.setCursor(0, 3);
      lcd.print("TANK 2: HIGH");
    }
    waterlevel2_startmillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }

  if (currentMillis - phmeter_startmillis >= phmeter_period)  //test whether the period has elapsed
  {
   // sensorValue = analogRead(pHPin);
   // pHValue = map(sensorValue, 0, 1023, 0, 14) / 10.0; 
   sensorValue = analogRead(A0);
pHValue = sensorValue* (18 / 1023.0);
sensorValue = (m * Voltage) - n;

Serial.print(" PH Value: ");
Serial.println(pHValue);
delay(500);

    lcd.setCursor(10, 2);
    //lcd.print(pHValue, 2);
    lcd.print(pHValue);

    lcd.setCursor(0, 2);
    lcd.print("pHValue: ");
    if (pHValue < 6.0) {
      lcd.print(int(pHValue));
      lcd.print(" ACIDIC!");
    } else if (pHValue = 7.0) {
      lcd.print(int(pHValue));
      lcd.print(" SAFE!");
    } else {
      lcd.print(int(pHValue));
      lcd.print(" ALKALINE!");
    }

    phmeter_startmillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }

  if (currentMillis - voltage_sensormillis >= voltagesensing_period)  //test whether the period has elapsed
  {
    voltage = voltageSensor.getRmsVoltage();
    if (voltage > 190)
    {
      digitalWrite(outlet_V1, LOW);
      digitalWrite(outlet_V2, LOW);
      digitalWrite(inverter_V1, HIGH);
      digitalWrite(inverter_V2, HIGH);
    }
    else {
      digitalWrite(outlet_V1, HIGH);
      digitalWrite(outlet_V2, HIGH);
      digitalWrite(inverter_V1, LOW);
      digitalWrite(inverter_V2, LOW);
    }
    voltage_sensormillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }

  if (currentMillis - lora_startmillis >= lora_period)  //test whether the period has elapsed
  {
    Serial.println(waterLevel);
    Serial.println(waterLevel2);
    Serial.println(flowRate);
    Serial.println(pHValue);
    Serial.println(voltage);

    dtostrf(flowRate, 4, 2, C_DATA); /*Double converted to string*/
    dtostrf(pHValue, 4, 2, D_DATA); /*Double converted to string*/
    dtostrf(pHValue, 4, 2, E_DATA); /*Double converted to string*/

    Serial.println(C_DATA);
    Serial.println(D_DATA);
    Serial.println(E_DATA);

    //int convrt_flowrate = flowRate *
    memset(DATA_TS, 0, sizeof(DATA_TS));  // make sure that DATA is empty
    snprintf(DATA_TS, sizeof(DATA_TS), "%c%d%c%d%c%s%c%s%c%s%c", delimiter, waterLevel, delimiter, waterLevel2, delimiter, C_DATA, delimiter, D_DATA, delimiter, E_DATA, delimiter);

    Serial.println(DATA_TS);
    while (!manager.sendto((uint8_t *)DATA_TS, sizeof(DATA_TS), CLIENT_ADDRESS)) {
      //delay(1);
    }
    lora_startmillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
}

void pulseCounter() {
  pulseCount++;
}
