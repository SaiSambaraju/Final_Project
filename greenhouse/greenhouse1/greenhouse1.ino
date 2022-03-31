#define analogRef_voltage 3.3

//----------------Mining setup-------------------------------------------
#pragma GCC optimize ("-Ofast")
/* For microcontrollers with custom LED pins, adjust the line below */
/* For 8-bit microcontrollers we should use 16 bit variables since the
difficulty is low, for all the other cases should be 32 bits. */
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
typedef uint16_t uintDiff;
#else
typedef uint32_t uintDiff;
#endif
// Arduino identifier library - https://github.com/ricaun
#include "uniqueID.h"
#include "sha1.h"

// Create globals
String lastblockhash = "";
String newblockhash = "";
String DUCOID = "";
uintDiff difficulty = 0;
uintDiff ducos1result = 0;
// 40+40+20+3 is the maximum size of a job
const uint16_t job_maxsize = 104;  
uint8_t job[job_maxsize];
//----------------------------------------------------------------------------

//sensor read pins
const int ldrpin = A3;
const int moisturesensor = A4;
const int tempPin = A0;
const int ultra_sens_trig = 7;
const int ultra_sens_echo = 8;

//effector pins
const int ledstrip = 6;
const int fanpin = 3;
const int pumppin = 5;

//dryest and wettest val of moisture sensor
const int dryest_val = 775;
const int wettest_val = 410;

//calibration variables 
int ldrvalue = 0; 
int ldrmin_value = 1023;
int ldrmax_value = 0;

//control variables for fan
int temperature;
int fan_same_temp_counter = 0;

//control variables for pump
bool pump_on = false;
bool pump_needs_on = false;
int moisture_value;
int moisture_percent;

//desired conditions
const int desired_temp = 25; 
const int desired_moisture_percent = 30;
bool water_container_empty = false;

//moisture threshold variable
const int moisture_low = 15; 

//---------------------------Mining procedures-------------------------------------------
uintDiff ducos1a(String lastblockhash, String newblockhash,
                 uintDiff difficulty) {
  newblockhash.toUpperCase();
  const char *c = newblockhash.c_str();
  uint8_t final_len = newblockhash.length() >> 1;
  for (uint8_t i = 0, j = 0; j < final_len; i += 2, j++)
    job[j] = ((((c[i] & 0x1F) + 9) % 25) << 4) + ((c[i + 1] & 0x1F) + 9) % 25;

    // Difficulty loop
  #if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
    // If the difficulty is too high for AVR architecture then return 0
    if (difficulty > 655) return 0;
  #endif
  for (uintDiff ducos1res = 0; ducos1res < difficulty * 100 + 1; ducos1res++) {
    Sha1.init();
    Sha1.print(lastblockhash + String(ducos1res));
    // Get SHA1 result
    uint8_t *hash_bytes = Sha1.result();
    if (memcmp(hash_bytes, job, SHA1_HASH_LEN * sizeof(char)) == 0) {
      // If expected hash is equal to the found hash, return the result
      return ducos1res;
    }
  }
  return 0;
}

String get_DUCOID() {
  String ID = "DUCOID";
  char buff[4];
  for (size_t i = 0; i < 8; i++) {
    sprintf(buff, "%02X", (uint8_t)UniqueID8[i]);
    ID += buff;
  }
  return ID;
}

void mine(){
    // Wait for serial data
  if (Serial.available() > 0) {
    memset(job, 0, job_maxsize);
    // Read last block hash
    lastblockhash = Serial.readStringUntil(',');
    // Read expected hash
    newblockhash = Serial.readStringUntil(',');
    // Read difficulty
    difficulty = strtoul(Serial.readStringUntil(',').c_str(), NULL, 10);
    // Clearing the receive buffer reading one job.
    while (Serial.available()) Serial.read();
    // Turn on the built-in led
    #if defined(ARDUINO_ARCH_AVR)
        PORTB = PORTB | B00100000;
    #else
        digitalWrite(LED_BUILTIN, LOW);
    #endif
    // Start time measurement
    uint32_t startTime = micros();
    // Call DUCO-S1A hasher
    ducos1result = ducos1a(lastblockhash, newblockhash, difficulty);
    // Calculate elapsed time
    uint32_t elapsedTime = micros() - startTime;
    // Turn on the built-in led
    #if defined(ARDUINO_ARCH_AVR)
        PORTB = PORTB & B11011111;
    #else
        digitalWrite(LED_BUILTIN, HIGH);
    #endif
    // Clearing the receive buffer before sending the result.
    while (Serial.available()) Serial.read();
    // Send result back to the program with share time
    Serial.print(String(ducos1result, 2) 
                 + "," 
                 + String(elapsedTime, 2) 
                 + "," 
                 + String(DUCOID) 
                 + "\n");
  }
}
//-------------------------------------------------------------------------



//calibrate function
void calibrate(){
  Serial.println("Calibrating");
  while(millis() < 5000){
    ldrvalue = analogRead(ldrpin);
    if(ldrvalue > ldrmax_value){
      ldrmax_value = ldrvalue;
    }
    if(ldrvalue < ldrmin_value){
      ldrmin_value = ldrvalue;
    }
  }
}

int get_temp_C(int temp){
  float voltage = temp * analogRef_voltage;
  voltage = voltage/1024;
  temp = (voltage - 0.5) * 100;
  return temp;
}

int get_moisture_perc(int moisture_raw){
  moisture_percent = map(moisture_raw, wettest_val, dryest_val, 100, 0);
  return moisture_percent;
}

int get_water_distance(){
    digitalWrite(ultra_sens_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(ultra_sens_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultra_sens_trig, LOW);
    int duration = pulseIn(ultra_sens_echo, HIGH);
    int distance = duration/58.2;
    return distance;
}

void setup() {
  pinMode(fanpin, OUTPUT);
  pinMode(pumppin, OUTPUT);
  pinMode(ledstrip, OUTPUT);
  pinMode(ultra_sens_trig, OUTPUT);
  pinMode(ultra_sens_echo, INPUT);
  calibrate();  //calibrate
  analogReference(EXTERNAL);
  //---------------------------------------
   DUCOID = get_DUCOID();
  // Open serial port
  Serial.begin(115200);         //mining setup
  Serial.setTimeout(10000);
  while (!Serial)
    ;  // For Arduino Leonardo or any board with the ATmega32U4
  Serial.flush(); 
  //---------------------------------------
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 62500;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect){  
  //Control system 1
  ldrvalue = analogRead(ldrpin);
  ldrvalue = map(constrain(ldrvalue, ldrmin_value, ldrmax_value), ldrmin_value, ldrmax_value, 255, 0);
  analogWrite(ledstrip, ldrvalue);
//  
//  //Control System 2
  temperature = get_temp_C(analogRead(tempPin));
  if(temperature > desired_temp and fan_same_temp_counter < 5){
    analogWrite(fanpin, 150);
    fan_same_temp_counter += 1;
  }
  else if(temperature > desired_temp and fan_same_temp_counter >= 5){
    analogWrite(fanpin, 255);
  }
  else{
    fan_same_temp_counter = 0;
    analogWrite(fanpin, 0);
  }
  
    //Control system 3
    moisture_percent = get_moisture_perc(analogRead(moisturesensor));
    int distance_to_water = get_water_distance();
    
    if (distance_to_water >= 9){
      water_container_empty = true;
      pump_needs_on = false;  
    }
    else{
      water_container_empty = false;
    }
    if(moisture_percent <= moisture_low and water_container_empty == false){
      pump_needs_on = true;  
    }
    else if(moisture_percent >= desired_moisture_percent){
      pump_needs_on = false;
      pump_on = false;
    }
   if(pump_needs_on){
    if(pump_on == false){
      analogWrite(pumppin, 200);
      pump_on = true;
      }
    else{
      analogWrite(pumppin, 0);
      pump_on = false;
      }
    }
    else{
      analogWrite(pumppin, 0);
      pump_on = false;
    }
}

void loop() {
  mine();
}
