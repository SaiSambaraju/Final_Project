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
//----------------Mining setup------------------------------------------------

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
//---------------------Mining procedures------------------------------------



//calibrate function
void calibrate(){
  //calibrates for the Leds, to give a known range for their brightness to operate in
  // Serial.println("Calibrating");
  //loop for 5 seconds 
  while(millis() < 5000){
    //record the highest and lowest values encountered in calibration 
    ldrvalue = analogRead(ldrpin);
    if(ldrvalue > ldrmax_value){
      ldrmax_value = ldrvalue;
    }
    if(ldrvalue < ldrmin_value){
      ldrmin_value = ldrvalue;
    }
  }
  //- NOTE the user will need to shine a light source on ldr and cover it to give it a good range, for proper function.
}

//returns a temperature in celcius 
int get_temp_C(int temp){
  //using the analog reference voltage for extra precision
  float voltage = temp * analogRef_voltage;
  voltage = voltage/1024;
  temp = (voltage - 0.5) * 100; //conversion to celcius
  return temp; //returns temperature
}


//maps the raw moisture value to a percentage, using the precomputed wettest and dryest values
int get_moisture_perc(int moisture_raw){
  moisture_percent = map(moisture_raw, wettest_val, dryest_val, 100, 0);
  return moisture_percent;
}


//function for calculating water distance from ultrasonic sensor
int get_water_distance(){

    digitalWrite(ultra_sens_trig, LOW);//set the trigger to low
    delayMicroseconds(2);//wait 

    digitalWrite(ultra_sens_trig, HIGH);//send a signal through trigger
    delayMicroseconds(10);//for 10 microseconds

    digitalWrite(ultra_sens_trig, LOW);//then make signal low

    int duration = pulseIn(ultra_sens_echo, HIGH);//work out the travel time from signal at echo pin
    int distance = duration/58.2; //calulate distance

    return distance; //returns the distance
}

void setup() {
  //setting the pin modes for the pins being used
  pinMode(fanpin, OUTPUT);
  pinMode(pumppin, OUTPUT);
  pinMode(ledstrip, OUTPUT);
  pinMode(ultra_sens_trig, OUTPUT);
  pinMode(ultra_sens_echo, INPUT);
  calibrate();  //calibrate before since millis requires interrupts
  analogReference(EXTERNAL); //reference the 3.3v from the arduino as a reference voltage for extra percision

  //---------------------------------------
   DUCOID = get_DUCOID();
  // Open serial port
  Serial.begin(115200);         //mining setup
  Serial.setTimeout(10000);
  while (!Serial)
    ;  // For Arduino Leonardo or any board with the ATmega32U4
  Serial.flush(); 
  //---------------------------------------

  noInterrupts(); //disable interrupts
  TCCR1A = 0; //clear these registers for timer 1 
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 62500;  //preload the compare register
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);  //enable tiemr compare interrupts
  TIMSK1 |= (1 << OCIE1A);
  interrupts(); //enable interrupts
}


//Interrupt service routine
ISR(TIMER1_COMPA_vect){  

  //--------------Control system 1---------- LED ------------------------
  //reads the LDR pin and maps the output signal between the calibrated maximum and minumum value
  ldrvalue = analogRead(ldrpin); 
  ldrvalue = map(constrain(ldrvalue, ldrmin_value, ldrmax_value), ldrmin_value, ldrmax_value, 255, 0);
  analogWrite(ledstrip, ldrvalue);


  //--------------Control System 2---------- FAN ------------------------
  //reads the temperature from the sensor and converts it to celcius
  temperature = get_temp_C(analogRead(tempPin));

  //if its greater than required, the fan is turned on, at a low speed
  if(temperature > desired_temp and fan_same_temp_counter < 5){
    analogWrite(fanpin, 150);
    fan_same_temp_counter += 1;
  }//if the temperature does not cool down for 5 consequetive interrupts, the fan is span faster
  else if(temperature > desired_temp and fan_same_temp_counter >= 5){
    analogWrite(fanpin, 255);
  }
  else{
    //once the temperature reaches desired level, the fan is turned off
    fan_same_temp_counter = 0;
    analogWrite(fanpin, 0);
  }
  
    //------------Control system 3---------- PUMP -----------------------
    //get moisture percentage from reading the sensor
    moisture_percent = get_moisture_perc(analogRead(moisturesensor));

    //use the ultrasonic sensor to get the level of water
    int distance_to_water = get_water_distance();
    
    //If there is no water in the container then we don't need to turn the pump on
    if (distance_to_water >= 9){
      water_container_empty = true;
      pump_needs_on = false;  
    }
    else{
      water_container_empty = false;
    }
    //if moisture percentage less than the threshold moisture variable we tell the pump it needs to turn on
    if(moisture_percent <= moisture_low and water_container_empty == false){
      pump_needs_on = true;  
    }
    else if(moisture_percent >= desired_moisture_percent){
      //the pump will need to turn on until the desired moisture level is reached
      pump_needs_on = false; 
      pump_on = false;
    }
   //The pump signal alternates each interrupt cycle to allow water to be absorbed by soil
   if(pump_needs_on){
    //if the pump was off in the previous interrupt cycle it is set to on
    if(pump_on == false){
      analogWrite(pumppin, 200);
      pump_on = true;
      }
    else{
      //if the pump was on in the pervious interrupt cycle, it is set to off, to allow water to seep through soil and be absorbed completely
      analogWrite(pumppin, 0);
      pump_on = false;
      }
    }
    else{
      //if the pump doesn't need to turn on, the state should be reset to off.
      analogWrite(pumppin, 0);
      pump_on = false;
    }
}

void loop() {
  mine(); //mine inbetween the interrupts
}
