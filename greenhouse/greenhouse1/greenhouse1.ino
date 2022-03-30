#define analogRef_voltage 3.3

//global variables

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


int temperature = 0;
int fan_same_temp_counter = 0;
int moisture_value;
int moisture_percent;

//control variables for pump
bool pump_on = false;
bool pump_needs_on = false;

//desired conditions
const int desired_temp = 25; 
const int desired_moisture_percent = 30;

//moisture threshold variable
const int moisture_low = 15; 


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

void setup() {
  pinMode(fanpin, OUTPUT);
  pinMode(pumppin, OUTPUT);
  pinMode(ledstrip, OUTPUT);
  pinMode(ultra_sens_trig, OUTPUT);
  pinMode(ultra_sens_echo, INPUT);
  calibrate();  //calibrate
  Serial.begin(9600);
  analogReference(EXTERNAL);
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
  Serial.println("Hello this is an interrupt!!");
  
  //Control system 1
//  ldrvalue = analogRead(ldrpin);
//  ldrvalue = map(constrain(ldrvalue, ldrmin_value, ldrmax_value), ldrmin_value, ldrmax_value, 255, 0);
//  analogWrite(ledstrip, ldrvalue);
//  
//  //Control System 2
//  temperature = get_temp_C(analogRead(tempPin));
//  Serial.println(temperature);
//  if(temperature > desired_temp and fan_same_temp_counter < 5){
//    analogWrite(fanpin, 150);
//    fan_same_temp_counter += 1;
//  }
//  else if(temperature > desired_temp and fan_same_temp_counter >= 5){
//    Serial.println("Fan is at 255");
//    analogWrite(fanpin, 255);
//  }
//  else{
//    fan_same_temp_counter = 0;
//    analogWrite(fanpin, 0);
//  }
  
  //Control system 3
//  moisture_percent = get_moisture_perc(analogRead(moisturesensor));
//  Serial.println(moisture_percent);
    digitalWrite(ultra_sens_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(ultra_sens_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultra_sens_trig, LOW);
    long duration = pulseIn(ultra_sens_echo, HIGH);
    long distance = duration/58.2;
    Serial.println(distance);
//  if(moisture_percent <= moisture_low){
//      pump_needs_on = true;  
//  }
//  else if(moisture_percent >= desired_moisture_percent){
//      pump_needs_on = false;
//      pump_on = false;
//  }
//  if(pump_needs_on){
//   if(pump_on == false){
//      Serial.println("pump turning on");
//      analogWrite(pumppin, 200);
//      pump_on = true;
//    }
//    else{
//      analogWrite(pumppin, 0);
//      pump_on = false;
//    }
//  }
}

void loop() {
  //add mining code
}
