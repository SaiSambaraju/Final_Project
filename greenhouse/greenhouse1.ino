#define analogRef_voltage 3.3
#define ledPin 13

const int ldrpin = A3; //pins
const int ledstrip = 11;
const int fanpin = 6;
const int tempPin = A0;
const int pumppin = 5;
const int moisturesensor = A4;
const int dryest_val = 775;
const int wettest_val = 410;

//calibration variables 
int ldrvalue = 0; //global varaibles
int ldrmin_value = 1023;
int ldrmax_value = 0;
int temperature = 0;
int fan_same_temp_counter = 0;
int moisture_value;
int moisture_percent;
int count = 0;
bool pump_on = false;

const int desired_temp = 25; 
const int desired_moisture_percent = 30;

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
  // put your setup code here, to run once:
  
  //calibrate();  //calibrate
  pinMode(fanpin, OUTPUT);
  analogWrite(fanpin, 255);
  Serial.begin(9600);
  analogReference(EXTERNAL);
  attachInterrupt(digitalPinToInterrupt(2), counter, RISING);
//  noInterrupts();
//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCNT1 = 0;
//  OCR1A = 62500;
//  TCCR1B |= (1 << WGM12);
//  TCCR1B |= (1 << CS12) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  interrupts();
}

void counter(){
  count++;
}

ISR(TIMER1_COMPA_vect){
  Serial.println("Hello this is an interrupt!!");
  //Control system 1
  ldrvalue = analogRead(ldrpin);
  ldrvalue = map(constrain(ldrvalue, ldrmin_value, ldrmax_value), ldrmin_value, ldrmax_value, 255, 0);
  analogWrite(ledstrip, ldrvalue);
  
  //Control System 2
//  temperature = get_temp_C(analogRead(tempPin));
//  Serial.println(temperature);
//  if(temperature > desired_temp and fan_same_temp_counter < 5){
////    Serial.println("Fan is at 150!!!");
//    analogWrite(fanpin, 150);
//    fan_same_temp_counter += 1;
//  }
//  else if(temperature > desired_temp and fan_same_temp_counter >= 5){
//    analogWrite(fanpin, 255);
//  }
//  else{
//    Serial.println("Fan is at 150!!!");
//    fan_same_temp_counter = 0;
//    digitalWrite(fanpin, LOW);
//  }
  
  //Control system 3
//  moisture_percent = get_moisture_perc(analogRead(moisturesensor));
//  Serial.println(moisture_percent);
//  if(moisture_percent < desired_moisture_percent and pump_on == false){
//    analogWrite(pumppin, 200);
//    pump_on = true;
//  }
//  else{
//    analogWrite(pumppin, 0);
//    pump_on = false;
//  }
}

void loop() {
//  delay(1000); //can be anything
  analogWrite(fanpin, 255);
}
