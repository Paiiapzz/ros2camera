#include <SPI.h>

#define ENC_IN A1 //interrupt
#define ENC_COUNT_REV 48 //value of ppr
#define FORWARDIN 1
#define BACKWARDIN 2
#define SAFTY_IN 3
#define FORWARDOUT 5
#define BACKWARDOUT 6 
#define SAFETY_OUT 7
#define NOISE_THRESHOLD 20 // Adjust this threshold as needed

byte address = 0x00;
int CS = 4;
int i = 0;
volatile long encoderValue = 0;
long currentMillis = 0;
long previousMillis = 0;
int interval = 500;
int rpm = 0;
float KmperHr = 0;
float DperR = 1.41/1000;
int stateFW = 0 ;
int stateBW = 0 ;
int statePedal = 0 ;
int adjust = 253 ;
int bit;
bool countENC = false ;
bool start = false ;
int previousState = LOW;
int currentState = LOW;

void setup() {
  pinMode(FORWARDIN, INPUT);
  pinMode(BACKWARDIN, INPUT);
  pinMode(SAFTY_IN, INPUT);
  pinMode(FORWARDOUT, OUTPUT);
  pinMode(BACKWARDOUT, OUTPUT);
  pinMode(SAFETY_OUT, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(ENC_IN, INPUT);
  SPI.begin();
  Serial.begin(9600);
  digitalPotWrite(0);
  previousMillis = millis();
}

void updateEncoder()
{
  currentState = analogRead(ENC_IN);

  if (currentState > 400 && previousState < 80) {
    encoderValue++; // Increment count on rising edge
  }
  previousState = currentState;
}

void loop() {
  updateEncoder();
  currentMillis = millis();
  // Serial.println(analogRead(ENC_IN));
  if(countENC){
  if (currentMillis - previousMillis> interval) {
      previousMillis = currentMillis;
      // stateFW = digitalRead(FORWARDIN);
      // stateBW = digitalRead(BACKWARDIN);
      // statePedal = digitalRead(SAFTY_IN);

      rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
  
      // Calculate Km/hr
      KmperHr = (float)((rpm*60)*DperR*2);
      Serial.print(i++);
      Serial.print(",");
      Serial.print(encoderValue);
      Serial.print(",");
      Serial.println(KmperHr);
   

      // Serial.print("Encoder :");
      // Serial.print(encoderValue);
      // Serial.print("\n");
      // Serial.print("rpm :");
      // Serial.print(rpm);
      // Serial.print("\n");
      // Serial.print("km/h :");
      // Serial.print(KmperHr);
      // Serial.print("\n");
      // Serial.print("\n");
      encoderValue = 0;
  }
    autospeed();
    // Serial.println(analogRead(A1));
  }
  else{
    digitalWrite(SAFETY_OUT, LOW);
    digitalWrite(FORWARDOUT, LOW);
    digitalPotWrite(0);
  }

  char inChar = Serial.read();
  if (inChar == 'c'){ //use for stop the car
    encoderValue=0;
    i=0;
    countENC = false;

  }
  if (inChar == 's'){ //use for start the car
    encoderValue=0;
    countENC = true;
    autospeed();
  }
  if (inChar == 'p'){ //use for set interval time
    encoderValue=0;
    digitalPotWrite(0);
    digitalWrite(SAFETY_OUT, HIGH);
    digitalWrite(FORWARDOUT,HIGH);
    delay(100);
    digitalPotWrite(20);
    delay(8000);
    digitalWrite(SAFETY_OUT, LOW);
    digitalWrite(FORWARDOUT,LOW);
    digitalPotWrite(0);
    Serial.print("encoderValue: ");
    Serial.println(encoderValue);
}
}


void autospeed(){ 
  digitalWrite(SAFETY_OUT, HIGH);
  digitalWrite(FORWARDOUT,HIGH);
  delay(50);
  digitalPotWrite(25); //set speed in binary value (0-255)
}
void runSpeed(float stateFW,float stateBW,float statePedal,float val)
{
  char inChar = Serial.read();
   if(statePedal == 1)
  {
    digitalWrite(SAFETY_OUT, HIGH);
    // digitalPotWrite(val);
    Serial.println("Pedal High");
       
  }
  else
  {
    digitalWrite(SAFETY_OUT, LOW);
    // digitalPotWrite(0);
  }
  if(stateFW == 1)
  {
     digitalWrite(FORWARDOUT, HIGH);
     Serial.println("FW High");
  }
  else
  {
    digitalWrite(FORWARDOUT, LOW);
  }
  if(stateBW == 1)
  {
     digitalWrite(BACKWARDOUT, HIGH);
     Serial.println("BW High");
  }
  else
  {
    digitalWrite(BACKWARDOUT, LOW);
  }
}

void digitalPotWrite(byte value){
    digitalWrite(CS, LOW);
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWrite(CS, HIGH);
}
