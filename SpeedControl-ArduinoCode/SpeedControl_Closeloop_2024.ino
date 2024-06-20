#include <SPI.h>

#define ENC_IN A1 //interrupt
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
int j = 0;
int ENC_COUNT_REV = 48;
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
int adjust_speed = 0, adjust_8bit =0 ; //set initial to 0 V
int bit;
bool countENC = false ;
bool start = false ;
int previousState = LOW;
int currentState = LOW;
float vall[11];
int count=0;
float currentSpeed,result,AvgSpeed;
float setSpeedd= 5.0;
float Kprop = 1,Kintegral = 0.00113,Kderiv =3.35;
float sigmaerror=0, dT,preverror,prev=0,adjust,prevadjust;

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
  if(countENC){
  if (currentMillis - previousMillis> interval) {
      previousMillis = currentMillis;
      stateFW = digitalRead(FORWARDIN);
      stateBW = digitalRead(BACKWARDIN);
      statePedal = 1;
      // if(encoderValue < 25){
      //   ENC_COUNT_REV = 34;
      // }
      // if(encoderValue >= 25 && encoderValue <=80){
      //   ENC_COUNT_REV = 0.414*encoderValue+24;
      // }
      // if(encoderValue > 80){
      //   ENC_COUNT_REV = 45;
      // }

      rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
  
      // Calculate Km/hr
      KmperHr = (float)(((rpm*60)*DperR)*2);
      averagefilter(KmperHr);
      j++;
      currentSpeed = KmperHr;

      if(j<20){
        AvgSpeed = KmperHr;
      }
      else{
        AvgSpeed = averagefilter(KmperHr);
      }
      Serial.print(j);
      Serial.print(",");
      Serial.print(encoderValue);
      Serial.print(",");
      Serial.print(KmperHr); //with out avg filter
      Serial.print(",");
      Serial.println(AvgSpeed); 

      adjust = SpeedControl(currentSpeed,setSpeedd,currentMillis,statePedal);
      if(adjust > 30){
        adjust = 30;
      }
      // Serial.println(adjust); 
      autospeed();
      encoderValue = 0;
  }
  }
  else{
    digitalWrite(SAFETY_OUT, LOW);
    digitalWrite(FORWARDOUT, LOW);
    digitalPotWrite(0);
  }

  char inChar = Serial.read();
  if (inChar == 'c'){
    encoderValue = 0;
    KmperHr = 0;
    j=0;
    preverror = 0;
    prevadjust = 0;
    countENC = false;
    prev=0;
    dT=0;
    sigmaerror= 0;
    currentMillis = 0;

  }
  if (inChar == 's'){
    encoderValue=0;
    countENC = true;
  }
  if (inChar == 'a'){
    encoderValue=0;
    digitalPotWrite(0);
    digitalWrite(SAFETY_OUT, HIGH);
    digitalWrite(FORWARDOUT,HIGH);
    delay(100);
    digitalPotWrite(30);
    delay(4000);
    digitalWrite(SAFETY_OUT, LOW);
    digitalWrite(FORWARDOUT,LOW);
    digitalPotWrite(0);
    Serial.print("encoderValue: ");
    Serial.println(encoderValue);
  }
}

float SpeedControl(float current,float set,float currentTime,float state)
{
    float error = set-current ; //unit km/hr
    // dT = currentTime - prev;
    dT = 500; 
    prev = currentTime;
    if(state == 1)
    {
      sigmaerror = sigmaerror+ (error*dT);
       
     adjust_speed = (error*Kprop)+(Kintegral*sigmaerror)+(Kderiv*((error-preverror)/dT)) ;
    if(error < 0.2 && error > -0.2 && preverror < 0.3 && preverror > -0.3)
     {
        adjust_speed = prevadjust;
     }
    }
    else
    {
      sigmaerror = 0;
      adjust_speed = 0;
      
    }
    // Serial.print("adjust_speed : ");
    // Serial.println(adjust_speed); 
    preverror = error;
    prevadjust = adjust_speed;
    adjust_8bit = 2.8438 * adjust_speed + 5.2142 ; //y = 2.8438x + 5.2142

    if(adjust_8bit <= 13){
      adjust_8bit = 13 ;
    }
    return adjust_8bit; 
}

void autospeed(){
  digitalWrite(SAFETY_OUT, HIGH);
  digitalWrite(FORWARDOUT, HIGH);
  delay(50);
  digitalPotWrite(adjust);
}

float averagefilter(float kmh)
{ 
  float sum = 0;
  if(count == 11)
  {
    count = 0;
  }
  else
  {  vall[count] = kmh;
  }
  for(int i = 0; i<11;i++)
  {
    sum += vall[i];
  }    
  result = sum/10;
  count++;
  return result;
  
}

void digitalPotWrite(byte value){
    digitalWrite(CS, LOW);
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWrite(CS, HIGH);
}
