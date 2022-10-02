#include <Pixy2.h>
#include <Servo.h>

//3 ultrasonic sensors
#define trigPin1 4
#define echoPin1 5
#define trigPin2 2
#define echoPin2 3
#define trigPin3 12
#define echoPin3 13

long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor;


Servo myservo; 
//DC CONTROLLERRR

const int ENA= 9; // PWM pin 6
const int in1 = 11;
const int in2 = 10;


// This is the main Pixy object 
Pixy2 pixy;



///COLOR SENSOOOOOOOOOOOOOOOOOOOOOOR IN



#include <Wire.h>
#include "Adafruit_TCS34725.h"
//#include <Servo.h>
//Servo myservo; 

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
int pos = 0;    // variable to store the servo position

// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

 
void setup()
{ 
  ///ULTRASONIIIIIIIIIIIIIIIIIIIIIIIIC IN





pinMode(trigPin1, OUTPUT);
pinMode(echoPin1, INPUT);
pinMode(trigPin2, OUTPUT);
pinMode(echoPin2, INPUT);
pinMode(trigPin3, OUTPUT);
pinMode(echoPin3, INPUT);
  
  //dc controlllereeeeeeeeeeeeeeer
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ENA, OUTPUT);

  
  Serial.begin(9600);
  Serial.print("Starting...\n");
  myservo.attach(8);
  pixy.init();





  ////////////////////////////COLOR SENSOOOOOOOOOOOOOOOOOOOOR IN

   if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
//    while (1); // halt!
  }

  // use these three pins to drive an LED
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
}

void slow() {
  analogWrite(ENA, 120);
}

void fast() {
  analogWrite(ENA, 255);
}
void stop1(){
  analogWrite(ENA, 0);
}
 
void loop()
{ 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  fast();
  myservo.write(30);
  //ULTRASONIIIIIIIICCC


  SonarSensor(trigPin1, echoPin1);
RightSensor = distance;
SonarSensor(trigPin2, echoPin2);
FrontSensor = distance;
SonarSensor(trigPin3, echoPin3);
LeftSensor = distance;

Serial.print(LeftSensor); //right
if(LeftSensor < 20 && LeftSensor > 10){
  Serial.print("right");
    Serial.print("dont turn");
  myservo.write(-90);
  //turn servo to the left
} 
else if(LeftSensor < 10){

    myservo.write(90);
  delay(200);
  myservo.write(0);
}
Serial.print(" - ");
Serial.print(FrontSensor); // left
if(FrontSensor < 30 && FrontSensor > 10){
  Serial.print("left\n");
  myservo.write(-180);
  
} else if(FrontSensor < 5) {
  Serial.print("dont turn\n");
//   myservo.write(90);
//  delay(200);
//  myservo.write(0);
}
Serial.print(" - ");
Serial.println(RightSensor);//front
//if(RightSensor < 20){
//  Serial.print("front");
//} 
}

void SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;

  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.print(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      
      pixy.ccc.blocks[i].print();
      if(pixy.ccc.blocks[i].m_signature == 3){
  Serial.print("red\n");
  slow();
  Serial.print(pixy.line.numVectors);
  Serial.print("hello\n");

//    myservo.attach(8);
    myservo.write(-90);
    delay(2000);
    myservo.write(30);
    fast();
//    myservo.detach();
  
} else if(pixy.ccc.blocks[i].m_signature == 4)
{
  slow();
//    myservo.attach(8); 
     myservo.write(-90);
    delay(2000);
    myservo.write(30);
//    myservo.detach();
fast();
  
  Serial.print("green\n");
 
}


  
    }

  }  





  /// COLOR SENSOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOR IN

  float red, green, blue;
  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));

//  Serial.print("\t");
//  Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
  Serial.print("\n");

//  uint16_t red, green, blue, clear;
//  
//  tcs.setInterrupt(false);  // turn on LED
//
//  delay(60);  // takes 50ms to read
//
//  tcs.getRawData(&red, &green, &blue, &clear);
//  
//  tcs.setInterrupt(true);  // turn off LED
//
//  Serial.print("C:\t"); Serial.print(int(clear)); 
//  Serial.print("R:\t"); Serial.print(int(red)); 
//  Serial.print("\tG:\t"); Serial.print(int(green)); 
//  Serial.print("\tB:\t"); Serial.print(int(blue));
//  Serial.println();


#if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(1, gammatable[(int)red]);
  ledcWrite(2, gammatable[(int)green]);
  ledcWrite(3, gammatable[(int)blue]);
#else
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
#endif

if(red > 70){
  slow();
  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
   myservo.write(90);
   delay(15); 
   myservo.write(30);// tell servo to go to position in variable 'pos'
        
   Serial.print("orange");
   fast();

}
else if(blue > 80){
  slow();
 myservo.write(-90);
  delay(15); 
 myservo.write(30);
 Serial.print("blue");// tell servo to go to position in variable 'pos'
  fast();
}
}
