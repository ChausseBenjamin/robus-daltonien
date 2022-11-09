#include <Wire.h>
#include "Adafruit_TCS34725.h"
 #include <SoftwareSerial.h>
  #include <SPI.h>
  #include<LibRobus.h>

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define RPIN 12
#define YPIN 2
#define GPIN 5
#define BPIN 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to groundé

// set to false if using a common cathode LED
#define commonAnode false

// our RGB -> eye-recognized gamma color
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void color(int red, int blue,  int green);
void turnoff();

void color(int red, int blue,  int green){

    if(red<blue && red>green){
        turnoff();
          digitalWrite(RPIN,HIGH);
        Serial.println("rouge");
       
        //pinMode(RPIN,OUTPUT);
        
    }
    else if(green>1.6*red && green<blue){
      turnoff();
     digitalWrite(GPIN,HIGH);
     Serial.println("vert");
    }

     else if(blue>1.7*red && blue>1.3*green){
         turnoff();
          digitalWrite(BPIN,HIGH);
     Serial.println("bleu");
    }
     else if(red>1.3*blue && red>green){
         turnoff();
          digitalWrite(YPIN,HIGH);
     Serial.println("jaune");
    }
    else if(){
        Serial.println("noir");
    }
    else{
       Serial.println("inconnu");  
    }
}

void turnoff()
    {
        digitalWrite(RPIN,LOW);
        digitalWrite(GPIN,LOW);
        digitalWrite(BPIN,LOW);
        digitalWrite(YPIN,LOW);

    }
