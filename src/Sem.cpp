#include <Arduino.h>

const byte Z_C=2, PWM = 3;


void setup(){
    pinMode(Z_C, INPUT);
    pinMode(PWM, OUTPUT);

}

void loop(){
    
    if(digitalRead(Z_C)){
        int val = map(analogRead(0),0,1023,50,9500);
        delayMicroseconds(val);
        digitalWrite(PWM,1);
        delayMicroseconds(20);
        digitalWrite(PWM,0);


    } 
}