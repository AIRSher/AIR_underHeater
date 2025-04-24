//Program read values from temperature sensor 'CurTrmVl'. 
//New values are entered using encoder 'NexTrmVal' 
//If CurTrmVl < NexTrmVl then open semmistor, else close.


#include <Arduino.h>

// pin announcement
#define Data 13
#define En 12
#define Latch_CL 11
#define Shift_Cl 10
#define RST 8
#define Termo 0

//encoder announcement
#define Z_C 2//pin of zero cross
#define PWM 3

#define MAX_PERIOD_16 (1000000UL * 1024UL / F_CPU * 65536UL)	// 4194304 (0.24 Гц) на 16 МГц
volatile int dimmer = 500;

//simbols announcement//
//numbers
int num[10]={
  0b00000011,//0
  0b10011111,//1
  0b00100101,//2 
  0b00001101,//3 
  0b10011001,//4 
  0b01001001,//5 
  0b01000001,//6 
  0b00011111,//7
  0b00000001,//8 
  0b00001001,//9 
  };

//letters
int let[6]={
  0b11100011,//L
  0b11110011,//I
  0b00010001,//A
  0b01001001,//S
  0b10000011,//U
  0b01110011//R


};
//anodes number
int zr[4]={
  0b10000000,// Zero anode
  0b01000000,// First anode
  0b00100000,// Second anode
  0b00010000, // Third anode
};




//Timer announcemet
unsigned long previousMillis = 0; // will store last time LED was updated
const long interval = 1000;  // interval at which to blink (milliseconds)


volatile int NexTrmVal = 60; // specified value of heater
volatile bool encFlag = 0;  // turn counter
volatile byte reset = 0, last = 0; 





void setup(){
  Serial.begin(9600);
  //encoder pin modes
  PCICR |= (1<<PCIE2);// Enable Pin Change Interrupt control register
  PCMSK2 |= (1<<PCINT22);// Selected  d6 as interrupt pin
  PCMSK2 |= (1<<PCINT21); // Selected  d5 as interrupt pin
  
  //attachInterrupt(0, encIsr, CHANGE);
  //attachInterrupt(1, encIsr, CHANGE);
  
  //shiftR pin modes
  pinMode(Data,1);
  pinMode(En,1);
  pinMode(Latch_CL,1);
  pinMode(Shift_Cl,1);
  pinMode(RST,1);

  //shift register setup
  digitalWrite(RST,1);
  digitalWrite(En,0);
  
  //triac pins setup
  pinMode(Z_C,INPUT_PULLUP); //setup ZeroCross pinMode
  pinMode(PWM,OUTPUT); // setup PWM pinMode
  
  // ==================INTERRUPT=====================
  EICRA = (1<<ISC01) | (1<<ISC00);// rising
  EIMSK |= (1<<INT0);// enable INT0 interrupts(PD2)
  
  TIMSK1 |= (1<<OCIE1A);  // enable OCR1A interrupt

  
}
ISR(PCINT2_vect){
  byte state = (PIND & 0b01100000) >> 5;  // D2 + D3
  if (reset && state == 0b11) {
    int prevCount = NexTrmVal;
    if (last == 0b01) NexTrmVal+=5;
    else if (last == 0b10) NexTrmVal-=5;
    if (prevCount != NexTrmVal) encFlag = 1;
    reset = 0;
  }
  if (!state) reset = 1;
  last = state;
}


void loop() {
    
  

  unsigned long currentMillis = millis();
  int CurTrmVl; //current termometr value 
  

  if (currentMillis - previousMillis >= interval) {
  previousMillis = currentMillis;
  

  CurTrmVl = analogRead(Termo);
  CurTrmVl = map(CurTrmVl,0,1023,0,400);
  //Serial.write(Termo);
  }
  
  
  //========== Dimmer value ===============
  if (NexTrmVal < CurTrmVl){
    dimmer+=5;
  }
  else if (NexTrmVal> CurTrmVl){
    dimmer-=5;
  }
  dimmer = constrain(dimmer, 500, 9300);
  
  




//====== Data selection for sending to display ========
  int T=0;
  bool delayFlag = false; // флаг задержки
  
  if (encFlag) {
    NexTrmVal = constrain(NexTrmVal, 0, 300); // верхняя и нижняя границы
    if (!delayFlag) { // если задержка не активирована
      T = NexTrmVal;
      delayFlag = true; // активируем задержку
      previousMillis = currentMillis; // сбрасываем таймер
    }
  encFlag=0;
  }
  else{
    T=CurTrmVl;
  }
   // Check if ends delay interval
  if (delayFlag && (currentMillis - previousMillis >= interval)) {
    delayFlag = false; // reset the flag of 
  }
  



//================ Sending value =================
  byte hundreds = T / 100;           // Get hundred counts
  byte tens = (T/ 10) % 10;         // Get tens counts
  byte units = T % 10;               // Get unit counts

  byte AndOrd;//anodes number/order
  byte Digit;//rank
  
  

  for(AndOrd=0; AndOrd<3;AndOrd++){

    if(AndOrd == 0){
      Digit=units;
    } 
    else if(AndOrd == 1){
      Digit=tens;
    }

    else if(AndOrd == 2){
      Digit=hundreds;
    }
  
    digitalWrite(Latch_CL, HIGH); // Data latches
    delay(1); // delay for effect seing 
  
    shiftOut(Data, Shift_Cl, LSBFIRST,zr[AndOrd]);
    shiftOut(Data, Shift_Cl, LSBFIRST, num[Digit]);
  
  
    digitalWrite(Latch_CL, LOW); // Reset the latch
    if(AndOrd==3){
      AndOrd=0;
    
  }}
  Serial.println(dimmer);
}



ISR(INT0_vect){
  digitalWrite(PWM,0); // switch off triac
//====================SET PERIOD===================
  uint32_t _period = dimmer; 
  _period = constrain(_period,1, MAX_PERIOD_16);
  uint32_t _cycles = F_CPU/1000000*_period;//calculations of nuber cycles per period
  uint8_t prescaler = 0x00;
  uint8_t divider = 0x00;

//==========Select prescaler=========
if(_cycles<65536UL){
  prescaler = 0x01;
  divider = 1UL; 
} else if (_cycles < 65536UL * 8) {
  prescaler = 0x02;
  divider = 8UL;
} else if (_cycles < 65536UL * 64) {
  prescaler = 0x03;
  divider = 64UL;
} else if (_cycles < 65536UL * 256) {
  prescaler = 0x04;
  divider = 256UL;
} else {
  prescaler = 0x05;
  divider = 1024UL;
}

uint16_t top = (_cycles < 65536UL * 1024 ? (_cycles / divider) : 65536UL) ;
TCCR1A = (TCCR1A & 0xF0);
TCCR1B = ((1 << WGM13) | (1 << WGM12) | prescaler);   // CTC mode + set prescaler
OCR1A = top - 1; // Set timer top
uint8_t clock = 0x00;
clock = (TCCR1B & 0x07); 
return (1000000UL / ((F_CPU / divider) / top));   // Return real timer period
}

ISR(TIMER1_COMPA_vect){
  digitalWrite(PWM,1);
  TCCR1B = (TCCR1B & 0xF8);
  TCNT1 = 0x00;
  
}



  
  
  
  
  
  
  












