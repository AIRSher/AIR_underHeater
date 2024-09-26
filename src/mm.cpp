 int time=0;
  time=constrain(time,0,1023);
  //setting up the power module
  if (CurTrmVl > NexTrmVal){
    time+=5;
  }
  else if(CurTrmVl < NexTrmVal){
    time-=5;
  }
  else if(CurTrmVl == NexTrmVal){
    time=time;
  }


  if(digitalRead(Z_C)){

    time = map(time,0,1023,50,9500);
    delayMicroseconds(time);
    digitalWrite(PWM,1);
    delayMicroseconds(20);
    digitalWrite(PWM,0);
  }


///////////////////////////////////////////////////////////////////////////////

int TERM(int T){
  T = analogRead(Termo);
  Serial.println(T);
  return (T);
}





int raw = 0;
float temp = 0;

void setup() {
    Serial.begin(9600);
    pinMode( A0, INPUT );
}

void loop() {
    raw = analogRead(A0);
    temp = ( raw/1023.0 )*5.0*1000/10;
    Serial.println(temp);
    delay(1000); 
}








float tempC; // определяем переменную для показаний температуры
int reading; // определяем переменную для считывания "сырых" данных с датчика
int tempPin = 0; // определяем контакт подключения датчика
 
void setup()
{
analogReference(INTERNAL);
}
 
void loop()
{
reading = analogRead(tempPin);
tempC = reading / 9.31;
}
Для калибровки потребуется иная прошивка:
int temp;    // освобождаем память для переменной "temp"
float grad; // освобождаем память для переменной "grad"

void setup() {
  pinMode(A0, INPUT); // сенсор LM35 подключим к аналоговому входу A0
  Serial.begin(9600);     // подключаем монитор порта
}

void loop() {
  temp = analogRead(A0); // переменная находится в интервале 0 - 1023
  grad = ( temp/1023.0 )*5.0*1000/10; // формулу можно изменять
  Serial.println(grad);              
      // выводим температуру на монитор

  delay(100); // ставим небольшую задержку
}



///////////////////////////////////////////////////////////////////
//Program read values from temperature sensor 'CurTrmVl'. 
//New values are entered using encoder 'NexTrmVal' 
//If CurTrmVl < NexTrmVl then open semmistor, else close.


#include <Arduino.h>

// pin announcement
#define Data 8
#define En 4
#define Latch_CL 7
#define Shift_Cl 6 
#define RST 5
#define Termo 0


//simbols announcement//
//numbers
int num[10]={
  0b00000010,//0
  0b10011111,//1
  0b00100100,//2 
  0b00001100,//3 
  0b10011000,//4 
  0b01001001,//5 
  0b01000000,//6 
  0b00011111,//7
  0b00000000,//8 
  0b00001000,//9 
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


//encoder announcement
const byte Z_C=9, PWM = 10;

//Timer announcemet
unsigned long previousMillis = 0; // will store last time LED was updated
const long interval = 1000;  // interval at which to blink (milliseconds)


volatile int NexTrmVal = 60; // specified value
volatile bool encFlag = 0;  // turn counter
volatile byte reset = 0, last = 0; 


void encIsr() {  
  byte state = (PIND & 0b1100) >> 2;  // D2 + D3
  if (reset && state == 0b11) {
    int prevCount = NexTrmVal;
    if (last == 0b10) NexTrmVal+=5;
    else if (last == 0b01) NexTrmVal-=5;
    if (prevCount != NexTrmVal) encFlag = 1;    
    reset = 0;
  }
  if (!state) reset = 1;
  last = state;
  
}


void setup(){
  //encoder pin modes
  attachInterrupt(0, encIsr, CHANGE);
  attachInterrupt(1, encIsr, CHANGE);
  
  //shiftR pin modes
  pinMode(Data,1);
  pinMode(En,1);
  pinMode(Latch_CL,1);
  pinMode(Shift_Cl,1);
  pinMode(RST,1);

  //shiift register setup
  digitalWrite(RST,1);
  digitalWrite(En,0);
  

  //semmister pin modes
  pinMode(Z_C, INPUT);
  pinMode(PWM, OUTPUT);

  Serial.begin(9600);
}



void loop() {
  
  unsigned long currentMillis = millis();
  int CurTrmVl; //current termometr value 
  

  if (currentMillis - previousMillis >= interval) {
  previousMillis = currentMillis;
  

  CurTrmVl = analogRead(Termo);
  CurTrmVl = map(CurTrmVl,0,1023,-15,500);
  
  }

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
   // Проверяем, истек ли интервал задержки
    if (delayFlag && (currentMillis - previousMillis >= interval)) {
        delayFlag = false; // сбрасываем флаг задержки после завершения интервала
    }


  byte hundreds = T / 100;           // Get hundred counts
  byte tens = (T/ 10) % 10;         // Get tens counts
  byte units = T % 10;               // Get unit counts

  byte AndOrd;//anodes number/order
  byte Digit;//rank
  
  Serial.println(T);

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
    delay(2); // delay for effect seing 
  
    shiftOut(Data, Shift_Cl, LSBFIRST,zr[AndOrd]);
    shiftOut(Data, Shift_Cl, LSBFIRST, num[Digit]);
  
  
    digitalWrite(Latch_CL, LOW); // Reset the latch
    if(AndOrd==3){
      AndOrd=0;
    
    }
  }

}

