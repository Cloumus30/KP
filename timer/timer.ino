#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>

// Init STM32 timer TIM1
STM32Timer ITimer0(TIM1);

void TimerHandler0(void){
//  if(Serial.available()){
//    Serial.println(Serial.readString());
//  }
Serial.println("intterupsi");
}

#define TIMER0_INTERVAL_MS  1000 //1s = 1000ms
unsigned long myTime;

void setup() {
  Serial.begin(9600);
  while(!Serial){
    
  }

//  interval in microsecs
  if(ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS*1000,TimerHandler0)){
    Serial.println("Starting ITimer0 Ok, millis()= " +String(millis()));
  }
  else{
    Serial.println("Can't Set ITimer0. Select another Freq or Timer");
  }
}
void loop() {
//  Serial.print("Time: ");
//  myTime = millis();
//
//  Serial.println(myTime); // prints time since program started
//  delay(1000);          // wait a second so as not to send massive amounts of data

Serial.println("Clooudias");
delay(500);
}
