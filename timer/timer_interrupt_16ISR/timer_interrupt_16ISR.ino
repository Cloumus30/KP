#include <SoftwareSerial.h>

#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>


//Init STM32 Timer TIM1;
STM32Timer ITimer(TIM1);

//Init STM32_ISR_Timer
//Each STM32_ISR_Timer can service 16 different ISR based timers
STM32_ISR_Timer ISR_Timer;

//Init Software Serial
SoftwareSerial mySerial(PA10,PA9); //RX,TX

void TimerHandler(void){
  ISR_Timer.run();
}

#define HW_TIMER_INTERVAL_US  100L

#define TIMER_INTERVAL_2S     200L
#define TIMER_INTERVAL_1S     1000L
#define TIMER_INTERVAL_4S   4000L

void doingSomething2s(){
  if(Serial1.available()){
    String inp = Serial1.readString();
    Serial1.print("Input: " + inp);
  }
}

void doingSomething1s(){
  Serial1.println("1 detik");
}

void doingSomething4s(){
  Serial1.println("4 detik");
}

void setup()
{
// Set Baudrate
Serial1.begin(9600);
Serial1.println("Hello World");
  delay(10);
//  Interval in microsecs
if(ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler)){
  Serial1.println("Starting ITimer Ok, millis() = "+String(millis()));
}
else{
  Serial1.println("can't set ITimer correctly, Select Another Freq or Interval");
}


  ISR_Timer.setInterval(TIMER_INTERVAL_2S, doingSomething2s);
  ISR_Timer.setInterval(TIMER_INTERVAL_1S, doingSomething1s);
  ISR_Timer.setInterval(TIMER_INTERVAL_4S, doingSomething4s);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
