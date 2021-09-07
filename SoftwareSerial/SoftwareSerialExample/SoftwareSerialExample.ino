/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(PA10, PA9); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
//  Serial.begin(57600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
//
//
//  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  Serial1.begin(9600);
  while(!Serial1){
    
  }
  Serial1.println("Hello, world?");
  pinMode(PC13,OUTPUT);
}

void loop() { // run over and over
//  if (mySerial.available()) {
//    mySerial.print("serial");
//    mySerial.println(mySerial.available());
//    mySerial.println(mySerial.readString());
//  }
//  if (Serial.available()) {
//    mySerial.write(Serial.read());
//  }

//  Serial.println("cloud");
  digitalWrite(PC13,LOW);
  delay(500);
  digitalWrite(PC13,HIGH);
  Serial1.println("cloud2");
  delay(1000);
}
