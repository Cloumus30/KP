int FSR1Pin = PA0;
int FSR2Pin = PA5;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){
    
  }
  analogReadResolution(12);
  Serial.println("START");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Membaca sensor FSR (satuan bit)
  int FSR1 = analogRead(FSR1Pin);
//  int FSR2 = analogRead(FSR2Pin);
//delay(1);
  // Mengubah nilai bit FSR menjadi voltase
  float FSR1Vol = FSR1/4095.0*3.26;
//  float FSR2Vol = FSR2/1023.0*3.3;

  // Mengubah nilai FSR ke Resistansi (Ohm)     
  float FSR1Res = ((3.3-FSR1Vol)*10000.0)/FSR1Vol;
//  float FSR2Res = ((3.3-FSR2Vol)*10000.0)/FSR2Vol;

  // print di serial
  Serial.print("FSR1 bit: ");
  Serial.print(FSR1);
//  Serial.print(" FSR2 bit: ");
//  Serial.println(FSR2);
  Serial.print("  FSR1 Voltase: ");
  Serial.println(FSR1Vol);
//  Serial.print(" FSR2 Voltase: ");
//  Serial.print(FSR2Vol);
//  Serial.print("  FSR1 Resistan: ");
//  Serial.print(FSR1Res);
//  Serial.print(" FSR2 Resistan: ");
//  Serial.println(FSR2Res);

  delay(500);
  
}
