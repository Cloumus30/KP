#include <Wire.h>

int16_t acRawX,acRawY,acRawZ;
int16_t gRawX,gRawY,gRawZ;
//float accOfsetX,accOfsetY,accOfsetZ,gyrOfsetX,gyrOfsetY,gyrOfsetZ;
unsigned long waktu_lalu,waktu_sekarang;
float dt;
float gRoll,gPitch,roll,pitch;
int16_t n_calibrate=100;
int TCAADDR = 0x70;

//tipe data Raw accelero
struct RawData{
  int16_t rawX;
  int16_t rawY;
  int16_t rawZ;
};

//tipe Data tidak Raw
struct DataAngle{
  float angleX;
  float angleY;
  float angleZ;
};

//Tipe Data untuk Ofset kalibrasi
struct OfsetData{
  float accOfsetX;
  float accOfsetY;
  float accOfsetZ;

  float gyrOfsetX;
  float gyrOfsetY;
  float gyrOfsetZ;
};

//Tipe Data Roll pitch
struct RollPitch{
  float roll;
  float pitch;
};

struct OfsetData ofsetDat0;
struct OfsetData ofsetDat1;
struct OfsetData ofsetDat2;
struct OfsetData ofsetDat3;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  while(!Serial){
    
  }
  Serial.println("Start");
  
//  Check TCa
  tcaCheck();
// Set up Sensor
  setMPU(0x68,0);
  setMPU(0x68,1);
  setMPU(0x68,2);
  setMPU(0x68,3);
  delay(10);

//  Kalibrasi Sensor
  ofsetDat0 = calibrate(0x68,0);
  ofsetDat1 = calibrate(0x68,1);
  ofsetDat2 = calibrate(0x68,2);
  ofsetDat3 = calibrate(0x68,3);
  waktu_lalu=millis();
  
}

void loop() {
  // put your main code here, to run repeatedly:
//Check TCa
  tcaCheck();

  
//  check apakah MPU6050 terkoneksi dengan baik
  i2cSensCheck(0x68,"MPU6050 bus 0",0);
  i2cSensCheck(0x68,"MPU6050 bus 1",1);
  i2cSensCheck(0x68,"MPU6050 bus 2",2);
  i2cSensCheck(0x68,"MPU6050 bus 3",3);
  
  struct RollPitch mpu0;
  mpu0 = getRollPitch(ofsetDat0,0x68, 0);

  struct RollPitch mpu1;
  mpu1 = getRollPitch(ofsetDat1 ,0x68, 1);

  struct RollPitch mpu2;
  mpu2 = getRollPitch(ofsetDat2,0x68, 2);
//
  struct RollPitch mpu3;
  mpu3 = getRollPitch(ofsetDat3 ,0x68, 3);
    
//    Serial.print("Roll 1 = ");
//    Serial.print(mpu1.roll);
    Serial.print(" Pitch 1 = ");
    Serial.print(mpu0.pitch);

//    Serial.print("  Roll 2 = ");
//    Serial.print(mpu2.roll);
    Serial.print(" Pitch 2 = ");
    Serial.print(mpu1.pitch);

//    Serial.print("  Roll 3 = ");
//    Serial.print(mpu3.roll);
    Serial.print(" Pitch 3 = ");
    Serial.print(mpu2.pitch);
//
//    Serial.print("  Roll 4 = ");
//    Serial.print(mpu4.roll);
    Serial.print(" Pitch 4 = ");
    Serial.println(mpu3.pitch);
    
    delay(50);  
    waktu_lalu = millis();  
}

//Fungsi untuk cek koneksi Tca984
void tcaCheck(){
  Wire.beginTransmission(0x70);
  int16_t error = Wire.endTransmission();
  while(error!=0){
    Serial.println(error);
    Serial.print("Cek koneksi kabel tca984 ");
    delay(500);
    Wire.beginTransmission(0x70);
    error = Wire.endTransmission();
  }
}

//Fungsi untuk mengaktifkan bus tca984
void tcaselect (uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//Fungsi untuk setup konfigurasi MPU6050
void setMPU(int address, int busTca){
  tcaselect(busTca);
  Serial.println("Set MPU6050");
  //  Reset Sensor
  Wire.beginTransmission(address);
  Wire.write(0x68);
  Wire.write(0b00000111);
  Wire.endTransmission();

  //  Reset Sensor
  Wire.beginTransmission(address);
  Wire.write(0x68);
  Wire.write(0b00000111);
  Wire.endTransmission();
  
//  matikan sleep mode dari sensor
  Wire.beginTransmission(address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

//  DLPF config
  Wire.beginTransmission(address);
  Wire.write(0x1A);
  Wire.write(0b00000110);
  Wire.endTransmission();

//Accel Config for scale 2g
  Wire.beginTransmission(address);
  Wire.write(0x1C);
  Wire.write(0b00000110);
  Wire.endTransmission();

//Gyro Config for scale 250 deg/s
  Wire.beginTransmission(address);
  Wire.write(0x1B);
  Wire.write(0);
  Wire.endTransmission();

}

// Fungsi untuk cek Sensor di jalur I2C
void i2cSensCheck(int address, String sensorName, int busTca){
  tcaselect(busTca);
  Wire.beginTransmission(address);
  int16_t error = Wire.endTransmission();
  while(error!=0){
    Serial.println(error);
    Serial.print("Cek koneksi kabel sensor ");
    Serial.println(sensorName);
    delay(500);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
  }
//  Serial.println("nice MPU terdetek");
}

//Fungsi untuk dapat Accelero Raw Data
struct RawData getRawAcc(int address, int busTca){
  tcaselect(busTca);
  //    Mendapatkan Data mentah Akselerometer
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    Wire.endTransmission();

    Wire.requestFrom(address,6);
    int16_t acRawX = Wire.read() << 8 | Wire.read();
    int16_t acRawY = Wire.read() << 8 | Wire.read();
    int16_t acRawZ = Wire.read() << 8 | Wire.read();

    struct RawData res;
    res.rawX = acRawX;
    res.rawY = acRawY;
    res.rawZ = acRawZ;
    return res;
}

// Fungsi Untuk Dapat Gyroscope Raw Data
struct RawData getRawGyr(int address, int busTca){
  tcaselect(busTca);
  //    Mendapatkan Data mentah Akselerometer
    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission();

    Wire.requestFrom(address,6);
    int16_t gyrRawX = Wire.read() << 8 | Wire.read();
    int16_t gyrRawY = Wire.read() << 8 | Wire.read();
    int16_t gyrRawZ = Wire.read() << 8 | Wire.read();

    struct RawData res;
    res.rawX = gyrRawX;
    res.rawY = gyrRawY;
    res.rawZ = gyrRawZ;
    return res;
}

//Fungsi untuk akselerometer satuan g
struct DataAngle getAccG(struct RawData accRaw ){
    float acXres = accRaw.rawX/16384.0;
    float acYres = accRaw.rawY/16384.0;
    float acZres = accRaw.rawZ/16384.0;

    struct DataAngle res;
    res.angleX = acXres;
    res.angleY = acYres;
    res.angleZ = acZres;

    return res;
}

//Fungsi untuk ubah data gyroscope satuan deg/s
struct DataAngle getGyrDeg(struct RawData gyrRaw){
    float gXres = gyrRaw.rawX/131.0;
    float gYres = gyrRaw.rawY/131.0;
    float gZres = gyrRaw.rawZ/131.0;

    struct DataAngle res;
    res.angleX = gXres;
    res.angleY = gYres;
    res.angleZ = gZres;

    return res;
}

//Fungsi untuk dapat Data roll dan Pitch beserta filter
struct RollPitch getRollPitch(struct OfsetData ofsetDat, int address, int busTca)
  {
  struct RollPitch res;
  
  //    Mendapatkan Data mentah Akselerometer
    struct RawData rawAccDat;
    rawAccDat = getRawAcc(address,busTca);

    struct DataAngle acRes;
    acRes = getAccG(rawAccDat);

    float acCalibratedX = acRes.angleX - ofsetDat.accOfsetX;
    float acCalibratedY = acRes.angleY - ofsetDat.accOfsetY;
    float acCalibratedZ = acRes.angleZ + (1-ofsetDat.accOfsetZ);
  
    float acRoll = (atan2(acCalibratedX,acCalibratedZ)*57.2958);
    float acPitch = (atan2(acCalibratedY ,acCalibratedZ)*57.2958);
    
//    float acRoll = (atan2(acRes.angleX,acRes.angleZ)*180)/PI;
//    float acPitch = (atan2(acRes.angleY ,acRes.angleZ)*180)/PI;    
    
//    float x = (atan2(acXres,acZres)*180)/PI;
//    float x = atan2(acXres,acZres)*RAD_TO_DEG;
//    float y = atan2(acYres,acZres)*RAD_TO_DEG;
//    float z = atan2(acZres,acXres/acYres)*RAD_TO_DEG;

//   mendapatkan data mentah gyroscope
    struct RawData rawGyrDat;
    rawGyrDat = getRawGyr(address, busTca);

    struct DataAngle gyrRes;
    gyrRes = getGyrDeg(rawGyrDat);

    float gyrCalibratedX = gyrRes.angleX - ofsetDat.gyrOfsetX;
    float gyrCalibratedY = gyrRes.angleY - ofsetDat.gyrOfsetY;
    float gyrCalibratedZ = gyrRes.angleZ - ofsetDat.gyrOfsetZ;
    
    waktu_sekarang = millis();
    
    dt = (waktu_sekarang-waktu_lalu)/1000.00;
    gRoll = acRoll + gyrCalibratedY*dt;
    gPitch = acPitch + gyrCalibratedX*dt;

//    gRoll = gyrCalibratedY*dt;
//    gPitch = acPitch + gyrCalibratedX*dt;

//    gRoll = acRoll + gyrRes.angleX*dt;
//    gPitch = acPitch + gyrRes.angleY*dt;
//
////  Complement Filter
      res.roll = acRoll*0.85+gRoll*0.15;
      res.pitch = acPitch*0.85+gPitch*0.15;

      return res;
}

//Fungsi untuk kalibrasi MPU6050
struct OfsetData calibrate(int address, int busTca){
  float accTempX,accTempY,accTempZ,gyrTempX,gyrTempY,gyrTempZ;
  Serial.println("Kalibrasi Mulai");
  for(int i=0; i<n_calibrate; i++){
//  check apakah MPU6050 terkoneksi dengan baik
    i2cSensCheck(address,"MPU6050", busTca);
  
//    Mendapatkan Data mentah Akselerometer
    struct RawData rawAccCal;
    rawAccCal = getRawAcc(address, busTca);

////  Mendapatkan Data hasil olahan akselerometer
    struct DataAngle acResCal;
    acResCal = getAccG(rawAccCal);
//    
    accTempX += acResCal.angleX;
    accTempY += acResCal.angleY;
    accTempZ += acResCal.angleZ;

//   mendapatkan data mentah gyroscope
    struct RawData rawGyrCal;
    rawGyrCal = getRawGyr(address, busTca);

    struct DataAngle gyrResCal;
    gyrResCal = getGyrDeg(rawGyrCal);
//    
    gyrTempX += gyrResCal.angleX;
    gyrTempY += gyrResCal.angleY;
    gyrTempZ += gyrResCal.angleZ;
  }
  struct OfsetData res;
  res.accOfsetX = accTempX/n_calibrate;
  res.accOfsetY = accTempY/n_calibrate;
  res.accOfsetZ = accTempZ/n_calibrate;

  res.gyrOfsetX = gyrTempX/n_calibrate;
  res.gyrOfsetY = gyrTempY/n_calibrate;
  res.gyrOfsetZ = gyrTempZ/n_calibrate;

  delay(100);
  return res;
}
