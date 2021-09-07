#include <Wire.h>

int16_t acRawX,acRawY,acRawZ;
int16_t gRawX,gRawY,gRawZ;
//float accOfsetX,accOfsetY,accOfsetZ,gyrOfsetX,gyrOfsetY,gyrOfsetZ;
unsigned long waktu_lalu,waktu_sekarang;
float dt;
float gRoll,gPitch,roll,pitch;
int16_t n_calibrate=3000;

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

struct OfsetData ofsetDat1;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();

// Set up Sensor
  setMPU(0x68);
  delay(10);

//  Kalibrasi Sensor
  ofsetDat1 = calibrate(0x68);
  waktu_lalu=millis();
  
  Serial.begin(115200);
  while(!Serial){
    
  }
  Serial.println("Start");
}

void loop() {
  // put your main code here, to run repeatedly:
  
//  check apakah MPU6050 terkoneksi dengan baik
  i2cSensCheck(0x68,"MPU6050");
  
//    Mendapatkan Data mentah Akselerometer
    struct RawData rawAccDat;
    rawAccDat = getRawAcc(0x68);

    struct DataAngle acRes;
    acRes = getAccG(rawAccDat);

    float acCalibratedX = acRes.angleX - ofsetDat1.accOfsetX;
    float acCalibratedY = acRes.angleY - ofsetDat1.accOfsetY;
    float acCalibratedZ = acRes.angleZ + (1-ofsetDat1.accOfsetZ);
  
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
    rawGyrDat = getRawGyr(0x68);

    struct DataAngle gyrRes;
    gyrRes = getGyrDeg(rawGyrDat);

    float gyrCalibratedX = gyrRes.angleX - ofsetDat1.gyrOfsetX;
    float gyrCalibratedY = gyrRes.angleY - ofsetDat1.gyrOfsetY;
    float gyrCalibratedZ = gyrRes.angleZ - ofsetDat1.gyrOfsetZ;
    
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
      roll = acRoll*0.15+gRoll*0.85;
      pitch = acPitch*0.85+gPitch*0.15;

//    roll = 0.90*(roll+gRoll*dt)+0.10*acRoll;
//    pitch = 0.10*(pitch+gPitch*dt)+0.90*acPitch;
    
    Serial.print("Roll = ");
    Serial.print(roll);
    Serial.print(" Pitch = ");
    Serial.println(pitch);

//    Serial.print("RollCom = ");
//    Serial.print(roll);
//    Serial.print(" RollAcc = ");
//    Serial.print(acRoll);
//    Serial.print(" RollGyr = ");
//    Serial.println(gRoll);
    
    delay(10);  
    waktu_lalu = millis();  
}



//Fungsi untuk setup konfigurasi MPU6050
void setMPU(int address){
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
void i2cSensCheck(int address, String sensorName){
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
}

//Fungsi untuk dapat Accelero Raw Data
struct RawData getRawAcc(int address){
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
struct RawData getRawGyr(int address){
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
struct RollPitch getRollPitch(struct OfsetData ofsetDat)
  {
  struct RollPitch res;
  
  //    Mendapatkan Data mentah Akselerometer
    struct RawData rawAccDat;
    rawAccDat = getRawAcc(0x68);

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
    rawGyrDat = getRawGyr(0x68);

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
      res.roll = acRoll*0.15+gRoll*0.85;
      res.pitch = acPitch*0.85+gPitch*0.15;

      return res;
}

//Fungsi untuk kalibrasi MPU6050
struct OfsetData calibrate(int address){
  float accTempX,accTempY,accTempZ,gyrTempX,gyrTempY,gyrTempZ;
  
  for(int i=0; i<n_calibrate; i++){
//  check apakah MPU6050 terkoneksi dengan baik
    i2cSensCheck(address,"MPU6050");
  
//    Mendapatkan Data mentah Akselerometer
    struct RawData rawAccCal;
    rawAccCal = getRawAcc(address);

////  Mendapatkan Data hasil olahan akselerometer
    struct DataAngle acResCal;
    acResCal = getAccG(rawAccCal);
//    
    accTempX += acResCal.angleX;
    accTempY += acResCal.angleY;
    accTempZ += acResCal.angleZ;

//   mendapatkan data mentah gyroscope
    struct RawData rawGyrCal;
    rawGyrCal = getRawGyr(address);

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

  delay(3000);
  return res;
}
