#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include<Wire.h>

#define MPU_addr 0x68
#define DEG_CONVERT 57.2957786

double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;               //Raw data cua MPU6050
uint32_t timer;                                         // Khoi tao timer
int pd = 3;
int senRead = 0;
double compAngleX, compAngleY, compAngleZ;              //angle sau khi dung complementary filter

//Khai bao dia chi, store data NRF
RF24 nrf(9, 10); // CNS, CE
const uint64_t pipe = 0xE8E8F0F0E1LL;
double data[10];
//---------------------------------------------------------------------------------
void setupMPU() {
  //MPU power config
  Wire.beginTransmission(MPU_addr);                     //Bat dau truyen voi dia chi cua MPU6050
  Wire.write(0x6B);                                     //Vao thanh ghi PWR_MGMT_1 (Power Management 1)
  Wire.write(0b00000000);                                //Set tat ca bit cua thanh ghi thanh 0 de "wake up" MPU6050
  Wire.endTransmission();

  //Gyro configuration
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);                                      //Vao thanh ghi GYRO_CONFIG
  Wire.write(0b00000000);                                //Thiet lap scale range la +/-250 (deg/s)
  Wire.endTransmission();

  //Accel configuration
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);                                       //Vao thanh ghi ACCEL_CONFIG
  Wire.write(0b00000000);                                 //Thiet lap scale range la +/-2g
  Wire.endTransmission();
}
//---------------------------------------------------------------------------------
void starting_angle() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                                       //Bat dau voi thanh ghi ACCEL_XOUT_H)
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 14, true);                   //goi tat ca 14 thanh ghi

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  //Tinh roll va pitch
  double roll = atan2(AcY, AcZ) * (DEG_CONVERT);
  double pitch = atan2(-AcX, AcZ) * (DEG_CONVERT);

  double gyroXangle = roll;
  double gyroYangle = pitch;
 
  double compAngleX = roll;
  double compAngleY = pitch;
  //double compAngleZ = 0;

  delay(500);
  timer = micros();
}
//---------------------------------------------------------------------------------
void Comp_filter() {
  //Loc raw data cua MPU dung thuat toan Complementary filter
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                                       //Bat dau voi thanh ghi ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);                   //goi tat ca 14 thanh ghi
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  double roll = atan2(AcY, AcZ) * (DEG_CONVERT);
  double pitch = atan2(-AcX, AcZ) * (DEG_CONVERT);

  //Chuyen raw data sang deg/s. (131 la` do nhay)
  double gyroXrate = GyX / 131.0;
  double gyroYrate = GyY / 131.0;
 //ouble gyroZrate = GyZ / 131.0;

  compAngleX = 0.98 * (compAngleX  + gyroXrate * dt) + 0.02 * roll;
  compAngleY = 0.98 * (compAngleY  + gyroYrate * dt) + 0.02 * pitch;
 
}
//---------------------------------------------------------------------------------
void setupNRF(){
  nrf.begin();
  nrf.openWritingPipe(pipe);
}
//---------------------------------------------------------------------------------
void sendNRF(){
  data[0] = compAngleX;
  data[1] = compAngleY;
  nrf.write(&data, sizeof(data));
}
//---------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  setupMPU();
  pinMode(pd, OUTPUT);
  digitalWrite(pd, HIGH);
  Serial.begin(115200);
  delay(100);
  setupNRF();
  starting_angle();
}
//---------------------------------------------------------------------------------
void loop() {
  Comp_filter();
  sendNRF();
  //Show Roll filter data
  Serial.print("Roll: ");
  Serial.print(compAngleX);
  Serial.print("\t");

  //Show Pitch filter data
  Serial.print("Pitch: ");
  Serial.print(compAngleY);
  Serial.print("\t");
  //Serial.print("Yaw: ");
  //Serial.print(compAngleZ);
  Serial.print("\t");
  Serial.print("\n");
 
  delay(2);
}
