#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

double data[10];
RF24 nrf(9,10);// CSN,CE
const uint64_t pipe = 0xE8E8F0F0E1LL;
//---------------------------------------------------------------------------------
void setup(){
 Serial.begin(115200);
 nrf.begin();
 nrf.openReadingPipe(1,pipe);
 nrf.startListening();
 }
//---------------------------------------------------------------------------------
void loop(){
  if (nrf.available())
  {
   bool done = false; 
   while (!done)
   {
     done = nrf.read(data, sizeof(data));

  Serial.print("Roll: "); Serial.print(data[0]);
  Serial.print("\t");
  Serial.print("Pitch: ");Serial.print(data[1]);
  Serial.print("\n");
     delay(2);
   }
  }
 }
