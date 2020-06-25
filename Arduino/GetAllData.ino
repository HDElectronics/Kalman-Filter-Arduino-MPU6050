#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

char serRead;

void setup(){
        Serial.begin(115200);
        Wire.begin();
        mpu.begin();
}

void loop(){
        
if(Serial.available())
{
        serRead = Serial.read();
        mpu.update();
}
switch (serRead)
{
        case '1':
                Serial.print(String(mpu.getRawAccX()));
                Serial.print("\n");
                break;
        case '2':
                Serial.print(String(mpu.getRawAccY()));
                Serial.print("\n");
                break;
        case '3':
                Serial.print(String(mpu.getRawAccZ()));
                Serial.print("\n");
                break;
        case '4':
                Serial.print(String(mpu.getRawGyroX()));
                Serial.print("\n");
                break;
        case '5':
                Serial.print(String(mpu.getRawGyroY()));
                Serial.print("\n");
                break;
        case '6':
                Serial.print(String(mpu.getRawGyroZ()));
                Serial.print("\n");
                break;
 }
 serRead = NULL;
}
